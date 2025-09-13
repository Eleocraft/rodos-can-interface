use core::cmp::min;

use defmt::Format;
use embassy_stm32::can::{enums::BusError, frame::Envelope, BufferedCanReceiver, Frame};
use embedded_can::Id;
use heapless::{FnvIndexMap, Vec};

/// Can frame for the RODOS can protocol
/// conatining the topic and data
pub struct RodosCanFrame<'a> {
    pub(super) topic: u16,
    pub(super) device: u8,
    pub(super) data: &'a [u8],
}

impl<'a> RodosCanFrame<'a> {
    pub fn topic(&self) -> u16 {
        self.topic
    }
    pub fn device(&self) -> u8 {
        self.device
    }
    pub fn data(&self) -> &'a [u8] {
        self.data
    }
}

/// Error enum for can frame decode errors
#[derive(Format)]
pub enum RodosCanDecodeError {
    WrongIDType,
    NoData,
}

/// Error enum for the all RODOS can receiving operations
#[derive(Format)]
pub enum RodosCanReceiveError {
    /// error in the underlying can error
    BusError(BusError),
    /// the can message could not be decoded as RODOS can message
    /// (It is likely not a RODOS can message. make sure not to use dupplicate ids!)
    CouldNotDecode(RodosCanDecodeError),
    /// one of the message frames has been dropped
    FrameDropped,
    /// the map for different sources is full
    SourceBufferFull,
    /// the message buffer for this specific map is full
    MessageBufferFull,
}

enum RodosCanFramePart {
    Head{
        data: Vec<u8, 5>,
        seq_len: usize,
    },
    Tail{
        data: Vec<u8, 7>,
        seq_num: usize,
    }
}

/// Module to send messages on a rodos can
pub struct RodosCanReceiver<const NUMBER_OF_SOURCES: usize, const MAX_PACKET_LENGTH: usize> {
    receiver: BufferedCanReceiver,
    frames: FnvIndexMap<u32, RodosPartialFrame<MAX_PACKET_LENGTH>, NUMBER_OF_SOURCES>,
}

struct RodosPartialFrame<const MAX_PACKET_LENGTH: usize> {
    data: Vec<u8, MAX_PACKET_LENGTH>,
    seq_num: usize,
    seq_len: usize,
}
impl<const MPL: usize> RodosPartialFrame<MPL> {
    fn new(seq_len: usize) -> Self {
        Self {
            data: Vec::new(),
            seq_num: 1,
            seq_len,
        }
    }
    fn uninitialized() -> Self {
        Self {
            data: Vec::new(),
            seq_num: 0,
            seq_len: 0,
        }
    }
    fn set_uninitialized(&mut self) {
        self.seq_num = 0;
    }
    fn is_uninitialized(&self) -> bool {
        self.seq_num == 0
    }
}

impl<const NUMBER_OF_SOURCES: usize, const MAX_PACKET_LENGTH: usize>
    RodosCanReceiver<NUMBER_OF_SOURCES, MAX_PACKET_LENGTH>
{
    /// create a new instance from BufferedCanReceiver
    pub(super) fn new(receiver: BufferedCanReceiver) -> Self {
        RodosCanReceiver {
            receiver,
            frames: FnvIndexMap::new(),
        }
    }
    /// take a u32 extended id and decode it to RODOS id parts
    fn decode_id(id: u32) -> (u16, u8) {
        let topic = (id >> 8) as u16;
        let device = id as u8;
        (topic, device)
    }
    /// take a can hal frame and decode it to RODOS message parts
    fn decode(frame: &Frame) -> Result<(u32, RodosCanFramePart), RodosCanDecodeError> {
        let Id::Extended(id) = frame.id() else {
            return Err(RodosCanDecodeError::WrongIDType);
        };
        let id = id.as_raw();

        if frame.data().len() <= 1 {
            // Not enough metadata in can msg
            return Err(RodosCanDecodeError::NoData);
        }
        let seq_num = frame.data()[0] as usize;
        if seq_num == 0 {
            // head frame part
            if frame.data().len() <= 3 {
                // Not enough metadata in can msg
                return Err(RodosCanDecodeError::NoData);
            }
            let seq_len = ((frame.data()[1] as usize) << 8) | frame.data()[2] as usize;
            let data = frame.data()[3..].try_into().unwrap();
            Ok((id, RodosCanFramePart::Head {
                data,
                seq_len,
            }))
        } else {
            let data = frame.data()[1..].try_into().unwrap();
            Ok((id, RodosCanFramePart::Tail {
                data,
                seq_num,
            }))
        }
    }
    fn process(&mut self, envelope: Envelope) -> Result<Option<u32>, RodosCanReceiveError> {
        let (id, frame_part) = Self::decode(&envelope.frame)
            .map_err(|e| RodosCanReceiveError::CouldNotDecode(e))?;

        if !self.frames.contains_key(&id) {
            // add entry if it doesn't already exist
            self.frames
                .insert(id, RodosPartialFrame::uninitialized())
                .map_err(|_| RodosCanReceiveError::SourceBufferFull)?;
        }
        
        let frame_ref = &mut self.frames[&id];

        match frame_part {
            RodosCanFramePart::Head { data, seq_len } => {
                // check if seq len is too long
                if seq_len > MAX_PACKET_LENGTH {
                    return Err(RodosCanReceiveError::MessageBufferFull);
                }
                // start new partial frame
                *frame_ref = RodosPartialFrame::new(seq_len);
                frame_ref.data.extend_from_slice(&data[0..min(data.len(), seq_len)]).unwrap();
            }
            RodosCanFramePart::Tail { data, seq_num } => {
                if frame_ref.is_uninitialized() {
                    return Ok(None); // if no head was parsed yet ignore this frame
                }
                if frame_ref.seq_num == seq_num {
                    let free_space = frame_ref.seq_len - frame_ref.data.len();
                    frame_ref.data.extend_from_slice(&data[0..min(data.len(), free_space)]).unwrap();

                    frame_ref.seq_num += 1;
                } else {
                    return Err(RodosCanReceiveError::FrameDropped);
                }
            }
        }
        
        // if buffer length >= seqence length, the frame is complete.
        // return the frame id
        if frame_ref.seq_len <= frame_ref.data.len() {
            frame_ref.set_uninitialized();
            Ok(Some(id))
        } else {
            Ok(None)
        }
    }
    /// receive the next rodos frame async
    pub async fn receive<'a>(&'a mut self) -> Result<RodosCanFrame<'a>, RodosCanReceiveError> {
        loop {
            let can_frame = self.receiver.receive().await.map_err(|e| RodosCanReceiveError::BusError(e))?;
            if let Some(id) = self.process(can_frame)? {
                let data = &self.frames[&id].data[..];
                let (topic, device) = Self::decode_id(id);
                return Ok(RodosCanFrame {
                    topic,
                    device,
                    data,
                });
            }
        }
    }
}
