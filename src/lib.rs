#![no_std]

pub mod receiver;
pub mod sender;

use embassy_stm32::can::{
    self, BufferedCan, CanConfigurator, RxBuf, TxBuf, filter::ExtendedFilter,
};
use embedded_can::ExtendedId;
use heapless::Vec;

const RODOS_CAN_ID: u8 = 0x1C;

pub struct Config;
pub struct Active;

/// Constructor and interface to read and write can messages with the RODOS protocol
pub struct RodosCanInterface<'d, const TX_BUF_SIZE: usize, const RX_BUF_SIZE: usize, State> {
    interface: BufferedCan<'d, TX_BUF_SIZE, RX_BUF_SIZE>,
    device_id: u8,
    _state: State,
}

impl<'d, const TX_BUF_SIZE: usize, const RX_BUF_SIZE: usize> RodosCanInterface<'d, TX_BUF_SIZE, RX_BUF_SIZE, Config> {
    /// # create an instance using a base can configurator, a bitrate and a list of topics
    ///
    /// this function takes a minimally configured CanConfigurator instance, a bitrate as well as
    /// the rodos id this device will send as and a list
    /// of topic - device value pairs. If the device is None, the topic will be accepted from all
    /// devices. to generate the CanConfigurator simply provide a periph bus, can rx and tx pins and an interrupt reference
    /// ```
    /// CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    /// ```
    pub fn new(
        mut can_configurator: CanConfigurator<'d>,
        rx_buf: &'static mut RxBuf<RX_BUF_SIZE>,
        tx_buf: &'static mut TxBuf<TX_BUF_SIZE>,
        bitrate: u32,
        device_id: u8,
        rodos_ids: &[(u16, Option<u8>)],
    ) -> Self {
        // reject all by default
        can_configurator.set_config(
            can::config::FdCanConfig::default()
                .set_global_filter(can::config::GlobalFilter::reject_all()),
        );
        // add filters for all relevant topics
        can_configurator.set_bitrate(bitrate);
        let mut filters = rodos_ids
            .into_iter()
            .map(|rodos_id| -> ExtendedFilter {
                let can_id_range_start: u32 =
                    (RODOS_CAN_ID as u32) << (16 + 8) | (rodos_id.0 as u32) << 8;
                let filter = if let Some(device) = rodos_id.1 {
                    can::filter::FilterType::DedicatedSingle(
                        ExtendedId::new(can_id_range_start | device as u32).unwrap(),
                    )
                } else {
                    let can_id_range_end: u32 = can_id_range_start | 0xFF;
                    can::filter::FilterType::Range {
                        to: ExtendedId::new(can_id_range_start).unwrap(),
                        from: ExtendedId::new(can_id_range_end).unwrap(),
                    }
                };
                ExtendedFilter {
                    filter,
                    action: can::filter::Action::StoreInFifo0,
                }
            })
            .take(8)
            .collect::<Vec<ExtendedFilter, 8>>();
        // fill up rest of the filter slots with disabled filters
        while !filters.is_full() {
            filters.push(ExtendedFilter::disable()).unwrap();
        }
        can_configurator
            .properties()
            .set_extended_filters(&filters.into_array().unwrap());

        // initialize buffered can
        let interface = can_configurator.into_normal_mode().buffered(
            tx_buf,
            rx_buf
        );

        Self {
            interface,
            device_id,
            _state: Config,
        }
    }
    /// # Split the configurator into a configured sender and receiver instance
    ///
    /// + The const parameter *NUMBER_OF_SOURCES* specifies the size of the map for
    /// incoming can message sources. One "source" is one device sending on one topic.
    /// As this is used to generate a hash map NUMBER_OF_SOURCES needs to be a power of 2
    ///
    /// + The const parameter *MAX_PACKET_LENGTH* specifies the size of the buffer allocated to each
    /// source. as one RODOS can message contains 5 bytes of payload this should be a multiple of 5
    pub fn split<const NUMBER_OF_SOURCES: usize, const MAX_PACKET_LENGTH: usize>(
        self,
    ) -> (
        receiver::RodosCanReceiver<NUMBER_OF_SOURCES, MAX_PACKET_LENGTH>,
        sender::RodosCanSender,
        RodosCanInterface<'d, TX_BUF_SIZE, RX_BUF_SIZE, Active>,
    ) {
        (
            receiver::RodosCanReceiver::new(self.interface.reader()),
            sender::RodosCanSender::new(self.interface.writer(), self.device_id),
            RodosCanInterface {
                interface: self.interface,
                device_id: self.device_id,
                _state: Active,
            },
        )
    }
}
