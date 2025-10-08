#![no_std]

//! This library provides a driver to interface with the
//! can protocol implemented by the linkinterfaceCAN gateway
//! in the RODOS real time operating system (developed by the
//! university of wuerzburg).
//! At the moment this driver can only be used with a buffered can
//! instance (a can mode where received messages are directly written
//! into static memory on hardware interrupt) as this is the mode
//! needed by the s²outh project this module was developed for.
//! Different modes might be added in the future if required.
//!
//! This driver was developed on the basis of the embassy_stm32 project

pub mod receiver;
pub mod sender;

#[cfg(feature = "stm32g0b1ke")]
use embassy_stm32::can::{
    self, filter::ExtendedFilter, BufferedCan, CanConfigurator, RxBuf, TxBuf
};
#[cfg(feature = "stm32f407vg")]
use embassy_stm32::can::{
    self, filter::BankConfig, BufferedCan, Can, Fifo, RxBuf, TxBuf
};
use embedded_can::ExtendedId;
use heapless::Vec;

use receiver::{RodosCanReceiver, RodosCanFrame, RodosCanReceiveError};
use sender::{RodosCanSender, RodosCanSendError};

const RODOS_CAN_ID: u8 = 0x1C;

/// Marker struct for the error mode that can filters are full
#[derive(Debug)]
pub struct FiltersFullError;

#[cfg(feature = "stm32g0b1ke")]
/// Can peripheral in configuration stage
pub struct ConfigPeriph<'d> {
    rodos_filters: Vec<ExtendedFilter, 8>,
    configurator: CanConfigurator<'d>
}
#[cfg(feature = "stm32f407vg")]
/// Can peripheral in configuration stage
pub struct ConfigPeriph<'d> {
    rodos_filters: Vec<BankConfig, 28>,
    configurator: Can<'d>
}

/// Can peripheral in split (buffered) state
pub struct SplitPeriph<'d, const TX_BUF_SIZE: usize, const RX_BUF_SIZE: usize> {
    _interface: BufferedCan<'d, TX_BUF_SIZE, RX_BUF_SIZE>,
}

pub struct ActivePeriph<'d,
    const NUMBER_OF_SOURCES: usize, const MAX_PACKAGE_LENGTH: usize,
    const TX_BUF_SIZE: usize, const RX_BUF_SIZE: usize> {
    receiver: RodosCanReceiver<NUMBER_OF_SOURCES, MAX_PACKAGE_LENGTH>,
    sender: RodosCanSender,
    interface: BufferedCan<'d, TX_BUF_SIZE, RX_BUF_SIZE>,
}

/// Constructor and interface to read and write can messages with the RODOS protocol
pub struct RodosCanInterface<State> {
    peripheral: State,
    device_id: u8,
}

#[cfg(feature = "stm32g0b1ke")]
impl<'d> RodosCanInterface<ConfigPeriph<'d>> {
    /// # create an instance using a base can configurator
    ///
    /// this function takes a minimally configured CanConfigurator instance
    /// as well as the rodos id this device will identify itself to other devices
    /// to generate the CanConfigurator simply provide a periph bus, can rx and tx pins and an interrupt reference
    /// ```
    /// CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    /// ```
    /// in principle the Can Interface can be put into active state and used
    /// directly following this function call, however you won't be able to\
    /// receive any messages without specifying at least one topic with
    /// ```
    /// pub fn add_receive_topic(&mut self, rodos_topic: u16, rodos_device_id: Option<u8>)
    /// ```
    pub fn new(
        mut configurator: CanConfigurator<'d>,
        device_id: u8,
    ) -> Self {
        // reject all can Ids by default
        configurator.set_config(
            can::config::FdCanConfig::default()
                .set_global_filter(can::config::GlobalFilter::reject_all()),
        );

        let rodos_filters = Vec::new();
        Self {
            peripheral: ConfigPeriph { rodos_filters, configurator },
            device_id,
        }
    }
    /// # add topic filter
    ///
    /// This function takes a rodos topic id (u16) as well as 
    /// an optional device Id (u8) and adds them to the hardware
    /// filter of receivable messages. If no device id is specified
    /// the topic will be accepted from all devices on the bus.
    /// You need to call this function at least once on the configurator
    /// to be able to receive messages
    pub fn add_receive_topic(&mut self, rodos_topic: u16, rodos_device_id: Option<u8>) -> Result<&mut Self, FiltersFullError> {
        let can_id_range_start: u32 =
            (RODOS_CAN_ID as u32) << (16 + 8) | (rodos_topic as u32) << 8;
        let filter = if let Some(device) = rodos_device_id {
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
        let extended_filter = ExtendedFilter {
            filter,
            action: can::filter::Action::StoreInFifo0,
        };
        self.peripheral.rodos_filters.push(extended_filter).map_err(|_| FiltersFullError)?;
        Ok(self)
    }
    /// # allow receiving messages from all telestion topics.
    ///
    /// This is strongly discuraged in production,
    /// as the RODOS gateway will likely flood the
    /// can bus with irrelevant messages
    pub fn allow_all_topics(&mut self) -> Result<&mut Self, FiltersFullError> {
        let can_id_range_start: u32 = (RODOS_CAN_ID as u32) << (8 * 3);
        let can_id_range_end: u32 = can_id_range_start | 0xFF_FF_FF;
        let filter = can::filter::FilterType::Range {
            to: ExtendedId::new(can_id_range_start).unwrap(),
            from: ExtendedId::new(can_id_range_end).unwrap(),
        };
        let extended_filter = ExtendedFilter {
            filter,
            action: can::filter::Action::StoreInFifo0,
        };
        self.peripheral.rodos_filters.push(extended_filter).map_err(|_| FiltersFullError)?;
        Ok(self)
    }
    /// # set can bitrate
    ///
    /// This function simply calls the set_bitrate function on the can configurator.
    /// In principle it does not matter whether you use this function or simply call
    /// set_bitrate on the configurator before passing it to ::new()
    pub fn set_bitrate(&mut self, bitrate: u32) -> &mut Self {
        self.peripheral.configurator.set_bitrate(bitrate);
        self
    }
    /// # Activate the can transmitter for sending and receiving
    ///
    /// + The const parameter *NUMBER_OF_SOURCES* specifies the size of the map for
    /// incoming can message sources. One "source" is one device sending on one topic.
    /// As this is used to generate a hash map NUMBER_OF_SOURCES needs to be a power of 2
    ///
    /// + The const parameter *MAX_PACKET_LENGTH* specifies the size of the buffer allocated to each
    /// source. as one RODOS can message contains 5 bytes of payload this should be a multiple of 5
    pub fn activate<const NUMBER_OF_SOURCES: usize, const MAX_PACKET_LENGTH: usize, const TX_BUF_SIZE: usize, const RX_BUF_SIZE: usize>(
        mut self,
        tx_buf: &'static mut TxBuf<TX_BUF_SIZE>,
        rx_buf: &'static mut RxBuf<RX_BUF_SIZE>,
    ) -> RodosCanInterface<ActivePeriph<'d, NUMBER_OF_SOURCES, MAX_PACKET_LENGTH, TX_BUF_SIZE, RX_BUF_SIZE>> {
        // fill up unused filter slots with disabled filters
        while !self.peripheral.rodos_filters.is_full() {
            self.peripheral.rodos_filters.push(ExtendedFilter::disable()).unwrap();
        }
        self.peripheral.configurator
            .properties()
            .set_extended_filters(&self.peripheral.rodos_filters.into_array().unwrap());

        // initialize buffered can
        let interface = self.peripheral.configurator.into_normal_mode().buffered(
            tx_buf,
            rx_buf
        );
        let receiver = receiver::RodosCanReceiver::new(interface.reader());
        let sender = sender::RodosCanSender::new(interface.writer(), self.device_id);
        RodosCanInterface {
            peripheral: ActivePeriph { interface, receiver, sender },
            device_id: self.device_id,
        }
    }
}

#[cfg(feature = "stm32f407vg")]
impl<'d> RodosCanInterface<ConfigPeriph<'d>> {
    /// # create an instance using a base can object
    ///
    /// this function takes a minimally configured CanConfigurator instance
    /// as well as the rodos id this device will identify itself to other devices
    /// to generate the Can simply provide a periph bus, can rx and tx pins and an interrupt reference
    /// ```
    /// Can::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    /// ```
    /// in principle the Can Interface can be put into active state and used
    /// directly following this function call, however you won't be able to\
    /// receive any messages without specifying at least one topic with
    /// ```
    /// pub fn add_receive_topic(&mut self, rodos_topic: u16, rodos_device_id: Option<u8>)
    /// ```
    pub fn new(
        configurator: Can<'d>,
        device_id: u8,
    ) -> Self {
        let rodos_filters = Vec::new();
        Self {
            peripheral: ConfigPeriph { rodos_filters, configurator },
            device_id,
        }
    }
    /// # add topic filter
    ///
    /// This function takes a rodos topic id (u16) as well as 
    /// an optional device Id (u8) and adds them to the hardware
    /// filter of receivable messages. If no device id is specified
    /// the topic will be accepted from all devices on the bus.
    /// You need to call this function at least once on the configurator
    /// to be able to receive messages
    pub fn add_receive_topic(&mut self, rodos_topic: u16, rodos_device_id: Option<u8>) -> Result<&mut Self, FiltersFullError> {
        let can_topic_id: u32 =
            (RODOS_CAN_ID as u32) << (16 + 8) | (rodos_topic as u32) << 8;
        let filter: BankConfig = if let Some(device) = rodos_device_id {
            [
                can::filter::ListEntry32::data_frames_with_id(ExtendedId::new(can_topic_id | device as u32).unwrap()),
                can::filter::ListEntry32::data_frames_with_id(ExtendedId::new(can_topic_id | device as u32).unwrap())
            ].into()
        } else {
            can::filter::Mask32::frames_with_ext_id(
                ExtendedId::new(can_topic_id).unwrap(),
                ExtendedId::new(0xFF_FF_FF_00).unwrap()
            ).into()
        };
        self.peripheral.rodos_filters.push(filter).map_err(|_| FiltersFullError)?;
        Ok(self)
    }
    /// # allow receiving messages from all telestion topics.
    ///
    /// This is strongly discuraged in production,
    /// as the RODOS gateway will likely flood the
    /// can bus with irrelevant messages
    pub fn allow_all_topics(&mut self) -> Result<&mut Self, FiltersFullError> {
        let can_topic_id: u32 = (RODOS_CAN_ID as u32) << (8 * 3);

        let filter = can::filter::Mask32::frames_with_ext_id(
            ExtendedId::new(can_topic_id).unwrap(),
            ExtendedId::new(0xFF_00_00_00).unwrap()
        ).into();
        self.peripheral.rodos_filters.push(filter).map_err(|_| FiltersFullError)?;
        Ok(self)
    }
    /// # set can bitrate
    ///
    /// This function simply calls the set_bitrate function on the can configurator.
    /// In principle it does not matter whether you use this function or simply call
    /// set_bitrate on the configurator before passing it to ::new()
    pub fn set_bitrate(&mut self, bitrate: u32) -> &mut Self {
        self.peripheral.configurator.set_bitrate(bitrate);
        self
    }
    /// # Activate the can transmitter for sending and receiving
    ///
    /// + The const parameter *NUMBER_OF_SOURCES* specifies the size of the map for
    /// incoming can message sources. One "source" is one device sending on one topic.
    /// As this is used to generate a hash map NUMBER_OF_SOURCES needs to be a power of 2
    ///
    /// + The const parameter *MAX_PACKET_LENGTH* specifies the size of the buffer allocated to each
    /// source. as one RODOS can message contains 5 bytes of payload this should be a multiple of 5
    pub async fn activate<const NUMBER_OF_SOURCES: usize, const MAX_PACKET_LENGTH: usize, const TX_BUF_SIZE: usize, const RX_BUF_SIZE: usize>(
        mut self,
        tx_buf: &'static mut TxBuf<TX_BUF_SIZE>,
        rx_buf: &'static mut RxBuf<RX_BUF_SIZE>,
    ) -> RodosCanInterface<ActivePeriph<'d, NUMBER_OF_SOURCES, MAX_PACKET_LENGTH, TX_BUF_SIZE, RX_BUF_SIZE>> {

        for (i, bank) in self.peripheral.rodos_filters.into_iter().enumerate() {
            self.peripheral.configurator.modify_filters().enable_bank(i as u8, Fifo::Fifo0, bank);
        }

        // initialize buffered can
        self.peripheral.configurator.enable().await;
        let interface = self.peripheral.configurator.buffered(
            tx_buf,
            rx_buf
        );
        let receiver = receiver::RodosCanReceiver::new(interface.reader());
        let sender = sender::RodosCanSender::new(interface.writer(), self.device_id);
        RodosCanInterface {
            peripheral: ActivePeriph { interface, receiver, sender },
            device_id: self.device_id,
        }
    }
}

impl<'d, const NUMBER_OF_SOURCES: usize, const MAX_PACKET_LENGTH: usize, const TX_BUF_SIZE: usize, const RX_BUF_SIZE: usize>
    RodosCanInterface<ActivePeriph<'d, NUMBER_OF_SOURCES, MAX_PACKET_LENGTH, TX_BUF_SIZE, RX_BUF_SIZE>> {
    /// # Split the configurator into a configured sender and receiver instance
    pub fn split_buffered(self) -> (
        receiver::RodosCanReceiver<NUMBER_OF_SOURCES, MAX_PACKET_LENGTH>,
        sender::RodosCanSender,
        RodosCanInterface<SplitPeriph<'d, TX_BUF_SIZE, RX_BUF_SIZE>>,
    ) {
        (
            self.peripheral.receiver,
            self.peripheral.sender,
            RodosCanInterface {
                peripheral: SplitPeriph { _interface: self.peripheral.interface },
                device_id: self.device_id,
            },
        )
    }
    /// receive the next rodos frame async
    pub async fn receive<'a>(&'a mut self) -> Result<RodosCanFrame<'a>, RodosCanReceiveError> {
        self.peripheral.receiver.receive().await
    }
    /// send rodos frame async
    pub async fn send(&mut self, topic: u16, data: &[u8]) -> Result<(), RodosCanSendError> {
        self.peripheral.sender.send(topic, data).await
    }
}
