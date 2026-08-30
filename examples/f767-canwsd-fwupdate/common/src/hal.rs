//! Nucleo-F767ZI board support shared by the updater and demo application.

use canopen_core::time::Clock;
use canopen_core::transport::{CanError, CanFrame};
use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::{peripherals, Peri};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Instant;

use crate::FlashOps;

/// Nucleo HSE is the 8 MHz MCO supplied by the on-board ST-Link.
pub fn clock_config() -> embassy_stm32::Config {
    use embassy_stm32::rcc::*;

    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz(8_000_000),
        mode: HseMode::Bypass,
    });
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL216,
        divp: Some(PllPDiv::DIV2),
        divq: Some(PllQDiv::DIV9),
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV4;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config
}

pub struct Nvm<'d>(pub Flash<'d, Blocking>);

impl<'d> Nvm<'d> {
    pub fn new(flash: Peri<'d, peripherals::FLASH>) -> Self {
        Self(Flash::new_blocking(flash))
    }
}

impl FlashOps for Nvm<'_> {
    type Error = embassy_stm32::flash::Error;

    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error> {
        self.0.blocking_write(offset, data)
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.0.blocking_erase(from, to)
    }
}

pub struct StatusLed<'d>(Output<'d>);

impl<'d> StatusLed<'d> {
    /// Green user LED LD1 on PB0, active high.
    pub fn new(pin: Peri<'d, peripherals::PB0>) -> Self {
        Self(Output::new(pin, Level::Low, Speed::Low))
    }

    pub fn set(&mut self, on: bool) {
        self.0.set_level(Level::from(on));
    }

    pub fn toggle(&mut self) {
        self.0.toggle();
    }
}

pub fn serial_from_uid() -> u32 {
    crate::CRC32.checksum(embassy_stm32::uid::uid().as_slice())
}

pub fn mac_from_uid() -> [u8; 6] {
    let uid = embassy_stm32::uid::uid();
    [
        ((uid[0] ^ uid[6]) | 0x02) & !0x01,
        uid[1] ^ uid[7],
        uid[2] ^ uid[8],
        uid[3] ^ uid[9],
        uid[4] ^ uid[10],
        uid[5] ^ uid[11],
    ]
}

pub struct EmbassyClock;

impl Clock for EmbassyClock {
    fn now_us(&self) -> u64 {
        Instant::now().as_micros()
    }
}

/// The updater's CAN bus is virtual: this side faces the CANopen node, while
/// `ws` faces the sole canwsd client.
pub static NODE_TX: Channel<CriticalSectionRawMutex, CanFrame, 64> = Channel::new();
pub static NODE_RX: Channel<CriticalSectionRawMutex, CanFrame, 64> = Channel::new();

pub struct ChannelTransport;

impl embedded_can::nb::Can for ChannelTransport {
    type Frame = CanFrame;
    type Error = CanError;

    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        NODE_TX
            .try_send(*frame)
            .map_err(|_| nb::Error::Other(CanError::TxBufferFull))?;
        Ok(None)
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        NODE_RX.try_receive().map_err(|_| nb::Error::WouldBlock)
    }
}
