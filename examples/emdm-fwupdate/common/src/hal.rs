//! Board support for the EMDM (STM32G431CB) carrier board.
//!
//! Shared by the bootloader and the application so that clock setup, CAN
//! bring-up and the flash wrapper exist in exactly one place.

use canopen_core::time::Clock;
use canopen_core::transport::{CanError, CanFrame};
use defmt::warn;
use embassy_stm32::can::filter::{StandardFilter, StandardFilterSlot};
use embassy_stm32::can::{CanConfigurator, CanRx, CanTx, Frame};
use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, can, peripherals, Peri};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Instant;

use crate::{FlashOps, CAN_BITRATE};

bind_interrupts!(pub struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<peripherals::FDCAN1>;
});

/// 24 MHz HSE crystal, PLL M6/N85/R2 to 170 MHz on SYSCLK/AHB/APB.
///
/// FDCAN is clocked from PCLK1 so the 500 kbit/s bit timing matches the rest
/// of the bus.
pub fn clock_config() -> embassy_stm32::Config {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(24_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV6,
            mul: PllMul::MUL85,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.boost = true;
        config.rcc.mux.fdcansel = mux::Fdcansel::PCLK1;
    }
    config
}

// ---------------------------------------------------------------------------
// Flash
// ---------------------------------------------------------------------------

/// [`FlashOps`] over the on-chip flash controller.
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

// ---------------------------------------------------------------------------
// PCB RGB LED — common cathode to ground, so every channel is active-high.
// ---------------------------------------------------------------------------

pub struct StatusLed<'d> {
    red: Output<'d>,
    green: Output<'d>,
    blue: Output<'d>,
}

impl<'d> StatusLed<'d> {
    pub fn new(
        red: Peri<'d, peripherals::PB9>,
        green: Peri<'d, peripherals::PC13>,
        blue: Peri<'d, peripherals::PA15>,
    ) -> Self {
        Self {
            red: Output::new(red, Level::Low, Speed::Low),
            green: Output::new(green, Level::Low, Speed::Low),
            blue: Output::new(blue, Level::Low, Speed::Low),
        }
    }

    pub fn set(&mut self, red: bool, green: bool, blue: bool) {
        self.red.set_level(Level::from(red));
        self.green.set_level(Level::from(green));
        self.blue.set_level(Level::from(blue));
    }

    pub fn off(&mut self) {
        self.set(false, false, false);
    }
}

// ---------------------------------------------------------------------------
// Identity
// ---------------------------------------------------------------------------

/// Fold the 96-bit factory UID into the 32-bit CANopen serial number.
///
/// Both images report the same value, so a host can follow one board across
/// the reboot from the application into the updater and back.
pub fn serial_from_uid() -> u32 {
    crate::CRC32.checksum(embassy_stm32::uid::uid().as_slice())
}

// ---------------------------------------------------------------------------
// Clock source for the CANopen stack
// ---------------------------------------------------------------------------

pub struct EmbassyClock;

impl Clock for EmbassyClock {
    fn now_us(&self) -> u64 {
        Instant::now().as_micros()
    }
}

// ---------------------------------------------------------------------------
// CAN plumbing
//
// The Embassy CAN driver is async while `Node::process` is a polling state
// machine, so frames are bridged through two channels.
// ---------------------------------------------------------------------------

pub static TX_CHANNEL: Channel<CriticalSectionRawMutex, CanFrame, 16> = Channel::new();

/// Deliberately deeper than the TX side. An SDO block transfer delivers up to
/// 127 back-to-back frames, which at 500 kbit/s is roughly 7.5 frames per
/// millisecond, while the protocol task only drains this once per millisecond.
/// 16 slots would leave almost no margin; 64 tolerates an 8 ms stall.
pub static RX_CHANNEL: Channel<CriticalSectionRawMutex, CanFrame, 64> = Channel::new();

pub struct ChannelTransport;

impl embedded_can::nb::Can for ChannelTransport {
    type Frame = CanFrame;
    type Error = CanError;

    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        TX_CHANNEL
            .try_send(*frame)
            .map_err(|_| nb::Error::Other(CanError::TxBufferFull))?;
        Ok(None)
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        RX_CHANNEL.try_receive().map_err(|_| nb::Error::WouldBlock)
    }
}

/// Bring FDCAN1 up on PA11/PA12 in classic CAN mode and spawn the two pump
/// tasks that move frames between the driver and [`ChannelTransport`].
pub fn init_can(
    spawner: &embassy_executor::Spawner,
    fdcan: Peri<'static, peripherals::FDCAN1>,
    rx: Peri<'static, peripherals::PA11>,
    tx: Peri<'static, peripherals::PA12>,
) {
    let mut can = CanConfigurator::new(fdcan, rx, tx, Irqs);
    can.set_bitrate(CAN_BITRATE);
    can.properties().set_standard_filter(
        StandardFilterSlot::_0,
        StandardFilter::accept_all_into_fifo0(),
    );
    let (tx, rx, _properties) = can.into_normal_mode().split();
    spawner.spawn(can_tx_task(tx).expect("allocate the CAN TX task"));
    spawner.spawn(can_rx_task(rx).expect("allocate the CAN RX task"));
}

#[embassy_executor::task]
async fn can_tx_task(mut tx: CanTx<'static>) {
    loop {
        let frame = TX_CHANNEL.receive().await;
        let Some(id) = embedded_can::StandardId::new(frame.raw_id()) else {
            warn!("dropping frame with invalid standard id {}", frame.raw_id());
            continue;
        };
        match Frame::new_data(id, frame.data()) {
            Ok(f) => {
                tx.write(&f).await;
            }
            Err(_) => warn!("dropping oversized frame for id {}", frame.raw_id()),
        }
    }
}

#[embassy_executor::task]
async fn can_rx_task(mut rx: CanRx<'static>) {
    loop {
        match rx.read().await {
            Ok(envelope) => {
                let embedded_can::Id::Standard(id) = embedded_can::Frame::id(&envelope.frame) else {
                    continue;
                };
                if let Some(frame) = CanFrame::new(id.as_raw(), envelope.frame.data()) {
                    // A full queue means the protocol task is not keeping up.
                    // Dropping is correct here: CANopen retries at its own layer.
                    let _ = RX_CHANNEL.try_send(frame);
                }
            }
            Err(e) => warn!("CAN RX bus error: {:?}", defmt::Debug2Format(&e)),
        }
    }
}
