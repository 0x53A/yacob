//! CANopen firmware-update node for Nucleo-G431KB.
//!
//! Implements CiA 302 programming objects for firmware download over SDO:
//!
//! - 0x1F50 sub1: Program Data — receives firmware chunks (domain, max 256 bytes)
//! - 0x1F51 sub1: Program Control — 3=clear flash, 1=validate, 0x80=reset
//! - 0x1F56 sub1: Program Software ID — current firmware CRC32
//! - 0x1F57 sub1: Flash Status — 0=idle, 1=programming, 2=valid, 0xFF=error
//!
//! Update protocol (master side):
//!   1. Write 3 to 0x1F51.1 (clear flash region)
//!   2. Repeatedly write firmware chunks to 0x1F50.1 (each SDO transfer = one chunk)
//!   3. Write CRC32 to 0x1F56.1
//!   4. Write 1 to 0x1F51.1 (validate — node checks CRC)
//!   5. Read 0x1F57.1 to confirm status == 2 (valid)
//!   6. Write 0x80 to 0x1F51.1 (reset into new firmware)
//!
//! ## API features demonstrated
//!
//! - `validate_write()` — rejects SDO writes to 0x1F50 when flash is in error state
//! - `Node::request_reset()` — CANopen-layer reset on program_control=0x80
//! - `NmtCommand::to_frame()` — build NMT frames from no_std code
//! - `events_dropped()` — monitor event queue overflow
//!
//! ## Remaining limitations
//!
//! - **Domain write is all-or-nothing**: no streaming callback for large transfers
//! - **SDO client buffer is only 256 bytes**: limits chunk size per transfer

#![no_std]
#![no_main]

use canopen_core::cobid::NodeId;
use canopen_core::node::{Node, NodeConfig, ResetType};
use canopen_core::time::Clock;
use canopen_core::transport::{CanError, CanFrame};
use canopen_core::OdEventSignal;
use canopen_derive::object_dictionary;

use core::cell::RefCell;
use crc::{Crc, CRC_32_ISO_HDLC};
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::can::filter::{StandardFilter, StandardFilterSlot};
use embassy_stm32::can::{CanConfigurator, CanRx, CanTx, Frame};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{bind_interrupts, can, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Ticker};

use {defmt_rtt as _, panic_probe as _};

const CRC: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

/// Flash region reserved for firmware updates.
/// On STM32G431KB: 128K total flash, bootloader in first 16K,
/// application at 0x0800_4000, update region at 0x0801_0000 (64K).
const FW_UPDATE_BASE: u32 = 0x0801_0000;
const FW_UPDATE_SIZE: u32 = 64 * 1024;

/// Flash status values (stored in OD 0x1F57 sub1).
mod flash_status {
    pub const IDLE: u32 = 0;
    pub const PROGRAMMING: u32 = 1;
    pub const VALID: u32 = 2;
    pub const ERROR: u32 = 0xFF;
}

// ---------- Object Dictionary ----------

object_dictionary! {
    #[validate_write(check_program_state)]
    pub struct FwUpdateOd {
        // Standard identity
        [0x1000] device_type: u32 = 0x0000_0191, ro;
        [0x1001] error_register: u8 = 0x00, ro;
        [0x1018] identity: record {
            [1] vendor_id: u32 = 0x0000_CAFE, ro;
            [2] product_code: u32 = 0x0002, ro;
            [3] revision: u32 = 0x0001_0000, ro;
            [4] serial_number: u32 = 0x0000_0001, ro;
        };

        // Application I/O (minimal — this node focuses on firmware update)
        [0x6000] status: record {
            [1] uptime_s: u32 = 0, ro, pdo;
        };

        tpdo[1](transmission_type = 255, inhibit_time = 0, event_timer = 1000) {
            uptime_s,
        };

        // CiA 302 Programming objects
        //
        // 0x1F50: Program Data — receives firmware chunks.
        // Each SDO write to this domain delivers one chunk (up to 256 bytes).
        // The application reads it on OdEvent and programs flash.
        [0x1F50] program_data: record {
            [1] firmware_chunk: domain<256>, rw;
        };

        // 0x1F51: Program Control — commands the programming state machine.
        //   3 = clear (erase flash region)
        //   1 = start (validate CRC, mark image good)
        //   0x80 = reset (reboot into new firmware)
        [0x1F51] program_control: record {
            [1] command: u8 = 0, rw;
        };

        // 0x1F56: Program Software Identification.
        // Master writes expected CRC32 here before validation.
        // Node reads it during validate step to compare.
        [0x1F56] program_sw_id: record {
            [1] expected_crc: u32 = 0, rw;
        };

        // 0x1F57: Flash Status Identification.
        //   0 = idle, 1 = programming, 2 = valid, 0xFF = error
        [0x1F57] flash_status_id: record {
            [1] flash_status: u32 = 0, ro;
        };
    }
}

impl FwUpdateOd {
    /// Reject firmware data writes when flash is in error state.
    /// This prevents the SDO server from accepting chunks that can't be programmed,
    /// giving the master a clean SDO abort instead of silently dropping data.
    fn check_program_state(&self, index: u16, subindex: u8, _data: &[u8]) -> Result<(), canopen_core::od::OdError> {
        // Reject writes to Program Data (0x1F50) when flash is in error state
        if index == 0x1F50 && subindex == 1 && self.flash_status == flash_status::ERROR {
            return Err(canopen_core::od::OdError::HardwareError);
        }
        // Reject Program Control commands when already in error (except clear=3)
        if index == 0x1F51 && subindex == 1 && self.flash_status == flash_status::ERROR {
            if !_data.is_empty() && _data[0] != 3 {
                return Err(canopen_core::od::OdError::HardwareError);
            }
        }
        Ok(())
    }
}

// ---------- Firmware programming state ----------

struct FwProgrammer {
    offset: u32,
    crc_state: crc::Digest<'static, u32>,
}

impl FwProgrammer {
    fn new() -> Self {
        Self {
            offset: 0,
            crc_state: CRC.digest(),
        }
    }

    /// Erase the update flash region.
    fn erase(&mut self) {
        info!("Erasing flash region 0x{:08X}..0x{:08X}",
              FW_UPDATE_BASE, FW_UPDATE_BASE + FW_UPDATE_SIZE);

        // In real firmware: use embassy_stm32::flash::Flash to erase pages.
        // For this example we just reset state.
        //
        // let mut flash = Flash::new_blocking(p.FLASH);
        // flash.blocking_erase(FW_UPDATE_BASE, FW_UPDATE_BASE + FW_UPDATE_SIZE).unwrap();

        self.offset = 0;
        self.crc_state = CRC.digest();
        info!("Flash erased, ready for programming");
    }

    /// Program one chunk to flash at the current offset.
    fn write_chunk(&mut self, data: &[u8]) -> bool {
        let addr = FW_UPDATE_BASE + self.offset;

        if self.offset + data.len() as u32 > FW_UPDATE_SIZE {
            error!("Chunk at offset 0x{:X} would exceed flash region", self.offset);
            return false;
        }

        // In real firmware:
        // flash.blocking_write(addr, data).unwrap();
        info!("Flash write: 0x{:08X} + {} bytes (total {})",
              addr, data.len(), self.offset + data.len() as u32);

        self.crc_state.update(data);
        self.offset += data.len() as u32;
        true
    }

    /// Validate the programmed image against expected CRC.
    fn validate(&self, expected_crc: u32) -> bool {
        // Clone the digest to finalize without consuming it
        let computed = self.crc_state.clone().finalize();
        info!("CRC check: computed=0x{:08X} expected=0x{:08X} ({} bytes)",
              computed, expected_crc, self.offset);
        computed == expected_crc
    }

    fn bytes_written(&self) -> u32 {
        self.offset
    }
}

// ---------- Shared state ----------

static TX_CHANNEL: Channel<CriticalSectionRawMutex, CanFrame, 16> = Channel::new();
static RX_CHANNEL: Channel<CriticalSectionRawMutex, CanFrame, 16> = Channel::new();
static EVENT_SIGNAL: OdEventSignal = OdEventSignal::new();

static NODE: Mutex<CriticalSectionRawMutex, RefCell<Option<Node<FwUpdateOd, 1, 0>>>> =
    Mutex::new(RefCell::new(None));

// ---------- Clock ----------

struct EmbassyClock;

impl Clock for EmbassyClock {
    fn now_us(&self) -> u64 {
        Instant::now().as_micros()
    }
}

// ---------- Channel-based Transport ----------

struct ChannelTransport;

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

// ---------- Interrupt bindings ----------

bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<peripherals::FDCAN1>;
});

// ---------- CAN TX task ----------

#[embassy_executor::task]
async fn can_tx_task(mut tx: CanTx<'static>) {
    loop {
        let frame = TX_CHANNEL.receive().await;
        let id = embedded_can::StandardId::new(frame.raw_id()).unwrap();
        match Frame::new_data(id, frame.data()) {
            Ok(f) => { tx.write(&f).await; }
            Err(_) => { warn!("Failed to create CAN frame"); }
        }
    }
}

// ---------- CAN RX task ----------

#[embassy_executor::task]
async fn can_rx_task(mut rx: CanRx<'static>) {
    loop {
        match rx.read().await {
            Ok(envelope) => {
                if let Some(can_frame) = {
                    let id = embedded_can::Frame::id(&envelope.frame);
                    CanFrame::new(
                        match id {
                            embedded_can::Id::Standard(sid) => sid.as_raw(),
                            embedded_can::Id::Extended(_) => continue,
                        },
                        envelope.frame.data(),
                    )
                } {
                    let _ = RX_CHANNEL.try_send(can_frame);
                }
            }
            Err(e) => {
                warn!("CAN RX bus error: {:?}", defmt::Debug2Format(&e));
            }
        }
    }
}

// ---------- Protocol task ----------

#[embassy_executor::task]
async fn protocol_task() {
    let mut transport = ChannelTransport;
    let clock = EmbassyClock;
    let mut ticker = Ticker::every(Duration::from_millis(1));

    loop {
        ticker.next().await;
        NODE.lock(|cell| {
            let mut borrow = cell.borrow_mut();
            let node = borrow.as_mut().unwrap();
            node.process(&mut transport, &clock);
        });
    }
}

// ---------- Main ----------

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("CANopen firmware-update node starting...");

    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL85,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.mux.fdcansel = mux::Fdcansel::PCLK1;
    }

    let p = embassy_stm32::init(config);

    // Status LED
    let mut led = Output::new(p.PB8, Level::Low, Speed::Low);

    // FDCAN1: PA11 (RX) / PA12 (TX), 500 kbit/s
    let mut can = CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    can.set_bitrate(500_000);
    can.properties()
        .set_standard_filter(StandardFilterSlot::_0, StandardFilter::accept_all_into_fifo0());
    let can = can.into_normal_mode();
    let (tx, rx, _props) = can.split();

    spawner.must_spawn(can_tx_task(tx));
    spawner.must_spawn(can_rx_task(rx));

    // CANopen node setup
    let node_id = NodeId::new(2).unwrap();
    let od = FwUpdateOd::new();
    let mut node: Node<FwUpdateOd, 1, 0> = Node::new(
        NodeConfig::<1, 0> {
            node_id,
            heartbeat_interval_ms: 500,
            auto_start: true,
            tpdo: od.tpdo_configs(node_id),
            rpdo: [],
        },
        od,
    );
    node.set_event_signal(&EVENT_SIGNAL);
    NODE.lock(|cell| cell.borrow_mut().replace(node));

    spawner.must_spawn(protocol_task());

    info!("FW update node {} running", node_id.raw());

    // ---------- Application logic ----------

    let mut programmer = FwProgrammer::new();
    let mut uptime_ticker = Ticker::every(Duration::from_secs(1));
    let mut uptime: u32 = 0;

    loop {
        match select(EVENT_SIGNAL.wait(), uptime_ticker.next()).await {
            Either::First(_) => {
                NODE.lock(|cell| {
                    let mut borrow = cell.borrow_mut();
                    let node = borrow.as_mut().unwrap();

                    while let Some(evt) = node.next_event() {
                        match (evt.index, evt.subindex) {
                            // Firmware chunk received via SDO
                            (0x1F50, 1) => {
                                // Read the domain data that was just written.
                                // Note: check_program_state() already rejected this
                                // write if flash was in error state, so we don't need
                                // to re-check here.
                                let chunk = node.od().firmware_chunk.as_slice();
                                if chunk.is_empty() {
                                    continue;
                                }

                                // Set programming state
                                node.od_mut().flash_status = flash_status::PROGRAMMING;
                                led.set_high();

                                if programmer.write_chunk(chunk) {
                                    info!("Chunk OK, {} bytes total", programmer.bytes_written());
                                } else {
                                    node.od_mut().flash_status = flash_status::ERROR;
                                    node.set_error(0x5000, canopen_core::error_register::GENERIC, &[]);
                                    error!("Flash write failed");
                                }
                            }

                            // Program control command
                            (0x1F51, 1) => {
                                let cmd = node.od().command;
                                info!("Program control: 0x{:02X}", cmd);

                                match cmd {
                                    // Clear — erase flash region
                                    3 => {
                                        programmer.erase();
                                        node.od_mut().flash_status = flash_status::IDLE;
                                        node.clear_all_errors();
                                        led.set_low();
                                    }
                                    // Start — validate programmed image
                                    1 => {
                                        let expected_crc = node.od().expected_crc;
                                        if programmer.validate(expected_crc) {
                                            info!("Firmware valid! {} bytes", programmer.bytes_written());
                                            node.od_mut().flash_status = flash_status::VALID;
                                            led.set_low();
                                        } else {
                                            error!("CRC mismatch!");
                                            node.od_mut().flash_status = flash_status::ERROR;
                                            node.set_error(0x5000, canopen_core::error_register::GENERIC, &[]);
                                        }
                                    }
                                    // Reset — reboot into new firmware
                                    0x80 => {
                                        if node.od().flash_status == flash_status::VALID {
                                            info!("Resetting into new firmware...");
                                            // CANopen-layer reset: sends boot heartbeat,
                                            // re-enters Initializing, then transitions per
                                            // auto_start on next process() call.
                                            node.request_reset(ResetType::Application);
                                            // In a real bootloader setup, we'd also set a
                                            // flag in backup RAM to tell the bootloader
                                            // which image to boot, then do a hard reset:
                                            // cortex_m::peripheral::SCB::sys_reset();
                                        } else {
                                            warn!("Cannot reset — firmware not validated (status={})",
                                                  node.od().flash_status);
                                        }
                                    }
                                    _ => {
                                        warn!("Unknown program control: 0x{:02X}", cmd);
                                    }
                                }
                            }

                            _ => {}
                        }
                    }
                });
            }

            // Uptime counter — sent via TPDO1 every second
            Either::Second(_) => {
                uptime += 1;
                NODE.lock(|cell| {
                    let mut borrow = cell.borrow_mut();
                    let node = borrow.as_mut().unwrap();
                    node.od_mut().uptime_s = uptime;
                });
            }
        }
    }
}
