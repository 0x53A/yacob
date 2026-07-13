//! Interrupt-driven CANopen node for Nucleo-G431KB.
//!
//! The CANopen protocol runs in a background task (heartbeat, SDO, PDO).
//! Application logic stays in main: button reads via EXTI, LED writes via
//! OD events. od_mut() auto-notifies changed TPDO-mapped fields.
//!
//! Between wakes the MCU is in WFI.

#![no_std]
#![no_main]

use canopen_core::cobid::NodeId;
use canopen_core::node::{Node, NodeConfig, SharedNode};
use canopen_core::time::Clock;
use canopen_core::transport::{CanError, CanFrame};
use canopen_derive::object_dictionary;

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::can::filter::{StandardFilter, StandardFilterSlot};
use embassy_stm32::can::{CanConfigurator, CanRx, CanTx, Frame};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::{bind_interrupts, can, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Ticker};

use {defmt_rtt as _, panic_probe as _};

// ---------- Object Dictionary ----------

object_dictionary! {
    pub struct NodeOd {
        [0x1000] device_type: u32 = 0x0000_0191, ro;
        [0x1001] error_register: u8 = 0x00, ro;
        [0x1018] identity: record {
            [1] vendor_id: u32 = 0x0000_CAFE, ro;
            [2] product_code: u32 = 0x0001, ro;
            [3] revision: u32 = 0x0001_0000, ro;
            [4] serial_number: u32 = 0x0000_0001, ro;
        };
        // CiA 401-style process I/O. Perspective is the physical process,
        // not the bus: an "input" is read from the world and published on
        // the bus (TPDO); an "output" is commanded from the bus (RPDO)
        // and driven into the world.
        [0x6000] inputs: record {
            [1] button: u8 = 0, ro, pdo;      // PB7 (0=released, 1=pressed)
        };
        [0x6200] outputs: record {
            [1] led: u8 = 0, rw, pdo;         // PB8 (0=off, 1=on)
        };

        // Bus-loopback test object. It has no physical-world meaning, so it
        // lives in the manufacturer-specific area (0x2000..=0x5FFF) instead
        // of the device-profile area. Names are from the device's view:
        // echo_in arrives from the bus, echo_out is sent back.
        [0x2000] echo: record {
            [1] echo_in: u16 = 0, rw, pdo;    // written by remote
            [2] echo_out: u16 = 0, ro, pdo;   // node mirrors echo_in here
        };

        // TPDO1: data this node sends (0x181 for node 1).
        // - event_driven: send on change, not tied to SYNC. Other options:
        //   sync_acyclic, sync_cyclic(N), or a raw CiA 301 value (e.g. 255).
        // - inhibit_time: minimum spacing between sends. event_timer: periodic
        //   fallback — send even if nothing changed (omit to disable). Both
        //   take unit suffixes (50ms, 0.1s, 500us) or raw CiA 301 values.
        // - Fields are packed into one CAN frame: [button (1 byte) | echo_out (2 bytes)]
        tpdo[1](transmission_type = event_driven, inhibit_time = 50ms, event_timer = 1s) {
            button,
            echo_out,
        };

        // RPDO1: data this node receives (0x201 for node 1).
        // - event_driven: apply values to the OD immediately on arrival. With
        //   sync_acyclic, values would be buffered until the next SYNC pulse
        //   (useful for coordinated updates).
        // - Fields are unpacked from the CAN frame: [led (1 byte) | echo_in (2 bytes)]
        // - Writing to these emits a typed NodeOdChange, which wakes main via
        //   NODE.wait_for_change().
        rpdo[1](transmission_type = event_driven) {
            led,
            echo_in,
        };
    }
}

// ---------- Shared state ----------

static TX_CHANNEL: Channel<CriticalSectionRawMutex, CanFrame, 16> = Channel::new();
static RX_CHANNEL: Channel<CriticalSectionRawMutex, CanFrame, 16> = Channel::new();

/// The CANopen node, shared between the protocol task and main.
static NODE: SharedNode<NodeOd, 1, 1> = SharedNode::new();

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
            Ok(f) => {
                tx.write(&f).await;
            }
            Err(_) => {
                warn!("Failed to create CAN frame");
            }
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

// ---------- Protocol task (background) ----------
//
// The CANopen stack runs here: heartbeat, SDO server, PDO engines.
// Application code doesn't touch this — it just calls od_mut() and
// reads od() from main.

#[embassy_executor::task]
async fn protocol_task() {
    let mut transport = ChannelTransport;
    let clock = EmbassyClock;
    let mut ticker = Ticker::every(Duration::from_millis(1));

    loop {
        ticker.next().await;
        NODE.with(|node| node.process(&mut transport, &clock));
    }
}

// ---------- Main ----------

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("CANopen node starting...");

    // Clock config for Nucleo-G431KB
    // Internal HSI 16MHz → PLL → 170MHz
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4, // 16MHz / 4 = 4MHz
            mul: PllMul::MUL85,      // 4MHz * 85 = 340MHz VCO
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2), // 340MHz / 2 = 170MHz
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.ahb_pre = AHBPrescaler::DIV1; // 170MHz
        config.rcc.apb1_pre = APBPrescaler::DIV1; // 170MHz
        config.rcc.apb2_pre = APBPrescaler::DIV1; // 170MHz
        config.rcc.mux.fdcansel = mux::Fdcansel::PCLK1;
    }

    let p = embassy_stm32::init(config);

    // GPIO setup
    let mut led = Output::new(p.PB8, Level::Low, Speed::Low);
    let mut button = ExtiInput::new(p.PB7, p.EXTI7, Pull::Up);

    // FDCAN1: PA11 (RX) / PA12 (TX), 500 kbit/s
    let mut can = CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    can.set_bitrate(500_000);
    can.properties().set_standard_filter(
        StandardFilterSlot::_0,
        StandardFilter::accept_all_into_fifo0(),
    );
    let can = can.into_normal_mode();
    let (tx, rx, _props) = can.split();

    spawner.must_spawn(can_tx_task(tx));
    spawner.must_spawn(can_rx_task(rx));

    // CANopen node — PDO config comes from the OD (declared in the macro above)
    let node_id = NodeId::new(1).unwrap();
    let od = NodeOd::new();
    let node: NodeOdNode = Node::new(
        NodeConfig {
            heartbeat_interval_ms: 500,
            auto_start: true,
            ..NodeConfig::from_od(&od, node_id)
        },
        od,
    );

    info!(
        "node {} running — TPDO1 {:#05X}, RPDO1 {:#05X}",
        node_id.raw(),
        node.tpdo_cob_id(0).unwrap(),
        node.rpdo_cob_id(0).unwrap()
    );

    NODE.init(node);

    // Protocol runs in the background
    spawner.must_spawn(protocol_task());

    // ---------- Application logic ----------
    //
    // Main loop: wait for either an OD event (remote wrote something)
    // or a button edge (local GPIO interrupt). No polling.

    loop {
        match select(NODE.wait_for_change(), button.wait_for_any_edge()).await {
            // Protocol stack changed the OD (SDO download or RPDO write).
            // next_change() decodes events into NodeOdChange — one variant per
            // writable OD entry, carrying the current value. The match is
            // exhaustive: adding a writable field to the OD is a compile
            // error here until it's handled.
            Either::First(_) => {
                NODE.with(|node| {
                    while let Some(change) = node.next_change() {
                        match change {
                            NodeOdChange::Led(v) => {
                                let on = v != 0;
                                info!("LED {}", if on { "on" } else { "off" });
                                if on {
                                    led.set_high()
                                } else {
                                    led.set_low()
                                }
                            }
                            NodeOdChange::EchoIn(v) => {
                                info!("echo {:#06X}", v);
                                node.od_mut().echo_out = v;
                            }
                        }
                    }
                });
            }

            // Button edge (EXTI interrupt)
            Either::Second(_) => {
                let pressed = button.is_low();
                info!("button {}", if pressed { "dn" } else { "up" });
                NODE.with(|node| node.od_mut().button = pressed as u8);
            }
        }
    }
}
