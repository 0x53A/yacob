#![no_std]
#![no_main]

use canopen_core::cobid::NodeId;
use canopen_core::node::{Node, NodeConfig};
use canopen_core::pdo::{RpdoConfig, TpdoConfig};
use canopen_core::time::Clock;
use canopen_core::transport::{CanFrame, Transport};
use canopen_derive::object_dictionary;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::can::filter::{StandardFilter, StandardFilterSlot};
use embassy_stm32::can::{CanConfigurator, CanRx, CanTx, Frame};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
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
        [0x6000] inputs: record {
            [1] input1: u8 = 0, ro, pdo;  // PB7 button (active low, pull-up)
            [2] input2: u16 = 0, ro, pdo; // uptime in seconds
        };
        [0x6200] outputs: record {
            [1] output1: u8 = 0, rw, pdo;  // PB8 LED (0=off, nonzero=on)
            [2] output2: u16 = 0, rw, pdo; // unused, for testing
        };
    }
}

// ---------- CAN channels ----------

static TX_CHANNEL: Channel<CriticalSectionRawMutex, CanFrame, 16> = Channel::new();
static RX_CHANNEL: Channel<CriticalSectionRawMutex, CanFrame, 16> = Channel::new();

// ---------- Clock ----------

struct EmbassyClock;

impl Clock for EmbassyClock {
    fn now_us(&self) -> u64 {
        Instant::now().as_micros()
    }
}

// ---------- Channel-based Transport ----------

struct ChannelTransport;

impl Transport for ChannelTransport {
    fn send(&mut self, frame: &CanFrame) -> Result<(), canopen_core::transport::TransportError> {
        TX_CHANNEL
            .try_send(*frame)
            .map_err(|_| canopen_core::transport::TransportError::TxBufferFull)
    }

    fn recv(&mut self) -> Option<CanFrame> {
        RX_CHANNEL.try_receive().ok()
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
        let id = embedded_can::StandardId::new(frame.id()).unwrap();
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
                if let Some(can_frame) = CanFrame::from_frame(&envelope.frame) {
                    let _ = RX_CHANNEL.try_send(can_frame);
                }
            }
            Err(e) => {
                warn!("CAN RX bus error: {:?}", defmt::Debug2Format(&e));
            }
        }
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
            prediv: PllPreDiv::DIV4,   // 16MHz / 4 = 4MHz
            mul: PllMul::MUL85,        // 4MHz * 85 = 340MHz VCO
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2), // 340MHz / 2 = 170MHz
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;  // 170MHz
        config.rcc.apb1_pre = APBPrescaler::DIV1;  // 170MHz
        config.rcc.apb2_pre = APBPrescaler::DIV1;  // 170MHz
        config.rcc.mux.fdcansel = mux::Fdcansel::PCLK1;
    }

    let p = embassy_stm32::init(config);

    // GPIO setup
    // PB8: onboard LED (active high on Nucleo-G431KB)
    let mut led = Output::new(p.PB8, Level::Low, Speed::Low);
    // PB7: button input (active low, external or jumper to GND)
    let button = Input::new(p.PB7, Pull::Up);

    // FDCAN1: PA11 (RX) / PA12 (TX)
    let mut can = CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);

    // 500 kbit/s classic CAN
    can.set_bitrate(500_000);

    // Accept all standard frames
    can.properties()
        .set_standard_filter(StandardFilterSlot::_0, StandardFilter::accept_all_into_fifo0());

    let can = can.into_normal_mode();
    let (tx, rx, _props) = can.split();

    spawner.must_spawn(can_tx_task(tx));
    spawner.must_spawn(can_rx_task(rx));

    // Create CANopen node
    let node_id = NodeId::new(1).unwrap();
    let node_config = NodeConfig::<1, 1> {
        node_id,
        heartbeat_interval_ms: 500,
        auto_start: true,
        tpdo: [TpdoConfig::default()],
        rpdo: [RpdoConfig::default()],
    };

    let od = NodeOd::new();
    let mut node = Node::new(node_config, od);
    let mut transport = ChannelTransport;
    let clock = EmbassyClock;

    info!("CANopen node {} running, heartbeat 500ms", node_id.raw());

    // Main loop: process protocol at ~1kHz
    let mut ticker = Ticker::every(Duration::from_millis(1));
    loop {
        // Read inputs → OD
        node.od_mut().input1 = if button.is_low() { 1 } else { 0 };
        node.od_mut().input2 = (Instant::now().as_secs() & 0xFFFF) as u16;

        // OD → outputs
        if node.od().output1 != 0 {
            led.set_high();
        } else {
            led.set_low();
        }

        node.process(&mut transport, &clock);
        ticker.next().await;
    }
}
