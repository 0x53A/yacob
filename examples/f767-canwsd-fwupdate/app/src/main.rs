//! Small relocatable application used to prove app-to-updater handoff.

#![no_std]
#![no_main]

use canopen_core::cobid::NodeId;
use canopen_core::node::{Node, NodeConfig, SharedNode};
use canopen_derive::object_dictionary;
use cortex_m::peripheral::SCB;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_net::StackResources;
use embassy_stm32::eth::{Ethernet, GenericPhy, PacketQueue, Sma};
use embassy_stm32::{bind_interrupts, eth, peripherals};
use embassy_time::{Duration, Ticker, Timer};
use static_cell::StaticCell;

use f767_canwsd_fwupdate_common as fw;
use fw::hal::{ChannelTransport, EmbassyClock, Nvm, StatusLed};

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    ETH => eth::InterruptHandler;
});

type Device = Ethernet<'static, peripherals::ETH, GenericPhy<Sma<'static, peripherals::ETH_SMA>>>;

object_dictionary! {
    #[validate_write(check_write)]
    pub struct AppOd {
        [0x1000] device_type: u32 = 0, ro;
        [0x1001] error_register: u8 = 0, ro;
        [0x1008] device_name: visible_string<32>, ro;
        [0x1018] identity: record {
            [1] vendor_id: u32 = 0, ro;
            [2] product_code: u32 = 0, ro;
            [3] revision: u32 = 1, ro;
            [4] serial_number: u32 = 0, ro;
        };
        [0x2F00] enter_update: u32 = 0, rw;
        [0x6000] demo_status: record {
            [1] uptime_s: u32 = 0, ro, pdo;
        };
        tpdo[1](transmission_type = event_driven, inhibit_time = 50ms, event_timer = 1s) {
            uptime_s,
        };
    }
}

impl AppOd {
    fn check_write(
        &self,
        index: u16,
        subindex: u8,
        data: &[u8],
    ) -> Result<(), canopen_core::od::OdError> {
        use canopen_core::od::OdError;
        if (index, subindex) == (0x2F00, 0) {
            let value = u32::from_le_bytes(data.try_into().map_err(|_| OdError::ValueRange)?);
            if value != fw::ENTER_UPDATE_MAGIC {
                return Err(OdError::ValueRange);
            }
        }
        Ok(())
    }
}

static NODE: SharedNode<AppOd, { AppOd::TPDO_COUNT }, { AppOd::RPDO_COUNT }> = SharedNode::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(fw::hal::clock_config());
    let (meta, _) = fw::load_meta();
    info!("F767 demo app, metadata seq {}", meta.seq);
    let mut led = StatusLed::new(p.PB0);
    let mut nvm = Nvm::new(p.FLASH);

    let mac = fw::hal::mac_from_uid();
    static PACKETS: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
    let device = Ethernet::new(
        PACKETS.init(PacketQueue::new()),
        p.ETH,
        Irqs,
        p.PA1,
        p.PA7,
        p.PC4,
        p.PC5,
        p.PG13,
        p.PB13,
        p.PG11,
        mac,
        p.ETH_SMA,
        p.PA2,
        p.PC1,
    );
    let mut dhcp = embassy_net::DhcpConfig::default();
    dhcp.hostname = Some(heapless::String::try_from("f767-demo").unwrap());
    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        device,
        embassy_net::Config::dhcpv4(dhcp),
        RESOURCES.init(StackResources::new()),
        0,
    );
    spawner.spawn(net_task(runner).unwrap());
    spawner.spawn(fw::ws::canwsd_task(stack).unwrap());

    let mut od = AppOd::new();
    od.vendor_id = fw::VENDOR_ID;
    od.product_code = fw::PRODUCT_CODE_APP;
    od.revision = meta.seq;
    od.serial_number = fw::hal::serial_from_uid();
    od.device_name.push_str("F767 CANopen demo app").unwrap();
    let id = NodeId::new(fw::NODE_ID).unwrap();
    let node: AppOdNode = Node::new(
        NodeConfig {
            heartbeat_interval_ms: 500,
            auto_start: true,
            ..NodeConfig::from_od(&od, id)
        },
        od,
    );
    NODE.init(node);
    spawner.spawn(protocol_task().unwrap());
    info!("demo app ready on TCP {}", fw::HTTP_PORT);

    let mut seconds = 0u32;
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        match select(NODE.wait_for_change(), ticker.next()).await {
            Either::First(()) => {
                while let Some(change) = NODE.with(|node| node.next_change()) {
                    let AppOdChange::EnterUpdate(_) = change;
                    let (current, _) = fw::load_meta();
                    let request = current.next(|r| r.update_requested = true);
                    match fw::append_meta(&mut nvm, request) {
                        Ok(()) => {
                            info!("entering updater");
                            Timer::after_millis(50).await;
                            SCB::sys_reset();
                        }
                        Err(_) => warn!("metadata journal full or flash write failed"),
                    }
                }
            }
            Either::Second(()) => {
                seconds = seconds.wrapping_add(1);
                NODE.with(|node| node.od_mut().uptime_s = seconds);
                led.toggle();
            }
        }
    }
}

#[embassy_executor::task]
async fn protocol_task() -> ! {
    let mut transport = ChannelTransport;
    let clock = EmbassyClock;
    loop {
        NODE.with(|node| node.process(&mut transport, &clock));
        Timer::after_millis(1).await;
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device>) -> ! {
    runner.run().await
}
