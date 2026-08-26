//! Demo application for the EMDM CANopen update PoC.
//!
//! Deliberately trivial, so that what is being demonstrated is the update
//! mechanism rather than the application: one RPDO bit drives the green
//! channel of the PCB LED, and a TPDO echoes the bit back together with an
//! uptime counter.
//!
//! It is linked at [`fw::APP_BASE`], not at the start of flash, and is reached
//! only through the bootloader's jump. The bootloader sets VTOR before
//! branching, so interrupt dispatch lands in this image's vector table.
//!
//! # Handing control back to the updater
//!
//! Writing [`fw::ENTER_UPDATE_MAGIC`] to object `0x2F00:0` appends a metadata
//! record with the one-shot `update_requested` flag and resets the MCU. The
//! bootloader sees the flag, clears it immediately, and stays in the updater.
//! Any other value is rejected with an SDO value-range abort so that a stray
//! write cannot take the node off the bus.

#![no_std]
#![no_main]

use canopen_core::cobid::NodeId;
use canopen_core::node::{Node, NodeConfig, SharedNode};
use canopen_derive::object_dictionary;
use cortex_m::peripheral::SCB;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Ticker, Timer};

use emdm_fwupdate_common as fw;
use fw::hal::{ChannelTransport, EmbassyClock, Nvm, StatusLed};

use {defmt_rtt as _, panic_probe as _};

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

        // The one mapped bit: RPDO1 bit 0 drives the LED.
        [0x6200] outputs: record {
            [1] led: bool = false, rw, pdo;
        };
        [0x6000] inputs: record {
            [1] led_state: bool = false, ro, pdo;
            [2] uptime_s: u32 = 0, ro, pdo;
        };

        // Write `ENTER_UPDATE_MAGIC` to reboot into the updater.
        [0x2F00] enter_update: u32 = 0, rw;

        rpdo[1](deadline = 2s) {
            led,
        };
        tpdo[1](transmission_type = event_driven, inhibit_time = 50ms, event_timer = 1s) {
            led_state,
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

static NODE: SharedNode<AppOd, { AppOd::TPDO_COUNT }, { AppOd::RPDO_COUNT }> =
    SharedNode::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(fw::hal::clock_config());
    let (meta, _) = fw::load_meta();
    info!(
        "EMDM demo application: node {}, linked at 0x{:08x}, image {} B, meta seq={}",
        fw::NODE_ID,
        fw::APP_BASE,
        meta.app_len,
        meta.seq
    );

    let mut led = StatusLed::new(p.PB9, p.PC13, p.PA15);
    let mut nvm = Nvm::new(p.FLASH);

    fw::hal::init_can(&spawner, p.FDCAN1, p.PA11, p.PA12);

    let node_id = NodeId::new(fw::NODE_ID).expect("node id in range");
    let mut od = AppOd::new();
    od.vendor_id = fw::VENDOR_ID;
    od.product_code = fw::PRODUCT_CODE_APP;
    od.revision = meta.seq;
    od.serial_number = fw::hal::serial_from_uid();
    od.device_name
        .push_str("EMDM CANopen demo app")
        .expect("device name fits");

    let node: AppOdNode = Node::new(
        NodeConfig {
            heartbeat_interval_ms: 500,
            auto_start: true,
            ..NodeConfig::from_od(&od, node_id)
        },
        od,
    );
    NODE.init(node);
    spawner.spawn(protocol_task().expect("allocate the protocol task"));
    info!("application ready: RPDO1 bit 0 -> green LED");

    let mut uptime = Ticker::every(Duration::from_secs(1));
    let mut seconds: u32 = 0;

    loop {
        match select(NODE.wait_for_change(), uptime.next()).await {
            Either::First(()) => {
                while let Some(change) = NODE.with(|n| n.next_change()) {
                    match change {
                        AppOdChange::Led(on) => {
                            led.set(false, on, false);
                            NODE.with(|n| n.od_mut().led_state = on);
                            info!("led {}", if on { "on" } else { "off" });
                        }
                        AppOdChange::EnterUpdate(_) => {
                            // `check_write` already rejected every value but
                            // the magic, so reaching here means it matched.
                            enter_update(&mut nvm);
                        }
                    }
                }
            }
            Either::Second(()) => {
                seconds = seconds.wrapping_add(1);
                NODE.with(|n| n.od_mut().uptime_s = seconds);

                // The RPDO deadline is the liveness check on the host: if it
                // stops refreshing, fall back to off rather than latching the
                // last commanded state.
                if NODE.with(|n| n.rpdo_deadline_expired(canopen_core::pdo::PdoNumber::of::<1>())) {
                    NODE.with(|n| {
                        n.od_mut().led = false;
                        n.od_mut().led_state = false;
                    });
                    led.set(false, false, false);
                }
            }
        }
    }
}

/// Record the one-shot update request and reset into the bootloader.
fn enter_update(nvm: &mut Nvm<'_>) {
    let (meta, _) = fw::load_meta();
    let request = meta.next(|r| r.update_requested = true);
    match fw::append_meta(nvm, &request) {
        Ok(()) => {
            info!("update requested (meta seq={}), resetting", request.seq);
            // Give the SDO response a chance to leave the controller before
            // the core disappears out from under it.
            cortex_m::asm::delay(170_000 * 20);
            SCB::sys_reset();
        }
        Err(e) => warn!("could not record the update request: {:?}", e),
    }
}

#[embassy_executor::task]
async fn protocol_task() -> ! {
    let mut transport = ChannelTransport;
    let clock = EmbassyClock;
    loop {
        NODE.with(|node| node.process(&mut transport, &clock));
        Timer::after(Duration::from_millis(1)).await;
    }
}
