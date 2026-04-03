//! Test GPIO integration: read button/uptime, toggle LED via SDO.

use canopen_core::cobid::NodeId;
use canopen_core::transport::Transport;
use canopen_linux::sdo_helpers::*;
use canopen_linux::slcan::{SlcanBitrate, SlcanTransport};
use std::time::Duration;

fn main() {
    let mut slcan =
        SlcanTransport::open("/dev/ttyACM1", SlcanBitrate::S6).expect("Failed to open SLCAN");

    let target = NodeId::new(1).unwrap();
    let timeout = Duration::from_secs(3);

    // Wait for heartbeat
    println!("Waiting for heartbeat...");
    let state = wait_heartbeat(&mut slcan, target, timeout).expect("No heartbeat");
    println!("  Node alive, state: 0x{:02X}\n", state);

    // Read button state (0x6000:1)
    println!("Reading button (0x6000:1)...");
    let data = sdo_upload(&mut slcan, target, 0x6000, 1, timeout).expect("SDO failed");
    println!("  Button: {} (0=released, 1=pressed)\n", data[0]);

    // Read uptime (0x6000:2)
    println!("Reading uptime (0x6000:2)...");
    let data = sdo_upload(&mut slcan, target, 0x6000, 2, timeout).expect("SDO failed");
    let uptime = u16::from_le_bytes([data[0], data[1]]);
    println!("  Uptime: {}s\n", uptime);

    // Turn LED on
    println!("Turning LED ON (0x6200:1 = 1)...");
    sdo_download(&mut slcan, target, 0x6200, 1, &[1], timeout).expect("SDO failed");
    println!("  LED should be ON now");
    std::thread::sleep(Duration::from_secs(2));

    // Turn LED off
    println!("Turning LED OFF (0x6200:1 = 0)...");
    sdo_download(&mut slcan, target, 0x6200, 1, &[0], timeout).expect("SDO failed");
    println!("  LED should be OFF now");

    // Read uptime again
    let data = sdo_upload(&mut slcan, target, 0x6000, 2, timeout).expect("SDO failed");
    let uptime2 = u16::from_le_bytes([data[0], data[1]]);
    println!("\n  Uptime: {}s (delta: {}s)", uptime2, uptime2 - uptime);
}
