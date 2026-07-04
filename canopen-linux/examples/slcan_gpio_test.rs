//! Test GPIO integration: read button, toggle LED, check echo loopback via SDO.

use canopen_core::cobid::NodeId;
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

    // Turn LED on
    println!("Turning LED ON (0x6200:1 = 1)...");
    sdo_download(&mut slcan, target, 0x6200, 1, &[1], timeout).expect("SDO failed");
    println!("  LED should be ON now");
    std::thread::sleep(Duration::from_secs(2));

    // Turn LED off
    println!("Turning LED OFF (0x6200:1 = 0)...");
    sdo_download(&mut slcan, target, 0x6200, 1, &[0], timeout).expect("SDO failed");
    println!("  LED should be OFF now");

    // Echo loopback: write echo_in (0x2000:1), node mirrors to echo_out (0x2000:2)
    println!("\nWriting echo_in (0x2000:1) = 0x1234...");
    sdo_download(&mut slcan, target, 0x2000, 1, &0x1234u16.to_le_bytes(), timeout)
        .expect("SDO failed");
    let data = sdo_upload(&mut slcan, target, 0x2000, 2, timeout).expect("SDO failed");
    let echoed = u16::from_le_bytes([data[0], data[1]]);
    println!("  echo_out (0x2000:2): {:#06X}", echoed);
    assert_eq!(echoed, 0x1234, "echo mismatch");
}
