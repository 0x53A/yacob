//! Example: SDO client talking to a node on vcan0.
//!
//! Run the vcan_node example first, then this one.

use canopen_core::cobid::NodeId;
use canopen_linux::sdo_helpers::*;
use canopen_linux::SocketcanTransport;
use std::time::Duration;

fn main() {
    let mut transport = SocketcanTransport::open("vcan0").expect("Failed to open vcan0");
    let target = NodeId::new(1).unwrap();
    let timeout = Duration::from_secs(2);

    println!("=== CANopen SDO Client ===\n");

    // Wait for heartbeat to confirm node is alive
    println!("Waiting for heartbeat from node {}...", target.raw());
    match wait_heartbeat(&mut transport, target, timeout) {
        Ok(state) => println!("  Node alive, state: 0x{:02X}", state),
        Err(e) => {
            eprintln!("  No heartbeat: {}", e);
            return;
        }
    }

    // Send NMT Start to enter Operational
    println!("\nSending NMT Start...");
    nmt_command(&mut transport, 0x01, target.raw()).unwrap();
    std::thread::sleep(Duration::from_millis(100));

    // Read device type (0x1000:0)
    println!("\nReading 0x1000:0 (Device Type)...");
    match sdo_upload(&mut transport, target, 0x1000, 0, timeout) {
        Ok(data) => {
            let val = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
            println!("  Device Type: 0x{:08X}", val);
        }
        Err(e) => eprintln!("  Error: {}", e),
    }

    // Read vendor ID (0x1018:1)
    println!("\nReading 0x1018:1 (Vendor ID)...");
    match sdo_upload(&mut transport, target, 0x1018, 1, timeout) {
        Ok(data) => {
            let val = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
            println!("  Vendor ID: 0x{:08X}", val);
        }
        Err(e) => eprintln!("  Error: {}", e),
    }

    // Write output1 (0x6200:1)
    println!("\nWriting 0x6200:1 (output1) = 0x42...");
    match sdo_download(&mut transport, target, 0x6200, 1, &[0x42], timeout) {
        Ok(()) => println!("  Write OK"),
        Err(e) => eprintln!("  Error: {}", e),
    }

    // Read back output1
    println!("\nReading 0x6200:1 (output1)...");
    match sdo_upload(&mut transport, target, 0x6200, 1, timeout) {
        Ok(data) => println!("  output1 = 0x{:02X}", data[0]),
        Err(e) => eprintln!("  Error: {}", e),
    }

    // Write output2 (0x6200:2)
    println!("\nWriting 0x6200:2 (output2) = 0xBEEF...");
    match sdo_download(
        &mut transport,
        target,
        0x6200,
        2,
        &0xBEEFu16.to_le_bytes(),
        timeout,
    ) {
        Ok(()) => println!("  Write OK"),
        Err(e) => eprintln!("  Error: {}", e),
    }

    // Try writing to read-only (should fail)
    println!("\nWriting 0x1000:0 (read-only, should fail)...");
    match sdo_download(&mut transport, target, 0x1000, 0, &[0; 4], timeout) {
        Ok(()) => eprintln!("  Unexpected success!"),
        Err(e) => println!("  Expected error: {}", e),
    }

    println!("\nDone!");
}
