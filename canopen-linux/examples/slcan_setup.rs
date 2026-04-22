//! One-time SLCAN adapter setup.
//!
//! Run this after plugging in the ESP32 SLCAN adapter to initialize it.
//! After this, all other tools (slcan_listen, HIL tests, etc.) will work
//! without needing python-can.
//!
//! Usage: cargo run --example slcan_setup -- /dev/ttyACM1

use embedded_can::nb::Can;
use canopen_linux::slcan::{SlcanBitrate, SlcanTransport};
use std::time::{Duration, Instant};

fn main() {
    let port = std::env::args().nth(1).unwrap_or("/dev/ttyACM1".into());

    println!("Resetting SLCAN adapter on {}...", port);

    // Reset via espflash
    let status = std::process::Command::new("espflash")
        .args(["reset", "--port", &port])
        .status();

    match status {
        Ok(s) if s.success() => println!("  Device reset OK"),
        _ => {
            eprintln!("  espflash reset failed — is espflash installed?");
            std::process::exit(1);
        }
    }

    // Wait for boot
    println!("Waiting for device to boot...");
    std::thread::sleep(Duration::from_secs(3));

    // Now open — this will init via S6/O
    println!("Initializing SLCAN (S6, O)...");
    let mut slcan = SlcanTransport::open(&port, SlcanBitrate::S6).expect("Failed to open SLCAN");

    // Verify
    println!("Verifying — listening for CAN frames...");
    let start = Instant::now();
    let mut count = 0;
    while start.elapsed() < Duration::from_secs(3) {
        if let Ok(frame) = slcan.receive() {
            if count == 0 {
                println!(
                    "  First frame: ID=0x{:03X} DLC={} data={:02X?}",
                    frame.raw_id(),
                    frame.raw_dlc(),
                    frame.data()
                );
            }
            count += 1;
        }
        std::thread::sleep(Duration::from_millis(1));
    }

    if count > 0 {
        println!("  Got {} frames in 3s — adapter is ready!", count);
    } else {
        eprintln!("  No frames received. Is the CAN bus active?");
        std::process::exit(1);
    }
}
