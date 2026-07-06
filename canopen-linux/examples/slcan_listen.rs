//! Listen for CAN frames via SLCAN on a serial port.
//!
//! Usage: cargo run --example slcan_listen -- /dev/ttyACM1

use canopen_linux::slcan::SlcanBitrate;
use embedded_can::nb::Can;
use std::time::{Duration, Instant};

fn main() {
    let port = std::env::args().nth(1).unwrap_or("/dev/ttyACM1".into());

    println!("Opening SLCAN on {}...", port);
    let mut slcan =
        canopen_linux::slcan::open(&port, SlcanBitrate::S6).expect("Failed to open SLCAN");

    println!("Listening for CAN frames (10 seconds)...");
    let start = Instant::now();
    let mut count = 0;

    while start.elapsed() < Duration::from_secs(10) {
        if let Ok(frame) = slcan.receive() {
            println!(
                "  ID=0x{:03X} DLC={} data={:02X?}",
                frame.raw_id(),
                frame.raw_dlc(),
                frame.data()
            );
            count += 1;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    println!("Received {} frames in 10 seconds", count);
}
