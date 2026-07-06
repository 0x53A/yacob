//! Blink the onboard LED via CANopen SDO.

use canopen_core::cobid::NodeId;
use canopen_linux::sdo_helpers::*;
use canopen_linux::slcan::SlcanBitrate;
use std::time::Duration;

fn main() {
    let mut slcan =
        canopen_linux::slcan::open("/dev/ttyACM1", SlcanBitrate::S6).expect("Failed to open SLCAN");
    let target = NodeId::new(1).unwrap();
    let timeout = Duration::from_secs(2);

    wait_heartbeat(&mut slcan, target, timeout).expect("No heartbeat");

    for i in 0..5 {
        sdo_download(&mut slcan, target, 0x6200, 1, &[1], timeout).unwrap();
        println!("blink {} ON", i + 1);
        std::thread::sleep(Duration::from_millis(300));

        sdo_download(&mut slcan, target, 0x6200, 1, &[0], timeout).unwrap();
        println!("blink {} OFF", i + 1);
        std::thread::sleep(Duration::from_millis(300));
    }
}
