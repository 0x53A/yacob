//! Example: run a CANopen node on vcan0.
//!
//! Setup:
//!   sudo modprobe vcan
//!   sudo ip link add dev vcan0 type vcan
//!   sudo ip link set up vcan0
//!
//! Then run:
//!   cargo run --example vcan_node
//!
//! In another terminal, use candump/cansend or the vcan_client example to interact.

use canopen_core::cobid::NodeId;
use canopen_core::node::{Node, NodeConfig};
use canopen_core::pdo::{RpdoConfig, TpdoConfig};
use canopen_core::time::Clock;
use canopen_derive::object_dictionary;
use canopen_linux::SocketcanTransport;
use std::time::Instant;

object_dictionary! {
    pub struct DemoOd {
        [0x1000] device_type: u32 = 0x0000_0191, ro;
        [0x1001] error_register: u8 = 0x00, ro;
        [0x1017] heartbeat_time: u16 = 500, rw;
        [0x1018] identity: record {
            [1] vendor_id: u32 = 0x0000_CAFE, ro;
            [2] product_code: u32 = 0x0001, ro;
            [3] revision: u32 = 0x0001_0000, ro;
            [4] serial_number: u32 = 0x0000_0001, ro;
        };
        [0x6000] inputs: record {
            [1] input1: u8 = 0, ro, pdo;
            [2] input2: u16 = 0, ro, pdo;
        };
        [0x6200] outputs: record {
            [1] output1: u8 = 0, rw, pdo;
            [2] output2: u16 = 0, rw, pdo;
        };
    }
}

struct StdClock {
    start: Instant,
}

impl StdClock {
    fn new() -> Self {
        Self {
            start: Instant::now(),
        }
    }
}

impl Clock for StdClock {
    fn now_us(&self) -> u64 {
        self.start.elapsed().as_micros() as u64
    }
}

fn main() {
    let node_id = NodeId::new(1).unwrap();

    let config = NodeConfig::<1, 1> {
        node_id,
        heartbeat_interval_ms: 500,
        auto_start: true,
        tpdo: [TpdoConfig::default()],
        rpdo: [RpdoConfig::default()],
    };

    let od = DemoOd::new();
    let mut node = Node::new(config, od);

    let mut transport = SocketcanTransport::open("vcan0").expect(
        "Failed to open vcan0. Set up with:\n  \
         sudo modprobe vcan\n  \
         sudo ip link add dev vcan0 type vcan\n  \
         sudo ip link set up vcan0",
    );

    let clock = StdClock::new();

    println!("CANopen node {} running on vcan0", node_id.raw());
    println!("Heartbeat every 500ms, use candump vcan0 to see traffic");
    println!("SDO on 0x601/0x581, heartbeat on 0x701");

    loop {
        node.process(&mut transport, &clock);
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
}
