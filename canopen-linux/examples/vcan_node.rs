//! Example: run a CANopen node on vcan0 or UDP multicast.
//!
//! Socketcan (default):
//!   sudo modprobe vcan
//!   sudo ip link add dev vcan0 type vcan
//!   sudo ip link set up vcan0
//!   cargo run --example vcan_node
//!
//! UDP multicast (no root needed):
//!   CAN_TRANSPORT=udp cargo run --example vcan_node
//!
//! In another terminal, use candump/cansend or the vcan_client example to interact.

use canopen_core::cobid::NodeId;
use canopen_core::lss::LssIdentity;
use canopen_core::node::{Node, NodeConfig};
use canopen_core::time::Clock;
use canopen_core::transport::CanFrame;
use canopen_derive::object_dictionary;
use canopen_linux::SocketcanTransport;
use canopen_linux::UdpMulticastTransport;
use std::time::Instant;

object_dictionary! {
    #[export_eds(path = "../interop-tests/vcan_node.eds")]
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

        // TPDO1: send input1 + input2 on event (0x181 for node 1)
        tpdo[1](transmission_type = 255, inhibit_time = 0, event_timer = 500) {
            input1,
            input2,
        };

        // RPDO1: receive output1 + output2 (0x201 for node 1)
        rpdo[1](transmission_type = 255) {
            output1,
            output2,
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

fn run_node(transport: &mut impl embedded_can::nb::Can<Frame = CanFrame>) {
    let node_id = NodeId::new(1).unwrap();
    let od = DemoOd::new();

    let config = NodeConfig::<1, 1> {
        node_id,
        heartbeat_interval_ms: 500,
        auto_start: false,
        tpdo: od.tpdo_configs(node_id),
        rpdo: od.rpdo_configs(node_id),
        identity: LssIdentity {
            vendor_id: 0xCAFE,
            product_code: 0x0001,
            revision: 0x00010000,
            serial: 0x00000001,
        },
    };

    let mut node: Node<DemoOd, 1, 1> = Node::new(config, od);
    let clock = StdClock::new();

    loop {
        node.process(transport, &clock);

        // Mirror output1 -> input1 for RPDO→TPDO echo test
        let out1 = node.od().output1;
        let out2 = node.od().output2;
        if node.od().input1 != out1 || node.od().input2 != out2 {
            let mut od = node.od_mut();
            od.input1 = out1;
            od.input2 = out2;
        }

        // EMCY test: writing 0xEE to output1 triggers an error,
        // writing 0x00 clears it
        while let Some(evt) = node.next_event() {
            if evt.index == 0x6200 && evt.subindex == 1 {
                match node.od().output1 {
                    0xEE => node.set_error(0x1000, canopen_core::error_register::GENERIC, &[]),
                    0x00 => node.clear_all_errors(),
                    _ => {}
                }
            }
        }

        std::thread::sleep(std::time::Duration::from_millis(1));
    }
}

fn main() {
    let transport_type = std::env::var("CAN_TRANSPORT").unwrap_or("socketcan".into());

    match transport_type.as_str() {
        "udp" => {
            eprintln!("CANopen node 1 running on UDP multicast (239.74.163.2:43113)");
            eprintln!("Heartbeat every 500ms, auto_start=false (waiting for NMT Start)");
            let mut transport = UdpMulticastTransport::new(None, None)
                .expect("Failed to create UDP multicast transport");
            run_node(&mut transport);
        }
        _ => {
            let iface = std::env::var("CAN_IFACE").unwrap_or("vcan0".into());
            eprintln!("CANopen node 1 running on {}", iface);
            eprintln!("Heartbeat every 500ms, auto_start=false (waiting for NMT Start)");
            let mut transport = SocketcanTransport::open(&iface).expect(&format!(
                "Failed to open {}. Set up with:\n  \
                     sudo modprobe vcan\n  \
                     sudo ip link add dev {} type vcan\n  \
                     sudo ip link set up {}",
                iface, iface, iface
            ));
            run_node(&mut transport);
        }
    }
}
