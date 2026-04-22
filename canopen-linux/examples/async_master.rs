//! Async CANopen master — monitors nodes and performs SDO operations using CanDemux.
//!
//! Demonstrates the async master-side API:
//! - `CanDemux` for routing CAN frames to SDO/heartbeat/PDO buffers
//! - `SdoDriver` with `DemuxSdoPort` for SDO transfers while buffering other traffic
//! - `upload_timed`/`download_timed` for deadline-based SDO with timeout
//! - `NmtCommand::to_frame()` for sending NMT commands
//! - Heartbeat monitoring via `CanDemux::recv_heartbeat()`
//!
//! Run with vcan_node or sensor_hub as the target:
//!   # Terminal 1: start a node
//!   CAN_TRANSPORT=udp cargo run --example vcan_node
//!   # Terminal 2: start the master
//!   CAN_TRANSPORT=udp cargo run --example async_master
//!
//! ## API notes
//!
//! The CanDemux pattern works well for masters that need to interleave SDO
//! operations with heartbeat monitoring. During an SDO transfer, incoming
//! heartbeats and PDOs are buffered in the demux. After the transfer, the
//! master drains the buffers to update its view of the network.
//!
//! For truly concurrent monitoring (heartbeat watchdog running independently
//! of SDO operations), you'd need separate tasks — either using Embassy on
//! embedded, or tokio/async-std on Linux. The CanDemux still helps by routing
//! frames to the right consumer.

use canopen_core::cobid::{CobId, NodeId, ParsedCobId};
use canopen_core::nmt::{NmtCommand, NmtState};
use canopen_core::sdo::driver::{AsyncCan, NbCanAsync, SdoDriver, SdoError};
use canopen_core::can_router::CanDemux;
use canopen_core::time::Clock;
use canopen_core::transport::CanFrame;
use canopen_linux::{SocketcanTransport, UdpMulticastTransport};
use std::time::{Duration, Instant};

struct StdClock(Instant);

impl StdClock {
    fn new() -> Self { Self(Instant::now()) }
}

impl Clock for StdClock {
    fn now_us(&self) -> u64 {
        self.0.elapsed().as_micros() as u64
    }
}

/// Minimal block_on executor (same pattern as sdo_helpers).
fn block_on<F: core::future::Future>(f: F) -> F::Output {
    use core::pin::pin;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    fn raw_waker() -> RawWaker {
        fn no_op(_: *const ()) {}
        fn clone(p: *const ()) -> RawWaker { RawWaker::new(p, &VTABLE) }
        const VTABLE: RawWakerVTable = RawWakerVTable::new(clone, no_op, no_op, no_op);
        RawWaker::new(core::ptr::null(), &VTABLE)
    }

    let waker = unsafe { Waker::from_raw(raw_waker()) };
    let mut cx = Context::from_waker(&waker);
    let mut f = pin!(f);

    loop {
        match f.as_mut().poll(&mut cx) {
            Poll::Ready(val) => return val,
            Poll::Pending => std::thread::sleep(Duration::from_micros(100)),
        }
    }
}

fn run_master(transport: impl embedded_can::nb::Can<Frame = CanFrame, Error: core::fmt::Debug>) {
    let clock = StdClock::new();
    let target = NodeId::new(1).unwrap();
    let sdo = SdoDriver::new(target);

    // Wrap the nb::Can transport as AsyncCan, then put it in a CanDemux.
    // The demux routes frames: SDO responses go to the SDO port, everything
    // else (heartbeats, PDOs) goes to internal buffers.
    let async_transport = NbCanAsync(transport);
    let mut demux = CanDemux::new(async_transport);

    println!("=== Async CANopen Master ===\n");

    // Step 1: Wait for heartbeat (proves the node is alive)
    println!("[1] Waiting for heartbeat from node {}...", target.raw());
    let hb_result = block_on(async {
        demux.recv_heartbeat_timed(Some(target), 5_000_000, &clock).await
    });
    match hb_result {
        Ok(Some(frame)) => {
            let state = NmtState::from_heartbeat_byte(frame.data()[0]);
            println!("    Heartbeat received: {:?}", state);
        }
        Ok(None) => {
            eprintln!("    No heartbeat within 5s — is the node running?");
            return;
        }
        Err(e) => {
            eprintln!("    Transport error: {:?}", e);
            return;
        }
    }

    // Step 2: Send NMT Start (transition to Operational)
    println!("\n[2] Sending NMT Start...");
    let nmt_frame = NmtCommand::StartRemoteNode.to_frame(target.raw());
    block_on(async {
        demux.transport_mut().transmit(&nmt_frame).await.unwrap();
    });
    std::thread::sleep(Duration::from_millis(200));

    // Step 3: Read device identity using timed SDO operations through CanDemux
    println!("\n[3] Reading device identity (SDO with 2s timeout)...");
    block_on(async {
        let timeout_us = 2_000_000; // 2 seconds

        // Create an SDO port from the demux — this filters SDO responses for our
        // target node while buffering heartbeats/PDOs in the demux.
        {
            let mut port = demux.sdo_port(target);

            // Read device type (0x1000:0) with timeout
            let mut buf = [0u8; 4];
            match sdo.upload_timed(0x1000, 0, &mut buf, &mut port, timeout_us, &clock).await {
                Ok(_) => println!("    Device Type: 0x{:08X}", u32::from_le_bytes(buf)),
                Err(SdoError::Timeout) => println!("    Device Type: TIMEOUT"),
                Err(e) => println!("    Device Type: error {:?}", e),
            }

            // Read vendor ID (0x1018:1)
            let mut buf = [0u8; 4];
            match sdo.upload_timed(0x1018, 1, &mut buf, &mut port, timeout_us, &clock).await {
                Ok(_) => println!("    Vendor ID:   0x{:08X}", u32::from_le_bytes(buf)),
                Err(e) => println!("    Vendor ID:   error {:?}", e),
            }

            // Read product code (0x1018:2)
            match sdo.upload_timed(0x1018, 2, &mut buf, &mut port, timeout_us, &clock).await {
                Ok(_) => println!("    Product:     0x{:08X}", u32::from_le_bytes(buf)),
                Err(e) => println!("    Product:     error {:?}", e),
            }

            // Read revision (0x1018:3)
            match sdo.upload_timed(0x1018, 3, &mut buf, &mut port, timeout_us, &clock).await {
                Ok(_) => println!("    Revision:    0x{:08X}", u32::from_le_bytes(buf)),
                Err(e) => println!("    Revision:    error {:?}", e),
            }
        }

        // After SDO operations, check what heartbeats/PDOs arrived in the background
        let mut hb_count = 0;
        while let Some(frame) = demux.try_recv_heartbeat() {
            hb_count += 1;
            let _ = frame; // process if needed
        }
        let mut pdo_count = 0;
        while let Some(frame) = demux.try_recv_pdo() {
            pdo_count += 1;
            let _ = frame;
        }
        if hb_count > 0 || pdo_count > 0 {
            println!("    (buffered during SDO: {} heartbeats, {} PDOs)", hb_count, pdo_count);
        }
    });

    // Step 4: Write and read back a value
    println!("\n[4] SDO write/read test...");
    block_on(async {
        let timeout_us = 2_000_000;
        let mut port = demux.sdo_port(target);

        // Write output1 = 0x42
        match sdo.download_timed(0x6200, 1, &[0x42], &mut port, timeout_us, &clock).await {
            Ok(()) => println!("    Write 0x6200:1 = 0x42: OK"),
            Err(e) => println!("    Write 0x6200:1: error {:?}", e),
        }

        // Read back
        let mut buf = [0u8; 1];
        match sdo.upload_timed(0x6200, 1, &mut buf, &mut port, timeout_us, &clock).await {
            Ok(_) => println!("    Read  0x6200:1 = 0x{:02X}", buf[0]),
            Err(e) => println!("    Read  0x6200:1: error {:?}", e),
        }
    });

    // Step 5: Demonstrate timeout handling (read from non-existent node)
    println!("\n[5] Timeout test (reading from non-existent node 99)...");
    let fake_target = NodeId::new(99).unwrap();
    let fake_sdo = SdoDriver::new(fake_target);
    block_on(async {
        let mut port = demux.sdo_port(fake_target);
        let start = Instant::now();
        match fake_sdo.upload_timed(0x1000, 0, &mut [0u8; 4], &mut port, 500_000, &clock).await {
            Err(SdoError::Timeout) => {
                println!("    Timed out after {}ms (expected)", start.elapsed().as_millis());
            }
            other => println!("    Unexpected result: {:?}", other),
        }
    });

    // Step 6: Monitor heartbeats for a few seconds
    println!("\n[6] Monitoring heartbeats for 3 seconds...");
    let monitor_end = Instant::now() + Duration::from_secs(3);
    let mut last_state: Option<NmtState> = None;
    while Instant::now() < monitor_end {
        // Poll the transport for one frame at a time
        block_on(async {
            match demux.transport_mut().receive().await {
                Ok(frame) => {
                    if let Some(cob) = CobId::new(frame.raw_id()) {
                        match cob.parse() {
                            ParsedCobId::Heartbeat(node) if node == target => {
                                let state = NmtState::from_heartbeat_byte(frame.data()[0]);
                                if state != last_state {
                                    println!("    Node {} state: {:?}", node.raw(), state);
                                    last_state = state;
                                }
                            }
                            ParsedCobId::Tpdo { pdo_num, node } => {
                                println!("    TPDO{} from node {}: {:02X?}",
                                    pdo_num, node.raw(), frame.data());
                            }
                            _ => {}
                        }
                    }
                }
                Err(_) => {}
            }
        });
        std::thread::sleep(Duration::from_millis(10));
    }

    println!("\nDone!");
}

fn main() {
    let transport_type = std::env::var("CAN_TRANSPORT").unwrap_or("socketcan".into());

    match transport_type.as_str() {
        "udp" => {
            eprintln!("Async master on UDP multicast (239.74.163.2:43113)");
            let transport = UdpMulticastTransport::new(None, None)
                .expect("Failed to create UDP multicast transport");
            run_master(transport);
        }
        _ => {
            let iface = std::env::var("CAN_IFACE").unwrap_or("vcan0".into());
            eprintln!("Async master on {}", iface);
            let transport = SocketcanTransport::open(&iface)
                .expect("Failed to open CAN interface");
            run_master(transport);
        }
    }
}
