//! Hardware-in-the-loop tests for the CANopen stack.
//!
//! Run with:
//!   SLCAN_PORT=/dev/ttyACM1 cargo test -p hil-tests -- --test-threads=1 --ignored
//!
//! Tests are #[ignore]d by default so `cargo test` in the workspace doesn't fail.

use canopen_core::cobid::NodeId;
use canopen_linux::sdo_helpers::*;
use canopen_linux::slcan::{SlcanBitrate, SlcanTransport};
use std::sync::Mutex;
use std::time::Duration;

const TIMEOUT: Duration = Duration::from_secs(3);

fn target_node() -> NodeId {
    NodeId::new(1).unwrap()
}

/// Shared transport — opened once, reused across all tests.
/// Tests MUST run with --test-threads=1.
static TRANSPORT: Mutex<Option<SlcanTransport>> = Mutex::new(None);

fn with_transport<R>(f: impl FnOnce(&mut SlcanTransport) -> R) -> R {
    let mut guard = TRANSPORT.lock().unwrap_or_else(|e| e.into_inner());
    if guard.is_none() {
        let port =
            std::env::var("SLCAN_PORT").unwrap_or_else(|_| "/dev/ttyACM1".to_string());
        *guard = Some(
            SlcanTransport::open(&port, SlcanBitrate::S6)
                .unwrap_or_else(|e| panic!("Failed to open SLCAN on {}: {}", port, e)),
        );
    }
    f(guard.as_mut().unwrap())
}

#[test]
#[ignore]
fn t01_heartbeat() {
    with_transport(|transport| {
        let state = wait_heartbeat(transport, target_node(), TIMEOUT)
            .expect("No heartbeat received");
        assert!(
            state == 0x7F || state == 0x05 || state == 0x00,
            "Unexpected heartbeat state: 0x{:02X}",
            state
        );
    });
}

#[test]
#[ignore]
fn t02_nmt_start() {
    with_transport(|transport| {
        let _ = wait_heartbeat(transport, target_node(), TIMEOUT);
        nmt_command(transport, 0x01, target_node().raw()).unwrap();
        std::thread::sleep(Duration::from_millis(100));
        let state = wait_heartbeat(transport, target_node(), TIMEOUT).unwrap();
        assert_eq!(state, 0x05, "Expected Operational after NMT Start");
    });
}

#[test]
#[ignore]
fn t03_sdo_read_device_type() {
    with_transport(|transport| {
        let data = sdo_upload(transport, target_node(), 0x1000, 0, TIMEOUT)
            .expect("SDO upload failed");
        assert_eq!(data.len(), 4);
        let val = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        assert_eq!(val, 0x0000_0191);
    });
}

#[test]
#[ignore]
fn t04_sdo_read_identity() {
    with_transport(|transport| {
        let data = sdo_upload(transport, target_node(), 0x1018, 1, TIMEOUT).unwrap();
        let vendor_id = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        assert_eq!(vendor_id, 0x0000_CAFE);

        let data = sdo_upload(transport, target_node(), 0x1018, 2, TIMEOUT).unwrap();
        let product_code = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        assert_eq!(product_code, 0x0001);
    });
}

#[test]
#[ignore]
fn t05_sdo_write_readback() {
    with_transport(|transport| {
        sdo_download(transport, target_node(), 0x6200, 1, &[0xAB], TIMEOUT)
            .expect("SDO download failed");
        let data = sdo_upload(transport, target_node(), 0x6200, 1, TIMEOUT).unwrap();
        assert_eq!(data[0], 0xAB);

        sdo_download(
            transport,
            target_node(),
            0x6200,
            2,
            &0xBEEFu16.to_le_bytes(),
            TIMEOUT,
        )
        .expect("SDO download failed");
        let data = sdo_upload(transport, target_node(), 0x6200, 2, TIMEOUT).unwrap();
        assert_eq!(u16::from_le_bytes([data[0], data[1]]), 0xBEEF);
    });
}

#[test]
#[ignore]
fn t06_sdo_read_only_reject() {
    with_transport(|transport| {
        let result = sdo_download(transport, target_node(), 0x1000, 0, &[0; 4], TIMEOUT);
        assert!(result.is_err());
    });
}

#[test]
#[ignore]
fn t07_sdo_not_found() {
    with_transport(|transport| {
        let result = sdo_upload(transport, target_node(), 0xFFFF, 0, TIMEOUT);
        assert!(result.is_err());
    });
}

#[test]
#[ignore]
fn t08_nmt_stop_preop() {
    with_transport(|transport| {
        nmt_command(transport, 0x01, target_node().raw()).unwrap();
        std::thread::sleep(Duration::from_millis(100));

        nmt_command(transport, 0x02, target_node().raw()).unwrap();
        std::thread::sleep(Duration::from_millis(100));
        let state = wait_heartbeat(transport, target_node(), TIMEOUT).unwrap();
        assert_eq!(state, 0x04, "Expected Stopped");

        nmt_command(transport, 0x80, target_node().raw()).unwrap();
        std::thread::sleep(Duration::from_millis(100));
        let state = wait_heartbeat(transport, target_node(), TIMEOUT).unwrap();
        assert_eq!(state, 0x7F, "Expected PreOp");

        nmt_command(transport, 0x01, target_node().raw()).unwrap();
    });
}
