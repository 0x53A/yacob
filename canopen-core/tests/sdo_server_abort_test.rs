//! SDO server abort codes for object access failures (CiA 301 Table 22).

use canopen_core::nmt::NmtState;
use canopen_core::sdo::server::SdoServer;
use canopen_core::sdo::AbortCode;
use canopen_derive::object_dictionary;

object_dictionary! {
    pub struct AbortOd {
        [0x1000] device_type: u32 = 0x191, ro;
        [0x1018] identity: record {
            [1] vendor_id: u32 = 1, ro;
        };
        [0x2000] level: u16 = 0, rw;
        [0x2001] label: visible_string<4>, rw;
    }
}

fn request(od: &mut AbortOd, req: [u8; 8]) -> u32 {
    let mut server = SdoServer::new();
    let mut resp = [0u8; 8];
    let mut events = heapless::Deque::<_, 4>::new();
    server
        .process(&req, od, &mut resp, &mut events, NmtState::PreOperational, 0)
        .unwrap();
    assert_eq!(resp[0], 0x80, "expected abort, got {resp:02X?}");
    u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]])
}

#[test]
fn missing_subindex_of_existing_object() {
    let mut od = AbortOd::new();
    let code = request(&mut od, [0x40, 0x18, 0x10, 0x09, 0, 0, 0, 0]);
    assert_eq!(code, AbortCode::SubindexNotFound as u32);
    let code = request(&mut od, [0x40, 0x99, 0x20, 0x00, 0, 0, 0, 0]);
    assert_eq!(code, AbortCode::ObjectNotFound as u32);
}

#[test]
fn length_mismatch_reports_direction() {
    let mut od = AbortOd::new();
    // Expedited, size indicated, 1 byte into a u16.
    let code = request(&mut od, [0x2F, 0x00, 0x20, 0x00, 1, 0, 0, 0]);
    assert_eq!(code, AbortCode::LengthTooLow as u32);
    // 4 bytes into a u16.
    let code = request(&mut od, [0x23, 0x00, 0x20, 0x00, 1, 2, 3, 4]);
    assert_eq!(code, AbortCode::LengthTooHigh as u32);
    assert_eq!(od.level, 0);
}

#[test]
fn overlong_string_is_length_too_high() {
    let mut od = AbortOd::new();
    // Segmented download initiate of 5 bytes into visible_string<4>.
    let mut server = SdoServer::new();
    let mut resp = [0u8; 8];
    let mut events = heapless::Deque::<_, 4>::new();
    server
        .process(&[0x21, 0x01, 0x20, 0x00, 5, 0, 0, 0], &mut od, &mut resp, &mut events, NmtState::PreOperational, 0)
        .unwrap();
    assert_eq!(resp[0], 0x60);
    server
        .process(&[0x05, b'h', b'e', b'l', b'l', b'o', 0, 0], &mut od, &mut resp, &mut events, NmtState::PreOperational, 0)
        .unwrap();
    assert_eq!(resp[0], 0x80);
    assert_eq!(
        u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]),
        AbortCode::LengthTooHigh as u32
    );
}

#[test]
fn segmented_transfer_disagreeing_with_declared_size() {
    // Declared 3, one 7-byte segment → too high. Declared 4, final segment
    // carrying 2 bytes (n = 5, c = 1) → too low.
    let cases = [
        (3u8, [0x01, b'a', b'b', b'c', b'd', b'e', b'f', b'g'], AbortCode::LengthTooHigh),
        (4, [0x01 | (5 << 1), b'a', b'b', 0, 0, 0, 0, 0], AbortCode::LengthTooLow),
    ];
    for (declared, segment, expected) in cases {
        let mut od = AbortOd::new();
        let mut server = SdoServer::new();
        let mut resp = [0u8; 8];
        let mut events = heapless::Deque::<_, 4>::new();
        let init = [0x21, 0x01, 0x20, 0x00, declared, 0, 0, 0];
        server
            .process(&init, &mut od, &mut resp, &mut events, NmtState::PreOperational, 0)
            .unwrap();
        assert_eq!(resp[0], 0x60, "declared {declared}: initiate accepted");
        server
            .process(&segment, &mut od, &mut resp, &mut events, NmtState::PreOperational, 1)
            .unwrap();
        assert_eq!(resp[0], 0x80, "declared {declared}: {resp:02X?}");
        let code = u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]);
        assert_eq!(code, expected as u32, "declared {declared}");
    }
}
