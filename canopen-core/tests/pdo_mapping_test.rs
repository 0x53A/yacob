//! PDO mapping widths: partial mappings carry the low bits of an object.

use canopen_core::pdo::engine::{PdoMapping, RpdoConfig, RpdoEngine, TpdoConfig, TpdoEngine};
use canopen_core::transport::CanFrame;
use canopen_derive::object_dictionary;

object_dictionary! {
    pub struct PartialOd {
        [0x1000] device_type: u32 = 0x191, ro;
        [0x6000] wide: u32 = 0, rw, pdo;
        [0x6001] half: u16 = 0, rw, pdo;
        [0x6002] signed: i16 = 0, rw, pdo;
        [0x6003] nibble: i8 = 0, rw, pdo;
    }
}

fn mapping(index: u16, bits: u8) -> PdoMapping {
    PdoMapping {
        index,
        subindex: 0,
        bit_length: bits,
    }
}

#[test]
fn rpdo_partial_mappings_widen_into_the_object() {
    let mut od = PartialOd::new();
    od.wide = 0xFFFF_FFFF;
    let mut config = RpdoConfig::<64>::default();
    config.od_number = 1;
    config.cob_id = 0x201;
    config.transmission_type = 255;
    config.enabled = true;
    for m in [
        mapping(0x6000, 16),
        mapping(0x6001, 8),
        mapping(0x6002, 12),
        mapping(0x6003, 4),
    ] {
        config.mappings.push(m).unwrap();
    }
    let mut engine = RpdoEngine::<1>::new([config]);
    let mut events = heapless::Deque::<_, 8>::new();
    // 16 bits 0x1234 | 8 bits 0x56 | 12 bits 0xFFE (-2) | 4 bits 0x9 (-7)
    // (40 bits = 5 bytes).
    let frame = CanFrame::new(0x201, &[0x34, 0x12, 0x56, 0xFE, 0x9F]).unwrap();
    assert!(engine.process(&frame, &mut od, &mut events, 0));
    assert_eq!(od.wide, 0x0000_1234, "upper bits are replaced, not kept");
    assert_eq!(od.half, 0x56);
    assert_eq!(od.signed, -2);
    assert_eq!(od.nibble, -7);
    assert_eq!(events.len(), 4);
}

#[test]
fn tpdo_partial_mapping_transmits_low_bits() {
    let mut od = PartialOd::new();
    od.wide = 0xAABB_1234;
    od.signed = -2;
    let mut config = TpdoConfig::<64>::default();
    config.od_number = 1;
    config.cob_id = 0x181;
    config.transmission_type = 255;
    config.event_timer_ms = 1;
    config.enabled = true;
    config.mappings.push(mapping(0x6000, 16)).unwrap();
    config.mappings.push(mapping(0x6002, 12)).unwrap();
    let mut engine = TpdoEngine::<1>::new([config]);
    let dirty = heapless::Vec::<(u16, u8), 1>::new();
    let mut out = heapless::Vec::<CanFrame, 1>::new();
    engine.poll(&od, 10_000, &dirty, &mut out);
    assert_eq!(out.len(), 1, "partial TPDO mapping must be built");
    assert_eq!(out[0].data(), &[0x34, 0x12, 0xFE, 0x0F]);
}
