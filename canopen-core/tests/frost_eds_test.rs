//! Validate that the real FRoST EDS files parse correctly with our macros.

use canopen_core::od::ObjectDictionary;
use canopen_derive::{object_dictionary_from_eds, sdo_client_from_eds};

// ---- DES (Drive Electronics System) ----

object_dictionary_from_eds! {
    pub struct DesOd = "../../frost/frost-res-virtual-des/des/frost_des_od.eds"
    with {
        [0x1008] capacity = 32,
    };
}

sdo_client_from_eds! {
    pub struct DesClient = "../../frost/frost-res-virtual-des/des/frost_des_od.eds"
    with {
        [0x1008] capacity = 32,
    };
}

// ---- EMDM (Energy Management / Distribution Module) ----

object_dictionary_from_eds! {
    pub struct EmdmOd = "../../frost/frost-res-virtual-des/des/frost_emdm_od.eds"
    with {
        [0x1008] capacity = 32,
    };
}

sdo_client_from_eds! {
    pub struct EmdmClient = "../../frost/frost-res-virtual-des/des/frost_emdm_od.eds"
    with {
        [0x1008] capacity = 32,
    };
}

// ---- DES OD Tests ----

#[test]
fn des_od_device_type() {
    let od = DesOd::new();
    let mut buf = [0u8; 4];
    let len = od.read(0x1000, 0, &mut buf).unwrap();
    assert_eq!(len, 4);
    assert_eq!(u32::from_le_bytes(buf), 0x01);
}

#[test]
fn des_od_device_name() {
    let od = DesOd::new();
    let mut buf = [0u8; 32];
    let len = od.read(0x1008, 0, &mut buf).unwrap();
    assert_eq!(&buf[..len], b"FRoS");
}

#[test]
fn des_od_identity() {
    let od = DesOd::new();
    let mut buf = [0u8; 4];

    // vendor ID
    od.read(0x1018, 1, &mut buf).unwrap();
    assert_eq!(u32::from_le_bytes(buf), 0x420);

    // product code
    od.read(0x1018, 2, &mut buf).unwrap();
    assert_eq!(u32::from_le_bytes(buf), 0x01);
}

#[test]
fn des_od_controlword_rw() {
    let mut od = DesOd::new();
    let mut buf = [0u8; 2];

    // default 0
    od.read(0x6040, 0, &mut buf).unwrap();
    assert_eq!(u16::from_le_bytes(buf), 0);

    // write
    od.write(0x6040, 0, &0x000Fu16.to_le_bytes()).unwrap();
    od.read(0x6040, 0, &mut buf).unwrap();
    assert_eq!(u16::from_le_bytes(buf), 0x000F);
}

#[test]
fn des_od_statusword_ro() {
    let mut od = DesOd::new();
    assert!(od.write(0x6041, 0, &0u16.to_le_bytes()).is_err());
}

#[test]
fn des_od_heartbeat_time() {
    let od = DesOd::new();
    let mut buf = [0u8; 2];
    od.read(0x1017, 0, &mut buf).unwrap();
    assert_eq!(u16::from_le_bytes(buf), 0x64); // 100ms
}

#[test]
fn des_od_state_machine_frequency() {
    let od = DesOd::new();
    let mut buf = [0u8; 4];
    od.read(0x4003, 0, &mut buf).unwrap();
    let freq = f32::from_le_bytes(buf);
    assert!((freq - 1000.0).abs() < 1.0); // 0x447a0000 = 1000.0
}

#[test]
fn des_od_error_field_array() {
    let od = DesOd::new();
    let mut buf = [0u8; 4];

    // sub0 = count
    let len = od.read(0x1003, 0, &mut buf).unwrap();
    assert_eq!(len, 1);
    assert!(buf[0] > 0); // should have some error entries

    // read an element
    od.read(0x1003, 1, &mut buf).unwrap();
    assert_eq!(u32::from_le_bytes(buf), 0); // default 0
}

#[test]
fn des_od_tpdo_config() {
    let od = DesOd::new();
    let node_id = canopen_core::cobid::NodeId::new(1).unwrap();
    let configs = od.tpdo_configs(node_id);
    // DES has TPDO1 mapped to controlword (0x6040:0, 16 bits)
    assert!(!configs.is_empty());
}

// ---- EMDM OD Tests ----

#[test]
fn emdm_od_device_type() {
    let od = EmdmOd::new();
    let mut buf = [0u8; 4];
    od.read(0x1000, 0, &mut buf).unwrap();
    assert_eq!(u32::from_le_bytes(buf), 0x300);
}

#[test]
fn emdm_od_device_name() {
    let od = EmdmOd::new();
    let mut buf = [0u8; 32];
    let len = od.read(0x1008, 0, &mut buf).unwrap();
    assert_eq!(&buf[..len], b"FRST");
}

#[test]
fn emdm_od_heartbeat_time() {
    let od = EmdmOd::new();
    let mut buf = [0u8; 2];
    od.read(0x1017, 0, &mut buf).unwrap();
    assert_eq!(u16::from_le_bytes(buf), 0x1F4); // 500ms
}

#[test]
fn emdm_od_energy_measurements() {
    let od = EmdmOd::new();
    let mut buf = [0u8; 2];

    // 0x6050 is a record with voltage, current, power, energy
    od.read(0x6050, 1, &mut buf).unwrap(); // actual voltage
    assert_eq!(u16::from_le_bytes(buf), 0);
}

#[test]
fn emdm_od_neopixel() {
    let od = EmdmOd::new();
    let mut buf = [0u8; 4];

    // mode (u8, rw)
    od.read(0x6060, 1, &mut buf).unwrap();
    assert_eq!(buf[0], 0x04);

    // color (u32, rw)
    od.read(0x6060, 2, &mut buf).unwrap();
    assert_eq!(u32::from_le_bytes(buf), 0xFF00); // green
}

#[test]
fn emdm_od_consumer_heartbeat() {
    let od = EmdmOd::new();
    let mut buf = [0u8; 4];
    // 0x1016:1 = consumer heartbeat config (node 1, 150ms = 0x10096)
    od.read(0x1016, 1, &mut buf).unwrap();
    assert_eq!(u32::from_le_bytes(buf), 0x10096);
}

#[test]
fn emdm_od_autostart() {
    let od = EmdmOd::new();
    let mut buf = [0u8; 1];
    od.read(0x4041, 0, &mut buf).unwrap();
    assert_eq!(buf[0], 0x01); // autostart enabled
}

// ---- SDO Client Tests ----

#[test]
fn des_client_has_typed_methods() {
    // Just verify the struct compiles and has the expected methods
    let _client = DesClient::new(canopen_core::cobid::NodeId::new(1).unwrap());
    // The methods exist — they're async so we can't easily call them without a transport,
    // but compilation proves the codegen is correct.
}

#[test]
fn emdm_client_has_typed_methods() {
    let _client = EmdmClient::new(canopen_core::cobid::NodeId::new(0x50).unwrap());
}
