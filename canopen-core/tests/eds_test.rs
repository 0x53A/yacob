use canopen_core::od::ObjectDictionary;
use canopen_derive::object_dictionary_from_eds;

object_dictionary_from_eds! {
    pub struct TestDeviceOd = "tests/test_device.eds";
}

#[test]
fn eds_od_read_device_type() {
    let od = TestDeviceOd::new();
    let mut buf = [0u8; 4];
    let len = od.read(0x1000, 0, &mut buf).unwrap();
    assert_eq!(len, 4);
    assert_eq!(u32::from_le_bytes(buf), 0xC8);
}

#[test]
fn eds_od_read_error_register() {
    let od = TestDeviceOd::new();
    let mut buf = [0u8; 1];
    let len = od.read(0x1001, 0, &mut buf).unwrap();
    assert_eq!(len, 1);
    assert_eq!(buf[0], 0);
}

#[test]
fn eds_od_rw_heartbeat_time() {
    let mut od = TestDeviceOd::new();
    // Read default
    let mut buf = [0u8; 2];
    let len = od.read(0x1017, 0, &mut buf).unwrap();
    assert_eq!(len, 2);
    assert_eq!(u16::from_le_bytes(buf), 1000);

    // Write new value
    od.write(0x1017, 0, &500u16.to_le_bytes()).unwrap();
    od.read(0x1017, 0, &mut buf).unwrap();
    assert_eq!(u16::from_le_bytes(buf), 500);
}

#[test]
fn eds_od_record_sub_entries() {
    let mut od = TestDeviceOd::new();

    // sub_count returns highest subindex
    assert_eq!(od.sub_count(0x2000), Some(2));

    // sensor_value is read-only
    let mut buf16 = [0u8; 2];
    od.read(0x2000, 1, &mut buf16).unwrap();
    assert!(od.write(0x2000, 1, &[0x01, 0x00]).is_err());

    // control_word is rw
    od.write(0x2000, 2, &0xABCDu16.to_le_bytes()).unwrap();
    od.read(0x2000, 2, &mut buf16).unwrap();
    assert_eq!(u16::from_le_bytes(buf16), 0xABCD);
}

#[test]
fn eds_od_lookup_pdo_mapping() {
    let od = TestDeviceOd::new();
    let meta = od.lookup(0x2000, 1).unwrap();
    assert!(meta.pdo_mappable);
    assert_eq!(meta.name, "application_data_sensor_value");

    let meta = od.lookup(0x1000, 0).unwrap();
    assert!(!meta.pdo_mappable);
}

// ---- EDS with PDOs, strings, and arrays ----

object_dictionary_from_eds! {
    pub struct PdoDeviceOd = "tests/test_pdo_device.eds"
    with {
        [0x1008] capacity = 32,
    };
}

#[test]
fn eds_pdo_device_string_default() {
    let od = PdoDeviceOd::new();

    // visible_string at 0x1008 should have the default from EDS
    let mut buf = [0u8; 64];
    let len = od.read(0x1008, 0, &mut buf).unwrap();
    assert_eq!(&buf[..len], b"TestPdoDevice");
}

#[test]
fn eds_pdo_device_array() {
    let od = PdoDeviceOd::new();
    let mut buf = [0u8; 4];

    // 0x1003 is an array of u32 with 4 elements
    let len = od.read(0x1003, 0, &mut buf).unwrap();
    assert_eq!(len, 1);
    assert_eq!(buf[0], 4); // sub0 = count

    // Read element
    let len = od.read(0x1003, 1, &mut buf).unwrap();
    assert_eq!(len, 4);
    assert_eq!(u32::from_le_bytes(buf), 0);
}

#[test]
fn eds_pdo_device_tpdo_params() {
    let od = PdoDeviceOd::new();
    let mut buf = [0u8; 4];

    // TPDO1 comm params should exist (generated from EDS PDO sections)
    // 0x1800:0 = highest subindex
    let len = od.read(0x1800, 0, &mut buf).unwrap();
    assert_eq!(len, 1);
    assert_eq!(buf[0], 5);

    // 0x1800:2 = transmission type = 0xFF
    let len = od.read(0x1800, 2, &mut buf).unwrap();
    assert_eq!(len, 1);
    assert_eq!(buf[0], 0xFF);

    // 0x1800:3 = inhibit time = 0x1F4 = 500
    let len = od.read(0x1800, 3, &mut buf).unwrap();
    assert_eq!(len, 2);
    assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 0x1F4);

    // 0x1800:5 = event timer = 0x64 = 100
    let len = od.read(0x1800, 5, &mut buf).unwrap();
    assert_eq!(len, 2);
    assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 0x64);
}

#[test]
fn eds_pdo_device_tpdo_mapping() {
    let od = PdoDeviceOd::new();
    let mut buf = [0u8; 4];

    // TPDO1 mapping count = 1
    let len = od.read(0x1A00, 0, &mut buf).unwrap();
    assert_eq!(len, 1);
    assert_eq!(buf[0], 1);

    // Mapping entry 1 = statusword (0x6041, sub 0, 16 bits)
    let len = od.read(0x1A00, 1, &mut buf).unwrap();
    assert_eq!(len, 4);
    assert_eq!(u32::from_le_bytes(buf), 0x6041_0010);
}

#[test]
fn eds_pdo_device_rpdo_mapping() {
    let od = PdoDeviceOd::new();
    let mut buf = [0u8; 4];

    // RPDO1 comm: transmission type = 0xFF
    let len = od.read(0x1400, 2, &mut buf).unwrap();
    assert_eq!(len, 1);
    assert_eq!(buf[0], 0xFF);

    // RPDO1 mapping count = 1
    let len = od.read(0x1600, 0, &mut buf).unwrap();
    assert_eq!(len, 1);
    assert_eq!(buf[0], 1);

    // Mapping entry 1 = controlword (0x6040, sub 0, 16 bits)
    let len = od.read(0x1600, 1, &mut buf).unwrap();
    assert_eq!(len, 4);
    assert_eq!(u32::from_le_bytes(buf), 0x6040_0010);
}

#[test]
fn eds_pdo_device_configs() {
    let od = PdoDeviceOd::new();
    let node_id = canopen_core::cobid::NodeId::new(5).unwrap();

    let tpdo_cfgs = od.tpdo_configs(node_id);
    assert_eq!(tpdo_cfgs.len(), 1);
    assert_eq!(tpdo_cfgs[0].cob_id, 0x180 + 5); // predefined default
    assert_eq!(tpdo_cfgs[0].transmission_type, 255);
    assert_eq!(tpdo_cfgs[0].inhibit_time_100us, 0x1F4);
    assert_eq!(tpdo_cfgs[0].event_timer_ms, 0x64);
    assert_eq!(tpdo_cfgs[0].mappings.len(), 1);

    let rpdo_cfgs = od.rpdo_configs(node_id);
    assert_eq!(rpdo_cfgs.len(), 1);
    assert_eq!(rpdo_cfgs[0].cob_id, 0x200 + 5);
    assert_eq!(rpdo_cfgs[0].transmission_type, 255);
    assert_eq!(rpdo_cfgs[0].mappings.len(), 1);
}
