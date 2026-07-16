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

object_dictionary_from_eds! {
    pub struct BoolMappedAsU8Od = "tests/test_bool_mapped_as_u8.eds";
}

object_dictionary_from_eds! {
    pub struct DummyMappedOd = "tests/test_dummy_mapping.eds";
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
fn eds_import_preserves_declared_pdo_mapping_length() {
    let od = BoolMappedAsU8Od::new();
    let mut buf = [0u8; 4];

    let len = od.read(0x1A00, 1, &mut buf).unwrap();
    assert_eq!(len, 4);
    assert_eq!(u32::from_le_bytes(buf), 0x2000_0008);

    let tpdo = od.tpdo_configs(canopen_core::cobid::NodeId::new(1).unwrap());
    let config = &tpdo[0];
    assert_eq!(config.mappings[0].index, 0x2000);
    assert_eq!(config.mappings[0].subindex, 0);
    assert_eq!(config.mappings[0].bit_length, 8);
}

#[test]
fn eds_import_preserves_dummy_pdo_mapping() {
    let od = DummyMappedOd::new();
    let mut buf = [0u8; 4];

    assert_eq!(od.read(0x1A00, 1, &mut buf), Ok(4));
    assert_eq!(u32::from_le_bytes(buf), 0x2000_0001);
    assert_eq!(od.read(0x1A00, 2, &mut buf), Ok(4));
    assert_eq!(u32::from_le_bytes(buf), 0x0001_0007);
    assert_eq!(od.read(0x1A00, 3, &mut buf), Ok(4));
    assert_eq!(u32::from_le_bytes(buf), 0x2001_0008);

    let tpdo = od.tpdo_configs(canopen_core::cobid::NodeId::new(1).unwrap());
    let config = &tpdo[0];
    assert_eq!(config.mappings.len(), 3);
    assert_eq!(config.mappings[1].index, 0x0001);
    assert_eq!(config.mappings[1].subindex, 0);
    assert_eq!(config.mappings[1].bit_length, 7);
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

    // RPDO1 comm sub 5: event timer (deadline monitoring) = 300 ms
    let len = od.read(0x1400, 5, &mut buf).unwrap();
    assert_eq!(len, 2);
    assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 0x12C);

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
    assert_eq!(tpdo_cfgs.len(), 2);
    assert_eq!(tpdo_cfgs[0].od_number, 1);
    assert_eq!(tpdo_cfgs[0].cob_id, 0x180 + 5); // predefined default
    assert_eq!(tpdo_cfgs[0].transmission_type, 255);
    assert_eq!(tpdo_cfgs[0].inhibit_time_100us, 0x1F4);
    assert_eq!(tpdo_cfgs[0].event_timer_ms, 0x64);
    assert_eq!(tpdo_cfgs[0].mappings.len(), 1);

    // TPDO7: beyond the pre-defined connection set; the EDS declares
    // $NODEID+0x1C2, which resolves per node (0x1C2 + 5 here)
    assert_eq!(tpdo_cfgs[1].od_number, 7);
    assert_eq!(tpdo_cfgs[1].cob_id, 0x1C7);
    assert_eq!(tpdo_cfgs[1].mappings.len(), 1);

    let rpdo_cfgs = od.rpdo_configs(node_id);
    assert_eq!(rpdo_cfgs.len(), 2);
    assert_eq!(rpdo_cfgs[0].od_number, 1);
    assert_eq!(rpdo_cfgs[0].cob_id, 0x200 + 5);
    assert_eq!(rpdo_cfgs[0].transmission_type, 255);
    assert_eq!(rpdo_cfgs[0].deadline_ms, 0x12C); // EDS sub 5 event timer
    assert_eq!(rpdo_cfgs[0].mappings.len(), 1);

    // RPDO2 is comm-only in the EDS (no mapping section): its comm params
    // must survive import so it can be remapped at runtime.
    assert_eq!(rpdo_cfgs[1].od_number, 2);
    assert_eq!(rpdo_cfgs[1].cob_id, 0x300 + 5);
    assert_eq!(rpdo_cfgs[1].deadline_ms, 0); // no sub 5 in the EDS
    assert_eq!(rpdo_cfgs[1].mappings.len(), 0);
}

#[test]
fn eds_import_mapping_mutability() {
    let mut od = PdoDeviceOd::new();

    // RPDO1/TPDO1 mapping records are AccessType=ro in the EDS -> immutable:
    // neither the unlock (sub 0) nor the entries accept writes.
    assert!(od.write(0x1600, 0, &[0]).is_err());
    assert!(od.write(0x1600, 1, &0x6040_0010u32.to_le_bytes()).is_err());
    assert!(od.write(0x1A00, 0, &[0]).is_err());

    // TPDO7's mapping record is AccessType=rw -> CiA 301 dynamic mapping,
    // remappable via the unlock protocol.
    od.write(0x1A06, 0, &[0]).unwrap();
    od.write(0x1A06, 1, &0x6041_0010u32.to_le_bytes()).unwrap();
    od.write(0x1A06, 0, &[1]).unwrap();

    // RPDO2 is comm-only (no mapping section in the EDS): it defaults to
    // mutable, otherwise it could never be mapped at runtime.
    od.write(0x1601, 1, &0x6040_0010u32.to_le_bytes()).unwrap();
    od.write(0x1601, 0, &[1]).unwrap();
}
