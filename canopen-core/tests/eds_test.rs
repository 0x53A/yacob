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

    // Read sub0 (number of entries)
    let mut buf = [0u8; 1];
    let len = od.read(0x2000, 0, &mut buf).unwrap();
    assert_eq!(len, 1);
    assert_eq!(buf[0], 2);

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
    assert_eq!(meta.name, "sensor_value");

    let meta = od.lookup(0x1000, 0).unwrap();
    assert!(!meta.pdo_mappable);
}
