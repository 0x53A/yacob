use canopen_core::od::ObjectDictionary;
use canopen_derive::object_dictionary;

object_dictionary! {
    pub struct TestOd {
        [0x1000] device_type: u32 = 0x0000_0191, ro;
        [0x1001] error_register: u8 = 0x00, ro;
        [0x1018] identity: record {
            [1] vendor_id: u32 = 0x0000_1234, ro;
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

#[test]
fn macro_od_read() {
    let od = TestOd::new();

    // Read device_type
    let mut buf = [0u8; 4];
    let len = od.read(0x1000, 0, &mut buf).unwrap();
    assert_eq!(len, 4);
    assert_eq!(u32::from_le_bytes(buf), 0x0000_0191);

    // Read vendor_id
    let len = od.read(0x1018, 1, &mut buf).unwrap();
    assert_eq!(len, 4);
    assert_eq!(u32::from_le_bytes(buf), 0x1234);
}

#[test]
fn macro_od_write() {
    let mut od = TestOd::new();

    // Write output1
    od.write(0x6200, 1, &[0xFF]).unwrap();
    assert_eq!(od.output1, 0xFF);

    // Write output2
    od.write(0x6200, 2, &0x1234u16.to_le_bytes()).unwrap();
    assert_eq!(od.output2, 0x1234);

    // Read-only should fail
    assert!(od.write(0x1000, 0, &[0; 4]).is_err());
}

#[test]
fn macro_od_lookup() {
    let od = TestOd::new();

    let meta = od.lookup(0x1000, 0).unwrap();
    assert_eq!(meta.name, "device_type");
    assert_eq!(meta.data_type, canopen_core::datatypes::DataType::U32);

    let meta = od.lookup(0x6000, 1).unwrap();
    assert!(meta.pdo_mappable);

    assert!(od.lookup(0xFFFF, 0).is_none());
}

#[test]
fn macro_od_sub_count() {
    let od = TestOd::new();
    assert_eq!(od.sub_count(0x1018), Some(4)); // identity has subs 1-4
    assert_eq!(od.sub_count(0x6000), Some(2)); // inputs has subs 1-2
}

#[test]
fn macro_od_with_sdo_server() {
    use canopen_core::sdo::SdoServer;

    let mut od = TestOd::new();
    let mut server = SdoServer::new();
    let mut resp = [0u8; 8];

    // SDO upload request for 0x1000:0
    let req = [0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0];
    server.process(&req, &mut od, &mut resp).unwrap();

    // Check expedited response
    assert_eq!(resp[4..8], 0x191u32.to_le_bytes());
}
