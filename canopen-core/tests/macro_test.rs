extern crate alloc;

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
        [0x2010] temperature: f64 = 0.0, rw;
        [0x1008] device_name: visible_string<32>, ro;
        [0x2020] firmware_blob: domain<512>, rw;
        [0x2021] serial_data: octet_string<64>, rw;
        [0x1003] pre_defined_error_field: array<u32, 8>, ro;
        [0x2030] writable_array: array<u16, 4>, rw;
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
    assert_eq!(
        od.write(0x6200, 1, &[0x12, 0x34]),
        Err(canopen_core::od::OdError::DataTypeMismatch)
    );

    // Write output2
    od.write(0x6200, 2, &0x1234u16.to_le_bytes()).unwrap();
    assert_eq!(od.output2, 0x1234);
    assert_eq!(
        od.write(0x6200, 2, &[0x78, 0x56, 0x34, 0x12]),
        Err(canopen_core::od::OdError::DataTypeMismatch)
    );

    // Read-only should fail
    assert!(od.write(0x1000, 0, &[0; 4]).is_err());

    // Write/read f64
    let val: f64 = 23.456;
    od.write(0x2010, 0, &val.to_le_bytes()).unwrap();
    assert_eq!(od.temperature, val);
    let mut buf8 = [0u8; 8];
    let len = od.read(0x2010, 0, &mut buf8).unwrap();
    assert_eq!(len, 8);
    assert_eq!(f64::from_le_bytes(buf8), val);
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
    let mut events: heapless::Deque<canopen_core::OdEvent, 16> = heapless::Deque::new();
    server
        .process(
            &req,
            &mut od,
            &mut resp,
            &mut events,
            canopen_core::NmtState::Operational,
            0,
        )
        .unwrap();

    // Check expedited response
    assert_eq!(resp[4..8], 0x191u32.to_le_bytes());
}

#[test]
fn variable_length_visible_string() {
    let mut od = TestOd::new();

    // Initially empty
    let mut buf = [0u8; 64];
    let len = od.read(0x1008, 0, &mut buf).unwrap();
    assert_eq!(len, 0);

    // Write is blocked (ro)
    assert!(od.write(0x1008, 0, b"hello").is_err());

    // Direct field access works
    od.device_name.push_str("TestDevice").unwrap();
    let len = od.read(0x1008, 0, &mut buf).unwrap();
    assert_eq!(len, 10);
    assert_eq!(&buf[..len], b"TestDevice");
}

#[test]
fn variable_length_domain() {
    let mut od = TestOd::new();

    // Initially empty
    let mut buf = [0u8; 128];
    let len = od.read(0x2020, 0, &mut buf).unwrap();
    assert_eq!(len, 0);

    // Write some data
    let data = [0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03];
    od.write(0x2020, 0, &data).unwrap();
    let len = od.read(0x2020, 0, &mut buf).unwrap();
    assert_eq!(len, 7);
    assert_eq!(&buf[..len], &data);

    // Overwrite with different size
    od.write(0x2020, 0, &[0xFF, 0x00]).unwrap();
    let len = od.read(0x2020, 0, &mut buf).unwrap();
    assert_eq!(len, 2);
    assert_eq!(&buf[..len], &[0xFF, 0x00]);
}

#[test]
fn variable_length_octet_string() {
    let mut od = TestOd::new();

    let data = b"binary\x00data";
    od.write(0x2021, 0, data).unwrap();

    let mut buf = [0u8; 64];
    let len = od.read(0x2021, 0, &mut buf).unwrap();
    assert_eq!(&buf[..len], data);
}

#[test]
fn variable_length_capacity_overflow() {
    let mut od = TestOd::new();

    // firmware_blob has capacity 512 — writing 513 bytes should fail
    let too_big = [0u8; 513];
    let result = od.write(0x2020, 0, &too_big);
    assert_eq!(result, Err(canopen_core::od::OdError::ValueTooLong));
}

#[test]
fn visible_string_metadata() {
    let od = TestOd::new();
    let meta = od.lookup(0x1008, 0).unwrap();
    assert_eq!(
        meta.data_type,
        canopen_core::datatypes::DataType::VisibleString
    );
    assert_eq!(meta.max_size, Some(32));
}

#[test]
fn array_sub0_returns_count() {
    let od = TestOd::new();
    let mut buf = [0u8; 4];

    // sub0 = 8 for error_field
    let len = od.read(0x1003, 0, &mut buf).unwrap();
    assert_eq!(len, 1);
    assert_eq!(buf[0], 8);

    // sub0 = 4 for writable_array
    let len = od.read(0x2030, 0, &mut buf).unwrap();
    assert_eq!(len, 1);
    assert_eq!(buf[0], 4);
}

#[test]
fn array_read_write_elements() {
    let mut od = TestOd::new();
    let mut buf = [0u8; 4];

    // All elements start at 0
    let len = od.read(0x2030, 1, &mut buf).unwrap();
    assert_eq!(len, 2);
    assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 0);

    // Write to sub2
    od.write(0x2030, 2, &42u16.to_le_bytes()).unwrap();
    assert_eq!(od.writable_array[1], 42);

    // Read back
    let len = od.read(0x2030, 2, &mut buf).unwrap();
    assert_eq!(len, 2);
    assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 42);
}

#[test]
fn array_out_of_range() {
    let od = TestOd::new();
    let mut buf = [0u8; 4];

    // sub9 is out of range for 8-element array
    assert_eq!(
        od.read(0x1003, 9, &mut buf),
        Err(canopen_core::od::OdError::NotFound)
    );
}

#[test]
fn array_readonly_rejects_write() {
    let mut od = TestOd::new();

    // error_field is ro
    assert_eq!(
        od.write(0x1003, 1, &0u32.to_le_bytes()),
        Err(canopen_core::od::OdError::ReadOnly)
    );

    // sub0 is always ro
    assert_eq!(
        od.write(0x2030, 0, &[1]),
        Err(canopen_core::od::OdError::ReadOnly)
    );
}

#[test]
fn array_sub_count() {
    let od = TestOd::new();
    assert_eq!(od.sub_count(0x1003), Some(8));
    assert_eq!(od.sub_count(0x2030), Some(4));
}

#[test]
fn array_metadata() {
    let od = TestOd::new();

    // sub0 metadata
    let meta = od.lookup(0x1003, 0).unwrap();
    assert_eq!(meta.data_type, canopen_core::datatypes::DataType::U8);
    assert_eq!(meta.access, canopen_core::od::AccessType::Ro);

    // element metadata
    let meta = od.lookup(0x1003, 3).unwrap();
    assert_eq!(meta.data_type, canopen_core::datatypes::DataType::U32);
    assert_eq!(meta.access, canopen_core::od::AccessType::Ro);
}

// ---- OD with PDO definitions ----

object_dictionary! {
    pub struct PdoTestOd {
        [0x1000] device_type: u32 = 0x0000_0191, ro;
        [0x6000] inputs: record {
            [1] button: u8 = 0, ro, pdo;
            [2] echo_in: u16 = 0, rw, pdo;
        };
        [0x6200] outputs: record {
            [1] led: u8 = 0, rw, pdo;
            [2] echo_out: u16 = 0, ro, pdo;
        };

        tpdo[1](transmission_type = 255, inhibit_time = 500, event_timer = 1000) {
            button,
            echo_out,
        };
        rpdo[1](transmission_type = 255) {
            led,
            echo_in,
        };
    }
}

#[test]
fn pdo_od_has_tpdo_comm_params() {
    let od = PdoTestOd::new();
    let mut buf = [0u8; 4];

    // 0x1800:0 — highest subindex = 5
    let len = od.read(0x1800, 0, &mut buf).unwrap();
    assert_eq!(len, 1);
    assert_eq!(buf[0], 5);

    // 0x1800:1 — COB-ID (default = 0, resolved at runtime)
    let len = od.read(0x1800, 1, &mut buf).unwrap();
    assert_eq!(len, 4);
    assert_eq!(u32::from_le_bytes(buf), 0); // 0 = predefined default

    // 0x1800:2 — transmission type = 255
    let len = od.read(0x1800, 2, &mut buf).unwrap();
    assert_eq!(len, 1);
    assert_eq!(buf[0], 255);

    // 0x1800:3 — inhibit time = 500
    let len = od.read(0x1800, 3, &mut buf).unwrap();
    assert_eq!(len, 2);
    assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 500);

    // 0x1800:5 — event timer = 1000
    let len = od.read(0x1800, 5, &mut buf).unwrap();
    assert_eq!(len, 2);
    assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 1000);
}

#[test]
fn pdo_od_has_tpdo_mapping_params() {
    let od = PdoTestOd::new();
    let mut buf = [0u8; 4];

    // 0x1A00:0 — mapping count = 2
    let len = od.read(0x1A00, 0, &mut buf).unwrap();
    assert_eq!(len, 1);
    assert_eq!(buf[0], 2);

    // 0x1A00:1 — button = 0x6000_0108 (index 0x6000, sub 1, 8 bits)
    let len = od.read(0x1A00, 1, &mut buf).unwrap();
    assert_eq!(len, 4);
    assert_eq!(u32::from_le_bytes(buf), 0x6000_0108);

    // 0x1A00:2 — echo_out = 0x6200_0210 (index 0x6200, sub 2, 16 bits)
    let len = od.read(0x1A00, 2, &mut buf).unwrap();
    assert_eq!(len, 4);
    assert_eq!(u32::from_le_bytes(buf), 0x6200_0210);
}

#[test]
fn pdo_od_has_rpdo_params() {
    let od = PdoTestOd::new();
    let mut buf = [0u8; 4];

    // RPDO comm: 0x1400:0 = 2, 0x1400:2 = 255
    assert_eq!(od.read(0x1400, 0, &mut buf).unwrap(), 1);
    assert_eq!(buf[0], 2);
    assert_eq!(od.read(0x1400, 2, &mut buf).unwrap(), 1);
    assert_eq!(buf[0], 255);

    // RPDO mapping: 0x1600:0 = 2
    assert_eq!(od.read(0x1600, 0, &mut buf).unwrap(), 1);
    assert_eq!(buf[0], 2);

    // 0x1600:1 — led = 0x6200_0108
    assert_eq!(od.read(0x1600, 1, &mut buf).unwrap(), 4);
    assert_eq!(u32::from_le_bytes(buf), 0x6200_0108);

    // 0x1600:2 — echo_in = 0x6000_0210
    assert_eq!(od.read(0x1600, 2, &mut buf).unwrap(), 4);
    assert_eq!(u32::from_le_bytes(buf), 0x6000_0210);
}

#[test]
fn pdo_od_mapping_lock_protocol() {
    let mut od = PdoTestOd::new();

    // Mapping entries are locked (count != 0), writes should fail
    assert!(od.write(0x1A00, 1, &0u32.to_le_bytes()).is_err());

    // Unlock: set mapping count to 0
    od.write(0x1A00, 0, &[0]).unwrap();

    // Now mapping entries should be writable
    od.write(0x1A00, 1, &0x6000_0210u32.to_le_bytes()).unwrap();

    // Verify it changed
    let mut buf = [0u8; 4];
    od.read(0x1A00, 1, &mut buf).unwrap();
    assert_eq!(u32::from_le_bytes(buf), 0x6000_0210);

    // Re-lock with new count
    od.write(0x1A00, 0, &[1]).unwrap();
    assert_eq!(od.tpdo_mapping_count[0], 1);

    // Locked again
    assert!(od.write(0x1A00, 1, &0u32.to_le_bytes()).is_err());
}

#[test]
fn pdo_od_comm_params_writable() {
    let mut od = PdoTestOd::new();

    // Write COB-ID
    od.write(0x1800, 1, &0x181u32.to_le_bytes()).unwrap();
    assert_eq!(od.tpdo_cob_id[0], 0x181);

    // Write transmission type
    od.write(0x1800, 2, &[1]).unwrap();
    assert_eq!(od.tpdo_transmission_type[0], 1);

    // sub 0 (highest_subindex) is read-only
    assert!(od.write(0x1800, 0, &[0]).is_err());
}

#[test]
fn pdo_od_tpdo_configs_helper() {
    let od = PdoTestOd::new();
    let node_id = canopen_core::cobid::NodeId::new(5).unwrap();
    let configs = od.tpdo_configs(node_id);

    assert_eq!(configs.len(), 1);
    assert_eq!(configs[0].cob_id, 0x180 + 5); // predefined default for TPDO1
    assert_eq!(configs[0].transmission_type, 255);
    assert_eq!(configs[0].inhibit_time_100us, 500);
    assert_eq!(configs[0].event_timer_ms, 1000);
    assert_eq!(configs[0].mappings.len(), 2);
    assert!(configs[0].enabled);
}

#[test]
fn pdo_od_rpdo_configs_helper() {
    let od = PdoTestOd::new();
    let node_id = canopen_core::cobid::NodeId::new(3).unwrap();
    let configs = od.rpdo_configs(node_id);

    assert_eq!(configs.len(), 1);
    assert_eq!(configs[0].cob_id, 0x200 + 3); // predefined default for RPDO1
    assert_eq!(configs[0].transmission_type, 255);
    assert_eq!(configs[0].mappings.len(), 2);
    assert!(configs[0].enabled);
}

#[test]
fn pdo_od_eds_includes_pdo_registers() {
    let eds = PdoTestOd::EDS;

    assert!(eds.contains("[1800]"));
    assert!(eds.contains("[1800sub5]"));
    assert!(eds.contains("DefaultValue=0x3E8"));
    assert!(eds.contains("[1A00]"));
    assert!(eds.contains("[1A00sub1]"));
    assert!(eds.contains("DefaultValue=0x60000108"));
    assert!(eds.contains("[1400]"));
    assert!(eds.contains("[1600]"));
    assert!(eds.contains("DefaultValue=0x62000108"));
    assert!(eds.contains("DefaultValue=$NODEID+0x180"));
    assert!(eds.contains("DefaultValue=$NODEID+0x200"));
}

// ---- Store EDS on-device (0x1021/0x1022) ----

#[test]
fn store_eds_compressed() {
    let od = TestOd::new();
    let compressed = TestOd::EDS_COMPRESSED;
    let mut buf = vec![0u8; compressed.len() + 64];
    let len = od.read(0x1021, 0, &mut buf).unwrap();
    assert_eq!(&buf[..len], compressed);

    // Verify it decompresses to the original EDS
    let decompressed = miniz_oxide::inflate::decompress_to_vec(&buf[..len]).unwrap();
    assert_eq!(decompressed, TestOd::EDS.as_bytes());
}

#[test]
fn store_eds_smaller_than_original() {
    let original = TestOd::EDS.as_bytes().len();
    let compressed = TestOd::EDS_COMPRESSED.len();
    assert!(
        compressed < original,
        "compressed {} >= original {}",
        compressed,
        original
    );
}

#[test]
fn store_format_is_compressed() {
    let od = TestOd::new();
    let mut buf = [0u8; 1];
    let len = od.read(0x1022, 0, &mut buf).unwrap();
    assert_eq!(len, 1);
    assert_eq!(buf[0], 1); // 1 = deflate compressed
}

#[test]
fn store_eds_is_readonly() {
    let mut od = TestOd::new();
    assert_eq!(
        od.write(0x1021, 0, &[0]),
        Err(canopen_core::od::OdError::ReadOnly)
    );
    assert_eq!(
        od.write(0x1022, 0, &[1]),
        Err(canopen_core::od::OdError::ReadOnly)
    );
}

#[test]
fn store_eds_metadata() {
    let od = TestOd::new();
    let meta = od.lookup(0x1021, 0).unwrap();
    assert_eq!(meta.data_type, canopen_core::datatypes::DataType::Domain);
    assert!(meta.max_size.is_some());

    let meta = od.lookup(0x1022, 0).unwrap();
    assert_eq!(meta.data_type, canopen_core::datatypes::DataType::U8);
}

// ---- EDS export ----

object_dictionary! {
    #[export_eds(path = "../target/test_export.eds")]
    pub struct EdsExportOd {
        [0x1000] device_type: u32 = 0x0000_0191, ro;
        [0x1001] error_register: u8 = 0x00, ro;
        [0x1018] identity: record {
            [1] vendor_id: u32 = 0x1234, ro;
            [2] product_code: u32 = 0x01, ro;
        };
        [0x1003] error_field: array<u32, 4>, ro;
        [0x1008] device_name: visible_string<32>, ro;
        [0x2000] blob: domain<256>, rw;
    }
}

#[test]
fn eds_export_generates_valid_file() {
    // The macro writes the file at compile time; verify it exists and has expected content
    let eds_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../target/test_export.eds");
    let content = std::fs::read_to_string(eds_path).expect("EDS file should exist");

    // Check key sections
    assert!(content.contains("[FileInfo]"));
    assert!(content.contains("[1000]"));
    assert!(content.contains("ObjectType=0x7")); // VAR
    assert!(content.contains("ObjectType=0x9")); // RECORD
    assert!(content.contains("ObjectType=0x8")); // ARRAY
    assert!(content.contains("DataType=0x0009")); // VisibleString
    assert!(content.contains("DataType=0x000F")); // Domain
}

#[test]
fn eds_const_available_at_runtime() {
    let eds = EdsExportOd::EDS;
    assert!(eds.contains("[FileInfo]"));
    assert!(eds.contains("[1000]"));
    assert!(eds.contains("DataType=0x0007")); // U32
}

// ---- use_alloc mode ----

object_dictionary! {
    #[use_alloc]
    pub struct AllocOd {
        [0x1000] device_type: u32 = 0x191, ro;
        [0x1008] device_name: visible_string, ro;
        [0x2020] firmware: domain, rw;
        [0x2021] serial: octet_string, rw;
    }
}

#[test]
fn alloc_od_variable_length_types() {
    let mut od = AllocOd::new();

    // device_name is a String, starts empty
    let mut buf = [0u8; 128];
    let len = od.read(0x1008, 0, &mut buf).unwrap();
    assert_eq!(len, 0);

    // Set via field access (it's a standard String)
    od.device_name = String::from("AllocDevice");
    let len = od.read(0x1008, 0, &mut buf).unwrap();
    assert_eq!(&buf[..len], b"AllocDevice");

    // Write domain via OD (no capacity limit)
    let big_data = vec![0xABu8; 2048];
    od.write(0x2020, 0, &big_data).unwrap();
    let mut big_buf = vec![0u8; 4096];
    let len = od.read(0x2020, 0, &mut big_buf).unwrap();
    assert_eq!(len, 2048);
    assert_eq!(&big_buf[..len], &big_data[..]);

    // Write octet_string
    od.write(0x2021, 0, b"binary\x00data").unwrap();
    let len = od.read(0x2021, 0, &mut buf).unwrap();
    assert_eq!(&buf[..len], b"binary\x00data");
}

#[test]
fn alloc_od_no_capacity_needed() {
    let od = AllocOd::new();
    // Metadata reports no max_size for alloc types
    let meta = od.lookup(0x1008, 0).unwrap();
    assert_eq!(
        meta.data_type,
        canopen_core::datatypes::DataType::VisibleString
    );
    assert_eq!(meta.max_size, None);
}

// ---- validate_write macro support ----

object_dictionary! {
    #[validate_write(check_value)]
    pub struct ValidatedOd {
        [0x1000] device_type: u32 = 0x0000_0191, ro;
        [0x2000] level: u16 = 0, rw;
    }
}

impl ValidatedOd {
    /// Application validation: level must be <= 1000.
    fn check_value(
        &self,
        index: u16,
        subindex: u8,
        data: &[u8],
    ) -> Result<(), canopen_core::od::OdError> {
        if index == 0x2000 && subindex == 0 && data.len() >= 2 {
            let val = u16::from_le_bytes([data[0], data[1]]);
            if val > 1000 {
                return Err(canopen_core::od::OdError::ValueRange);
            }
        }
        Ok(())
    }
}

#[test]
fn validate_write_macro_accepts_valid() {
    let mut od = ValidatedOd::new();
    // Valid value: accepted by validate_write, then written
    od.write(0x2000, 0, &500u16.to_le_bytes()).unwrap();
    assert_eq!(od.level, 500);
}

#[test]
fn validate_write_macro_rejects_invalid() {
    let mut od = ValidatedOd::new();
    od.level = 42;
    // Invalid value: rejected by validate_write before write
    let result = od.validate_write(0x2000, 0, &2000u16.to_le_bytes());
    assert_eq!(result, Err(canopen_core::od::OdError::ValueRange));
    // Value unchanged
    assert_eq!(od.level, 42);
}

#[test]
fn validate_write_macro_sdo_server_rejects() {
    use canopen_core::nmt::NmtState;
    use canopen_core::od::OdEvent;
    use canopen_core::sdo::SdoServer;

    let mut od = ValidatedOd::new();
    od.level = 42;
    let mut server = SdoServer::new();
    let mut resp = [0u8; 8];
    let mut events: heapless::Deque<OdEvent, 16> = heapless::Deque::new();

    // SDO expedited download: write 2000 (> 1000 limit) to 0x2000:0
    let req = [
        0x2B, // CCS=1, n=2, e=1, s=1 (expedited, 2 bytes)
        0x00, 0x20, // index 0x2000
        0x00, // subindex 0
        0xD0, 0x07, // value 2000 in LE
        0x00, 0x00,
    ];
    server
        .process(
            &req,
            &mut od,
            &mut resp,
            &mut events,
            NmtState::Operational,
            0,
        )
        .unwrap();

    // Should have gotten an abort response (SCS=4 = 0x80)
    assert_eq!(
        resp[0] & 0xE0,
        0x80,
        "expected abort response, got 0x{:02X}",
        resp[0]
    );
    // Abort code should be ValueRangeExceeded = 0x0609_0030
    let abort_code = u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]);
    assert_eq!(
        abort_code, 0x0609_0030,
        "expected ValueRangeExceeded abort code"
    );
    // Value should NOT have been written
    assert_eq!(od.level, 42);
    // No event should have been pushed
    assert!(events.is_empty());
}

// ---- New DSL surface: keywords, unit suffixes, consts, change enum ----

object_dictionary! {
    pub struct SugarOd {
        [0x6000] inputs: record {
            [1] button: u8 = 0, ro, pdo;
        };
        [0x6200] outputs: record {
            [1] led: u8 = 0, rw, pdo;
        };
        [0x2000] echo: record {
            [1] echo_in: u16 = 0, rw, pdo;
            [2] echo_out: u16 = 0, ro, pdo;
        };
        [0x2020] firmware: domain<64>, rw;
        [0x2030] thresholds: array<u16, 4>, rw;

        // 50ms in 100µs units = 500; 1s in ms = 1000
        tpdo[1](transmission_type = event_driven, inhibit_time = 50ms, event_timer = 1s) {
            button,
            echo_out,
        };
        // float with unit suffix: 0.1s = 100ms → event_timer = 100
        tpdo[2](transmission_type = sync_cyclic(4), event_timer = 0.1s) {
            echo_out,
        };
        rpdo[1](transmission_type = event_driven) {
            led,
            echo_in,
        };
    }
}

#[test]
fn dsl_transmission_type_keywords_and_time_suffixes() {
    let od = SugarOd::new();
    let node_id = canopen_core::cobid::NodeId::new(1).unwrap();
    let tpdo = od.tpdo_configs(node_id);

    assert_eq!(tpdo[0].transmission_type, 255); // event_driven
    assert_eq!(tpdo[0].inhibit_time_100us, 500); // 50ms
    assert_eq!(tpdo[0].event_timer_ms, 1000); // 1s

    assert_eq!(tpdo[1].transmission_type, 4); // sync_cyclic(4)
    assert_eq!(tpdo[1].event_timer_ms, 100); // 0.1s

    let rpdo = od.rpdo_configs(node_id);
    assert_eq!(rpdo[0].transmission_type, 255);
}

#[test]
fn generated_address_consts() {
    assert_eq!(SugarOd::BUTTON, (0x6000, 1));
    assert_eq!(SugarOd::LED, (0x6200, 1));
    assert_eq!(SugarOd::ECHO_IN, (0x2000, 1));
    assert_eq!(SugarOd::FIRMWARE, (0x2020, 0));
    assert_eq!(SugarOd::THRESHOLDS_INDEX, 0x2030);

    // Consts are structural-match, so they work as match patterns.
    let evt = (0x6200u16, 1u8);
    match evt {
        SugarOd::LED => {}
        _ => panic!("const pattern did not match"),
    }
}

#[test]
fn change_enum_decodes_events() {
    use canopen_core::od::{OdChanges, OdEvent, OdEventSource};

    let mut od = SugarOd::new();
    od.led = 1;
    od.echo_in = 0xBEEF;
    od.thresholds[2] = 77;

    let evt = |index, subindex| OdEvent {
        index,
        subindex,
        source: OdEventSource::Sdo,
    };

    assert_eq!(od.decode_event(evt(0x6200, 1)), Some(SugarOdChange::Led(1)));
    assert_eq!(
        od.decode_event(evt(0x2000, 1)),
        Some(SugarOdChange::EchoIn(0xBEEF))
    );
    // Variable-length entries: unit variant, value read from the OD if needed.
    assert_eq!(
        od.decode_event(evt(0x2020, 0)),
        Some(SugarOdChange::Firmware)
    );
    // Arrays carry (subindex, value).
    assert_eq!(
        od.decode_event(evt(0x2030, 3)),
        Some(SugarOdChange::Thresholds(3, 77))
    );

    // Read-only entries can't be changed by the stack: no variant.
    assert_eq!(od.decode_event(evt(0x6000, 1)), None);
    // Auto-generated PDO comm params have no application-level variant.
    assert_eq!(od.decode_event(evt(0x1800, 2)), None);
}

#[test]
fn node_next_change_drains_typed() {
    use canopen_core::node::{Node, NodeConfig};

    let node_id = canopen_core::cobid::NodeId::new(1).unwrap();
    let od = SugarOd::new();
    let config = NodeConfig {
        heartbeat_interval_ms: 500,
        auto_start: true,
        ..NodeConfig::from_od(&od, node_id)
    };
    assert_eq!(config.tpdo.len(), 2);
    assert_eq!(config.rpdo.len(), 1);

    // The generated alias fixes the PDO counts.
    let mut node: SugarOdNode = Node::new(config, od);

    // Simulate an SDO write to `led` via the OD + event path: write through
    // the SDO server would enqueue an event; here we check the typed drain on
    // an empty queue plus the COB-ID accessors.
    assert_eq!(node.next_change(), None);
    assert_eq!(node.tpdo_cob_id(0), Some(0x181));
    assert_eq!(node.rpdo_cob_id(0), Some(0x201));
}

#[test]
fn transmission_type_enum_round_trip() {
    use canopen_core::TransmissionType;

    assert_eq!(TransmissionType::EventDriven.raw(), 255);
    assert_eq!(TransmissionType::SyncAcyclic.raw(), 0);
    assert_eq!(TransmissionType::SyncCyclic(4).raw(), 4);
    assert_eq!(u8::from(TransmissionType::RtrSync), 252);

    assert_eq!(
        TransmissionType::from_raw(255),
        Some(TransmissionType::EventDriven)
    );
    assert_eq!(
        TransmissionType::from_raw(7),
        Some(TransmissionType::SyncCyclic(7))
    );
    assert_eq!(TransmissionType::from_raw(245), None); // reserved
}
