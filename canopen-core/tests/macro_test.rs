extern crate alloc;

use canopen_core::od::ObjectDictionary;
use canopen_core::PdoNumber;
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

        tpdo[1](transmission_type = 255, inhibit_time = 500, event_timer = 1000, mapping = mutable) {
            button,
            echo_out,
        };
        // rpdo keeps the default: mapping = immutable
        rpdo[1](transmission_type = 255, deadline = 250) {
            led,
            echo_in,
        };
    }
}

object_dictionary! {
    pub struct BoolPdoTestOd {
        [0x1000] device_type: u32 = 0x0000_0191, ro;
        [0x2000] status: record {
            [1] limit_low: bool = false, ro, pdo;
            [2] limit_high: bool = false, ro, pdo;
            [3] flag3: bool = false, ro, pdo;
            [4] flag4: bool = false, ro, pdo;
            [5] flag5: bool = false, ro, pdo;
            [6] flag6: bool = false, ro, pdo;
            [7] flag7: bool = false, ro, pdo;
            [8] flag8: bool = false, ro, pdo;
            [9] flag9: bool = false, ro, pdo;
        };
        [0x2001] command: record {
            [1] enable: bool = false, rw, pdo;
            [2] reset_fault: bool = false, rw, pdo;
        };

        tpdo[1](transmission_type = event_driven) {
            limit_low,
            limit_high,
            flag3,
            flag4,
            flag5,
            flag6,
            flag7,
            flag8,
            flag9,
        };
        rpdo[1](transmission_type = event_driven) {
            enable,
            reset_fault,
        };
    }
}

#[test]
fn dsl_bool_pdo_mappings_are_one_bit() {
    let od = BoolPdoTestOd::new();
    let node_id = canopen_core::cobid::NodeId::new(1).unwrap();

    let tpdo = od.tpdo_configs(node_id);
    assert_eq!(tpdo[0].mappings[0].index, 0x2000);
    assert_eq!(tpdo[0].mappings[0].subindex, 1);
    assert_eq!(tpdo[0].mappings[0].bit_length, 1);
    assert_eq!(tpdo[0].mappings[1].index, 0x2000);
    assert_eq!(tpdo[0].mappings[1].subindex, 2);
    assert_eq!(tpdo[0].mappings[1].bit_length, 1);
    assert_eq!(tpdo[0].mappings.len(), 9);
    assert_eq!(tpdo[0].mappings[8].index, 0x2000);
    assert_eq!(tpdo[0].mappings[8].subindex, 9);
    assert_eq!(tpdo[0].mappings[8].bit_length, 1);

    let rpdo = od.rpdo_configs(node_id);
    assert_eq!(rpdo[0].mappings[0].index, 0x2001);
    assert_eq!(rpdo[0].mappings[0].subindex, 1);
    assert_eq!(rpdo[0].mappings[0].bit_length, 1);
    assert_eq!(rpdo[0].mappings[1].index, 0x2001);
    assert_eq!(rpdo[0].mappings[1].subindex, 2);
    assert_eq!(rpdo[0].mappings[1].bit_length, 1);

    let mut buf = [0u8; 4];
    assert_eq!(od.read(0x1A00, 1, &mut buf), Ok(4));
    assert_eq!(u32::from_le_bytes(buf), 0x2000_0101);
    assert_eq!(od.read(0x1A00, 2, &mut buf), Ok(4));
    assert_eq!(u32::from_le_bytes(buf), 0x2000_0201);
    assert_eq!(od.read(0x1A00, 9, &mut buf), Ok(4));
    assert_eq!(u32::from_le_bytes(buf), 0x2000_0901);
    assert_eq!(od.read(0x1600, 1, &mut buf), Ok(4));
    assert_eq!(u32::from_le_bytes(buf), 0x2001_0101);
    assert_eq!(od.read(0x1600, 2, &mut buf), Ok(4));
    assert_eq!(u32::from_le_bytes(buf), 0x2001_0201);
}

#[test]
fn dsl_bool_pdo_mappings_pack_end_to_end() {
    use canopen_core::pdo::{RpdoEngine, TpdoEngine};
    use canopen_core::transport::CanFrame;
    use canopen_core::{NodeConfig, OdEvent};
    use heapless::{Deque, Vec};

    let mut od = BoolPdoTestOd::new();
    od.limit_low = true;
    od.limit_high = false;
    od.flag3 = true;
    od.flag4 = false;
    od.flag5 = true;
    od.flag6 = false;
    od.flag7 = true;
    od.flag8 = false;
    od.flag9 = true;

    let node_id = canopen_core::cobid::NodeId::new(1).unwrap();
    let config = NodeConfig::<1, 1>::from_od(&od, node_id);

    let mut tpdo = TpdoEngine::new(config.tpdo);
    let mut dirty = Vec::<(u16, u8), 16>::new();
    dirty.push((0x2000, 1)).unwrap();
    let mut out = Vec::<CanFrame, 1>::new();
    tpdo.poll(&od, 0, &dirty, &mut out);

    assert_eq!(out.len(), 1);
    assert_eq!(out[0].raw_id(), 0x181);
    assert_eq!(out[0].data(), &[0b0101_0101, 0b0000_0001]);

    let mut rpdo = RpdoEngine::new(config.rpdo);
    let mut events = Deque::<OdEvent, 4>::new();
    let frame = CanFrame::new(0x201, &[0b0000_0001]).unwrap();
    assert!(rpdo.process(&frame, &mut od, &mut events, 0));

    assert!(od.enable);
    assert!(!od.reset_fault);
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

    // RPDO comm: 0x1400:0 = 5 (subs 3/4 absent), 0x1400:2 = 255
    assert_eq!(od.read(0x1400, 0, &mut buf).unwrap(), 1);
    assert_eq!(buf[0], 5);
    assert_eq!(od.read(0x1400, 2, &mut buf).unwrap(), 1);
    assert_eq!(buf[0], 255);

    // 0x1400:5 — deadline monitoring (event timer) = 250 ms
    assert_eq!(od.read(0x1400, 5, &mut buf).unwrap(), 2);
    assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 250);

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
fn mutable_pdo_mapping_accepts_more_than_eight_one_bit_entries() {
    let mut od = PdoTestOd::new();

    od.write(0x1A00, 0, &[0]).unwrap();
    for sub in 1u8..=9 {
        let mapping = 0x6000_0101u32;
        od.write(0x1A00, sub, &mapping.to_le_bytes()).unwrap();
    }
    od.write(0x1A00, 0, &[9]).unwrap();

    let node_id = canopen_core::cobid::NodeId::new(1).unwrap();
    let configs = od.tpdo_configs(node_id);
    assert_eq!(configs[0].mappings.len(), 9);
    assert!(configs[0].mappings.iter().all(|m| m.bit_length == 1));
}

#[test]
fn mutable_pdo_mapping_rejects_capacity_and_payload_overflow() {
    let mut od = PdoTestOd::new();

    od.write(0x1A00, 0, &[0]).unwrap();
    assert_eq!(
        od.write(0x1A00, 1, &0x6000_0141u32.to_le_bytes()),
        Err(canopen_core::od::OdError::ValueRange)
    );

    od.write(0x1A00, 1, &0x6000_0140u32.to_le_bytes()).unwrap();
    od.write(0x1A00, 2, &0x6000_0101u32.to_le_bytes()).unwrap();
    assert_eq!(
        od.write(0x1A00, 0, &[2]),
        Err(canopen_core::od::OdError::ValueRange)
    );
    assert_eq!(
        od.write(
            0x1A00,
            0,
            &[(canopen_core::pdo::PDO_MAX_MAPPINGS + 1) as u8]
        ),
        Err(canopen_core::od::OdError::ValueRange)
    );
}

#[test]
fn pdo_od_immutable_mapping_rejects_writes() {
    let mut od = PdoTestOd::new();

    // RPDO1 mapping is immutable (the default): neither the unlock (sub 0)
    // nor the entries accept writes — the mapping is a device invariant.
    assert!(od.write(0x1600, 0, &[0]).is_err());
    assert!(od.write(0x1600, 1, &0x6200_0210u32.to_le_bytes()).is_err());

    // Reads still work and metadata reports const access.
    let mut buf = [0u8; 4];
    assert_eq!(od.read(0x1600, 0, &mut buf).unwrap(), 1);
    assert_eq!(buf[0], 2);
    let meta = canopen_core::od::ObjectDictionary::lookup(&od, 0x1600, 1).unwrap();
    assert_eq!(meta.access, canopen_core::od::AccessType::Const);

    // The mutable TPDO mapping still reports rw.
    let meta = canopen_core::od::ObjectDictionary::lookup(&od, 0x1A00, 1).unwrap();
    assert_eq!(meta.access, canopen_core::od::AccessType::Rw);
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
    assert!(eds.contains("[1A00sub40]"));
    assert!(eds.contains("HighLimit=64"));
    assert!(eds.contains("DefaultValue=0x60000108"));
    assert!(eds.contains("[1400]"));
    assert!(eds.contains("[1600]"));
    assert!(eds.contains("[1600sub2]"));
    assert!(!eds.contains("[1600sub3]"));
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
        rpdo[1](transmission_type = event_driven, deadline = 0.5s) {
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
    assert_eq!(rpdo[0].deadline_ms, 500); // 0.5s
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
    assert_eq!(node.tpdo_cob_id(PdoNumber::of::<1>()).unwrap().raw(), 0x181);
    assert_eq!(node.rpdo_cob_id(PdoNumber::of::<1>()).unwrap().raw(), 0x201);
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

// ---- const access ----

object_dictionary! {
    pub struct ConstAccessOd {
        [0x1000] device_type: u32 = 0x191, const;
        [0x2000] level: u16 = 7, rw;
    }
}

#[test]
fn const_access_behaves_like_ro() {
    let mut od = ConstAccessOd::new();

    let meta = od.lookup(0x1000, 0).unwrap();
    assert_eq!(meta.access, canopen_core::od::AccessType::Const);

    let mut buf = [0u8; 8];
    let len = od.read(0x1000, 0, &mut buf).unwrap();
    assert_eq!(len, 4);
    assert_eq!(u32::from_le_bytes(buf[..4].try_into().unwrap()), 0x191);

    // Writes are rejected
    assert!(od.write(0x1000, 0, &0u32.to_le_bytes()).is_err());

    // Generated EDS declares const access
    assert!(ConstAccessOd::EDS.contains("AccessType=const"));
}

// ---- PDOs beyond the pre-defined connection set (numbers > 4) ----

object_dictionary! {
    pub struct ExtPdoOd {
        [0x1000] device_type: u32 = 0x0000_0191, ro;
        [0x6000] inputs: record {
            [1] in1: u16 = 0, ro, pdo;
            [2] in2: u16 = 0, ro, pdo;
        };
        [0x6200] outputs: record {
            [1] out1: u16 = 0, rw, pdo;
        };

        tpdo[1](transmission_type = event_driven) {
            in1,
        };
        // Sparse numbering is allowed; >4 requires an explicit COB-ID.
        tpdo[5](cob_id = 0x1B1, transmission_type = event_driven) {
            in2,
        };
        // Node-relative COB-ID: resolved as base + node_id, so multiple
        // devices sharing this OD get distinct COB-IDs.
        tpdo[6](cob_id = node_id + 0x1C0, transmission_type = event_driven) {
            in1,
        };
        rpdo[5](cob_id = 0x231, transmission_type = event_driven) {
            out1,
        };
    }
}

#[test]
fn ext_pdo_configs_resolve_numbers_and_cob_ids() {
    let od = ExtPdoOd::new();
    let node_id = canopen_core::cobid::NodeId::new(5).unwrap();

    let tpdo = od.tpdo_configs(node_id);
    assert_eq!(tpdo.len(), 3);
    assert_eq!(tpdo[0].od_number, 1);
    assert_eq!(tpdo[0].cob_id, 0x185); // predefined default
    assert!(tpdo[0].enabled);
    assert_eq!(tpdo[1].od_number, 5);
    assert_eq!(tpdo[1].cob_id, 0x1B1); // explicit absolute
    assert!(tpdo[1].enabled);
    assert_eq!(tpdo[2].od_number, 6);
    assert_eq!(tpdo[2].cob_id, 0x1C5); // node-relative: 0x1C0 + node 5
    assert!(tpdo[2].enabled);

    // Node-relative COB-IDs differ per node — same OD, different device
    let other = od.tpdo_configs(canopen_core::cobid::NodeId::new(9).unwrap());
    assert_eq!(other[2].cob_id, 0x1C9);
    assert_eq!(other[1].cob_id, 0x1B1); // absolute stays fixed

    let rpdo = od.rpdo_configs(node_id);
    assert_eq!(rpdo.len(), 1);
    assert_eq!(rpdo[0].od_number, 5);
    assert_eq!(rpdo[0].cob_id, 0x231);
    assert!(rpdo[0].enabled);
}

#[test]
fn ext_pdo_comm_params_at_shifted_od_index() {
    let od = ExtPdoOd::new();
    let mut buf = [0u8; 4];

    // TPDO5 comm params live at 0x1804, mapping at 0x1A04
    od.read(0x1804, 1, &mut buf).unwrap();
    assert_eq!(u32::from_le_bytes(buf), 0x1B1);
    od.read(0x1A04, 0, &mut buf[..1]).unwrap();
    assert_eq!(buf[0], 1);

    // RPDO5 comm params live at 0x1404, mapping at 0x1604
    od.read(0x1404, 1, &mut buf).unwrap();
    assert_eq!(u32::from_le_bytes(buf), 0x231);

    // There is no TPDO2-4 / RPDO1-4, so those indices must not exist
    assert!(od.read(0x1801, 1, &mut buf).is_err());
    assert!(od.read(0x1400, 1, &mut buf).is_err());
}

#[test]
fn ext_pdo_eds_contains_shifted_sections() {
    assert!(ExtPdoOd::EDS.contains("[1804]"));
    assert!(ExtPdoOd::EDS.contains("[1A04]"));
    assert!(ExtPdoOd::EDS.contains("[1404]"));
    assert!(ExtPdoOd::EDS.contains("[1604]"));
    // Absolute COB-IDs appear verbatim; predefined defaults and
    // node-relative ones as $NODEID expressions
    assert!(ExtPdoOd::EDS.contains("0x1B1"));
    assert!(ExtPdoOd::EDS.contains("$NODEID+0x180"));
    assert!(ExtPdoOd::EDS.contains("$NODEID+0x1C0"));
}

struct FixedClock(u64);
impl canopen_core::time::Clock for FixedClock {
    fn now_us(&self) -> u64 {
        self.0
    }
}

fn drain(
    transport: &mut canopen_core::transport::MailboxTransport<32, 32>,
) -> Vec<canopen_core::transport::CanFrame> {
    let mut out = Vec::new();
    while let Some(f) = transport.next_to_transmit() {
        out.push(f);
    }
    out
}

#[test]
fn ext_pdo_node_resolves_and_exchanges_frames() {
    use canopen_core::node::{Node, NodeConfig};
    use canopen_core::transport::{CanFrame, MailboxTransport};

    let od = ExtPdoOd::new();
    let node_id = canopen_core::cobid::NodeId::new(5).unwrap();
    let config = NodeConfig {
        auto_start: true,
        heartbeat_interval_ms: 0,
        ..NodeConfig::from_od(&od, node_id)
    };
    let mut node: Node<ExtPdoOd, 3, 1> = Node::new(config, od);
    let mut transport = MailboxTransport::<32, 32>::new();

    // Node::new mirrors resolved COB-IDs into the OD: the defaulted TPDO1
    // and node-relative TPDO6 now read back as real values instead of 0.
    let mut buf = [0u8; 4];
    node.od().read(0x1800, 1, &mut buf).unwrap();
    assert_eq!(u32::from_le_bytes(buf), 0x185);
    node.od().read(0x1804, 1, &mut buf).unwrap();
    assert_eq!(u32::from_le_bytes(buf), 0x1B1);
    node.od().read(0x1805, 1, &mut buf).unwrap();
    assert_eq!(u32::from_le_bytes(buf), 0x1C5);

    // Boot (heartbeat frame), then the node is Operational (auto_start)
    node.process(&mut transport, &FixedClock(0));
    drain(&mut transport);

    // TPDO5: change in2, expect an event-driven frame on the explicit COB-ID
    node.od_mut().in2 = 0xBEEF;
    node.process(&mut transport, &FixedClock(1_000));
    let frames = drain(&mut transport);
    assert!(
        frames
            .iter()
            .any(|f| f.raw_id() == 0x1B1 && f.data() == 0xBEEFu16.to_le_bytes()),
        "expected TPDO5 frame on 0x1B1, got {frames:?}"
    );

    // RPDO5: frame on 0x231 writes out1
    transport
        .store_received(CanFrame::new(0x231, &0x1234u16.to_le_bytes()).unwrap())
        .unwrap();
    node.process(&mut transport, &FixedClock(2_000));
    assert_eq!(node.od().out1, 0x1234);
}

#[test]
fn ext_pdo_survives_reset() {
    use canopen_core::node::{Node, NodeConfig, ResetType};
    use canopen_core::transport::MailboxTransport;

    let od = ExtPdoOd::new();
    let node_id = canopen_core::cobid::NodeId::new(5).unwrap();
    let config = NodeConfig {
        auto_start: true,
        heartbeat_interval_ms: 0,
        ..NodeConfig::from_od(&od, node_id)
    };
    let mut node: Node<ExtPdoOd, 3, 1> = Node::new(config, od);
    let mut transport = MailboxTransport::<32, 32>::new();
    node.process(&mut transport, &FixedClock(0));

    // request_reset re-syncs PDO config from the OD; resolved COB-IDs must survive
    node.request_reset(ResetType::Communication);
    node.process(&mut transport, &FixedClock(1_000));

    assert_eq!(node.tpdo_cob_id(PdoNumber::of::<1>()).unwrap().raw(), 0x185);
    assert_eq!(node.tpdo_cob_id(PdoNumber::of::<5>()).unwrap().raw(), 0x1B1);
    assert_eq!(node.tpdo_cob_id(PdoNumber::of::<6>()).unwrap().raw(), 0x1C5);
    assert_eq!(node.rpdo_cob_id(PdoNumber::of::<5>()).unwrap().raw(), 0x231);
    assert!(
        node.tpdo_engine()
            .config(PdoNumber::of::<1>())
            .unwrap()
            .enabled
    );
    assert!(
        node.tpdo_engine()
            .config(PdoNumber::of::<5>())
            .unwrap()
            .enabled
    );
    assert!(
        node.tpdo_engine()
            .config(PdoNumber::of::<6>())
            .unwrap()
            .enabled
    );

    // Sparse numbering: undeclared numbers answer None/false — never an
    // off-by-slot neighbor. (TPDO2 is not declared; slot 1 holds TPDO5.)
    assert!(node.tpdo_cob_id(PdoNumber::of::<2>()).is_none());
    assert!(node.rpdo_cob_id(PdoNumber::of::<1>()).is_none()); // only RPDO5 exists
    assert!(!node.rpdo_deadline_expired(PdoNumber::of::<1>()));
}

#[test]
fn rpdo_deadline_configured_at_runtime_via_sdo() {
    use canopen_core::node::{Node, NodeConfig};
    use canopen_core::od::OdEventSource;
    use canopen_core::transport::{CanFrame, MailboxTransport};

    let od = PdoTestOd::new();
    let node_id = canopen_core::cobid::NodeId::new(1).unwrap();
    let config = NodeConfig {
        auto_start: true,
        heartbeat_interval_ms: 0,
        ..NodeConfig::from_od(&od, node_id)
    };
    // The DSL declared deadline = 250 ms; it lands in the config via the OD.
    assert_eq!(config.rpdo[0].deadline_ms, 250);
    let mut node: PdoTestOdNode = Node::new(config, od);
    let mut transport = MailboxTransport::<32, 32>::new();
    node.process(&mut transport, &FixedClock(0));
    drain(&mut transport);

    // PDO comm params are locked while Operational (CiA 301): drop to
    // Pre-Operational, tighten the deadline to 50 ms via SDO (expedited
    // write to 0x1400:5), then restart. Node::process resyncs the PDO
    // engines after writes to 0x1400-0x1BFF.
    transport
        .store_received(CanFrame::new(0x000, &[0x80, 0x01]).unwrap())
        .unwrap();
    let sdo = CanFrame::new(0x601, &[0x2B, 0x00, 0x14, 0x05, 50, 0, 0, 0]).unwrap();
    transport.store_received(sdo).unwrap();
    node.process(&mut transport, &FixedClock(1_000));
    let resp = drain(&mut transport);
    assert_eq!(resp.len(), 1, "expected SDO response, got {resp:?}");
    assert_eq!(resp[0].data()[0], 0x60, "SDO write aborted: {resp:?}");
    assert_eq!(
        node.rpdo_engine()
            .config(PdoNumber::of::<1>())
            .unwrap()
            .deadline_ms,
        50
    );
    transport
        .store_received(CanFrame::new(0x000, &[0x01, 0x01]).unwrap())
        .unwrap();
    node.process(&mut transport, &FixedClock(1_500));
    while node.next_event().is_some() {}

    // Arm with one RPDO frame (led + echo_in), then 60 ms of silence.
    transport
        .store_received(CanFrame::new(0x201, &[0x01, 0x02, 0x03]).unwrap())
        .unwrap();
    node.process(&mut transport, &FixedClock(2_000));
    while node.next_event().is_some() {}
    assert!(!node.rpdo_deadline_expired(PdoNumber::of::<1>()));

    node.process(&mut transport, &FixedClock(62_100));
    assert!(node.rpdo_deadline_expired(PdoNumber::of::<1>()));
    let evt = node.next_event().unwrap();
    assert_eq!(evt.index, 0x1400);
    assert_eq!(evt.source, OdEventSource::RpdoDeadline);
    // Last received values are retained while the deadline is expired.
    assert_eq!(node.od().led, 0x01);
}

// ---- INTEGER24 / UNSIGNED24 ----

object_dictionary! {
    pub struct I24Od {
        [0x1000] device_type: u32 = 0x0000_0000, ro;
        [0x2000] sensor: record {
            [1] raw24: i24 = 0, ro, pdo;
            [2] setpoint24: i24 = 0, rw, pdo;
            [3] counter24: u24 = 0, rw;
        };
        [0x2001] history24: array<i24, 2>, rw;

        tpdo[1](transmission_type = event_driven) {
            raw24,
        };
    }
}

#[test]
fn i24_repr_and_roundtrip() {
    let mut od = I24Od::new();

    // Rust repr is i32/u32.
    od.raw24 = -1_234_567;
    let _: i32 = od.raw24;
    let _: u32 = od.counter24;

    // Read: 3 bytes LE, two's complement.
    let mut buf = [0u8; 8];
    assert_eq!(od.read(0x2000, 1, &mut buf), Ok(3));
    let ext = if (buf[2] & 0x80) != 0 { 0xFF } else { 0 };
    assert_eq!(
        i32::from_le_bytes([buf[0], buf[1], buf[2], ext]),
        -1_234_567
    );

    // Write: negative value sign-extends into the i32 repr.
    let neg = (-42_i32).to_le_bytes();
    assert_eq!(od.write(0x2000, 2, &neg[..3]), Ok(()));
    assert_eq!(od.setpoint24, -42);

    // Write: positive value stays positive.
    let pos = 0x7FFFFF_i32.to_le_bytes();
    assert_eq!(od.write(0x2000, 2, &pos[..3]), Ok(()));
    assert_eq!(od.setpoint24, 0x7FFFFF);

    // u24 zero-extends.
    assert_eq!(od.write(0x2000, 3, &[0xFF, 0xFF, 0xFF]), Ok(()));
    assert_eq!(od.counter24, 0xFF_FFFF);

    // Wrong size is rejected.
    assert_eq!(
        od.write(0x2000, 2, &pos[..4]),
        Err(canopen_core::od::OdError::DataTypeMismatch)
    );

    // Array elements behave the same.
    assert_eq!(od.write(0x2001, 1, &neg[..3]), Ok(()));
    assert_eq!(od.history24[0], -42);
    assert_eq!(od.read(0x2001, 1, &mut buf), Ok(3));
}

#[test]
fn i24_metadata_and_mapping() {
    use canopen_core::datatypes::DataType;
    let od = I24Od::new();

    let meta = od.lookup(0x2000, 1).unwrap();
    assert_eq!(meta.data_type, DataType::I24);
    assert_eq!(meta.data_type.size(), Some(3));
    let meta = od.lookup(0x2000, 3).unwrap();
    assert_eq!(meta.data_type, DataType::U24);

    // PDO mapping entry: 24-bit length.
    let mut buf = [0u8; 8];
    assert_eq!(od.read(0x1A00, 1, &mut buf), Ok(4));
    let mapping = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
    assert_eq!(mapping, 0x2000_0118);

    // EDS export carries the CiA codes.
    assert!(I24Od::EDS.contains("DataType=0x0010"));
    assert!(I24Od::EDS.contains("DataType=0x0016"));
}

// ---- Additional SDO servers (0x1201+) ----

// DiagOd: a node with a second ("diagnostics") SDO server alongside the
// default. Node-relative COB-IDs so multiple devices can share this OD.
object_dictionary! {
    pub struct DiagOd {
        [0x1000] device_type: u32 = 0x0000_0191, ro;
        [0x2000] level: u16 = 7, rw;

        sdo_server[2](cob_rx = node_id + 0x640, cob_tx = node_id + 0x5C0);
    }
}

#[test]
fn sdo_server_od_records_and_count() {
    // The declaration count is exposed, and the 0x1201 record reads back the
    // node-relative-resolved COB-IDs after Node::new mirrors them in.
    assert_eq!(DiagOd::SDO_COUNT, 1);

    use canopen_core::node::{Node, NodeConfig};
    let od = DiagOd::new();
    let node_id = canopen_core::cobid::NodeId::new(5).unwrap();
    let config = NodeConfig {
        heartbeat_interval_ms: 0,
        ..NodeConfig::from_od(&od, node_id)
    };
    let node: Node<DiagOd, 0, 0, 1> = Node::new(config, od);

    let mut b1 = [0u8; 1];
    node.od().read(0x1201, 0, &mut b1).unwrap();
    assert_eq!(b1[0], 2, "highest subindex");

    let mut b4 = [0u8; 4];
    node.od().read(0x1201, 1, &mut b4).unwrap();
    assert_eq!(u32::from_le_bytes(b4), 0x645, "cob_rx = node_id + 0x640");
    node.od().read(0x1201, 2, &mut b4).unwrap();
    assert_eq!(u32::from_le_bytes(b4), 0x5C5, "cob_tx = node_id + 0x5C0");

    // The records are const: writes to the COB-IDs are rejected.
    use canopen_core::od::{ObjectDictionary, OdError};
    let mut writable = DiagOd::new();
    assert_eq!(
        writable.write(0x1201, 1, &0x777u32.to_le_bytes()),
        Err(OdError::ReadOnly)
    );
    assert_eq!(
        writable.write(0x1201, 2, &0x777u32.to_le_bytes()),
        Err(OdError::ReadOnly)
    );

    // EDS export describes the additional server as a read-only, node-relative
    // SDO Server Parameter record.
    assert!(DiagOd::EDS.contains("[1201]"));
    assert!(DiagOd::EDS.contains("$NODEID+0x640"));
    assert!(DiagOd::EDS.contains("$NODEID+0x5C0"));
}

#[test]
fn sdo_server_second_channel_serves_and_is_independent() {
    use canopen_core::node::{Node, NodeConfig};
    use canopen_core::transport::{CanFrame, MailboxTransport};

    let od = DiagOd::new();
    let node_id = canopen_core::cobid::NodeId::new(5).unwrap();
    let config = NodeConfig {
        auto_start: true,
        heartbeat_interval_ms: 0,
        ..NodeConfig::from_od(&od, node_id)
    };
    let mut node: Node<DiagOd, 0, 0, 1> = Node::new(config, od);
    let mut transport = MailboxTransport::<32, 32>::new();
    node.process(&mut transport, &FixedClock(0));
    drain(&mut transport);

    // Default channel (0x605 -> 0x585): expedited upload of 0x1000:0.
    transport
        .store_received(CanFrame::new(0x605, &[0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0]).unwrap())
        .unwrap();
    // Diagnostics channel (0x645 -> 0x5C5): the same upload, interleaved.
    transport
        .store_received(CanFrame::new(0x645, &[0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0]).unwrap())
        .unwrap();
    node.process(&mut transport, &FixedClock(1_000));
    let frames = drain(&mut transport);

    let default_resp = frames.iter().find(|f| f.raw_id() == 0x585);
    let diag_resp = frames.iter().find(|f| f.raw_id() == 0x5C5);
    assert!(default_resp.is_some(), "default channel response on 0x585");
    assert!(diag_resp.is_some(), "diag channel response on 0x5C5");
    // Both are expedited upload responses (cs 0x43) carrying device_type 0x191.
    for resp in [default_resp.unwrap(), diag_resp.unwrap()] {
        assert_eq!(resp.data()[0], 0x43);
        assert_eq!(u32::from_le_bytes(resp.data()[4..8].try_into().unwrap()), 0x191);
    }

    // Writing via the diagnostics channel updates the shared OD, and the
    // response comes back on the diag tx COB-ID.
    transport
        .store_received(
            CanFrame::new(0x645, &[0x2B, 0x00, 0x20, 0x00, 0x2A, 0x00, 0, 0]).unwrap(),
        )
        .unwrap();
    node.process(&mut transport, &FixedClock(2_000));
    let frames = drain(&mut transport);
    let dl_resp = frames.iter().find(|f| f.raw_id() == 0x5C5).unwrap();
    assert_eq!(dl_resp.data()[0], 0x60, "download acknowledged");
    assert_eq!(node.od().level, 0x2A);
}

#[test]
fn sdo_server_channels_have_independent_transfer_state() {
    use canopen_core::node::{Node, NodeConfig};
    use canopen_core::transport::{CanFrame, MailboxTransport};

    let od = DiagOd::new();
    let node_id = canopen_core::cobid::NodeId::new(5).unwrap();
    let config = NodeConfig {
        auto_start: true,
        heartbeat_interval_ms: 0,
        ..NodeConfig::from_od(&od, node_id)
    };
    let mut node: Node<DiagOd, 0, 0, 1> = Node::new(config, od);
    let mut transport = MailboxTransport::<32, 32>::new();
    node.process(&mut transport, &FixedClock(0));
    drain(&mut transport);

    // Start a segmented upload of the (large) EDS object on the default channel
    // without finishing it — this parks a transfer in the default server.
    transport
        .store_received(CanFrame::new(0x605, &[0x40, 0x21, 0x10, 0x00, 0, 0, 0, 0]).unwrap())
        .unwrap();
    node.process(&mut transport, &FixedClock(1_000));
    let init = drain(&mut transport);
    assert!(init.iter().any(|f| f.raw_id() == 0x585), "default upload init");

    // Meanwhile a full expedited transaction on the diag channel must complete
    // normally, unaffected by the parked default transfer.
    transport
        .store_received(CanFrame::new(0x645, &[0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0]).unwrap())
        .unwrap();
    node.process(&mut transport, &FixedClock(2_000));
    let frames = drain(&mut transport);
    let diag = frames.iter().find(|f| f.raw_id() == 0x5C5).unwrap();
    assert_eq!(diag.data()[0], 0x43);
    assert_eq!(u32::from_le_bytes(diag.data()[4..8].try_into().unwrap()), 0x191);

    // The default channel's segmented transfer is still alive: continue it and
    // get the first segment back.
    transport
        .store_received(CanFrame::new(0x605, &[0x60, 0, 0, 0, 0, 0, 0, 0]).unwrap())
        .unwrap();
    node.process(&mut transport, &FixedClock(3_000));
    let frames = drain(&mut transport);
    assert!(
        frames.iter().any(|f| f.raw_id() == 0x585),
        "default segmented transfer continued independently"
    );
}

// CollideOd: an absolute diagnostics rx COB-ID (0x605) that coincides with the
// *default* server's rx (0x600 + node_id) only for node id 5. The macro cannot
// catch this (absolute vs the default's node-relative COB-ID), so it is the
// residual case enforced by Node::new's runtime assert.
object_dictionary! {
    pub struct CollideOd {
        [0x1000] device_type: u32 = 0x0000_0191, ro;
        sdo_server[2](cob_rx = 0x605, cob_tx = 0x5C5);
    }
}

#[test]
#[should_panic(expected = "collide")]
fn sdo_server_runtime_collision_with_default_panics() {
    use canopen_core::node::{Node, NodeConfig};
    let od = CollideOd::new();
    // node id 5 -> default rx = 0x605, colliding with the diagnostics rx.
    let node_id = canopen_core::cobid::NodeId::new(5).unwrap();
    let config = NodeConfig {
        heartbeat_interval_ms: 0,
        ..NodeConfig::from_od(&od, node_id)
    };
    let _node: Node<CollideOd, 0, 0, 1> = Node::new(config, od);
}

#[test]
fn sdo_server_no_collision_at_other_node_id() {
    use canopen_core::node::{Node, NodeConfig};
    let od = CollideOd::new();
    // node id 6 -> default rx = 0x606, tx = 0x586; no overlap with 0x605/0x5C5.
    let node_id = canopen_core::cobid::NodeId::new(6).unwrap();
    let config = NodeConfig {
        heartbeat_interval_ms: 0,
        ..NodeConfig::from_od(&od, node_id)
    };
    let _node: Node<CollideOd, 0, 0, 1> = Node::new(config, od);
}
