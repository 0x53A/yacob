use canopen_core::cobid::NodeId;
use canopen_core::pdo::PdoDecodeError;
use canopen_core::transport::CanFrame;
use canopen_derive::{device_from_eds, pdo_client_from_eds};

device_from_eds! {
    pub struct TestPdo = "tests/test_pdo_device.eds";
}

device_from_eds! {
    pub struct NamesDevice = "tests/test_sdo_client_names.eds";
}

pdo_client_from_eds! {
    pub struct RawTestPdo = "tests/test_pdo_device.eds";
}

pdo_client_from_eds! {
    pub struct AmbiguousPdo = "tests/test_pdo_ambiguous.eds";
}

#[test]
fn generated_client_resolves_cob_ids_and_encodes_rpdo() {
    let client = TestPdo::new(NodeId::new(5).unwrap());

    assert_eq!(client.tpdo1_cob_id().raw(), 0x185);
    assert_eq!(client.rpdo1_cob_id().raw(), 0x205);

    let frame = client.rpdo1(TestPdoRpdo1 {
        controlword: 0xBEEF,
    });
    assert_eq!(frame.raw_id(), 0x205);
    assert_eq!(frame.data(), &[0xEF, 0xBE]);
}

#[test]
fn pdo_only_client_remains_available() {
    let client = RawTestPdo::new(NodeId::new(5).unwrap());
    let frame = client.rpdo1(RawTestPdoRpdo1 {
        controlword: 0x1234,
    });
    assert_eq!(frame.raw_id(), 0x205);
    assert_eq!(frame.data(), &[0x34, 0x12]);
}

#[test]
fn generated_client_decodes_tpdo_and_checks_the_frame() {
    let client = TestPdo::new(NodeId::new(5).unwrap());
    let frame = CanFrame::new(0x185, &[0x34, 0x12]).unwrap();

    assert_eq!(
        client.decode(&frame).unwrap(),
        Some(TestPdoTpdo::Tpdo1(TestPdoTpdo1 { statusword: 0x1234 }))
    );

    let wrong_id = CanFrame::new(0x186, &[0x34, 0x12]).unwrap();
    assert_eq!(client.decode(&wrong_id), Ok(None));

    let short = CanFrame::new(0x185, &[0x34]).unwrap();
    assert_eq!(
        client.decode(&short),
        Err(PdoDecodeError::PayloadTooShort {
            expected: 2,
            actual: 1,
        })
    );
}

#[test]
fn custom_tpdo_is_matched_exactly_before_generic_classification() {
    let client = TestPdo::new(NodeId::new(5).unwrap());
    // EDS TPDO7 is $NODEID+0x1C2 = 0x1C7. The generic predefined-range
    // classifier calls this TPDO1 from node 0x47; the device decoder knows it
    // is this node's TPDO7.
    let frame = CanFrame::new(0x1C7, &[0x34, 0x12]).unwrap();

    assert_eq!(
        client.decode(&frame).unwrap(),
        Some(TestPdoTpdo::Tpdo7(TestPdoTpdo7 {
            controlword: 0x1234,
        }))
    );
}

#[test]
fn node_relative_and_absolute_tpdos_cannot_shadow_each_other() {
    let client = AmbiguousPdo::new(NodeId::new(7).unwrap());
    let frame = CanFrame::new(0x187, &[0x34, 0x12]).unwrap();

    assert_eq!(
        client.decode(&frame),
        Err(PdoDecodeError::AmbiguousCobId { cob_id: 0x187 })
    );
}

#[allow(dead_code)]
async fn device_sdo_methods_are_always_addressed(
    device: &NamesDevice,
    can: &mut impl canopen_core::sdo::AsyncCan<Error = core::convert::Infallible>,
) {
    let _ = device.read_i2032_s00_max_motor_speed(can).await;
    let _ = device.read_i6080_s00_max_motor_speed(can).await;
}
