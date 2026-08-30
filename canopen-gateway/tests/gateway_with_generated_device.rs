use std::convert::Infallible;
use std::time::{Duration, Instant};

use canopen_core::cobid::NodeId;
use canopen_core::transport::CanFrame;
use canopen_derive::device_from_eds;
use canopen_gateway::{ConfirmResult, LatestPdoSet, PdoSlot, PdoToTopic, TopicToPdo, Validity};

device_from_eds! {
    pub struct Motor = "tests/gateway_device.eds";
}

#[derive(Debug, PartialEq, Eq)]
struct StatusMessage {
    statusword: u16,
}

#[derive(Clone, Copy)]
struct CommandMessage {
    controlword: u16,
}

#[test]
fn generated_pdos_compose_without_a_yaml_mapping_layer() {
    let inbound_device = Motor::new(NodeId::new(5).unwrap());
    let tpdo1 = PdoToTopic::new(
        move |frame: &CanFrame| match inbound_device.decode(frame)? {
            Some(MotorTpdo::Tpdo1(payload)) => {
                Ok::<Option<MotorTpdo1>, canopen_core::pdo::PdoDecodeError>(Some(payload))
            }
            None => Ok::<Option<MotorTpdo1>, canopen_core::pdo::PdoDecodeError>(None),
        },
        |pdo: MotorTpdo1| {
            Ok::<_, Infallible>(StatusMessage {
                statusword: pdo.statusword,
            })
        },
    );

    let status_frame = CanFrame::new(0x185, &[0x34, 0x12]).unwrap();
    assert_eq!(
        tpdo1.on_frame(&status_frame).unwrap(),
        Some(StatusMessage { statusword: 0x1234 })
    );

    let slot = PdoSlot::<1>::new(0).unwrap();
    let outbound_device = Motor::new(NodeId::new(5).unwrap());
    let rpdo1 = TopicToPdo::new(
        slot,
        Validity::For(Duration::from_millis(100)),
        move |message: CommandMessage| {
            Ok::<_, Infallible>(outbound_device.rpdo1(MotorRpdo1 {
                controlword: message.controlword,
            }))
        },
    );
    let now = Instant::now();
    let mut outgoing = LatestPdoSet::new();
    rpdo1
        .on_message(
            &mut outgoing,
            CommandMessage {
                controlword: 0xbeef,
            },
            now,
        )
        .unwrap();
    let prepared = outgoing.prepare_next(now).unwrap();
    assert_eq!(prepared.frame.raw_id(), 0x205);
    assert_eq!(prepared.frame.data(), &[0xef, 0xbe]);
    assert_eq!(outgoing.confirm(prepared), ConfirmResult::Confirmed);
}
