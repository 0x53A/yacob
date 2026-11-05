//! Offline review reproductions, against f3da028 (2026-09-07).
//!
//! Fixed defects, including R05's SDO response refusal, run as regressions.
//! See `_Tasks/review-2026-09-07.md` for scope, evidence, and recommendations.

use canopen_core::cobid::NodeId;
use canopen_core::nmt::NmtCommand;
use canopen_core::node::{Node, NodeConfig};
use canopen_core::sdo::{AsyncCan, SdoClient, SdoClientResult, SdoDriver, SdoError};
use canopen_core::slcan::{SerialPort, SlcanTransport};
use canopen_core::time::Clock;
use canopen_core::transport::{CanError, CanFrame, MailboxTransport};
use canopen_derive::object_dictionary;
use core::cell::Cell;
use core::future::Future;
use core::task::{Context, Poll};
use embedded_can::nb::Can;
use std::collections::VecDeque;
use std::task::Waker;

fn once<F: Future>(future: F) -> F::Output {
    let mut cx = Context::from_waker(Waker::noop());
    match core::pin::pin!(future).as_mut().poll(&mut cx) {
        Poll::Ready(value) => value,
        Poll::Pending => panic!("fixture should complete synchronously"),
    }
}

fn node_id() -> NodeId {
    NodeId::new(1).unwrap()
}

#[test]
fn upload_response_must_match_requested_object() {
    let mut client = SdoClient::<256>::new(node_id());
    client.start_upload(0x2000, 0);
    let result = client.process_response(&[0x43, 0x01, 0x20, 0, 1, 2, 3, 4]);
    assert!(!matches!(result, SdoClientResult::UploadComplete { .. }));
}

#[test]
fn upload_segment_encoder_rejects_oversized_payload_without_panicking() {
    for len in 8..=16 {
        let data = [0x55; 16];
        let result = std::panic::catch_unwind(|| {
            canopen_core::sdo::protocol::encode_upload_segment_response(false, &data[..len], true)
        });
        assert!(result.is_ok(), "oversized segment {len} panicked");
        assert!(result.unwrap().is_none());
    }
    for len in 0..=7 {
        for toggle in [false, true] {
            let data = [0x55; 7];
            let encoded = canopen_core::sdo::protocol::encode_upload_segment_response(
                toggle,
                &data[..len],
                true,
            )
            .unwrap();
            assert_eq!(&encoded[1..1 + len], &data[..len]);
            assert_eq!((encoded[0] >> 1) & 7, (7 - len) as u8);
            assert_eq!(encoded[0] & 0x10 != 0, toggle);
        }
    }
}

#[test]
fn download_response_must_match_requested_object() {
    let mut client = SdoClient::<256>::new(node_id());
    client.start_download(0x2000, 0, &[1]).unwrap();
    let result = client.process_response(&[0x60, 0x01, 0x20, 0, 0, 0, 0, 0]);
    assert!(!matches!(result, SdoClientResult::DownloadComplete));
}

struct Responses(VecDeque<CanFrame>);
impl Responses {
    fn one(payload: [u8; 8]) -> Self {
        Self(VecDeque::from([CanFrame::new(0x581, &payload).unwrap()]))
    }
}
impl AsyncCan for Responses {
    type Error = ();
    async fn transmit(&mut self, _: &CanFrame) -> Result<(), ()> {
        Ok(())
    }
    async fn receive(&mut self) -> Result<CanFrame, ()> {
        self.0.pop_front().ok_or(())
    }
}

#[test]
fn typed_read_must_check_wire_width() {
    let mut can = Responses::one([0x4f, 0, 0x20, 0, 0xff, 0, 0, 0]);
    let result = once(SdoDriver::new(node_id()).read_u32(0x2000, 0, &mut can));
    assert!(result.is_err(), "accepted mismatched wire type: {result:?}");
}

#[test]
fn typed_read_accepts_expedited_reply_without_size() {
    // e=1, s=0: legal per CiA 301, four payload bytes, object is the prefix.
    let mut can = Responses::one([0x42, 0, 0x20, 0, 0x34, 0x12, 0xAA, 0xBB]);
    let result = once(SdoDriver::new(node_id()).read_u16(0x2000, 0, &mut can));
    assert_eq!(result.unwrap(), 0x1234);
    let mut can = Responses::one([0x42, 0, 0x20, 0, 0x78, 0x56, 0x34, 0x12]);
    let result = once(SdoDriver::new(node_id()).read_u32(0x2000, 0, &mut can));
    assert_eq!(result.unwrap(), 0x1234_5678);
}

#[test]
fn upload_must_not_succeed_with_truncated_output() {
    let mut can = Responses::one([0x43, 0, 0x20, 0, 1, 2, 3, 4]);
    let mut buffer = [0; 1];
    let result = once(SdoDriver::new(node_id()).upload(0x2000, 0, &mut buffer, &mut can));
    assert!(
        result.is_err(),
        "reported success despite truncation: {result:?}"
    );
}

#[test]
fn small_client_buffer_must_not_panic_on_expedited_upload() {
    let result = std::panic::catch_unwind(|| {
        let mut client = SdoClient::<1>::new(node_id());
        client.start_upload(0x2000, 0);
        client.process_response(&[0x43, 0, 0x20, 0, 1, 2, 3, 4])
    });
    assert!(result.is_ok(), "wire input caused an out-of-bounds panic");
}

#[test]
fn segmented_upload_must_match_announced_length() {
    let mut client = SdoClient::<256>::new(node_id());
    client.start_upload(0x2000, 0);
    assert!(matches!(
        client.process_response(&[0x41, 0, 0x20, 0, 10, 0, 0, 0]),
        SdoClientResult::SendNext(_)
    ));
    // Server announced ten bytes, then ends the transfer after one.
    let result = client.process_response(&[0x0d, 0x55, 0, 0, 0, 0, 0, 0]);
    assert!(!matches!(result, SdoClientResult::UploadComplete { .. }));
}

struct TestClock(Cell<u64>);
impl Clock for TestClock {
    fn now_us(&self) -> u64 {
        self.0.get()
    }
}

struct StalledTx;
impl AsyncCan for StalledTx {
    type Error = ();
    async fn transmit(&mut self, _: &CanFrame) -> Result<(), ()> {
        core::future::pending().await
    }
    async fn receive(&mut self) -> Result<CanFrame, ()> {
        core::future::pending().await
    }
}

#[test]
fn timed_transfer_must_expire_while_tx_is_pending() {
    let driver = SdoDriver::new(node_id());
    let clock = TestClock(Cell::new(0));
    let mut can = StalledTx;
    let mut buffer = [0; 4];
    let future = driver.upload_timed(0x2000, 0, &mut buffer, &mut can, 100, &clock);
    let mut future = core::pin::pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    assert!(future.as_mut().poll(&mut cx).is_pending());
    clock.0.set(101);
    assert!(matches!(
        future.as_mut().poll(&mut cx),
        Poll::Ready(Err(SdoError::Timeout))
    ));
}

struct StallAfterInitiate {
    sends: usize,
    response: CanFrame,
}

#[test]
fn timed_sdo_tx_and_rx_deadlines_survive_clock_wrap() {
    struct PendingIo {
        block_tx: bool,
    }
    impl AsyncCan for PendingIo {
        type Error = ();
        async fn transmit(&mut self, _: &CanFrame) -> Result<(), ()> {
            if self.block_tx {
                core::future::pending().await
            } else {
                Ok(())
            }
        }
        async fn receive(&mut self) -> Result<CanFrame, ()> {
            core::future::pending().await
        }
    }
    for upload in [false, true] {
        for block_tx in [false, true] {
            let clock = TestClock(Cell::new(u64::MAX - 50));
            let driver = SdoDriver::new(node_id());
            let mut can = PendingIo { block_tx };
            let future = async {
                if upload {
                    driver
                        .upload_timed(0x2000, 0, &mut [0; 4], &mut can, 100, &clock)
                        .await
                        .map(|_| ())
                } else {
                    driver
                        .download_timed(0x2000, 0, &[42], &mut can, 100, &clock)
                        .await
                }
            };
            let mut future = core::pin::pin!(future);
            let mut cx = Context::from_waker(Waker::noop());
            assert!(future.as_mut().poll(&mut cx).is_pending());
            clock.0.set(48); // elapsed 99 us
            assert!(future.as_mut().poll(&mut cx).is_pending());
            clock.0.set(49); // elapsed 100 us
            assert!(matches!(
                future.as_mut().poll(&mut cx),
                Poll::Ready(Err(SdoError::Timeout))
            ));
        }
    }
}
impl AsyncCan for StallAfterInitiate {
    type Error = ();
    async fn transmit(&mut self, _: &CanFrame) -> Result<(), ()> {
        self.sends += 1;
        if self.sends == 1 {
            Ok(())
        } else {
            core::future::pending().await
        }
    }
    async fn receive(&mut self) -> Result<CanFrame, ()> {
        Ok(self.response)
    }
}

#[test]
fn timed_upload_and_download_expire_during_segment_transmit() {
    for upload in [true, false] {
        let clock = TestClock(Cell::new(0));
        let driver = SdoDriver::new(node_id());
        let mut can = StallAfterInitiate {
            sends: 0,
            response: CanFrame::new(
                0x581,
                &[if upload { 0x41 } else { 0x60 }, 0, 0x20, 0, 5, 0, 0, 0],
            )
            .unwrap(),
        };
        let future = async {
            if upload {
                driver
                    .upload_timed(0x2000, 0, &mut [0; 5], &mut can, 100, &clock)
                    .await
                    .map(|_| ())
            } else {
                driver
                    .download_timed(0x2000, 0, &[1; 5], &mut can, 100, &clock)
                    .await
            }
        };
        let mut future = core::pin::pin!(future);
        let mut cx = Context::from_waker(Waker::noop());
        assert!(future.as_mut().poll(&mut cx).is_pending());
        clock.0.set(101);
        assert!(matches!(
            future.as_mut().poll(&mut cx),
            Poll::Ready(Err(SdoError::Timeout))
        ));
    }
}

object_dictionary! {
    pub struct ReviewOd {
        [0x2000] command: u8 = 0, rw, pdo;
        [0x2001] blob: domain<16>, rw;
        rpdo[1](transmission_type = sync_acyclic, deadline = 100ms) {
            command,
        };
    }
}

fn make_node() -> Node<ReviewOd, 0, 1> {
    let od = ReviewOd::new();
    let config = NodeConfig {
        auto_start: true,
        ..NodeConfig::from_od(&od, node_id())
    };
    Node::new(config, od)
}

#[test]
fn stop_start_batch_must_discard_previous_command() {
    let mut node = make_node();
    let mut transport = MailboxTransport::<16, 16>::new();
    let clock = TestClock(Cell::new(0));
    node.process(&mut transport, &clock);
    transport
        .store_received(CanFrame::new(0x201, &[42]).unwrap())
        .unwrap();
    node.process(&mut transport, &clock);
    assert_eq!(node.od().command, 0);
    transport
        .store_received(NmtCommand::StopRemoteNode.to_frame(1))
        .unwrap();
    transport
        .store_received(NmtCommand::StartRemoteNode.to_frame(1))
        .unwrap();
    transport
        .store_received(CanFrame::new(0x80, &[]).unwrap())
        .unwrap();
    node.process(&mut transport, &clock);
    assert_eq!(
        node.od().command,
        0,
        "pre-stop command committed after restart"
    );
}

#[derive(Default)]
struct RefusingCan {
    rx: VecDeque<CanFrame>,
    tx: Vec<CanFrame>,
    refuse: bool,
    refuse_id: Option<u16>,
    attempts: usize,
}
impl Can for RefusingCan {
    type Frame = CanFrame;
    type Error = CanError;
    fn transmit(&mut self, frame: &CanFrame) -> nb::Result<Option<CanFrame>, CanError> {
        self.attempts += 1;
        if self.refuse || self.refuse_id == Some(frame.raw_id()) {
            Err(nb::Error::WouldBlock)
        } else {
            self.tx.push(*frame);
            Ok(None)
        }
    }
    fn receive(&mut self) -> nb::Result<CanFrame, CanError> {
        self.rx.pop_front().ok_or(nb::Error::WouldBlock)
    }
}

#[test]
fn sdo_response_must_survive_temporary_tx_refusal() {
    let mut node = make_node();
    let mut can = RefusingCan::default();
    let clock = TestClock(Cell::new(0));
    node.process(&mut can, &clock);
    can.tx.clear();
    can.refuse = true;
    can.rx
        .push_back(CanFrame::new(0x601, &[0x40, 0, 0x20, 0, 0, 0, 0, 0]).unwrap());
    node.process(&mut can, &clock);
    can.refuse = false;
    clock.0.set(1);
    node.process(&mut can, &clock);
    assert!(can.tx.iter().any(|f| f.raw_id() == 0x581));
    let count = can.tx.len();
    node.process(&mut can, &clock);
    assert_eq!(can.tx.len(), count, "accepted response was sent twice");
}

fn queue_sdo(can: &mut RefusingCan, data: [u8; 8]) {
    can.rx.push_back(CanFrame::new(0x601, &data).unwrap());
}

#[test]
fn sdo_writes_are_ignored_while_stopped_or_initializing_on_all_channels() {
    for command in [2, 129, 130] {
        let od = ReviewOd::new();
        let base = NodeConfig::from_od(&od, node_id());
        let config = NodeConfig {
            node_id: base.node_id,
            identity: base.identity,
            heartbeat_interval_ms: 0,
            auto_start: true,
            tpdo: base.tpdo,
            rpdo: base.rpdo,
            sdo_servers: [canopen_core::sdo::SdoServerConfig {
                number: 2,
                cob_rx: 0x641,
                cob_tx: 0x5c1,
            }],
        };
        let mut node: Node<ReviewOd, 0, 1, 1> = Node::new(config, od);
        let mut can = RefusingCan::default();
        let clock = TestClock(Cell::new(0));
        node.process(&mut can, &clock);
        can.tx.clear();
        can.rx.push_back(CanFrame::new(0, &[command, 1]).unwrap());
        for id in [0x601, 0x641] {
            can.rx
                .push_back(CanFrame::new(id, &[0x2f, 0, 0x20, 0, 42, 0, 0, 0]).unwrap());
        }
        node.process(&mut can, &clock);
        assert_eq!(node.od().command, 0, "NMT command {command}");
        assert!(!can.tx.iter().any(|f| matches!(f.raw_id(), 0x581 | 0x5c1)));
    }
}

#[test]
fn stop_start_discards_refused_sdo_response() {
    let mut node = make_node();
    let mut can = RefusingCan::default();
    let clock = TestClock(Cell::new(0));
    node.process(&mut can, &clock);
    can.tx.clear();
    can.refuse = true;
    queue_sdo(&mut can, [0x40, 0, 0x20, 0, 0, 0, 0, 0]);
    node.process(&mut can, &clock);
    can.rx.push_back(NmtCommand::StopRemoteNode.to_frame(1));
    can.rx.push_back(NmtCommand::StartRemoteNode.to_frame(1));
    can.refuse = false;
    node.process(&mut can, &clock);
    assert!(!can.tx.iter().any(|f| f.raw_id() == 0x581));
}

#[test]
fn stop_start_cancels_segmented_download_but_preoperational_preserves_it() {
    for stop in [false, true] {
        let mut node = make_node();
        let mut can = RefusingCan::default();
        let clock = TestClock(Cell::new(0));
        node.process(&mut can, &clock);
        queue_sdo(&mut can, [0x21, 1, 0x20, 0, 2, 0, 0, 0]);
        node.process(&mut can, &clock);
        can.tx.clear();
        can.rx
            .push_back(CanFrame::new(0, &[if stop { 2 } else { 128 }, 1]).unwrap());
        if stop {
            can.rx.push_back(NmtCommand::StartRemoteNode.to_frame(1));
        }
        queue_sdo(&mut can, [0x0b, 42, 43, 0, 0, 0, 0, 0]);
        node.process(&mut can, &clock);
        let reply = can.tx.iter().find(|f| f.raw_id() == 0x581).unwrap();
        assert_eq!(reply.data()[0], if stop { 0x80 } else { 0x20 });
        assert_eq!(
            node.od().blob.as_slice(),
            if stop { &[][..] } else { &[42, 43][..] }
        );
    }
}

#[test]
fn empty_upload_roundtrips_as_zero_bytes() {
    let mut node = make_node();
    let mut can = RefusingCan::default();
    let clock = TestClock(Cell::new(0));
    node.process(&mut can, &clock);
    can.tx.clear();
    let mut client = SdoClient::<16>::new(node_id());
    can.rx.push_back(client.start_upload(0x2001, 0));
    for _ in 0..3 {
        node.process(&mut can, &clock);
        let frame = can.tx.remove(0);
        match client.process_response(frame.data().try_into().unwrap()) {
            SdoClientResult::SendNext(frame) => can.rx.push_back(frame),
            SdoClientResult::UploadComplete { data_len } => {
                assert_eq!(data_len, 0);
                return;
            }
            _ => panic!("empty upload failed"),
        }
    }
    panic!("empty upload did not complete");
}

#[test]
fn segmented_download_rejects_declared_length_mismatch_without_commit() {
    for declared in [0, 1, 3] {
        let mut node = make_node();
        let mut can = RefusingCan::default();
        let clock = TestClock(Cell::new(0));
        node.process(&mut can, &clock);
        queue_sdo(&mut can, [0x21, 1, 0x20, 0, declared, 0, 0, 0]);
        node.process(&mut can, &clock);
        can.tx.clear();
        queue_sdo(&mut can, [0x0b, 42, 43, 0, 0, 0, 0, 0]); // final, two bytes
        node.process(&mut can, &clock);
        assert_eq!(can.tx[0].data()[0], 0x80, "declared {declared}");
        assert!(node.od().blob.is_empty());
        assert!(node.next_event().is_none());
    }
}

#[test]
fn server_abort_does_not_leave_previous_download_alive() {
    let mut node = make_node();
    let mut can = RefusingCan::default();
    let clock = TestClock(Cell::new(0));
    node.process(&mut can, &clock);
    queue_sdo(&mut can, [0x21, 1, 0x20, 0, 2, 0, 0, 0]);
    node.process(&mut can, &clock);
    queue_sdo(&mut can, [0x40, 0xff, 0xff, 0, 0, 0, 0, 0]);
    node.process(&mut can, &clock);
    assert_eq!(can.tx.last().unwrap().data()[0], 0x80);
    queue_sdo(&mut can, [0x0b, 42, 43, 0, 0, 0, 0, 0]);
    node.process(&mut can, &clock);
    assert_eq!(can.tx.last().unwrap().data()[0], 0x80);
    assert!(node.od().blob.is_empty());
}

#[test]
fn download_unknown_size_and_zero_size_remain_distinct() {
    for (init, declared, segment, expected) in [
        (0x20, 0, [0x0b, 42, 43, 0, 0, 0, 0, 0], &[42, 43][..]),
        (0x21, 0, [0x0f, 0, 0, 0, 0, 0, 0, 0], &[][..]),
        (0x21, 2, [0x0b, 42, 43, 0, 0, 0, 0, 0], &[42, 43][..]),
    ] {
        let mut node = make_node();
        let mut can = RefusingCan::default();
        let clock = TestClock(Cell::new(0));
        node.process(&mut can, &clock);
        queue_sdo(&mut can, [init, 1, 0x20, 0, declared, 0, 0, 0]);
        node.process(&mut can, &clock);
        queue_sdo(&mut can, segment);
        node.process(&mut can, &clock);
        assert_eq!(can.tx.last().unwrap().data()[0], 0x20);
        assert_eq!(node.od().blob.as_slice(), expected);
    }
}

#[test]
fn refused_emcy_remains_pending_until_accepted() {
    let mut node = make_node();
    let mut can = RefusingCan::default();
    let clock = TestClock(Cell::new(0));
    node.process(&mut can, &clock);
    can.tx.clear();
    node.set_error(0x1000, 1, &[]);
    can.refuse = true;
    node.process(&mut can, &clock);
    can.refuse = false;
    node.process(&mut can, &clock);
    assert_eq!(can.tx.iter().filter(|f| f.raw_id() == 0x81).count(), 1);
}

#[test]
fn stopped_node_defers_emcy_but_keeps_heartbeat_running() {
    for resume in [1, 128] {
        let od = ReviewOd::new();
        let mut config = NodeConfig::from_od(&od, node_id());
        config.heartbeat_interval_ms = 1;
        config.auto_start = true;
        let mut node: Node<ReviewOd, 0, 1> = Node::new(config, od);
        let mut can = RefusingCan::default();
        let clock = TestClock(Cell::new(0));
        node.process(&mut can, &clock);
        can.tx.clear();
        node.set_error(0x1000, 1, &[]);
        can.rx.push_back(NmtCommand::StopRemoteNode.to_frame(1));
        clock.0.set(1_000);
        node.process(&mut can, &clock);
        assert!(!can.tx.iter().any(|f| f.raw_id() == 0x81));
        assert_eq!(node.error_register(), 1);
        assert!(can
            .tx
            .iter()
            .any(|f| f.raw_id() == 0x701 && f.data() == [4]));

        // Error history remains queued, including an error reset during Stop.
        node.clear_error(1);
        node.process(&mut can, &clock);
        assert!(!can.tx.iter().any(|f| f.raw_id() == 0x81));
        can.rx.push_back(CanFrame::new(0, &[resume, 1]).unwrap());
        node.process(&mut can, &clock);
        let codes: Vec<_> = can
            .tx
            .iter()
            .filter(|f| f.raw_id() == 0x81)
            .map(|f| u16::from_le_bytes([f.data()[0], f.data()[1]]))
            .collect();
        assert_eq!(codes, [0x1000, 0]);
    }
}

#[test]
fn reset_tick_does_not_emit_a_periodic_heartbeat_as_a_second_bootup() {
    for command in [129, 130] {
        let od = ReviewOd::new();
        let mut config = NodeConfig::from_od(&od, node_id());
        config.heartbeat_interval_ms = 1;
        let mut node: Node<ReviewOd, 0, 1> = Node::new(config, od);
        let mut can = RefusingCan::default();
        let clock = TestClock(Cell::new(0));
        node.process(&mut can, &clock);
        can.tx.clear();
        node.set_error(0x1000, 1, &[]);
        clock.0.set(1_000);
        can.rx.push_back(CanFrame::new(0, &[command, 1]).unwrap());
        node.process(&mut can, &clock);
        assert!(can.tx.is_empty(), "traffic before reset boot: {:?}", can.tx);
        node.process(&mut can, &clock);
        assert_eq!(can.tx.iter().filter(|f| f.raw_id() == 0x701).count(), 1);
        assert_eq!(can.tx[0].data(), &[0]);
        assert!(can.tx.iter().any(|f| f.raw_id() == 0x81));
        clock.0.set(2_000);
        node.process(&mut can, &clock);
        assert_eq!(can.tx.last().unwrap().data(), &[127]);
    }
}

#[test]
fn queued_nmt_start_cannot_skip_reset_initialization() {
    for command in [129, 130] {
        let mut node = make_node();
        let mut can = RefusingCan::default();
        let clock = TestClock(Cell::new(0));
        node.process(&mut can, &clock);
        can.tx.clear();
        can.rx.push_back(CanFrame::new(0, &[command, 1]).unwrap());
        can.rx.push_back(NmtCommand::StartRemoteNode.to_frame(1));
        queue_sdo(&mut can, [0x2f, 0, 0x20, 0, 42, 0, 0, 0]);
        can.rx.push_back(CanFrame::new(0x201, &[99]).unwrap());
        can.rx.push_back(CanFrame::new(0x80, &[]).unwrap());
        node.process(&mut can, &clock);
        assert_eq!(node.state(), canopen_core::nmt::NmtState::Initializing);
        assert_eq!(node.od().command, 0);
        assert!(can.tx.is_empty());
        node.process(&mut can, &clock);
        assert_eq!(node.state(), canopen_core::nmt::NmtState::Operational);
        assert_eq!(node.od().command, 0);
        // A fresh request after the automatic boot/start is accepted.
        queue_sdo(&mut can, [0x2f, 0, 0x20, 0, 43, 0, 0, 0]);
        node.process(&mut can, &clock);
        assert_eq!(node.od().command, 43);
    }
}

#[test]
fn heartbeat_retries_immediately_with_current_nmt_state() {
    let od = ReviewOd::new();
    let mut config = NodeConfig::from_od(&od, node_id());
    config.heartbeat_interval_ms = 1;
    config.auto_start = true;
    let mut node: Node<ReviewOd, 0, 1> = Node::new(config, od);
    let mut can = RefusingCan::default();
    let clock = TestClock(Cell::new(0));
    node.process(&mut can, &clock);
    can.tx.clear();
    can.refuse = true;
    clock.0.set(1_000);
    node.process(&mut can, &clock);
    can.refuse = false;
    can.rx.push_back(CanFrame::new(0, &[2, 1]).unwrap());
    clock.0.set(1_001);
    node.process(&mut can, &clock);
    let heartbeat = can.tx.iter().find(|f| f.raw_id() == 0x701).unwrap();
    assert_eq!(heartbeat.data(), &[4]);
}

#[test]
fn malformed_nmt_lengths_cannot_change_state() {
    for len in 0..=8 {
        let mut node = make_node();
        let mut can = RefusingCan::default();
        let clock = TestClock(Cell::new(0));
        node.process(&mut can, &clock);
        let data = [2, 1, 0, 0, 0, 0, 0, 0];
        can.rx.push_back(CanFrame::new(0, &data[..len]).unwrap());
        node.process(&mut can, &clock);
        assert_eq!(
            node.state(),
            if len == 2 {
                canopen_core::nmt::NmtState::Stopped
            } else {
                canopen_core::nmt::NmtState::Operational
            },
            "DLC {len}"
        );
    }
}

object_dictionary! {
    pub struct HeartbeatReviewOd {
        [0x1016] consumers: array<u32, 1>, rw;
    }
}

#[test]
fn malformed_heartbeat_lengths_cannot_refresh_consumer() {
    use canopen_core::heartbeat::HeartbeatMonitorState;
    for len in 0..=8 {
        let mut od = HeartbeatReviewOd::new();
        od.consumers[0] = (2 << 16) | 200;
        let config = NodeConfig::from_od(&od, node_id());
        let mut node: Node<HeartbeatReviewOd, 0, 0> = Node::new(config, od);
        let mut can = RefusingCan::default();
        let clock = TestClock(Cell::new(0));
        can.rx.push_back(CanFrame::new(0x702, &[5]).unwrap());
        node.process(&mut can, &clock);
        let data = [5, 0, 0, 0, 0, 0, 0, 0];
        can.rx
            .push_back(CanFrame::new(0x702, &data[..len]).unwrap());
        clock.0.set(190_000);
        node.process(&mut can, &clock);
        clock.0.set(200_001);
        node.process(&mut can, &clock);
        let state = node.heartbeat_status(NodeId::new(2).unwrap());
        if len == 1 {
            assert!(state.is_alive());
        } else {
            assert!(
                matches!(state, HeartbeatMonitorState::Timeout { .. }),
                "DLC {len}: {state:?}"
            );
        }
    }
}

object_dictionary! {
    pub struct TxReviewOd {
        [0x2000] value: u8 = 0, rw, pdo;
        tpdo[1](transmission_type = event_driven, inhibit_time = 10ms, event_timer = 1ms) { value, };
    }
}

object_dictionary! {
    pub struct MappingReviewOd {
        [0x2000] value: u8 = 0, rw, pdo;
        [0x2001] hidden: u8 = 0, rw;
        [0x2002] input: u8 = 0, ro, pdo;
        [0x2003] output: u8 = 0, wo, pdo;
        tpdo[1](mapping = mutable) { value, };
        rpdo[1](mapping = mutable) { value, };
    }
}

#[test]
fn sdo_pdo_identifiers_reject_unsupported_bits_without_mutating_configuration() {
    use canopen_core::od::ObjectDictionary;
    for index in [0x1400u16, 0x1800] {
        for bit in 11..=29 {
            for disabled in [0, 0x8000_0000u32] {
                let od = MappingReviewOd::new();
                let config = NodeConfig::from_od(&od, node_id());
                let mut node: Node<MappingReviewOd, 1, 1> = Node::new(config, od);
                let mut can = RefusingCan::default();
                let clock = TestClock(Cell::new(0));
                node.process(&mut can, &clock);
                let mut before = [0; 4];
                node.od().read(index, 1, &mut before).unwrap();
                let mut request = [0x23, index as u8, (index >> 8) as u8, 1, 0, 0, 0, 0];
                request[4..].copy_from_slice(&(disabled | (1 << bit) | 0x321).to_le_bytes());
                queue_sdo(&mut can, request);
                node.process(&mut can, &clock);
                let response = can
                    .tx
                    .iter()
                    .rev()
                    .find(|f| f.raw_id() == 0x581)
                    .unwrap();
                assert_eq!(response.data()[0], 0x80, "accepted bit {bit} at {index:x}");
                assert_eq!(&response.data()[4..], &0x0609_0030u32.to_le_bytes());
                let mut after = [0; 4];
                node.od().read(index, 1, &mut after).unwrap();
                assert_eq!(after, before);
                assert!(node.next_event().is_none());
            }
        }
    }
}

#[test]
fn mapping_activation_rejects_missing_ineligible_and_wrong_direction_objects() {
    use canopen_core::od::ObjectDictionary;
    for map_index in [0x1600u16, 0x1a00] {
        let wrong_direction = if map_index == 0x1600 {
            0x2002_0008
        } else {
            0x2003_0008
        };
        for descriptor in [0xffff_0008u32, 0x2000_0108, 0x2001_0008, wrong_direction] {
            let mut od = MappingReviewOd::new();
            od.write(map_index, 0, &[0]).unwrap();
            od.write(map_index, 1, &descriptor.to_le_bytes()).unwrap();
            assert!(od.write(map_index, 0, &[1]).is_err());
            let mut count = [99];
            od.read(map_index, 0, &mut count).unwrap();
            assert_eq!(count, [0]);

            let config = NodeConfig::from_od(&od, node_id());
            let mut node: Node<MappingReviewOd, 1, 1> = Node::new(config, od);
            let mut can = RefusingCan::default();
            let clock = TestClock(Cell::new(0));
            node.process(&mut can, &clock);
            can.tx.clear();
            queue_sdo(
                &mut can,
                [0x2f, map_index as u8, (map_index >> 8) as u8, 0, 1, 0, 0, 0],
            );
            node.process(&mut can, &clock);
            let frame = can.tx.iter().find(|f| f.raw_id() == 0x581).unwrap();
            assert_eq!(frame.data()[0], 0x80);
            assert_eq!(&frame.data()[4..], &0x0604_0041u32.to_le_bytes());
        }
        let mut od = MappingReviewOd::new();
        od.write(map_index, 0, &[0]).unwrap();
        od.write(map_index, 1, &0x2000_0008u32.to_le_bytes())
            .unwrap();
        od.write(map_index, 0, &[1]).unwrap();
    }
}

#[test]
fn generated_od_reads_reject_short_buffers_without_partial_writes() {
    use canopen_core::od::{ObjectDictionary, OdError};
    let od = MappingReviewOd::new();
    for (index, sub, size) in [
        (0x2000, 0, 1),
        (0x1800, 0, 1),
        (0x1800, 1, 4),
        (0x1800, 3, 2),
        (0x1a00, 1, 4),
        (0x1600, 0, 1),
    ] {
        for len in 0..size {
            let mut buf = [0xa5; 8];
            assert_eq!(
                od.read(index, sub, &mut buf[..len]),
                Err(OdError::ValueTooLong)
            );
            assert_eq!(buf, [0xa5; 8]);
        }
    }
    assert_eq!(od.read(0x2003, 0, &mut []), Err(OdError::WriteOnly));
    assert_eq!(od.read(0xffff, 0, &mut []), Err(OdError::NotFound));
}

#[test]
fn valid_dynamic_mapping_changes_actual_pdo_wire_fields() {
    use canopen_core::od::ObjectDictionary;
    let mut od = MappingReviewOd::new();
    od.input = 73;
    od.write(0x1a00, 0, &[0]).unwrap();
    od.write(0x1a00, 1, &0x2002_0008u32.to_le_bytes()).unwrap();
    od.write(0x1a00, 0, &[1]).unwrap();
    od.write(0x1600, 0, &[0]).unwrap();
    od.write(0x1600, 1, &0x2003_0008u32.to_le_bytes()).unwrap();
    od.write(0x1600, 0, &[1]).unwrap();
    let mut config = NodeConfig::from_od(&od, node_id());
    config.auto_start = true;
    let mut node: Node<MappingReviewOd, 1, 1> = Node::new(config, od);
    let mut can = RefusingCan::default();
    let clock = TestClock(Cell::new(0));
    node.process(&mut can, &clock);
    can.tx.clear();
    node.notify_changed(0x2002, 0);
    can.rx.push_back(CanFrame::new(0x201, &[42]).unwrap());
    node.process(&mut can, &clock);
    assert_eq!(node.od().output, 42);
    assert_eq!(node.od().value, 0);
    let frame = can.tx.iter().find(|f| f.raw_id() == 0x181).unwrap();
    assert_eq!(frame.data(), &[73]);
}

#[test]
fn block_download_with_declared_zero_does_not_commit_nonempty_data() {
    let mut node = make_node();
    let mut can = RefusingCan::default();
    let clock = TestClock(Cell::new(0));
    node.process(&mut can, &clock);
    queue_sdo(&mut can, [0xc2, 1, 0x20, 0, 0, 0, 0, 0]);
    node.process(&mut can, &clock);
    queue_sdo(&mut can, [0x81, 42, 0, 0, 0, 0, 0, 0]);
    node.process(&mut can, &clock);
    queue_sdo(&mut can, [0xd9, 0, 0, 0, 0, 0, 0, 0]); // six unused bytes
    node.process(&mut can, &clock);
    assert_eq!(can.tx.last().unwrap().data()[0], 0x80);
    assert!(node.od().blob.is_empty());
}

#[test]
fn block_download_cannot_truncate_a_final_segment_past_capacity() {
    object_dictionary! {
        pub struct LargeDownloadOd {
            [0x2000] data: domain<1024>, rw;
        }
    }
    for overflow in [false, true] {
        for indicated in [false, true] {
            let mut od = LargeDownloadOd::new();
            od.data.push(9).unwrap();
            let mut server = canopen_core::sdo::server::SdoServer::new();
            let mut events = heapless::Deque::<canopen_core::od::OdEvent, 8>::new();
            let mut response = [0; 8];
            let mut init = [0xc0, 0, 0x20, 0, 0, 0, 0, 0];
            if indicated {
                init[0] |= 2;
                init[4..].copy_from_slice(&889u32.to_le_bytes());
            }
            let state = canopen_core::nmt::NmtState::PreOperational;
            server
                .process(&init, &mut od, &mut response, &mut events, state, 0)
                .unwrap();
            for seq in 1..=127 {
                let mut segment = [42; 8];
                segment[0] = seq | if seq == 127 && !overflow { 0x80 } else { 0 };
                let _ = server.process(&segment, &mut od, &mut response, &mut events, state, 0);
            }
            if overflow {
                server
                    .process(
                        &[0x81, 43, 43, 43, 43, 43, 43, 43],
                        &mut od,
                        &mut response,
                        &mut events,
                        state,
                        0,
                    )
                    .unwrap();
                assert_eq!(response[0], 0x80, "overflow segment was acknowledged");
                assert_eq!(&response[4..], &0x0504_0005u32.to_le_bytes());
            }
            server
                .process(
                    &[0xc1, 0, 0, 0, 0, 0, 0, 0],
                    &mut od,
                    &mut response,
                    &mut events,
                    state,
                    0,
                )
                .unwrap();
            if overflow {
                assert_eq!(od.data.as_slice(), &[9]);
                assert!(events.is_empty());
            } else {
                assert_eq!(response[0], 0xa1);
                assert_eq!(od.data.as_slice(), &[42; 889]);
                assert_eq!(events.len(), 1);
            }
        }
    }
}

#[test]
fn segmented_roundtrip_every_supported_transfer_length() {
    object_dictionary! {
        pub struct RoundtripOd {
            [0x2000] data: domain<889>, rw;
        }
    }
    for len in 0..=889 {
        let expected: Vec<u8> = (0..len).map(|n| (n * 37 + len) as u8).collect();
        let mut od = RoundtripOd::new();
        let mut server = canopen_core::sdo::server::SdoServer::new();
        let mut client = SdoClient::<889>::new(node_id());
        let mut events = heapless::Deque::<canopen_core::od::OdEvent, 8>::new();
        for upload in [false, true] {
            let mut request = if upload {
                client.start_upload(0x2000, 0)
            } else {
                client.start_download(0x2000, 0, &expected).unwrap()
            };
            let mut complete = false;
            for _ in 0..130 {
                let mut response = [0; 8];
                server
                    .process(
                        request.data().try_into().unwrap(),
                        &mut od,
                        &mut response,
                        &mut events,
                        canopen_core::nmt::NmtState::PreOperational,
                        0,
                    )
                    .unwrap();
                match client.process_response(&response) {
                    SdoClientResult::SendNext(frame) => request = frame,
                    SdoClientResult::DownloadComplete if !upload => {
                        assert_eq!(od.data.as_slice(), expected);
                        assert_eq!(events.len(), 1);
                        complete = true;
                        break;
                    }
                    SdoClientResult::UploadComplete { data_len } if upload => {
                        assert_eq!(data_len, len);
                        assert_eq!(client.data(), expected);
                        complete = true;
                        break;
                    }
                    _ => panic!("unexpected transfer result: length {len}, upload {upload}"),
                }
            }
            assert!(
                complete,
                "transfer did not finish: length {len}, upload {upload}"
            );
        }
    }
}

#[test]
fn malformed_sdo_command_sequences_do_not_panic() {
    let mut node = make_node();
    let mut can = RefusingCan::default();
    let clock = TestClock(Cell::new(0));
    node.process(&mut can, &clock);
    let mut random = 0x1234_5678u32;
    for i in 0..32_768u64 {
        let mut data = [0u8; 8];
        for byte in &mut data {
            random ^= random << 13;
            random ^= random >> 17;
            random ^= random << 5;
            *byte = random as u8;
        }
        // Frequently address a real writable domain so successful initiation
        // and malformed follow-up frames mix, rather than only unknown indices.
        if i % 3 == 0 {
            data[1..4].copy_from_slice(&[1, 0x20, 0]);
        }
        can.refuse = i % 7 == 0;
        clock.0.set(i * 1000);
        queue_sdo(&mut can, data);
        node.process(&mut can, &clock);
        can.tx.clear();
        while node.next_event().is_some() {}
    }
}

#[test]
fn async_tpdo_retries_latest_value_and_timer_respects_inhibit() {
    let od = TxReviewOd::new();
    let mut config = NodeConfig::from_od(&od, node_id());
    config.auto_start = true;
    let mut node: Node<TxReviewOd, 1, 0> = Node::new(config, od);
    let mut can = RefusingCan::default();
    let clock = TestClock(Cell::new(0));
    node.process(&mut can, &clock);
    can.tx.clear();
    node.od_mut().value = 1;
    clock.0.set(10_000);
    can.refuse = true;
    node.process(&mut can, &clock);
    node.od_mut().value = 2;
    can.refuse = false;
    clock.0.set(10_001);
    node.process(&mut can, &clock);
    let pdo = can.tx.iter().find(|f| f.raw_id() == 0x181).unwrap();
    assert_eq!(pdo.data(), &[2]);
    can.tx.clear();
    clock.0.set(11_001);
    node.process(&mut can, &clock);
    assert!(!can.tx.iter().any(|f| f.raw_id() == 0x181));
    clock.0.set(20_001);
    node.process(&mut can, &clock);
    assert!(can.tx.iter().any(|f| f.raw_id() == 0x181));
}

#[test]
fn pending_sdo_blocks_new_writes_without_repeating_the_original_write() {
    let mut node = make_node();
    let mut can = RefusingCan::default();
    let clock = TestClock(Cell::new(0));
    node.process(&mut can, &clock);
    can.refuse = true;
    can.attempts = 0;
    queue_sdo(&mut can, [0x2f, 0, 0x20, 0, 42, 0, 0, 0]);
    queue_sdo(&mut can, [0x2f, 0, 0x20, 0, 99, 0, 0, 0]);
    node.process(&mut can, &clock);
    assert_eq!(node.od().command, 42);
    assert_eq!(can.attempts, 1, "refused response retried within same poll");
    assert!(node.next_event().is_some());
    can.refuse = false;
    node.process(&mut can, &clock);
    assert_eq!(node.od().command, 42);
    assert!(node.next_event().is_none(), "write replayed on retry");
}

#[test]
fn resets_and_client_abort_discard_pending_sdo_response() {
    for mode in 0..5 {
        let mut node = make_node();
        let mut can = RefusingCan::default();
        let clock = TestClock(Cell::new(0));
        node.process(&mut can, &clock);
        can.refuse = true;
        queue_sdo(&mut can, [0x40, 0, 0x20, 0, 0, 0, 0, 0]);
        node.process(&mut can, &clock);
        match mode {
            0 => node.request_reset(canopen_core::node::ResetType::Application),
            1 => node.request_reset(canopen_core::node::ResetType::Communication),
            2 | 3 => can
                .rx
                .push_back(CanFrame::new(0, &[if mode == 2 { 129 } else { 130 }, 1]).unwrap()),
            _ => queue_sdo(&mut can, [0x80, 0, 0x20, 0, 0, 0, 0, 0]),
        }
        can.tx.clear();
        can.refuse = false;
        node.process(&mut can, &clock);
        node.process(&mut can, &clock);
        assert!(
            !can.tx.iter().any(|f| f.raw_id() == 0x581),
            "mode {mode} replayed response"
        );
    }
}

#[test]
fn expired_response_is_not_replayed_after_transport_recovers() {
    let mut node = make_node();
    let mut can = RefusingCan::default();
    let clock = TestClock(Cell::new(0));
    node.process(&mut can, &clock);
    can.refuse = true;
    queue_sdo(&mut can, [0x40, 0, 0x20, 0, 0, 0, 0, 0]);
    node.process(&mut can, &clock);
    can.tx.clear();
    can.refuse = false;
    clock.0.set(5_000_000);
    node.process(&mut can, &clock);
    assert!(!can.tx.iter().any(|f| f.raw_id() == 0x581));
    queue_sdo(&mut can, [0x40, 0, 0x20, 0, 0, 0, 0, 0]);
    node.process(&mut can, &clock);
    assert!(can.tx.iter().any(|f| f.raw_id() == 0x581));
}

#[test]
fn timeout_abort_survives_temporary_tx_refusal() {
    let mut node = make_node();
    let mut can = RefusingCan::default();
    let clock = TestClock(Cell::new(0));
    node.process(&mut can, &clock);
    queue_sdo(&mut can, [0x21, 1, 0x20, 0, 8, 0, 0, 0]);
    node.process(&mut can, &clock);
    can.tx.clear();
    can.refuse = true;
    clock.0.set(5_000_000);
    node.process(&mut can, &clock);
    can.refuse = false;
    clock.0.set(5_000_001);
    node.process(&mut can, &clock);
    let abort = can.tx.iter().find(|f| f.raw_id() == 0x581).unwrap();
    assert_eq!(abort.data(), &[0x80, 1, 0x20, 0, 0, 0, 4, 5]);
}

#[test]
fn blocked_sdo_channel_does_not_block_another_channel_or_sync_rpdo() {
    let od = ReviewOd::new();
    let base = NodeConfig::from_od(&od, node_id());
    let config = NodeConfig {
        node_id: base.node_id,
        identity: base.identity,
        heartbeat_interval_ms: 0,
        auto_start: true,
        tpdo: base.tpdo,
        rpdo: base.rpdo,
        sdo_servers: [canopen_core::sdo::SdoServerConfig {
            number: 2,
            cob_rx: 0x641,
            cob_tx: 0x5c1,
        }],
    };
    let mut node: Node<ReviewOd, 0, 1, 1> = Node::new(config, od);
    let mut can = RefusingCan::default();
    let clock = TestClock(Cell::new(0));
    node.process(&mut can, &clock);
    can.tx.clear();
    can.refuse_id = Some(0x5c1);
    can.rx
        .push_back(CanFrame::new(0x641, &[0x40, 0, 0x20, 0, 0, 0, 0, 0]).unwrap());
    node.process(&mut can, &clock);
    queue_sdo(&mut can, [0x40, 0, 0x20, 0, 0, 0, 0, 0]);
    can.rx.push_back(CanFrame::new(0x201, &[73]).unwrap());
    can.rx.push_back(CanFrame::new(0x80, &[]).unwrap());
    node.process(&mut can, &clock);
    assert_eq!(node.od().command, 73);
    assert!(can.tx.iter().any(|f| f.raw_id() == 0x581));
    assert!(!can.tx.iter().any(|f| f.raw_id() == 0x5c1));
    can.refuse_id = None;
    node.process(&mut can, &clock);
    let reply = can.tx.iter().find(|f| f.raw_id() == 0x5c1).unwrap();
    assert_eq!(
        reply.data()[4],
        0,
        "retry must retain the original snapshot"
    );

    can.refuse_id = Some(0x5c1);
    can.rx
        .push_back(CanFrame::new(0x641, &[0x40, 0, 0x20, 0, 0, 0, 0, 0]).unwrap());
    node.process(&mut can, &clock);
    node.request_reset(canopen_core::node::ResetType::Communication);
    can.tx.clear();
    can.refuse_id = None;
    node.process(&mut can, &clock);
    assert!(!can.tx.iter().any(|f| f.raw_id() == 0x5c1));
}

#[test]
fn block_upload_retries_refused_segment_without_skipping_data() {
    let mut node = make_node();
    node.od_mut()
        .blob
        .extend_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        .unwrap();
    let mut can = RefusingCan::default();
    let clock = TestClock(Cell::new(0));
    node.process(&mut can, &clock);
    queue_sdo(&mut can, [0xa0, 1, 0x20, 0, 127, 0, 0, 0]);
    node.process(&mut can, &clock);
    can.tx.clear();
    can.refuse = true;
    can.attempts = 0;
    queue_sdo(&mut can, [0xa3, 0, 0, 0, 0, 0, 0, 0]);
    node.process(&mut can, &clock);
    assert_eq!(can.attempts, 1);
    can.refuse = false;
    node.process(&mut can, &clock);
    let frames: Vec<_> = can.tx.iter().filter(|f| f.raw_id() == 0x581).collect();
    assert_eq!(frames.len(), 2);
    assert_eq!(frames[0].data(), &[1, 1, 2, 3, 4, 5, 6, 7]);
    assert_eq!(frames[1].data(), &[0x82, 8, 9, 10, 0, 0, 0, 0]);
}

#[test]
fn full_mailbox_must_report_would_block() {
    let mut can = MailboxTransport::<1, 1>::new();
    let frame = CanFrame::new(0x701, &[5]).unwrap();
    can.transmit(&frame).unwrap();
    assert!(matches!(can.transmit(&frame), Err(nb::Error::WouldBlock)));
}

struct BrokenSerial;
impl SerialPort for BrokenSerial {
    type Error = ();
    fn read(&mut self, _: &mut [u8]) -> Result<usize, ()> {
        Err(())
    }
    fn write_all(&mut self, _: &[u8]) -> Result<(), ()> {
        Ok(())
    }
}

#[test]
fn serial_io_failure_must_remain_observable() {
    let mut can = SlcanTransport::new(BrokenSerial);
    assert!(matches!(can.receive(), Err(nb::Error::Other(_))));
}

#[test]
fn empty_download_must_not_encode_four_data_bytes() {
    let mut client = SdoClient::<256>::new(node_id());
    let frame = client.start_download(0x2000, 0, &[]).unwrap();
    // Expedited's two-bit unused-byte field cannot encode four unused bytes.
    assert_eq!(
        frame.data()[0] & 0x02,
        0,
        "empty value needs segmented encoding"
    );
}

#[cfg(feature = "alloc")]
#[test]
fn dcf_must_reject_overflowing_value() {
    let parsed = canopen_core::dcf::Dcf::parse(
        "[2000]\nDataType=0x0005\nAccessType=rw\nParameterValue=256\n",
    );
    assert!(
        parsed.is_err(),
        "invalid configuration accepted: {parsed:?}"
    );
}

#[cfg(feature = "alloc")]
#[test]
fn dcf_must_preserve_negative_integer() {
    let parsed = canopen_core::dcf::Dcf::parse(
        "[2000]\nDataType=0x0003\nAccessType=rw\nParameterValue=-1\n",
    )
    .unwrap();
    assert_eq!(parsed.entries.len(), 1);
    assert_eq!(parsed.entries[0].data, [0xff, 0xff]);
}

#[test]
fn download_roundtrips_empty_expedited_and_segmented_values() {
    for length in 0..=9 {
        let mut node = make_node();
        let mut can = RefusingCan::default();
        let clock = TestClock(Cell::new(0));
        node.process(&mut can, &clock);
        can.tx.clear();
        let mut client = SdoClient::<16>::new(node_id());
        let value = vec![0x55; length];
        let mut request = client.start_download(0x2001, 0, &value).unwrap();
        let mut complete = false;
        for _ in 0..8 {
            can.rx.push_back(request);
            node.process(&mut can, &clock);
            let response = can.tx.pop().expect("SDO response");
            match client.process_response(response.data().try_into().unwrap()) {
                SdoClientResult::SendNext(next) => request = next,
                SdoClientResult::DownloadComplete => {
                    complete = true;
                    break;
                }
                _ => panic!("download failed for length {length}"),
            }
        }
        assert!(complete);
        assert_eq!(node.od().blob.as_slice(), value);
    }
}

#[test]
fn segmented_upload_accepts_exact_and_unspecified_lengths() {
    for init in [
        [0x41, 0, 0x20, 0, 1, 0, 0, 0],
        [0x40, 0, 0x20, 0, 0, 0, 0, 0],
    ] {
        let mut client = SdoClient::<1>::new(node_id());
        client.start_upload(0x2000, 0);
        assert!(matches!(
            client.process_response(&init),
            SdoClientResult::SendNext(_)
        ));
        assert!(matches!(
            client.process_response(&[0x0d, 0x55, 0, 0, 0, 0, 0, 0]),
            SdoClientResult::UploadComplete { data_len: 1 }
        ));
        assert_eq!(client.data(), &[0x55]);
    }
}

#[test]
fn communication_reset_invalidates_rpdo_freshness_with_autostart() {
    let mut node = make_node();
    let mut can = RefusingCan::default();
    let clock = TestClock(Cell::new(0));
    node.process(&mut can, &clock);
    can.rx.push_back(CanFrame::new(0x201, &[42]).unwrap());
    node.process(&mut can, &clock);
    let number = canopen_core::PdoNumber::of::<1>();
    assert!(!node.rpdo_deadline_expired(number));
    node.request_reset(canopen_core::node::ResetType::Communication);
    node.process(&mut can, &clock);
    assert!(node.rpdo_deadline_expired(number));
}

#[cfg(feature = "alloc")]
#[test]
fn dcf_integer_boundaries_and_invalid_hex_are_checked() {
    for (ty, value, expected) in [
        (0x0001, "0x00", vec![0]),
        (0x0001, "0x01", vec![1]),
        (0x0002, "-128", vec![0x80]),
        (0x0002, "127", vec![0x7f]),
        (0x0002, "0xff", vec![0xff]),
        (0x0005, "255", vec![0xff]),
        (0x0010, "-8388608", vec![0, 0, 0x80]),
        (0x0016, "16777215", vec![0xff; 3]),
        (
            0x0015,
            "-9223372036854775808",
            i64::MIN.to_le_bytes().to_vec(),
        ),
        (0x001b, "18446744073709551615", vec![0xff; 8]),
    ] {
        let text = format!("[2000]\nDataType={ty}\nParameterValue={value}\n");
        let parsed = canopen_core::dcf::Dcf::parse(&text).unwrap();
        assert_eq!(parsed.entries[0].data, expected, "type {ty}, value {value}");
    }
    for (ty, value) in [
        (0x0002, "-129"),
        (0x0002, "128"),
        (0x0005, "-1"),
        (0x0010, "8388608"),
        (0x0016, "16777216"),
        (0x0001, "garbage"),
        (0x000f, "0xabc"),
        (0x000f, "0xa€"),
    ] {
        let text = format!("[2000]\nDataType={ty}\nParameterValue={value}\n");
        assert!(
            canopen_core::dcf::Dcf::parse(&text).is_err(),
            "type {ty}, value {value}"
        );
    }
}
