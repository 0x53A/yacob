//! SdoDriver client-side abort behavior.

use canopen_core::cobid::NodeId;
use canopen_core::sdo::protocol::AbortCode;
use canopen_core::sdo::{AsyncCan, SdoDriver, SdoError};
use canopen_core::time::Clock;
use canopen_core::transport::CanFrame;
use core::cell::Cell;
use core::future::Future;
use core::task::{Context, Poll, Waker};
use std::collections::VecDeque;

struct Recorder {
    responses: VecDeque<CanFrame>,
    sent: Vec<CanFrame>,
}

impl AsyncCan for Recorder {
    type Error = ();
    async fn transmit(&mut self, frame: &CanFrame) -> Result<(), ()> {
        self.sent.push(*frame);
        Ok(())
    }
    async fn receive(&mut self) -> Result<CanFrame, ()> {
        match self.responses.pop_front() {
            Some(frame) => Ok(frame),
            None => core::future::pending().await,
        }
    }
}

fn recorder(responses: &[[u8; 8]]) -> Recorder {
    Recorder {
        responses: responses
            .iter()
            .map(|r| CanFrame::new(0x581, r).unwrap())
            .collect(),
        sent: Vec::new(),
    }
}

fn abort_code(frame: &CanFrame) -> Option<u32> {
    let d = frame.data();
    (frame.raw_id() == 0x601 && d[0] == 0x80).then(|| u32::from_le_bytes([d[4], d[5], d[6], d[7]]))
}

struct StepClock(Cell<u64>);
impl Clock for StepClock {
    fn now_us(&self) -> u64 {
        // Every query advances time, so a pending receive eventually expires.
        let now = self.0.get();
        self.0.set(now + 1_000);
        now
    }
}

fn run<F: Future>(future: F) -> F::Output {
    let mut cx = Context::from_waker(Waker::noop());
    let mut future = core::pin::pin!(future);
    for _ in 0..10_000 {
        if let Poll::Ready(v) = future.as_mut().poll(&mut cx) {
            return v;
        }
    }
    panic!("future did not complete");
}

#[test]
fn protocol_error_sends_abort() {
    // Segmented upload announced, then a download response arrives.
    let mut can = recorder(&[
        [0x41, 0x00, 0x20, 0x00, 20, 0, 0, 0],
        [0x60, 0x00, 0x20, 0x00, 0, 0, 0, 0],
    ]);
    let driver = SdoDriver::new(NodeId::new(1).unwrap());
    let mut buf = [0u8; 64];
    let result = run(driver.upload(0x2000, 0, &mut buf, &mut can));
    assert!(matches!(result, Err(SdoError::ProtocolError)), "{result:?}");
    let last = can.sent.last().unwrap();
    assert_eq!(abort_code(last), Some(AbortCode::InvalidCommandSpecifier as u32));
    assert_eq!(&last.data()[1..4], &[0x00, 0x20, 0x00]);
}

#[test]
fn timeout_sends_abort() {
    let mut can = recorder(&[]);
    let driver = SdoDriver::new(NodeId::new(1).unwrap());
    let clock = StepClock(Cell::new(0));
    let result = run(driver.download_timed(0x2000, 0, &[1, 2, 3, 4, 5], &mut can, 50_000, &clock));
    assert!(matches!(result, Err(SdoError::Timeout)), "{result:?}");
    assert_eq!(
        abort_code(can.sent.last().unwrap()),
        Some(AbortCode::SdoProtocolTimeout as u32)
    );
}

#[test]
fn server_abort_and_success_send_no_abort() {
    let mut can = recorder(&[[0x80, 0x00, 0x20, 0x00, 0x00, 0x00, 0x02, 0x06]]);
    let driver = SdoDriver::new(NodeId::new(1).unwrap());
    let result = run(driver.read_u8(0x2000, 0, &mut can));
    assert!(matches!(result, Err(SdoError::Aborted(_))));
    assert!(can.sent.iter().all(|f| abort_code(f).is_none()));

    let mut can = recorder(&[[0x4F, 0x00, 0x20, 0x00, 7, 0, 0, 0]]);
    assert_eq!(run(driver.read_u8(0x2000, 0, &mut can)).unwrap(), 7);
    assert_eq!(can.sent.len(), 1);
}

#[test]
fn signed_and_wide_typed_helpers_roundtrip_wire_bytes() {
    let driver = SdoDriver::new(NodeId::new(1).unwrap());
    let mut can = recorder(&[[0x4B, 0x00, 0x20, 0x00, 0xFE, 0xFF, 0, 0]]);
    assert_eq!(run(driver.read_i16(0x2000, 0, &mut can)).unwrap(), -2);
    let mut can = recorder(&[[0x60, 0x00, 0x20, 0x00, 0, 0, 0, 0]]);
    run(driver.write_i32(0x2000, 0, -1, &mut can)).unwrap();
    assert_eq!(can.sent[0].data(), &[0x23, 0x00, 0x20, 0x00, 0xFF, 0xFF, 0xFF, 0xFF]);
}
