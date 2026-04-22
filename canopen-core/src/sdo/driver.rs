//! Async SDO driver — drives complete SDO transfers over an async CAN transport.
//!
//! ```ignore
//! let sdo = SdoDriver::new(target_node);
//! let mut buf = [0u8; 4];
//! let len = sdo.upload(0x6041, 0, &mut buf, &mut can).await?;
//! let status = u16::from_le_bytes(buf[..2].try_into().unwrap());
//! ```

use crate::cobid::{CobId, NodeId};
use crate::sdo::client::{SdoClient, SdoClientResult};
use crate::sdo::AbortCode;
use crate::time::Clock;
use crate::transport::CanFrame;

/// Async CAN transport trait, modeled after embedded_hal_async patterns.
///
/// Implementations should provide async send/receive for CAN frames.
/// For split TX/RX drivers (e.g. Embassy), wrap both halves in a single struct.
pub trait AsyncCan {
    type Error: core::fmt::Debug;

    /// Send a CAN frame.
    fn transmit(
        &mut self,
        frame: &CanFrame,
    ) -> impl core::future::Future<Output = Result<(), Self::Error>>;

    /// Receive the next CAN frame from the bus.
    fn receive(&mut self) -> impl core::future::Future<Output = Result<CanFrame, Self::Error>>;
}

/// Errors from async SDO operations.
#[derive(Debug)]
pub enum SdoError<E: core::fmt::Debug> {
    /// Transfer aborted by the remote node.
    Aborted(AbortCode),
    /// Protocol error (unexpected response, toggle mismatch, etc.)
    ProtocolError,
    /// CAN transport error.
    Transport(E),
    /// Transfer timed out waiting for a response.
    Timeout,
}

impl<E: core::fmt::Debug> core::fmt::Display for SdoError<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Aborted(code) => write!(f, "SDO aborted: {:?}", code),
            Self::ProtocolError => write!(f, "SDO protocol error"),
            Self::Transport(e) => write!(f, "CAN transport error: {:?}", e),
            Self::Timeout => write!(f, "SDO timeout"),
        }
    }
}

/// High-level async SDO driver for a single remote node.
///
/// Stateless between transfers — each `upload`/`download` drives a complete
/// SDO transaction to completion.
pub struct SdoDriver {
    target: NodeId,
    response_cob: u16,
}

impl SdoDriver {
    pub fn new(target: NodeId) -> Self {
        Self {
            response_cob: CobId::sdo_tx(target).raw(),
            target,
        }
    }

    pub fn target(&self) -> NodeId {
        self.target
    }

    /// Read a value from the remote node's object dictionary.
    ///
    /// Returns the number of bytes written into `buf`.
    /// For expedited transfers (≤4 bytes), this completes in one round-trip.
    /// For segmented transfers, multiple frames are exchanged automatically.
    pub async fn upload<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        buf: &mut [u8],
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<usize, SdoError<E>> {
        let mut client = SdoClient::<256>::new(self.target);
        let req = client.start_upload(index, subindex);
        can.transmit(&req).await.map_err(SdoError::Transport)?;

        loop {
            let frame = self.receive_response(can).await?;
            let data: [u8; 8] = frame.data().try_into().unwrap();

            match client.process_response(&data) {
                SdoClientResult::UploadComplete { data_len } => {
                    let src = &client.data()[..data_len];
                    let copy_len = data_len.min(buf.len());
                    buf[..copy_len].copy_from_slice(&src[..copy_len]);
                    return Ok(data_len);
                }
                SdoClientResult::SendNext(next) => {
                    can.transmit(&next).await.map_err(SdoError::Transport)?;
                }
                SdoClientResult::Aborted(code) => return Err(SdoError::Aborted(code)),
                SdoClientResult::DownloadComplete | SdoClientResult::Error => {
                    return Err(SdoError::ProtocolError)
                }
            }
        }
    }

    /// Write a value to the remote node's object dictionary.
    pub async fn download<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        data: &[u8],
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<(), SdoError<E>> {
        let mut client = SdoClient::<256>::new(self.target);
        let req = client.start_download(index, subindex, data);
        can.transmit(&req).await.map_err(SdoError::Transport)?;

        loop {
            let frame = self.receive_response(can).await?;
            let resp: [u8; 8] = frame.data().try_into().unwrap();

            match client.process_response(&resp) {
                SdoClientResult::DownloadComplete => return Ok(()),
                SdoClientResult::SendNext(next) => {
                    can.transmit(&next).await.map_err(SdoError::Transport)?;
                }
                SdoClientResult::Aborted(code) => return Err(SdoError::Aborted(code)),
                SdoClientResult::UploadComplete { .. } | SdoClientResult::Error => {
                    return Err(SdoError::ProtocolError)
                }
            }
        }
    }

    /// Typed read helpers.
    pub async fn read_u8<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<u8, SdoError<E>> {
        let mut buf = [0u8; 1];
        self.upload(index, subindex, &mut buf, can).await?;
        Ok(buf[0])
    }

    pub async fn read_u16<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<u16, SdoError<E>> {
        let mut buf = [0u8; 2];
        self.upload(index, subindex, &mut buf, can).await?;
        Ok(u16::from_le_bytes(buf))
    }

    pub async fn read_u32<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<u32, SdoError<E>> {
        let mut buf = [0u8; 4];
        self.upload(index, subindex, &mut buf, can).await?;
        Ok(u32::from_le_bytes(buf))
    }

    pub async fn read_i32<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<i32, SdoError<E>> {
        let mut buf = [0u8; 4];
        self.upload(index, subindex, &mut buf, can).await?;
        Ok(i32::from_le_bytes(buf))
    }

    pub async fn read_f32<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<f32, SdoError<E>> {
        let mut buf = [0u8; 4];
        self.upload(index, subindex, &mut buf, can).await?;
        Ok(f32::from_le_bytes(buf))
    }

    /// Typed write helpers.
    pub async fn write_u8<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        val: u8,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<(), SdoError<E>> {
        self.download(index, subindex, &[val], can).await
    }

    pub async fn write_u16<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        val: u16,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<(), SdoError<E>> {
        self.download(index, subindex, &val.to_le_bytes(), can).await
    }

    pub async fn write_u32<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        val: u32,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<(), SdoError<E>> {
        self.download(index, subindex, &val.to_le_bytes(), can).await
    }

    pub async fn write_f32<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        val: f32,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<(), SdoError<E>> {
        self.download(index, subindex, &val.to_le_bytes(), can).await
    }

    /// Read a value with a timeout.
    ///
    /// Like [`upload`](Self::upload), but aborts with [`SdoError::Timeout`] if
    /// the transfer doesn't complete within `timeout_us` microseconds.
    ///
    /// ```ignore
    /// let clock = EmbassyClock;
    /// let len = sdo.upload_timed(0x1000, 0, &mut buf, &mut can, 2_000_000, &clock).await?;
    /// ```
    pub async fn upload_timed<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        buf: &mut [u8],
        can: &mut impl AsyncCan<Error = E>,
        timeout_us: u64,
        clock: &impl Clock,
    ) -> Result<usize, SdoError<E>> {
        let deadline = clock.now_us().wrapping_add(timeout_us);
        let mut client = SdoClient::<256>::new(self.target);
        let req = client.start_upload(index, subindex);
        can.transmit(&req).await.map_err(SdoError::Transport)?;

        loop {
            let frame = self.receive_response_timed(can, deadline, clock).await?;
            let data: [u8; 8] = frame.data().try_into().unwrap();

            match client.process_response(&data) {
                SdoClientResult::UploadComplete { data_len } => {
                    let src = &client.data()[..data_len];
                    let copy_len = data_len.min(buf.len());
                    buf[..copy_len].copy_from_slice(&src[..copy_len]);
                    return Ok(data_len);
                }
                SdoClientResult::SendNext(next) => {
                    can.transmit(&next).await.map_err(SdoError::Transport)?;
                }
                SdoClientResult::Aborted(code) => return Err(SdoError::Aborted(code)),
                SdoClientResult::DownloadComplete | SdoClientResult::Error => {
                    return Err(SdoError::ProtocolError)
                }
            }
        }
    }

    /// Write a value with a timeout.
    ///
    /// Like [`download`](Self::download), but aborts with [`SdoError::Timeout`] if
    /// the transfer doesn't complete within `timeout_us` microseconds.
    pub async fn download_timed<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        data: &[u8],
        can: &mut impl AsyncCan<Error = E>,
        timeout_us: u64,
        clock: &impl Clock,
    ) -> Result<(), SdoError<E>> {
        let deadline = clock.now_us().wrapping_add(timeout_us);
        let mut client = SdoClient::<256>::new(self.target);
        let req = client.start_download(index, subindex, data);
        can.transmit(&req).await.map_err(SdoError::Transport)?;

        loop {
            let frame = self.receive_response_timed(can, deadline, clock).await?;
            let resp: [u8; 8] = frame.data().try_into().unwrap();

            match client.process_response(&resp) {
                SdoClientResult::DownloadComplete => return Ok(()),
                SdoClientResult::SendNext(next) => {
                    can.transmit(&next).await.map_err(SdoError::Transport)?;
                }
                SdoClientResult::Aborted(code) => return Err(SdoError::Aborted(code)),
                SdoClientResult::UploadComplete { .. } | SdoClientResult::Error => {
                    return Err(SdoError::ProtocolError)
                }
            }
        }
    }

    /// Receive a frame, filtering for this node's SDO response COB-ID.
    async fn receive_response<E: core::fmt::Debug>(
        &self,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<CanFrame, SdoError<E>> {
        loop {
            let frame = can.receive().await.map_err(SdoError::Transport)?;
            if frame.raw_id() == self.response_cob && frame.raw_dlc() == 8 {
                return Ok(frame);
            }
            // Skip non-SDO frames (heartbeat, PDO, etc.)
        }
    }

    /// Receive with deadline check before each await.
    async fn receive_response_timed<E: core::fmt::Debug>(
        &self,
        can: &mut impl AsyncCan<Error = E>,
        deadline: u64,
        clock: &impl Clock,
    ) -> Result<CanFrame, SdoError<E>> {
        loop {
            if clock.now_us() >= deadline {
                return Err(SdoError::Timeout);
            }
            let frame = can.receive().await.map_err(SdoError::Transport)?;
            if frame.raw_id() == self.response_cob && frame.raw_dlc() == 8 {
                return Ok(frame);
            }
        }
    }
}

/// Wraps an `embedded_can::nb::Can` transport into `AsyncCan` by polling with yield.
///
/// Works with any async runtime. Each `WouldBlock` yields back to the executor.
pub struct NbCanAsync<T>(pub T);

impl<T: embedded_can::nb::Can<Frame = CanFrame>> AsyncCan for NbCanAsync<T>
where
    T::Error: core::fmt::Debug,
{
    type Error = T::Error;

    async fn transmit(&mut self, frame: &CanFrame) -> Result<(), Self::Error> {
        loop {
            match self.0.transmit(frame) {
                Ok(_) => return Ok(()),
                Err(nb::Error::WouldBlock) => {
                    let mut yielded = false;
                    core::future::poll_fn(|cx| {
                        if yielded {
                            core::task::Poll::Ready(())
                        } else {
                            yielded = true;
                            cx.waker().wake_by_ref();
                            core::task::Poll::Pending
                        }
                    })
                    .await;
                }
                Err(nb::Error::Other(e)) => return Err(e),
            }
        }
    }

    async fn receive(&mut self) -> Result<CanFrame, Self::Error> {
        loop {
            match self.0.receive() {
                Ok(frame) => return Ok(frame),
                Err(nb::Error::WouldBlock) => {
                    // Yield once, then retry
                    let mut yielded = false;
                    core::future::poll_fn(|cx| {
                        if yielded {
                            core::task::Poll::Ready(())
                        } else {
                            yielded = true;
                            cx.waker().wake_by_ref();
                            core::task::Poll::Pending
                        }
                    })
                    .await;
                }
                Err(nb::Error::Other(e)) => return Err(e),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nmt::NmtState;
    use crate::od::OdEvent;
    use crate::sdo::server::SdoServer;

    /// In-memory async CAN that wires an SDO client to an SDO server.
    /// Each transmit from the client is immediately processed by the server,
    /// and the response is queued for the next receive.
    struct MockAsyncCan<'a, OD: ObjectDictionary> {
        server: SdoServer,
        od: &'a mut OD,
        pending_response: Option<CanFrame>,
    }

    impl<'a, OD: ObjectDictionary> MockAsyncCan<'a, OD> {
        fn new(od: &'a mut OD) -> Self {
            Self {
                server: SdoServer::new(),
                od,
                pending_response: None,
            }
        }
    }

    #[derive(Debug)]
    struct MockError;

    impl<OD: ObjectDictionary> AsyncCan for MockAsyncCan<'_, OD> {
        type Error = MockError;

        async fn transmit(&mut self, frame: &CanFrame) -> Result<(), MockError> {
            let mut req = [0u8; 8];
            req.copy_from_slice(frame.data());
            let mut resp = [0u8; 8];
            let mut events: heapless::Deque<OdEvent, 16> = heapless::Deque::new();
            if self
                .server
                .process(&req, self.od, &mut resp, &mut events, NmtState::Operational, 0)
                .is_ok()
            {
                let resp_cob = 0x580 + 1; // node 1
                self.pending_response = CanFrame::new(resp_cob, &resp);
            }
            Ok(())
        }

        async fn receive(&mut self) -> Result<CanFrame, MockError> {
            self.pending_response.take().ok_or(MockError)
        }
    }

    // Simple test OD
    use crate::datatypes::DataType;
    use crate::od::*;

    struct TestOd {
        device_type: u32,
        value: u16,
        blob: [u8; 20],
    }

    static DRIVER_TEST_META: &[OdEntryMeta] = &[
        OdEntryMeta {
            index: 0x1000,
            subindex: 0,
            data_type: DataType::U32,
            access: AccessType::Ro,
            pdo_mappable: false,
            name: "device_type",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x2000,
            subindex: 0,
            data_type: DataType::U16,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "value",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x2001,
            subindex: 0,
            data_type: DataType::OctetString,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "blob",
            max_size: None,
        },
    ];

    impl ObjectDictionary for TestOd {
        fn lookup(&self, index: u16, subindex: u8) -> Option<&'static OdEntryMeta> {
            DRIVER_TEST_META
                .iter()
                .find(|e| e.index == index && e.subindex == subindex)
        }
        fn read(&self, index: u16, subindex: u8, buf: &mut [u8]) -> Result<usize, OdError> {
            match (index, subindex) {
                (0x1000, 0) => {
                    buf[..4].copy_from_slice(&self.device_type.to_le_bytes());
                    Ok(4)
                }
                (0x2000, 0) => {
                    buf[..2].copy_from_slice(&self.value.to_le_bytes());
                    Ok(2)
                }
                (0x2001, 0) => {
                    buf[..20].copy_from_slice(&self.blob);
                    Ok(20)
                }
                _ => Err(OdError::NotFound),
            }
        }
        fn write(&mut self, index: u16, subindex: u8, data: &[u8]) -> Result<(), OdError> {
            match (index, subindex) {
                (0x1000, 0) => Err(OdError::ReadOnly),
                (0x2000, 0) => {
                    self.value = u16::from_le_bytes([data[0], data[1]]);
                    Ok(())
                }
                (0x2001, 0) => {
                    self.blob = [0; 20];
                    self.blob[..data.len()].copy_from_slice(data);
                    Ok(())
                }
                _ => Err(OdError::NotFound),
            }
        }
        fn sub_count(&self, _: u16) -> Option<u8> {
            Some(0)
        }
    }

    // Helper to run async test in a minimal executor
    fn block_on<F: core::future::Future>(f: F) -> F::Output {
        use core::pin::pin;
        use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

        fn raw_waker() -> RawWaker {
            fn no_op(_: *const ()) {}
            fn clone(p: *const ()) -> RawWaker {
                RawWaker::new(p, &VTABLE)
            }
            const VTABLE: RawWakerVTable =
                RawWakerVTable::new(clone, no_op, no_op, no_op);
            RawWaker::new(core::ptr::null(), &VTABLE)
        }

        let waker = unsafe { Waker::from_raw(raw_waker()) };
        let mut cx = Context::from_waker(&waker);
        let mut f = pin!(f);

        loop {
            match f.as_mut().poll(&mut cx) {
                Poll::Ready(val) => return val,
                Poll::Pending => {} // spin — fine for tests
            }
        }
    }

    #[test]
    fn driver_expedited_upload() {
        let mut od = TestOd {
            device_type: 0xDEAD_BEEF,
            value: 0,
            blob: [0; 20],
        };
        let target = NodeId::new(1).unwrap();
        let driver = SdoDriver::new(target);
        let mut can = MockAsyncCan::new(&mut od);

        block_on(async {
            let val = driver.read_u32(0x1000, 0, &mut can).await.unwrap();
            assert_eq!(val, 0xDEAD_BEEF);
        });
    }

    #[test]
    fn driver_expedited_download() {
        let mut od = TestOd {
            device_type: 0,
            value: 0,
            blob: [0; 20],
        };
        let target = NodeId::new(1).unwrap();
        let driver = SdoDriver::new(target);
        let mut can = MockAsyncCan::new(&mut od);

        block_on(async {
            driver.write_u16(0x2000, 0, 0xCAFE, &mut can).await.unwrap();
        });
        assert_eq!(od.value, 0xCAFE);
    }

    #[test]
    fn driver_segmented_upload() {
        let mut od = TestOd {
            device_type: 0,
            value: 0,
            blob: [0xBB; 20],
        };
        let target = NodeId::new(1).unwrap();
        let driver = SdoDriver::new(target);
        let mut can = MockAsyncCan::new(&mut od);

        block_on(async {
            let mut buf = [0u8; 32];
            let len = driver.upload(0x2001, 0, &mut buf, &mut can).await.unwrap();
            assert_eq!(len, 20);
            assert_eq!(&buf[..20], &[0xBB; 20]);
        });
    }

    #[test]
    fn driver_segmented_download() {
        let mut od = TestOd {
            device_type: 0,
            value: 0,
            blob: [0; 20],
        };
        let target = NodeId::new(1).unwrap();
        let driver = SdoDriver::new(target);
        let mut can = MockAsyncCan::new(&mut od);

        let data: [u8; 20] = core::array::from_fn(|i| (i + 1) as u8);
        block_on(async {
            driver.download(0x2001, 0, &data, &mut can).await.unwrap();
        });
        assert_eq!(od.blob, data);
    }

    #[test]
    fn driver_abort_on_not_found() {
        let mut od = TestOd {
            device_type: 0,
            value: 0,
            blob: [0; 20],
        };
        let target = NodeId::new(1).unwrap();
        let driver = SdoDriver::new(target);
        let mut can = MockAsyncCan::new(&mut od);

        block_on(async {
            let result = driver.read_u32(0xFFFF, 0, &mut can).await;
            match result {
                Err(SdoError::Aborted(AbortCode::ObjectNotFound)) => {}
                other => panic!("expected ObjectNotFound, got {:?}", other),
            }
        });
    }

    // OD that uses validate_write to reject values > 1000
    struct ValidatingOd {
        value: u16,
    }

    static VALIDATING_META: &[OdEntryMeta] = &[
        OdEntryMeta {
            index: 0x2000, subindex: 0, data_type: DataType::U16,
            access: AccessType::Rw, pdo_mappable: false, name: "value",
            max_size: None,
        },
    ];

    impl ObjectDictionary for ValidatingOd {
        fn lookup(&self, index: u16, subindex: u8) -> Option<&'static OdEntryMeta> {
            VALIDATING_META.iter().find(|e| e.index == index && e.subindex == subindex)
        }
        fn read(&self, index: u16, subindex: u8, buf: &mut [u8]) -> Result<usize, OdError> {
            match (index, subindex) {
                (0x2000, 0) => { buf[..2].copy_from_slice(&self.value.to_le_bytes()); Ok(2) }
                _ => Err(OdError::NotFound),
            }
        }
        fn write(&mut self, index: u16, subindex: u8, data: &[u8]) -> Result<(), OdError> {
            match (index, subindex) {
                (0x2000, 0) => { self.value = u16::from_le_bytes([data[0], data[1]]); Ok(()) }
                _ => Err(OdError::NotFound),
            }
        }
        fn sub_count(&self, _: u16) -> Option<u8> { Some(0) }

        fn validate_write(&self, index: u16, subindex: u8, data: &[u8]) -> Result<(), OdError> {
            if index == 0x2000 && subindex == 0 {
                let val = u16::from_le_bytes([data[0], data[1]]);
                if val > 1000 {
                    return Err(OdError::ValueRange);
                }
            }
            Ok(())
        }
    }

    #[test]
    fn validate_write_accepts_valid_value() {
        let mut od = ValidatingOd { value: 0 };
        let target = NodeId::new(1).unwrap();
        let driver = SdoDriver::new(target);
        let mut can = MockAsyncCan::new(&mut od);

        block_on(async {
            driver.write_u16(0x2000, 0, 500, &mut can).await.unwrap();
        });
        assert_eq!(od.value, 500);
    }

    #[test]
    fn validate_write_rejects_out_of_range() {
        let mut od = ValidatingOd { value: 42 };
        let target = NodeId::new(1).unwrap();
        let driver = SdoDriver::new(target);
        let mut can = MockAsyncCan::new(&mut od);

        block_on(async {
            let result = driver.write_u16(0x2000, 0, 2000, &mut can).await;
            match result {
                Err(SdoError::Aborted(AbortCode::ValueRangeExceeded)) => {}
                other => panic!("expected ValueRangeExceeded, got {:?}", other),
            }
        });
        // Value should NOT have been written
        assert_eq!(od.value, 42);
    }

    // Test clock that advances on each call
    use core::cell::Cell;
    struct TickingClock(Cell<u64>);
    impl TickingClock {
        fn new() -> Self { Self(Cell::new(0)) }
    }
    impl crate::time::Clock for TickingClock {
        fn now_us(&self) -> u64 {
            let v = self.0.get();
            // Advance 100ms per call to simulate time passing
            self.0.set(v + 100_000);
            v
        }
    }

    #[test]
    fn timed_upload_succeeds_within_deadline() {
        let mut od = TestOd {
            device_type: 0xDEAD_BEEF,
            value: 0,
            blob: [0; 20],
        };
        let target = NodeId::new(1).unwrap();
        let driver = SdoDriver::new(target);
        let mut can = MockAsyncCan::new(&mut od);
        let clock = TickingClock::new();

        block_on(async {
            let val = driver
                .upload_timed(0x1000, 0, &mut [0u8; 4], &mut can, 5_000_000, &clock)
                .await
                .unwrap();
            assert_eq!(val, 4);
        });
    }

    /// MockAsyncCan that never produces a response — simulates a dead node.
    struct DeadNodeCan;
    impl AsyncCan for DeadNodeCan {
        type Error = MockError;
        async fn transmit(&mut self, _frame: &CanFrame) -> Result<(), MockError> { Ok(()) }
        async fn receive(&mut self) -> Result<CanFrame, MockError> {
            // Return a non-matching frame (heartbeat) so the driver loops
            // and re-checks the deadline
            Ok(CanFrame::new(0x701, &[0x05]).unwrap())
        }
    }

    #[test]
    fn timed_upload_times_out() {
        let target = NodeId::new(1).unwrap();
        let driver = SdoDriver::new(target);
        let mut can = DeadNodeCan;
        let clock = TickingClock::new();

        block_on(async {
            let result = driver
                .upload_timed(0x1000, 0, &mut [0u8; 4], &mut can, 500_000, &clock)
                .await;
            match result {
                Err(SdoError::Timeout) => {} // expected
                other => panic!("expected Timeout, got {:?}", other),
            }
        });
    }

    #[test]
    fn timed_download_times_out() {
        let target = NodeId::new(1).unwrap();
        let driver = SdoDriver::new(target);
        let mut can = DeadNodeCan;
        let clock = TickingClock::new();

        block_on(async {
            let result = driver
                .download_timed(0x2000, 0, &42u16.to_le_bytes(), &mut can, 500_000, &clock)
                .await;
            match result {
                Err(SdoError::Timeout) => {}
                other => panic!("expected Timeout, got {:?}", other),
            }
        });
    }
}
