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
use crate::sdo::protocol::{encode_abort, AbortCode};
use crate::sdo::SdoAbort;
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
    Aborted(SdoAbort),
    /// Protocol error (unexpected response, toggle mismatch, etc.)
    ProtocolError,
    /// CAN transport error.
    Transport(E),
    /// Transfer deadline expired during transmission or reception.
    Timeout,
    /// Transfer data does not fit the client or caller's buffer.
    TooLarge,
}

impl<E: core::fmt::Debug> core::fmt::Display for SdoError<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Aborted(code) => write!(f, "SDO aborted: {code}"),
            Self::ProtocolError => write!(f, "SDO protocol error"),
            Self::Transport(e) => write!(f, "CAN transport error: {:?}", e),
            Self::Timeout => write!(f, "SDO timeout"),
            Self::TooLarge => write!(f, "SDO transfer data exceeds buffer capacity"),
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

/// One transfer budget, shared across every TX and RX wait. Compare elapsed
/// time rather than an absolute end instant so an arbitrary clock epoch can
/// cross u64::MAX without making a new transfer immediately time out.
#[derive(Clone, Copy)]
struct TransferDeadline {
    started: u64,
    timeout_us: u64,
}

impl TransferDeadline {
    fn remaining(self, now: u64) -> u64 {
        self.timeout_us
            .saturating_sub(now.wrapping_sub(self.started))
    }
}

/// Deadline state of one transfer: `None` waits indefinitely.
type Deadline<'c, C> = Option<(TransferDeadline, &'c C)>;

/// Clock stand-in for untimed transfers; never consulted.
struct NoClock;

impl Clock for NoClock {
    fn now_us(&self) -> u64 {
        0
    }
}

/// Generate a typed read/write helper pair for a fixed-width type.
macro_rules! typed_access {
    ($read:ident, $write:ident, $ty:ty, $width:literal, $doc:literal) => {
        #[doc = concat!("Read ", $doc, " (exact ", stringify!($width), "-byte wire width).")]
        pub async fn $read<E: core::fmt::Debug>(
            &self,
            index: u16,
            subindex: u8,
            can: &mut impl AsyncCan<Error = E>,
        ) -> Result<$ty, SdoError<E>> {
            let buf = self.read_fixed::<$width, E>(index, subindex, can).await?;
            Ok(<$ty>::from_le_bytes(buf))
        }

        #[doc = concat!("Write ", $doc, ".")]
        pub async fn $write<E: core::fmt::Debug>(
            &self,
            index: u16,
            subindex: u8,
            val: $ty,
            can: &mut impl AsyncCan<Error = E>,
        ) -> Result<(), SdoError<E>> {
            self.download(index, subindex, &val.to_le_bytes(), can).await
        }
    };
}

impl SdoDriver {
    /// Largest object this driver can upload or download in one transfer
    /// (the size of its internal [`SdoClient`] buffer).
    pub const MAX_TRANSFER: usize = 256;

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
    ///
    /// When the transfer fails while the server may still be inside it
    /// (protocol error, timeout, client buffer exceeded), the driver sends a
    /// best-effort SDO abort so the server channel is free for a retry.
    pub async fn upload<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        buf: &mut [u8],
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<usize, SdoError<E>> {
        self.upload_impl(index, subindex, buf, can, None::<(TransferDeadline, &NoClock)>)
            .await
            .map(|(len, _)| len)
    }

    /// Write a value to the remote node's object dictionary.
    ///
    /// Sends a best-effort abort on failure, like [`upload`](Self::upload).
    pub async fn download<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        data: &[u8],
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<(), SdoError<E>> {
        self.download_impl(index, subindex, data, can, None::<(TransferDeadline, &NoClock)>)
            .await
    }

    /// Read a value with a timeout.
    ///
    /// Like [`upload`](Self::upload), but aborts with [`SdoError::Timeout`] if
    /// the transfer doesn't complete within `timeout_us` microseconds.
    ///
    /// On a completely silent bus the timeout fires via an armed timer when
    /// the `embassy` feature is enabled; without it, the executor must re-poll
    /// periodically (as the canopen-linux blocking helpers do).
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
        let deadline = TransferDeadline {
            started: clock.now_us(),
            timeout_us,
        };
        self.upload_impl(index, subindex, buf, can, Some((deadline, clock)))
            .await
            .map(|(len, _)| len)
    }

    /// Write a value with a timeout.
    ///
    /// Like [`download`](Self::download), but aborts with [`SdoError::Timeout`] if
    /// the transfer doesn't complete within `timeout_us` microseconds. See
    /// [`upload_timed`](Self::upload_timed) for the silent-bus timeout
    /// requirements.
    pub async fn download_timed<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        data: &[u8],
        can: &mut impl AsyncCan<Error = E>,
        timeout_us: u64,
        clock: &impl Clock,
    ) -> Result<(), SdoError<E>> {
        let deadline = TransferDeadline {
            started: clock.now_us(),
            timeout_us,
        };
        self.download_impl(index, subindex, data, can, Some((deadline, clock)))
            .await
    }

    /// Upload, also reporting whether the server indicated the size (see
    /// [`SdoClient::upload_size_indicated`]).
    async fn upload_impl<E: core::fmt::Debug, C: Clock>(
        &self,
        index: u16,
        subindex: u8,
        buf: &mut [u8],
        can: &mut impl AsyncCan<Error = E>,
        deadline: Deadline<'_, C>,
    ) -> Result<(usize, bool), SdoError<E>> {
        let mut client = SdoClient::<{ Self::MAX_TRANSFER }>::new(self.target);
        let req = client.start_upload(index, subindex);
        self.send(can, &req, deadline).await?;

        let result = async {
            loop {
                let frame = self.receive_response(can, deadline).await?;
                let data: [u8; 8] = frame.data().try_into().unwrap();
                match client.process_response(&data) {
                    SdoClientResult::UploadComplete { data_len } => {
                        // The server has finished; nothing to abort.
                        if data_len > buf.len() {
                            return Ok(Err(SdoError::TooLarge));
                        }
                        buf[..data_len].copy_from_slice(&client.data()[..data_len]);
                        return Ok(Ok((data_len, client.upload_size_indicated())));
                    }
                    SdoClientResult::SendNext(next) => self.send(can, &next, deadline).await?,
                    SdoClientResult::Aborted(code) => return Ok(Err(SdoError::Aborted(code))),
                    SdoClientResult::IgnoredAbort => {}
                    SdoClientResult::DownloadComplete | SdoClientResult::Error => {
                        return Err(SdoError::ProtocolError)
                    }
                }
            }
        }
        .await;
        self.finish(can, index, subindex, result).await
    }

    async fn download_impl<E: core::fmt::Debug, C: Clock>(
        &self,
        index: u16,
        subindex: u8,
        data: &[u8],
        can: &mut impl AsyncCan<Error = E>,
        deadline: Deadline<'_, C>,
    ) -> Result<(), SdoError<E>> {
        let mut client = SdoClient::<{ Self::MAX_TRANSFER }>::new(self.target);
        let req = client
            .start_download(index, subindex, data)
            .map_err(|()| SdoError::TooLarge)?;
        self.send(can, &req, deadline).await?;

        let result = async {
            loop {
                let frame = self.receive_response(can, deadline).await?;
                let resp: [u8; 8] = frame.data().try_into().unwrap();
                match client.process_response(&resp) {
                    SdoClientResult::DownloadComplete => return Ok(Ok(())),
                    SdoClientResult::SendNext(next) => self.send(can, &next, deadline).await?,
                    SdoClientResult::Aborted(code) => return Ok(Err(SdoError::Aborted(code))),
                    SdoClientResult::IgnoredAbort => {}
                    SdoClientResult::UploadComplete { .. } | SdoClientResult::Error => {
                        return Err(SdoError::ProtocolError)
                    }
                }
            }
        }
        .await;
        self.finish(can, index, subindex, result).await
    }

    /// Resolve a transfer outcome. `Err` means the transfer broke off while
    /// the server may still hold it open (`Ok(Err(_))` are terminal outcomes
    /// the server already knows about): send a best-effort abort first.
    async fn finish<T, E: core::fmt::Debug>(
        &self,
        can: &mut impl AsyncCan<Error = E>,
        index: u16,
        subindex: u8,
        result: Result<Result<T, SdoError<E>>, SdoError<E>>,
    ) -> Result<T, SdoError<E>> {
        match result {
            Ok(outcome) => outcome,
            Err(error) => {
                let code = match error {
                    SdoError::Timeout => Some(AbortCode::SdoProtocolTimeout),
                    SdoError::ProtocolError => Some(AbortCode::InvalidCommandSpecifier),
                    // A transport failure leaves nothing to send the abort on.
                    _ => None,
                };
                if let Some(code) = code {
                    self.send_abort(can, index, subindex, code).await;
                }
                Err(error)
            }
        }
    }

    /// Offer an abort to the transport without waiting for TX space: the
    /// transfer has already failed, and a stuck transport must not turn that
    /// failure into a hang.
    async fn send_abort<E: core::fmt::Debug>(
        &self,
        can: &mut impl AsyncCan<Error = E>,
        index: u16,
        subindex: u8,
        code: AbortCode,
    ) {
        use core::future::Future;
        use core::task::Poll;
        let Some(frame) = CanFrame::new(
            CobId::sdo_rx(self.target).raw(),
            &encode_abort(index, subindex, code),
        ) else {
            return;
        };
        let mut transmit = core::pin::pin!(can.transmit(&frame));
        core::future::poll_fn(|cx| {
            let _ = transmit.as_mut().poll(cx);
            Poll::Ready(())
        })
        .await;
    }

    /// Read exactly the wire width of a fixed-size type.
    async fn read_fixed<const N: usize, E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<[u8; N], SdoError<E>> {
        // Room for a size-less expedited reply (always four bytes).
        let mut buf = [0u8; 8];
        let (len, size_indicated) = self
            .upload_impl(index, subindex, &mut buf, can, None::<(TransferDeadline, &NoClock)>)
            .await?;
        // A size-less expedited reply carries four bytes of which the object
        // occupies the leading N (CiA 301); otherwise the width must match.
        let width_ok = len == N || (!size_indicated && len == 4 && N < 4);
        if !width_ok {
            return Err(SdoError::ProtocolError);
        }
        let mut out = [0; N];
        out.copy_from_slice(&buf[..N]);
        Ok(out)
    }

    typed_access!(read_u8, write_u8, u8, 1, "an UNSIGNED8");
    typed_access!(read_i8, write_i8, i8, 1, "an INTEGER8");
    typed_access!(read_u16, write_u16, u16, 2, "an UNSIGNED16");
    typed_access!(read_i16, write_i16, i16, 2, "an INTEGER16");
    typed_access!(read_u32, write_u32, u32, 4, "an UNSIGNED32");
    typed_access!(read_i32, write_i32, i32, 4, "an INTEGER32");
    typed_access!(read_f32, write_f32, f32, 4, "a REAL32");
    typed_access!(read_u64, write_u64, u64, 8, "an UNSIGNED64");
    typed_access!(read_i64, write_i64, i64, 8, "an INTEGER64");
    typed_access!(read_f64, write_f64, f64, 8, "a REAL64");

    /// Read a BOOLEAN (one byte; any nonzero value is `true`).
    pub async fn read_bool<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<bool, SdoError<E>> {
        Ok(self.read_fixed::<1, E>(index, subindex, can).await?[0] != 0)
    }

    /// Write a BOOLEAN as one byte (0 or 1).
    pub async fn write_bool<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        val: bool,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<(), SdoError<E>> {
        self.download(index, subindex, &[val as u8], can).await
    }

    /// Read an INTEGER24 (3 bytes on the wire), sign-extended into `i32`.
    pub async fn read_i24<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<i32, SdoError<E>> {
        let buf = self.read_fixed::<3, E>(index, subindex, can).await?;
        let ext = if (buf[2] & 0x80) != 0 { 0xFF } else { 0x00 };
        Ok(i32::from_le_bytes([buf[0], buf[1], buf[2], ext]))
    }

    /// Read an UNSIGNED24 (3 bytes on the wire) into `u32`.
    pub async fn read_u24<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<u32, SdoError<E>> {
        let buf = self.read_fixed::<3, E>(index, subindex, can).await?;
        Ok(u32::from_le_bytes([buf[0], buf[1], buf[2], 0]))
    }

    /// Write an INTEGER24 (3 bytes on the wire). Values outside the 24-bit
    /// range are truncated.
    pub async fn write_i24<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        val: i32,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<(), SdoError<E>> {
        self.download(index, subindex, &val.to_le_bytes()[..3], can)
            .await
    }

    /// Write an UNSIGNED24 (3 bytes on the wire). Values outside the 24-bit
    /// range are truncated.
    pub async fn write_u24<E: core::fmt::Debug>(
        &self,
        index: u16,
        subindex: u8,
        val: u32,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<(), SdoError<E>> {
        self.download(index, subindex, &val.to_le_bytes()[..3], can)
            .await
    }

    /// Transmit, bounded by the transfer deadline when there is one.
    async fn send<E: core::fmt::Debug, C: Clock>(
        &self,
        can: &mut impl AsyncCan<Error = E>,
        frame: &CanFrame,
        deadline: Deadline<'_, C>,
    ) -> Result<(), SdoError<E>> {
        let Some((deadline, clock)) = deadline else {
            return can.transmit(frame).await.map_err(SdoError::Transport);
        };
        let transmit = can.transmit(frame);
        with_deadline(transmit, deadline, clock)
            .await?
            .map_err(SdoError::Transport)
    }

    /// Receive the next frame on this node's SDO response COB-ID. A frame on
    /// that COB-ID must be a full 8-byte SDO frame.
    async fn receive_response<E: core::fmt::Debug, C: Clock>(
        &self,
        can: &mut impl AsyncCan<Error = E>,
        deadline: Deadline<'_, C>,
    ) -> Result<CanFrame, SdoError<E>> {
        loop {
            let frame = match deadline {
                None => can.receive().await,
                Some((deadline, clock)) => with_deadline(can.receive(), deadline, clock).await?,
            }
            .map_err(SdoError::Transport)?;
            if frame.raw_id() != self.response_cob {
                continue;
            }
            if frame.raw_dlc() != 8 {
                return Err(SdoError::ProtocolError);
            }
            return Ok(frame);
        }
    }
}

/// Run `fut` until it completes or the transfer deadline passes.
///
/// The `clock` deadline is checked on every poll, which is sufficient for
/// executors that re-poll periodically (like the canopen-linux blocking
/// helpers). With the `embassy` feature an `embassy_time::Timer` is armed as
/// well, so the timeout also fires on wake-driven executors when the bus is
/// completely silent.
async fn with_deadline<F: core::future::Future, E: core::fmt::Debug>(
    fut: F,
    deadline: TransferDeadline,
    clock: &impl Clock,
) -> Result<F::Output, SdoError<E>> {
    #[cfg(feature = "embassy")]
    use core::future::Future as _;
    use core::task::Poll;

    let remaining = deadline.remaining(clock.now_us());
    if remaining == 0 {
        return Err(SdoError::Timeout);
    }
    #[cfg(feature = "embassy")]
    let mut timer = core::pin::pin!(embassy_time::Timer::after(
        embassy_time::Duration::from_micros(remaining)
    ));
    #[cfg(not(feature = "embassy"))]
    let _ = remaining;
    let mut fut = core::pin::pin!(fut);
    core::future::poll_fn(|cx| {
        if deadline.remaining(clock.now_us()) == 0 {
            return Poll::Ready(Err(SdoError::Timeout));
        }
        if let Poll::Ready(output) = fut.as_mut().poll(cx) {
            return Poll::Ready(Ok(output));
        }
        #[cfg(feature = "embassy")]
        if timer.as_mut().poll(cx).is_ready() {
            return Poll::Ready(Err(SdoError::Timeout));
        }
        Poll::Pending
    })
    .await
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

    struct NeverRespondsCan;

    impl AsyncCan for NeverRespondsCan {
        type Error = MockError;

        async fn transmit(&mut self, _frame: &CanFrame) -> Result<(), MockError> {
            Ok(())
        }

        async fn receive(&mut self) -> Result<CanFrame, MockError> {
            core::future::pending().await
        }
    }

    struct MalformedResponseCan {
        response: CanFrame,
    }

    impl MalformedResponseCan {
        fn with_dlc(dlc: usize) -> Self {
            Self {
                response: CanFrame::new(0x581, &[0; 8][..dlc]).unwrap(),
            }
        }
    }

    impl AsyncCan for MalformedResponseCan {
        type Error = MockError;

        async fn transmit(&mut self, _frame: &CanFrame) -> Result<(), MockError> {
            Ok(())
        }

        async fn receive(&mut self) -> Result<CanFrame, MockError> {
            Ok(self.response)
        }
    }

    struct CellClock(core::cell::Cell<u64>);

    impl Clock for CellClock {
        fn now_us(&self) -> u64 {
            self.0.get()
        }
    }

    impl<OD: ObjectDictionary> AsyncCan for MockAsyncCan<'_, OD> {
        type Error = MockError;

        async fn transmit(&mut self, frame: &CanFrame) -> Result<(), MockError> {
            let mut req = [0u8; 8];
            req.copy_from_slice(frame.data());
            let mut resp = [0u8; 8];
            let mut events: heapless::Deque<OdEvent, 16> = heapless::Deque::new();
            if self
                .server
                .process(
                    &req,
                    self.od,
                    &mut resp,
                    &mut events,
                    NmtState::Operational,
                    0,
                )
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

    /// No-op waker for manual polling in tests.
    fn noop_waker() -> core::task::Waker {
        use core::task::{RawWaker, RawWakerVTable, Waker};

        fn no_op(_: *const ()) {}
        fn clone(p: *const ()) -> RawWaker {
            RawWaker::new(p, &VTABLE)
        }
        const VTABLE: RawWakerVTable = RawWakerVTable::new(clone, no_op, no_op, no_op);
        unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) }
    }

    // Helper to run async test in a minimal executor
    fn block_on<F: core::future::Future>(f: F) -> F::Output {
        use core::pin::pin;
        use core::task::{Context, Poll};

        let waker = noop_waker();
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
    fn malformed_matching_response_dlc_is_a_protocol_error() {
        let target = NodeId::new(1).unwrap();
        let driver = SdoDriver::new(target);

        for dlc in 0..8 {
            let mut upload_can = MalformedResponseCan::with_dlc(dlc);
            let upload = block_on(driver.upload(0x1000, 0, &mut [0; 4], &mut upload_can));
            assert!(
                matches!(upload, Err(SdoError::ProtocolError)),
                "upload DLC {dlc}"
            );

            let mut download_can = MalformedResponseCan::with_dlc(dlc);
            let download = block_on(driver.download(0x2000, 0, &[1], &mut download_can));
            assert!(
                matches!(download, Err(SdoError::ProtocolError)),
                "download DLC {dlc}"
            );

            let clock = CellClock(core::cell::Cell::new(0));
            let mut timed_upload_can = MalformedResponseCan::with_dlc(dlc);
            let timed_upload = block_on(driver.upload_timed(
                0x1000,
                0,
                &mut [0; 4],
                &mut timed_upload_can,
                1_000_000,
                &clock,
            ));
            assert!(
                matches!(timed_upload, Err(SdoError::ProtocolError)),
                "timed upload DLC {dlc}"
            );

            let mut timed_download_can = MalformedResponseCan::with_dlc(dlc);
            let timed_download = block_on(driver.download_timed(
                0x2000,
                0,
                &[1],
                &mut timed_download_can,
                1_000_000,
                &clock,
            ));
            assert!(
                matches!(timed_download, Err(SdoError::ProtocolError)),
                "timed download DLC {dlc}"
            );
        }
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
                Err(SdoError::Aborted(code))
                    if code.known() == Some(crate::sdo::AbortCode::ObjectNotFound) => {}
                other => panic!("expected ObjectNotFound, got {:?}", other),
            }
        });
    }

    // OD that uses validate_write to reject values > 1000
    struct ValidatingOd {
        value: u16,
    }

    static VALIDATING_META: &[OdEntryMeta] = &[OdEntryMeta {
        index: 0x2000,
        subindex: 0,
        data_type: DataType::U16,
        access: AccessType::Rw,
        pdo_mappable: false,
        name: "value",
        max_size: None,
    }];

    impl ObjectDictionary for ValidatingOd {
        fn lookup(&self, index: u16, subindex: u8) -> Option<&'static OdEntryMeta> {
            VALIDATING_META
                .iter()
                .find(|e| e.index == index && e.subindex == subindex)
        }
        fn read(&self, index: u16, subindex: u8, buf: &mut [u8]) -> Result<usize, OdError> {
            match (index, subindex) {
                (0x2000, 0) => {
                    buf[..2].copy_from_slice(&self.value.to_le_bytes());
                    Ok(2)
                }
                _ => Err(OdError::NotFound),
            }
        }
        fn write(&mut self, index: u16, subindex: u8, data: &[u8]) -> Result<(), OdError> {
            match (index, subindex) {
                (0x2000, 0) => {
                    self.value = u16::from_le_bytes([data[0], data[1]]);
                    Ok(())
                }
                _ => Err(OdError::NotFound),
            }
        }
        fn sub_count(&self, _: u16) -> Option<u8> {
            Some(0)
        }

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
                Err(SdoError::Aborted(code))
                    if code.known() == Some(crate::sdo::AbortCode::ValueRangeExceeded) => {}
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
        fn new() -> Self {
            Self(Cell::new(0))
        }
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
        async fn transmit(&mut self, _frame: &CanFrame) -> Result<(), MockError> {
            Ok(())
        }
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

    #[test]
    fn timed_upload_times_out_while_receive_is_pending() {
        use core::future::Future;
        use core::pin::pin;
        use core::task::{Context, Poll};

        let target = NodeId::new(1).unwrap();
        let driver = SdoDriver::new(target);
        let mut can = NeverRespondsCan;
        let clock = CellClock(core::cell::Cell::new(0));
        let mut buf = [0u8; 4];
        let mut fut = pin!(driver.upload_timed(0x1000, 0, &mut buf, &mut can, 100, &clock));
        let waker = noop_waker();
        let mut cx = Context::from_waker(&waker);

        assert!(matches!(fut.as_mut().poll(&mut cx), Poll::Pending));
        clock.0.set(101);

        assert!(matches!(
            fut.as_mut().poll(&mut cx),
            Poll::Ready(Err(SdoError::Timeout))
        ));
    }

    #[test]
    fn download_larger_than_client_buffer_returns_too_large() {
        let target = NodeId::new(1).unwrap();
        let driver = SdoDriver::new(target);
        // Driver's internal SdoClient buffer is 256 bytes.
        let data = [0x55u8; 257];

        // Fails before any CAN interaction — NeverRespondsCan would hang otherwise.
        let mut can = NeverRespondsCan;
        let result = block_on(driver.download(0x2001, 0, &data, &mut can));
        assert!(matches!(result, Err(SdoError::TooLarge)));
    }
}
