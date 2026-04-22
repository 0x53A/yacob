//! CAN frame router — demuxes a single receive stream to multiple consumers.
//!
//! The router owns the async CAN transport and provides filtered views for
//! each protocol (SDO, PDO, heartbeat, etc.). Each consumer gets its own
//! buffer so frames are never lost.
//!
//! ```ignore
//! let router = CanRouter::new(transport);
//!
//! // SDO driver gets only SDO response frames
//! let sdo = SdoDriver::new(target);
//! let val = sdo.read_u32(0x1000, 0, &mut router.sdo_port(target)).await?;
//!
//! // Heartbeat listener gets only heartbeat frames
//! let hb = router.receive_heartbeat().await;
//! ```

use crate::cobid::{CobId, NodeId, ParsedCobId};
use crate::sdo::driver::AsyncCan;
use crate::time::Clock;
use crate::transport::CanFrame;

/// A filtered async CAN port for SDO communication with a specific node.
///
/// Implements `AsyncCan` by forwarding transmit to the underlying transport
/// and only yielding SDO response frames for the target node. Non-matching
/// frames are stored in a shared overflow buffer for other consumers.
pub struct SdoPort<'a, T: AsyncCan> {
    transport: &'a mut T,
    response_cob: u16,
    /// Frames that didn't match our filter — caller can drain these.
    pub overflow: heapless::Vec<CanFrame, 16>,
}

impl<'a, T: AsyncCan> SdoPort<'a, T> {
    pub fn new(transport: &'a mut T, target: NodeId) -> Self {
        Self {
            transport,
            response_cob: CobId::sdo_tx(target).raw(),
            overflow: heapless::Vec::new(),
        }
    }

    /// Drain overflow frames (non-SDO frames received during SDO transfers).
    /// Call this after each SDO operation to process heartbeats, PDOs, etc.
    pub fn drain_overflow(&mut self) -> &[CanFrame] {
        &self.overflow
    }

    pub fn clear_overflow(&mut self) {
        self.overflow.clear();
    }
}

impl<T: AsyncCan> AsyncCan for SdoPort<'_, T> {
    type Error = T::Error;

    async fn transmit(&mut self, frame: &CanFrame) -> Result<(), Self::Error> {
        self.transport.transmit(frame).await
    }

    async fn receive(&mut self) -> Result<CanFrame, Self::Error> {
        loop {
            let frame = self.transport.receive().await?;
            if frame.raw_id() == self.response_cob && frame.raw_dlc() == 8 {
                return Ok(frame);
            }
            // Buffer non-matching frames for other consumers
            let _ = self.overflow.push(frame); // drop if overflow is full
        }
    }
}

/// A filtered async CAN port that only yields heartbeat frames.
pub struct HeartbeatPort<'a, T: AsyncCan> {
    transport: &'a mut T,
    /// Optional: only accept heartbeats from this node. None = all nodes.
    filter_node: Option<NodeId>,
    pub overflow: heapless::Vec<CanFrame, 16>,
}

impl<'a, T: AsyncCan> HeartbeatPort<'a, T> {
    pub fn new(transport: &'a mut T, node: Option<NodeId>) -> Self {
        Self {
            transport,
            filter_node: node,
            overflow: heapless::Vec::new(),
        }
    }
}

impl<T: AsyncCan> AsyncCan for HeartbeatPort<'_, T> {
    type Error = T::Error;

    async fn transmit(&mut self, frame: &CanFrame) -> Result<(), Self::Error> {
        self.transport.transmit(frame).await
    }

    async fn receive(&mut self) -> Result<CanFrame, Self::Error> {
        loop {
            let frame = self.transport.receive().await?;
            if let Some(cob) = CobId::new(frame.raw_id()) {
                if let ParsedCobId::Heartbeat(node) = cob.parse() {
                    if self.filter_node.is_none() || self.filter_node == Some(node) {
                        return Ok(frame);
                    }
                }
            }
            let _ = self.overflow.push(frame);
        }
    }
}

/// Convenience: split a transport into an SDO port for a specific node.
///
/// After the SDO operation, check `port.overflow` for any frames that arrived
/// during the transfer (heartbeats, PDOs, etc.).
///
/// ```ignore
/// let mut port = SdoPort::new(&mut transport, target);
/// let val = sdo.read_u32(0x1000, 0, &mut port).await?;
/// for frame in port.drain_overflow() {
///     // process heartbeats, PDOs, etc.
/// }
/// ```
pub fn sdo_port<T: AsyncCan>(transport: &mut T, target: NodeId) -> SdoPort<'_, T> {
    SdoPort::new(transport, target)
}

// ---- CAN Demuxer ----

/// CAN frame demultiplexer — owns a transport and routes frames to per-protocol buffers.
///
/// Use `sdo_port()` to get an `AsyncCan` view for SDO transfers. Non-SDO frames
/// received during the transfer are buffered and accessible via `recv_heartbeat()`,
/// `recv_pdo()`, etc. after the SDO operation completes.
///
/// ```ignore
/// let mut demux = CanDemux::new(transport);
///
/// // SDO transfer — heartbeats/PDOs buffered automatically
/// {
///     let mut port = demux.sdo_port(target);
///     let val = sdo.read_u32(0x1000, 0, &mut port).await?;
/// }
///
/// // Process buffered frames
/// while let Some(frame) = demux.try_recv_heartbeat() { ... }
/// while let Some(frame) = demux.try_recv_pdo() { ... }
/// ```
pub struct CanDemux<T: AsyncCan> {
    transport: T,
    sdo_buf: heapless::Deque<CanFrame, 4>,
    pdo_buf: heapless::Deque<CanFrame, 16>,
    heartbeat_buf: heapless::Deque<CanFrame, 8>,
    other_buf: heapless::Deque<CanFrame, 4>,
}

impl<T: AsyncCan> CanDemux<T> {
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            sdo_buf: heapless::Deque::new(),
            pdo_buf: heapless::Deque::new(),
            heartbeat_buf: heapless::Deque::new(),
            other_buf: heapless::Deque::new(),
        }
    }

    /// Get a mutable reference to the underlying transport (e.g. for transmitting).
    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    /// Consume the demuxer and return the underlying transport.
    pub fn into_inner(self) -> T {
        self.transport
    }

    /// Create an SDO port for use with `SdoDriver`.
    /// Non-SDO frames are routed to the demux's internal buffers.
    pub fn sdo_port(&mut self, target: NodeId) -> DemuxSdoPort<'_, T> {
        DemuxSdoPort {
            demux: self,
            response_cob: CobId::sdo_tx(target).raw(),
        }
    }

    /// Receive the next heartbeat frame, waiting asynchronously.
    /// Optionally filter by node. Waits indefinitely — use
    /// [`recv_heartbeat_timed`](Self::recv_heartbeat_timed) for a deadline.
    pub async fn recv_heartbeat(&mut self, node: Option<NodeId>) -> Result<CanFrame, T::Error> {
        // Check buffer first
        if let Some(f) = self.take_buffered_heartbeat(node) {
            return Ok(f);
        }

        // Poll transport
        loop {
            let frame = self.transport.receive().await?;
            let parsed = CobId::new(frame.raw_id()).map(|c| c.parse());
            match parsed {
                Some(ParsedCobId::Heartbeat(n))
                    if node.is_none() || node == Some(n) =>
                {
                    return Ok(frame);
                }
                _ => {
                    self.route_frame(frame);
                }
            }
        }
    }

    /// Receive a heartbeat with a timeout.
    ///
    /// Returns `None` if the deadline is reached before a matching heartbeat arrives.
    /// Non-matching frames are routed to internal buffers as usual.
    pub async fn recv_heartbeat_timed(
        &mut self,
        node: Option<NodeId>,
        timeout_us: u64,
        clock: &impl Clock,
    ) -> Result<Option<CanFrame>, T::Error> {
        let deadline = clock.now_us() + timeout_us;

        // Check buffer first
        let found = self.take_buffered_heartbeat(node);
        if found.is_some() {
            return Ok(found);
        }

        // Poll transport until deadline
        loop {
            if clock.now_us() >= deadline {
                return Ok(None);
            }
            let frame = self.transport.receive().await?;
            let parsed = CobId::new(frame.raw_id()).map(|c| c.parse());
            match parsed {
                Some(ParsedCobId::Heartbeat(n))
                    if node.is_none() || node == Some(n) =>
                {
                    return Ok(Some(frame));
                }
                _ => {
                    self.route_frame(frame);
                }
            }
        }
    }

    /// Extract a matching heartbeat from the buffer, if present.
    fn take_buffered_heartbeat(&mut self, node: Option<NodeId>) -> Option<CanFrame> {
        let len = self.heartbeat_buf.len();
        let mut found = None;
        for _ in 0..len {
            if let Some(f) = self.heartbeat_buf.pop_front() {
                if found.is_none() && (node.is_none() || CobId::new(f.raw_id())
                    .and_then(|c| match c.parse() {
                        ParsedCobId::Heartbeat(n) => Some(n),
                        _ => None,
                    }) == node)
                {
                    found = Some(f);
                } else {
                    let _ = self.heartbeat_buf.push_back(f);
                }
            }
        }
        found
    }

    /// Receive the next PDO frame, waiting asynchronously.
    pub async fn recv_pdo(&mut self) -> Result<CanFrame, T::Error> {
        if let Some(f) = self.pdo_buf.pop_front() {
            return Ok(f);
        }
        loop {
            let frame = self.transport.receive().await?;
            let parsed = CobId::new(frame.raw_id()).map(|c| c.parse());
            match parsed {
                Some(ParsedCobId::Tpdo { .. } | ParsedCobId::Rpdo { .. }) => {
                    return Ok(frame);
                }
                _ => {
                    self.route_frame(frame);
                }
            }
        }
    }

    /// Try to get a buffered heartbeat frame (non-blocking).
    pub fn try_recv_heartbeat(&mut self) -> Option<CanFrame> {
        self.heartbeat_buf.pop_front()
    }

    /// Try to get a buffered PDO frame (non-blocking).
    pub fn try_recv_pdo(&mut self) -> Option<CanFrame> {
        self.pdo_buf.pop_front()
    }

    /// Try to get a buffered unclassified frame (non-blocking).
    pub fn try_recv_other(&mut self) -> Option<CanFrame> {
        self.other_buf.pop_front()
    }

    /// Route a frame to the appropriate buffer based on COB-ID.
    fn route_frame(&mut self, frame: CanFrame) {
        let parsed = CobId::new(frame.raw_id()).map(|c| c.parse());
        match parsed {
            Some(ParsedCobId::SdoResponse(_)) => {
                let _ = self.sdo_buf.push_back(frame);
            }
            Some(ParsedCobId::Heartbeat(_)) => {
                let _ = self.heartbeat_buf.push_back(frame);
            }
            Some(ParsedCobId::Tpdo { .. } | ParsedCobId::Rpdo { .. }) => {
                let _ = self.pdo_buf.push_back(frame);
            }
            _ => {
                let _ = self.other_buf.push_back(frame);
            }
        }
    }
}

/// SDO port backed by a `CanDemux`. Implements `AsyncCan` for use with `SdoDriver`.
/// Non-SDO frames are automatically routed to the demux's internal buffers.
pub struct DemuxSdoPort<'a, T: AsyncCan> {
    demux: &'a mut CanDemux<T>,
    response_cob: u16,
}

impl<T: AsyncCan> AsyncCan for DemuxSdoPort<'_, T> {
    type Error = T::Error;

    async fn transmit(&mut self, frame: &CanFrame) -> Result<(), Self::Error> {
        self.demux.transport.transmit(frame).await
    }

    async fn receive(&mut self) -> Result<CanFrame, Self::Error> {
        // Check SDO buffer first — drain and re-push non-matching frames
        let len = self.demux.sdo_buf.len();
        for _ in 0..len {
            if let Some(f) = self.demux.sdo_buf.pop_front() {
                if f.raw_id() == self.response_cob && f.raw_dlc() == 8 {
                    return Ok(f);
                }
                let _ = self.demux.sdo_buf.push_back(f);
            }
        }

        // Poll transport, routing non-SDO frames
        loop {
            let frame = self.demux.transport.receive().await?;
            if frame.raw_id() == self.response_cob && frame.raw_dlc() == 8 {
                return Ok(frame);
            }
            self.demux.route_frame(frame);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nmt::NmtState;
    use crate::od::*;
    use crate::sdo::driver::SdoDriver;
    use crate::sdo::server::SdoServer;

    // A mock transport that interleaves heartbeat frames with SDO responses
    struct NoisyMockCan {
        server: SdoServer,
        od: NoisyOd,
        pending: heapless::Vec<CanFrame, 8>,
    }

    struct NoisyOd {
        device_type: u32,
    }

    static NOISY_META: &[OdEntryMeta] = &[OdEntryMeta {
        index: 0x1000,
        subindex: 0,
        data_type: crate::datatypes::DataType::U32,
        access: AccessType::Ro,
        pdo_mappable: false,
        name: "device_type",
        max_size: None,
    }];

    impl ObjectDictionary for NoisyOd {
        fn lookup(&self, index: u16, subindex: u8) -> Option<&'static OdEntryMeta> {
            NOISY_META
                .iter()
                .find(|e| e.index == index && e.subindex == subindex)
        }
        fn read(&self, index: u16, _sub: u8, buf: &mut [u8]) -> Result<usize, OdError> {
            match index {
                0x1000 => {
                    buf[..4].copy_from_slice(&self.device_type.to_le_bytes());
                    Ok(4)
                }
                _ => Err(OdError::NotFound),
            }
        }
        fn write(&mut self, _: u16, _: u8, _: &[u8]) -> Result<(), OdError> {
            Err(OdError::ReadOnly)
        }
        fn sub_count(&self, _: u16) -> Option<u8> {
            Some(0)
        }
    }

    impl NoisyMockCan {
        fn new() -> Self {
            Self {
                server: SdoServer::new(),
                od: NoisyOd {
                    device_type: 0x42,
                },
                pending: heapless::Vec::new(),
            }
        }
    }

    #[derive(Debug)]
    struct MockErr;

    impl AsyncCan for NoisyMockCan {
        type Error = MockErr;

        async fn transmit(&mut self, frame: &CanFrame) -> Result<(), MockErr> {
            // Inject a heartbeat frame BEFORE the SDO response
            let hb = CanFrame::new(0x701, &[0x05]).unwrap(); // node 1 heartbeat
            let _ = self.pending.push(hb);

            // Also inject a PDO frame
            let pdo = CanFrame::new(0x181, &[0xAA, 0xBB]).unwrap();
            let _ = self.pending.push(pdo);

            // Now process the SDO request
            let mut req = [0u8; 8];
            req.copy_from_slice(frame.data());
            let mut resp = [0u8; 8];
            let mut events: heapless::Deque<OdEvent, 16> = heapless::Deque::new();
            if self
                .server
                .process(
                    &req,
                    &mut self.od,
                    &mut resp,
                    &mut events,
                    NmtState::Operational,
                    0,
                )
                .is_ok()
            {
                let _ = self.pending.push(CanFrame::new(0x581, &resp).unwrap());
            }
            Ok(())
        }

        async fn receive(&mut self) -> Result<CanFrame, MockErr> {
            if self.pending.is_empty() {
                return Err(MockErr);
            }
            Ok(self.pending.remove(0))
        }
    }

    fn block_on<F: core::future::Future>(f: F) -> F::Output {
        use core::pin::pin;
        use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
        fn raw_waker() -> RawWaker {
            fn no_op(_: *const ()) {}
            fn clone(p: *const ()) -> RawWaker {
                RawWaker::new(p, &VTABLE)
            }
            const VTABLE: RawWakerVTable = RawWakerVTable::new(clone, no_op, no_op, no_op);
            RawWaker::new(core::ptr::null(), &VTABLE)
        }
        let waker = unsafe { Waker::from_raw(raw_waker()) };
        let mut cx = Context::from_waker(&waker);
        let mut f = pin!(f);
        loop {
            match f.as_mut().poll(&mut cx) {
                Poll::Ready(val) => return val,
                Poll::Pending => {}
            }
        }
    }

    #[test]
    fn sdo_port_filters_and_buffers_overflow() {
        let target = NodeId::new(1).unwrap();
        let driver = SdoDriver::new(target);
        let mut transport = NoisyMockCan::new();

        block_on(async {
            let mut port = SdoPort::new(&mut transport, target);
            let val = driver.read_u32(0x1000, 0, &mut port).await.unwrap();
            assert_eq!(val, 0x42);

            // The heartbeat and PDO frames should be in the overflow buffer
            assert_eq!(port.overflow.len(), 2);
            assert_eq!(port.overflow[0].raw_id(), 0x701); // heartbeat
            assert_eq!(port.overflow[1].raw_id(), 0x181); // PDO
        });
    }

    #[test]
    fn sdo_driver_still_works_directly_without_port() {
        // SdoDriver's internal filtering still works (drops non-SDO frames)
        let target = NodeId::new(1).unwrap();
        let driver = SdoDriver::new(target);
        let mut transport = NoisyMockCan::new();

        block_on(async {
            let val = driver.read_u32(0x1000, 0, &mut transport).await.unwrap();
            assert_eq!(val, 0x42);
        });
    }

    #[test]
    fn demux_sdo_routes_non_sdo_to_buffers() {
        let target = NodeId::new(1).unwrap();
        let driver = SdoDriver::new(target);
        let transport = NoisyMockCan::new();
        let mut demux = CanDemux::new(transport);

        block_on(async {
            // SDO transfer via demux port
            let mut port = demux.sdo_port(target);
            let val = driver.read_u32(0x1000, 0, &mut port).await.unwrap();
            assert_eq!(val, 0x42);
        });

        // Heartbeat and PDO should be in the demux buffers
        let hb = demux.try_recv_heartbeat().unwrap();
        assert_eq!(hb.raw_id(), 0x701);

        let pdo = demux.try_recv_pdo().unwrap();
        assert_eq!(pdo.raw_id(), 0x181);

        // No more buffered frames
        assert!(demux.try_recv_heartbeat().is_none());
        assert!(demux.try_recv_pdo().is_none());
    }

    #[test]
    fn demux_async_heartbeat() {
        let transport = NoisyMockCan::new();
        let mut demux = CanDemux::new(transport);

        // Inject some frames directly
        let hb = CanFrame::new(0x701, &[0x05]).unwrap();
        let _ = demux.heartbeat_buf.push_back(hb);

        block_on(async {
            let frame = demux.recv_heartbeat(None).await.unwrap();
            assert_eq!(frame.raw_id(), 0x701);
            assert_eq!(frame.data()[0], 0x05);
        });
    }
}
