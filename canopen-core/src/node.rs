use crate::cobid::{CobId, NodeId, ParsedCobId};
use crate::emcy::EmcyProducer;
use crate::heartbeat::HeartbeatProducer;
use crate::lss::{LssIdentity, LssSlave};
use crate::nmt::{NmtCommand, NmtHandler, NmtState, NmtTransition};
use crate::od::ObjectDictionary;
use crate::od::OdEvent;
#[cfg(feature = "embassy")]
use crate::od::OdEventSignal;
use crate::pdo::{PdoMapping, RpdoConfig, RpdoEngine, TpdoConfig, TpdoEngine};
use crate::sdo::SdoServer;
use crate::time::Clock;
use crate::transport::CanFrame;
use heapless::{Deque, Vec};

/// Type of reset requested via [`Node::request_reset`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResetType {
    /// Full reset: re-initialize all application and communication parameters.
    Application,
    /// Communication-only reset: re-initialize PDO config, heartbeat, etc.
    /// Application data in the OD is preserved.
    Communication,
}

/// Configuration for creating a Node.
pub struct NodeConfig<const TPDO: usize = 4, const RPDO: usize = 4> {
    pub node_id: NodeId,
    pub heartbeat_interval_ms: u16,
    /// If true, the node transitions directly to Operational after boot,
    /// without waiting for an NMT Start command from a master.
    pub auto_start: bool,
    pub tpdo: [TpdoConfig; TPDO],
    pub rpdo: [RpdoConfig; RPDO],
    /// LSS identity (0x1018). Set to default if LSS is not needed.
    pub identity: LssIdentity,
}

impl<const TPDO: usize, const RPDO: usize> NodeConfig<TPDO, RPDO> {
    /// Build a config with PDO settings pulled from the OD (declared in the
    /// `object_dictionary!` macro) and defaults for the rest: heartbeat every
    /// 1000 ms, no auto-start, default LSS identity.
    ///
    /// Override the defaults with struct update syntax:
    /// ```ignore
    /// let config = NodeConfig {
    ///     heartbeat_interval_ms: 500,
    ///     auto_start: true,
    ///     ..NodeConfig::from_od(&od, node_id)
    /// };
    /// ```
    pub fn from_od(od: &impl crate::pdo::PdoConfigSource<TPDO, RPDO>, node_id: NodeId) -> Self {
        Self {
            node_id,
            heartbeat_interval_ms: 1000,
            auto_start: false,
            tpdo: od.tpdo_configs(node_id),
            rpdo: od.rpdo_configs(node_id),
            identity: LssIdentity::default(),
        }
    }
}

/// A CANopen node. Ties together NMT, SDO server, PDO engines, and heartbeat.
///
/// Generic over the object dictionary type, PDO counts, event queue size, and dirty set size.
pub struct Node<
    OD: ObjectDictionary,
    const TPDO: usize = 4,
    const RPDO: usize = 4,
    const EVT_QUEUE: usize = 16,
    const DIRTY_SET: usize = 8,
> {
    node_id: NodeId,
    od: OD,
    nmt: NmtHandler,
    sdo_server: SdoServer,
    tpdo: TpdoEngine<TPDO>,
    rpdo: RpdoEngine<RPDO>,
    heartbeat: HeartbeatProducer,
    emcy: EmcyProducer,
    lss: LssSlave,
    booted: bool,
    auto_start: bool,
    event_queue: Deque<OdEvent, EVT_QUEUE>,
    dirty_set: Vec<(u16, u8), DIRTY_SET>,
    event_overflow_count: u32,
    #[cfg(feature = "embassy")]
    event_signal: Option<&'static OdEventSignal>,
}

impl<
        OD: ObjectDictionary,
        const TPDO: usize,
        const RPDO: usize,
        const EVT_QUEUE: usize,
        const DIRTY_SET: usize,
    > Node<OD, TPDO, RPDO, EVT_QUEUE, DIRTY_SET>
{
    pub fn new(config: NodeConfig<TPDO, RPDO>, od: OD) -> Self {
        Self {
            node_id: config.node_id,
            od,
            nmt: NmtHandler::new(),
            sdo_server: SdoServer::new(),
            tpdo: TpdoEngine::new(config.tpdo),
            rpdo: RpdoEngine::new(config.rpdo),
            heartbeat: HeartbeatProducer::new(config.node_id, config.heartbeat_interval_ms),
            emcy: EmcyProducer::new(config.node_id),
            lss: LssSlave::new(config.identity, config.node_id.raw()),
            booted: false,
            auto_start: config.auto_start,
            event_queue: Deque::new(),
            dirty_set: Vec::new(),
            event_overflow_count: 0,
            #[cfg(feature = "embassy")]
            event_signal: None,
        }
    }

    pub fn node_id(&self) -> NodeId {
        self.node_id
    }

    pub fn state(&self) -> NmtState {
        self.nmt.state()
    }

    pub fn od(&self) -> &OD {
        &self.od
    }

    /// Returns a guard that provides mutable access to the OD.
    ///
    /// When the guard is dropped, any TPDO-mapped fields that changed are
    /// automatically marked dirty (triggering event-driven TPDOs on the next
    /// `process()` call). If the embassy feature is enabled and an event signal
    /// is set, it is also signaled.
    ///
    /// For batch updates, bind the guard to avoid repeated snapshots:
    /// ```ignore
    /// {
    ///     let mut od = node.od_mut();
    ///     od.button = 1;
    ///     od.echo_out = 42;
    /// } // single diff, notifies both
    /// ```
    pub fn od_mut(&mut self) -> OdGuard<'_, OD, TPDO, RPDO, EVT_QUEUE, DIRTY_SET>
    where
        OD: Clone,
    {
        let snapshot = self.od.clone();
        OdGuard {
            node: self,
            snapshot,
        }
    }

    pub fn tpdo_engine(&self) -> &TpdoEngine<TPDO> {
        &self.tpdo
    }

    pub fn tpdo_engine_mut(&mut self) -> &mut TpdoEngine<TPDO> {
        &mut self.tpdo
    }

    pub fn rpdo_engine(&self) -> &RpdoEngine<RPDO> {
        &self.rpdo
    }

    pub fn rpdo_engine_mut(&mut self) -> &mut RpdoEngine<RPDO> {
        &mut self.rpdo
    }

    /// Report an emergency error. Sends an EMCY frame on the next `process()`
    /// call and sets the corresponding bits in the error register.
    ///
    /// `register_bits` should be a combination of `emcy::error_register::*` constants.
    pub fn set_error(&mut self, error_code: u16, register_bits: u8, vendor_data: &[u8]) {
        self.emcy.set_error(error_code, register_bits, vendor_data);
    }

    /// Clear error register bits. If all errors are cleared, sends an
    /// "error reset" EMCY frame (code 0x0000) on the next `process()` call.
    pub fn clear_error(&mut self, register_bits: u8) {
        self.emcy.clear_error(register_bits);
    }

    /// Clear all errors and send an error-reset EMCY.
    pub fn clear_all_errors(&mut self) {
        self.emcy.clear_all();
    }

    /// Current error register value (OD 0x1001).
    pub fn error_register(&self) -> u8 {
        self.emcy.error_register()
    }

    /// Drain the next OD event from the queue.
    ///
    /// Call this in the main loop after `process()` to react to OD changes
    /// made by the protocol stack (SDO downloads, RPDO writes).
    pub fn next_event(&mut self) -> Option<OdEvent> {
        self.event_queue.pop_front()
    }

    /// Drain the next OD event and decode it into the OD's typed change enum.
    ///
    /// Events without an application-level variant (e.g. writes to
    /// auto-generated PDO communication parameters) are skipped. The value
    /// carried by the variant is read at drain time, so a burst of writes to
    /// the same entry yields the freshest value.
    pub fn next_change(&mut self) -> Option<OD::Change>
    where
        OD: crate::od::OdChanges,
    {
        while let Some(evt) = self.event_queue.pop_front() {
            if let Some(change) = self.od.decode_event(evt) {
                return Some(change);
            }
        }
        None
    }

    /// Resolved COB-ID of TPDO `n` (0-indexed: 0 = TPDO1), if configured.
    pub fn tpdo_cob_id(&self, n: usize) -> Option<u16> {
        self.tpdo.config(n).map(|c| c.cob_id)
    }

    /// Resolved COB-ID of RPDO `n` (0-indexed: 0 = RPDO1), if configured.
    pub fn rpdo_cob_id(&self, n: usize) -> Option<u16> {
        self.rpdo.config(n).map(|c| c.cob_id)
    }

    /// Number of events dropped due to event queue overflow.
    ///
    /// When the event queue (size `EVT_QUEUE`) is full, the oldest event is
    /// evicted to make room for the new one. This counter tracks how many
    /// events were lost. If this grows, increase `EVT_QUEUE` or drain events
    /// faster.
    pub fn events_dropped(&self) -> u32 {
        self.event_overflow_count
    }

    /// Request a CANopen-layer reset from the application.
    ///
    /// This triggers the same reset sequence as an NMT Reset command from an
    /// external master: the node aborts any in-progress SDO transfer, re-syncs
    /// PDO config from the OD, and re-enters Initializing. On the next
    /// `process()` call, the boot sequence runs (boot heartbeat, transition to
    /// PreOperational or Operational per `auto_start`).
    ///
    /// Use `ResetApplication` for a full reset (e.g., after firmware update) or
    /// `ResetCommunication` to re-initialize only communication parameters.
    pub fn request_reset(&mut self, reset_type: ResetType) {
        self.booted = false;
        self.sdo_server.abort_transfer();
        self.sync_pdo_from_od();
        match reset_type {
            ResetType::Application => {
                self.nmt = NmtHandler::new();
            }
            ResetType::Communication => {
                self.nmt = NmtHandler::new();
            }
        }
    }

    /// Mark an OD entry as changed by the application.
    ///
    /// On the next `process()` call, if an event-driven TPDO (type 254/255)
    /// maps this entry and the inhibit time has elapsed, the TPDO is sent.
    pub fn notify_changed(&mut self, index: u16, subindex: u8) {
        if !self
            .dirty_set
            .iter()
            .any(|&(i, s)| i == index && s == subindex)
        {
            let _ = self.dirty_set.push((index, subindex));
        }
    }

    /// Set an async event signal. When the protocol stack modifies an OD entry,
    /// the signal is woken so an async task can react without polling.
    ///
    /// The signal should live in a `static` so it can be awaited from a separate
    /// async task:
    ///
    /// ```ignore
    /// static EVENT_SIGNAL: OdEventSignal = OdEventSignal::new();
    ///
    /// // Setup:
    /// node.set_event_signal(&EVENT_SIGNAL);
    ///
    /// // Async task:
    /// loop {
    ///     EVENT_SIGNAL.wait().await;
    ///     // access node (e.g. via Mutex) to drain events
    /// }
    /// ```
    #[cfg(feature = "embassy")]
    pub fn set_event_signal(&mut self, signal: &'static OdEventSignal) {
        self.event_signal = Some(signal);
    }

    /// Main processing function. Call this periodically (e.g., every 1ms).
    ///
    /// Drains received frames from the transport, handles protocol logic,
    /// and queues outgoing frames (heartbeat, PDO, SDO responses).
    pub fn process(
        &mut self,
        transport: &mut impl embedded_can::nb::Can<Frame = CanFrame>,
        clock: &impl Clock,
    ) {
        let now = clock.now_us();

        // Boot sequence
        if !self.booted {
            self.booted = true;
            let boot_frame = self.heartbeat.send_boot(now);
            let _ = transport.transmit(&boot_frame);
            self.nmt.boot_complete();
            if self.auto_start {
                self.nmt
                    .process_command(NmtCommand::StartRemoteNode, 0, self.node_id);
            }
        }

        // Drain incoming frames
        let events_before = self.event_queue.len();
        loop {
            match transport.receive() {
                Ok(frame) => self.dispatch_frame(&frame, transport, now),
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => break,
            }
        }

        // SDO timeout check — abort stale transfers
        if let Some(abort_frame) = self.sdo_server.check_timeout(now) {
            let resp_cob = CobId::sdo_tx(self.node_id);
            if let Some(frame) = CanFrame::new(resp_cob.raw(), &abort_frame) {
                let _ = transport.transmit(&frame);
            }
        }

        // SDO block upload — send pending sub-block segments
        while let Some(seg_data) = self.sdo_server.poll_block_upload(now) {
            let resp_cob = CobId::sdo_tx(self.node_id);
            if let Some(frame) = CanFrame::new(resp_cob.raw(), &seg_data) {
                let _ = transport.transmit(&frame);
            }
        }

        // Check if any SDO write targeted PDO parameter entries → resync engines
        if self.event_queue.len() > events_before {
            let mut pdo_changed = false;
            // Peek at new events (they're at the tail)
            for i in events_before..self.event_queue.len() {
                if let Some(evt) = self.event_queue.get(i) {
                    if (evt.index >= 0x1400 && evt.index <= 0x1BFF)
                        && evt.source == crate::od::OdEventSource::Sdo
                    {
                        pdo_changed = true;
                        break;
                    }
                }
            }
            if pdo_changed {
                self.sync_pdo_from_od();
            }
        }

        // Heartbeat
        if let Some(hb) = self.heartbeat.poll(now, self.nmt.state()) {
            let _ = transport.transmit(&hb);
        }

        // EMCY (drain all pending frames — burst errors queue several)
        while let Some(emcy_frame) = self.emcy.take_pending() {
            let _ = transport.transmit(&emcy_frame);
        }

        // TPDO event timers + dirty-triggered sends (only in Operational)
        if self.nmt.state() == NmtState::Operational {
            let mut out = Vec::<CanFrame, TPDO>::new();
            self.tpdo.poll(&self.od, now, &self.dirty_set, &mut out);
            for frame in &out {
                let _ = transport.transmit(frame);
            }
        }
        self.dirty_set.clear();

        // Wake async waiters if events are pending
        #[cfg(feature = "embassy")]
        if !self.event_queue.is_empty() {
            if let Some(signal) = self.event_signal {
                signal.signal(());
            }
        }
    }

    /// Re-read PDO configuration from the OD (entries 0x1400-0x1BFF) and
    /// update the PDO engines. Call after SDO writes to PDO parameter entries,
    /// or at init to load config from OD.
    pub fn sync_pdo_from_od(&mut self) {
        let mut buf4 = [0u8; 4];
        let mut buf2 = [0u8; 2];

        for n in 0..TPDO {
            let comm_idx = 0x1800 + n as u16;
            let map_idx = 0x1A00 + n as u16;

            if let Some(config) = self.tpdo.config_mut(n) {
                if self.od.read(comm_idx, 1, &mut buf4).is_ok() {
                    let raw = u32::from_le_bytes(buf4);
                    config.enabled = (raw & 0x8000_0000) == 0;
                    config.cob_id = (raw & 0x7FF) as u16;
                }
                if self.od.read(comm_idx, 2, &mut buf4).is_ok() {
                    config.transmission_type = buf4[0];
                }
                if self.od.read(comm_idx, 3, &mut buf2).is_ok() {
                    config.inhibit_time_100us = u16::from_le_bytes(buf2);
                }
                if self.od.read(comm_idx, 5, &mut buf2).is_ok() {
                    config.event_timer_ms = u16::from_le_bytes(buf2);
                }

                // Read mapping count, then mapping entries
                config.mappings.clear();
                if self.od.read(map_idx, 0, &mut buf4).is_ok() {
                    let count = buf4[0].min(8);
                    for sub in 1..=count {
                        if self.od.read(map_idx, sub, &mut buf4).is_ok() {
                            let _ = config
                                .mappings
                                .push(PdoMapping::from_mapping_value(u32::from_le_bytes(buf4)));
                        }
                    }
                }
            }
        }

        for n in 0..RPDO {
            let comm_idx = 0x1400 + n as u16;
            let map_idx = 0x1600 + n as u16;

            if let Some(config) = self.rpdo.config_mut(n) {
                if self.od.read(comm_idx, 1, &mut buf4).is_ok() {
                    let raw = u32::from_le_bytes(buf4);
                    config.enabled = (raw & 0x8000_0000) == 0;
                    config.cob_id = (raw & 0x7FF) as u16;
                }
                if self.od.read(comm_idx, 2, &mut buf4).is_ok() {
                    config.transmission_type = buf4[0];
                }

                config.mappings.clear();
                if self.od.read(map_idx, 0, &mut buf4).is_ok() {
                    let count = buf4[0].min(8);
                    for sub in 1..=count {
                        if self.od.read(map_idx, sub, &mut buf4).is_ok() {
                            let _ = config
                                .mappings
                                .push(PdoMapping::from_mapping_value(u32::from_le_bytes(buf4)));
                        }
                    }
                }
            }
        }
    }

    fn dispatch_frame(
        &mut self,
        frame: &CanFrame,
        transport: &mut impl embedded_can::nb::Can<Frame = CanFrame>,
        now: u64,
    ) {
        let Some(cob) = CobId::new(frame.raw_id()) else {
            return;
        };

        match cob.parse() {
            ParsedCobId::Nmt => {
                if frame.raw_dlc() >= 2 {
                    let data = frame.data();
                    if let Some(cmd) = NmtCommand::from_byte(data[0]) {
                        let transition = self.nmt.process_command(cmd, data[1], self.node_id);
                        match transition {
                            NmtTransition::ResetApplication => {
                                // Reset everything: PDO config and application state.
                                // The application should handle resetting OD values
                                // to defaults via the event queue or other mechanism.
                                self.booted = false;
                                self.sdo_server.abort_transfer();
                                self.sync_pdo_from_od();
                            }
                            NmtTransition::ResetCommunication => {
                                // Reset only communication parameters (PDO, heartbeat).
                                // Application data in the OD is preserved.
                                self.booted = false;
                                self.sdo_server.abort_transfer();
                                self.sync_pdo_from_od();
                            }
                            _ => {}
                        }
                    }
                }
            }

            ParsedCobId::SdoRequest(node) if node == self.node_id => {
                if frame.raw_dlc() >= 8 {
                    let req: [u8; 8] = frame.data().try_into().unwrap();
                    let mut resp = [0u8; 8];
                    let was_full = self.event_queue.is_full();
                    if self
                        .sdo_server
                        .process(
                            &req,
                            &mut self.od,
                            &mut resp,
                            &mut self.event_queue,
                            self.nmt.state(),
                            now,
                        )
                        .is_ok()
                    {
                        let resp_cob = CobId::sdo_tx(self.node_id);
                        if let Some(resp_frame) = CanFrame::new(resp_cob.raw(), &resp) {
                            let _ = transport.transmit(&resp_frame);
                        }
                    }
                    if was_full && self.event_queue.is_full() {
                        // Queue was full before and after — if an event was pushed, one was dropped
                        self.event_overflow_count = self.event_overflow_count.saturating_add(1);
                    }
                }
            }

            ParsedCobId::Sync => {
                if self.nmt.state() == NmtState::Operational {
                    let mut out = Vec::<CanFrame, TPDO>::new();
                    self.tpdo.on_sync(&self.od, &mut out);
                    for f in &out {
                        let _ = transport.transmit(f);
                    }
                }
            }

            ParsedCobId::Rpdo { .. } | ParsedCobId::Tpdo { .. } => {
                // For RPDO: we receive on RPDO COB-IDs (which are the remote's TPDO)
                // The RPDO engine matches on COB-ID, so just feed it the frame
                if self.nmt.state() == NmtState::Operational {
                    let (_, dropped) = self.rpdo.process_with_drop_count(
                        frame,
                        &mut self.od,
                        &mut self.event_queue,
                    );
                    self.event_overflow_count = self.event_overflow_count.saturating_add(dropped);
                }
            }

            _ => {
                // Check for LSS (fixed COB-IDs, not part of standard COB-ID scheme)
                if frame.raw_id() == crate::lss::LSS_REQUEST_COB {
                    if let Some(resp) = self.lss.process(frame) {
                        let _ = transport.transmit(&resp);
                    }
                }
            }
        }
    }
}

/// RAII guard for mutable OD access with automatic change detection.
///
/// On drop, diffs TPDO-mapped fields against a snapshot taken at creation.
/// Changed fields are added to the node's dirty set, triggering event-driven
/// TPDOs on the next `process()` call. Also fires the embassy event signal
/// if set.
pub struct OdGuard<
    'a,
    OD: ObjectDictionary + Clone,
    const TPDO: usize,
    const RPDO: usize,
    const EVT_QUEUE: usize,
    const DIRTY_SET: usize,
> {
    node: &'a mut Node<OD, TPDO, RPDO, EVT_QUEUE, DIRTY_SET>,
    snapshot: OD,
}

impl<
        OD: ObjectDictionary + Clone,
        const TPDO: usize,
        const RPDO: usize,
        const EVT_QUEUE: usize,
        const DIRTY_SET: usize,
    > core::ops::Deref for OdGuard<'_, OD, TPDO, RPDO, EVT_QUEUE, DIRTY_SET>
{
    type Target = OD;
    fn deref(&self) -> &OD {
        &self.node.od
    }
}

impl<
        OD: ObjectDictionary + Clone,
        const TPDO: usize,
        const RPDO: usize,
        const EVT_QUEUE: usize,
        const DIRTY_SET: usize,
    > core::ops::DerefMut for OdGuard<'_, OD, TPDO, RPDO, EVT_QUEUE, DIRTY_SET>
{
    fn deref_mut(&mut self) -> &mut OD {
        &mut self.node.od
    }
}

impl<
        OD: ObjectDictionary + Clone,
        const TPDO: usize,
        const RPDO: usize,
        const EVT_QUEUE: usize,
        const DIRTY_SET: usize,
    > Drop for OdGuard<'_, OD, TPDO, RPDO, EVT_QUEUE, DIRTY_SET>
{
    fn drop(&mut self) {
        // Collect changed (index, subindex) pairs first to avoid borrow conflict
        let mut changed: [(u16, u8); 32] = [(0, 0); 32];
        let mut count = 0usize;

        for n in 0..TPDO {
            if let Some(config) = self.node.tpdo.config(n) {
                for mapping in config.mappings.iter() {
                    let mut old = [0u8; 8];
                    let mut new = [0u8; 8];
                    let old_len = self
                        .snapshot
                        .read(mapping.index, mapping.subindex, &mut old)
                        .unwrap_or(0);
                    let new_len = self
                        .node
                        .od
                        .read(mapping.index, mapping.subindex, &mut new)
                        .unwrap_or(0);
                    if old_len != new_len || old[..old_len] != new[..new_len] {
                        if count < changed.len() {
                            changed[count] = (mapping.index, mapping.subindex);
                            count += 1;
                        }
                    }
                }
            }
        }

        for i in 0..count {
            self.node.notify_changed(changed[i].0, changed[i].1);
        }

        // Wake async waiters so process() runs sooner
        #[cfg(feature = "embassy")]
        if count > 0 {
            if let Some(signal) = self.node.event_signal {
                signal.signal(());
            }
        }

        let _ = count; // suppress unused warning when embassy feature is off
    }
}

/// A [`Node`] in a `static`, shared between the protocol task and application
/// code. Wraps the `Mutex<RefCell<Option<Node>>>` pattern that Embassy
/// firmware otherwise spells out at every access.
///
/// ```ignore
/// static NODE: SharedNode<MyOd, 1, 1> = SharedNode::new();
///
/// // Setup:
/// NODE.init(node);
///
/// // Anywhere:
/// NODE.with(|node| node.od_mut().button = 1);
/// ```
#[cfg(feature = "embassy")]
pub struct SharedNode<
    OD: ObjectDictionary,
    const TPDO: usize = 4,
    const RPDO: usize = 4,
    const EVT_QUEUE: usize = 16,
    const DIRTY_SET: usize = 8,
> {
    inner: embassy_sync::blocking_mutex::Mutex<
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        core::cell::RefCell<Option<Node<OD, TPDO, RPDO, EVT_QUEUE, DIRTY_SET>>>,
    >,
}

#[cfg(feature = "embassy")]
impl<
        OD: ObjectDictionary,
        const TPDO: usize,
        const RPDO: usize,
        const EVT_QUEUE: usize,
        const DIRTY_SET: usize,
    > SharedNode<OD, TPDO, RPDO, EVT_QUEUE, DIRTY_SET>
{
    pub const fn new() -> Self {
        Self {
            inner: embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(None)),
        }
    }

    /// Store the node. Call once during setup, before any `with()`.
    pub fn init(&self, node: Node<OD, TPDO, RPDO, EVT_QUEUE, DIRTY_SET>) {
        self.inner.lock(|cell| {
            cell.borrow_mut().replace(node);
        });
    }

    /// Run `f` with exclusive access to the node (inside a critical section —
    /// keep it short).
    ///
    /// Panics if called before [`init`](Self::init).
    pub fn with<R>(
        &self,
        f: impl FnOnce(&mut Node<OD, TPDO, RPDO, EVT_QUEUE, DIRTY_SET>) -> R,
    ) -> R {
        self.inner.lock(|cell| {
            let mut borrow = cell.borrow_mut();
            let node = borrow
                .as_mut()
                .expect("SharedNode::with() called before init()");
            f(node)
        })
    }
}

#[cfg(feature = "embassy")]
impl<
        OD: ObjectDictionary,
        const TPDO: usize,
        const RPDO: usize,
        const EVT_QUEUE: usize,
        const DIRTY_SET: usize,
    > Default for SharedNode<OD, TPDO, RPDO, EVT_QUEUE, DIRTY_SET>
{
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datatypes::DataType;
    use crate::lss::LssIdentity;
    use crate::od::*;
    use crate::pdo::PdoMapping;
    use crate::transport::MailboxTransport;

    struct MinimalOd {
        device_type: u32,
        error_reg: u8,
    }

    static MINIMAL_META: &[OdEntryMeta] = &[
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
            index: 0x1001,
            subindex: 0,
            data_type: DataType::U8,
            access: AccessType::Ro,
            pdo_mappable: false,
            name: "error_register",
            max_size: None,
        },
    ];

    impl ObjectDictionary for MinimalOd {
        fn lookup(&self, index: u16, subindex: u8) -> Option<&'static OdEntryMeta> {
            MINIMAL_META
                .iter()
                .find(|e| e.index == index && e.subindex == subindex)
        }
        fn read(&self, index: u16, subindex: u8, buf: &mut [u8]) -> Result<usize, OdError> {
            match (index, subindex) {
                (0x1000, 0) => {
                    buf[..4].copy_from_slice(&self.device_type.to_le_bytes());
                    Ok(4)
                }
                (0x1001, 0) => {
                    buf[0] = self.error_reg;
                    Ok(1)
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

    struct TestClock(u64);
    impl Clock for TestClock {
        fn now_us(&self) -> u64 {
            self.0
        }
    }

    #[test]
    fn node_boots_and_sends_heartbeat() {
        let config = NodeConfig::<1, 1> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 100,
            auto_start: false,
            tpdo: [TpdoConfig::default()],
            rpdo: [RpdoConfig::default()],
            identity: LssIdentity::default(),
        };
        let od = MinimalOd {
            device_type: 0x191,
            error_reg: 0,
        };
        let mut node: Node<MinimalOd, 1, 1> = Node::new(config, od);
        let mut transport = MailboxTransport::<16, 16>::new();

        // First process: should send boot heartbeat
        node.process(&mut transport, &TestClock(0));

        let frame = transport.next_to_transmit().unwrap();
        assert_eq!(frame.raw_id(), 0x701); // heartbeat for node 1
        assert_eq!(frame.data()[0], 0x00); // Initializing state in boot message

        assert_eq!(node.state(), NmtState::PreOperational);
    }

    #[test]
    fn node_responds_to_nmt_and_sdo() {
        let config = NodeConfig::<1, 1> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 0, // disabled for this test
            auto_start: false,
            tpdo: [TpdoConfig::default()],
            rpdo: [RpdoConfig::default()],
            identity: LssIdentity::default(),
        };
        let od = MinimalOd {
            device_type: 0x191,
            error_reg: 0,
        };
        let mut node: Node<MinimalOd, 1, 1> = Node::new(config, od);
        let mut transport = MailboxTransport::<16, 16>::new();

        // Boot
        node.process(&mut transport, &TestClock(0));
        let _ = transport.next_to_transmit(); // drain boot heartbeat

        // Send NMT start
        let nmt_frame = CanFrame::new(0x000, &[0x01, 0x01]).unwrap(); // Start node 1
        transport.store_received(nmt_frame).unwrap();
        node.process(&mut transport, &TestClock(1000));
        assert_eq!(node.state(), NmtState::Operational);

        // Send SDO upload request for 0x1000:0
        let sdo_req = CanFrame::new(0x601, &[0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0]).unwrap();
        transport.store_received(sdo_req).unwrap();
        node.process(&mut transport, &TestClock(2000));

        let resp = transport.next_to_transmit().unwrap();
        assert_eq!(resp.raw_id(), 0x581); // SDO response from node 1
                                          // Check expedited upload response contains device_type
        assert_eq!(&resp.data()[4..8], &0x191u32.to_le_bytes());
    }

    // --- OD with writable + PDO-mappable entries for event tests ---

    #[derive(Clone)]
    struct EventTestOd {
        device_type: u32,
        output1: u8,
        output2: u8,
        input1: u8,
    }

    static EVENT_TEST_META: &[OdEntryMeta] = &[
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
            index: 0x6200,
            subindex: 1,
            data_type: DataType::U8,
            access: AccessType::Rw,
            pdo_mappable: true,
            name: "output1",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x6200,
            subindex: 2,
            data_type: DataType::U8,
            access: AccessType::Rw,
            pdo_mappable: true,
            name: "output2",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x6000,
            subindex: 1,
            data_type: DataType::U8,
            access: AccessType::Ro,
            pdo_mappable: true,
            name: "input1",
            max_size: None,
        },
    ];

    impl ObjectDictionary for EventTestOd {
        fn lookup(&self, index: u16, subindex: u8) -> Option<&'static OdEntryMeta> {
            EVENT_TEST_META
                .iter()
                .find(|e| e.index == index && e.subindex == subindex)
        }
        fn read(&self, index: u16, subindex: u8, buf: &mut [u8]) -> Result<usize, OdError> {
            match (index, subindex) {
                (0x1000, 0) => {
                    buf[..4].copy_from_slice(&self.device_type.to_le_bytes());
                    Ok(4)
                }
                (0x6200, 1) => {
                    buf[0] = self.output1;
                    Ok(1)
                }
                (0x6200, 2) => {
                    buf[0] = self.output2;
                    Ok(1)
                }
                (0x6000, 1) => {
                    buf[0] = self.input1;
                    Ok(1)
                }
                _ => Err(OdError::NotFound),
            }
        }
        fn write(&mut self, index: u16, subindex: u8, data: &[u8]) -> Result<(), OdError> {
            match (index, subindex) {
                (0x6200, 1) => {
                    self.output1 = data[0];
                    Ok(())
                }
                (0x6200, 2) => {
                    self.output2 = data[0];
                    Ok(())
                }
                _ => Err(OdError::ReadOnly),
            }
        }
        fn sub_count(&self, _: u16) -> Option<u8> {
            Some(0)
        }
    }

    fn make_event_node() -> (Node<EventTestOd, 1, 1>, MailboxTransport<32, 32>) {
        let mut rpdo_mappings = heapless::Vec::<PdoMapping, 8>::new();
        rpdo_mappings
            .push(PdoMapping {
                index: 0x6200,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();
        rpdo_mappings
            .push(PdoMapping {
                index: 0x6200,
                subindex: 2,
                bit_length: 8,
            })
            .unwrap();

        let mut tpdo_mappings = heapless::Vec::<PdoMapping, 8>::new();
        tpdo_mappings
            .push(PdoMapping {
                index: 0x6000,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();

        let config = NodeConfig::<1, 1> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 0,
            auto_start: true,
            tpdo: [TpdoConfig {
                cob_id: 0x181,
                transmission_type: 255,
                inhibit_time_100us: 0,
                event_timer_ms: 0, // no timer, only dirty-triggered
                mappings: tpdo_mappings,
                enabled: true,
            }],
            rpdo: [RpdoConfig {
                cob_id: 0x201,
                transmission_type: 255,
                mappings: rpdo_mappings,
                enabled: true,
            }],
            identity: LssIdentity::default(),
        };
        let od = EventTestOd {
            device_type: 0x191,
            output1: 0,
            output2: 0,
            input1: 0,
        };
        let mut node: Node<EventTestOd, 1, 1> = Node::new(config, od);
        let mut transport = MailboxTransport::<32, 32>::new();

        // Boot + auto-start
        node.process(&mut transport, &TestClock(0));
        // Drain boot heartbeat
        while transport.next_to_transmit().is_some() {}
        assert_eq!(node.state(), NmtState::Operational);

        (node, transport)
    }

    #[test]
    fn sdo_download_generates_event() {
        let (mut node, mut transport) = make_event_node();

        // SDO expedited download: write 0x42 to 0x6200:1
        let sdo_req =
            CanFrame::new(0x601, &[0x2F, 0x00, 0x62, 0x01, 0x42, 0x00, 0x00, 0x00]).unwrap();
        transport.store_received(sdo_req).unwrap();
        node.process(&mut transport, &TestClock(1000));

        let evt = node.next_event().unwrap();
        assert_eq!(evt.index, 0x6200);
        assert_eq!(evt.subindex, 1);
        assert_eq!(evt.source, OdEventSource::Sdo);
        assert!(node.next_event().is_none());
    }

    #[test]
    fn rpdo_generates_events_per_mapped_entry() {
        let (mut node, mut transport) = make_event_node();

        // Send RPDO frame with two mapped bytes
        let rpdo_frame = CanFrame::new(0x201, &[0xAA, 0xBB]).unwrap();
        transport.store_received(rpdo_frame).unwrap();
        node.process(&mut transport, &TestClock(1000));

        let evt1 = node.next_event().unwrap();
        assert_eq!(evt1.index, 0x6200);
        assert_eq!(evt1.subindex, 1);
        assert_eq!(evt1.source, OdEventSource::Rpdo);

        let evt2 = node.next_event().unwrap();
        assert_eq!(evt2.index, 0x6200);
        assert_eq!(evt2.subindex, 2);
        assert_eq!(evt2.source, OdEventSource::Rpdo);

        assert!(node.next_event().is_none());
        assert_eq!(node.od().output1, 0xAA);
        assert_eq!(node.od().output2, 0xBB);
    }

    #[test]
    fn event_queue_overflow_drops_oldest() {
        // Use a tiny queue of size 2
        let mut rpdo_mappings = heapless::Vec::<PdoMapping, 8>::new();
        rpdo_mappings
            .push(PdoMapping {
                index: 0x6200,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();
        rpdo_mappings
            .push(PdoMapping {
                index: 0x6200,
                subindex: 2,
                bit_length: 8,
            })
            .unwrap();

        let config = NodeConfig::<1, 1> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 0,
            auto_start: true,
            tpdo: [TpdoConfig::default()],
            rpdo: [RpdoConfig {
                cob_id: 0x201,
                transmission_type: 255,
                mappings: rpdo_mappings,
                enabled: true,
            }],
            identity: LssIdentity::default(),
        };
        let od = EventTestOd {
            device_type: 0x191,
            output1: 0,
            output2: 0,
            input1: 0,
        };
        // EVT_QUEUE = 2, so two RPDO events will fill it, then SDO will overflow
        let mut node: Node<EventTestOd, 1, 1, 2, 8> = Node::new(config, od);
        let mut transport = MailboxTransport::<32, 32>::new();

        node.process(&mut transport, &TestClock(0));
        while transport.next_to_transmit().is_some() {}

        // RPDO with 2 mappings fills the queue
        let rpdo_frame = CanFrame::new(0x201, &[0x11, 0x22]).unwrap();
        transport.store_received(rpdo_frame).unwrap();
        // SDO write will try to push a 3rd event, dropping the oldest
        let sdo_req =
            CanFrame::new(0x601, &[0x2F, 0x00, 0x62, 0x01, 0x33, 0x00, 0x00, 0x00]).unwrap();
        transport.store_received(sdo_req).unwrap();
        node.process(&mut transport, &TestClock(1000));

        // Oldest (0x6200:1 RPDO) should have been dropped
        let evt1 = node.next_event().unwrap();
        assert_eq!(evt1.index, 0x6200);
        assert_eq!(evt1.subindex, 2);
        assert_eq!(evt1.source, OdEventSource::Rpdo);

        let evt2 = node.next_event().unwrap();
        assert_eq!(evt2.index, 0x6200);
        assert_eq!(evt2.subindex, 1);
        assert_eq!(evt2.source, OdEventSource::Sdo);

        assert!(node.next_event().is_none());
    }

    #[test]
    fn event_queue_drop_count_includes_drops_inside_multi_mapping_rpdo() {
        let mut rpdo_mappings = heapless::Vec::<PdoMapping, 8>::new();
        rpdo_mappings
            .push(PdoMapping {
                index: 0x6200,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();
        rpdo_mappings
            .push(PdoMapping {
                index: 0x6200,
                subindex: 2,
                bit_length: 8,
            })
            .unwrap();

        let config = NodeConfig::<1, 1> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 0,
            auto_start: true,
            tpdo: [TpdoConfig::default()],
            rpdo: [RpdoConfig {
                cob_id: 0x201,
                transmission_type: 255,
                mappings: rpdo_mappings,
                enabled: true,
            }],
            identity: LssIdentity::default(),
        };
        let od = EventTestOd {
            device_type: 0x191,
            output1: 0,
            output2: 0,
            input1: 0,
        };
        let mut node: Node<EventTestOd, 1, 1, 1, 8> = Node::new(config, od);
        let mut transport = MailboxTransport::<32, 32>::new();

        node.process(&mut transport, &TestClock(0));
        while transport.next_to_transmit().is_some() {}

        let rpdo_frame = CanFrame::new(0x201, &[0x11, 0x22]).unwrap();
        transport.store_received(rpdo_frame).unwrap();
        node.process(&mut transport, &TestClock(1000));

        assert_eq!(node.events_dropped(), 1);
    }

    #[test]
    fn notify_changed_triggers_tpdo() {
        let (mut node, mut transport) = make_event_node();

        // No TPDO should be sent without notification (event_timer_ms=0, no dirty)
        node.process(&mut transport, &TestClock(1000));
        assert!(transport.next_to_transmit().is_none());

        // Update input and notify
        node.od_mut().input1 = 0x55;
        node.notify_changed(0x6000, 1);
        node.process(&mut transport, &TestClock(2000));

        let frame = transport.next_to_transmit().unwrap();
        assert_eq!(frame.raw_id(), 0x181);
        assert_eq!(frame.data()[0], 0x55);
    }

    #[test]
    fn notify_changed_respects_inhibit_time() {
        let mut tpdo_mappings = heapless::Vec::<PdoMapping, 8>::new();
        tpdo_mappings
            .push(PdoMapping {
                index: 0x6000,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();

        let config = NodeConfig::<1, 1> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 0,
            auto_start: true,
            tpdo: [TpdoConfig {
                cob_id: 0x181,
                transmission_type: 255,
                inhibit_time_100us: 1000, // 100ms inhibit
                event_timer_ms: 0,
                mappings: tpdo_mappings,
                enabled: true,
            }],
            rpdo: [RpdoConfig::default()],
            identity: LssIdentity::default(),
        };
        let od = EventTestOd {
            device_type: 0x191,
            output1: 0,
            output2: 0,
            input1: 0x10,
        };
        let mut node: Node<EventTestOd, 1, 1> = Node::new(config, od);
        let mut transport = MailboxTransport::<32, 32>::new();

        node.process(&mut transport, &TestClock(0));
        while transport.next_to_transmit().is_some() {}

        // First notify: should send (elapsed since last_send=0 is 50ms, but last_send starts at 0)
        node.od_mut().input1 = 0x11;
        node.notify_changed(0x6000, 1);
        node.process(&mut transport, &TestClock(50_000)); // 50ms - less than 100ms inhibit
                                                          // last_send_us[0] is 0, elapsed = 50ms < 100ms inhibit, should NOT send
        assert!(transport.next_to_transmit().is_none());

        // At 100ms, should send
        node.notify_changed(0x6000, 1);
        node.process(&mut transport, &TestClock(100_000));
        let frame = transport.next_to_transmit().unwrap();
        assert_eq!(frame.data()[0], 0x11);

        // Immediately after, within inhibit time, should not send
        node.od_mut().input1 = 0x22;
        node.notify_changed(0x6000, 1);
        node.process(&mut transport, &TestClock(150_000)); // 50ms after last send
        assert!(transport.next_to_transmit().is_none());
    }

    #[test]
    fn notify_changed_does_not_trigger_sync_tpdo() {
        let mut tpdo_mappings = heapless::Vec::<PdoMapping, 8>::new();
        tpdo_mappings
            .push(PdoMapping {
                index: 0x6000,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();

        let config = NodeConfig::<1, 1> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 0,
            auto_start: true,
            tpdo: [TpdoConfig {
                cob_id: 0x181,
                transmission_type: 1, // sync cyclic, NOT event-driven
                inhibit_time_100us: 0,
                event_timer_ms: 0,
                mappings: tpdo_mappings,
                enabled: true,
            }],
            rpdo: [RpdoConfig::default()],
            identity: LssIdentity::default(),
        };
        let od = EventTestOd {
            device_type: 0x191,
            output1: 0,
            output2: 0,
            input1: 0x42,
        };
        let mut node: Node<EventTestOd, 1, 1> = Node::new(config, od);
        let mut transport = MailboxTransport::<32, 32>::new();

        node.process(&mut transport, &TestClock(0));
        while transport.next_to_transmit().is_some() {}

        // notify_changed should NOT trigger a sync-type TPDO
        node.notify_changed(0x6000, 1);
        node.process(&mut transport, &TestClock(1000));
        assert!(transport.next_to_transmit().is_none());
    }

    #[test]
    fn od_guard_auto_notifies_on_drop() {
        let (mut node, mut transport) = make_event_node();

        // No explicit notify_changed — the guard does it on drop
        node.od_mut().input1 = 0x77;
        node.process(&mut transport, &TestClock(1000));

        let frame = transport.next_to_transmit().unwrap();
        assert_eq!(frame.raw_id(), 0x181);
        assert_eq!(frame.data()[0], 0x77);
    }

    #[test]
    fn od_guard_batch_update() {
        let (mut node, mut transport) = make_event_node();

        // Batch update: one snapshot, one diff
        {
            let mut od = node.od_mut();
            od.input1 = 0xAA;
            // output1 is RPDO-mapped, not TPDO — shouldn't trigger
            od.output1 = 0xBB;
        }
        node.process(&mut transport, &TestClock(1000));

        let frame = transport.next_to_transmit().unwrap();
        assert_eq!(frame.data()[0], 0xAA);
        // Only one TPDO sent (input1 changed)
        assert!(transport.next_to_transmit().is_none());
    }

    #[test]
    fn request_reset_reboots_node() {
        let config = NodeConfig::<1, 1> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 100,
            auto_start: false,
            tpdo: [TpdoConfig::default()],
            rpdo: [RpdoConfig::default()],
            identity: LssIdentity::default(),
        };
        let od = MinimalOd {
            device_type: 0x191,
            error_reg: 0,
        };
        let mut node: Node<MinimalOd, 1, 1> = Node::new(config, od);
        let mut transport = MailboxTransport::<16, 16>::new();

        // Boot and go to PreOperational
        node.process(&mut transport, &TestClock(0));
        while transport.next_to_transmit().is_some() {}
        assert_eq!(node.state(), NmtState::PreOperational);

        // Request reset — should go back to Initializing
        node.request_reset(ResetType::Application);
        assert_eq!(node.state(), NmtState::Initializing);

        // Next process() should re-boot (send boot heartbeat, transition to PreOp)
        node.process(&mut transport, &TestClock(100_000));
        let frame = transport.next_to_transmit().unwrap();
        assert_eq!(frame.raw_id(), 0x701);
        assert_eq!(frame.data()[0], 0x00); // boot heartbeat
        assert_eq!(node.state(), NmtState::PreOperational);
    }

    #[test]
    fn od_guard_no_change_no_notify() {
        let (mut node, mut transport) = make_event_node();

        // Write the same value — no change, no TPDO
        node.od_mut().input1 = 0; // already 0
        node.process(&mut transport, &TestClock(1000));
        assert!(transport.next_to_transmit().is_none());
    }
}
