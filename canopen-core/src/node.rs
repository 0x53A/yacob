use crate::cobid::{CobId, NodeId, ParsedCobId};
use crate::emcy::EmcyProducer;
use crate::heartbeat::{
    HeartbeatEvent, HeartbeatEventBuf, HeartbeatMonitor, HeartbeatMonitorState, HeartbeatProducer,
};
use crate::lss::{LssIdentity, LssSlave};
use crate::nmt::{NmtCommand, NmtHandler, NmtState, NmtTransition};
use crate::od::ObjectDictionary;
use crate::od::OdEvent;
#[cfg(feature = "embassy")]
use crate::od::OdEventSignal;
use crate::pdo::{PdoMapping, PdoNumber, RpdoConfig, RpdoEngine, TpdoConfig, TpdoEngine};
use crate::sdo::{SdoServer, SdoServerConfig};
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
///
/// `SDO` is the number of *additional* SDO servers (0x1201+); the default
/// server (0x1200) is always present and not counted here. It defaults to 0, so
/// nodes that don't declare extra SDO channels pay no cost.
pub struct NodeConfig<const TPDO: usize = 4, const RPDO: usize = 4, const SDO: usize = 0> {
    pub node_id: NodeId,
    /// Producer heartbeat interval in ms (0 = disabled). Fallback only: if
    /// the OD declares 0x1017 (Producer heartbeat time), that entry is the
    /// source of truth — its value wins at init, and SDO writes to it change
    /// the runtime period.
    pub heartbeat_interval_ms: u16,
    /// If true, the node transitions directly to Operational after boot,
    /// without waiting for an NMT Start command from a master.
    pub auto_start: bool,
    pub tpdo: [TpdoConfig; TPDO],
    pub rpdo: [RpdoConfig; RPDO],
    /// Additional SDO servers (0x1201+), resolved COB-IDs. Empty for the common
    /// single-server node.
    pub sdo_servers: [SdoServerConfig; SDO],
    /// LSS identity (0x1018). Set to default if LSS is not needed.
    pub identity: LssIdentity,
}

impl<const TPDO: usize, const RPDO: usize, const SDO: usize> NodeConfig<TPDO, RPDO, SDO> {
    /// Build a config with PDO and additional-SDO-server settings pulled from
    /// the OD (declared in the `object_dictionary!` macro) and defaults for the
    /// rest: heartbeat every 1000 ms, no auto-start, default LSS identity.
    ///
    /// Override the defaults with struct update syntax:
    /// ```ignore
    /// let config = NodeConfig {
    ///     heartbeat_interval_ms: 500,
    ///     auto_start: true,
    ///     ..NodeConfig::from_od(&od, node_id)
    /// };
    /// ```
    pub fn from_od(
        od: &(impl crate::pdo::PdoConfigSource<TPDO, RPDO>
              + crate::sdo::SdoServerConfigSource<SDO>),
        node_id: NodeId,
    ) -> Self {
        Self {
            node_id,
            heartbeat_interval_ms: 1000,
            auto_start: false,
            tpdo: od.tpdo_configs(node_id),
            rpdo: od.rpdo_configs(node_id),
            sdo_servers: od.sdo_server_configs(node_id),
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
    const SDO: usize = 0,
    const EVT_QUEUE: usize = 16,
    const DIRTY_SET: usize = 8,
> {
    node_id: NodeId,
    od: OD,
    nmt: NmtHandler,
    sdo_server: SdoServer,
    /// Additional SDO servers (0x1201+), each an independent transfer state
    /// machine. `SDO` is 0 for the common single-server node.
    extra_sdo: [SdoServer; SDO],
    /// Resolved rx COB-ID (client→server) for each additional SDO server,
    /// parallel to `extra_sdo`. Dispatch matches incoming frames against these.
    extra_sdo_rx: [u16; SDO],
    /// Resolved tx COB-ID (server→client) for each additional SDO server.
    extra_sdo_tx: [u16; SDO],
    tpdo: TpdoEngine<TPDO>,
    rpdo: RpdoEngine<RPDO>,
    heartbeat: HeartbeatProducer,
    hb_monitors: [Option<HeartbeatMonitor>; MAX_HEARTBEAT_MONITORS],
    hb_event_queue: Deque<HeartbeatEvent, HB_EVENT_QUEUE>,
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

/// Maximum number of 0x1016 consumer heartbeat entries a [`Node`] monitors.
/// OD entries beyond this many subindices are readable/writable via SDO but
/// not monitored.
pub const MAX_HEARTBEAT_MONITORS: usize = 8;

/// Size of the typed heartbeat-consumer event queue (drained via
/// [`Node::next_heartbeat_event`]). On overflow the oldest event is evicted
/// and [`Node::events_dropped`] is incremented.
const HB_EVENT_QUEUE: usize = 8;

/// Selects which SDO server an incoming request is dispatched to.
#[derive(Clone, Copy)]
enum SdoServerSel {
    /// The default server (0x1200, `0x600/0x580 + node_id`).
    Default,
    /// An additional server (0x1201+); index into `extra_sdo`.
    Extra(usize),
}

/// 0-based offset of a PDO's comm/mapping records from the range base
/// (0x1400/0x1600/0x1800/0x1A00): explicit CANopen number if set, else the
/// engine slot.
#[inline]
fn pdo_od_offset(od_number: u16, slot: usize) -> u16 {
    if od_number == 0 {
        slot as u16
    } else {
        od_number - 1
    }
}

impl<
        OD: ObjectDictionary,
        const TPDO: usize,
        const RPDO: usize,
        const SDO: usize,
        const EVT_QUEUE: usize,
        const DIRTY_SET: usize,
    > Node<OD, TPDO, RPDO, SDO, EVT_QUEUE, DIRTY_SET>
{
    pub fn new(config: NodeConfig<TPDO, RPDO, SDO>, od: OD) -> Self {
        let extra_sdo_rx = config.sdo_servers.map(|s| s.cob_rx);
        let extra_sdo_tx = config.sdo_servers.map(|s| s.cob_tx);
        let mut node = Self {
            node_id: config.node_id,
            od,
            nmt: NmtHandler::new(),
            sdo_server: SdoServer::new(),
            extra_sdo: [const { SdoServer::new() }; SDO],
            extra_sdo_rx,
            extra_sdo_tx,
            tpdo: TpdoEngine::new(config.tpdo),
            rpdo: RpdoEngine::new(config.rpdo),
            heartbeat: HeartbeatProducer::new(config.node_id, config.heartbeat_interval_ms),
            hb_monitors: [const { None }; MAX_HEARTBEAT_MONITORS],
            hb_event_queue: Deque::new(),
            emcy: EmcyProducer::new(config.node_id),
            lss: LssSlave::new(config.identity, config.node_id.raw()),
            booted: false,
            auto_start: config.auto_start,
            event_queue: Deque::new(),
            dirty_set: Vec::new(),
            event_overflow_count: 0,
            #[cfg(feature = "embassy")]
            event_signal: None,
        };
        node.write_pdo_cob_ids_to_od();
        // Mirror resolved additional-SDO-server COB-IDs into the OD (0x1201+)
        // so SDO reads return real values (node-relative resolved).
        node.od.store_sdo_server_cob_ids(config.node_id);
        // A hard invariant, not a debug-only check: colliding COB-IDs make the
        // dispatcher steal frames from the default server (extras are matched
        // first) or answer on the wrong COB-ID. The config is fixed at build
        // time, so a violation is a deterministic programming error — fail fast
        // in every build rather than silently misroute SDO traffic in release.
        // Most cases are already rejected at compile time by the macro; this
        // catches the node-id-dependent residue (an absolute COB-ID that lands
        // on the default or another server for this particular node id).
        assert!(
            node.extra_sdo_cob_ids_are_unique(),
            "additional SDO server COB-IDs collide with the default server, \
             each other, or are zero (for node id {})",
            config.node_id.raw()
        );
        node.sync_heartbeat_from_od(false);
        node
    }

    /// Additional SDO server rx/tx COB-IDs must be non-zero, distinct from each
    /// other, and distinct from the default SDO server's `0x600/0x580 +
    /// node_id`. Enforced by an assert in [`Node::new`] (see there).
    fn extra_sdo_cob_ids_are_unique(&self) -> bool {
        let def_rx = CobId::sdo_rx(self.node_id).raw();
        let def_tx = CobId::sdo_tx(self.node_id).raw();
        for i in 0..SDO {
            let (rx, tx) = (self.extra_sdo_rx[i], self.extra_sdo_tx[i]);
            if rx == 0 || tx == 0 || rx == tx {
                return false;
            }
            if rx == def_rx || rx == def_tx || tx == def_rx || tx == def_tx {
                return false;
            }
            for j in (i + 1)..SDO {
                let (orx, otx) = (self.extra_sdo_rx[j], self.extra_sdo_tx[j]);
                if rx == orx || rx == otx || tx == orx || tx == otx {
                    return false;
                }
            }
        }
        true
    }

    /// Re-read heartbeat configuration from the OD: producer heartbeat time
    /// (0x1017) and consumer heartbeat entries (0x1016). Called at init,
    /// after SDO writes, and on reset. If the OD has no 0x1017 entry, the
    /// `NodeConfig` interval stays in effect.
    ///
    /// With `preserve_monitor_state`, monitors whose 0x1016 entry is
    /// unchanged keep their runtime state (used for SDO rewrites, so touching
    /// one entry doesn't restart the others). Without it, every monitor
    /// restarts in `Waiting` (used for local resets: after reinitialization,
    /// monitoring starts with the first heartbeat again).
    fn sync_heartbeat_from_od(&mut self, preserve_monitor_state: bool) {
        let mut buf2 = [0u8; 2];
        if self.od.read(0x1017, 0, &mut buf2).is_ok() {
            self.heartbeat.set_interval_ms(u16::from_le_bytes(buf2));
        }

        // 0x1016 subindex N ↔ monitor slot N-1. Entry format:
        // bits 0..16 consumer time in ms (0 = disabled), bits 16..24 node id.
        let mut buf4 = [0u8; 4];
        for slot in 0..MAX_HEARTBEAT_MONITORS {
            let configured = self
                .od
                .read(0x1016, (slot + 1) as u8, &mut buf4)
                .ok()
                .and_then(|_| {
                    let raw = u32::from_le_bytes(buf4);
                    let time_ms = (raw & 0xFFFF) as u16;
                    if time_ms == 0 {
                        return None;
                    }
                    NodeId::new(((raw >> 16) & 0xFF) as u8).map(|node| (node, time_ms))
                });
            self.hb_monitors[slot] = match (configured, self.hb_monitors[slot].take()) {
                (Some((node, time)), Some(existing))
                    if preserve_monitor_state
                        && existing.node_id() == node
                        && existing.timeout_ms() == time =>
                {
                    Some(existing)
                }
                (Some((node, time)), _) => Some(HeartbeatMonitor::new(node, time)),
                (None, _) => None,
            };
        }
    }

    /// Current producer heartbeat interval in ms (0 = disabled).
    pub fn heartbeat_interval_ms(&self) -> u16 {
        self.heartbeat.interval_ms()
    }

    /// Mirror the resolved PDO COB-IDs into the OD comm-param entries.
    ///
    /// ODs generated by `object_dictionary!` store 0 for "predefined default"
    /// COB-IDs; the actual value is only resolved once the node ID is known.
    /// Writing it back makes SDO reads of 0x1400/0x1800 sub 1 return the real
    /// COB-ID and makes `sync_pdo_from_od` idempotent across resets.
    fn write_pdo_cob_ids_to_od(&mut self) {
        for n in 0..TPDO {
            if let Some(config) = self.tpdo.config_slot(n) {
                let off = pdo_od_offset(config.od_number, n);
                let raw = config.cob_id as u32 | if config.enabled { 0 } else { 0x8000_0000 };
                let _ = self.od.write(0x1800 + off, 1, &raw.to_le_bytes());
            }
        }
        for n in 0..RPDO {
            if let Some(config) = self.rpdo.config_slot(n) {
                let off = pdo_od_offset(config.od_number, n);
                let raw = config.cob_id as u32 | if config.enabled { 0 } else { 0x8000_0000 };
                let _ = self.od.write(0x1400 + off, 1, &raw.to_le_bytes());
            }
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
    pub fn od_mut(&mut self) -> OdGuard<'_, OD, TPDO, RPDO, SDO, EVT_QUEUE, DIRTY_SET>
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

    /// Resolved COB-ID of the TPDO with the given CANopen number
    /// (`PdoNumber::of::<1>()` = TPDO1), if declared.
    pub fn tpdo_cob_id(&self, number: PdoNumber) -> Option<CobId> {
        // cob_id 0 = predefined default not yet resolved — never a valid PDO
        // COB-ID (it is the NMT id), so report it as absent.
        self.tpdo
            .config(number)
            .filter(|c| c.cob_id != 0)
            .and_then(|c| CobId::new(c.cob_id))
    }

    /// Resolved COB-ID of the RPDO with the given CANopen number
    /// (`PdoNumber::of::<1>()` = RPDO1), if declared.
    pub fn rpdo_cob_id(&self, number: PdoNumber) -> Option<CobId> {
        self.rpdo
            .config(number)
            .filter(|c| c.cob_id != 0)
            .and_then(|c| CobId::new(c.cob_id))
    }

    /// Does the RPDO with the given CANopen number lack an in-deadline
    /// reception? (`PdoNumber::of::<79>()` = RPDO79, sparse or not.)
    ///
    /// Deadline monitoring (CiA 301 RPDO event timer, comm param sub 5) is
    /// enabled per RPDO via `deadline = ...` in the DSL or an SDO write to
    /// 0x1400 + N - 1 sub 5 (in Pre-Operational). For a monitored RPDO this
    /// level query reads `true` whenever the data is not fresh: **initially,
    /// before the first frame ever arrives** (including right after entering
    /// Operational), and again once the gap since the last frame exceeds the
    /// deadline — until the next reception clears it. Unmonitored or
    /// undeclared RPDOs always read `false`. The mapped OD entries keep
    /// their last received values throughout.
    ///
    /// The `OdEventSource::RpdoDeadline` event channel is narrower: it arms
    /// on first reception and fires once per silence period, so no event (and
    /// no app-level EMCY built on it) occurs before the counterpart has ever
    /// spoken.
    ///
    /// Detection latency equals the `process()` call period. The stack never
    /// sends EMCY automatically — report `EmcyErrorCode::RpdoTimeout`
    /// (0x8250) via [`set_error`](Self::set_error) if the bus should know.
    pub fn rpdo_deadline_expired(&self, number: PdoNumber) -> bool {
        self.rpdo.deadline_expired(number)
    }

    /// Drain the next heartbeat-consumer event (0x1016 monitoring).
    ///
    /// Emitted for monitored nodes: monitoring started, remote NMT state
    /// change, remote reset (boot-up frame), timeout, and recovery. The stack
    /// only reports — reacting (e.g. EMCY 0x8130, stopping outputs) is
    /// application policy.
    pub fn next_heartbeat_event(&mut self) -> Option<HeartbeatEvent> {
        self.hb_event_queue.pop_front()
    }

    /// Level query for a monitored node's heartbeat (0x1016 consumer).
    ///
    /// `Disabled` if no 0x1016 entry monitors that node;
    /// `Waiting` until its first heartbeat arrives; use
    /// [`HeartbeatMonitorState::is_alive`] as the freshness ("valid") flag,
    /// analogous to [`rpdo_deadline_expired`](Self::rpdo_deadline_expired).
    pub fn heartbeat_status(&self, node: NodeId) -> HeartbeatMonitorState {
        self.hb_monitors
            .iter()
            .flatten()
            .find(|m| m.node_id() == node)
            .map(|m| m.state())
            .unwrap_or(HeartbeatMonitorState::Disabled)
    }

    /// Number of events dropped due to event queue overflow (OD events and
    /// heartbeat-consumer events combined).
    ///
    /// When a queue is full, the oldest event is evicted to make room for the
    /// new one. This counter tracks how many events were lost. If this grows,
    /// increase `EVT_QUEUE` or drain events faster.
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
        self.abort_all_sdo_transfers();
        self.sync_pdo_from_od();
        self.sync_heartbeat_from_od(false);
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
        loop {
            match transport.receive() {
                Ok(frame) => self.dispatch_frame(&frame, transport, now),
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => break,
            }
        }

        // SDO timeout check — abort stale transfers (default + extra servers)
        if let Some(abort_frame) = self.sdo_server.check_timeout(now) {
            let resp_cob = CobId::sdo_tx(self.node_id);
            if let Some(frame) = CanFrame::new(resp_cob.raw(), &abort_frame) {
                let _ = transport.transmit(&frame);
            }
        }
        for i in 0..SDO {
            if let Some(abort_frame) = self.extra_sdo[i].check_timeout(now) {
                if let Some(frame) = CanFrame::new(self.extra_sdo_tx[i], &abort_frame) {
                    let _ = transport.transmit(&frame);
                }
            }
        }

        // SDO block upload — send pending sub-block segments (default + extra)
        while let Some(seg_data) = self.sdo_server.poll_block_upload(now) {
            let resp_cob = CobId::sdo_tx(self.node_id);
            if let Some(frame) = CanFrame::new(resp_cob.raw(), &seg_data) {
                let _ = transport.transmit(&frame);
            }
        }
        for i in 0..SDO {
            while let Some(seg_data) = self.extra_sdo[i].poll_block_upload(now) {
                if let Some(frame) = CanFrame::new(self.extra_sdo_tx[i], &seg_data) {
                    let _ = transport.transmit(&frame);
                }
            }
        }

        // RPDO deadline monitoring (CiA 301 event timer, comm param sub 5).
        // Only meaningful in Operational — outside it, PDO traffic
        // legitimately stops, so the monitoring state is discarded and
        // re-arms on the first reception after returning to Operational.
        if self.nmt.state() == NmtState::Operational {
            let mut expired = Vec::<PdoNumber, RPDO>::new();
            self.rpdo.check_deadlines(now, &mut expired);
            for &number in &expired {
                if self.event_queue.is_full() {
                    let _ = self.event_queue.pop_front();
                    self.event_overflow_count = self.event_overflow_count.saturating_add(1);
                }
                let _ = self.event_queue.push_back(OdEvent {
                    index: 0x1400 + number.od_offset(),
                    subindex: 0,
                    source: crate::od::OdEventSource::RpdoDeadline,
                });
            }
        } else {
            self.rpdo.reset_deadline_monitoring();
        }

        // Heartbeat production
        if let Some(hb) = self.heartbeat.poll(now, self.nmt.state()) {
            let _ = transport.transmit(&hb);
        }

        // Heartbeat consumer timeouts (0x1016). Unlike RPDO deadlines this is
        // not gated on Operational: error control stays active in every
        // post-boot NMT state.
        for monitor in self.hb_monitors.iter_mut().flatten() {
            if let Some(evt) = monitor.check_timeout(now) {
                if self.hb_event_queue.is_full() {
                    let _ = self.hb_event_queue.pop_front();
                    self.event_overflow_count = self.event_overflow_count.saturating_add(1);
                }
                let _ = self.hb_event_queue.push_back(evt);
            }
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
        if !self.event_queue.is_empty() || !self.hb_event_queue.is_empty() {
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
            if let Some(config) = self.tpdo.config_slot_mut(n) {
                let off = pdo_od_offset(config.od_number, n);
                let comm_idx = 0x1800 + off;
                let map_idx = 0x1A00 + off;

                if self.od.read(comm_idx, 1, &mut buf4).is_ok() {
                    let raw = u32::from_le_bytes(buf4);
                    // COB-ID 0 means "not resolved yet" (predefined default
                    // pending) — never a valid PDO COB-ID, so treat as disabled.
                    config.enabled = (raw & 0x8000_0000) == 0 && (raw & 0x7FF) != 0;
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
            if let Some(config) = self.rpdo.config_slot_mut(n) {
                let off = pdo_od_offset(config.od_number, n);
                let comm_idx = 0x1400 + off;
                let map_idx = 0x1600 + off;

                if self.od.read(comm_idx, 1, &mut buf4).is_ok() {
                    let raw = u32::from_le_bytes(buf4);
                    config.enabled = (raw & 0x8000_0000) == 0 && (raw & 0x7FF) != 0;
                    config.cob_id = (raw & 0x7FF) as u16;
                }
                if self.od.read(comm_idx, 2, &mut buf4).is_ok() {
                    config.transmission_type = buf4[0];
                }
                if self.od.read(comm_idx, 5, &mut buf2).is_ok() {
                    config.deadline_ms = u16::from_le_bytes(buf2);
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

    /// Abort any in-progress transfer on every SDO server (default + extra).
    /// Used on NMT reset and external resync so a stale transfer on one channel
    /// cannot outlive a reset.
    fn abort_all_sdo_transfers(&mut self) {
        self.sdo_server.abort_transfer();
        for i in 0..SDO {
            self.extra_sdo[i].abort_transfer();
        }
    }

    /// Process an SDO request against the selected server (the default 0x1200
    /// channel or an additional 0x1201+ channel) and transmit the response on
    /// that server's tx COB-ID. All servers share the OD, event queue, and
    /// config-resync path; only the transfer state machine and COB-IDs differ.
    fn handle_sdo_request(
        &mut self,
        sel: SdoServerSel,
        frame: &CanFrame,
        transport: &mut impl embedded_can::nb::Can<Frame = CanFrame>,
        now: u64,
    ) {
        if frame.raw_dlc() < 8 {
            return;
        }
        let req: [u8; 8] = frame.data().try_into().unwrap();
        let mut resp = [0u8; 8];
        let state = self.nmt.state();
        let was_full = self.event_queue.is_full();

        // Disjoint field borrows: the selected server plus `self.od` /
        // `self.event_queue`. Kept in per-arm expressions so the borrow checker
        // sees the fields as separate.
        let ok = match sel {
            SdoServerSel::Default => self
                .sdo_server
                .process(&req, &mut self.od, &mut resp, &mut self.event_queue, state, now)
                .is_ok(),
            SdoServerSel::Extra(i) => self.extra_sdo[i]
                .process(&req, &mut self.od, &mut resp, &mut self.event_queue, state, now)
                .is_ok(),
        };

        if ok {
            let resp_cob = match sel {
                SdoServerSel::Default => CobId::sdo_tx(self.node_id).raw(),
                SdoServerSel::Extra(i) => self.extra_sdo_tx[i],
            };
            if let Some(resp_frame) = CanFrame::new(resp_cob, &resp) {
                let _ = transport.transmit(&resp_frame);
            }
        }
        if was_full && self.event_queue.is_full() {
            // Queue was full before and after — if an event was pushed, one was dropped
            self.event_overflow_count = self.event_overflow_count.saturating_add(1);
        }
        // Resync runtime services whose config lives in the OD. Signalled
        // directly by the SDO server rather than via the event queue: an
        // overflowing queue must not be able to swallow a config change.
        let committed = match sel {
            SdoServerSel::Default => self.sdo_server.take_committed_write(),
            SdoServerSel::Extra(i) => self.extra_sdo[i].take_committed_write(),
        };
        if let Some((index, _)) = committed {
            if (0x1400..=0x1BFF).contains(&index) {
                self.sync_pdo_from_od();
            }
            if index == 0x1016 || index == 0x1017 {
                self.sync_heartbeat_from_od(true);
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

        // Additional SDO servers (0x1201+) match by their configured rx COB-ID,
        // which is arbitrary and need not fall in the default SDO request range.
        // Check them first; the default server is handled in the parse below.
        for i in 0..SDO {
            if self.extra_sdo_rx[i] == frame.raw_id() {
                self.handle_sdo_request(SdoServerSel::Extra(i), frame, transport, now);
                return;
            }
        }

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
                                self.abort_all_sdo_transfers();
                                self.sync_pdo_from_od();
                                self.sync_heartbeat_from_od(false);
                            }
                            NmtTransition::ResetCommunication => {
                                // Reset only communication parameters (PDO, heartbeat).
                                // Application data in the OD is preserved.
                                self.booted = false;
                                self.abort_all_sdo_transfers();
                                self.sync_pdo_from_od();
                                self.sync_heartbeat_from_od(false);
                            }
                            _ => {}
                        }
                    }
                }
            }

            ParsedCobId::SdoRequest(node) if node == self.node_id => {
                self.handle_sdo_request(SdoServerSel::Default, frame, transport, now);
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
                        now,
                    );
                    self.event_overflow_count = self.event_overflow_count.saturating_add(dropped);
                }
            }

            ParsedCobId::Heartbeat(remote) => {
                if frame.raw_dlc() >= 1 {
                    let mut events = HeartbeatEventBuf::new();
                    for monitor in self.hb_monitors.iter_mut().flatten() {
                        if monitor.process(remote, frame.data()[0], now, &mut events) {
                            // 0x1016 validation rejects duplicate node ids,
                            // so at most one monitor matches.
                            break;
                        }
                    }
                    for evt in events {
                        if self.hb_event_queue.is_full() {
                            let _ = self.hb_event_queue.pop_front();
                            self.event_overflow_count = self.event_overflow_count.saturating_add(1);
                        }
                        let _ = self.hb_event_queue.push_back(evt);
                    }
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
    const SDO: usize,
    const EVT_QUEUE: usize,
    const DIRTY_SET: usize,
> {
    node: &'a mut Node<OD, TPDO, RPDO, SDO, EVT_QUEUE, DIRTY_SET>,
    snapshot: OD,
}

impl<
        OD: ObjectDictionary + Clone,
        const TPDO: usize,
        const RPDO: usize,
        const SDO: usize,
        const EVT_QUEUE: usize,
        const DIRTY_SET: usize,
    > core::ops::Deref for OdGuard<'_, OD, TPDO, RPDO, SDO, EVT_QUEUE, DIRTY_SET>
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
        const SDO: usize,
        const EVT_QUEUE: usize,
        const DIRTY_SET: usize,
    > core::ops::DerefMut for OdGuard<'_, OD, TPDO, RPDO, SDO, EVT_QUEUE, DIRTY_SET>
{
    fn deref_mut(&mut self) -> &mut OD {
        &mut self.node.od
    }
}

impl<
        OD: ObjectDictionary + Clone,
        const TPDO: usize,
        const RPDO: usize,
        const SDO: usize,
        const EVT_QUEUE: usize,
        const DIRTY_SET: usize,
    > Drop for OdGuard<'_, OD, TPDO, RPDO, SDO, EVT_QUEUE, DIRTY_SET>
{
    fn drop(&mut self) {
        // Collect changed (index, subindex) pairs first to avoid borrow conflict
        let mut changed: [(u16, u8); 32] = [(0, 0); 32];
        let mut count = 0usize;

        for n in 0..TPDO {
            if let Some(config) = self.node.tpdo.config_slot(n) {
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
/// code. Wraps the `Mutex<RefCell<Option<Node>>>` and OD event signal pattern
/// that Embassy firmware otherwise spells out at every access.
///
/// ```ignore
/// static NODE: SharedNode<MyOd, 1, 1> = SharedNode::new();
///
/// // Setup:
/// NODE.init(node);
///
/// // Anywhere:
/// NODE.with(|node| node.od_mut().button = 1);
///
/// // Async task:
/// NODE.wait_for_change().await;
/// ```
#[cfg(feature = "embassy")]
pub struct SharedNode<
    OD: ObjectDictionary,
    const TPDO: usize = 4,
    const RPDO: usize = 4,
    const SDO: usize = 0,
    const EVT_QUEUE: usize = 16,
    const DIRTY_SET: usize = 8,
> {
    inner: embassy_sync::blocking_mutex::Mutex<
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        core::cell::RefCell<Option<Node<OD, TPDO, RPDO, SDO, EVT_QUEUE, DIRTY_SET>>>,
    >,
    event_signal: OdEventSignal,
}

#[cfg(feature = "embassy")]
impl<
        OD: ObjectDictionary,
        const TPDO: usize,
        const RPDO: usize,
        const SDO: usize,
        const EVT_QUEUE: usize,
        const DIRTY_SET: usize,
    > SharedNode<OD, TPDO, RPDO, SDO, EVT_QUEUE, DIRTY_SET>
{
    pub const fn new() -> Self {
        Self {
            inner: embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(None)),
            event_signal: OdEventSignal::new(),
        }
    }

    /// Store the node. Call once during setup, before any `with()`.
    ///
    /// `SharedNode` installs its internal OD event signal before storing the
    /// node, so application code can await [`wait_for_change`](Self::wait_for_change)
    /// without wiring a separate static signal by hand.
    pub fn init(&'static self, mut node: Node<OD, TPDO, RPDO, SDO, EVT_QUEUE, DIRTY_SET>) {
        node.set_event_signal(&self.event_signal);
        self.inner.lock(|cell| {
            cell.borrow_mut().replace(node);
        });
    }

    /// Wait until the node signals that OD-related work may be available.
    ///
    /// This is a wake signal, not a queue. Drain changes with
    /// [`Node::next_change`] after this returns. It can also wake for
    /// application-side `od_mut()` changes so the protocol task can process
    /// event-driven TPDOs promptly.
    pub async fn wait_for_change(&'static self) {
        self.event_signal.wait().await;
    }

    /// Run `f` with exclusive access to the node (inside a critical section —
    /// keep it short).
    ///
    /// Panics if called before [`init`](Self::init).
    pub fn with<R>(
        &self,
        f: impl FnOnce(&mut Node<OD, TPDO, RPDO, SDO, EVT_QUEUE, DIRTY_SET>) -> R,
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
        const SDO: usize,
        const EVT_QUEUE: usize,
        const DIRTY_SET: usize,
    > Default for SharedNode<OD, TPDO, RPDO, SDO, EVT_QUEUE, DIRTY_SET>
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
    use crate::pdo::{PdoMapping, PDO_MAX_MAPPINGS};
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
            sdo_servers: [],
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
            sdo_servers: [],
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
        let mut rpdo_mappings = heapless::Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
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

        let mut tpdo_mappings = heapless::Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
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
                od_number: 0,
                cob_id: 0x181,
                transmission_type: 255,
                inhibit_time_100us: 0,
                event_timer_ms: 0, // no timer, only dirty-triggered
                mappings: tpdo_mappings,
                enabled: true,
            }],
            rpdo: [RpdoConfig {
                od_number: 0,
                cob_id: 0x201,
                transmission_type: 255,
                deadline_ms: 0,
                mappings: rpdo_mappings,
                enabled: true,
            }],
            sdo_servers: [],
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

    fn make_deadline_node() -> (Node<EventTestOd, 1, 1>, MailboxTransport<32, 32>) {
        let mut rpdo_mappings = heapless::Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        rpdo_mappings
            .push(PdoMapping {
                index: 0x6200,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();

        let config = NodeConfig::<1, 1> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 0,
            auto_start: true,
            tpdo: [TpdoConfig::default()],
            rpdo: [RpdoConfig {
                od_number: 0,
                cob_id: 0x201,
                transmission_type: 255,
                deadline_ms: 100,
                mappings: rpdo_mappings,
                enabled: true,
            }],
            sdo_servers: [],
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
        node.process(&mut transport, &TestClock(0));
        while transport.next_to_transmit().is_some() {}
        assert_eq!(node.state(), NmtState::Operational);
        (node, transport)
    }

    #[test]
    fn placeholder_pdos_not_addressable_and_unresolved_cob_id_is_none() {
        let mut rpdo_mappings = heapless::Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        rpdo_mappings
            .push(PdoMapping {
                index: 0x6200,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();

        let config = NodeConfig::<1, 2> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 0,
            auto_start: false,
            // TPDO slot is a pure placeholder; RPDO slot 1 too.
            tpdo: [TpdoConfig::default()],
            rpdo: [
                RpdoConfig {
                    od_number: 0,
                    cob_id: 0x201,
                    transmission_type: 255,
                    deadline_ms: 0,
                    mappings: rpdo_mappings,
                    enabled: true,
                },
                RpdoConfig::default(),
            ],
            sdo_servers: [],
            identity: LssIdentity::default(),
        };
        let od = EventTestOd {
            device_type: 0x191,
            output1: 0,
            output2: 0,
            input1: 0,
        };
        let node: Node<EventTestOd, 1, 2> = Node::new(config, od);

        // Declared RPDO1 is addressable; the padding slot is not "RPDO2".
        assert_eq!(node.rpdo_cob_id(PdoNumber::of::<1>()).unwrap().raw(), 0x201);
        assert!(node.rpdo_cob_id(PdoNumber::of::<2>()).is_none());
        assert!(!node.rpdo_deadline_expired(PdoNumber::of::<2>()));
        // Placeholder TPDO slot is not "TPDO1" either.
        assert!(node.tpdo_cob_id(PdoNumber::of::<1>()).is_none());
    }

    #[test]
    fn rpdo_deadline_initially_expired_without_events() {
        let (mut node, mut transport) = make_deadline_node();

        // Immediately after going Operational, before any reception: the
        // level flag reads true (no in-deadline data exists), but no event
        // is queued — the edge channel arms on first reception.
        assert!(node.rpdo_deadline_expired(PdoNumber::of::<1>()));
        assert!(node.next_event().is_none());

        // Still true and still event-free after arbitrary silence.
        node.process(&mut transport, &TestClock(60_000_000));
        assert!(node.rpdo_deadline_expired(PdoNumber::of::<1>()));
        assert!(node.next_event().is_none());

        // First reception clears the flag.
        transport
            .store_received(CanFrame::new(0x201, &[0x01]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(60_010_000));
        assert!(!node.rpdo_deadline_expired(PdoNumber::of::<1>()));
    }

    #[test]
    fn rpdo_deadline_fires_event_and_flag_after_silence() {
        let (mut node, mut transport) = make_deadline_node();

        // Silence before the first reception: level flag up, no events.
        node.process(&mut transport, &TestClock(10_000_000));
        assert!(node.rpdo_deadline_expired(PdoNumber::of::<1>()));
        assert!(node.next_event().is_none());

        // First reception arms monitoring and clears the flag.
        transport
            .store_received(CanFrame::new(0x201, &[0x55]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(10_000_000));
        assert!(!node.rpdo_deadline_expired(PdoNumber::of::<1>()));
        let evt = node.next_event().unwrap();
        assert_eq!(evt.source, OdEventSource::Rpdo);

        // 150 ms of silence > 100 ms deadline: flag + one event, value kept.
        node.process(&mut transport, &TestClock(10_150_000));
        assert!(node.rpdo_deadline_expired(PdoNumber::of::<1>()));
        assert_eq!(node.od().output1, 0x55);
        let evt = node.next_event().unwrap();
        assert_eq!(evt.index, 0x1400);
        assert_eq!(evt.subindex, 0);
        assert_eq!(evt.source, OdEventSource::RpdoDeadline);

        // Edge-triggered: continued silence produces no further events.
        node.process(&mut transport, &TestClock(10_300_000));
        assert!(node.next_event().is_none());
        assert!(node.rpdo_deadline_expired(PdoNumber::of::<1>()));

        // Reception clears the flag and re-arms; a second silence fires again.
        transport
            .store_received(CanFrame::new(0x201, &[0x66]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(10_400_000));
        assert!(!node.rpdo_deadline_expired(PdoNumber::of::<1>()));
        assert_eq!(node.next_event().unwrap().source, OdEventSource::Rpdo);

        node.process(&mut transport, &TestClock(10_600_000));
        assert!(node.rpdo_deadline_expired(PdoNumber::of::<1>()));
        assert_eq!(
            node.next_event().unwrap().source,
            OdEventSource::RpdoDeadline
        );
    }

    #[test]
    fn rpdo_deadline_cleared_when_leaving_operational() {
        let (mut node, mut transport) = make_deadline_node();

        // Arm monitoring.
        transport
            .store_received(CanFrame::new(0x201, &[0x77]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(0));
        while node.next_event().is_some() {}

        // NMT Enter Pre-Operational: PDO traffic legitimately stops. The
        // edge state is discarded; the level flag returns to "no fresh data".
        transport
            .store_received(CanFrame::new(0x000, &[0x80, 0x01]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(50_000));
        assert_eq!(node.state(), NmtState::PreOperational);

        // Long silence outside Operational: flag up, but no expiry events.
        node.process(&mut transport, &TestClock(10_000_000));
        assert!(node.rpdo_deadline_expired(PdoNumber::of::<1>()));
        assert!(node.next_event().is_none());

        // Back to Operational: edge channel stays disarmed (no events) until
        // the next frame; the level flag stays up meanwhile.
        transport
            .store_received(CanFrame::new(0x000, &[0x01, 0x01]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(10_050_000));
        assert_eq!(node.state(), NmtState::Operational);
        node.process(&mut transport, &TestClock(20_000_000));
        assert!(node.rpdo_deadline_expired(PdoNumber::of::<1>()));
        assert!(node.next_event().is_none());

        // A frame restores freshness.
        transport
            .store_received(CanFrame::new(0x201, &[0x78]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(20_010_000));
        assert!(!node.rpdo_deadline_expired(PdoNumber::of::<1>()));
    }

    #[test]
    fn rpdo_deadline_event_index_does_not_truncate_slots_above_255() {
        let mut rpdo: [RpdoConfig; 257] = core::array::from_fn(|_| RpdoConfig::default());

        let mut mappings = heapless::Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                index: 0x6200,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();

        // Slot 256 is RPDO257, whose comm parameter index is 0x1500.
        rpdo[256] = RpdoConfig {
            od_number: 257,
            cob_id: 0x201,
            transmission_type: 255,
            deadline_ms: 100,
            mappings,
            enabled: true,
        };

        let config = NodeConfig::<0, 257> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 0,
            auto_start: true,
            tpdo: [],
            rpdo,
            sdo_servers: [],
            identity: LssIdentity::default(),
        };

        let od = EventTestOd {
            device_type: 0x191,
            output1: 0,
            output2: 0,
            input1: 0,
        };
        let mut node: Node<EventTestOd, 0, 257> = Node::new(config, od);
        let mut transport = MailboxTransport::<32, 32>::new();

        node.process(&mut transport, &TestClock(0));
        while transport.next_to_transmit().is_some() {}

        transport
            .store_received(CanFrame::new(0x201, &[0x55]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(1_000));
        while node.next_event().is_some() {}

        node.process(&mut transport, &TestClock(102_000));

        let evt = node.next_event().unwrap();
        assert_eq!(evt.source, OdEventSource::RpdoDeadline);
        assert_eq!(evt.index, 0x1500);
        assert!(node.rpdo_deadline_expired(PdoNumber::of::<257>()));
    }

    #[test]
    fn event_queue_overflow_drops_oldest() {
        // Use a tiny queue of size 2
        let mut rpdo_mappings = heapless::Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
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
                od_number: 0,
                cob_id: 0x201,
                transmission_type: 255,
                deadline_ms: 0,
                mappings: rpdo_mappings,
                enabled: true,
            }],
            sdo_servers: [],
            identity: LssIdentity::default(),
        };
        let od = EventTestOd {
            device_type: 0x191,
            output1: 0,
            output2: 0,
            input1: 0,
        };
        // EVT_QUEUE = 2, so two RPDO events will fill it, then SDO will overflow
        let mut node: Node<EventTestOd, 1, 1, 0, 2, 8> = Node::new(config, od);
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
        let mut rpdo_mappings = heapless::Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
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
                od_number: 0,
                cob_id: 0x201,
                transmission_type: 255,
                deadline_ms: 0,
                mappings: rpdo_mappings,
                enabled: true,
            }],
            sdo_servers: [],
            identity: LssIdentity::default(),
        };
        let od = EventTestOd {
            device_type: 0x191,
            output1: 0,
            output2: 0,
            input1: 0,
        };
        let mut node: Node<EventTestOd, 1, 1, 0, 1, 8> = Node::new(config, od);
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
        let mut tpdo_mappings = heapless::Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
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
                od_number: 0,
                cob_id: 0x181,
                transmission_type: 255,
                inhibit_time_100us: 1000, // 100ms inhibit
                event_timer_ms: 0,
                mappings: tpdo_mappings,
                enabled: true,
            }],
            rpdo: [RpdoConfig::default()],
            sdo_servers: [],
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
        let mut tpdo_mappings = heapless::Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
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
                od_number: 0,
                cob_id: 0x181,
                transmission_type: 1, // sync cyclic, NOT event-driven
                inhibit_time_100us: 0,
                event_timer_ms: 0,
                mappings: tpdo_mappings,
                enabled: true,
            }],
            rpdo: [RpdoConfig::default()],
            sdo_servers: [],
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
            sdo_servers: [],
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

    // --- OD with 0x1017 producer + 0x1016 consumer entries ---

    #[derive(Clone)]
    struct HbOd {
        producer_time: u16,
        consumer: [u32; 3],
    }

    static HB_META: &[OdEntryMeta] = &[
        OdEntryMeta {
            index: 0x1016,
            subindex: 0,
            data_type: DataType::U8,
            access: AccessType::Ro,
            pdo_mappable: false,
            name: "consumer_heartbeat_count",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x1016,
            subindex: 1,
            data_type: DataType::U32,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "consumer_heartbeat_1",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x1016,
            subindex: 2,
            data_type: DataType::U32,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "consumer_heartbeat_2",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x1016,
            subindex: 3,
            data_type: DataType::U32,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "consumer_heartbeat_3",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x1017,
            subindex: 0,
            data_type: DataType::U16,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "producer_heartbeat_time",
            max_size: None,
        },
    ];

    impl ObjectDictionary for HbOd {
        fn lookup(&self, index: u16, subindex: u8) -> Option<&'static OdEntryMeta> {
            HB_META
                .iter()
                .find(|e| e.index == index && e.subindex == subindex)
        }
        fn read(&self, index: u16, subindex: u8, buf: &mut [u8]) -> Result<usize, OdError> {
            match (index, subindex) {
                (0x1017, 0) => {
                    buf[..2].copy_from_slice(&self.producer_time.to_le_bytes());
                    Ok(2)
                }
                (0x1016, 0) => {
                    buf[0] = 3;
                    Ok(1)
                }
                (0x1016, sub @ 1..=3) => {
                    buf[..4].copy_from_slice(&self.consumer[(sub as usize) - 1].to_le_bytes());
                    Ok(4)
                }
                _ => Err(OdError::NotFound),
            }
        }
        fn write(&mut self, index: u16, subindex: u8, data: &[u8]) -> Result<(), OdError> {
            match (index, subindex) {
                (0x1017, 0) if data.len() == 2 => {
                    self.producer_time = u16::from_le_bytes(data.try_into().unwrap());
                    Ok(())
                }
                (0x1016, sub @ 1..=3) if data.len() == 4 => {
                    self.consumer[(sub as usize) - 1] =
                        u32::from_le_bytes(data.try_into().unwrap());
                    Ok(())
                }
                (0x1016 | 0x1017, _) => Err(OdError::DataTypeMismatch),
                _ => Err(OdError::NotFound),
            }
        }
        fn sub_count(&self, index: u16) -> Option<u8> {
            match index {
                0x1016 => Some(3),
                _ => None,
            }
        }
    }

    fn make_hb_node(od: HbOd) -> (Node<HbOd, 1, 1>, MailboxTransport<32, 32>) {
        let config = NodeConfig::<1, 1> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 1000,
            auto_start: false,
            tpdo: [TpdoConfig::default()],
            rpdo: [RpdoConfig::default()],
            sdo_servers: [],
            identity: LssIdentity::default(),
        };
        (Node::new(config, od), MailboxTransport::<32, 32>::new())
    }

    /// Drain transmitted frames, returning the heartbeat state bytes seen.
    fn drain_heartbeats(transport: &MailboxTransport<32, 32>) -> heapless::Vec<u8, 8> {
        let mut out = heapless::Vec::new();
        while let Some(frame) = transport.next_to_transmit() {
            if frame.raw_id() == 0x701 {
                let _ = out.push(frame.data()[0]);
            }
        }
        out
    }

    #[test]
    fn producer_interval_from_od_wins_over_config() {
        // NodeConfig says 1000ms, OD 0x1017 says 100ms — OD wins.
        let (mut node, mut transport) = make_hb_node(HbOd {
            producer_time: 100,
            consumer: [0; 3],
        });
        assert_eq!(node.heartbeat_interval_ms(), 100);

        node.process(&mut transport, &TestClock(0)); // boot frame
        assert_eq!(drain_heartbeats(&transport), &[0x00]);

        node.process(&mut transport, &TestClock(50_000));
        assert!(drain_heartbeats(&transport).is_empty());

        node.process(&mut transport, &TestClock(100_000));
        assert_eq!(drain_heartbeats(&transport), &[0x7F]);
    }

    #[test]
    fn sdo_write_1017_changes_cadence() {
        let (mut node, mut transport) = make_hb_node(HbOd {
            producer_time: 1000,
            consumer: [0; 3],
        });
        node.process(&mut transport, &TestClock(0)); // boot
        while transport.next_to_transmit().is_some() {}

        // SDO expedited write: 0x1017:0 = 50ms
        let req = CanFrame::new(0x601, &[0x2B, 0x17, 0x10, 0x00, 50, 0, 0, 0]).unwrap();
        transport.store_received(req).unwrap();
        node.process(&mut transport, &TestClock(10_000));
        let resp = transport.next_to_transmit().unwrap();
        assert_eq!(resp.raw_id(), 0x581);
        assert_eq!(resp.data()[0], 0x60); // download response, not abort
        assert_eq!(node.heartbeat_interval_ms(), 50);

        // Old 1000ms cadence no longer applies; the next heartbeat is due
        // 50ms after the boot frame (last-sent timestamp), then every 50ms.
        node.process(&mut transport, &TestClock(20_000));
        assert!(drain_heartbeats(&transport).is_empty());
        node.process(&mut transport, &TestClock(50_000));
        assert_eq!(drain_heartbeats(&transport), &[0x7F]);
        node.process(&mut transport, &TestClock(70_000));
        assert!(drain_heartbeats(&transport).is_empty());
        node.process(&mut transport, &TestClock(100_000));
        assert_eq!(drain_heartbeats(&transport), &[0x7F]);
    }

    #[test]
    fn sdo_write_1017_zero_disables_producer() {
        let (mut node, mut transport) = make_hb_node(HbOd {
            producer_time: 100,
            consumer: [0; 3],
        });
        node.process(&mut transport, &TestClock(0)); // boot
        while transport.next_to_transmit().is_some() {}

        let req = CanFrame::new(0x601, &[0x2B, 0x17, 0x10, 0x00, 0, 0, 0, 0]).unwrap();
        transport.store_received(req).unwrap();
        node.process(&mut transport, &TestClock(10_000));
        let resp = transport.next_to_transmit().unwrap();
        assert_eq!(resp.data()[0], 0x60);
        assert_eq!(node.heartbeat_interval_ms(), 0);

        for t in [100_000u64, 500_000, 5_000_000, 60_000_000] {
            node.process(&mut transport, &TestClock(t));
            assert!(drain_heartbeats(&transport).is_empty());
        }
    }

    #[test]
    fn consumer_monitors_node_from_od() {
        let remote = NodeId::new(5).unwrap();
        // Monitor node 5 with 200ms consumer time
        let (mut node, mut transport) = make_hb_node(HbOd {
            producer_time: 0,
            consumer: [(5 << 16) | 200, 0, 0],
        });
        node.process(&mut transport, &TestClock(0)); // boot
        while transport.next_to_transmit().is_some() {}

        assert_eq!(
            node.heartbeat_status(remote),
            HeartbeatMonitorState::Waiting
        );
        // Nothing monitored for other nodes
        assert_eq!(
            node.heartbeat_status(NodeId::new(6).unwrap()),
            HeartbeatMonitorState::Disabled
        );

        // Configured-but-silent node never times out (monitoring not started)
        node.process(&mut transport, &TestClock(10_000_000));
        assert!(node.next_heartbeat_event().is_none());

        // First heartbeat starts monitoring
        let hb = CanFrame::new(0x705, &[0x05]).unwrap();
        transport.store_received(hb).unwrap();
        node.process(&mut transport, &TestClock(10_100_000));
        assert_eq!(
            node.next_heartbeat_event(),
            Some(HeartbeatEvent::Started {
                node: remote,
                state: NmtState::Operational
            })
        );
        assert!(node.heartbeat_status(remote).is_alive());

        // 150ms later: still fresh
        node.process(&mut transport, &TestClock(10_250_000));
        assert!(node.next_heartbeat_event().is_none());

        // 250ms after last heartbeat: timeout
        node.process(&mut transport, &TestClock(10_350_000));
        assert_eq!(
            node.next_heartbeat_event(),
            Some(HeartbeatEvent::Timeout { node: remote })
        );
        assert!(!node.heartbeat_status(remote).is_alive());

        // Heartbeat resumes: recovery
        let hb = CanFrame::new(0x705, &[0x05]).unwrap();
        transport.store_received(hb).unwrap();
        node.process(&mut transport, &TestClock(10_400_000));
        assert_eq!(
            node.next_heartbeat_event(),
            Some(HeartbeatEvent::Recovered {
                node: remote,
                state: NmtState::Operational
            })
        );
        assert!(node.heartbeat_status(remote).is_alive());
    }

    #[test]
    fn consumer_configured_at_runtime_via_sdo() {
        let remote = NodeId::new(9).unwrap();
        let (mut node, mut transport) = make_hb_node(HbOd {
            producer_time: 0,
            consumer: [0; 3],
        });
        node.process(&mut transport, &TestClock(0)); // boot
        while transport.next_to_transmit().is_some() {}
        assert_eq!(
            node.heartbeat_status(remote),
            HeartbeatMonitorState::Disabled
        );

        // 0x1016:1 = node 9, 100ms
        let value: u32 = (9 << 16) | 100;
        let mut req = [0x23, 0x16, 0x10, 0x01, 0, 0, 0, 0];
        req[4..8].copy_from_slice(&value.to_le_bytes());
        transport
            .store_received(CanFrame::new(0x601, &req).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(10_000));
        let resp = transport.next_to_transmit().unwrap();
        assert_eq!(resp.data()[0], 0x60);
        assert_eq!(
            node.heartbeat_status(remote),
            HeartbeatMonitorState::Waiting
        );

        // Boot-up frame also starts monitoring
        transport
            .store_received(CanFrame::new(0x709, &[0x00]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(20_000));
        assert_eq!(
            node.next_heartbeat_event(),
            Some(HeartbeatEvent::Started {
                node: remote,
                state: NmtState::Initializing
            })
        );

        // Rewriting the same value must not reset the running monitor
        transport
            .store_received(CanFrame::new(0x601, &req).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(30_000));
        let resp = transport.next_to_transmit().unwrap();
        assert_eq!(resp.data()[0], 0x60);
        assert!(node.heartbeat_status(remote).is_alive());

        // Writing 0 disables the monitor
        let req0 = [0x23, 0x16, 0x10, 0x01, 0, 0, 0, 0];
        transport
            .store_received(CanFrame::new(0x601, &req0).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(40_000));
        let resp = transport.next_to_transmit().unwrap();
        assert_eq!(resp.data()[0], 0x60);
        assert_eq!(
            node.heartbeat_status(remote),
            HeartbeatMonitorState::Disabled
        );
    }

    /// Expect an SDO abort with the given code in response to a 0x1016 write.
    fn expect_1016_abort(value: u32, subindex: u8, existing: [u32; 3], code: u32) {
        let (mut node, mut transport) = make_hb_node(HbOd {
            producer_time: 0,
            consumer: existing,
        });
        node.process(&mut transport, &TestClock(0));
        while transport.next_to_transmit().is_some() {}

        let mut req = [0x23, 0x16, 0x10, subindex, 0, 0, 0, 0];
        req[4..8].copy_from_slice(&value.to_le_bytes());
        transport
            .store_received(CanFrame::new(0x601, &req).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(10_000));
        let resp = transport.next_to_transmit().unwrap();
        assert_eq!(
            resp.data()[0],
            0x80,
            "expected abort for value {value:#010X}"
        );
        assert_eq!(
            u32::from_le_bytes(resp.data()[4..8].try_into().unwrap()),
            code
        );
    }

    #[test]
    fn invalid_1016_writes_abort() {
        // Node id 0 with non-zero time
        expect_1016_abort(100, 1, [0; 3], 0x0609_0030);
        // Node id > 127
        expect_1016_abort((200 << 16) | 100, 1, [0; 3], 0x0609_0030);
        // Reserved bits set
        expect_1016_abort(0xFF00_0000 | (5 << 16) | 100, 1, [0; 3], 0x0609_0030);
        // Reserved bits are rejected even on a disabled entry (time 0)
        expect_1016_abort(0x0100_0000, 1, [0; 3], 0x0609_0030);
        // Same node monitored by another active entry
        expect_1016_abort((5 << 16) | 300, 2, [(5 << 16) | 200, 0, 0], 0x0604_0043);
    }

    #[test]
    fn valid_1016_overwrite_of_same_slot_is_not_a_duplicate() {
        // Changing the timeout of the entry that already monitors the node
        // must be allowed (the duplicate check skips the written subindex).
        let remote = NodeId::new(5).unwrap();
        let (mut node, mut transport) = make_hb_node(HbOd {
            producer_time: 0,
            consumer: [(5 << 16) | 200, 0, 0],
        });
        node.process(&mut transport, &TestClock(0));
        while transport.next_to_transmit().is_some() {}

        let value: u32 = (5 << 16) | 500;
        let mut req = [0x23, 0x16, 0x10, 0x01, 0, 0, 0, 0];
        req[4..8].copy_from_slice(&value.to_le_bytes());
        transport
            .store_received(CanFrame::new(0x601, &req).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(10_000));
        let resp = transport.next_to_transmit().unwrap();
        assert_eq!(resp.data()[0], 0x60);
        assert_eq!(
            node.heartbeat_status(remote),
            HeartbeatMonitorState::Waiting
        );
    }

    #[test]
    fn config_resync_survives_full_event_queue() {
        // The runtime resync is signalled directly by the SDO server; a full
        // OD event queue (default EVT_QUEUE = 16, oldest evicted on push)
        // must not swallow a 0x1017 config change.
        let (mut node, mut transport) = make_hb_node(HbOd {
            producer_time: 1000,
            consumer: [0; 3],
        });
        node.process(&mut transport, &TestClock(0)); // boot
        while transport.next_to_transmit().is_some() {}

        // Fill the event queue past capacity without draining: 20 distinct
        // writes to 0x1016:1 (rewriting the same slot is always valid).
        for i in 0..20u32 {
            let value: u32 = (9 << 16) | (100 + i);
            let mut req = [0x23, 0x16, 0x10, 0x01, 0, 0, 0, 0];
            req[4..8].copy_from_slice(&value.to_le_bytes());
            transport
                .store_received(CanFrame::new(0x601, &req).unwrap())
                .unwrap();
            node.process(&mut transport, &TestClock(1_000 + i as u64));
            while transport.next_to_transmit().is_some() {}
        }
        assert!(node.events_dropped() > 0, "queue should have overflowed");

        // Queue is full — the 0x1017 write must still reach the producer.
        let req = CanFrame::new(0x601, &[0x2B, 0x17, 0x10, 0x00, 77, 0, 0, 0]).unwrap();
        transport.store_received(req).unwrap();
        node.process(&mut transport, &TestClock(50_000));
        let resp = transport.next_to_transmit().unwrap();
        assert_eq!(resp.data()[0], 0x60);
        assert_eq!(node.heartbeat_interval_ms(), 77);
    }

    #[test]
    fn local_reset_restarts_heartbeat_monitors() {
        // SDO rewrites of an unchanged entry keep monitor state, but a local
        // reset reinitializes: monitoring starts with the first heartbeat
        // again.
        let remote = NodeId::new(5).unwrap();
        let (mut node, mut transport) = make_hb_node(HbOd {
            producer_time: 0,
            consumer: [(5 << 16) | 200, 0, 0],
        });
        node.process(&mut transport, &TestClock(0)); // boot
        while transport.next_to_transmit().is_some() {}

        transport
            .store_received(CanFrame::new(0x705, &[0x05]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(10_000));
        assert!(node.heartbeat_status(remote).is_alive());

        node.request_reset(ResetType::Communication);
        assert_eq!(
            node.heartbeat_status(remote),
            HeartbeatMonitorState::Waiting
        );

        // No stale timeout after the reset — the monitor is waiting again.
        node.process(&mut transport, &TestClock(20_000)); // re-boot
        while transport.next_to_transmit().is_some() {}
        node.process(&mut transport, &TestClock(10_000_000));
        while let Some(evt) = node.next_heartbeat_event() {
            assert!(
                !matches!(evt, HeartbeatEvent::Timeout { .. }),
                "timeout fired from pre-reset monitor state"
            );
        }
    }
}
