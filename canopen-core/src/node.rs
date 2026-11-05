use crate::cobid::{CobId, NodeId};
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
use crate::pdo::{
    PdoMapping, PdoNumber, RpdoConfig, RpdoEngine, TpdoConfig, TpdoEngine, PDO_MAX_MAPPINGS,
};
use crate::sdo::{ReservedCobIds, SdoCobIds, SdoContext, SdoServer, SdoServerConfig};
use crate::sync::{SyncCobId, SyncConsumer, SyncProducer, SyncWindow};
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
        od: &(impl crate::pdo::PdoConfigSource<TPDO, RPDO> + crate::sdo::SdoServerConfigSource<SDO>),
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
    sdo_tx: PendingSdoTx,
    /// Additional SDO servers (0x1201+), each an independent transfer state
    /// machine. `SDO` is 0 for the common single-server node.
    extra_sdo: [SdoServer; SDO],
    extra_sdo_pending_tx: [PendingSdoTx; SDO],
    /// Resolved rx COB-ID (client→server) for each additional SDO server,
    /// parallel to `extra_sdo`. Dispatch matches incoming frames against these.
    extra_sdo_rx: [u16; SDO],
    /// Resolved tx COB-ID (server→client) for each additional SDO server.
    extra_sdo_tx: [u16; SDO],
    tpdo: TpdoEngine<TPDO>,
    rpdo: RpdoEngine<RPDO>,
    /// SYNC consumer on the OD 0x1005 COB-ID (0x080 unless relocated).
    sync: SyncConsumer,
    /// SYNC producer, active only when 0x1005 bit 30 and 0x1006 say so.
    sync_producer: SyncProducer,
    /// OD 0x1007 synchronous-PDO transmission window.
    sync_window: SyncWindow,
    /// Synchronous TPDOs refused by a full transport, retained only until the
    /// current window closes or the next SYNC supersedes them.
    pending_sync_tpdo: Vec<CanFrame, TPDO>,
    /// Avoid retrying a refused TPDO twice at the same clock instant (the
    /// pulse path already made the first attempt).
    sync_tpdo_last_attempt_us: Option<u64>,
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
    reset_event: Option<ResetType>,
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

/// Capacity of the reserved-COB-ID list handed to the SDO server so it can
/// reject a 0x1005 download that would move SYNC onto another service.
///
/// Holds EMCY, this node's own heartbeat, LSS request/response, every declared
/// PDO and every heartbeat consumer — enough for 20 PDOs alongside the full 8
/// monitors, comfortably past the 4 + 4 of the pre-defined connection set. A
/// node declaring more than that cannot be fully checked, and the server then
/// refuses to relocate SYNC at all rather than allow a collision it could not
/// see (see [`ReservedCobIds::truncated`]).
const RESERVED_COB_IDS: usize = 32;

/// Selects which SDO server an incoming request is dispatched to.
#[derive(Clone, Copy)]
enum SdoServerSel {
    /// The default server (0x1200, `0x600/0x580 + node_id`).
    Default,
    /// An additional server (0x1201+); index into `extra_sdo`.
    Extra(usize),
}

/// One protocol-owned response, not a general outgoing FIFO. The residence
/// limit also covers expedited responses, which have no active transfer timer.
#[derive(Clone, Copy, Default)]
struct PendingSdoTx {
    frame: Option<(CanFrame, u64)>,
    refused_this_poll: bool,
}

impl PendingSdoTx {
    fn begin_poll(&mut self, server: &mut SdoServer, cob: u16, now: u64) {
        self.refused_this_poll = false;
        if self
            .frame
            .is_some_and(|(_, since)| now.wrapping_sub(since) >= crate::sdo::server::SDO_TIMEOUT_US)
        {
            // Do not replay a stale response, even if the transport recovers now.
            self.frame = None;
            server.abort_transfer();
        } else if let Some(abort) = server.check_timeout(now) {
            self.queue(cob, abort, now);
        }
    }

    fn queue(&mut self, cob: u16, data: [u8; 8], now: u64) {
        self.frame = CanFrame::new(cob, &data).map(|frame| (frame, now));
    }

    fn try_send(&mut self, transport: &mut impl embedded_can::nb::Can<Frame = CanFrame>) {
        if self.refused_this_poll {
            return;
        }
        if let Some((frame, _)) = self.frame {
            match transport.transmit(&frame) {
                Ok(_) => self.frame = None,
                Err(_) => self.refused_this_poll = true,
            }
        }
    }

    fn service(
        &mut self,
        server: &mut SdoServer,
        cob: u16,
        transport: &mut impl embedded_can::nb::Can<Frame = CanFrame>,
        now: u64,
    ) {
        self.try_send(transport);
        // At most one sub-block per poll; never advance beyond a refused frame.
        for _ in 0..127 {
            if self.frame.is_some() {
                break;
            }
            let Some(data) = server.poll_block_upload(now) else {
                break;
            };
            self.queue(cob, data, now);
            self.try_send(transport);
        }
    }
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
            sdo_tx: PendingSdoTx::default(),
            extra_sdo: [const { SdoServer::new() }; SDO],
            extra_sdo_pending_tx: [PendingSdoTx::default(); SDO],
            extra_sdo_rx,
            extra_sdo_tx,
            tpdo: TpdoEngine::new(config.tpdo),
            rpdo: RpdoEngine::new(config.rpdo),
            sync: SyncConsumer::default_cob(),
            sync_producer: SyncProducer::new(),
            sync_window: SyncWindow::new(),
            pending_sync_tpdo: Vec::new(),
            sync_tpdo_last_attempt_us: None,
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
            reset_event: None,
            #[cfg(feature = "embassy")]
            event_signal: None,
        };
        node.write_pdo_cob_ids_to_od();
        // Mirror resolved additional-SDO-server COB-IDs into the OD (0x1201+)
        // so SDO reads return real values (node-relative resolved).
        node.od.store_sdo_server_cob_ids(config.node_id);
        node.sync_heartbeat_from_od(false);
        // Before the assert below: the SYNC COB-ID it checks against is the
        // configured one, not necessarily 0x080.
        node.sync_config_from_od();
        // A hard invariant, not a debug-only check: colliding COB-IDs make the
        // dispatcher steal frames from the default server (extras are matched
        // first) or answer on the wrong COB-ID. The config is fixed at build
        // time, so a violation is a deterministic programming error — fail fast
        // in every build rather than silently misroute SDO traffic in release.
        // SDO-vs-SDO collisions are mostly rejected at compile time by the
        // macro; this catches the node-id-dependent residue (an absolute COB-ID
        // landing on the default or another server for this particular node id)
        // plus everything the macro cannot see at all: PDO COB-IDs and
        // heartbeat consumers come from the OD and are only known here.
        assert!(
            node.sdo_cob_ids_are_unique(),
            "SDO server COB-IDs collide with another configured \
             CANopen service or are zero (for node id {})",
            config.node_id.raw()
        );
        // Same class of invariant, same fail-fast reasoning: SYNC is
        // dispatched before every service below it, so a collision here would
        // silently mute a PDO, a heartbeat consumer or LSS.
        assert!(
            node.sync_cob_id_is_unique(),
            "the SYNC COB-ID (OD 0x1005) collides with another configured \
             CANopen service on this node (node id {})",
            config.node_id.raw()
        );
        node
    }

    /// Every SDO server rx/tx COB-ID must be distinct from every COB-ID another
    /// configured service on this node already uses: NMT, SYNC, EMCY, own and
    /// monitored heartbeats, LSS, and the declared PDOs. Additional SDO server
    /// IDs must also be non-zero and distinct from the default and each other.
    /// Enforced by an assert in [`Node::new`] (see there).
    ///
    /// This is the static half of the invariant; the runtime half lives in the
    /// SDO server's download validation (see [`SdoCobIds`]), which stops a
    /// client from moving a PDO or heartbeat consumer onto an SDO channel after
    /// boot. Disabled PDOs are checked too — a COB-ID that is merely parked
    /// today can be enabled by a later write to sub 1.
    fn sdo_cob_ids_are_unique(&self) -> bool {
        let def_rx = CobId::sdo_rx(self.node_id).raw();
        let def_tx = CobId::sdo_tx(self.node_id).raw();
        let sdo_cobs = SdoCobIds {
            default_rx: def_rx,
            default_tx: def_tx,
            extra_rx: &self.extra_sdo_rx,
            extra_tx: &self.extra_sdo_tx,
        };
        let fixed = [
            CobId::nmt().raw(),
            // The configured SYNC COB-ID (0x080 by default); 0 when the OD
            // describes no usable SYNC object, which claims nothing because
            // the rx/tx != 0 check above already ran.
            self.sync.cob_id().unwrap_or(0),
            CobId::emergency(self.node_id).raw(),
            def_rx,
            def_tx,
            CobId::heartbeat(self.node_id).raw(),
            crate::lss::LSS_REQUEST_COB,
            crate::lss::LSS_RESPONSE_COB,
        ];
        for i in 0..SDO {
            let (rx, tx) = (self.extra_sdo_rx[i], self.extra_sdo_tx[i]);
            if rx == 0 || tx == 0 || rx == tx {
                return false;
            }
            if fixed.contains(&rx) || fixed.contains(&tx) {
                return false;
            }
            for j in (i + 1)..SDO {
                let (orx, otx) = (self.extra_sdo_rx[j], self.extra_sdo_tx[j]);
                if rx == orx || rx == otx || tx == orx || tx == otx {
                    return false;
                }
            }
        }
        for slot in 0..TPDO {
            if let Some(config) = self.tpdo.config_slot(slot) {
                if sdo_cobs.claims(config.cob_id) {
                    return false;
                }
            }
        }
        for slot in 0..RPDO {
            if let Some(config) = self.rpdo.config_slot(slot) {
                if sdo_cobs.claims(config.cob_id) {
                    return false;
                }
            }
        }
        for monitor in self.hb_monitors.iter().flatten() {
            if sdo_cobs.claims(CobId::heartbeat(monitor.node_id()).raw()) {
                return false;
            }
        }
        true
    }

    /// COB-IDs this node's non-SDO services occupy, for the SYNC relocation
    /// guard. Parked (disabled) PDOs are included: a COB-ID that is merely
    /// unused today can be enabled later without being rewritten.
    ///
    /// The bool is false when the list overflowed [`RESERVED_COB_IDS`] and the
    /// remaining COB-IDs are therefore unknown.
    fn reserved_cob_ids(&self) -> (Vec<u16, RESERVED_COB_IDS>, bool) {
        let fixed = [
            CobId::emergency(self.node_id).raw(),
            CobId::heartbeat(self.node_id).raw(),
            crate::lss::LSS_REQUEST_COB,
            crate::lss::LSS_RESPONSE_COB,
        ];
        let pdos = (0..TPDO)
            .filter_map(|slot| self.tpdo.config_slot(slot).map(|c| c.cob_id))
            .chain((0..RPDO).filter_map(|slot| self.rpdo.config_slot(slot).map(|c| c.cob_id)));
        let monitored = self
            .hb_monitors
            .iter()
            .flatten()
            .map(|m| CobId::heartbeat(m.node_id()).raw());

        let mut out = Vec::new();
        let mut complete = true;
        for cob in fixed.into_iter().chain(pdos).chain(monitored) {
            if cob != 0 && !out.contains(&cob) && out.push(cob).is_err() {
                complete = false;
            }
        }
        (out, complete)
    }

    /// The configured SYNC COB-ID must not be one another CANopen service on
    /// this node already uses.
    ///
    /// [`dispatch_frame`](Self::dispatch_frame) matches SYNC ahead of
    /// everything but the SDO channels and returns unconditionally, so a
    /// collision silently mutes the other service. Enforced by an assert in
    /// [`Node::new`] for the static config; the runtime half is the SDO
    /// server's 0x1005 download validation, which checks the same list.
    fn sync_cob_id_is_unique(&self) -> bool {
        let Some(sync) = self.sync.cob_id() else {
            return true; // no SYNC object, nothing to collide with
        };
        let sdo_cobs = SdoCobIds {
            default_rx: CobId::sdo_rx(self.node_id).raw(),
            default_tx: CobId::sdo_tx(self.node_id).raw(),
            extra_rx: &self.extra_sdo_rx,
            extra_tx: &self.extra_sdo_tx,
        };
        if sync == CobId::nmt().raw() || sdo_cobs.claims(sync) {
            return false;
        }
        // Built directly here rather than via `reserved_cob_ids`: at
        // construction there is no capacity limit to work around, so the check
        // is exact however many PDOs the node declares.
        if sync == CobId::emergency(self.node_id).raw()
            || sync == CobId::heartbeat(self.node_id).raw()
            || sync == crate::lss::LSS_REQUEST_COB
            || sync == crate::lss::LSS_RESPONSE_COB
        {
            return false;
        }
        for slot in 0..TPDO {
            if self
                .tpdo
                .config_slot(slot)
                .is_some_and(|c| c.cob_id == sync)
            {
                return false;
            }
        }
        for slot in 0..RPDO {
            if self
                .rpdo
                .config_slot(slot)
                .is_some_and(|c| c.cob_id == sync)
            {
                return false;
            }
        }
        !self
            .hb_monitors
            .iter()
            .flatten()
            .any(|m| CobId::heartbeat(m.node_id()).raw() == sync)
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

    /// Re-read the SYNC objects from the OD: 0x1005 (COB-ID SYNC message),
    /// 0x1006 (communication cycle period, µs), 0x1007 (synchronous window
    /// length, µs) and 0x1019 (synchronous counter overflow value). Called at
    /// init, after SDO writes to those entries, and on reset — the same live
    /// wiring 0x1017 gets.
    ///
    /// An OD that declares none of them keeps the CiA 301 defaults: consume
    /// SYNC on 0x080, produce none. Declaring 0x1006 alone does *not* start
    /// production — CiA 301 gates that on 0x1005 bit 30, so a node can be told
    /// the cycle period (to size its own timing) without becoming the master.
    /// Invalid or unsupported static 0x1005 values and invalid static 0x1019
    /// values are programming errors and panic, matching the hard COB-ID
    /// invariants checked by [`Node::new`]. SDO writes are aborted before they
    /// can place such values in the OD.
    fn sync_config_from_od(&mut self) {
        let mut buf4 = [0u8; 4];

        let sync_obj = if self.od.read(0x1005, 0, &mut buf4).is_ok() {
            let raw = u32::from_le_bytes(buf4);
            assert!(
                crate::sync::is_supported_sync_cob_id(raw),
                "OD 0x1005 contains an invalid or unsupported SYNC COB-ID value ({raw:#010X})"
            );
            // Checked above; keep the decoder fallible for callers that parse
            // untrusted values outside Node construction.
            SyncCobId::from_raw(raw)
        } else {
            Some(SyncCobId::DEFAULT)
        };
        // The configured value was checked above; only a missing object takes
        // the default path.
        let (cob_id, produce) = match sync_obj {
            Some(s) => (s.cob_id, s.produce),
            None => unreachable!(),
        };
        self.sync.set_cob_id(cob_id);

        let period_us = if self.od.read(0x1006, 0, &mut buf4).is_ok() {
            u32::from_le_bytes(buf4) as u64
        } else {
            0
        };
        let window_us = if self.od.read(0x1007, 0, &mut buf4).is_ok() {
            u32::from_le_bytes(buf4) as u64
        } else {
            0
        };
        self.sync_window.configure(window_us);
        let mut buf1 = [0u8; 1];
        let overflow = if self.od.read(0x1019, 0, &mut buf1).is_ok() {
            buf1[0]
        } else {
            0
        };
        assert!(
            crate::sync::is_valid_sync_counter_overflow(overflow),
            "OD 0x1019 contains an invalid SYNC counter overflow value ({overflow})"
        );
        self.sync_producer
            .configure(if produce { cob_id } else { 0 }, period_us, overflow);
    }

    /// Re-initialise everything SYNC-related that a CANopen reset touches:
    /// the configuration (0x1005/0x1006/0x1007/0x1019) and the runtime state
    /// built on top of it.
    ///
    /// The runtime reset is separate from `sync_config_from_od` on purpose.
    /// Reconfiguration deliberately preserves the cycle phase and counters
    /// when the OD values did not change, so an unrelated SDO write cannot
    /// shift the SYNC phase — but a reset re-initialises communication
    /// objects, so the phase, the counters and any PDO data still waiting for
    /// a pulse must not survive it.
    fn reset_sync_state(&mut self) {
        self.sync_config_from_od();
        self.sync.reset();
        self.sync_producer.reset();
        self.sync_window.reset();
        self.pending_sync_tpdo.clear();
        self.sync_tpdo_last_attempt_us = None;
        self.rpdo.discard_pending();
        self.rpdo.reset_deadline_monitoring();
        self.dirty_set.clear();
        self.tpdo.reset_communication_state();
    }

    /// Discard PDO work at the transition, before another queued NMT/SYNC
    /// frame can restart the node in the same processing call.
    fn leave_operational(&mut self) {
        self.rpdo.reset_deadline_monitoring();
        self.rpdo.discard_pending();
        self.tpdo.discard_sync_events();
        self.tpdo.discard_async_events();
        self.dirty_set.clear();
        self.sync_window.reset();
        self.pending_sync_tpdo.clear();
        self.sync_tpdo_last_attempt_us = None;
    }

    /// COB-ID this node consumes SYNC on (OD 0x1005). A missing object uses
    /// 0x080; construction rejects invalid or unsupported declared values.
    /// The `Option` mirrors [`SyncConsumer::cob_id`](crate::sync::SyncConsumer::cob_id).
    pub fn sync_cob_id(&self) -> Option<u16> {
        self.sync.cob_id()
    }

    /// SYNC production period in µs (OD 0x1006), or 0 when this node is not
    /// the SYNC producer.
    pub fn sync_period_us(&self) -> u64 {
        self.sync_producer.interval_us()
    }

    /// OD 0x1007 synchronous window length in µs. Zero means no window
    /// limit. A missing object has the same zero default.
    pub fn sync_window_us(&self) -> u64 {
        self.sync_window.length_us()
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

    /// Report an emergency error and set the corresponding register bits.
    /// Queues an EMCY for `process()` to send in Pre-Operational or Operational
    /// once the transport accepts it; Stopped defers transmission.
    ///
    /// `register_bits` should be a combination of `emcy::error_register::*` constants.
    pub fn set_error(&mut self, error_code: u16, register_bits: u8, vendor_data: &[u8]) {
        self.emcy.set_error(error_code, register_bits, vendor_data);
    }

    /// Clear error register bits. If all errors are cleared, sends an
    /// "error reset" EMCY frame (code 0x0000) under the same transmission
    /// conditions as [`set_error`](Self::set_error).
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
        self.reset_event = Some(reset_type);
        self.booted = false;
        self.abort_all_sdo_transfers();
        self.sync_pdo_from_od();
        self.sync_heartbeat_from_od(false);
        self.reset_sync_state();
        match reset_type {
            ResetType::Application => {
                self.nmt = NmtHandler::new();
            }
            ResetType::Communication => {
                self.nmt = NmtHandler::new();
            }
        }
    }

    /// Take the most recent CANopen reset observed by this node.
    ///
    /// Communication reinitialization is owned by the stack. Applications
    /// use this event for state outside the OD that distinguishes Reset
    /// Application from Reset Communication. Multiple unobserved resets
    /// coalesce to the latest type.
    pub fn take_reset_event(&mut self) -> Option<ResetType> {
        self.reset_event.take()
    }

    /// Mark an OD entry as changed by the application.
    ///
    /// On the next `process()` call, an asynchronous event-driven TPDO (type
    /// 254/255) mapping this entry is sent when its inhibit time permits. A
    /// synchronous acyclic TPDO (type 0) latches the event and sends at the
    /// next SYNC. Synchronous cyclic TPDOs (types 1..=240) ignore it.
    pub fn notify_changed(&mut self, index: u16, subindex: u8) {
        // The per-TPDO latch is authoritative. The address set remains as a
        // compatibility/input surface for the PDO engine, but exhausting its
        // bounded capacity must not lose a transmission trigger.
        self.tpdo.notify_changed(index, subindex);
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

        // Latch application events for synchronous acyclic TPDOs before any
        // SYNC in this cycle is consumed. `dirty_set` is cleared at the end of
        // every cycle, but the SYNC that should carry an event is usually in a
        // later one — without the latch the trigger would only ever survive
        // when the change and the pulse landed in the same call.
        self.tpdo.latch_sync_events(&self.dirty_set);

        // Expire stale SDO work before accepting requests. Retry only after RX,
        // so a queued NMT reset/client abort can cancel it before handoff.
        self.sdo_tx
            .begin_poll(&mut self.sdo_server, CobId::sdo_tx(self.node_id).raw(), now);
        for i in 0..SDO {
            self.extra_sdo_pending_tx[i].begin_poll(
                &mut self.extra_sdo[i],
                self.extra_sdo_tx[i],
                now,
            );
        }

        // Drain incoming frames
        loop {
            match transport.receive() {
                Ok(frame) => self.dispatch_frame(&frame, transport, now),
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => break,
            }
        }

        // SDO responses, timeout aborts and block segments share the same
        // bounded pending slot on each independently serviced channel.
        self.sdo_tx.service(
            &mut self.sdo_server,
            CobId::sdo_tx(self.node_id).raw(),
            transport,
            now,
        );
        for i in 0..SDO {
            self.extra_sdo_pending_tx[i].service(
                &mut self.extra_sdo[i],
                self.extra_sdo_tx[i],
                transport,
                now,
            );
        }

        // SYNC production (OD 0x1005 bit 30 + 0x1006). A producing node also
        // applies the pulse to itself: the transports here do not loop TX back
        // to RX, so waiting to receive our own SYNC would mean never seeing
        // one. Stopped takes no part in bus traffic beyond NMT and error
        // control, so no SYNC is generated there.
        // The local pulse is applied only once the transport has accepted the
        // frame. Dropping it silently would be worse than a missing frame:
        // the bus would never see
        // the pulse while this node committed its synchronous RPDOs and
        // advanced its TPDO cycle, leaving the two disagreeing about which
        // pulse is which. On refusal the producer keeps the pulse and offers
        // it again next cycle.
        if self.booted && self.nmt.state() != NmtState::Stopped {
            if let Some(sync_frame) = self.sync_producer.poll(now) {
                if transport.transmit(&sync_frame).is_ok() {
                    self.sync_producer.confirm_sent(now);
                    self.handle_sync(transport, now);
                }
            }
        }

        // A synchronous TPDO refused by a full transport remains eligible
        // only inside OD 0x1007's window. The pulse path already made one
        // attempt at `now`, so this is a no-op in that same call and becomes a
        // retry on a later processing tick.
        self.retry_pending_sync_tpdo(transport, now);

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
            // PDO data that never reached a SYNC must not act after the node
            // has stopped processing PDOs: buffered RPDO frames would land in
            // the OD, and a latched type-0 trigger would fire a TPDO, on the
            // first pulse after returning to Operational.
            self.leave_operational();
        }

        // A reset received above leaves boot pending until the next call.
        // Do not emit a periodic zero-state frame masquerading as boot-up.
        if self.booted {
            if let Some(hb) = self.heartbeat.pending(now, self.nmt.state()) {
                if transport.transmit(&hb).is_ok() {
                    self.heartbeat.confirm_sent(now);
                }
            }
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

        // EMCY is available only in Pre-Operational and Operational. Keep
        // the bounded error history while stopped, but do not transmit it.
        if self.booted
            && matches!(
                self.nmt.state(),
                NmtState::PreOperational | NmtState::Operational
            )
        {
            while let Some(emcy_frame) = self.emcy.peek_pending() {
                if transport.transmit(emcy_frame).is_err() {
                    break;
                }
                self.emcy.take_pending();
            }
        }

        // TPDO event timers + dirty-triggered sends (only in Operational)
        if self.nmt.state() == NmtState::Operational {
            self.tpdo
                .poll_with(&self.od, now, &self.dirty_set, |frame| {
                    transport.transmit(frame).is_ok()
                });
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
                    let count = buf4[0].min(PDO_MAX_MAPPINGS as u8);
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
                    let count = buf4[0].min(PDO_MAX_MAPPINGS as u8);
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
        self.sdo_tx = PendingSdoTx::default();
        for i in 0..SDO {
            self.extra_sdo[i].abort_transfer();
            self.extra_sdo_pending_tx[i] = PendingSdoTx::default();
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
        // SDO is available only in Pre-Operational and Operational. In
        // particular, requests queued behind a reset must wait for boot.
        if !matches!(
            self.nmt.state(),
            NmtState::PreOperational | NmtState::Operational
        )
            || frame.raw_dlc() != 8
        {
            return;
        }
        let req: [u8; 8] = frame.data().try_into().unwrap();
        let pending = match sel {
            SdoServerSel::Default => &mut self.sdo_tx,
            SdoServerSel::Extra(i) => &mut self.extra_sdo_pending_tx[i],
        };
        if req[0] == 0x80 {
            // Client aborts must still be serviced while TX is blocked.
            pending.frame = None;
        } else if pending.frame.is_some() {
            return;
        }
        let mut resp = [0u8; 8];
        let state = self.nmt.state();
        let was_full = self.event_queue.is_full();
        let heartbeat_consumer_count = self
            .od
            .sub_count(0x1016)
            .unwrap_or(0)
            .min(MAX_HEARTBEAT_MONITORS as u8);

        // The COB-ID plan every server validates config downloads against, so a
        // client cannot park a PDO or a heartbeat producer on an SDO channel,
        // nor move SYNC onto any other service
        // (`Node::new` enforces the same invariants for the static config).
        let (reserved, reserved_complete) = self.reserved_cob_ids();
        let ctx = SdoContext {
            nmt_state: state,
            now_us: now,
            heartbeat_consumer_count,
            cob_ids: SdoCobIds {
                default_rx: CobId::sdo_rx(self.node_id).raw(),
                default_tx: CobId::sdo_tx(self.node_id).raw(),
                extra_rx: &self.extra_sdo_rx,
                extra_tx: &self.extra_sdo_tx,
            },
            reserved_cob_ids: ReservedCobIds {
                ids: &reserved,
                truncated: !reserved_complete,
            },
            sync_cob_id: self.sync.cob_id().unwrap_or(0),
        };

        // Disjoint field borrows: the selected server plus `self.od` /
        // `self.event_queue`. Kept in per-arm expressions so the borrow checker
        // sees the fields as separate.
        let ok = match sel {
            SdoServerSel::Default => self
                .sdo_server
                .process_with_context(&req, &mut self.od, &mut resp, &mut self.event_queue, ctx)
                .is_ok(),
            SdoServerSel::Extra(i) => self.extra_sdo[i]
                .process_with_context(&req, &mut self.od, &mut resp, &mut self.event_queue, ctx)
                .is_ok(),
        };

        if ok {
            let resp_cob = match sel {
                SdoServerSel::Default => CobId::sdo_tx(self.node_id).raw(),
                SdoServerSel::Extra(i) => self.extra_sdo_tx[i],
            };
            let pending = match sel {
                SdoServerSel::Default => &mut self.sdo_tx,
                SdoServerSel::Extra(i) => &mut self.extra_sdo_pending_tx[i],
            };
            pending.queue(resp_cob, resp, now);
            pending.try_send(transport);
        }
        // Resync runtime services whose config lives in the OD. Signalled
        // directly by the SDO server rather than via the event queue: an
        // overflowing queue must not be able to swallow a config change.
        let committed = match sel {
            SdoServerSel::Default => self.sdo_server.take_committed_write(),
            SdoServerSel::Extra(i) => self.extra_sdo[i].take_committed_write(),
        };
        // Every committed download emits one OD event. An upload or rejected
        // download emits none, so queue fullness alone is not evidence that an
        // event was dropped.
        if was_full && committed.is_some() {
            self.event_overflow_count = self.event_overflow_count.saturating_add(1);
        }
        if let Some((index, _)) = committed {
            if (0x1400..=0x1BFF).contains(&index) {
                self.sync_pdo_from_od();
            }
            if index == 0x1016 || index == 0x1017 {
                self.sync_heartbeat_from_od(true);
            }
            if index == 0x1005 || index == 0x1006 || index == 0x1007 || index == 0x1019 {
                self.sync_config_from_od();
            }
        }
    }

    /// Retry synchronous TPDOs which the transport refused on the pulse.
    ///
    /// The generic `embedded_can::nb::Can` boundary can tell us a frame was
    /// refused, but has no operation for recalling a frame after the driver
    /// accepted it. We therefore enforce 0x1007 on every stack-owned pending
    /// frame; transports should keep accepted synchronous traffic prompt (as
    /// CAN priority normally does) because the stack cannot cancel their
    /// private hardware queue after the window closes.
    fn retry_pending_sync_tpdo(
        &mut self,
        transport: &mut impl embedded_can::nb::Can<Frame = CanFrame>,
        now: u64,
    ) {
        if self.pending_sync_tpdo.is_empty() {
            return;
        }
        if self.nmt.state() != NmtState::Operational || !self.sync_window.allows_transmit(now) {
            self.pending_sync_tpdo.clear();
            self.sync_tpdo_last_attempt_us = None;
            return;
        }
        if self.sync_tpdo_last_attempt_us == Some(now) {
            return;
        }
        self.sync_tpdo_last_attempt_us = Some(now);

        let mut refused = Vec::<CanFrame, TPDO>::new();
        for frame in &self.pending_sync_tpdo {
            if transport.transmit(frame).is_err() {
                let _ = refused.push(*frame);
            }
        }
        self.pending_sync_tpdo = refused;
        if self.pending_sync_tpdo.is_empty() {
            self.sync_tpdo_last_attempt_us = None;
        }
    }

    /// One SYNC pulse: commit buffered synchronous RPDOs, then sample and send
    /// synchronous TPDOs.
    ///
    /// The order is the point of SYNC. Setpoints received since the last pulse
    /// land in the OD first, as one coherent update, so a TPDO sampled in the
    /// same pulse reports the post-SYNC state rather than a mix of both sides.
    ///
    /// Gated on Operational: PDOs do not flow in any other NMT state, and
    /// `process()` discards buffered RPDO data on the way out of it.
    fn handle_sync(
        &mut self,
        transport: &mut impl embedded_can::nb::Can<Frame = CanFrame>,
        now: u64,
    ) {
        if self.nmt.state() != NmtState::Operational {
            return;
        }

        // A new pulse supersedes any TPDO the previous cycle never placed in
        // the transport. Open the new 0x1007 window before making this pulse's
        // transmit attempts.
        self.pending_sync_tpdo.clear();
        self.sync_window.open(now);
        self.sync_tpdo_last_attempt_us = Some(now);

        let dropped = self.rpdo.on_sync(&mut self.od, &mut self.event_queue);
        self.event_overflow_count = self.event_overflow_count.saturating_add(dropped);

        let mut out = Vec::<CanFrame, TPDO>::new();
        self.tpdo.on_sync(&self.od, &mut out);
        for f in &out {
            if transport.transmit(f).is_err() {
                let _ = self.pending_sync_tpdo.push(*f);
            }
        }
        if self.pending_sync_tpdo.is_empty() {
            self.sync_tpdo_last_attempt_us = None;
        }
    }

    fn process_rpdo_frame(&mut self, frame: &CanFrame, now: u64) {
        if self.nmt.state() == NmtState::Operational {
            let (_, dropped) =
                self.rpdo
                    .process_with_drop_count(frame, &mut self.od, &mut self.event_queue, now);
            self.event_overflow_count = self.event_overflow_count.saturating_add(dropped);
        }
    }

    fn dispatch_frame(
        &mut self,
        frame: &CanFrame,
        transport: &mut impl embedded_can::nb::Can<Frame = CanFrame>,
        now: u64,
    ) {
        // Additional SDO servers (0x1201+) match by their configured rx COB-ID,
        // which is arbitrary and need not fall in the default SDO request range.
        // Check them first; the default server is handled in the parse below.
        for i in 0..SDO {
            if self.extra_sdo_rx[i] == frame.raw_id() {
                self.handle_sdo_request(SdoServerSel::Extra(i), frame, transport, now);
                return;
            }
        }

        // SYNC, on the OD 0x1005 COB-ID — configurable, so it cannot be left
        // to the pre-defined-connection-set parse below. The COB-ID is
        // node-owned: a frame on it is never offered to another service, even
        // when it is malformed (DLC > 1) and `process` rejects it.
        if self.sync.cob_id() == Some(frame.raw_id()) {
            if self.sync.process(frame).is_some() {
                self.handle_sync(transport, now);
            }
            return;
        }

        // The remaining protocols also route by their exact configured/fixed
        // COB-IDs. Do not use `CobId::parse()` here: its PDO classification is
        // only a pre-defined-connection-set heuristic and custom PDOs may live
        // anywhere in the 11-bit ID space.
        if frame.raw_id() == CobId::nmt().raw() {
            if frame.raw_dlc() == 2 {
                let data = frame.data();
                if let Some(cmd) = NmtCommand::from_byte(data[0]) {
                    let transition = self.nmt.process_command(cmd, data[1], self.node_id);
                    match transition {
                        NmtTransition::ResetApplication => {
                            self.reset_event = Some(ResetType::Application);
                            self.booted = false;
                            self.abort_all_sdo_transfers();
                            self.sync_pdo_from_od();
                            self.sync_heartbeat_from_od(false);
                            self.reset_sync_state();
                        }
                        NmtTransition::ResetCommunication => {
                            self.reset_event = Some(ResetType::Communication);
                            self.booted = false;
                            self.abort_all_sdo_transfers();
                            self.sync_pdo_from_od();
                            self.sync_heartbeat_from_od(false);
                            self.reset_sync_state();
                        }
                        NmtTransition::StateChanged
                            if self.nmt.state() != NmtState::Operational =>
                        {
                            self.leave_operational();
                            if self.nmt.state() == NmtState::Stopped {
                                // Cancel before draining the next frame: Stop
                                // and Start can arrive in this same RX batch.
                                self.abort_all_sdo_transfers();
                            }
                        }
                        _ => {}
                    }
                }
            }
            return;
        }

        if frame.raw_id() == CobId::sdo_rx(self.node_id).raw() {
            self.handle_sdo_request(SdoServerSel::Default, frame, transport, now);
            return;
        }

        if frame.raw_id() == crate::lss::LSS_REQUEST_COB {
            if let Some(resp) = self.lss.process(frame) {
                let _ = transport.transmit(&resp);
            }
            return;
        }

        let heartbeat_monitor = if frame.raw_dlc() == 1 {
            self.hb_monitors
                .iter_mut()
                .flatten()
                .find(|monitor| CobId::heartbeat(monitor.node_id()).raw() == frame.raw_id())
        } else {
            None
        };
        if let Some(monitor) = heartbeat_monitor {
            let mut events = HeartbeatEventBuf::new();
            // The exact COB-ID lookup makes the supplied node tautologically
            // the monitor's own remote node.
            let remote = monitor.node_id();
            monitor.process(remote, frame.data()[0], now, &mut events);
            for evt in events {
                if self.hb_event_queue.is_full() {
                    let _ = self.hb_event_queue.pop_front();
                    self.event_overflow_count = self.event_overflow_count.saturating_add(1);
                }
                let _ = self.hb_event_queue.push_back(evt);
            }
            return;
        }

        // The RPDO engine performs the authoritative exact-COB-ID match,
        // including custom IDs outside every predefined PDO range.
        self.process_rpdo_frame(frame, now);
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
        // Coalesce changes directly per TPDO. This is the behavior we need on
        // drop, and unlike collecting addresses in a fixed scratch array it
        // covers every valid mapping count and every declared TPDO.
        let mut changed_tpdos = [false; TPDO];

        for (slot, changed) in changed_tpdos.iter_mut().enumerate() {
            if let Some(config) = self.node.tpdo.config_slot(slot) {
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
                        *changed = true;
                        break;
                    }
                }
            }
        }

        let mut any_changed = false;
        for (slot, changed) in changed_tpdos.into_iter().enumerate() {
            if changed {
                self.node.tpdo.latch_slot_event(slot);
                any_changed = true;
            }
        }

        // Wake async waiters so process() runs sooner
        #[cfg(feature = "embassy")]
        if any_changed {
            if let Some(signal) = self.node.event_signal {
                signal.signal(());
            }
        }

        let _ = any_changed; // suppress unused warning when embassy feature is off
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

    fn make_event_node_with_dirty<const DIRTY_SET: usize>() -> (
        Node<EventTestOd, 1, 1, 0, 16, DIRTY_SET>,
        MailboxTransport<32, 32>,
    ) {
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
        let mut node: Node<EventTestOd, 1, 1, 0, 16, DIRTY_SET> = Node::new(config, od);
        let mut transport = MailboxTransport::<32, 32>::new();

        // Boot + auto-start
        node.process(&mut transport, &TestClock(0));
        // Drain boot heartbeat
        while transport.next_to_transmit().is_some() {}
        assert_eq!(node.state(), NmtState::Operational);

        (node, transport)
    }

    fn make_event_node() -> (Node<EventTestOd, 1, 1>, MailboxTransport<32, 32>) {
        make_event_node_with_dirty::<8>()
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
    fn custom_rpdo_cob_id_outside_predefined_ranges_is_dispatched() {
        let (mut node, mut transport) = make_event_node();
        node.rpdo_engine_mut()
            .config_mut(PdoNumber::of::<1>())
            .unwrap()
            .cob_id = 0x123;

        let rpdo_frame = CanFrame::new(0x123, &[0xAA, 0xBB]).unwrap();
        transport.store_received(rpdo_frame).unwrap();
        node.process(&mut transport, &TestClock(1000));

        assert_eq!(node.od().output1, 0xAA);
        assert_eq!(node.od().output2, 0xBB);
    }

    #[test]
    fn custom_rpdo_in_an_unmonitored_heartbeat_range_is_dispatched() {
        let (mut node, mut transport) = make_event_node();
        node.rpdo_engine_mut()
            .config_mut(PdoNumber::of::<1>())
            .unwrap()
            .cob_id = 0x712;

        let rpdo_frame = CanFrame::new(0x712, &[0xAA, 0xBB]).unwrap();
        transport.store_received(rpdo_frame).unwrap();
        node.process(&mut transport, &TestClock(1000));

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

        // A read-only SDO upload does not enqueue an OD event. It must not be
        // counted as another drop merely because the queue remains full.
        transport
            .store_received(CanFrame::new(0x601, &[0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(2_000));
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

        // At 100ms, the original event should still be pending; no second
        // notification is needed merely because inhibit time elapsed.
        node.process(&mut transport, &TestClock(100_000));
        let frame = transport.next_to_transmit().unwrap();
        assert_eq!(frame.data()[0], 0x11);

        // Immediately after, within inhibit time, should not send
        node.od_mut().input1 = 0x22;
        node.notify_changed(0x6000, 1);
        node.process(&mut transport, &TestClock(150_000)); // 50ms after last send
        assert!(transport.next_to_transmit().is_none());

        // The coalesced event samples the newest value when inhibit expires.
        node.process(&mut transport, &TestClock(200_000));
        let frame = transport.next_to_transmit().unwrap();
        assert_eq!(frame.data()[0], 0x22);
    }

    #[test]
    fn reset_discards_an_inhibited_event_driven_tpdo() {
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
            // auto_start re-enters Operational in the same process() call that
            // performs the reset, so the "left Operational" discard path never
            // runs. The reset itself has to clear the latch.
            auto_start: true,
            tpdo: [TpdoConfig {
                od_number: 0,
                cob_id: 0x181,
                transmission_type: 255,
                inhibit_time_100us: 500, // 50ms inhibit
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
            input1: 0,
        };
        let mut node: Node<EventTestOd, 1, 1> = Node::new(config, od);
        let mut transport = MailboxTransport::<32, 32>::new();

        node.process(&mut transport, &TestClock(0));
        while transport.next_to_transmit().is_some() {}

        node.od_mut().input1 = 0x11;
        node.process(&mut transport, &TestClock(100_000));
        assert_eq!(transport.next_to_transmit().unwrap().data(), &[0x11]);

        // Second change inside the inhibit window: latched, not yet sent.
        node.od_mut().input1 = 0x22;
        node.process(&mut transport, &TestClock(110_000));
        assert!(transport.next_to_transmit().is_none());

        node.request_reset(ResetType::Application);
        node.process(&mut transport, &TestClock(120_000));
        while transport.next_to_transmit().is_some() {}

        // Well past the inhibit time, with no post-reset application change.
        node.process(&mut transport, &TestClock(400_000));
        let leftover = transport.next_to_transmit();
        assert!(
            leftover.is_none(),
            "pre-reset TPDO event transmitted after reset: {leftover:?}"
        );
    }

    #[test]
    fn od_guard_tpdo_trigger_does_not_depend_on_dirty_address_capacity() {
        let (mut node, mut transport) = make_event_node_with_dirty::<0>();

        {
            let mut od = node.od_mut();
            od.input1 = 0x66;
        }
        node.process(&mut transport, &TestClock(1_000));

        let frame = transport.next_to_transmit().expect("latched TPDO event");
        assert_eq!(frame.raw_id(), 0x181);
        assert_eq!(frame.data(), &[0x66]);
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
        assert_eq!(node.take_reset_event(), Some(ResetType::Application));
        assert_eq!(node.take_reset_event(), None);

        // Next process() should re-boot (send boot heartbeat, transition to PreOp)
        node.process(&mut transport, &TestClock(100_000));
        let frame = transport.next_to_transmit().unwrap();
        assert_eq!(frame.raw_id(), 0x701);
        assert_eq!(frame.data()[0], 0x00); // boot heartbeat
        assert_eq!(node.state(), NmtState::PreOperational);
    }

    #[test]
    fn nmt_reset_event_distinguishes_application_and_communication() {
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
        node.process(&mut transport, &TestClock(0));
        while transport.next_to_transmit().is_some() {}

        transport
            .store_received(NmtCommand::ResetCommunication.to_frame(1))
            .unwrap();
        node.process(&mut transport, &TestClock(1));
        assert_eq!(node.take_reset_event(), Some(ResetType::Communication));

        transport
            .store_received(NmtCommand::ResetNode.to_frame(1))
            .unwrap();
        node.process(&mut transport, &TestClock(2));
        assert_eq!(node.take_reset_event(), Some(ResetType::Application));
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

    #[derive(Clone)]
    struct WideHbOd {
        consumer: [u32; 9],
    }

    impl ObjectDictionary for WideHbOd {
        fn lookup(&self, _index: u16, _subindex: u8) -> Option<&'static OdEntryMeta> {
            None
        }

        fn read(&self, index: u16, subindex: u8, buf: &mut [u8]) -> Result<usize, OdError> {
            match (index, subindex) {
                (0x1016, 0) => {
                    buf[0] = 9;
                    Ok(1)
                }
                (0x1016, sub @ 1..=9) => {
                    buf[..4].copy_from_slice(&self.consumer[(sub as usize) - 1].to_le_bytes());
                    Ok(4)
                }
                _ => Err(OdError::NotFound),
            }
        }

        fn write(&mut self, index: u16, subindex: u8, data: &[u8]) -> Result<(), OdError> {
            match (index, subindex) {
                (0x1016, sub @ 1..=9) if data.len() == 4 => {
                    self.consumer[(sub as usize) - 1] =
                        u32::from_le_bytes(data.try_into().unwrap());
                    Ok(())
                }
                (0x1016, _) => Err(OdError::DataTypeMismatch),
                _ => Err(OdError::NotFound),
            }
        }

        fn sub_count(&self, index: u16) -> Option<u8> {
            (index == 0x1016).then_some(9)
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

    /// The runtime half of the COB-ID uniqueness invariant on the heartbeat
    /// side: a consumer entry whose producer heartbeats on a COB-ID an SDO
    /// server occupies is rejected. Those frames are dispatched to the SDO
    /// server, so the monitor could never see them.
    #[test]
    fn consumer_entry_on_sdo_channel_cob_id_aborts() {
        // Diagnostics channel parked in the heartbeat range: 0x710/0x711 are
        // the heartbeat COB-IDs of nodes 16 and 17.
        let config = NodeConfig::<1, 1, 1> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 0,
            auto_start: false,
            tpdo: [TpdoConfig::default()],
            rpdo: [RpdoConfig::default()],
            sdo_servers: [SdoServerConfig {
                number: 2,
                cob_rx: 0x710,
                cob_tx: 0x711,
            }],
            identity: LssIdentity::default(),
        };
        let mut node: Node<HbOd, 1, 1, 1> = Node::new(
            config,
            HbOd {
                producer_time: 0,
                consumer: [0; 3],
            },
        );
        let mut transport = MailboxTransport::<32, 32>::new();
        node.process(&mut transport, &TestClock(0));
        while transport.next_to_transmit().is_some() {}

        let mut write_1016 = |node: &mut Node<HbOd, 1, 1, 1>, value: u32, at: u64| {
            let mut req = [0x23, 0x16, 0x10, 0x01, 0, 0, 0, 0];
            req[4..8].copy_from_slice(&value.to_le_bytes());
            transport
                .store_received(CanFrame::new(0x601, &req).unwrap())
                .unwrap();
            node.process(&mut transport, &TestClock(at));
            let resp = transport.next_to_transmit().unwrap();
            while transport.next_to_transmit().is_some() {}
            resp
        };

        // Node 16 heartbeats on 0x710 = the diagnostics rx COB-ID.
        let resp = write_1016(&mut node, (16 << 16) | 100, 10_000);
        assert_eq!(resp.data()[0], 0x80, "abort");
        assert_eq!(
            u32::from_le_bytes(resp.data()[4..8].try_into().unwrap()),
            0x0604_0043
        );
        // Node 17 heartbeats on 0x711 = the diagnostics tx COB-ID.
        let resp = write_1016(&mut node, (17 << 16) | 100, 20_000);
        assert_eq!(resp.data()[0], 0x80, "abort");
        // Node 18 (0x712) is free — the guard is not a blanket ban.
        let resp = write_1016(&mut node, (18 << 16) | 100, 30_000);
        assert_eq!(resp.data()[0], 0x60, "download acknowledged");
        assert_eq!(
            node.heartbeat_status(NodeId::new(18).unwrap()),
            HeartbeatMonitorState::Waiting
        );
    }

    #[test]
    fn unmonitored_consumer_entry_may_share_sdo_channel_cob_id() {
        let config = NodeConfig::<0, 0, 1> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 0,
            auto_start: false,
            tpdo: [],
            rpdo: [],
            sdo_servers: [SdoServerConfig {
                number: 2,
                cob_rx: 0x710,
                cob_tx: 0x711,
            }],
            identity: LssIdentity::default(),
        };
        let mut node: Node<WideHbOd, 0, 0, 1> = Node::new(config, WideHbOd { consumer: [0; 9] });
        let mut transport = MailboxTransport::<32, 32>::new();
        node.process(&mut transport, &TestClock(0));
        while transport.next_to_transmit().is_some() {}

        // Subindex 9 is stored in the OD but lies beyond the node's eight
        // HeartbeatMonitor slots. It therefore cannot create a dispatch
        // collision even though node 16 heartbeats on the diagnostics rx ID.
        let value = (16u32 << 16) | 100;
        let mut req = [0x23, 0x16, 0x10, 0x09, 0, 0, 0, 0];
        req[4..8].copy_from_slice(&value.to_le_bytes());
        transport
            .store_received(CanFrame::new(0x601, &req).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(10_000));

        let resp = transport.next_to_transmit().unwrap();
        assert_eq!(resp.data()[0], 0x60, "download acknowledged");
        assert_eq!(node.od().consumer[8], value);
        assert_eq!(
            node.heartbeat_status(NodeId::new(16).unwrap()),
            HeartbeatMonitorState::Disabled
        );
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

    // ---- SYNC (0x1005 / 0x1006 / 0x1007 / 0x1019) ----

    /// Sets a slot's transmission type in the engine, as an SDO write to the
    /// comm parameter would.
    fn set_rpdo_tx_type(node: &mut Node<EventTestOd, 1, 1>, tt: u8) {
        node.rpdo_engine_mut()
            .config_mut(PdoNumber::of::<1>())
            .unwrap()
            .transmission_type = tt;
    }

    fn set_tpdo_tx_type(node: &mut Node<EventTestOd, 1, 1>, tt: u8) {
        node.tpdo_engine_mut()
            .config_mut(PdoNumber::of::<1>())
            .unwrap()
            .transmission_type = tt;
    }

    fn sync_frame() -> CanFrame {
        CanFrame::new(0x080, &[]).unwrap()
    }

    /// Drain the transmit mailbox, returning the frames on `cob_id`.
    fn drain_frames_with_id<const TX: usize, const RX: usize>(
        transport: &MailboxTransport<TX, RX>,
        cob_id: u16,
    ) -> heapless::Vec<CanFrame, 8> {
        let mut found = heapless::Vec::new();
        while let Some(f) = transport.next_to_transmit() {
            if f.raw_id() == cob_id {
                let _ = found.push(f);
            }
        }
        found
    }

    #[test]
    fn synchronous_rpdo_reaches_the_od_only_on_sync() {
        let (mut node, mut transport) = make_event_node();
        set_rpdo_tx_type(&mut node, crate::pdo::engine::SYNC_CYCLIC_1);

        transport
            .store_received(CanFrame::new(0x201, &[0xAA, 0xBB]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(1000));
        assert_eq!(node.od().output1, 0, "committed before the SYNC");
        assert!(node.next_event().is_none(), "event fired before the commit");

        transport.store_received(sync_frame()).unwrap();
        node.process(&mut transport, &TestClock(2000));
        assert_eq!(node.od().output1, 0xAA);
        assert_eq!(node.od().output2, 0xBB);
        assert_eq!(node.next_event().unwrap().source, OdEventSource::Rpdo);
    }

    #[test]
    fn sync_tpdo_samples_after_the_rpdo_commit() {
        // A TPDO mapping the same entry an RPDO writes must report the
        // post-SYNC value: buffered setpoints commit before TPDOs sample.
        let (mut node, mut transport) = make_event_node();
        set_rpdo_tx_type(&mut node, crate::pdo::engine::SYNC_CYCLIC_1);
        set_tpdo_tx_type(&mut node, crate::pdo::engine::SYNC_CYCLIC_1);
        {
            let tpdo = node
                .tpdo_engine_mut()
                .config_mut(PdoNumber::of::<1>())
                .unwrap();
            tpdo.mappings.clear();
            tpdo.mappings
                .push(PdoMapping {
                    index: 0x6200,
                    subindex: 1,
                    bit_length: 8,
                })
                .unwrap();
        }

        transport
            .store_received(CanFrame::new(0x201, &[0x77, 0x00]).unwrap())
            .unwrap();
        transport.store_received(sync_frame()).unwrap();
        node.process(&mut transport, &TestClock(1000));

        let tpdos = drain_frames_with_id(&transport, 0x181);
        assert_eq!(tpdos.len(), 1);
        assert_eq!(tpdos[0].data(), &[0x77]);
    }

    #[test]
    fn leaving_operational_discards_buffered_rpdo() {
        let (mut node, mut transport) = make_event_node();
        set_rpdo_tx_type(&mut node, crate::pdo::engine::SYNC_CYCLIC_1);

        transport
            .store_received(CanFrame::new(0x201, &[0xAA, 0xBB]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(1000));

        // NMT Enter Pre-Operational, then back to Operational.
        transport
            .store_received(CanFrame::new(0x000, &[0x80, 0x01]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(2000));
        assert_eq!(node.state(), NmtState::PreOperational);
        transport
            .store_received(CanFrame::new(0x000, &[0x01, 0x01]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(3000));
        assert_eq!(node.state(), NmtState::Operational);

        // The pre-Pre-Op frame must not surface now.
        transport.store_received(sync_frame()).unwrap();
        node.process(&mut transport, &TestClock(4000));
        assert_eq!(node.od().output1, 0);
    }

    // An OD that declares the SYNC objects, so
    // 0x1005/0x1006/0x1007/0x1019 are live.
    #[derive(Clone)]
    struct SyncOd {
        device_type: u32,
        sync_cob_id: u32,
        cycle_period_us: u32,
        sync_window_us: u32,
        counter_overflow: u8,
    }

    static SYNC_OD_META: &[OdEntryMeta] = &[
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
            index: 0x1005,
            subindex: 0,
            data_type: DataType::U32,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "cob_id_sync",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x1006,
            subindex: 0,
            data_type: DataType::U32,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "communication_cycle_period",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x1007,
            subindex: 0,
            data_type: DataType::U32,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "synchronous_window_length",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x1019,
            subindex: 0,
            data_type: DataType::U8,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "sync_counter_overflow",
            max_size: None,
        },
    ];

    impl ObjectDictionary for SyncOd {
        fn lookup(&self, index: u16, subindex: u8) -> Option<&'static OdEntryMeta> {
            SYNC_OD_META
                .iter()
                .find(|e| e.index == index && e.subindex == subindex)
        }
        fn read(&self, index: u16, subindex: u8, buf: &mut [u8]) -> Result<usize, OdError> {
            match (index, subindex) {
                (0x1000, 0) => {
                    buf[..4].copy_from_slice(&self.device_type.to_le_bytes());
                    Ok(4)
                }
                (0x1005, 0) => {
                    buf[..4].copy_from_slice(&self.sync_cob_id.to_le_bytes());
                    Ok(4)
                }
                (0x1006, 0) => {
                    buf[..4].copy_from_slice(&self.cycle_period_us.to_le_bytes());
                    Ok(4)
                }
                (0x1007, 0) => {
                    buf[..4].copy_from_slice(&self.sync_window_us.to_le_bytes());
                    Ok(4)
                }
                (0x1019, 0) => {
                    buf[0] = self.counter_overflow;
                    Ok(1)
                }
                _ => Err(OdError::NotFound),
            }
        }
        fn write(&mut self, index: u16, subindex: u8, data: &[u8]) -> Result<(), OdError> {
            match (index, subindex) {
                (0x1005, 0) => {
                    self.sync_cob_id = u32::from_le_bytes(data.try_into().unwrap());
                    Ok(())
                }
                (0x1006, 0) => {
                    self.cycle_period_us = u32::from_le_bytes(data.try_into().unwrap());
                    Ok(())
                }
                (0x1007, 0) => {
                    self.sync_window_us = u32::from_le_bytes(data.try_into().unwrap());
                    Ok(())
                }
                (0x1019, 0) => {
                    self.counter_overflow = data[0];
                    Ok(())
                }
                _ => Err(OdError::ReadOnly),
            }
        }
        fn sub_count(&self, _: u16) -> Option<u8> {
            Some(0)
        }
    }

    fn make_sync_node(od: SyncOd) -> (Node<SyncOd, 0, 0>, MailboxTransport<32, 32>) {
        let config = NodeConfig::<0, 0> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 0,
            auto_start: true,
            tpdo: [],
            rpdo: [],
            sdo_servers: [],
            identity: LssIdentity::default(),
        };
        let mut node: Node<SyncOd, 0, 0> = Node::new(config, od);
        let mut transport = MailboxTransport::<32, 32>::new();
        node.process(&mut transport, &TestClock(0));
        while transport.next_to_transmit().is_some() {}
        (node, transport)
    }

    fn sync_od(sync_cob_id: u32, cycle_period_us: u32, counter_overflow: u8) -> SyncOd {
        SyncOd {
            device_type: 0x191,
            sync_cob_id,
            cycle_period_us,
            sync_window_us: 0,
            counter_overflow,
        }
    }

    fn sync_od_with_window(window_us: u32) -> SyncOd {
        SyncOd {
            sync_window_us: window_us,
            ..sync_od(0x080, 0, 0)
        }
    }

    #[test]
    fn sync_cob_id_defaults_and_follows_1005() {
        let (node, _) = make_sync_node(sync_od(0x080, 0, 0));
        assert_eq!(node.sync_cob_id(), Some(0x080));
        assert_eq!(node.sync_period_us(), 0, "consumer only without bit 30");
        assert_eq!(node.sync_window_us(), 0, "missing/zero 0x1007 is unlimited");

        let (node, _) = make_sync_node(sync_od(0x123, 0, 0));
        assert_eq!(node.sync_cob_id(), Some(0x123));
    }

    #[test]
    #[should_panic(expected = "OD 0x1005 contains an invalid or unsupported")]
    fn node_new_rejects_an_invalid_static_sync_cob_id() {
        // Static OD defaults obey the same rule as SDO downloads: COB-ID 0
        // would claim NMT and is never a disabled-SYNC encoding.
        let _ = make_sync_node(sync_od(0, 0, 0));
    }

    #[test]
    #[should_panic(expected = "OD 0x1019 contains an invalid")]
    fn node_new_rejects_an_invalid_static_sync_counter_overflow() {
        let _ = make_sync_node(sync_od(0x080, 0, 241));
    }

    #[test]
    fn sdo_write_to_1005_relocates_the_consumer() {
        let (mut node, mut transport) = make_sync_node(sync_od(0x080, 0, 0));

        // Expedited 4-byte download of 0x123 to 0x1005:00.
        transport
            .store_received(
                CanFrame::new(0x601, &[0x23, 0x05, 0x10, 0x00, 0x23, 0x01, 0x00, 0x00]).unwrap(),
            )
            .unwrap();
        node.process(&mut transport, &TestClock(1000));

        assert_eq!(node.sync_cob_id(), Some(0x123));
    }

    #[test]
    fn sdo_write_to_1007_updates_the_synchronous_window() {
        let (mut node, mut transport) = make_sync_node(sync_od(0x080, 0, 0));

        // Expedited 4-byte download of 750 µs to 0x1007:00.
        transport
            .store_received(
                CanFrame::new(0x601, &[0x23, 0x07, 0x10, 0x00, 0xEE, 0x02, 0x00, 0x00]).unwrap(),
            )
            .unwrap();
        node.process(&mut transport, &TestClock(1_000));

        let response = transport.next_to_transmit().expect("SDO response");
        assert_eq!(response.data()[0], 0x60);
        assert_eq!(node.sync_window_us(), 750);
    }

    #[test]
    fn sdo_write_to_1005_rejects_unsupported_and_colliding_ids() {
        for value in [
            crate::sync::SYNC_FRAME_29BIT | 0x080, // 29-bit SYNC: not implemented
            0x8000_0000,                           // reserved bit 31
            0x0000_0000,                           // COB-ID 0 is NMT
            0x0000_0601,                           // this node's SDO request ID
        ] {
            let (mut node, mut transport) = make_sync_node(sync_od(0x080, 0, 0));
            let mut req = [0x23u8, 0x05, 0x10, 0x00, 0, 0, 0, 0];
            req[4..].copy_from_slice(&value.to_le_bytes());
            transport
                .store_received(CanFrame::new(0x601, &req).unwrap())
                .unwrap();
            node.process(&mut transport, &TestClock(1000));

            let resp = transport.next_to_transmit().expect("SDO response");
            assert_eq!(resp.data()[0], 0x80, "expected abort for {value:#010X}");
            assert_eq!(
                node.sync_cob_id(),
                Some(0x080),
                "SYNC moved despite the abort for {value:#010X}"
            );
        }
    }

    #[test]
    fn sync_producer_runs_on_the_communication_cycle_period() {
        // 0x1005 bit 30 set + 0x1006 = 10 ms.
        let (mut node, mut transport) =
            make_sync_node(sync_od(crate::sync::SYNC_GEN | 0x080, 10_000, 0));
        assert_eq!(node.sync_period_us(), 10_000);

        // The boot process() call armed the producer; nothing is due yet.
        node.process(&mut transport, &TestClock(9_000));
        assert!(drain_frames_with_id(&transport, 0x080).is_empty());

        node.process(&mut transport, &TestClock(10_000));
        let syncs = drain_frames_with_id(&transport, 0x080);
        assert_eq!(syncs.len(), 1);
        assert_eq!(syncs[0].raw_dlc(), 0, "no counter byte without 0x1019");
    }

    #[test]
    fn sync_producer_emits_counter_when_1019_is_set() {
        let (mut node, mut transport) =
            make_sync_node(sync_od(crate::sync::SYNC_GEN | 0x080, 10_000, 2));

        node.process(&mut transport, &TestClock(10_000));
        node.process(&mut transport, &TestClock(20_000));
        node.process(&mut transport, &TestClock(30_000));

        let syncs = drain_frames_with_id(&transport, 0x080);
        assert_eq!(syncs.len(), 3);
        let counters: heapless::Vec<u8, 8> = syncs.iter().map(|f| f.data()[0]).collect();
        assert_eq!(counters.as_slice(), &[1, 2, 1]);
    }

    #[test]
    fn sync_acyclic_tpdo_fires_on_a_later_pulse() {
        // The dirty set is cleared at the end of every process() call, so a
        // type-0 trigger has to be latched: an application event and the SYNC
        // that carries it are almost never in the same cycle.
        let (mut node, mut transport) = make_event_node();
        set_tpdo_tx_type(&mut node, crate::pdo::engine::SYNC_ACYCLIC);

        // Cycle 1: application event, no SYNC.
        node.notify_changed(0x6000, 1);
        node.process(&mut transport, &TestClock(1000));
        assert!(drain_frames_with_id(&transport, 0x181).is_empty());

        // Cycle 2: nothing dirty any more, still no SYNC.
        node.process(&mut transport, &TestClock(2000));
        assert!(drain_frames_with_id(&transport, 0x181).is_empty());

        // Cycle 3: the pulse arrives and must carry the cycle-1 event.
        transport.store_received(sync_frame()).unwrap();
        node.process(&mut transport, &TestClock(3000));
        assert_eq!(
            drain_frames_with_id(&transport, 0x181).len(),
            1,
            "type-0 trigger lost between pulses"
        );

        // Acyclic: one transmission per event, so the next pulse is silent.
        transport.store_received(sync_frame()).unwrap();
        node.process(&mut transport, &TestClock(4000));
        assert!(drain_frames_with_id(&transport, 0x181).is_empty());
    }

    #[test]
    fn leaving_operational_discards_latched_sync_acyclic_trigger() {
        let (mut node, mut transport) = make_event_node();
        set_tpdo_tx_type(&mut node, crate::pdo::engine::SYNC_ACYCLIC);

        node.notify_changed(0x6000, 1);
        node.process(&mut transport, &TestClock(1000));

        transport
            .store_received(CanFrame::new(0x000, &[0x80, 0x01]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(2000));
        transport
            .store_received(CanFrame::new(0x000, &[0x01, 0x01]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(3000));
        while transport.next_to_transmit().is_some() {}

        transport.store_received(sync_frame()).unwrap();
        node.process(&mut transport, &TestClock(4000));
        assert!(drain_frames_with_id(&transport, 0x181).is_empty());
    }

    #[test]
    fn no_sync_produced_without_the_generator_bit() {
        // 0x1006 alone must not make the node the SYNC master.
        let (mut node, mut transport) = make_sync_node(sync_od(0x080, 10_000, 0));
        assert_eq!(node.sync_period_us(), 0);
        node.process(&mut transport, &TestClock(50_000));
        assert!(drain_frames_with_id(&transport, 0x080).is_empty());
    }

    fn make_window_tpdo_node(window_us: u32) -> Node<SyncOd, 1, 0> {
        let config = NodeConfig::<1, 0> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 0,
            auto_start: true,
            tpdo: [TpdoConfig {
                od_number: 0,
                cob_id: 0x181,
                transmission_type: crate::pdo::engine::SYNC_CYCLIC_1,
                inhibit_time_100us: 0,
                event_timer_ms: 0,
                mappings: heapless::Vec::new(),
                enabled: true,
            }],
            rpdo: [],
            sdo_servers: [],
            identity: LssIdentity::default(),
        };
        Node::new(config, sync_od_with_window(window_us))
    }

    #[test]
    fn refused_sync_tpdo_retries_inside_1007_window() {
        let mut node = make_window_tpdo_node(500);
        let mut transport = MailboxTransport::<1, 32>::new();

        // Boot-up occupies the only TX slot. The pulse makes TPDO1 due, but
        // the transport refuses it and Node retains it for this window.
        node.process(&mut transport, &TestClock(0));
        transport.store_received(sync_frame()).unwrap();
        node.process(&mut transport, &TestClock(1_000));
        let boot = transport.next_to_transmit().expect("boot-up frame");
        assert_eq!(boot.raw_id(), 0x701);
        assert!(transport.next_to_transmit().is_none());

        // The boundary itself remains inside the window.
        node.process(&mut transport, &TestClock(1_500));
        let tpdo = transport.next_to_transmit().expect("retried TPDO");
        assert_eq!(tpdo.raw_id(), 0x181);
    }

    #[test]
    fn refused_sync_tpdo_is_discarded_after_1007_window() {
        let mut node = make_window_tpdo_node(500);
        let mut transport = MailboxTransport::<1, 32>::new();

        node.process(&mut transport, &TestClock(0));
        transport.store_received(sync_frame()).unwrap();
        node.process(&mut transport, &TestClock(1_000));
        let _ = transport.next_to_transmit().expect("boot-up frame");

        node.process(&mut transport, &TestClock(1_501));
        assert!(
            transport.next_to_transmit().is_none(),
            "stale synchronous TPDO escaped after the window"
        );

        // A fresh pulse opens a fresh window and schedules the next TPDO.
        transport.store_received(sync_frame()).unwrap();
        node.process(&mut transport, &TestClock(2_000));
        let tpdo = transport.next_to_transmit().expect("fresh-cycle TPDO");
        assert_eq!(tpdo.raw_id(), 0x181);
    }

    #[test]
    fn nmt_reset_discards_a_refused_sync_tpdo_and_window_state() {
        let mut node = make_window_tpdo_node(500);
        let mut transport = MailboxTransport::<1, 32>::new();

        node.process(&mut transport, &TestClock(0));
        transport.store_received(sync_frame()).unwrap();
        node.process(&mut transport, &TestClock(1_000));

        // Reset Communication while the boot-up frame still keeps the TPDO
        // pending. Reset must remove it rather than let it escape after reboot.
        transport
            .store_received(CanFrame::new(0x000, &[0x82, 0x01]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(1_100));
        let _ = transport
            .next_to_transmit()
            .expect("original boot-up frame");

        node.process(&mut transport, &TestClock(1_200));
        let reboot = transport.next_to_transmit().expect("reset boot-up frame");
        assert_eq!(reboot.raw_id(), 0x701);
        node.process(&mut transport, &TestClock(1_300));
        assert!(
            transport.next_to_transmit().is_none(),
            "pre-reset synchronous TPDO survived the communication reset"
        );
    }

    /// A SYNC-aware node that also declares PDOs, so relocating SYNC has
    /// something to collide with. Mappings are irrelevant here — only the
    /// COB-IDs are.
    fn make_sync_node_with_pdos(od: SyncOd) -> (Node<SyncOd, 1, 1>, MailboxTransport<32, 32>) {
        let config = NodeConfig::<1, 1> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 0,
            auto_start: true,
            tpdo: [TpdoConfig {
                od_number: 0,
                cob_id: 0x181,
                transmission_type: 255,
                inhibit_time_100us: 0,
                event_timer_ms: 0,
                mappings: heapless::Vec::new(),
                enabled: true,
            }],
            rpdo: [RpdoConfig {
                od_number: 0,
                cob_id: 0x201,
                transmission_type: 255,
                deadline_ms: 0,
                mappings: heapless::Vec::new(),
                enabled: true,
            }],
            sdo_servers: [],
            identity: LssIdentity::default(),
        };
        let mut node: Node<SyncOd, 1, 1> = Node::new(config, od);
        let mut transport = MailboxTransport::<32, 32>::new();
        node.process(&mut transport, &TestClock(0));
        while transport.next_to_transmit().is_some() {}
        (node, transport)
    }

    #[test]
    fn sdo_write_to_1005_rejects_ids_other_services_own() {
        // SYNC is dispatched ahead of all of these, so relocating onto one
        // would mute it outright rather than merely conflict.
        for (value, what) in [
            (0x0000_0201u32, "RPDO1"),
            (0x0000_0181, "TPDO1"),
            (0x0000_0081, "EMCY"),
            (0x0000_0701, "own heartbeat"),
            (0x0000_07E5, "LSS request"),
            (0x0000_07E4, "LSS response"),
            (0x0000_0601, "SDO request"),
            (0x0000_0581, "SDO response"),
        ] {
            let (mut node, mut transport) = make_sync_node_with_pdos(sync_od(0x080, 0, 0));
            let mut req = [0x23u8, 0x05, 0x10, 0x00, 0, 0, 0, 0];
            req[4..].copy_from_slice(&value.to_le_bytes());
            transport
                .store_received(CanFrame::new(0x601, &req).unwrap())
                .unwrap();
            node.process(&mut transport, &TestClock(1000));

            let resp = transport.next_to_transmit().expect("SDO response");
            assert_eq!(resp.data()[0], 0x80, "expected abort for {what}");
            assert_eq!(
                u32::from_le_bytes(resp.data()[4..8].try_into().unwrap()),
                0x0604_0043,
                "wrong abort code for {what}"
            );
            assert_eq!(node.sync_cob_id(), Some(0x080), "SYNC moved onto {what}");
        }
    }

    #[test]
    fn sdo_write_to_1005_allows_a_free_cob_id() {
        let (mut node, mut transport) = make_sync_node_with_pdos(sync_od(0x080, 0, 0));
        transport
            .store_received(
                CanFrame::new(0x601, &[0x23, 0x05, 0x10, 0x00, 0x23, 0x01, 0x00, 0x00]).unwrap(),
            )
            .unwrap();
        node.process(&mut transport, &TestClock(1000));

        let resp = transport.next_to_transmit().expect("SDO response");
        assert_eq!(resp.data()[0], 0x60, "download should be accepted");
        assert_eq!(node.sync_cob_id(), Some(0x123));
    }

    #[test]
    #[should_panic(expected = "SYNC COB-ID")]
    fn node_new_rejects_a_sync_cob_id_that_collides_with_a_pdo() {
        // The static half of the same invariant.
        let _ = make_sync_node_with_pdos(sync_od(0x201, 0, 0));
    }

    #[test]
    fn sdo_write_to_1019_enforces_the_counter_range() {
        // CiA 301: 0 disables the counter, 2..=240 is the wrap value. 1 and
        // anything above 240 are not legal.
        for (value, accepted) in [
            (0u8, true),
            (1, false),
            (2, true),
            (240, true),
            (241, false),
        ] {
            let (mut node, mut transport) = make_sync_node(sync_od(0x080, 0, 0));
            transport
                .store_received(
                    CanFrame::new(0x601, &[0x2F, 0x19, 0x10, 0x00, value, 0, 0, 0]).unwrap(),
                )
                .unwrap();
            node.process(&mut transport, &TestClock(1000));

            let resp = transport.next_to_transmit().expect("SDO response");
            if accepted {
                assert_eq!(resp.data()[0], 0x60, "0x1019 = {value} should be accepted");
            } else {
                assert_eq!(resp.data()[0], 0x80, "0x1019 = {value} should abort");
            }
        }
    }

    #[test]
    fn refused_sync_transmission_is_retried_without_advancing_the_counter() {
        // A one-frame TX queue remains occupied by the boot-up frame, so each
        // SYNC transmit attempt fails until the queue is drained. heapless
        // queues deliberately do not support capacity zero.
        let config = NodeConfig::<0, 0> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 0,
            auto_start: true,
            tpdo: [],
            rpdo: [],
            sdo_servers: [],
            identity: LssIdentity::default(),
        };
        let mut node: Node<SyncOd, 0, 0> =
            Node::new(config, sync_od(crate::sync::SYNC_GEN | 0x080, 10_000, 3));
        let mut transport = MailboxTransport::<1, 32>::new();

        node.process(&mut transport, &TestClock(0)); // boot fills TX; producer arms
        node.process(&mut transport, &TestClock(10_000)); // pulse due, refused
        node.process(&mut transport, &TestClock(20_000)); // still refused

        let boot = transport.next_to_transmit().expect("queued boot-up frame");
        assert_eq!(boot.raw_id(), 0x701);
        assert!(
            transport.next_to_transmit().is_none(),
            "SYNC was queued despite a full TX queue"
        );

        // With room again, the retained pulse goes out carrying counter 1.
        node.process(&mut transport, &TestClock(30_000));
        let syncs = drain_frames_with_id(&transport, 0x080);
        assert_eq!(syncs.len(), 1);
        assert_eq!(syncs[0].data(), &[1], "counter advanced on refused sends");
    }

    #[test]
    fn nmt_reset_restarts_the_sync_producer_cycle() {
        let (mut node, mut transport) =
            make_sync_node(sync_od(crate::sync::SYNC_GEN | 0x080, 10_000, 3));

        node.process(&mut transport, &TestClock(10_000));
        node.process(&mut transport, &TestClock(20_000));
        let before = drain_frames_with_id(&transport, 0x080);
        assert_eq!(before.len(), 2);
        assert_eq!(before[1].data(), &[2]);

        // Reset Communication re-initialises communication objects. The OD
        // values are unchanged, so `configure` alone would keep the old phase
        // and counter — the reset has to clear them explicitly.
        transport
            .store_received(CanFrame::new(0x000, &[0x82, 0x01]).unwrap())
            .unwrap();
        node.process(&mut transport, &TestClock(30_000));
        while transport.next_to_transmit().is_some() {}

        // The next process call performs the boot sequence and establishes
        // the producer's new time base.
        node.process(&mut transport, &TestClock(31_000));
        while transport.next_to_transmit().is_some() {}

        // Re-armed: no pulse until a full period after the reset...
        node.process(&mut transport, &TestClock(40_999));
        assert!(
            drain_frames_with_id(&transport, 0x080).is_empty(),
            "pulse fired before the post-reset cycle elapsed"
        );
        // ...and the counter starts over.
        node.process(&mut transport, &TestClock(41_000));
        let after = drain_frames_with_id(&transport, 0x080);
        assert_eq!(after.len(), 1);
        assert_eq!(after[0].data(), &[1], "counter carried across the reset");
    }
}
