use crate::od::{ObjectDictionary, OdEvent, OdEventSource};
use crate::transport::CanFrame;
use heapless::{Deque, Vec};

/// Maximum PDO payload size in bits.
///
/// A classic CANopen PDO carries at most 8 data bytes = 64 bits. This is a
/// fixed CiA 301 / classic-CAN fact, independent of how many mapping entries a
/// PDO holds. Enforced against the sum of mapped entry bit lengths.
pub const PDO_MAX_PAYLOAD_BITS: usize = 64;

/// Default storage capacity for PDO mapping records.
///
/// With bit-granular mapping, the [`PDO_MAX_PAYLOAD_BITS`] budget permits up to
/// 64 one-bit mapped objects in one PDO, so mapping storage defaults to that
/// worst case. The payload-size limit is still enforced separately (a full 64
/// entries only fits if every entry is one bit).
///
/// This must equal the `PDO_MAX_MAPPINGS` in `canopen-derive`'s `dsl.rs`: the
/// derive crate sizes generated arrays against this constant but uses its own
/// copy for the parse-time entry-count check.
pub const PDO_MAX_MAPPINGS: usize = 64;

/// One PDO mapping entry: which OD entry maps to which bits in the PDO frame.
#[derive(Clone, Copy, Debug, Default)]
pub struct PdoMapping {
    /// OD index
    pub index: u16,
    /// OD subindex
    pub subindex: u8,
    /// Length in bits.
    pub bit_length: u8,
}

impl PdoMapping {
    /// Decode a CANopen PDO mapping value (0xIIIISSLL).
    pub const fn from_mapping_value(val: u32) -> Self {
        Self {
            index: (val >> 16) as u16,
            subindex: (val >> 8) as u8,
            bit_length: val as u8,
        }
    }

    /// Encode to CANopen PDO mapping value.
    pub const fn to_mapping_value(self) -> u32 {
        (self.index as u32) << 16 | (self.subindex as u32) << 8 | self.bit_length as u32
    }
}

/// CiA 301 dummy mapping: entries mapping a static data type carry no data —
/// TPDOs emit zero bits, RPDOs skip the bits without an OD write. Keep in
/// sync with the same range check in canopen-derive/src/eds_parser.rs.
const fn is_dummy_mapping_index(index: u16) -> bool {
    matches!(index, 0x0001..=0x0007)
}

const fn bytes_for_bits(bits: usize) -> usize {
    (bits + 7) / 8
}

// ---- Transmission type constants (CiA 301) ----

/// Synchronous, acyclic — sent only when triggered, on the next SYNC.
pub const SYNC_ACYCLIC: u8 = 0;
/// Synchronous, cyclic — sent every N SYNC messages. Use [`sync_cyclic`] to construct.
pub const SYNC_CYCLIC_1: u8 = 1;
/// Event-driven, manufacturer-specific trigger.
pub const EVENT_DRIVEN_MANUFACTURER: u8 = 254;
/// Event-driven, device-profile-specific trigger.
pub const EVENT_DRIVEN: u8 = 255;

/// Synchronous, cyclic transmission type: send every `n` SYNCs (1..=240).
pub const fn sync_cyclic(n: u8) -> u8 {
    assert!(n >= 1 && n <= 240, "sync_cyclic: n must be 1..=240");
    n
}

/// [`TransmissionType::is_synchronous`] on the raw wire value, for the engines
/// (which store transmission types as the `u8` the OD holds).
const fn raw_is_synchronous(raw: u8) -> bool {
    raw <= 240
}

/// PDO transmission type (CiA 301 §7.5.2.31).
///
/// Typed view of the raw `u8` transmission type. Both forms are accepted
/// everywhere: use the enum for readability, or raw values if you speak
/// fluent CANopen (`TransmissionType::from_raw(255)`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TransmissionType {
    /// 0 — synchronous acyclic: transmitted on the next SYNC after a trigger.
    SyncAcyclic,
    /// 1..=240 — synchronous cyclic: transmitted every N SYNC messages.
    SyncCyclic(u8),
    /// 252 — RTR-only, sampled on SYNC (TPDO only).
    RtrSync,
    /// 253 — RTR-only, event-driven (TPDO only).
    RtrEvent,
    /// 254 — event-driven, manufacturer-specific trigger.
    EventDrivenManufacturer,
    /// 255 — event-driven, device-profile-specific trigger.
    EventDriven,
}

impl TransmissionType {
    /// The raw CiA 301 wire value.
    pub const fn raw(self) -> u8 {
        match self {
            Self::SyncAcyclic => SYNC_ACYCLIC,
            Self::SyncCyclic(n) => {
                assert!(n >= 1 && n <= 240, "SyncCyclic: n must be 1..=240");
                n
            }
            Self::RtrSync => 252,
            Self::RtrEvent => 253,
            Self::EventDrivenManufacturer => EVENT_DRIVEN_MANUFACTURER,
            Self::EventDriven => EVENT_DRIVEN,
        }
    }

    /// True for the synchronous types (0..=240), which are the ones tied to
    /// the SYNC pulse: a TPDO transmits on it, an RPDO buffers received data
    /// until it. Everything else is event-driven or RTR.
    pub const fn is_synchronous(self) -> bool {
        matches!(self, Self::SyncAcyclic | Self::SyncCyclic(_))
    }

    /// Decode a raw CiA 301 value. Returns `None` for the reserved
    /// range 241..=251.
    pub const fn from_raw(raw: u8) -> Option<Self> {
        match raw {
            0 => Some(Self::SyncAcyclic),
            1..=240 => Some(Self::SyncCyclic(raw)),
            252 => Some(Self::RtrSync),
            253 => Some(Self::RtrEvent),
            254 => Some(Self::EventDrivenManufacturer),
            255 => Some(Self::EventDriven),
            _ => None,
        }
    }
}

impl From<TransmissionType> for u8 {
    fn from(tt: TransmissionType) -> u8 {
        tt.raw()
    }
}

/// CANopen PDO number (1..=512) — the `N` in `tpdo[N]`/`rpdo[N]`, the number
/// that determines the OD comm/mapping record addresses (`0x1400 + N - 1`,
/// …), and the numbering used by every public API.
///
/// Not to be confused with the dense engine slot: with sparse declarations
/// (`rpdo[1]` and `rpdo[79]`), RPDO79 sits in engine slot 1 but is always
/// addressed as PDO number 79. Slots are an internal storage detail.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PdoNumber(u16);

impl PdoNumber {
    pub const fn new(n: u16) -> Option<Self> {
        if n >= 1 && n <= 512 {
            Some(Self(n))
        } else {
            None
        }
    }

    /// Compile-time-checked constructor for literals: `PdoNumber::of::<79>()`.
    /// An out-of-range `N` fails the build, so no unwrap is needed.
    pub const fn of<const N: u16>() -> Self {
        const {
            assert!(N >= 1 && N <= 512, "PDO number must be 1..=512");
        }
        Self(N)
    }

    /// The raw 1-based CANopen PDO number.
    pub const fn raw(self) -> u16 {
        self.0
    }

    /// 0-based offset from the comm/mapping record range base
    /// (0x1400/0x1600/0x1800/0x1A00).
    pub const fn od_offset(self) -> u16 {
        self.0 - 1
    }
}

impl TryFrom<u16> for PdoNumber {
    type Error = ();
    fn try_from(n: u16) -> Result<Self, ()> {
        Self::new(n).ok_or(())
    }
}

impl From<PdoNumber> for u16 {
    fn from(n: PdoNumber) -> u16 {
        n.0
    }
}

/// CANopen number of the PDO stored in `slot`: the explicit `od_number` if
/// set, else slot + 1 (dense default).
pub(crate) fn pdo_number_for_slot(od_number: u16, slot: usize) -> u16 {
    if od_number == 0 {
        slot as u16 + 1
    } else {
        od_number
    }
}

/// Source of PDO configuration — implemented by `object_dictionary!`-generated
/// ODs, whose PDO declarations carry everything needed to build the configs.
///
/// [`crate::node::NodeConfig::from_od`] uses this to pull PDO configuration
/// straight from the OD instead of the application repeating it.
pub trait PdoConfigSource<const TPDO: usize, const RPDO: usize> {
    /// Build TPDO configs from current OD values. COB-IDs of 0 are resolved
    /// to the predefined defaults (0x180 + node_id, ...) using `node_id`.
    fn tpdo_configs(&self, node_id: crate::cobid::NodeId) -> [TpdoConfig; TPDO];
    /// Build RPDO configs from current OD values.
    fn rpdo_configs(&self, node_id: crate::cobid::NodeId) -> [RpdoConfig; RPDO];
}

/// Configuration for one TPDO (transmit PDO).
#[derive(Clone, Debug)]
pub struct TpdoConfig<const MAX_MAPPINGS: usize = PDO_MAX_MAPPINGS> {
    /// 1-based CANopen PDO number (TPDO1 = 1). Determines the OD index of the
    /// communication/mapping parameter records (0x1800/0x1A00 + number - 1).
    /// 0 = derive from the engine slot (slot n is PDO n+1).
    pub od_number: u16,
    /// COB-ID for this PDO
    pub cob_id: u16,
    /// Transmission type: 0=sync acyclic, 1-240=sync cyclic, 254/255=event-driven
    pub transmission_type: u8,
    /// Inhibit time in units of 100us
    pub inhibit_time_100us: u16,
    /// Event timer in ms (0=disabled)
    pub event_timer_ms: u16,
    /// PDO mappings
    pub mappings: Vec<PdoMapping, MAX_MAPPINGS>,
    /// Whether this PDO is enabled
    pub enabled: bool,
}

impl<const N: usize> Default for TpdoConfig<N> {
    fn default() -> Self {
        Self {
            od_number: 0,
            cob_id: 0,
            transmission_type: 255,
            inhibit_time_100us: 0,
            event_timer_ms: 0,
            mappings: Vec::new(),
            enabled: false,
        }
    }
}

impl<const N: usize> TpdoConfig<N> {
    /// A pure padding slot (`TpdoConfig::default()` in a hand-built array):
    /// nothing about it was declared, so it is not addressable by PDO number.
    /// Explicitly declared-but-disabled PDOs (nonzero `od_number` or COB-ID,
    /// or mappings) are not placeholders.
    pub(crate) fn is_placeholder(&self) -> bool {
        self.od_number == 0 && self.cob_id == 0 && !self.enabled && self.mappings.is_empty()
    }
}

/// Configuration for one RPDO (receive PDO).
#[derive(Clone, Debug)]
pub struct RpdoConfig<const MAX_MAPPINGS: usize = PDO_MAX_MAPPINGS> {
    /// 1-based CANopen PDO number (RPDO1 = 1). Determines the OD index of the
    /// communication/mapping parameter records (0x1400/0x1600 + number - 1).
    /// 0 = derive from the engine slot (slot n is PDO n+1).
    pub od_number: u16,
    /// COB-ID for this PDO
    pub cob_id: u16,
    /// Transmission type
    pub transmission_type: u8,
    /// Deadline monitoring timeout in ms (the CiA 301 "event timer", comm
    /// param sub 5 — the spec defines its RPDO function as deadline
    /// monitoring). 0 = disabled.
    ///
    /// Monitoring arms on first reception; if no frame arrives within the
    /// timeout, the slot's deadline-expired flag is set until the next
    /// reception re-arms it. The mapped OD entries are never touched — they
    /// keep the last received values.
    ///
    /// Configure with margin: the deadline must exceed the sender's period
    /// enough to absorb bus jitter and the receiver's `process()` tick
    /// (typically 1.5–2× the expected period, mirroring the heartbeat
    /// producer/consumer convention).
    pub deadline_ms: u16,
    /// PDO mappings
    pub mappings: Vec<PdoMapping, MAX_MAPPINGS>,
    /// Whether this PDO is enabled
    pub enabled: bool,
}

impl<const N: usize> Default for RpdoConfig<N> {
    fn default() -> Self {
        Self {
            od_number: 0,
            cob_id: 0,
            transmission_type: 255,
            deadline_ms: 0,
            mappings: Vec::new(),
            enabled: false,
        }
    }
}

impl<const N: usize> RpdoConfig<N> {
    /// A pure padding slot (`RpdoConfig::default()` in a hand-built array) —
    /// see [`TpdoConfig::is_placeholder`].
    pub(crate) fn is_placeholder(&self) -> bool {
        self.od_number == 0 && self.cob_id == 0 && !self.enabled && self.mappings.is_empty()
    }
}

/// TPDO engine. Manages up to N transmit PDOs.
pub struct TpdoEngine<const N: usize = 4> {
    pdos: [TpdoConfig; N],
    sync_counter: [u8; N],
    last_send_us: [u64; N],
    /// Application event latched for a synchronous acyclic TPDO (type 0),
    /// waiting for the SYNC that will transmit it.
    ///
    /// CiA 301 transmits type 0 on the SYNC *following* an application event,
    /// which is generally not the same processing cycle: the caller's dirty
    /// set is rebuilt and cleared every cycle, so the trigger has to be
    /// latched here or it is lost between pulses.
    sync_event_pending: [bool; N],
    /// Application event latched for an asynchronous event-driven TPDO
    /// (types 254/255), waiting for inhibit time to expire.
    ///
    /// Changes are coalesced per TPDO: when the PDO becomes eligible it samples
    /// the latest process image once. Keeping this state per PDO also means
    /// correctness does not depend on the caller's bounded dirty-address set.
    async_event_pending: [bool; N],
}

impl<const N: usize> TpdoEngine<N> {
    pub fn new(pdos: [TpdoConfig; N]) -> Self {
        Self {
            pdos,
            sync_counter: [0; N],
            last_send_us: [0; N],
            sync_event_pending: [false; N],
            async_event_pending: [false; N],
        }
    }

    /// Config of the TPDO with the given CANopen number (TPDO1 = 1), if declared.
    pub fn config(&self, number: PdoNumber) -> Option<&TpdoConfig> {
        self.slot_for_number(number).map(|s| &self.pdos[s])
    }

    /// Mutable config of the TPDO with the given CANopen number, if declared.
    pub fn config_mut(&mut self, number: PdoNumber) -> Option<&mut TpdoConfig> {
        self.slot_for_number(number).map(|s| &mut self.pdos[s])
    }

    pub(crate) fn config_slot(&self, slot: usize) -> Option<&TpdoConfig> {
        self.pdos.get(slot)
    }

    pub(crate) fn config_slot_mut(&mut self, slot: usize) -> Option<&mut TpdoConfig> {
        self.pdos.get_mut(slot)
    }

    /// Latch an application change in every TPDO which maps it.
    ///
    /// Type 0 retains the trigger until SYNC. Types 254/255 retain it until
    /// inhibit time permits transmission. Cyclic synchronous types ignore
    /// application changes.
    pub(crate) fn notify_changed(&mut self, index: u16, subindex: u8) {
        for slot in 0..N {
            if self.pdos[slot].enabled
                && self.pdos[slot]
                    .mappings
                    .iter()
                    .any(|m| m.index == index && m.subindex == subindex)
            {
                self.latch_slot_event(slot);
            }
        }
    }

    /// Latch an already mapping-resolved TPDO event.
    ///
    /// Used by [`crate::node::OdGuard`], which has already compared every
    /// mapping against its OD snapshot and can therefore avoid a bounded list
    /// of changed addresses entirely.
    pub(crate) fn latch_slot_event(&mut self, slot: usize) {
        let Some(pdo) = self.pdos.get(slot) else {
            return;
        };
        if !pdo.enabled {
            return;
        }
        match pdo.transmission_type {
            0 => self.sync_event_pending[slot] = true,
            254 | 255 => self.async_event_pending[slot] = true,
            _ => {}
        }
    }

    fn slot_for_number(&self, number: PdoNumber) -> Option<usize> {
        self.pdos.iter().enumerate().find_map(|(slot, c)| {
            (!c.is_placeholder() && pdo_number_for_slot(c.od_number, slot) == number.raw())
                .then_some(slot)
        })
    }

    /// Latch application events for synchronous acyclic TPDOs (type 0).
    ///
    /// Call once per processing cycle, before any SYNC in that cycle is
    /// consumed, with the same dirty set [`poll`](Self::poll) gets. The latch
    /// is what makes type 0 work across cycles: the dirty set is cleared at
    /// the end of every cycle, but the SYNC that should carry the event may
    /// be many cycles away. [`on_sync`](Self::on_sync) consumes it.
    pub fn latch_sync_events<const DIRTY: usize>(&mut self, dirty: &Vec<(u16, u8), DIRTY>) {
        if dirty.is_empty() {
            return;
        }
        for i in 0..N {
            if self.pdos[i].enabled
                && self.pdos[i].transmission_type == 0
                && self.has_dirty_mapping(i, dirty)
            {
                self.sync_event_pending[i] = true;
            }
        }
    }

    /// Drop every latched synchronous acyclic trigger.
    ///
    /// Called when leaving Operational, where PDOs stop flowing: an event
    /// that never reached a SYNC must not fire one on re-entry. Companion to
    /// [`RpdoEngine::discard_pending`].
    pub fn discard_sync_events(&mut self) {
        self.sync_event_pending = [false; N];
    }

    /// Drop asynchronous application events which never became eligible.
    ///
    /// PDO traffic stops outside Operational, so carrying an inhibited event
    /// across that boundary would emit stale work after re-entry. Companion to
    /// [`discard_sync_events`](Self::discard_sync_events).
    pub fn discard_async_events(&mut self) {
        self.async_event_pending = [false; N];
    }

    /// Reset all runtime transmission state without changing PDO config.
    ///
    /// NMT reset re-initialises communication objects, so cyclic TPDO
    /// divisors, acyclic application-event latches and inhibited asynchronous
    /// events all start fresh. Merely leaving Operational discards the latched
    /// events too, but keeps the cyclic divisors; it does not imply a
    /// communication reset.
    ///
    /// The asynchronous latch has to be cleared here as well as on the
    /// Operational boundary: a node with `auto_start` re-enters Operational in
    /// the same `process()` call that performs the reset, so the boundary path
    /// never runs and a pre-reset event would otherwise be transmitted once
    /// its inhibit time expired.
    pub(crate) fn reset_communication_state(&mut self) {
        self.sync_counter = [0; N];
        self.sync_event_pending = [false; N];
        self.async_event_pending = [false; N];
    }

    /// Called on SYNC reception. Returns frames to transmit for sync-triggered PDOs.
    ///
    /// Synchronous cyclic PDOs (1..=240) fire on their own SYNC count.
    /// Synchronous acyclic PDOs (type 0) fire once per application event
    /// latched by [`latch_sync_events`](Self::latch_sync_events).
    pub fn on_sync<OD: ObjectDictionary>(&mut self, od: &OD, out: &mut Vec<CanFrame, N>) {
        for i in 0..N {
            if !self.pdos[i].enabled {
                continue;
            }
            let tt = self.pdos[i].transmission_type;
            if tt == 0 {
                // Sync acyclic: one transmission per latched application
                // event, on the first SYNC after it.
                if core::mem::take(&mut self.sync_event_pending[i]) {
                    if let Some(frame) = self.build_pdo_frame(i, od) {
                        let _ = out.push(frame);
                    }
                }
                continue;
            }
            if tt >= 1 && tt <= 240 {
                // Sync cyclic - send every `tt` SYNCs
                self.sync_counter[i] += 1;
                if self.sync_counter[i] >= tt {
                    self.sync_counter[i] = 0;
                    if let Some(frame) = self.build_pdo_frame(i, od) {
                        let _ = out.push(frame);
                    }
                }
            }
            // 254/255 are event-driven, handled by poll()
        }
    }

    /// Does the dirty set contain any entry this PDO maps?
    fn has_dirty_mapping<const DIRTY: usize>(
        &self,
        pdo_idx: usize,
        dirty: &Vec<(u16, u8), DIRTY>,
    ) -> bool {
        !dirty.is_empty()
            && self.pdos[pdo_idx].mappings.iter().any(|m| {
                dirty
                    .iter()
                    .any(|&(idx, sub)| idx == m.index && sub == m.subindex)
            })
    }

    /// Called periodically. Checks event timers and dirty entries for event-driven PDOs.
    ///
    /// If any mapped entry is in the `dirty` set and the inhibit time has elapsed,
    /// the PDO is sent immediately (event-driven). The dirty set should be cleared
    /// by the caller after this returns.
    pub fn poll<OD: ObjectDictionary, const DIRTY: usize>(
        &mut self,
        od: &OD,
        now_us: u64,
        dirty: &Vec<(u16, u8), DIRTY>,
        out: &mut Vec<CanFrame, N>,
    ) {
        for i in 0..N {
            if !self.pdos[i].enabled {
                continue;
            }
            let tt = self.pdos[i].transmission_type;
            if tt != 254 && tt != 255 {
                continue;
            }

            let inhibit_us = self.pdos[i].inhibit_time_100us as u64 * 100;
            let elapsed = now_us.wrapping_sub(self.last_send_us[i]);

            // Latch the application event before checking inhibit time. The
            // caller clears its dirty set after this poll, while the event may
            // need to survive for several processing cycles.
            if self.has_dirty_mapping(i, dirty) {
                self.async_event_pending[i] = true;
            }

            // Send on dirty trigger (respecting inhibit time) or event timer
            let timer_trigger = self.pdos[i].event_timer_ms > 0 && {
                let interval_us = self.pdos[i].event_timer_ms as u64 * 1000;
                elapsed >= interval_us
            };

            let application_trigger = self.async_event_pending[i] && elapsed >= inhibit_us;
            if application_trigger || timer_trigger {
                if let Some(frame) = self.build_pdo_frame(i, od) {
                    if out.push(frame).is_ok() {
                        self.last_send_us[i] = now_us;
                        // A timer transmission carries the same latest process
                        // image and therefore also satisfies a pending event.
                        self.async_event_pending[i] = false;
                    }
                }
            }
        }
    }

    fn build_pdo_frame<OD: ObjectDictionary>(&self, pdo_idx: usize, od: &OD) -> Option<CanFrame> {
        let pdo = &self.pdos[pdo_idx];
        let mut data = [0u8; 8];
        let mut object_data = [0u8; 8];
        let mut bit_offset: usize = 0;

        for mapping in &pdo.mappings {
            let bit_len = mapping.bit_length as usize;
            let end_bit = bit_offset + bit_len;
            if end_bit > 64 {
                return None; // PDO too long
            }
            if is_dummy_mapping_index(mapping.index) {
                bit_offset = end_bit;
                continue;
            }
            let object_len = bytes_for_bits(bit_len);
            if od
                .read(
                    mapping.index,
                    mapping.subindex,
                    &mut object_data[..object_len],
                )
                .is_err()
            {
                return None;
            }
            super::copy_pdo_bits(
                &object_data[..object_len],
                0,
                &mut data,
                bit_offset,
                bit_len,
            );
            object_data[..object_len].fill(0);
            bit_offset = end_bit;
        }

        let total_bytes = (bit_offset + 7) / 8;
        CanFrame::new(pdo.cob_id, &data[..total_bytes])
    }
}

/// Unpack a received RPDO frame into the OD along `pdo`'s mappings, pushing
/// one `OdEvent` per entry successfully written. Returns the number of events
/// dropped because the queue was full.
///
/// Shared by immediate (event-driven) and deferred (synchronous, applied on
/// SYNC) commits so both produce identical OD writes and events.
fn apply_mapped<OD: ObjectDictionary, const EVT_QUEUE: usize>(
    pdo: &RpdoConfig,
    data: &[u8],
    od: &mut OD,
    events: &mut Deque<OdEvent, EVT_QUEUE>,
) -> u32 {
    let mut bit_offset: usize = 0;
    let mut object_data = [0u8; 8];
    let mut dropped = 0u32;

    for mapping in &pdo.mappings {
        let bit_len = mapping.bit_length as usize;
        let end_bit = bit_offset + bit_len;
        let object_len = bytes_for_bits(bit_len);
        if end_bit > data.len() * 8 || object_len > object_data.len() {
            break;
        }
        if is_dummy_mapping_index(mapping.index) {
            bit_offset = end_bit;
            continue;
        }
        super::copy_pdo_bits(data, bit_offset, &mut object_data, 0, bit_len);
        if od
            .write(mapping.index, mapping.subindex, &object_data[..object_len])
            .is_ok()
        {
            if events.is_full() {
                let _ = events.pop_front();
                dropped = dropped.saturating_add(1);
            }
            let _ = events.push_back(OdEvent {
                index: mapping.index,
                subindex: mapping.subindex,
                source: OdEventSource::Rpdo,
            });
        }
        object_data[..object_len].fill(0);
        bit_offset = end_bit;
    }
    dropped
}

/// Exact classic-CAN payload length implied by an RPDO mapping.
///
/// A configuration exceeding the 64-bit PDO budget is structurally invalid
/// and therefore has no acceptable wire length.
fn mapped_payload_len(pdo: &RpdoConfig) -> Option<usize> {
    let total_bits = pdo
        .mappings
        .iter()
        .map(|mapping| mapping.bit_length as usize)
        .sum::<usize>();
    (total_bits <= PDO_MAX_PAYLOAD_BITS).then_some(bytes_for_bits(total_bits))
}

/// RPDO engine. Manages up to N receive PDOs.
///
/// Synchronous RPDOs (transmission type 0..=240) do not write the OD at
/// reception: their frame is buffered and committed by
/// [`on_sync`](Self::on_sync), so a set of setpoints spread over several PDOs
/// becomes visible to the application as one coherent update. Event-driven
/// types (254/255) commit immediately, at reception.
///
/// Optionally performs deadline monitoring per slot (CiA 301 RPDO event
/// timer): when [`RpdoConfig::deadline_ms`] is nonzero, monitoring arms on
/// the first received frame and [`check_deadlines`](Self::check_deadlines)
/// flags slots whose silence exceeds the deadline. Detection latency equals
/// the caller's polling period. Monitoring watches the *bus*, so it is
/// stamped at reception for synchronous RPDOs too, not at SYNC commit.
pub struct RpdoEngine<const N: usize = 4> {
    pdos: [RpdoConfig; N],
    /// Frame received by a synchronous RPDO, awaiting the next SYNC. Latest
    /// frame wins — CiA 301 keeps only the most recent data per PDO.
    pending: [Option<CanFrame>; N],
    /// Timestamp of the last received frame per slot (valid when `armed`).
    last_rx_us: [u64; N],
    /// Deadline monitoring armed: at least one frame received since
    /// (re)entering Operational.
    armed: [bool; N],
    /// Deadline currently expired (set by `check_deadlines`, cleared by the
    /// next reception). Mapped OD entries keep their last received values.
    expired: [bool; N],
}

impl<const N: usize> RpdoEngine<N> {
    pub fn new(pdos: [RpdoConfig; N]) -> Self {
        Self {
            pdos,
            pending: [None; N],
            last_rx_us: [0; N],
            armed: [false; N],
            expired: [false; N],
        }
    }

    /// Config of the RPDO with the given CANopen number (RPDO1 = 1), if declared.
    pub fn config(&self, number: PdoNumber) -> Option<&RpdoConfig> {
        self.slot_for_number(number).map(|s| &self.pdos[s])
    }

    /// Mutable config of the RPDO with the given CANopen number, if declared.
    pub fn config_mut(&mut self, number: PdoNumber) -> Option<&mut RpdoConfig> {
        self.slot_for_number(number).map(|s| &mut self.pdos[s])
    }

    pub(crate) fn config_slot(&self, slot: usize) -> Option<&RpdoConfig> {
        self.pdos.get(slot)
    }

    pub(crate) fn config_slot_mut(&mut self, slot: usize) -> Option<&mut RpdoConfig> {
        self.pdos.get_mut(slot)
    }

    fn slot_for_number(&self, number: PdoNumber) -> Option<usize> {
        self.pdos.iter().enumerate().find_map(|(slot, c)| {
            (!c.is_placeholder() && pdo_number_for_slot(c.od_number, slot) == number.raw())
                .then_some(slot)
        })
    }

    /// Process an incoming CAN frame. If it matches an RPDO, write mapped values to OD.
    /// Returns true if the frame was consumed by an RPDO.
    ///
    /// Pushes one `OdEvent` per successfully written mapped entry.
    /// A matching frame is consumed even when its payload length does not
    /// exactly match the configured mapping. Such a malformed frame is
    /// otherwise ignored: it neither writes/buffers data nor refreshes the
    /// deadline. For a structurally valid frame, mapped OD writes remain
    /// independent; one rejected write does not suppress the other entries.
    ///
    /// `now_us` stamps a structurally valid reception for deadline monitoring;
    /// reception re-arms an expired deadline.
    ///
    /// A frame for a *synchronous* RPDO is consumed (returns true) but only
    /// buffered — it writes the OD and emits its events on the next
    /// [`on_sync`](Self::on_sync).
    pub fn process<OD: ObjectDictionary, const EVT_QUEUE: usize>(
        &mut self,
        frame: &CanFrame,
        od: &mut OD,
        events: &mut Deque<OdEvent, EVT_QUEUE>,
        now_us: u64,
    ) -> bool {
        self.process_with_drop_count(frame, od, events, now_us).0
    }

    pub fn process_with_drop_count<OD: ObjectDictionary, const EVT_QUEUE: usize>(
        &mut self,
        frame: &CanFrame,
        od: &mut OD,
        events: &mut Deque<OdEvent, EVT_QUEUE>,
        now_us: u64,
    ) -> (bool, u32) {
        let Some(slot) = self
            .pdos
            .iter()
            .position(|pdo| pdo.enabled && frame.raw_id() == pdo.cob_id)
        else {
            return (false, 0);
        };

        if mapped_payload_len(&self.pdos[slot]) != Some(frame.data().len()) {
            return (true, 0);
        }

        // Deadline monitoring watches the bus, so it is stamped here even when
        // the data itself waits for a SYNC.
        self.last_rx_us[slot] = now_us;
        self.armed[slot] = true;
        self.expired[slot] = false;

        if raw_is_synchronous(self.pdos[slot].transmission_type) {
            self.pending[slot] = Some(*frame);
            return (true, 0);
        }

        (
            true,
            apply_mapped(&self.pdos[slot], frame.data(), od, events),
        )
    }

    /// Commit every buffered synchronous RPDO to the OD, in slot order.
    ///
    /// Returns the number of `OdEvent`s dropped to queue overflow. Call this
    /// on the SYNC pulse, *before* sampling synchronous TPDOs, so a TPDO in
    /// the same pulse sees the post-SYNC state.
    pub fn on_sync<OD: ObjectDictionary, const EVT_QUEUE: usize>(
        &mut self,
        od: &mut OD,
        events: &mut Deque<OdEvent, EVT_QUEUE>,
    ) -> u32 {
        let mut dropped = 0u32;
        for slot in 0..N {
            let Some(frame) = self.pending[slot].take() else {
                continue;
            };
            dropped =
                dropped.saturating_add(apply_mapped(&self.pdos[slot], frame.data(), od, events));
        }
        dropped
    }

    /// Is a frame buffered for the RPDO with the given CANopen number, waiting
    /// for the next SYNC? Always false for event-driven RPDOs, which never
    /// buffer.
    pub fn has_pending(&self, number: PdoNumber) -> bool {
        match self.slot_for_number(number) {
            Some(slot) => self.pending[slot].is_some(),
            None => false,
        }
    }

    /// Drop every buffered synchronous RPDO without applying it.
    ///
    /// Called when leaving Operational: data that never reached a SYNC has no
    /// business landing in the OD after the node has stopped processing PDOs.
    /// Companion to [`reset_deadline_monitoring`](Self::reset_deadline_monitoring).
    pub fn discard_pending(&mut self) {
        self.pending = [None; N];
    }

    /// Check all RPDOs for expired deadlines. Pushes the CANopen PDO number
    /// of each *newly* expired RPDO into `out` (edge-triggered: fires once
    /// per silence period and re-arms on the next reception).
    ///
    /// An RPDO is monitored when it is enabled, has a nonzero `deadline_ms`,
    /// and has received at least one frame (monitoring starts on first
    /// reception — before the counterpart ever spoke, its silence is not an
    /// error). The comparison is strictly `elapsed > deadline`.
    pub fn check_deadlines(&mut self, now_us: u64, out: &mut Vec<PdoNumber, N>) {
        for slot in 0..N {
            let pdo = &self.pdos[slot];
            if !pdo.enabled || pdo.deadline_ms == 0 || !self.armed[slot] || self.expired[slot] {
                continue;
            }
            let deadline_us = pdo.deadline_ms as u64 * 1000;
            if now_us.wrapping_sub(self.last_rx_us[slot]) > deadline_us {
                self.expired[slot] = true;
                // Out-of-range od_number (>512 in a hand-built config) still
                // sets the level flag but cannot be reported as a number.
                if let Some(number) = PdoNumber::new(pdo_number_for_slot(pdo.od_number, slot)) {
                    let _ = out.push(number);
                }
            }
        }
    }

    /// Level query: does the RPDO with the given CANopen number lack an
    /// in-deadline reception?
    ///
    /// `true` whenever a monitored RPDO (enabled, nonzero `deadline_ms`) has
    /// no fresh data: **initially, before the first frame ever arrives**, and
    /// again from expiry until the next reception. Unmonitored or undeclared
    /// RPDOs always read `false`. The mapped OD entries keep their last
    /// received values throughout.
    ///
    /// This is deliberately broader than the edge-triggered
    /// [`check_deadlines`](Self::check_deadlines) channel, which only fires
    /// after a counterpart that *has* spoken goes silent.
    pub fn deadline_expired(&self, number: PdoNumber) -> bool {
        match self.slot_for_number(number) {
            Some(slot) => self.deadline_expired_slot(slot),
            None => false,
        }
    }

    pub(crate) fn deadline_expired_slot(&self, slot: usize) -> bool {
        let Some(pdo) = self.pdos.get(slot) else {
            return false;
        };
        pdo.enabled && pdo.deadline_ms > 0 && (!self.armed[slot] || self.expired[slot])
    }

    /// Forget all deadline monitoring state (arm flags, expiry flags).
    ///
    /// Called when leaving Operational: PDO traffic legitimately stops there,
    /// so silence must not count against the deadline. Monitoring re-arms per
    /// slot on the first reception after returning to Operational.
    pub fn reset_deadline_monitoring(&mut self) {
        self.armed = [false; N];
        self.expired = [false; N];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datatypes::DataType;
    use crate::od::*;

    /// An empty dirty set, for TPDO tests that do not exercise the
    /// application-event trigger (sync-cyclic and event-timer paths).
    fn no_dirty() -> Vec<(u16, u8), 8> {
        Vec::new()
    }

    /// A dirty set naming a single OD entry.
    fn dirty(index: u16, subindex: u8) -> Vec<(u16, u8), 8> {
        let mut v = Vec::new();
        v.push((index, subindex)).unwrap();
        v
    }

    struct PdoTestOd {
        input1: u8,
        input2: u16,
        input_flag1: bool,
        input_flag2: bool,
        output1: u8,
        output_flag1: bool,
        output_flag2: bool,
    }

    static PDO_TEST_META: &[OdEntryMeta] = &[
        OdEntryMeta {
            index: 0x6000,
            subindex: 1,
            data_type: DataType::U8,
            access: AccessType::Ro,
            pdo_mappable: true,
            name: "input1",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x6000,
            subindex: 2,
            data_type: DataType::U16,
            access: AccessType::Ro,
            pdo_mappable: true,
            name: "input2",
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
            index: 0x6001,
            subindex: 1,
            data_type: DataType::Boolean,
            access: AccessType::Ro,
            pdo_mappable: true,
            name: "input_flag1",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x6001,
            subindex: 2,
            data_type: DataType::Boolean,
            access: AccessType::Ro,
            pdo_mappable: true,
            name: "input_flag2",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x6201,
            subindex: 1,
            data_type: DataType::Boolean,
            access: AccessType::Rw,
            pdo_mappable: true,
            name: "output_flag1",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x6201,
            subindex: 2,
            data_type: DataType::Boolean,
            access: AccessType::Rw,
            pdo_mappable: true,
            name: "output_flag2",
            max_size: None,
        },
    ];

    impl ObjectDictionary for PdoTestOd {
        fn lookup(&self, index: u16, subindex: u8) -> Option<&'static OdEntryMeta> {
            PDO_TEST_META
                .iter()
                .find(|e| e.index == index && e.subindex == subindex)
        }
        fn read(&self, index: u16, subindex: u8, buf: &mut [u8]) -> Result<usize, OdError> {
            match (index, subindex) {
                (0x6000, 1) => {
                    buf[0] = self.input1;
                    Ok(1)
                }
                (0x6000, 2) => {
                    buf[..2].copy_from_slice(&self.input2.to_le_bytes());
                    Ok(2)
                }
                (0x6200, 1) => {
                    buf[0] = self.output1;
                    Ok(1)
                }
                (0x6001, 1) => {
                    buf[0] = u8::from(self.input_flag1);
                    Ok(1)
                }
                (0x6001, 2) => {
                    buf[0] = u8::from(self.input_flag2);
                    Ok(1)
                }
                (0x6201, 1) => {
                    buf[0] = u8::from(self.output_flag1);
                    Ok(1)
                }
                (0x6201, 2) => {
                    buf[0] = u8::from(self.output_flag2);
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
                (0x6201, 1) => {
                    self.output_flag1 = data[0] != 0;
                    Ok(())
                }
                (0x6201, 2) => {
                    self.output_flag2 = data[0] != 0;
                    Ok(())
                }
                _ => Err(OdError::ReadOnly),
            }
        }
        fn sub_count(&self, _index: u16) -> Option<u8> {
            None
        }
    }

    #[test]
    fn tpdo_sync_cyclic() {
        let mut mappings = Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                index: 0x6000,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();
        mappings
            .push(PdoMapping {
                index: 0x6000,
                subindex: 2,
                bit_length: 16,
            })
            .unwrap();

        let config = TpdoConfig {
            od_number: 0,
            cob_id: 0x181,
            transmission_type: 1, // every SYNC
            inhibit_time_100us: 0,
            event_timer_ms: 0,
            mappings,
            enabled: true,
        };

        let mut engine = TpdoEngine::new([config]);
        let od = PdoTestOd {
            input1: 0x42,
            input2: 0x1234,
            input_flag1: false,
            input_flag2: false,
            output1: 0,
            output_flag1: false,
            output_flag2: false,
        };
        let mut out = Vec::<CanFrame, 1>::new();

        engine.on_sync(&od, &mut out);
        assert_eq!(out.len(), 1);

        let frame = &out[0];
        assert_eq!(frame.raw_id(), 0x181);
        assert_eq!(frame.data(), &[0x42, 0x34, 0x12]); // u8 + u16 LE
    }

    #[test]
    fn rpdo_process() {
        let mut mappings = Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                index: 0x6200,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();

        let config = RpdoConfig {
            od_number: 0,
            cob_id: 0x201,
            transmission_type: 255,
            deadline_ms: 0,
            mappings,
            enabled: true,
        };

        let mut engine = RpdoEngine::new([config]);
        let mut od = PdoTestOd {
            input1: 0,
            input2: 0,
            input_flag1: false,
            input_flag2: false,
            output1: 0,
            output_flag1: false,
            output_flag2: false,
        };
        let mut events: Deque<OdEvent, 16> = Deque::new();

        let frame = CanFrame::new(0x201, &[0xFF]).unwrap();
        assert!(engine.process(&frame, &mut od, &mut events, 0));
        assert_eq!(od.output1, 0xFF);

        // Should have generated an RPDO event
        let evt = events.pop_front().unwrap();
        assert_eq!(evt.index, 0x6200);
        assert_eq!(evt.subindex, 1);
        assert_eq!(evt.source, OdEventSource::Rpdo);

        // Non-matching frame
        let frame2 = CanFrame::new(0x301, &[0x00]).unwrap();
        assert!(!engine.process(&frame2, &mut od, &mut events, 0));
        assert_eq!(od.output1, 0xFF); // unchanged
    }

    #[test]
    fn tpdo_packs_boolean_mappings_as_bits() {
        let mut mappings = Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                index: 0x6001,
                subindex: 1,
                bit_length: 1,
            })
            .unwrap();
        mappings
            .push(PdoMapping {
                index: 0x6001,
                subindex: 2,
                bit_length: 1,
            })
            .unwrap();
        mappings
            .push(PdoMapping {
                index: 0x6000,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();

        let config = TpdoConfig {
            od_number: 0,
            cob_id: 0x181,
            transmission_type: 1,
            inhibit_time_100us: 0,
            event_timer_ms: 0,
            mappings,
            enabled: true,
        };

        let mut engine = TpdoEngine::new([config]);
        let od = PdoTestOd {
            input1: 0b1010_1100,
            input2: 0,
            input_flag1: true,
            input_flag2: false,
            output1: 0,
            output_flag1: false,
            output_flag2: false,
        };
        let mut out = Vec::<CanFrame, 1>::new();

        engine.on_sync(&od, &mut out);

        assert_eq!(out[0].data(), &[0b1011_0001, 0b0000_0010]);
    }

    #[test]
    fn rpdo_unpacks_boolean_mappings_from_bits() {
        let mut mappings = Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                index: 0x6201,
                subindex: 1,
                bit_length: 1,
            })
            .unwrap();
        mappings
            .push(PdoMapping {
                index: 0x6201,
                subindex: 2,
                bit_length: 1,
            })
            .unwrap();
        mappings
            .push(PdoMapping {
                index: 0x6200,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();

        let config = RpdoConfig {
            od_number: 0,
            cob_id: 0x201,
            transmission_type: 255,
            deadline_ms: 0,
            mappings,
            enabled: true,
        };

        let mut engine = RpdoEngine::new([config]);
        let mut od = PdoTestOd {
            input1: 0,
            input2: 0,
            input_flag1: false,
            input_flag2: false,
            output1: 0,
            output_flag1: false,
            output_flag2: false,
        };
        let mut events: Deque<OdEvent, 16> = Deque::new();
        let frame = CanFrame::new(0x201, &[0b1110_0101, 0b0000_0010]).unwrap();

        assert!(engine.process(&frame, &mut od, &mut events, 0));

        assert!(od.output_flag1);
        assert!(!od.output_flag2);
        assert_eq!(od.output1, 0b1011_1001);
    }

    #[test]
    fn tpdo_dummy_mapping_pads_with_zero_bits() {
        let mut mappings = Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                index: 0x6001,
                subindex: 1,
                bit_length: 1,
            })
            .unwrap();
        mappings
            .push(PdoMapping {
                index: 0x0001,
                subindex: 0,
                bit_length: 7,
            })
            .unwrap();
        mappings
            .push(PdoMapping {
                index: 0x6000,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();

        let config = TpdoConfig {
            od_number: 0,
            cob_id: 0x181,
            transmission_type: 1,
            inhibit_time_100us: 0,
            event_timer_ms: 0,
            mappings,
            enabled: true,
        };

        let mut engine = TpdoEngine::new([config]);
        let od = PdoTestOd {
            input1: 0xA5,
            input2: 0,
            input_flag1: true,
            input_flag2: false,
            output1: 0,
            output_flag1: false,
            output_flag2: false,
        };
        let mut out = Vec::<CanFrame, 1>::new();

        engine.on_sync(&od, &mut out);

        assert_eq!(out[0].data(), &[0x01, 0xA5]);
    }

    #[test]
    fn rpdo_dummy_mapping_skips_padding_bits() {
        let mut mappings = Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                index: 0x6201,
                subindex: 1,
                bit_length: 1,
            })
            .unwrap();
        mappings
            .push(PdoMapping {
                index: 0x0001,
                subindex: 0,
                bit_length: 7,
            })
            .unwrap();
        mappings
            .push(PdoMapping {
                index: 0x6200,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();

        let config = RpdoConfig {
            od_number: 0,
            cob_id: 0x201,
            transmission_type: 255,
            deadline_ms: 0,
            mappings,
            enabled: true,
        };

        let mut engine = RpdoEngine::new([config]);
        let mut od = PdoTestOd {
            input1: 0,
            input2: 0,
            input_flag1: false,
            input_flag2: false,
            output1: 0,
            output_flag1: false,
            output_flag2: false,
        };
        let mut events: Deque<OdEvent, 16> = Deque::new();
        let frame = CanFrame::new(0x201, &[0x01, 0xA5]).unwrap();

        assert!(engine.process(&frame, &mut od, &mut events, 0));

        assert!(od.output_flag1);
        assert_eq!(od.output1, 0xA5);
        assert_eq!(events.len(), 2);
    }

    fn deadline_test_engine(deadline_ms: u16) -> RpdoEngine<1> {
        let mut mappings = Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                index: 0x6200,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();
        RpdoEngine::new([RpdoConfig {
            od_number: 0,
            cob_id: 0x201,
            transmission_type: 255,
            deadline_ms,
            mappings,
            enabled: true,
        }])
    }

    fn deadline_test_od() -> PdoTestOd {
        PdoTestOd {
            input1: 0,
            input2: 0,
            input_flag1: false,
            input_flag2: false,
            output1: 0,
            output_flag1: false,
            output_flag2: false,
        }
    }

    fn two_entry_rpdo(transmission_type: u8, deadline_ms: u16) -> RpdoEngine<1> {
        let mut mappings = Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                index: 0x6200,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();
        mappings
            .push(PdoMapping {
                index: 0x6201,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();
        RpdoEngine::new([RpdoConfig {
            od_number: 0,
            cob_id: 0x201,
            transmission_type,
            deadline_ms,
            mappings,
            enabled: true,
        }])
    }

    #[test]
    fn malformed_rpdo_length_changes_nothing_and_does_not_refresh_deadline() {
        let mut engine = two_entry_rpdo(EVENT_DRIVEN, 100);
        let mut od = deadline_test_od();
        let mut events: Deque<OdEvent, 16> = Deque::new();

        let valid = CanFrame::new(0x201, &[0x11, 1]).unwrap();
        assert!(engine.process(&valid, &mut od, &mut events, 0));
        assert_eq!(od.output1, 0x11);
        assert!(od.output_flag1);
        events.clear();

        let mut expired = Vec::<PdoNumber, 1>::new();
        engine.check_deadlines(100_001, &mut expired);
        assert!(engine.deadline_expired(PdoNumber::of::<1>()));

        // Both frames match the RPDO COB-ID and are consumed, but neither is
        // a complete two-byte process image.
        let short = CanFrame::new(0x201, &[0x22]).unwrap();
        assert!(engine.process(&short, &mut od, &mut events, 200_000));
        let long = CanFrame::new(0x201, &[0x33, 0, 0x44]).unwrap();
        assert!(engine.process(&long, &mut od, &mut events, 300_000));

        assert_eq!(od.output1, 0x11);
        assert!(od.output_flag1);
        assert!(events.is_empty());
        assert!(engine.deadline_expired(PdoNumber::of::<1>()));
    }

    #[test]
    fn structurally_valid_rpdo_continues_after_one_od_write_is_rejected() {
        let mut mappings = Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                // PdoTestOd rejects writes to this read-only input.
                index: 0x6000,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();
        mappings
            .push(PdoMapping {
                index: 0x6200,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();
        let mut engine = RpdoEngine::new([RpdoConfig {
            od_number: 0,
            cob_id: 0x201,
            transmission_type: EVENT_DRIVEN,
            deadline_ms: 100,
            mappings,
            enabled: true,
        }]);
        let mut od = deadline_test_od();
        let mut events: Deque<OdEvent, 16> = Deque::new();

        let frame = CanFrame::new(0x201, &[0xAA, 0x55]).unwrap();
        assert!(engine.process(&frame, &mut od, &mut events, 10_000));

        assert_eq!(od.input1, 0);
        assert_eq!(od.output1, 0x55);
        assert_eq!(events.len(), 1);
        let event = events.pop_front().unwrap();
        assert_eq!((event.index, event.subindex), (0x6200, 1));
        assert!(!engine.deadline_expired(PdoNumber::of::<1>()));
    }

    #[test]
    fn pdo_number_bounds_and_offset() {
        assert!(PdoNumber::new(0).is_none());
        assert!(PdoNumber::new(513).is_none());
        assert_eq!(PdoNumber::new(1).unwrap().raw(), 1);
        assert_eq!(PdoNumber::of::<512>().od_offset(), 511);
        assert_eq!(PdoNumber::try_from(79u16).unwrap(), PdoNumber::of::<79>());
        assert_eq!(u16::from(PdoNumber::of::<79>()), 79);
    }

    #[test]
    fn sparse_rpdo_addressed_by_number_not_slot() {
        // Slot 0 = RPDO1, slot 1 = RPDO79 (sparse). Number-based lookups must
        // hit the right config; undeclared numbers answer None.
        let mut cfg1 = RpdoConfig {
            od_number: 1,
            cob_id: 0x201,
            enabled: true,
            ..RpdoConfig::default()
        };
        cfg1.deadline_ms = 0;
        let cfg79 = RpdoConfig {
            od_number: 79,
            cob_id: 0x351,
            deadline_ms: 100,
            enabled: true,
            ..RpdoConfig::default()
        };
        let engine = RpdoEngine::new([cfg1, cfg79]);

        assert_eq!(engine.config(PdoNumber::of::<1>()).unwrap().cob_id, 0x201);
        assert_eq!(engine.config(PdoNumber::of::<79>()).unwrap().cob_id, 0x351);
        assert!(engine.config(PdoNumber::of::<2>()).is_none());

        // RPDO79 is monitored and unarmed -> flag up; RPDO1 unmonitored.
        assert!(engine.deadline_expired(PdoNumber::of::<79>()));
        assert!(!engine.deadline_expired(PdoNumber::of::<1>()));
    }

    #[test]
    fn placeholder_configs_not_addressable_by_number() {
        // Hand-built capacity array: one real RPDO + one default() padding
        // slot. The placeholder must not be addressable as "RPDO2".
        let real = RpdoConfig {
            od_number: 0,
            cob_id: 0x201,
            enabled: true,
            ..RpdoConfig::default()
        };
        let engine = RpdoEngine::new([real, RpdoConfig::default()]);
        assert!(engine.config(PdoNumber::of::<1>()).is_some());
        assert!(engine.config(PdoNumber::of::<2>()).is_none());
        assert!(!engine.deadline_expired(PdoNumber::of::<2>()));

        // Explicitly declared-but-disabled PDOs stay addressable.
        let disabled = TpdoConfig {
            od_number: 3,
            cob_id: 0x183,
            enabled: false,
            ..TpdoConfig::default()
        };
        let tengine = TpdoEngine::new([disabled]);
        assert!(tengine.config(PdoNumber::of::<3>()).is_some());
    }

    #[test]
    fn rpdo_deadline_not_armed_before_first_reception() {
        let mut engine = deadline_test_engine(100);
        let mut expired = Vec::<PdoNumber, 1>::new();

        // No frame ever received: the level flag reads true (no in-deadline
        // data exists yet), but the edge channel stays silent, however long —
        // a counterpart that never spoke cannot "go silent".
        assert!(engine.deadline_expired(PdoNumber::of::<1>()));
        engine.check_deadlines(10_000_000, &mut expired);
        assert!(expired.is_empty());
        assert!(engine.deadline_expired(PdoNumber::of::<1>()));
    }

    #[test]
    fn rpdo_deadline_edge_triggers_once_and_rearms_on_reception() {
        let mut engine = deadline_test_engine(100);
        let mut od = deadline_test_od();
        let mut events: Deque<OdEvent, 16> = Deque::new();
        let frame = CanFrame::new(0x201, &[0x11]).unwrap();

        // First reception at t=0 arms monitoring.
        assert!(engine.process(&frame, &mut od, &mut events, 0));

        // Exactly at the deadline: not expired (strictly greater than).
        let mut expired = Vec::<PdoNumber, 1>::new();
        engine.check_deadlines(100_000, &mut expired);
        assert!(expired.is_empty());

        // Past the deadline: fires once...
        engine.check_deadlines(100_001, &mut expired);
        assert_eq!(expired.as_slice(), &[PdoNumber::of::<1>()]);
        assert!(engine.deadline_expired(PdoNumber::of::<1>()));

        // ...and only once per silence period (edge-triggered).
        expired.clear();
        engine.check_deadlines(200_000, &mut expired);
        assert!(expired.is_empty());
        assert!(engine.deadline_expired(PdoNumber::of::<1>())); // level flag stays up

        // OD keeps the last received value throughout the timeout.
        assert_eq!(od.output1, 0x11);

        // Reception clears the flag and re-arms monitoring.
        assert!(engine.process(&frame, &mut od, &mut events, 300_000));
        assert!(!engine.deadline_expired(PdoNumber::of::<1>()));
        engine.check_deadlines(350_000, &mut expired);
        assert!(expired.is_empty());

        // A second silence period fires again.
        engine.check_deadlines(400_001, &mut expired);
        assert_eq!(expired.as_slice(), &[PdoNumber::of::<1>()]);
    }

    #[test]
    fn rpdo_deadline_disabled_when_timer_zero() {
        let mut engine = deadline_test_engine(0);
        let mut od = deadline_test_od();
        let mut events: Deque<OdEvent, 16> = Deque::new();
        let frame = CanFrame::new(0x201, &[0x22]).unwrap();

        assert!(engine.process(&frame, &mut od, &mut events, 0));
        let mut expired = Vec::<PdoNumber, 1>::new();
        engine.check_deadlines(10_000_000, &mut expired);
        assert!(expired.is_empty());
        assert!(!engine.deadline_expired(PdoNumber::of::<1>()));
    }

    #[test]
    fn rpdo_deadline_reset_disarms_monitoring() {
        let mut engine = deadline_test_engine(100);
        let mut od = deadline_test_od();
        let mut events: Deque<OdEvent, 16> = Deque::new();
        let frame = CanFrame::new(0x201, &[0x33]).unwrap();

        assert!(engine.process(&frame, &mut od, &mut events, 0));
        let mut expired = Vec::<PdoNumber, 1>::new();
        engine.check_deadlines(200_000, &mut expired);
        assert!(engine.deadline_expired(PdoNumber::of::<1>()));

        // Leaving Operational: edge state cleared, silence no longer counts.
        // The level flag reads true again (back to "no in-deadline data"),
        // but no new edge fires until a frame arrives and traffic stops again.
        engine.reset_deadline_monitoring();
        assert!(engine.deadline_expired(PdoNumber::of::<1>()));
        expired.clear();
        engine.check_deadlines(10_000_000, &mut expired);
        assert!(expired.is_empty());
    }

    #[test]
    fn tpdo_event_timer() {
        let mut mappings = Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                index: 0x6000,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();

        let config = TpdoConfig {
            od_number: 0,
            cob_id: 0x181,
            transmission_type: 255, // event-driven
            inhibit_time_100us: 0,
            event_timer_ms: 100, // every 100ms
            mappings,
            enabled: true,
        };

        let mut engine = TpdoEngine::new([config]);
        let od = PdoTestOd {
            input1: 0x99,
            input2: 0,
            input_flag1: false,
            input_flag2: false,
            output1: 0,
            output_flag1: false,
            output_flag2: false,
        };
        let dirty = Vec::<(u16, u8), 8>::new();

        // At t=0, diff is 0 which is < 100ms
        let mut out = Vec::<CanFrame, 1>::new();
        engine.poll(&od, 0, &dirty, &mut out);
        assert_eq!(out.len(), 0);

        // At t=100ms - first send
        engine.poll(&od, 100_000, &dirty, &mut out);
        assert_eq!(out.len(), 1);

        // 150ms later - too early
        out.clear();
        engine.poll(&od, 150_000, &dirty, &mut out);
        assert_eq!(out.len(), 0);

        // 200ms - due again
        out.clear();
        engine.poll(&od, 200_000, &dirty, &mut out);
        assert_eq!(out.len(), 1);
    }

    // ---- SYNC: synchronous RPDO buffering, synchronous acyclic TPDOs ----

    fn zeroed_od() -> PdoTestOd {
        PdoTestOd {
            input1: 0,
            input2: 0,
            input_flag1: false,
            input_flag2: false,
            output1: 0,
            output_flag1: false,
            output_flag2: false,
        }
    }

    /// One RPDO on 0x201 mapping 0x6200:1 (u8), with the given transmission
    /// type and deadline.
    fn rpdo_on_output1(transmission_type: u8, deadline_ms: u16) -> RpdoEngine<1> {
        let mut mappings = Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                index: 0x6200,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();
        RpdoEngine::new([RpdoConfig {
            od_number: 0,
            cob_id: 0x201,
            transmission_type,
            deadline_ms,
            mappings,
            enabled: true,
        }])
    }

    #[test]
    fn rpdo_synchronous_defers_od_write_until_sync() {
        let mut engine = rpdo_on_output1(SYNC_CYCLIC_1, 0);
        let mut od = zeroed_od();
        let mut events: Deque<OdEvent, 16> = Deque::new();

        // The frame is consumed, but nothing reaches the OD yet — and no event
        // fires, because visibility follows commit time, not arrival time.
        let frame = CanFrame::new(0x201, &[0xFF]).unwrap();
        assert!(engine.process(&frame, &mut od, &mut events, 0));
        assert_eq!(od.output1, 0);
        assert!(events.is_empty());
        assert!(engine.has_pending(PdoNumber::of::<1>()));

        assert_eq!(engine.on_sync(&mut od, &mut events), 0);
        assert_eq!(od.output1, 0xFF);
        assert_eq!(events.pop_front().unwrap().index, 0x6200);
        assert!(!engine.has_pending(PdoNumber::of::<1>()));

        // A second SYNC with nothing buffered is a no-op.
        od.output1 = 0x11;
        engine.on_sync(&mut od, &mut events);
        assert_eq!(od.output1, 0x11);
        assert!(events.is_empty());
    }

    #[test]
    fn rpdo_synchronous_keeps_only_the_latest_frame() {
        let mut engine = rpdo_on_output1(SYNC_ACYCLIC, 0);
        let mut od = zeroed_od();
        let mut events: Deque<OdEvent, 16> = Deque::new();

        for value in [0x11u8, 0x22, 0x33] {
            let frame = CanFrame::new(0x201, &[value]).unwrap();
            assert!(engine.process(&frame, &mut od, &mut events, 0));
        }
        engine.on_sync(&mut od, &mut events);

        assert_eq!(od.output1, 0x33);
        // One commit, so one event — not one per frame received.
        assert_eq!(events.len(), 1);
    }

    #[test]
    fn malformed_synchronous_rpdo_does_not_replace_pending_or_refresh_deadline() {
        let mut engine = two_entry_rpdo(SYNC_CYCLIC_1, 100);
        let mut od = zeroed_od();
        let mut events: Deque<OdEvent, 16> = Deque::new();

        let valid = CanFrame::new(0x201, &[0x11, 1]).unwrap();
        assert!(engine.process(&valid, &mut od, &mut events, 0));
        let short = CanFrame::new(0x201, &[0x22]).unwrap();
        assert!(engine.process(&short, &mut od, &mut events, 50_000));

        assert!(engine.has_pending(PdoNumber::of::<1>()));
        engine.on_sync(&mut od, &mut events);
        assert_eq!(od.output1, 0x11);
        assert!(od.output_flag1);
        assert_eq!(events.len(), 2);

        let mut expired = Vec::<PdoNumber, 1>::new();
        engine.check_deadlines(100_001, &mut expired);
        assert_eq!(expired.as_slice(), &[PdoNumber::of::<1>()]);
    }

    #[test]
    fn rpdo_event_driven_still_commits_immediately() {
        let mut engine = rpdo_on_output1(EVENT_DRIVEN, 0);
        let mut od = zeroed_od();
        let mut events: Deque<OdEvent, 16> = Deque::new();

        let frame = CanFrame::new(0x201, &[0xAB]).unwrap();
        assert!(engine.process(&frame, &mut od, &mut events, 0));
        assert_eq!(od.output1, 0xAB);
        assert!(!engine.has_pending(PdoNumber::of::<1>()));
        assert_eq!(events.len(), 1);
    }

    #[test]
    fn rpdo_discard_pending_drops_uncommitted_data() {
        let mut engine = rpdo_on_output1(SYNC_CYCLIC_1, 0);
        let mut od = zeroed_od();
        let mut events: Deque<OdEvent, 16> = Deque::new();

        let frame = CanFrame::new(0x201, &[0xFF]).unwrap();
        engine.process(&frame, &mut od, &mut events, 0);
        engine.discard_pending();

        engine.on_sync(&mut od, &mut events);
        assert_eq!(od.output1, 0);
        assert!(events.is_empty());
    }

    #[test]
    fn rpdo_deadline_is_stamped_at_reception_not_at_sync() {
        // Deadline monitoring watches the bus: a synchronous RPDO whose frames
        // keep arriving is alive even if no SYNC ever commits them.
        let mut engine = rpdo_on_output1(SYNC_CYCLIC_1, 100);
        let mut od = zeroed_od();
        let mut events: Deque<OdEvent, 16> = Deque::new();

        let frame = CanFrame::new(0x201, &[0x01]).unwrap();
        engine.process(&frame, &mut od, &mut events, 1_000_000);

        let mut expired = Vec::<PdoNumber, 1>::new();
        engine.check_deadlines(1_050_000, &mut expired);
        assert!(expired.is_empty());
        assert!(!engine.deadline_expired(PdoNumber::of::<1>()));

        engine.check_deadlines(1_200_000, &mut expired);
        assert_eq!(expired.as_slice(), &[PdoNumber::of::<1>()]);
    }

    #[test]
    fn tpdo_sync_acyclic_fires_only_after_an_application_event() {
        let mut mappings = Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                index: 0x6000,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();
        let mut engine = TpdoEngine::new([TpdoConfig {
            od_number: 0,
            cob_id: 0x181,
            transmission_type: SYNC_ACYCLIC,
            inhibit_time_100us: 0,
            event_timer_ms: 0,
            mappings,
            enabled: true,
        }]);
        let mut od = zeroed_od();
        od.input1 = 0x42;

        // Nothing changed since the last pulse: type 0 is acyclic, so silence.
        let mut out = Vec::<CanFrame, 1>::new();
        engine.latch_sync_events(&no_dirty());
        engine.on_sync(&od, &mut out);
        assert!(out.is_empty());

        // A change to a *different* entry is not this PDO's trigger.
        engine.latch_sync_events(&dirty(0x6000, 2));
        engine.on_sync(&od, &mut out);
        assert!(out.is_empty());

        engine.latch_sync_events(&dirty(0x6000, 1));
        engine.on_sync(&od, &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].raw_id(), 0x181);
        assert_eq!(out[0].data(), &[0x42]);

        // One transmission per event: the next pulse is silent again.
        out.clear();
        engine.latch_sync_events(&no_dirty());
        engine.on_sync(&od, &mut out);
        assert!(out.is_empty(), "type 0 repeated without a new event");
    }

    #[test]
    fn tpdo_sync_acyclic_trigger_survives_cycles_without_a_sync() {
        // The regression the latch exists for: the caller's dirty set is
        // cleared every cycle, so an event several cycles before the pulse
        // must still be transmitted on it.
        let mut mappings = Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                index: 0x6000,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();
        let mut engine = TpdoEngine::new([TpdoConfig {
            od_number: 0,
            cob_id: 0x181,
            transmission_type: SYNC_ACYCLIC,
            inhibit_time_100us: 0,
            event_timer_ms: 0,
            mappings,
            enabled: true,
        }]);
        let mut od = zeroed_od();
        od.input1 = 0x42;

        // Cycle 1: the application changes a mapped entry, no SYNC arrives.
        engine.latch_sync_events(&dirty(0x6000, 1));

        // Cycles 2 and 3: nothing dirty any more, still no SYNC.
        let mut out = Vec::<CanFrame, 1>::new();
        engine.latch_sync_events(&no_dirty());
        engine.latch_sync_events(&no_dirty());
        assert!(out.is_empty());

        // Cycle 4: the pulse finally arrives and must carry the old event.
        engine.latch_sync_events(&no_dirty());
        engine.on_sync(&od, &mut out);
        assert_eq!(out.len(), 1, "trigger lost between pulses");
        assert_eq!(out[0].data(), &[0x42]);
    }

    #[test]
    fn tpdo_discard_sync_events_drops_latched_triggers() {
        let mut mappings = Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                index: 0x6000,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();
        let mut engine = TpdoEngine::new([TpdoConfig {
            od_number: 0,
            cob_id: 0x181,
            transmission_type: SYNC_ACYCLIC,
            inhibit_time_100us: 0,
            event_timer_ms: 0,
            mappings,
            enabled: true,
        }]);
        let od = zeroed_od();

        engine.latch_sync_events(&dirty(0x6000, 1));
        engine.discard_sync_events();

        let mut out = Vec::<CanFrame, 1>::new();
        engine.on_sync(&od, &mut out);
        assert!(out.is_empty());
    }

    #[test]
    fn tpdo_reset_communication_state_restarts_the_cyclic_divisor() {
        let mut mappings = Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                index: 0x6000,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();
        let mut engine = TpdoEngine::new([TpdoConfig {
            od_number: 0,
            cob_id: 0x181,
            transmission_type: 2,
            inhibit_time_100us: 0,
            event_timer_ms: 0,
            mappings,
            enabled: true,
        }]);
        let od = zeroed_od();
        let mut out = Vec::<CanFrame, 1>::new();

        // One pulse puts the divisor halfway to its transmission.
        engine.on_sync(&od, &mut out);
        assert!(out.is_empty());

        engine.reset_communication_state();

        // Reset Communication starts the divisor over; the first post-reset
        // pulse must not complete the pre-reset count.
        engine.on_sync(&od, &mut out);
        assert!(out.is_empty());
        engine.on_sync(&od, &mut out);
        assert_eq!(out.len(), 1);
    }

    #[test]
    fn tpdo_reset_communication_state_drops_an_inhibited_async_event() {
        let mut mappings = Vec::<PdoMapping, PDO_MAX_MAPPINGS>::new();
        mappings
            .push(PdoMapping {
                index: 0x6000,
                subindex: 1,
                bit_length: 8,
            })
            .unwrap();
        let mut engine = TpdoEngine::new([TpdoConfig {
            od_number: 0,
            cob_id: 0x181,
            transmission_type: 255,
            inhibit_time_100us: 500, // 50 ms
            event_timer_ms: 0,
            mappings,
            enabled: true,
        }]);
        let od = zeroed_od();
        let mut out = Vec::<CanFrame, 1>::new();
        let no_dirty = Vec::<(u16, u8), 1>::new();

        // First change transmits and starts the inhibit window.
        engine.notify_changed(0x6000, 1);
        engine.poll(&od, 100_000, &no_dirty, &mut out);
        assert_eq!(out.len(), 1);
        out.clear();

        // Second change lands inside the window and stays latched.
        engine.notify_changed(0x6000, 1);
        engine.poll(&od, 110_000, &no_dirty, &mut out);
        assert!(out.is_empty());

        engine.reset_communication_state();

        // The latched event belonged to the pre-reset session. Well past the
        // inhibit time it must not be transmitted.
        engine.poll(&od, 400_000, &no_dirty, &mut out);
        assert!(
            out.is_empty(),
            "pre-reset asynchronous event transmitted after reset: {out:?}"
        );
    }
}
