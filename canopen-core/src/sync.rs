//! SYNC object (CiA 301 §7.2.5): the pulse that coordinates synchronous PDOs.
//!
//! [`Node`](crate::node::Node) owns a [`SyncConsumer`] and a [`SyncProducer`]
//! and configures both from the OD — 0x1005 (COB-ID SYNC message), 0x1006
//! (communication cycle period), 0x1007 (synchronous window length) and
//! 0x1019 (synchronous counter overflow value). A node whose OD declares none
//! of them keeps the CiA 301 defaults: consume SYNC on 0x080, produce none,
//! and do not limit the synchronous-PDO window.

use crate::transport::CanFrame;

/// Pre-defined SYNC COB-ID (CiA 301 connection set), the default when the OD
/// declares no 0x1005.
pub const SYNC_COB_ID_DEFAULT: u16 = 0x080;

/// OD 0x1005 bit 30 — the device generates SYNC.
pub const SYNC_GEN: u32 = 1 << 30;

/// OD 0x1005 bit 29 — 29-bit (extended) SYNC frames. This stack is 11-bit
/// only, so an entry with this bit set describes no usable SYNC object.
pub const SYNC_FRAME_29BIT: u32 = 1 << 29;

/// Reserved bits of OD 0x1005: bit 31, and bits 11..=28 above an 11-bit
/// COB-ID. Only bits 29 (frame type) and 30 (generator) carry meaning.
pub const SYNC_COB_ID_RESERVED: u32 = 0x8000_0000 | 0x1FFF_F800;

/// Largest CiA 301 synchronous counter overflow value (OD 0x1019). Valid
/// values are 0 (SYNC carries no counter) or 2..=240; 1 is not permitted.
pub const SYNC_COUNTER_OVERFLOW_MAX: u8 = 240;

/// Is `value` a legal OD 0x1019 synchronous counter overflow value?
pub const fn is_valid_sync_counter_overflow(value: u8) -> bool {
    value == 0 || (value >= 2 && value <= SYNC_COUNTER_OVERFLOW_MAX)
}

/// Is `raw` an OD 0x1005 value this stack can use?
///
/// COB-ID 0 is NMT, bit 29 requests an unsupported extended SYNC frame, and
/// the remaining high bits are reserved by CiA 301. Keeping this check in one
/// place makes initial OD validation and SDO download validation agree.
pub const fn is_supported_sync_cob_id(raw: u32) -> bool {
    raw & (SYNC_FRAME_29BIT | SYNC_COB_ID_RESERVED) == 0 && raw & 0x7FF != 0
}

/// Decoded OD 0x1005 (COB-ID SYNC message).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SyncCobId {
    /// 11-bit COB-ID the SYNC object lives on.
    pub cob_id: u16,
    /// Bit 30: this device generates SYNC (it is the SYNC master).
    pub produce: bool,
}

impl SyncCobId {
    /// CiA 301 default: consume SYNC on 0x080, produce none.
    pub const DEFAULT: Self = Self {
        cob_id: SYNC_COB_ID_DEFAULT,
        produce: false,
    };

    /// Decode an OD 0x1005 value.
    ///
    /// `None` means the entry describes no usable SYNC object, and the node
    /// then neither consumes nor produces SYNC. Three cases reach it: a COB-ID
    /// of 0 (which is the NMT COB-ID — never a SYNC object, and consuming it
    /// would swallow NMT commands), the 29-bit frame bit, which this stack
    /// does not implement, and any reserved bit being set. Note that 0x1005
    /// has no valid/invalid bit: unlike a PDO COB-ID, bit 31 is reserved.
    ///
    /// Reserved bits are rejected rather than masked away so a malformed OD
    /// default fails the same way the equivalent SDO download does, instead
    /// of quietly resolving to a different SYNC object than was written.
    pub const fn from_raw(raw: u32) -> Option<Self> {
        if !is_supported_sync_cob_id(raw) {
            return None;
        }
        Some(Self {
            cob_id: (raw & 0x7FF) as u16,
            produce: raw & SYNC_GEN != 0,
        })
    }

    /// Encode back to an OD 0x1005 value.
    pub const fn to_raw(self) -> u32 {
        self.cob_id as u32 | if self.produce { SYNC_GEN } else { 0 }
    }
}

impl Default for SyncCobId {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// SYNC consumer: recognises SYNC frames on the configured COB-ID.
pub struct SyncConsumer {
    /// 0 = disabled (no usable SYNC object configured).
    cob_id: u16,
    counter: u8,
}

impl SyncConsumer {
    /// A consumer listening on `cob_id`. Pass 0 to start disabled.
    pub const fn new(cob_id: u16) -> Self {
        Self { cob_id, counter: 0 }
    }

    /// A consumer on the pre-defined COB-ID 0x080.
    pub const fn default_cob() -> Self {
        Self::new(SYNC_COB_ID_DEFAULT)
    }

    /// The COB-ID SYNC is consumed on, or `None` when no usable SYNC object
    /// is configured.
    pub const fn cob_id(&self) -> Option<u16> {
        if self.cob_id == 0 {
            None
        } else {
            Some(self.cob_id)
        }
    }

    /// Point the consumer at a different COB-ID (0 disables it). The synthetic
    /// counter restarts only when the COB-ID actually changes, so re-reading
    /// an unchanged 0x1005 does not disturb a running cycle.
    pub fn set_cob_id(&mut self, cob_id: u16) {
        if self.cob_id != cob_id {
            self.cob_id = cob_id;
            self.counter = 0;
        }
    }

    /// Forget the synthetic pulse count without changing the COB-ID.
    ///
    /// For NMT reset, which re-initialises communication objects: the count
    /// restarts with the new cycle. Ordinary reconfiguration must not do this
    /// (see [`set_cob_id`](Self::set_cob_id)).
    pub fn reset(&mut self) {
        self.counter = 0;
    }

    /// Process a frame. Returns the SYNC counter value when the frame is a
    /// SYNC on the configured COB-ID.
    ///
    /// A SYNC carries either no data or a single counter byte (CiA 301
    /// §7.2.5.2.1); anything longer is malformed and rejected. Without a
    /// counter byte the returned value is synthetic — this consumer's own
    /// wrapping `u8` count of pulses seen. It wraps through zero after 255;
    /// callers must not treat it as a never-decreasing timestamp.
    pub fn process(&mut self, frame: &CanFrame) -> Option<u8> {
        if self.cob_id == 0 || frame.raw_id() != self.cob_id || frame.raw_dlc() > 1 {
            return None;
        }
        if frame.raw_dlc() == 1 {
            let counter = frame.data()[0];
            self.counter = counter;
            Some(counter)
        } else {
            self.counter = self.counter.wrapping_add(1);
            Some(self.counter)
        }
    }
}

impl Default for SyncConsumer {
    fn default() -> Self {
        Self::default_cob()
    }
}

/// Runtime state for OD 0x1007, the synchronous PDO transmission window.
///
/// A nonzero window permits synchronous TPDO transmission only from the SYNC
/// pulse through `length_us` microseconds after it. Zero disables the limit.
/// The window is opened by each accepted/received SYNC and is inactive before
/// the first pulse and after a communication reset or leaving Operational.
pub(crate) struct SyncWindow {
    length_us: u64,
    opened_at_us: u64,
    active: bool,
}

impl SyncWindow {
    pub const fn new() -> Self {
        Self {
            length_us: 0,
            opened_at_us: 0,
            active: false,
        }
    }

    /// Apply the OD value without shifting an already-open window.
    pub fn configure(&mut self, length_us: u64) {
        self.length_us = length_us;
    }

    pub const fn length_us(&self) -> u64 {
        self.length_us
    }

    pub fn open(&mut self, now_us: u64) {
        self.opened_at_us = now_us;
        self.active = true;
    }

    pub fn reset(&mut self) {
        self.opened_at_us = 0;
        self.active = false;
    }

    /// Whether a synchronous TPDO may still be offered to the transport.
    ///
    /// The boundary itself is inside the window; it is passed only when
    /// `elapsed > length`. Wrapping subtraction supports wrapping clocks.
    pub fn allows_transmit(&self, now_us: u64) -> bool {
        self.active
            && (self.length_us == 0 || now_us.wrapping_sub(self.opened_at_us) <= self.length_us)
    }
}

impl Default for SyncWindow {
    fn default() -> Self {
        Self::new()
    }
}

/// SYNC producer: generates SYNC frames at the OD 0x1006 communication cycle
/// period. Inert until configured with a nonzero COB-ID *and* period.
pub struct SyncProducer {
    /// 0 = disabled (this node is not the SYNC master).
    cob_id: u16,
    /// Communication cycle period in µs. 0 = disabled.
    interval_us: u64,
    /// OD 0x1019: 0 = SYNC carries no data, 2..=240 = counter byte wrapping
    /// at this value.
    overflow: u8,
    last_sent_us: u64,
    /// False until the first [`poll`](Self::poll) establishes a time base.
    armed: bool,
    counter: u8,
}

impl SyncProducer {
    /// A disabled producer.
    pub const fn new() -> Self {
        Self {
            cob_id: 0,
            interval_us: 0,
            overflow: 0,
            last_sent_us: 0,
            armed: false,
            counter: 0,
        }
    }

    /// Apply a configuration. `cob_id` or `interval_us` of 0 disables
    /// production.
    ///
    /// The cycle phase and counter are only reset when something actually
    /// changed, so re-reading unchanged OD entries (after an unrelated SDO
    /// write, say) does not shift the SYNC phase.
    pub fn configure(&mut self, cob_id: u16, interval_us: u64, overflow: u8) {
        if self.cob_id == cob_id && self.interval_us == interval_us && self.overflow == overflow {
            return;
        }
        self.cob_id = cob_id;
        self.interval_us = interval_us;
        self.overflow = overflow;
        self.armed = false;
        self.counter = 0;
    }

    /// True when this node is configured to produce SYNC.
    pub const fn is_enabled(&self) -> bool {
        self.cob_id != 0 && self.interval_us != 0
    }

    /// Communication cycle period in µs (0 = not producing).
    pub const fn interval_us(&self) -> u64 {
        if self.is_enabled() {
            self.interval_us
        } else {
            0
        }
    }

    /// Forget the cycle phase and counter without changing the configuration.
    ///
    /// [`configure`](Self::configure) deliberately keeps them when the OD
    /// values are unchanged, so an unrelated SDO write cannot shift the SYNC
    /// phase. A reset is the opposite intent: communication objects are
    /// re-initialised, so the first pulse afterwards starts a fresh cycle at
    /// counter 1, one full period after the reset.
    pub fn reset(&mut self) {
        self.armed = false;
        self.last_sent_us = 0;
        self.counter = 0;
    }

    /// The SYNC frame due at `now_us`, if any.
    ///
    /// **The cycle does not advance until [`confirm_sent`](Self::confirm_sent)
    /// acknowledges the frame reached the transport.** A pulse the transport
    /// refused must not advance the phase or the counter: the bus would never
    /// see it, while this node would have moved on — the two would disagree
    /// about which pulse is which. Leaving the phase alone means the same
    /// frame is offered again on the next call, so a transient full TX queue
    /// costs a late pulse rather than a lost one.
    ///
    /// The first call only establishes the time base — the first SYNC goes out
    /// one full period later. Without that, a clock whose epoch is far from
    /// zero (a wall clock rather than time-since-boot) would fire immediately
    /// and then again after the period, putting a short cycle at the start.
    pub fn poll(&mut self, now_us: u64) -> Option<CanFrame> {
        if !self.is_enabled() {
            return None;
        }
        if !self.armed {
            self.armed = true;
            self.last_sent_us = now_us;
            return None;
        }
        if now_us.wrapping_sub(self.last_sent_us) < self.interval_us {
            return None;
        }
        if self.overflow >= 2 {
            CanFrame::new(self.cob_id, &[self.peek_counter()])
        } else {
            CanFrame::new(self.cob_id, &[])
        }
    }

    /// Commit the pulse [`poll`](Self::poll) handed out: advance the cycle
    /// phase and the counter. Call only once the transport accepted the frame.
    pub fn confirm_sent(&mut self, now_us: u64) {
        self.last_sent_us = now_us;
        if self.overflow >= 2 {
            self.counter = self.peek_counter();
        }
    }

    /// The counter value the next pulse will carry. CiA 301: it runs
    /// 1..=overflow, then wraps to 1.
    const fn peek_counter(&self) -> u8 {
        if self.counter >= self.overflow {
            1
        } else {
            self.counter + 1
        }
    }
}

impl Default for SyncProducer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cob_id_decode() {
        assert_eq!(SyncCobId::from_raw(0x80), Some(SyncCobId::DEFAULT));
        assert_eq!(
            SyncCobId::from_raw(SYNC_GEN | 0x80),
            Some(SyncCobId {
                cob_id: 0x80,
                produce: true
            })
        );
        // Relocated SYNC.
        assert_eq!(
            SyncCobId::from_raw(0x123),
            Some(SyncCobId {
                cob_id: 0x123,
                produce: false
            })
        );
        // No usable SYNC object: COB-ID 0 is NMT, 29-bit is unsupported, and
        // reserved bits must not be silently masked away.
        assert_eq!(SyncCobId::from_raw(0), None);
        assert_eq!(SyncCobId::from_raw(SYNC_GEN), None);
        assert_eq!(SyncCobId::from_raw(SYNC_FRAME_29BIT | 0x80), None);
        assert_eq!(SyncCobId::from_raw(0x8000_0080), None, "bit 31 reserved");
        assert_eq!(SyncCobId::from_raw(0x0000_0880), None, "bit 11 reserved");
        assert_eq!(SyncCobId::from_raw(0x1000_0080), None, "bit 28 reserved");
    }

    #[test]
    fn counter_overflow_range() {
        assert!(is_valid_sync_counter_overflow(0));
        assert!(!is_valid_sync_counter_overflow(1));
        assert!(is_valid_sync_counter_overflow(2));
        assert!(is_valid_sync_counter_overflow(240));
        assert!(!is_valid_sync_counter_overflow(241));
        assert!(!is_valid_sync_counter_overflow(255));
    }

    #[test]
    fn cob_id_roundtrip() {
        for raw in [0x80u32, 0x123, SYNC_GEN | 0x80, SYNC_GEN | 0x7FF] {
            assert_eq!(SyncCobId::from_raw(raw).unwrap().to_raw(), raw);
        }
    }

    #[test]
    fn consumer_counts_dataless_syncs() {
        let mut c = SyncConsumer::default_cob();
        let sync = CanFrame::new(0x080, &[]).unwrap();
        assert_eq!(c.process(&sync), Some(1));
        assert_eq!(c.process(&sync), Some(2));
    }

    #[test]
    fn consumer_takes_counter_byte_when_present() {
        let mut c = SyncConsumer::default_cob();
        assert_eq!(c.process(&CanFrame::new(0x080, &[7]).unwrap()), Some(7));
        // The synthetic counter continues from the received value.
        assert_eq!(c.process(&CanFrame::new(0x080, &[]).unwrap()), Some(8));
    }

    #[test]
    fn consumer_ignores_other_cob_ids_and_malformed_syncs() {
        let mut c = SyncConsumer::default_cob();
        assert_eq!(c.process(&CanFrame::new(0x081, &[]).unwrap()), None);
        // A SYNC carries 0 or 1 bytes; longer is not a SYNC.
        assert_eq!(c.process(&CanFrame::new(0x080, &[1, 2]).unwrap()), None);
    }

    #[test]
    fn consumer_follows_relocated_cob_id() {
        let mut c = SyncConsumer::default_cob();
        c.set_cob_id(0x123);
        assert_eq!(c.cob_id(), Some(0x123));
        assert_eq!(c.process(&CanFrame::new(0x080, &[]).unwrap()), None);
        assert_eq!(c.process(&CanFrame::new(0x123, &[]).unwrap()), Some(1));

        c.set_cob_id(0);
        assert_eq!(c.cob_id(), None);
        assert_eq!(c.process(&CanFrame::new(0x123, &[]).unwrap()), None);
    }

    #[test]
    fn producer_disabled_by_default() {
        let mut p = SyncProducer::new();
        assert!(!p.is_enabled());
        assert!(p.poll(0).is_none());
        assert!(p.poll(10_000_000).is_none());
    }

    #[test]
    fn producer_arms_before_first_sync() {
        let mut p = SyncProducer::new();
        p.configure(0x080, 1000, 0);
        // Epoch far from zero: the first poll must not fire immediately.
        assert!(p.poll(5_000_000).is_none());
        assert!(p.poll(5_000_999).is_none());
        let frame = p.poll(5_001_000).expect("SYNC after one period");
        assert_eq!(frame.raw_id(), 0x080);
        assert_eq!(frame.raw_dlc(), 0);
    }

    /// Poll and, if a frame is due, confirm it — the normal path where the
    /// transport accepted the pulse.
    fn poll_sent(p: &mut SyncProducer, now_us: u64) -> Option<CanFrame> {
        let frame = p.poll(now_us)?;
        p.confirm_sent(now_us);
        Some(frame)
    }

    #[test]
    fn producer_counter_wraps_at_overflow() {
        let mut p = SyncProducer::new();
        p.configure(0x080, 1000, 3);
        poll_sent(&mut p, 0);
        let mut seen = [0u8; 4];
        for (i, slot) in seen.iter_mut().enumerate() {
            let frame = poll_sent(&mut p, 1000 * (i as u64 + 1)).unwrap();
            assert_eq!(frame.raw_dlc(), 1);
            *slot = frame.data()[0];
        }
        assert_eq!(seen, [1, 2, 3, 1]);
    }

    #[test]
    fn producer_keeps_phase_across_identical_reconfigure() {
        let mut p = SyncProducer::new();
        p.configure(0x080, 1000, 0);
        poll_sent(&mut p, 0);
        assert!(poll_sent(&mut p, 1000).is_some());
        // Same values again: must not re-arm and swallow the next pulse.
        p.configure(0x080, 1000, 0);
        assert!(poll_sent(&mut p, 2000).is_some());
        // A real change re-arms.
        p.configure(0x080, 2000, 0);
        assert!(poll_sent(&mut p, 3000).is_none());
    }

    #[test]
    fn unconfirmed_pulse_does_not_advance_the_cycle() {
        let mut p = SyncProducer::new();
        p.configure(0x080, 1000, 3);
        poll_sent(&mut p, 0);

        // Transport refused: no confirm. The same pulse must be offered again,
        // carrying the same counter — the bus never saw the first attempt.
        let refused = p.poll(1000).expect("pulse due");
        assert_eq!(refused.data(), &[1]);
        let retried = p.poll(1500).expect("refused pulse retried");
        assert_eq!(retried.data(), &[1], "counter advanced without a send");

        p.confirm_sent(1500);
        assert!(
            p.poll(2000).is_none(),
            "phase restarts from the accepted send"
        );
        let next = poll_sent(&mut p, 2500).expect("next pulse");
        assert_eq!(next.data(), &[2]);
    }

    #[test]
    fn reset_restarts_phase_and_counter() {
        let mut p = SyncProducer::new();
        p.configure(0x080, 1000, 3);
        poll_sent(&mut p, 0);
        assert_eq!(poll_sent(&mut p, 1000).unwrap().data(), &[1]);
        assert_eq!(poll_sent(&mut p, 2000).unwrap().data(), &[2]);

        p.reset();
        // Re-arms: the first pulse is one full period after the reset, not
        // immediately, and the counter starts over.
        assert!(p.poll(9_000).is_none(), "fired without re-arming");
        assert_eq!(poll_sent(&mut p, 10_000).unwrap().data(), &[1]);
    }

    #[test]
    fn consumer_reset_restarts_synthetic_count() {
        let mut c = SyncConsumer::default_cob();
        let sync = CanFrame::new(0x080, &[]).unwrap();
        assert_eq!(c.process(&sync), Some(1));
        assert_eq!(c.process(&sync), Some(2));
        c.reset();
        assert_eq!(c.process(&sync), Some(1));
    }

    #[test]
    fn synchronous_window_zero_is_unlimited_after_a_pulse() {
        let mut window = SyncWindow::new();
        assert!(
            !window.allows_transmit(0),
            "no window before the first SYNC"
        );
        window.open(1_000);
        assert!(window.allows_transmit(1_000));
        assert!(window.allows_transmit(u64::MAX));
    }

    #[test]
    fn synchronous_window_includes_its_boundary_and_then_closes() {
        let mut window = SyncWindow::new();
        window.configure(500);
        window.open(1_000);
        assert!(window.allows_transmit(1_500));
        assert!(!window.allows_transmit(1_501));

        window.open(2_000);
        assert!(window.allows_transmit(2_001), "the next SYNC reopens it");
        window.reset();
        assert!(!window.allows_transmit(2_001));
    }
}
