use std::array;
use std::time::{Duration, Instant};

use canopen_core::transport::CanFrame;

/// How long a topic sample may wait before CAN transmission.
///
/// There is intentionally no `Default`: a gateway author must consciously
/// choose whether stale commands can ever be transmitted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Validity {
    /// The sample is valid while `now - received_at < duration`.
    For(Duration),
    /// The sample never expires. Appropriate for configuration, rarely for
    /// motion or actuator commands.
    Forever,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PdoSlotError {
    pub requested: usize,
    pub slot_count: usize,
}

/// Typed slot identifier, checked once at construction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PdoSlot<const N: usize>(usize);

impl<const N: usize> PdoSlot<N> {
    pub fn new(index: usize) -> Result<Self, PdoSlotError> {
        if index < N {
            Ok(Self(index))
        } else {
            Err(PdoSlotError {
                requested: index,
                slot_count: N,
            })
        }
    }

    pub const fn index(self) -> usize {
        self.0
    }
}

#[derive(Clone, Copy, Debug)]
struct PendingPdo {
    frame: CanFrame,
    received_at: Instant,
    valid_for: Option<Duration>,
    generation: u64,
}

impl PendingPdo {
    fn expired(&self, now: Instant) -> bool {
        self.valid_for
            .is_some_and(|duration| now.saturating_duration_since(self.received_at) >= duration)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SlotStats {
    pub offered: u64,
    pub replaced: u64,
    pub expired: u64,
    pub confirmed: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OfferResult {
    pub generation: u64,
    pub replaced_pending: bool,
}

/// A frame selected for transmission.
///
/// Call [`LatestPdoSet::confirm`] only after the transport accepted this exact
/// frame. Until then `prepare_next` returns the selected slot again (possibly
/// with a newer generation if the topic replaced it).
#[derive(Clone, Copy, Debug)]
pub struct PreparedPdo<const N: usize> {
    pub slot: PdoSlot<N>,
    pub generation: u64,
    pub frame: CanFrame,
    pub received_at: Instant,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConfirmResult {
    Confirmed,
    /// The topic delivered a newer value after this frame was prepared. The
    /// newer value remains pending.
    NewerPending,
    /// This selection is not the set's current in-flight item.
    NotInFlight,
}

/// Latest-only, independently buffered PDO transmit slots.
///
/// A slot has capacity one: offering a second sample replaces the first.
/// Selection is round-robin between slots. Only one selection may be in flight
/// because a physical CAN transmitter is serial; it is retained until explicit
/// confirmation or abandonment. Expired samples are removed before selection.
pub struct LatestPdoSet<const N: usize> {
    slots: [Option<PendingPdo>; N],
    stats: [SlotStats; N],
    cursor: usize,
    in_flight: Option<usize>,
    next_generation: u64,
}

impl<const N: usize> Default for LatestPdoSet<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> LatestPdoSet<N> {
    pub fn new() -> Self {
        assert!(N > 0, "a PDO set needs at least one slot");
        Self {
            slots: array::from_fn(|_| None),
            stats: array::from_fn(|_| SlotStats::default()),
            cursor: 0,
            in_flight: None,
            next_generation: 1,
        }
    }

    pub fn offer(
        &mut self,
        slot: PdoSlot<N>,
        frame: CanFrame,
        received_at: Instant,
        validity: Validity,
    ) -> OfferResult {
        let valid_for = match validity {
            Validity::For(duration) => Some(duration),
            Validity::Forever => None,
        };
        let index = slot.index();
        let replaced_pending = self.slots[index].is_some();
        let generation = self.next_generation;
        self.next_generation = self
            .next_generation
            .checked_add(1)
            .expect("PDO generation exhausted");
        self.slots[index] = Some(PendingPdo {
            frame,
            received_at,
            valid_for,
            generation,
        });
        let stats = &mut self.stats[index];
        stats.offered = stats.offered.saturating_add(1);
        stats.replaced = stats.replaced.saturating_add(u64::from(replaced_pending));
        OfferResult {
            generation,
            replaced_pending,
        }
    }

    /// Select the next fresh frame. An unconfirmed selection is sticky; if its
    /// slot is updated before retry, the newer frame supersedes it.
    pub fn prepare_next(&mut self, now: Instant) -> Option<PreparedPdo<N>> {
        if let Some(index) = self.in_flight {
            match self.slots[index] {
                Some(pending) if !pending.expired(now) => {
                    return Some(Self::prepared(index, pending));
                }
                Some(_) => {
                    self.slots[index] = None;
                    self.stats[index].expired = self.stats[index].expired.saturating_add(1);
                    self.in_flight = None;
                }
                None => self.in_flight = None,
            }
        }

        for offset in 0..N {
            let index = (self.cursor + offset) % N;
            let Some(pending) = self.slots[index] else {
                continue;
            };
            if pending.expired(now) {
                self.slots[index] = None;
                self.stats[index].expired = self.stats[index].expired.saturating_add(1);
                continue;
            }
            self.in_flight = Some(index);
            return Some(Self::prepared(index, pending));
        }
        None
    }

    fn prepared(index: usize, pending: PendingPdo) -> PreparedPdo<N> {
        PreparedPdo {
            slot: PdoSlot(index),
            generation: pending.generation,
            frame: pending.frame,
            received_at: pending.received_at,
        }
    }

    /// Commit transport acceptance. A generation mismatch never clears a
    /// replacement sample.
    pub fn confirm(&mut self, prepared: PreparedPdo<N>) -> ConfirmResult {
        let index = prepared.slot.index();
        if self.in_flight != Some(index) {
            return ConfirmResult::NotInFlight;
        }
        self.in_flight = None;
        self.cursor = (index + 1) % N;
        match self.slots[index] {
            Some(pending) if pending.generation == prepared.generation => {
                self.slots[index] = None;
                self.stats[index].confirmed = self.stats[index].confirmed.saturating_add(1);
                ConfirmResult::Confirmed
            }
            Some(_) => ConfirmResult::NewerPending,
            None => ConfirmResult::NotInFlight,
        }
    }

    /// Release the current selection after a transport error while retaining
    /// the sample (subject to its original validity deadline).
    pub fn abandon(&mut self, prepared: PreparedPdo<N>) -> bool {
        if self.in_flight == Some(prepared.slot.index()) {
            self.in_flight = None;
            true
        } else {
            false
        }
    }

    pub fn clear(&mut self, slot: PdoSlot<N>) -> Option<CanFrame> {
        if self.in_flight == Some(slot.index()) {
            self.in_flight = None;
        }
        self.slots[slot.index()].take().map(|pending| pending.frame)
    }

    pub fn stats(&self, slot: PdoSlot<N>) -> SlotStats {
        self.stats[slot.index()]
    }

    pub fn is_empty(&self) -> bool {
        self.slots.iter().all(Option::is_none)
    }
}

/// Code-first topic-to-PDO mapping backed by one slot in a [`LatestPdoSet`].
pub struct TopicToPdo<const N: usize, Encode> {
    slot: PdoSlot<N>,
    validity: Validity,
    encode: Encode,
}

impl<const N: usize, Encode> TopicToPdo<N, Encode> {
    pub const fn new(slot: PdoSlot<N>, validity: Validity, encode: Encode) -> Self {
        Self {
            slot,
            validity,
            encode,
        }
    }

    pub fn on_message<Message, Error>(
        &self,
        set: &mut LatestPdoSet<N>,
        message: Message,
        received_at: Instant,
    ) -> Result<OfferResult, Error>
    where
        Encode: Fn(Message) -> Result<CanFrame, Error>,
    {
        let frame = (self.encode)(message)?;
        Ok(set.offer(self.slot, frame, received_at, self.validity))
    }

    pub const fn slot(&self) -> PdoSlot<N> {
        self.slot
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(value: u8) -> CanFrame {
        CanFrame::new(0x201, &[value]).unwrap()
    }

    fn slots() -> [PdoSlot<3>; 3] {
        [
            PdoSlot::new(0).unwrap(),
            PdoSlot::new(1).unwrap(),
            PdoSlot::new(2).unwrap(),
        ]
    }

    #[test]
    fn latest_sample_replaces_old_sample_per_slot() {
        let at = Instant::now();
        let [one, two, _] = slots();
        let mut set = LatestPdoSet::new();
        set.offer(one, frame(1), at, Validity::For(Duration::from_secs(1)));
        let result = set.offer(one, frame(2), at, Validity::For(Duration::from_secs(1)));
        set.offer(two, frame(3), at, Validity::For(Duration::from_secs(1)));
        assert!(result.replaced_pending);
        assert_eq!(set.prepare_next(at).unwrap().frame.data(), &[2]);
        assert_eq!(set.stats(one).replaced, 1);
    }

    #[test]
    fn samples_expire_at_the_exact_deadline() {
        let at = Instant::now();
        let [one, _, _] = slots();
        let mut set = LatestPdoSet::new();
        set.offer(one, frame(1), at, Validity::For(Duration::from_millis(100)));
        assert!(set.prepare_next(at + Duration::from_millis(99)).is_some());
        let selected = set.prepare_next(at + Duration::from_millis(99)).unwrap();
        assert!(set.abandon(selected));
        assert!(set.prepare_next(at + Duration::from_millis(100)).is_none());
        assert_eq!(set.stats(one).expired, 1);
    }

    #[test]
    fn confirmation_is_required_and_retry_is_sticky() {
        let at = Instant::now();
        let [one, two, _] = slots();
        let mut set = LatestPdoSet::new();
        set.offer(one, frame(1), at, Validity::Forever);
        set.offer(two, frame(2), at, Validity::Forever);
        let first = set.prepare_next(at).unwrap();
        assert_eq!(first.slot, one);
        let retry = set.prepare_next(at).unwrap();
        assert_eq!(retry.slot, first.slot);
        assert_eq!(retry.generation, first.generation);
        assert_eq!(retry.frame.data(), first.frame.data());
        assert_eq!(set.confirm(first), ConfirmResult::Confirmed);
        assert_eq!(set.prepare_next(at).unwrap().slot, two);
    }

    #[test]
    fn newer_sample_is_never_cleared_by_old_confirmation() {
        let at = Instant::now();
        let [one, _, _] = slots();
        let mut set = LatestPdoSet::new();
        set.offer(one, frame(1), at, Validity::Forever);
        let old = set.prepare_next(at).unwrap();
        set.offer(one, frame(2), at, Validity::Forever);
        assert_eq!(set.confirm(old), ConfirmResult::NewerPending);
        let new = set.prepare_next(at).unwrap();
        assert_eq!(new.frame.data(), &[2]);
    }

    #[test]
    fn round_robin_prevents_a_hot_low_slot_from_starving_others() {
        let at = Instant::now();
        let [one, two, _] = slots();
        let mut set = LatestPdoSet::new();
        set.offer(one, frame(1), at, Validity::Forever);
        set.offer(two, frame(2), at, Validity::Forever);
        let first = set.prepare_next(at).unwrap();
        assert_eq!(first.slot, one);
        assert_eq!(set.confirm(first), ConfirmResult::Confirmed);
        set.offer(one, frame(3), at, Validity::Forever);
        assert_eq!(set.prepare_next(at).unwrap().slot, two);
    }

    #[test]
    fn topic_mapper_only_offers_after_successful_encoding() {
        let at = Instant::now();
        let [one, _, _] = slots();
        let route = TopicToPdo::new(one, Validity::Forever, |value: u8| {
            if value == 0 {
                Err("zero")
            } else {
                Ok(frame(value))
            }
        });
        let mut set = LatestPdoSet::new();
        assert_eq!(route.on_message(&mut set, 0, at), Err("zero"));
        assert!(set.is_empty());
        route.on_message(&mut set, 9, at).unwrap();
        assert_eq!(set.prepare_next(at).unwrap().frame.data(), &[9]);
    }
}
