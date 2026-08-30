use std::time::{Duration, Instant};

use canopen_core::transport::CanFrame;

/// Failure while turning a received PDO into a middleware message.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PdoToTopicError<Decode, Map> {
    /// The CAN frame matched this PDO, but its wire payload was invalid.
    Decode(Decode),
    /// The PDO decoded correctly, but application-level conversion rejected it.
    Map(Map),
}

/// A stateless, exact mapping from one received PDO to one topic message.
///
/// `on_frame` has deliberately unbuffered semantics:
///
/// - an unrelated frame returns `Ok(None)`;
/// - a matching valid frame returns exactly one owned message;
/// - malformed and application-invalid payloads are distinct errors;
/// - the mapper retains no previous message and never replays one.
///
/// The decode closure is a natural fit for an EDS-generated `decode_tpdoN`
/// method. Keeping it as a closure also supports dynamically configured PDOs.
pub struct PdoToTopic<Decode, Map> {
    decode: Decode,
    map: Map,
}

impl<Decode, Map> PdoToTopic<Decode, Map> {
    pub const fn new(decode: Decode, map: Map) -> Self {
        Self { decode, map }
    }

    pub fn on_frame<Pdo, Message, DecodeError, MapError>(
        &self,
        frame: &CanFrame,
    ) -> Result<Option<Message>, PdoToTopicError<DecodeError, MapError>>
    where
        Decode: Fn(&CanFrame) -> Result<Option<Pdo>, DecodeError>,
        Map: Fn(Pdo) -> Result<Message, MapError>,
    {
        let Some(pdo) = (self.decode)(frame).map_err(PdoToTopicError::Decode)? else {
            return Ok(None);
        };
        (self.map)(pdo).map(Some).map_err(PdoToTopicError::Map)
    }
}

/// Freshness of a last-observed value at a particular instant.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FreshState {
    Missing,
    Fresh { age: Duration },
    Stale { age: Duration },
}

/// Latest observation plus its receive timestamp.
///
/// This is intended for gateway status and snapshots, not for PDO-to-topic
/// delivery. [`get_fresh`](Self::get_fresh) refuses to return stale data; a
/// caller must use [`last`](Self::last) to opt into stale-value access.
#[derive(Clone, Debug)]
pub struct FreshValue<T> {
    last: Option<(Instant, T)>,
}

impl<T> Default for FreshValue<T> {
    fn default() -> Self {
        Self { last: None }
    }
}

impl<T> FreshValue<T> {
    pub const fn new() -> Self {
        Self { last: None }
    }

    pub fn observe(&mut self, value: T, received_at: Instant) -> Option<T> {
        self.last
            .replace((received_at, value))
            .map(|(_, previous)| previous)
    }

    pub fn state(&self, now: Instant, max_age: Duration) -> FreshState {
        let Some((received_at, _)) = &self.last else {
            return FreshState::Missing;
        };
        let age = now.saturating_duration_since(*received_at);
        if age < max_age {
            FreshState::Fresh { age }
        } else {
            FreshState::Stale { age }
        }
    }

    /// Returns a value only while `age < max_age`. Equality is stale, which
    /// removes ambiguity at watchdog boundaries.
    pub fn get_fresh(&self, now: Instant, max_age: Duration) -> Option<&T> {
        match self.state(now, max_age) {
            FreshState::Fresh { .. } => self.last.as_ref().map(|(_, value)| value),
            FreshState::Missing | FreshState::Stale { .. } => None,
        }
    }

    /// Explicit escape hatch for diagnostics that intentionally show stale
    /// values alongside a freshness flag.
    pub fn last(&self) -> Option<(Instant, &T)> {
        self.last.as_ref().map(|(at, value)| (*at, value))
    }

    pub fn clear(&mut self) -> Option<T> {
        self.last.take().map(|(_, value)| value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pdo_mapping_is_unbuffered_and_distinguishes_failures() {
        let mapping = PdoToTopic::new(
            |frame: &CanFrame| match frame.raw_id() {
                0x181 if frame.data().len() == 2 => {
                    Ok(Some(u16::from_le_bytes(frame.data().try_into().unwrap())))
                }
                0x181 => Err("bad DLC"),
                _ => Ok(None),
            },
            |value| {
                if value <= 1000 {
                    Ok(value * 2)
                } else {
                    Err("range")
                }
            },
        );

        let unrelated = CanFrame::new(0x182, &[1, 0]).unwrap();
        assert_eq!(mapping.on_frame(&unrelated), Ok(None));

        let valid = CanFrame::new(0x181, &[21, 0]).unwrap();
        assert_eq!(mapping.on_frame(&valid), Ok(Some(42)));
        // Calling with an unrelated frame cannot replay 42.
        assert_eq!(mapping.on_frame(&unrelated), Ok(None));

        let malformed = CanFrame::new(0x181, &[1]).unwrap();
        assert_eq!(
            mapping.on_frame(&malformed),
            Err(PdoToTopicError::Decode("bad DLC"))
        );
        let invalid = CanFrame::new(0x181, &1001u16.to_le_bytes()).unwrap();
        assert_eq!(
            mapping.on_frame(&invalid),
            Err(PdoToTopicError::Map("range"))
        );
    }

    #[test]
    fn stale_values_are_not_returned_implicitly() {
        let start = Instant::now();
        let mut value = FreshValue::new();
        assert_eq!(
            value.state(start, Duration::from_millis(100)),
            FreshState::Missing
        );

        value.observe(7, start);
        assert_eq!(
            value.get_fresh(
                start + Duration::from_millis(99),
                Duration::from_millis(100)
            ),
            Some(&7)
        );
        assert_eq!(
            value.get_fresh(
                start + Duration::from_millis(100),
                Duration::from_millis(100)
            ),
            None
        );
        assert_eq!(value.last(), Some((start, &7)));
    }
}
