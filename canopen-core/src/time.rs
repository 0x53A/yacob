/// Minimal time source. User provides an implementation.
///
/// On embedded (Embassy): wraps `embassy_time::Instant`.
/// On Linux: wraps `std::time::Instant`.
pub trait Clock {
    /// Current time in microseconds since some arbitrary epoch.
    fn now_us(&self) -> u64;
}
