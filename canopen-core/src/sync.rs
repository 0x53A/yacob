use crate::transport::CanFrame;

/// SYNC consumer. Detects incoming SYNC frames.
pub struct SyncConsumer {
    counter: u8,
}

impl SyncConsumer {
    pub const fn new() -> Self {
        Self { counter: 0 }
    }

    /// Process a SYNC frame. Returns the SYNC counter value if present.
    /// SYNC frames have COB-ID 0x080, 0 or 1 data byte.
    pub fn process(&mut self, frame: &CanFrame) -> Option<u8> {
        if frame.id() != 0x080 {
            return None;
        }
        let counter = if frame.dlc() >= 1 {
            frame.data()[0]
        } else {
            self.counter = self.counter.wrapping_add(1);
            self.counter
        };
        Some(counter)
    }
}

/// SYNC producer. Generates SYNC frames at a configurable interval.
pub struct SyncProducer {
    interval_us: u64,
    last_sent_us: u64,
    counter: u8,
    use_counter: bool,
}

impl SyncProducer {
    pub const fn new(interval_us: u64, use_counter: bool) -> Self {
        Self {
            interval_us,
            last_sent_us: 0,
            counter: 0,
            use_counter,
        }
    }

    pub fn poll(&mut self, now_us: u64) -> Option<CanFrame> {
        if self.interval_us == 0 {
            return None;
        }
        if now_us.wrapping_sub(self.last_sent_us) >= self.interval_us {
            self.last_sent_us = now_us;
            if self.use_counter {
                self.counter = self.counter.wrapping_add(1);
                Some(CanFrame::new(0x080, &[self.counter]).unwrap())
            } else {
                Some(CanFrame::new(0x080, &[]).unwrap())
            }
        } else {
            None
        }
    }
}
