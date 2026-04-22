use crate::cobid::{CobId, NodeId};
use crate::nmt::NmtState;
use crate::transport::CanFrame;

/// Heartbeat producer. Generates heartbeat frames at a configurable interval.
pub struct HeartbeatProducer {
    node_id: NodeId,
    interval_us: u64,
    last_sent_us: u64,
}

impl HeartbeatProducer {
    pub const fn new(node_id: NodeId, interval_ms: u16) -> Self {
        Self {
            node_id,
            interval_us: interval_ms as u64 * 1000,
            last_sent_us: 0,
        }
    }

    /// Set the heartbeat interval. 0 disables heartbeat.
    pub fn set_interval_ms(&mut self, ms: u16) {
        self.interval_us = ms as u64 * 1000;
    }

    pub const fn interval_ms(&self) -> u16 {
        (self.interval_us / 1000) as u16
    }

    /// Check if a heartbeat is due. Returns the frame to send, if any.
    pub fn poll(&mut self, now_us: u64, state: NmtState) -> Option<CanFrame> {
        if self.interval_us == 0 {
            return None;
        }
        if now_us.wrapping_sub(self.last_sent_us) >= self.interval_us {
            self.last_sent_us = now_us;
            let cob = CobId::heartbeat(self.node_id);
            Some(CanFrame::new(cob.raw(), &[state.heartbeat_byte()]).unwrap())
        } else {
            None
        }
    }

    /// Force sending a heartbeat immediately (e.g., on boot).
    pub fn send_boot(&mut self, now_us: u64) -> CanFrame {
        self.last_sent_us = now_us;
        let cob = CobId::heartbeat(self.node_id);
        CanFrame::new(cob.raw(), &[NmtState::Initializing.heartbeat_byte()]).unwrap()
    }
}

/// Heartbeat consumer. Monitors heartbeat from a remote node.
pub struct HeartbeatConsumer {
    target_node: NodeId,
    timeout_us: u64,
    last_seen_us: u64,
    last_state: Option<NmtState>,
    timed_out: bool,
}

impl HeartbeatConsumer {
    pub const fn new(target_node: NodeId, timeout_ms: u16) -> Self {
        Self {
            target_node,
            timeout_us: timeout_ms as u64 * 1000,
            last_seen_us: 0,
            last_state: None,
            timed_out: false,
        }
    }

    /// Feed a received heartbeat frame. Returns true if it was for our target.
    pub fn process(&mut self, node: NodeId, state_byte: u8, now_us: u64) -> bool {
        if node != self.target_node {
            return false;
        }
        self.last_seen_us = now_us;
        self.timed_out = false;
        self.last_state = match state_byte {
            0x00 => Some(NmtState::Initializing),
            0x04 => Some(NmtState::Stopped),
            0x05 => Some(NmtState::Operational),
            0x7F => Some(NmtState::PreOperational),
            _ => None,
        };
        true
    }

    /// Check for timeout. Returns true if the node has timed out (transition edge).
    pub fn check_timeout(&mut self, now_us: u64) -> bool {
        if self.timeout_us == 0 || self.last_state.is_none() {
            return false;
        }
        if !self.timed_out && now_us.wrapping_sub(self.last_seen_us) > self.timeout_us {
            self.timed_out = true;
            return true;
        }
        false
    }

    pub const fn last_state(&self) -> Option<NmtState> {
        self.last_state
    }

    pub const fn is_timed_out(&self) -> bool {
        self.timed_out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heartbeat_producer_interval() {
        let node = NodeId::new(1).unwrap();
        let mut hb = HeartbeatProducer::new(node, 100); // 100ms

        // At t=0, diff is 0 which is < 100ms, no heartbeat yet
        assert!(hb.poll(0, NmtState::PreOperational).is_none());

        // At t=100ms, first heartbeat
        let frame = hb.poll(100_000, NmtState::PreOperational).unwrap();
        assert_eq!(frame.raw_id(), 0x701);
        assert_eq!(frame.data(), &[0x7F]);

        // 50ms later - too early
        assert!(hb.poll(150_000, NmtState::PreOperational).is_none());

        // 200ms - due again
        let frame = hb.poll(200_000, NmtState::Operational).unwrap();
        assert_eq!(frame.data(), &[0x05]);
    }

    #[test]
    fn heartbeat_producer_disabled() {
        let node = NodeId::new(1).unwrap();
        let mut hb = HeartbeatProducer::new(node, 0);
        assert!(hb.poll(0, NmtState::Operational).is_none());
        assert!(hb.poll(1_000_000, NmtState::Operational).is_none());
    }

    #[test]
    fn heartbeat_consumer_timeout() {
        let node = NodeId::new(5).unwrap();
        let mut hc = HeartbeatConsumer::new(node, 200); // 200ms timeout

        // First heartbeat
        assert!(hc.process(node, 0x05, 1_000_000));
        assert_eq!(hc.last_state(), Some(NmtState::Operational));
        assert!(!hc.is_timed_out());

        // Check at 1.1s - not timed out
        assert!(!hc.check_timeout(1_100_000));

        // Check at 1.3s - timed out (200ms since last)
        assert!(hc.check_timeout(1_300_000));
        assert!(hc.is_timed_out());

        // Second check doesn't re-trigger
        assert!(!hc.check_timeout(1_400_000));

        // New heartbeat resets
        assert!(hc.process(node, 0x7F, 1_500_000));
        assert!(!hc.is_timed_out());
    }
}
