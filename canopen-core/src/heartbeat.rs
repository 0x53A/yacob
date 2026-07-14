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

/// Lifecycle state of one monitored heartbeat producer (0x1016 entry).
///
/// Monitoring starts with the first heartbeat (or boot-up) frame from the
/// producer, per CiA 301: a configured-but-never-seen node is `Waiting`, not
/// failed. `Timeout` means the node *was* alive and then went silent — the
/// two are deliberately distinct for diagnostics.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeartbeatMonitorState {
    /// Not monitored: no matching 0x1016 entry, or its consumer time is 0.
    Disabled,
    /// Configured, but no heartbeat/boot-up seen yet. No timeout can fire.
    Waiting,
    /// Last heartbeat is within the consumer time.
    Alive {
        nmt_state: NmtState,
        last_seen_us: u64,
    },
    /// Was alive, then the consumer time elapsed without a heartbeat.
    Timeout {
        last_nmt_state: NmtState,
        last_seen_us: u64,
        timed_out_us: u64,
    },
}

impl HeartbeatMonitorState {
    /// The "valid" flag: heartbeat seen and fresh.
    pub const fn is_alive(&self) -> bool {
        matches!(self, Self::Alive { .. })
    }
}

/// Typed heartbeat-consumer event, drained via `Node::next_heartbeat_event()`.
///
/// The stack only reports; operational consequences (EMCY, state changes,
/// resets) are application policy.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeartbeatEvent {
    /// First heartbeat/boot-up frame from a monitored node — monitoring is
    /// now active.
    Started { node: NodeId, state: NmtState },
    /// A monitored node reported a different NMT state than before.
    StateChanged {
        node: NodeId,
        old: NmtState,
        new: NmtState,
    },
    /// A monitored node sent a boot-up frame after previously reporting a
    /// non-boot state — it was reset. Emitted in addition to
    /// [`StateChanged`](Self::StateChanged) / [`Recovered`](Self::Recovered).
    RemoteReset { node: NodeId },
    /// A monitored node's consumer time elapsed without a heartbeat.
    ///
    /// The stack never sends EMCY by itself; this is the spot where an
    /// auto-trigger would make sense per CiA recommendations — applications
    /// conventionally report [`EmcyErrorCode::HeartbeatError`] (0x8130) with
    /// `error_register::COMMUNICATION` and the node id in the vendor bytes.
    ///
    /// [`EmcyErrorCode::HeartbeatError`]: crate::emcy::EmcyErrorCode::HeartbeatError
    Timeout { node: NodeId },
    /// A heartbeat arrived from a monitored node that had timed out.
    Recovered { node: NodeId, state: NmtState },
}

/// Maximum number of extra events one heartbeat frame can produce
/// (a state-carrying event plus `RemoteReset`).
pub type HeartbeatEventBuf = heapless::Vec<HeartbeatEvent, 2>;

/// Monitors the heartbeat of one remote node (one 0x1016 entry).
///
/// Frames with an unknown NMT state byte are ignored entirely (conformant
/// producers only send 0x00/0x04/0x05/0x7F; bit 7 toggling is legacy node
/// guarding, which this stack does not implement).
pub struct HeartbeatMonitor {
    node_id: NodeId,
    timeout_ms: u16,
    state: HeartbeatMonitorState,
}

impl HeartbeatMonitor {
    pub const fn new(node_id: NodeId, timeout_ms: u16) -> Self {
        Self {
            node_id,
            timeout_ms,
            state: HeartbeatMonitorState::Waiting,
        }
    }

    /// The monitored producer's node id.
    pub const fn node_id(&self) -> NodeId {
        self.node_id
    }

    /// Consumer time in ms (bits 0..16 of the 0x1016 entry).
    pub const fn timeout_ms(&self) -> u16 {
        self.timeout_ms
    }

    /// Current monitor state (never `Disabled` — a disabled entry has no monitor).
    pub const fn state(&self) -> HeartbeatMonitorState {
        self.state
    }

    /// Feed a received heartbeat frame. Returns true if it was for our node
    /// (whether or not it produced events).
    pub fn process(
        &mut self,
        node: NodeId,
        state_byte: u8,
        now_us: u64,
        events: &mut HeartbeatEventBuf,
    ) -> bool {
        if node != self.node_id {
            return false;
        }
        let Some(new_state) = NmtState::from_heartbeat_byte(state_byte) else {
            return true;
        };
        let node = self.node_id;
        match self.state {
            HeartbeatMonitorState::Disabled | HeartbeatMonitorState::Waiting => {
                let _ = events.push(HeartbeatEvent::Started {
                    node,
                    state: new_state,
                });
            }
            HeartbeatMonitorState::Alive { nmt_state: old, .. } => {
                if new_state != old {
                    let _ = events.push(HeartbeatEvent::StateChanged {
                        node,
                        old,
                        new: new_state,
                    });
                    if new_state == NmtState::Initializing {
                        let _ = events.push(HeartbeatEvent::RemoteReset { node });
                    }
                }
            }
            HeartbeatMonitorState::Timeout { last_nmt_state, .. } => {
                let _ = events.push(HeartbeatEvent::Recovered {
                    node,
                    state: new_state,
                });
                if new_state == NmtState::Initializing && last_nmt_state != NmtState::Initializing {
                    let _ = events.push(HeartbeatEvent::RemoteReset { node });
                }
            }
        }
        self.state = HeartbeatMonitorState::Alive {
            nmt_state: new_state,
            last_seen_us: now_us,
        };
        true
    }

    /// Check for timeout. Returns the `Timeout` event on the alive → timed-out
    /// edge; monitoring only re-arms on the next reception.
    pub fn check_timeout(&mut self, now_us: u64) -> Option<HeartbeatEvent> {
        if let HeartbeatMonitorState::Alive {
            nmt_state,
            last_seen_us,
        } = self.state
        {
            if self.timeout_ms != 0
                && now_us.wrapping_sub(last_seen_us) > self.timeout_ms as u64 * 1000
            {
                self.state = HeartbeatMonitorState::Timeout {
                    last_nmt_state: nmt_state,
                    last_seen_us,
                    timed_out_us: now_us,
                };
                return Some(HeartbeatEvent::Timeout { node: self.node_id });
            }
        }
        None
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
    fn monitor_starts_on_first_heartbeat() {
        let node = NodeId::new(5).unwrap();
        let mut m = HeartbeatMonitor::new(node, 200);
        assert_eq!(m.state(), HeartbeatMonitorState::Waiting);

        // No timeout while waiting — monitoring hasn't started
        assert!(m.check_timeout(10_000_000).is_none());
        assert_eq!(m.state(), HeartbeatMonitorState::Waiting);

        // Frames for other nodes are not ours
        let mut evts = HeartbeatEventBuf::new();
        assert!(!m.process(NodeId::new(6).unwrap(), 0x05, 1_000_000, &mut evts));
        assert!(evts.is_empty());

        // First heartbeat → Started + Alive
        assert!(m.process(node, 0x05, 1_000_000, &mut evts));
        assert_eq!(
            evts.as_slice(),
            &[HeartbeatEvent::Started {
                node,
                state: NmtState::Operational
            }]
        );
        assert!(m.state().is_alive());
    }

    #[test]
    fn monitor_timeout_and_recovery() {
        let node = NodeId::new(5).unwrap();
        let mut m = HeartbeatMonitor::new(node, 200); // 200ms consumer time
        let mut evts = HeartbeatEventBuf::new();

        m.process(node, 0x05, 1_000_000, &mut evts);
        evts.clear();

        // 100ms later — still alive
        assert!(m.check_timeout(1_100_000).is_none());

        // 300ms since last — timed out (edge)
        assert_eq!(
            m.check_timeout(1_300_000),
            Some(HeartbeatEvent::Timeout { node })
        );
        assert!(matches!(
            m.state(),
            HeartbeatMonitorState::Timeout {
                last_nmt_state: NmtState::Operational,
                last_seen_us: 1_000_000,
                timed_out_us: 1_300_000,
            }
        ));

        // Edge does not re-trigger
        assert!(m.check_timeout(1_400_000).is_none());

        // Heartbeat resumes → Recovered, alive again
        assert!(m.process(node, 0x7F, 1_500_000, &mut evts));
        assert_eq!(
            evts.as_slice(),
            &[HeartbeatEvent::Recovered {
                node,
                state: NmtState::PreOperational
            }]
        );
        assert!(m.state().is_alive());
    }

    #[test]
    fn monitor_state_change_and_remote_reset() {
        let node = NodeId::new(5).unwrap();
        let mut m = HeartbeatMonitor::new(node, 200);
        let mut evts = HeartbeatEventBuf::new();

        m.process(node, 0x7F, 1_000_000, &mut evts); // PreOperational
        evts.clear();

        // Same state → no event, refreshes liveness
        m.process(node, 0x7F, 1_100_000, &mut evts);
        assert!(evts.is_empty());

        // PreOperational → Operational
        m.process(node, 0x05, 1_200_000, &mut evts);
        assert_eq!(
            evts.as_slice(),
            &[HeartbeatEvent::StateChanged {
                node,
                old: NmtState::PreOperational,
                new: NmtState::Operational,
            }]
        );
        evts.clear();

        // Boot-up after Operational → StateChanged + RemoteReset
        m.process(node, 0x00, 1_300_000, &mut evts);
        assert_eq!(
            evts.as_slice(),
            &[
                HeartbeatEvent::StateChanged {
                    node,
                    old: NmtState::Operational,
                    new: NmtState::Initializing,
                },
                HeartbeatEvent::RemoteReset { node },
            ]
        );
    }

    #[test]
    fn monitor_reset_during_timeout_reports_remote_reset() {
        let node = NodeId::new(5).unwrap();
        let mut m = HeartbeatMonitor::new(node, 100);
        let mut evts = HeartbeatEventBuf::new();

        m.process(node, 0x05, 1_000_000, &mut evts);
        assert_eq!(
            m.check_timeout(2_000_000),
            Some(HeartbeatEvent::Timeout { node })
        );
        evts.clear();

        // Boot-up while timed out: the reset likely caused the silence
        m.process(node, 0x00, 2_500_000, &mut evts);
        assert_eq!(
            evts.as_slice(),
            &[
                HeartbeatEvent::Recovered {
                    node,
                    state: NmtState::Initializing
                },
                HeartbeatEvent::RemoteReset { node },
            ]
        );
    }

    #[test]
    fn monitor_ignores_unknown_state_bytes() {
        let node = NodeId::new(5).unwrap();
        let mut m = HeartbeatMonitor::new(node, 200);
        let mut evts = HeartbeatEventBuf::new();

        // Unknown byte while waiting: claimed, but monitoring does not start
        assert!(m.process(node, 0xFF, 1_000_000, &mut evts));
        assert!(evts.is_empty());
        assert_eq!(m.state(), HeartbeatMonitorState::Waiting);
    }
}
