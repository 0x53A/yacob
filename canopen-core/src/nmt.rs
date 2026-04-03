use crate::cobid::NodeId;

/// CANopen NMT states.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NmtState {
    Initializing,
    PreOperational,
    Operational,
    Stopped,
}

impl NmtState {
    /// Heartbeat state byte encoding.
    pub const fn heartbeat_byte(self) -> u8 {
        match self {
            Self::Initializing => 0x00,
            Self::Stopped => 0x04,
            Self::Operational => 0x05,
            Self::PreOperational => 0x7F,
        }
    }
}

/// NMT commands sent by the NMT master.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NmtCommand {
    StartRemoteNode,
    StopRemoteNode,
    EnterPreOperational,
    ResetNode,
    ResetCommunication,
}

impl NmtCommand {
    pub const fn from_byte(b: u8) -> Option<Self> {
        match b {
            0x01 => Some(Self::StartRemoteNode),
            0x02 => Some(Self::StopRemoteNode),
            0x80 => Some(Self::EnterPreOperational),
            0x81 => Some(Self::ResetNode),
            0x82 => Some(Self::ResetCommunication),
            _ => None,
        }
    }

    pub const fn to_byte(self) -> u8 {
        match self {
            Self::StartRemoteNode => 0x01,
            Self::StopRemoteNode => 0x02,
            Self::EnterPreOperational => 0x80,
            Self::ResetNode => 0x81,
            Self::ResetCommunication => 0x82,
        }
    }
}

/// NMT state machine handler.
pub struct NmtHandler {
    state: NmtState,
}

impl NmtHandler {
    pub const fn new() -> Self {
        Self {
            state: NmtState::Initializing,
        }
    }

    pub const fn state(&self) -> NmtState {
        self.state
    }

    /// Called after node initialization completes.
    pub fn boot_complete(&mut self) {
        self.state = NmtState::PreOperational;
    }

    /// Process an incoming NMT command (from COB-ID 0x000).
    /// `target` is the node ID byte from the NMT frame (0 = all nodes).
    /// Returns true if the state changed.
    pub fn process_command(
        &mut self,
        cmd: NmtCommand,
        target: u8,
        our_node: NodeId,
    ) -> bool {
        if target != 0 && target != our_node.raw() {
            return false;
        }
        let new_state = match cmd {
            NmtCommand::StartRemoteNode => NmtState::Operational,
            NmtCommand::StopRemoteNode => NmtState::Stopped,
            NmtCommand::EnterPreOperational => NmtState::PreOperational,
            NmtCommand::ResetNode | NmtCommand::ResetCommunication => {
                NmtState::Initializing
            }
        };
        let changed = self.state != new_state;
        self.state = new_state;
        changed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nmt_boot_sequence() {
        let mut nmt = NmtHandler::new();
        assert_eq!(nmt.state(), NmtState::Initializing);
        nmt.boot_complete();
        assert_eq!(nmt.state(), NmtState::PreOperational);
    }

    #[test]
    fn nmt_commands() {
        let node = NodeId::new(5).unwrap();
        let mut nmt = NmtHandler::new();
        nmt.boot_complete();

        // Start (broadcast)
        assert!(nmt.process_command(NmtCommand::StartRemoteNode, 0, node));
        assert_eq!(nmt.state(), NmtState::Operational);

        // Stop (addressed)
        assert!(nmt.process_command(NmtCommand::StopRemoteNode, 5, node));
        assert_eq!(nmt.state(), NmtState::Stopped);

        // Wrong target - no change
        assert!(!nmt.process_command(NmtCommand::StartRemoteNode, 3, node));
        assert_eq!(nmt.state(), NmtState::Stopped);

        // Reset
        assert!(nmt.process_command(NmtCommand::ResetNode, 0, node));
        assert_eq!(nmt.state(), NmtState::Initializing);
    }

    #[test]
    fn heartbeat_bytes() {
        assert_eq!(NmtState::Initializing.heartbeat_byte(), 0x00);
        assert_eq!(NmtState::Stopped.heartbeat_byte(), 0x04);
        assert_eq!(NmtState::Operational.heartbeat_byte(), 0x05);
        assert_eq!(NmtState::PreOperational.heartbeat_byte(), 0x7F);
    }

    #[test]
    fn nmt_command_roundtrip() {
        for &cmd in &[
            NmtCommand::StartRemoteNode,
            NmtCommand::StopRemoteNode,
            NmtCommand::EnterPreOperational,
            NmtCommand::ResetNode,
            NmtCommand::ResetCommunication,
        ] {
            assert_eq!(NmtCommand::from_byte(cmd.to_byte()), Some(cmd));
        }
    }
}
