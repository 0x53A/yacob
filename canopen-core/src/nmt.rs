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

    /// Decode a heartbeat state byte.
    pub const fn from_heartbeat_byte(byte: u8) -> Option<Self> {
        match byte {
            0x00 => Some(Self::Initializing),
            0x04 => Some(Self::Stopped),
            0x05 => Some(Self::Operational),
            0x7F => Some(Self::PreOperational),
            _ => None,
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

    /// Build a CAN frame for this NMT command.
    ///
    /// `target_node` is the target node ID (1-127), or 0 to broadcast to all nodes.
    ///
    /// ```
    /// # use canopen_core::nmt::NmtCommand;
    /// let frame = NmtCommand::StartRemoteNode.to_frame(0); // broadcast start
    /// assert_eq!(frame.data(), &[0x01, 0x00]);
    ///
    /// let frame = NmtCommand::ResetNode.to_frame(5); // reset node 5
    /// assert_eq!(frame.data(), &[0x81, 0x05]);
    /// ```
    pub fn to_frame(self, target_node: u8) -> crate::transport::CanFrame {
        // NMT command frames always use COB-ID 0x000
        crate::transport::CanFrame::new(0x000, &[self.to_byte(), target_node]).unwrap()
    }
}

/// Result of processing an NMT command, indicating what kind of
/// reset (if any) the node should perform.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NmtTransition {
    /// No state change occurred (wrong target or same state).
    None,
    /// Normal state transition (Start, Stop, EnterPreOp).
    StateChanged,
    /// ResetNode — node should reset all application parameters to defaults,
    /// re-initialize PDOs and heartbeat, then boot.
    ResetApplication,
    /// ResetCommunication — node should reset only communication parameters
    /// (PDO config, heartbeat) to defaults, keep application data, then boot.
    ResetCommunication,
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
    /// Returns the type of transition that occurred.
    pub fn process_command(
        &mut self,
        cmd: NmtCommand,
        target: u8,
        our_node: NodeId,
    ) -> NmtTransition {
        if target != 0 && target != our_node.raw() {
            return NmtTransition::None;
        }
        let new_state = match cmd {
            NmtCommand::StartRemoteNode => NmtState::Operational,
            NmtCommand::StopRemoteNode => NmtState::Stopped,
            NmtCommand::EnterPreOperational => NmtState::PreOperational,
            NmtCommand::ResetNode | NmtCommand::ResetCommunication => {
                NmtState::Initializing
            }
        };
        if self.state == new_state {
            return NmtTransition::None;
        }
        self.state = new_state;
        match cmd {
            NmtCommand::ResetNode => NmtTransition::ResetApplication,
            NmtCommand::ResetCommunication => NmtTransition::ResetCommunication,
            _ => NmtTransition::StateChanged,
        }
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
        assert_eq!(nmt.process_command(NmtCommand::StartRemoteNode, 0, node), NmtTransition::StateChanged);
        assert_eq!(nmt.state(), NmtState::Operational);

        // Stop (addressed)
        assert_eq!(nmt.process_command(NmtCommand::StopRemoteNode, 5, node), NmtTransition::StateChanged);
        assert_eq!(nmt.state(), NmtState::Stopped);

        // Wrong target - no change
        assert_eq!(nmt.process_command(NmtCommand::StartRemoteNode, 3, node), NmtTransition::None);
        assert_eq!(nmt.state(), NmtState::Stopped);

        // ResetNode
        assert_eq!(nmt.process_command(NmtCommand::ResetNode, 0, node), NmtTransition::ResetApplication);
        assert_eq!(nmt.state(), NmtState::Initializing);

        // ResetCommunication
        nmt.boot_complete();
        assert_eq!(nmt.process_command(NmtCommand::ResetCommunication, 0, node), NmtTransition::ResetCommunication);
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
    fn heartbeat_byte_roundtrip() {
        for state in [NmtState::Initializing, NmtState::Stopped, NmtState::Operational, NmtState::PreOperational] {
            assert_eq!(NmtState::from_heartbeat_byte(state.heartbeat_byte()), Some(state));
        }
        assert_eq!(NmtState::from_heartbeat_byte(0xFF), None);
    }

    #[test]
    fn nmt_command_to_frame() {
        use embedded_can::Frame;

        let frame = NmtCommand::StartRemoteNode.to_frame(0);
        assert_eq!(frame.data(), &[0x01, 0x00]);
        assert_eq!(frame.id(), embedded_can::Id::Standard(embedded_can::StandardId::new(0).unwrap()));

        let frame = NmtCommand::ResetNode.to_frame(5);
        assert_eq!(frame.data(), &[0x81, 0x05]);
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
