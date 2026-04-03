use crate::cobid::{CobId, NodeId, ParsedCobId};
use crate::heartbeat::HeartbeatProducer;
use crate::nmt::{NmtCommand, NmtHandler, NmtState};
use crate::od::ObjectDictionary;
use crate::pdo::{RpdoConfig, RpdoEngine, TpdoConfig, TpdoEngine};
use crate::sdo::SdoServer;
use crate::time::Clock;
use crate::transport::{CanFrame, Transport};
use heapless::Vec;

/// Configuration for creating a Node.
pub struct NodeConfig<const TPDO: usize = 4, const RPDO: usize = 4> {
    pub node_id: NodeId,
    pub heartbeat_interval_ms: u16,
    /// If true, the node transitions directly to Operational after boot,
    /// without waiting for an NMT Start command from a master.
    pub auto_start: bool,
    pub tpdo: [TpdoConfig; TPDO],
    pub rpdo: [RpdoConfig; RPDO],
}

/// A CANopen node. Ties together NMT, SDO server, PDO engines, and heartbeat.
///
/// Generic over the object dictionary type and PDO counts.
pub struct Node<OD: ObjectDictionary, const TPDO: usize = 4, const RPDO: usize = 4> {
    node_id: NodeId,
    od: OD,
    nmt: NmtHandler,
    sdo_server: SdoServer,
    tpdo: TpdoEngine<TPDO>,
    rpdo: RpdoEngine<RPDO>,
    heartbeat: HeartbeatProducer,
    booted: bool,
    auto_start: bool,
}

impl<OD: ObjectDictionary, const TPDO: usize, const RPDO: usize> Node<OD, TPDO, RPDO> {
    pub fn new(config: NodeConfig<TPDO, RPDO>, od: OD) -> Self {
        Self {
            node_id: config.node_id,
            od,
            nmt: NmtHandler::new(),
            sdo_server: SdoServer::new(),
            tpdo: TpdoEngine::new(config.tpdo),
            rpdo: RpdoEngine::new(config.rpdo),
            heartbeat: HeartbeatProducer::new(config.node_id, config.heartbeat_interval_ms),
            booted: false,
            auto_start: config.auto_start,
        }
    }

    pub fn node_id(&self) -> NodeId {
        self.node_id
    }

    pub fn state(&self) -> NmtState {
        self.nmt.state()
    }

    pub fn od(&self) -> &OD {
        &self.od
    }

    pub fn od_mut(&mut self) -> &mut OD {
        &mut self.od
    }

    pub fn tpdo_engine(&self) -> &TpdoEngine<TPDO> {
        &self.tpdo
    }

    pub fn tpdo_engine_mut(&mut self) -> &mut TpdoEngine<TPDO> {
        &mut self.tpdo
    }

    pub fn rpdo_engine(&self) -> &RpdoEngine<RPDO> {
        &self.rpdo
    }

    pub fn rpdo_engine_mut(&mut self) -> &mut RpdoEngine<RPDO> {
        &mut self.rpdo
    }

    /// Main processing function. Call this periodically (e.g., every 1ms).
    ///
    /// Drains received frames from the transport, handles protocol logic,
    /// and queues outgoing frames (heartbeat, PDO, SDO responses).
    pub fn process(&mut self, transport: &mut impl Transport, clock: &impl Clock) {
        let now = clock.now_us();

        // Boot sequence
        if !self.booted {
            self.booted = true;
            let boot_frame = self.heartbeat.send_boot(now);
            let _ = transport.send(&boot_frame);
            self.nmt.boot_complete();
            if self.auto_start {
                self.nmt
                    .process_command(NmtCommand::StartRemoteNode, 0, self.node_id);
            }
        }

        // Drain incoming frames
        while let Some(frame) = transport.recv() {
            self.dispatch_frame(&frame, transport);
        }

        // Heartbeat
        if let Some(hb) = self.heartbeat.poll(now, self.nmt.state()) {
            let _ = transport.send(&hb);
        }

        // TPDO event timers (only in Operational)
        if self.nmt.state() == NmtState::Operational {
            let mut out = Vec::<CanFrame, TPDO>::new();
            self.tpdo.poll(&self.od, now, &mut out);
            for frame in &out {
                let _ = transport.send(frame);
            }
        }
    }

    fn dispatch_frame(&mut self, frame: &CanFrame, transport: &mut impl Transport) {
        let Some(cob) = CobId::new(frame.id()) else {
            return;
        };

        match cob.parse() {
            ParsedCobId::Nmt => {
                if frame.dlc() >= 2 {
                    let data = frame.data();
                    if let Some(cmd) = NmtCommand::from_byte(data[0]) {
                        self.nmt.process_command(cmd, data[1], self.node_id);
                    }
                }
            }

            ParsedCobId::SdoRequest(node) if node == self.node_id => {
                if frame.dlc() >= 8 {
                    let req: [u8; 8] = frame.data().try_into().unwrap();
                    let mut resp = [0u8; 8];
                    if self.sdo_server.process(&req, &mut self.od, &mut resp).is_ok() {
                        let resp_cob = CobId::sdo_tx(self.node_id);
                        if let Some(resp_frame) = CanFrame::new(resp_cob.raw(), &resp) {
                            let _ = transport.send(&resp_frame);
                        }
                    }
                }
            }

            ParsedCobId::Sync => {
                if self.nmt.state() == NmtState::Operational {
                    let mut out = Vec::<CanFrame, TPDO>::new();
                    self.tpdo.on_sync(&self.od, &mut out);
                    for f in &out {
                        let _ = transport.send(f);
                    }
                }
            }

            ParsedCobId::Rpdo { .. } | ParsedCobId::Tpdo { .. } => {
                // For RPDO: we receive on RPDO COB-IDs (which are the remote's TPDO)
                // The RPDO engine matches on COB-ID, so just feed it the frame
                if self.nmt.state() == NmtState::Operational {
                    self.rpdo.process(frame, &mut self.od);
                }
            }

            _ => {} // ignore
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::od::*;
    use crate::datatypes::DataType;
    use crate::transport::MailboxTransport;

    struct MinimalOd {
        device_type: u32,
        error_reg: u8,
    }

    static MINIMAL_META: &[OdEntryMeta] = &[
        OdEntryMeta {
            index: 0x1000, subindex: 0, data_type: DataType::U32,
            access: AccessType::Ro, pdo_mappable: false, name: "device_type",
        },
        OdEntryMeta {
            index: 0x1001, subindex: 0, data_type: DataType::U8,
            access: AccessType::Ro, pdo_mappable: false, name: "error_register",
        },
    ];

    impl ObjectDictionary for MinimalOd {
        fn lookup(&self, index: u16, subindex: u8) -> Option<&'static OdEntryMeta> {
            MINIMAL_META.iter().find(|e| e.index == index && e.subindex == subindex)
        }
        fn read(&self, index: u16, subindex: u8, buf: &mut [u8]) -> Result<usize, OdError> {
            match (index, subindex) {
                (0x1000, 0) => { buf[..4].copy_from_slice(&self.device_type.to_le_bytes()); Ok(4) }
                (0x1001, 0) => { buf[0] = self.error_reg; Ok(1) }
                _ => Err(OdError::NotFound),
            }
        }
        fn write(&mut self, _: u16, _: u8, _: &[u8]) -> Result<(), OdError> {
            Err(OdError::ReadOnly)
        }
        fn sub_count(&self, _: u16) -> Option<u8> { Some(0) }
    }

    struct TestClock(u64);
    impl Clock for TestClock {
        fn now_us(&self) -> u64 { self.0 }
    }

    #[test]
    fn node_boots_and_sends_heartbeat() {
        let config = NodeConfig::<1, 1> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 100,
            auto_start: false,
            tpdo: [TpdoConfig::default()],
            rpdo: [RpdoConfig::default()],
        };
        let od = MinimalOd { device_type: 0x191, error_reg: 0 };
        let mut node = Node::new(config, od);
        let mut transport = MailboxTransport::<16, 16>::new();

        // First process: should send boot heartbeat
        node.process(&mut transport, &TestClock(0));

        let frame = transport.next_to_transmit().unwrap();
        assert_eq!(frame.id(), 0x701); // heartbeat for node 1
        assert_eq!(frame.data()[0], 0x00); // Initializing state in boot message

        assert_eq!(node.state(), NmtState::PreOperational);
    }

    #[test]
    fn node_responds_to_nmt_and_sdo() {
        let config = NodeConfig::<1, 1> {
            node_id: NodeId::new(1).unwrap(),
            heartbeat_interval_ms: 0, // disabled for this test
            auto_start: false,
            tpdo: [TpdoConfig::default()],
            rpdo: [RpdoConfig::default()],
        };
        let od = MinimalOd { device_type: 0x191, error_reg: 0 };
        let mut node = Node::new(config, od);
        let mut transport = MailboxTransport::<16, 16>::new();

        // Boot
        node.process(&mut transport, &TestClock(0));
        let _ = transport.next_to_transmit(); // drain boot heartbeat

        // Send NMT start
        let nmt_frame = CanFrame::new(0x000, &[0x01, 0x01]).unwrap(); // Start node 1
        transport.store_received(nmt_frame).unwrap();
        node.process(&mut transport, &TestClock(1000));
        assert_eq!(node.state(), NmtState::Operational);

        // Send SDO upload request for 0x1000:0
        let sdo_req = CanFrame::new(0x601, &[0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0]).unwrap();
        transport.store_received(sdo_req).unwrap();
        node.process(&mut transport, &TestClock(2000));

        let resp = transport.next_to_transmit().unwrap();
        assert_eq!(resp.id(), 0x581); // SDO response from node 1
        // Check expedited upload response contains device_type
        assert_eq!(
            &resp.data()[4..8],
            &0x191u32.to_le_bytes()
        );
    }
}
