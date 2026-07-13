//! CANopen event decoding — semantic view of raw CAN frames.
//!
//! This is the decoding layer of the master application model (see
//! `_Tasks/master-application-model.md`): raw frame distribution and CANopen
//! interpretation are separate concerns. Consumers subscribe to raw
//! [`CanFrame`]s (e.g. via [`bus::SharedCanBus`](crate::bus::SharedCanBus) on
//! std) and decode the ones they care about.
//!
//! PDO classification is a **pre-defined-range classifier**: `pdo_num` and
//! `node` are inferred purely from where the COB-ID falls in the CiA 301
//! pre-defined connection set ranges (TPDO1 = `0x180 + node` etc.), not from
//! any device's actual PDO configuration. A custom/remapped COB-ID *inside*
//! those ranges is attributed to the range it lands in — e.g. a TPDO5 mapped
//! to 0x1B1 decodes as `Tpdo { pdo_num: 1, node: 0x31 }` — and one outside
//! them decodes as `None`. Interpreting remapped PDOs correctly requires the
//! remote node's PDO configuration, which this layer intentionally does not
//! have; treat `pdo_num`/`node` as a hint for monitoring, not ground truth.

use crate::cobid::{CobId, NodeId, ParsedCobId};
use crate::emcy::EmcyMessage;
use crate::nmt::NmtState;
use crate::transport::CanFrame;

/// A decoded CANopen bus event.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanOpenEvent {
    /// NMT command from the master. `target == None` is a broadcast.
    Nmt {
        command: u8,
        target: Option<NodeId>,
    },
    Sync,
    Emcy(EmcyMessage),
    Heartbeat {
        node: NodeId,
        state: NmtState,
    },
    Tpdo {
        pdo_num: u8,
        node: NodeId,
        data: [u8; 8],
        len: u8,
    },
    Rpdo {
        pdo_num: u8,
        node: NodeId,
        data: [u8; 8],
        len: u8,
    },
    SdoResponse {
        node: NodeId,
        data: [u8; 8],
    },
    SdoRequest {
        node: NodeId,
        data: [u8; 8],
    },
}

impl CanOpenEvent {
    /// Decode a raw frame. Returns `None` for frames that are not valid
    /// CANopen messages under the pre-defined connection set (unknown
    /// COB-IDs, malformed payloads, unknown NMT state bytes).
    pub fn decode(frame: &CanFrame) -> Option<Self> {
        let cob = CobId::new(frame.raw_id())?;
        let data = frame.data();
        match cob.parse() {
            ParsedCobId::Nmt => {
                if data.len() < 2 {
                    return None;
                }
                let target = if data[1] == 0 {
                    None
                } else {
                    Some(NodeId::new(data[1])?)
                };
                Some(Self::Nmt {
                    command: data[0],
                    target,
                })
            }
            ParsedCobId::Sync => Some(Self::Sync),
            ParsedCobId::Emergency(_) => EmcyMessage::parse(frame).map(Self::Emcy),
            ParsedCobId::Heartbeat(node) => {
                let state = NmtState::from_heartbeat_byte(*data.first()?)?;
                Some(Self::Heartbeat { node, state })
            }
            ParsedCobId::Tpdo { pdo_num, node } => {
                let (payload, len) = copy_payload(data);
                Some(Self::Tpdo {
                    pdo_num,
                    node,
                    data: payload,
                    len,
                })
            }
            ParsedCobId::Rpdo { pdo_num, node } => {
                let (payload, len) = copy_payload(data);
                Some(Self::Rpdo {
                    pdo_num,
                    node,
                    data: payload,
                    len,
                })
            }
            ParsedCobId::SdoResponse(node) => Some(Self::SdoResponse {
                node,
                data: data.try_into().ok()?,
            }),
            ParsedCobId::SdoRequest(node) => Some(Self::SdoRequest {
                node,
                data: data.try_into().ok()?,
            }),
            ParsedCobId::Unknown(_) => None,
        }
    }

    /// The node this event concerns, if any (`Sync` and broadcast `Nmt`
    /// have none).
    pub fn node(&self) -> Option<NodeId> {
        match self {
            Self::Nmt { target, .. } => *target,
            Self::Sync => None,
            Self::Emcy(msg) => Some(msg.node),
            Self::Heartbeat { node, .. }
            | Self::Tpdo { node, .. }
            | Self::Rpdo { node, .. }
            | Self::SdoResponse { node, .. }
            | Self::SdoRequest { node, .. } => Some(*node),
        }
    }

    /// Whether this event concerns the given node. Broadcast NMT commands
    /// match every node; `Sync` matches none.
    pub fn matches_node(&self, node: NodeId) -> bool {
        match self {
            Self::Nmt { target: None, .. } => true,
            Self::Sync => false,
            _ => self.node() == Some(node),
        }
    }
}

fn copy_payload(data: &[u8]) -> ([u8; 8], u8) {
    let mut payload = [0u8; 8];
    let len = data.len().min(8);
    payload[..len].copy_from_slice(&data[..len]);
    (payload, len as u8)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::emcy::build_emcy_frame;

    fn node(id: u8) -> NodeId {
        NodeId::new(id).unwrap()
    }

    #[test]
    fn decodes_heartbeat() {
        let f = CanFrame::new(0x721, &[0x05]).unwrap();
        assert_eq!(
            CanOpenEvent::decode(&f),
            Some(CanOpenEvent::Heartbeat {
                node: node(0x21),
                state: NmtState::Operational
            })
        );
        // Unknown state byte → not a valid heartbeat
        let bad = CanFrame::new(0x721, &[0x42]).unwrap();
        assert_eq!(CanOpenEvent::decode(&bad), None);
    }

    #[test]
    fn decodes_emcy() {
        let f = build_emcy_frame(node(0x21), 0x2310, 0x02, &[0xAB]);
        match CanOpenEvent::decode(&f) {
            Some(CanOpenEvent::Emcy(msg)) => {
                assert_eq!(msg.node, node(0x21));
                assert_eq!(msg.error_code, 0x2310);
            }
            other => panic!("expected Emcy, got {other:?}"),
        }
    }

    #[test]
    fn decodes_tpdo_with_len() {
        let f = CanFrame::new(0x1A2, &[1, 2, 3]).unwrap(); // TPDO1, node 0x22
        match CanOpenEvent::decode(&f) {
            Some(CanOpenEvent::Tpdo {
                pdo_num,
                node: n,
                data,
                len,
            }) => {
                assert_eq!(pdo_num, 0);
                assert_eq!(n, node(0x22));
                assert_eq!(len, 3);
                assert_eq!(&data[..3], &[1, 2, 3]);
            }
            other => panic!("expected Tpdo, got {other:?}"),
        }
    }

    #[test]
    fn decodes_sdo_response_requires_dlc8() {
        let full = CanFrame::new(0x5A1, &[0x43, 0, 0x10, 0, 0x42, 0, 0, 0]).unwrap();
        assert!(matches!(
            CanOpenEvent::decode(&full),
            Some(CanOpenEvent::SdoResponse { .. })
        ));
        let short = CanFrame::new(0x5A1, &[0x43, 0]).unwrap();
        assert_eq!(CanOpenEvent::decode(&short), None);
    }

    #[test]
    fn nmt_broadcast_matches_all_nodes() {
        let f = CanFrame::new(0x000, &[0x01, 0x00]).unwrap();
        let ev = CanOpenEvent::decode(&f).unwrap();
        assert!(ev.matches_node(node(1)));
        assert!(ev.matches_node(node(0x7F)));
        assert_eq!(ev.node(), None);

        let targeted = CanFrame::new(0x000, &[0x01, 0x21]).unwrap();
        let ev = CanOpenEvent::decode(&targeted).unwrap();
        assert!(ev.matches_node(node(0x21)));
        assert!(!ev.matches_node(node(0x22)));
    }

    #[test]
    fn sync_matches_no_node() {
        let f = CanFrame::new(0x080, &[]).unwrap();
        let ev = CanOpenEvent::decode(&f).unwrap();
        assert_eq!(ev, CanOpenEvent::Sync);
        assert!(!ev.matches_node(node(1)));
    }
}
