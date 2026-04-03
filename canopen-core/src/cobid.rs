/// 7-bit CANopen node ID (1..=127).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct NodeId(u8);

impl NodeId {
    pub const fn new(id: u8) -> Option<Self> {
        if id >= 1 && id <= 127 {
            Some(Self(id))
        } else {
            None
        }
    }

    pub const fn raw(self) -> u8 {
        self.0
    }
}

/// A parsed 11-bit CANopen COB-ID.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CobId(u16);

impl CobId {
    pub const fn new(raw: u16) -> Option<Self> {
        if raw <= 0x7FF {
            Some(Self(raw))
        } else {
            None
        }
    }

    pub const fn raw(self) -> u16 {
        self.0
    }

    pub fn to_standard_id(self) -> embedded_can::StandardId {
        // Safety: we enforce <= 0x7FF in new()
        embedded_can::StandardId::new(self.0).unwrap()
    }

    pub fn parse(self) -> ParsedCobId {
        let raw = self.0;
        match raw {
            0x000 => ParsedCobId::Nmt,
            0x080 => ParsedCobId::Sync,
            0x081..=0x0FF => match NodeId::new((raw & 0x7F) as u8) {
                Some(n) => ParsedCobId::Emergency(n),
                None => ParsedCobId::Unknown(raw),
            },
            0x181..=0x1FF => match NodeId::new((raw - 0x180) as u8) {
                Some(n) => ParsedCobId::Tpdo { pdo_num: 0, node: n },
                None => ParsedCobId::Unknown(raw),
            },
            0x201..=0x27F => match NodeId::new((raw - 0x200) as u8) {
                Some(n) => ParsedCobId::Rpdo { pdo_num: 0, node: n },
                None => ParsedCobId::Unknown(raw),
            },
            0x281..=0x2FF => match NodeId::new((raw - 0x280) as u8) {
                Some(n) => ParsedCobId::Tpdo { pdo_num: 1, node: n },
                None => ParsedCobId::Unknown(raw),
            },
            0x301..=0x37F => match NodeId::new((raw - 0x300) as u8) {
                Some(n) => ParsedCobId::Rpdo { pdo_num: 1, node: n },
                None => ParsedCobId::Unknown(raw),
            },
            0x381..=0x3FF => match NodeId::new((raw - 0x380) as u8) {
                Some(n) => ParsedCobId::Tpdo { pdo_num: 2, node: n },
                None => ParsedCobId::Unknown(raw),
            },
            0x401..=0x47F => match NodeId::new((raw - 0x400) as u8) {
                Some(n) => ParsedCobId::Rpdo { pdo_num: 2, node: n },
                None => ParsedCobId::Unknown(raw),
            },
            0x481..=0x4FF => match NodeId::new((raw - 0x480) as u8) {
                Some(n) => ParsedCobId::Tpdo { pdo_num: 3, node: n },
                None => ParsedCobId::Unknown(raw),
            },
            0x501..=0x57F => match NodeId::new((raw - 0x500) as u8) {
                Some(n) => ParsedCobId::Rpdo { pdo_num: 3, node: n },
                None => ParsedCobId::Unknown(raw),
            },
            0x581..=0x5FF => match NodeId::new((raw - 0x580) as u8) {
                Some(n) => ParsedCobId::SdoResponse(n),
                None => ParsedCobId::Unknown(raw),
            },
            0x601..=0x67F => match NodeId::new((raw - 0x600) as u8) {
                Some(n) => ParsedCobId::SdoRequest(n),
                None => ParsedCobId::Unknown(raw),
            },
            0x701..=0x77F => match NodeId::new((raw - 0x700) as u8) {
                Some(n) => ParsedCobId::Heartbeat(n),
                None => ParsedCobId::Unknown(raw),
            },
            _ => ParsedCobId::Unknown(raw),
        }
    }
}

/// Semantic interpretation of a COB-ID.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParsedCobId {
    Nmt,
    Sync,
    Emergency(NodeId),
    Tpdo { pdo_num: u8, node: NodeId },
    Rpdo { pdo_num: u8, node: NodeId },
    SdoResponse(NodeId),
    SdoRequest(NodeId),
    Heartbeat(NodeId),
    Unknown(u16),
}

/// COB-ID construction helpers.
impl CobId {
    pub const fn nmt() -> Self {
        Self(0x000)
    }

    pub const fn sync() -> Self {
        Self(0x080)
    }

    pub const fn emergency(node: NodeId) -> Self {
        Self(0x080 + node.raw() as u16)
    }

    pub const fn tpdo(pdo_num: u8, node: NodeId) -> Self {
        let base = match pdo_num {
            0 => 0x180,
            1 => 0x280,
            2 => 0x380,
            3 => 0x480,
            _ => 0x180, // default to TPDO1
        };
        Self(base + node.raw() as u16)
    }

    pub const fn rpdo(pdo_num: u8, node: NodeId) -> Self {
        let base = match pdo_num {
            0 => 0x200,
            1 => 0x300,
            2 => 0x400,
            3 => 0x500,
            _ => 0x200,
        };
        Self(base + node.raw() as u16)
    }

    pub const fn sdo_tx(node: NodeId) -> Self {
        Self(0x580 + node.raw() as u16)
    }

    pub const fn sdo_rx(node: NodeId) -> Self {
        Self(0x600 + node.raw() as u16)
    }

    pub const fn heartbeat(node: NodeId) -> Self {
        Self(0x700 + node.raw() as u16)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn node_id_range() {
        assert!(NodeId::new(0).is_none());
        assert!(NodeId::new(1).is_some());
        assert!(NodeId::new(127).is_some());
        assert!(NodeId::new(128).is_none());
    }

    #[test]
    fn cobid_parse_roundtrip() {
        let node = NodeId::new(5).unwrap();
        assert_eq!(CobId::heartbeat(node).parse(), ParsedCobId::Heartbeat(node));
        assert_eq!(CobId::sdo_rx(node).parse(), ParsedCobId::SdoRequest(node));
        assert_eq!(CobId::sdo_tx(node).parse(), ParsedCobId::SdoResponse(node));
        assert_eq!(
            CobId::tpdo(2, node).parse(),
            ParsedCobId::Tpdo { pdo_num: 2, node }
        );
        assert_eq!(
            CobId::rpdo(1, node).parse(),
            ParsedCobId::Rpdo { pdo_num: 1, node }
        );
        assert_eq!(CobId::nmt().parse(), ParsedCobId::Nmt);
        assert_eq!(CobId::sync().parse(), ParsedCobId::Sync);
    }

    #[test]
    fn cobid_raw_values() {
        let node = NodeId::new(1).unwrap();
        assert_eq!(CobId::heartbeat(node).raw(), 0x701);
        assert_eq!(CobId::sdo_rx(node).raw(), 0x601);
        assert_eq!(CobId::sdo_tx(node).raw(), 0x581);
        assert_eq!(CobId::tpdo(0, node).raw(), 0x181);
        assert_eq!(CobId::rpdo(0, node).raw(), 0x201);
    }
}
