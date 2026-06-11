//! Layer Setting Services (LSS) slave implementation.
//!
//! LSS allows an LSS master to discover and configure unconfigured nodes
//! on the bus. The slave responds on COB-ID 0x7E4 (LSS request) and
//! sends responses on 0x7E5 (LSS response).
//!
//! Supported services:
//! - Switch Mode Global (waiting ↔ configuration)
//! - Switch Mode Selective (by identity: vendor/product/revision/serial)
//! - Configure Node ID
//! - Store Configuration (signals the application via callback)
//! - Identity inquiry (vendor, product, revision, serial, node ID)

use crate::transport::CanFrame;

/// LSS request COB-ID (master → slave).
pub const LSS_REQUEST_COB: u16 = 0x7E5;
/// LSS response COB-ID (slave → master).
pub const LSS_RESPONSE_COB: u16 = 0x7E4;

/// LSS command specifiers (CiA 305).
#[allow(dead_code)]
mod cs {
    pub const SWITCH_MODE_GLOBAL: u8 = 0x04;
    pub const CONFIGURE_NODE_ID: u8 = 0x11;
    pub const CONFIGURE_BIT_TIMING: u8 = 0x13;
    pub const STORE_CONFIGURATION: u8 = 0x17;
    pub const SWITCH_VENDOR: u8 = 0x40;
    pub const SWITCH_PRODUCT: u8 = 0x41;
    pub const SWITCH_REVISION: u8 = 0x42;
    pub const SWITCH_SERIAL: u8 = 0x43;
    pub const INQUIRE_VENDOR: u8 = 0x5A;
    pub const INQUIRE_PRODUCT: u8 = 0x5B;
    pub const INQUIRE_REVISION: u8 = 0x5C;
    pub const INQUIRE_SERIAL: u8 = 0x5D;
    pub const INQUIRE_NODE_ID: u8 = 0x5E;
}

/// Identity of a CANopen node (0x1018 record).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LssIdentity {
    pub vendor_id: u32,
    pub product_code: u32,
    pub revision: u32,
    pub serial: u32,
}

/// LSS slave mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LssMode {
    Waiting,
    Configuration,
}

/// Events the LSS slave wants the application to handle.
#[derive(Clone, Copy, Debug)]
pub enum LssEvent {
    /// Node ID was configured to the given value.
    NodeIdConfigured(u8),
    /// Master requested "store configuration" — the application should
    /// save the current node ID to persistent storage.
    StoreConfiguration,
}

/// LSS slave state machine.
pub struct LssSlave {
    identity: LssIdentity,
    node_id: u8,
    mode: LssMode,
    /// Partial selective switch state: tracking which identity fields matched.
    selective_match: [bool; 4],
    pending_event: Option<LssEvent>,
}

impl LssSlave {
    pub const fn new(identity: LssIdentity, node_id: u8) -> Self {
        Self {
            identity,
            node_id,
            mode: LssMode::Waiting,
            selective_match: [false; 4],
            pending_event: None,
        }
    }

    pub const fn mode(&self) -> LssMode {
        self.mode
    }

    pub const fn node_id(&self) -> u8 {
        self.node_id
    }

    /// Take the next pending event for the application.
    pub fn take_event(&mut self) -> Option<LssEvent> {
        self.pending_event.take()
    }

    /// Process an incoming LSS request frame (COB-ID 0x7E5).
    /// Returns a response frame to send on 0x7E4, if any.
    pub fn process(&mut self, frame: &CanFrame) -> Option<CanFrame> {
        if frame.raw_id() != LSS_REQUEST_COB || frame.raw_dlc() < 8 {
            return None;
        }

        let data = frame.data();
        let cmd = data[0];

        match cmd {
            cs::SWITCH_MODE_GLOBAL => {
                let new_mode = data[1];
                match new_mode {
                    0 => {
                        self.mode = LssMode::Waiting;
                        self.selective_match = [false; 4];
                    }
                    1 => {
                        self.mode = LssMode::Configuration;
                    }
                    _ => {}
                }
                None // no response for global switch
            }

            cs::SWITCH_VENDOR => {
                let val = u32::from_le_bytes([data[1], data[2], data[3], data[4]]);
                self.selective_match[0] = val == self.identity.vendor_id;
                None
            }
            cs::SWITCH_PRODUCT => {
                let val = u32::from_le_bytes([data[1], data[2], data[3], data[4]]);
                self.selective_match[1] = val == self.identity.product_code;
                None
            }
            cs::SWITCH_REVISION => {
                let val = u32::from_le_bytes([data[1], data[2], data[3], data[4]]);
                self.selective_match[2] = val == self.identity.revision;
                None
            }
            cs::SWITCH_SERIAL => {
                let val = u32::from_le_bytes([data[1], data[2], data[3], data[4]]);
                self.selective_match[3] = val == self.identity.serial;
                // Check if all four match
                if self.selective_match.iter().all(|&m| m) {
                    self.mode = LssMode::Configuration;
                    // Respond with switch state response (0x44)
                    let mut resp = [0u8; 8];
                    resp[0] = 0x44;
                    return self.make_response(&resp);
                }
                self.selective_match = [false; 4];
                None
            }

            cs::CONFIGURE_NODE_ID if self.mode == LssMode::Configuration => {
                let new_id = data[1];
                let mut resp = [0u8; 8];
                resp[0] = cs::CONFIGURE_NODE_ID;
                if new_id >= 1 && new_id <= 127 {
                    self.node_id = new_id;
                    resp[1] = 0; // success
                    self.pending_event = Some(LssEvent::NodeIdConfigured(new_id));
                } else if new_id == 0xFF {
                    // "unconfigured" — valid per CiA 305
                    self.node_id = new_id;
                    resp[1] = 0;
                    self.pending_event = Some(LssEvent::NodeIdConfigured(new_id));
                } else {
                    resp[1] = 1; // node ID out of range
                }
                self.make_response(&resp)
            }

            cs::STORE_CONFIGURATION if self.mode == LssMode::Configuration => {
                self.pending_event = Some(LssEvent::StoreConfiguration);
                let mut resp = [0u8; 8];
                resp[0] = cs::STORE_CONFIGURATION;
                resp[1] = 0; // success
                self.make_response(&resp)
            }

            cs::INQUIRE_VENDOR if self.mode == LssMode::Configuration => {
                let mut resp = [0u8; 8];
                resp[0] = cs::INQUIRE_VENDOR;
                resp[1..5].copy_from_slice(&self.identity.vendor_id.to_le_bytes());
                self.make_response(&resp)
            }
            cs::INQUIRE_PRODUCT if self.mode == LssMode::Configuration => {
                let mut resp = [0u8; 8];
                resp[0] = cs::INQUIRE_PRODUCT;
                resp[1..5].copy_from_slice(&self.identity.product_code.to_le_bytes());
                self.make_response(&resp)
            }
            cs::INQUIRE_REVISION if self.mode == LssMode::Configuration => {
                let mut resp = [0u8; 8];
                resp[0] = cs::INQUIRE_REVISION;
                resp[1..5].copy_from_slice(&self.identity.revision.to_le_bytes());
                self.make_response(&resp)
            }
            cs::INQUIRE_SERIAL if self.mode == LssMode::Configuration => {
                let mut resp = [0u8; 8];
                resp[0] = cs::INQUIRE_SERIAL;
                resp[1..5].copy_from_slice(&self.identity.serial.to_le_bytes());
                self.make_response(&resp)
            }
            cs::INQUIRE_NODE_ID if self.mode == LssMode::Configuration => {
                let mut resp = [0u8; 8];
                resp[0] = cs::INQUIRE_NODE_ID;
                resp[1] = self.node_id;
                self.make_response(&resp)
            }

            _ => None, // ignore unknown or commands when not in configuration mode
        }
    }

    fn make_response(&self, data: &[u8; 8]) -> Option<CanFrame> {
        CanFrame::new(LSS_RESPONSE_COB, data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_identity() -> LssIdentity {
        LssIdentity {
            vendor_id: 0xCAFE,
            product_code: 0x0001,
            revision: 0x00010000,
            serial: 0x00000001,
        }
    }

    #[test]
    fn switch_mode_global() {
        let mut lss = LssSlave::new(test_identity(), 1);
        assert_eq!(lss.mode(), LssMode::Waiting);

        // Switch to configuration
        let req = CanFrame::new(LSS_REQUEST_COB, &[0x04, 0x01, 0, 0, 0, 0, 0, 0]).unwrap();
        assert!(lss.process(&req).is_none()); // no response
        assert_eq!(lss.mode(), LssMode::Configuration);

        // Switch back to waiting
        let req = CanFrame::new(LSS_REQUEST_COB, &[0x04, 0x00, 0, 0, 0, 0, 0, 0]).unwrap();
        lss.process(&req);
        assert_eq!(lss.mode(), LssMode::Waiting);
    }

    #[test]
    fn selective_switch_by_identity() {
        let mut lss = LssSlave::new(test_identity(), 1);

        // Send matching identity fields
        let vendor = CanFrame::new(LSS_REQUEST_COB, &{
            let mut d = [0u8; 8];
            d[0] = 0x40;
            d[1..5].copy_from_slice(&0xCAFEu32.to_le_bytes());
            d
        })
        .unwrap();
        let product = CanFrame::new(LSS_REQUEST_COB, &{
            let mut d = [0u8; 8];
            d[0] = 0x41;
            d[1..5].copy_from_slice(&0x0001u32.to_le_bytes());
            d
        })
        .unwrap();
        let revision = CanFrame::new(LSS_REQUEST_COB, &{
            let mut d = [0u8; 8];
            d[0] = 0x42;
            d[1..5].copy_from_slice(&0x00010000u32.to_le_bytes());
            d
        })
        .unwrap();
        let serial = CanFrame::new(LSS_REQUEST_COB, &{
            let mut d = [0u8; 8];
            d[0] = 0x43;
            d[1..5].copy_from_slice(&0x00000001u32.to_le_bytes());
            d
        })
        .unwrap();

        assert!(lss.process(&vendor).is_none());
        assert!(lss.process(&product).is_none());
        assert!(lss.process(&revision).is_none());
        let resp = lss.process(&serial).unwrap();
        assert_eq!(resp.data()[0], 0x44); // switch state response
        assert_eq!(lss.mode(), LssMode::Configuration);
    }

    #[test]
    fn selective_switch_wrong_serial() {
        let mut lss = LssSlave::new(test_identity(), 1);

        let vendor = CanFrame::new(LSS_REQUEST_COB, &{
            let mut d = [0u8; 8];
            d[0] = 0x40;
            d[1..5].copy_from_slice(&0xCAFEu32.to_le_bytes());
            d
        })
        .unwrap();
        let product = CanFrame::new(LSS_REQUEST_COB, &{
            let mut d = [0u8; 8];
            d[0] = 0x41;
            d[1..5].copy_from_slice(&0x0001u32.to_le_bytes());
            d
        })
        .unwrap();
        let revision = CanFrame::new(LSS_REQUEST_COB, &{
            let mut d = [0u8; 8];
            d[0] = 0x42;
            d[1..5].copy_from_slice(&0x00010000u32.to_le_bytes());
            d
        })
        .unwrap();
        // Wrong serial
        let serial = CanFrame::new(LSS_REQUEST_COB, &{
            let mut d = [0u8; 8];
            d[0] = 0x43;
            d[1..5].copy_from_slice(&0x99999999u32.to_le_bytes());
            d
        })
        .unwrap();

        lss.process(&vendor);
        lss.process(&product);
        lss.process(&revision);
        assert!(lss.process(&serial).is_none());
        assert_eq!(lss.mode(), LssMode::Waiting);
    }

    #[test]
    fn configure_node_id() {
        let mut lss = LssSlave::new(test_identity(), 1);
        // Enter configuration
        let switch = CanFrame::new(LSS_REQUEST_COB, &[0x04, 0x01, 0, 0, 0, 0, 0, 0]).unwrap();
        lss.process(&switch);

        // Configure node ID = 5
        let cfg = CanFrame::new(LSS_REQUEST_COB, &[0x11, 5, 0, 0, 0, 0, 0, 0]).unwrap();
        let resp = lss.process(&cfg).unwrap();
        assert_eq!(resp.data()[0], 0x11); // configure node ID response
        assert_eq!(resp.data()[1], 0); // success
        assert_eq!(lss.node_id(), 5);

        let evt = lss.take_event().unwrap();
        assert!(matches!(evt, LssEvent::NodeIdConfigured(5)));
    }

    #[test]
    fn configure_node_id_invalid() {
        let mut lss = LssSlave::new(test_identity(), 1);
        let switch = CanFrame::new(LSS_REQUEST_COB, &[0x04, 0x01, 0, 0, 0, 0, 0, 0]).unwrap();
        lss.process(&switch);

        // ID 0 is invalid (not 1-127 or 0xFF)
        let cfg = CanFrame::new(LSS_REQUEST_COB, &[0x11, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        let resp = lss.process(&cfg).unwrap();
        assert_eq!(resp.data()[1], 1); // error: out of range
        assert_eq!(lss.node_id(), 1); // unchanged
    }

    #[test]
    fn inquire_identity() {
        let mut lss = LssSlave::new(test_identity(), 42);
        let switch = CanFrame::new(LSS_REQUEST_COB, &[0x04, 0x01, 0, 0, 0, 0, 0, 0]).unwrap();
        lss.process(&switch);

        let req = CanFrame::new(LSS_REQUEST_COB, &[0x5A, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        let resp = lss.process(&req).unwrap();
        assert_eq!(resp.data()[0], 0x5A);
        assert_eq!(
            u32::from_le_bytes(resp.data()[1..5].try_into().unwrap()),
            0xCAFE
        );

        let req = CanFrame::new(LSS_REQUEST_COB, &[0x5E, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        let resp = lss.process(&req).unwrap();
        assert_eq!(resp.data()[0], 0x5E);
        assert_eq!(resp.data()[1], 42);
    }
}
