use crate::cobid::{CobId, NodeId};
use crate::transport::CanFrame;

/// Emergency error codes (CiA 301).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum EmcyErrorCode {
    NoError = 0x0000,
    GenericError = 0x1000,
    CurrentGeneric = 0x2000,
    VoltageGeneric = 0x3000,
    TemperatureGeneric = 0x4000,
    DeviceHardwareGeneric = 0x5000,
    DeviceSoftwareGeneric = 0x6000,
    AdditionalModulesGeneric = 0x7000,
    MonitoringGeneric = 0x8000,
    CommunicationGeneric = 0x8100,
    ProtocolError = 0x8200,
    /// RPDO timeout — deadline monitoring (event timer) expired without
    /// reception. The stack never sends this automatically; applications
    /// that detect an expired deadline (`Node::rpdo_deadline_expired`) may
    /// report it via `Node::set_error`, conventionally with
    /// `error_register::COMMUNICATION` and the PDO number in the vendor bytes.
    RpdoTimeout = 0x8250,
    ExternalError = 0x9000,
    ManufacturerSpecific = 0xFF00,
}

/// Build an EMCY frame.
///
/// - `error_code`: 16-bit emergency error code
/// - `error_register`: contents of OD 0x1001
/// - `vendor_data`: up to 5 bytes of manufacturer-specific data
pub fn build_emcy_frame(
    node_id: NodeId,
    error_code: u16,
    error_register: u8,
    vendor_data: &[u8],
) -> CanFrame {
    let cob = CobId::emergency(node_id);
    let mut data = [0u8; 8];
    data[0] = (error_code & 0xFF) as u8;
    data[1] = (error_code >> 8) as u8;
    data[2] = error_register;
    let vlen = vendor_data.len().min(5);
    data[3..3 + vlen].copy_from_slice(&vendor_data[..vlen]);
    CanFrame::new(cob.raw(), &data).unwrap()
}

/// A received EMCY message (consumer side).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EmcyMessage {
    pub node: NodeId,
    pub error_code: u16,
    pub error_register: u8,
    /// Manufacturer-specific bytes (frame bytes 3..8).
    pub vendor_data: [u8; 5],
}

impl EmcyMessage {
    /// `true` for the "error reset / no error" message (code 0x0000).
    pub const fn is_error_reset(&self) -> bool {
        self.error_code == 0x0000
    }

    /// Parse a received EMCY frame. Returns `None` if the COB-ID is not an
    /// EMCY id or the frame is shorter than the mandatory 8 bytes.
    pub fn parse(frame: &CanFrame) -> Option<Self> {
        let node = match CobId::new(frame.raw_id())?.parse() {
            crate::cobid::ParsedCobId::Emergency(n) => n,
            _ => return None,
        };
        let data = frame.data();
        if data.len() < 8 {
            return None;
        }
        let mut vendor_data = [0u8; 5];
        vendor_data.copy_from_slice(&data[3..8]);
        Some(Self {
            node,
            error_code: u16::from_le_bytes([data[0], data[1]]),
            error_register: data[2],
            vendor_data,
        })
    }
}

/// Error register bits (CiA 301, object 0x1001).
pub mod error_register {
    pub const GENERIC: u8 = 1 << 0;
    pub const CURRENT: u8 = 1 << 1;
    pub const VOLTAGE: u8 = 1 << 2;
    pub const TEMPERATURE: u8 = 1 << 3;
    pub const COMMUNICATION: u8 = 1 << 4;
    pub const DEVICE_PROFILE: u8 = 1 << 5;
    pub const MANUFACTURER: u8 = 1 << 7;
}

/// EMCY producer. Queues emergency frames and tracks the error register.
///
/// Call `set_error()` to report a new error. The frame will be sent on the
/// next `Node::process()` call. Call `clear_error()` to clear error bits
/// and send an "error reset" EMCY (code 0x0000).
///
/// Up to 4 EMCY frames can be pending at once (burst errors between two
/// `Node::process()` calls). On overflow the oldest frame is dropped — later
/// frames carry the accumulated error register.
pub struct EmcyProducer {
    node_id: NodeId,
    error_register: u8,
    pending: heapless::Deque<CanFrame, 4>,
}

impl EmcyProducer {
    pub const fn new(node_id: NodeId) -> Self {
        Self {
            node_id,
            error_register: 0,
            pending: heapless::Deque::new(),
        }
    }

    fn queue(&mut self, frame: CanFrame) {
        if self.pending.is_full() {
            self.pending.pop_front();
        }
        let _ = self.pending.push_back(frame);
    }

    /// Current error register value (for OD 0x1001 reads).
    pub const fn error_register(&self) -> u8 {
        self.error_register
    }

    /// Report an error. Sets the corresponding error register bits and
    /// queues an EMCY frame with the given error code and optional vendor data.
    pub fn set_error(&mut self, error_code: u16, register_bits: u8, vendor_data: &[u8]) {
        self.error_register |= register_bits | error_register::GENERIC;
        self.queue(build_emcy_frame(
            self.node_id,
            error_code,
            self.error_register,
            vendor_data,
        ));
    }

    /// Clear error register bits. If the register becomes 0, sends an
    /// "error reset" EMCY frame (code 0x0000).
    pub fn clear_error(&mut self, register_bits: u8) {
        self.error_register &= !register_bits;
        if self.error_register == 0 {
            // Also clear the generic bit
            self.queue(build_emcy_frame(
                self.node_id,
                EmcyErrorCode::NoError as u16,
                0,
                &[],
            ));
        }
    }

    /// Clear all errors and send error-reset EMCY.
    pub fn clear_all(&mut self) {
        self.error_register = 0;
        self.queue(build_emcy_frame(
            self.node_id,
            EmcyErrorCode::NoError as u16,
            0,
            &[],
        ));
    }

    /// Take the oldest pending EMCY frame to transmit, if any.
    pub fn take_pending(&mut self) -> Option<CanFrame> {
        self.pending.pop_front()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn emcy_frame_format() {
        let node = NodeId::new(5).unwrap();
        let frame = build_emcy_frame(node, 0x1000, 0x01, &[0xAA, 0xBB]);
        assert_eq!(frame.raw_id(), 0x085); // 0x80 + 5
        assert_eq!(frame.data()[0..2], [0x00, 0x10]); // error code LE
        assert_eq!(frame.data()[2], 0x01); // error register
        assert_eq!(frame.data()[3], 0xAA);
        assert_eq!(frame.data()[4], 0xBB);
    }

    #[test]
    fn emcy_parse_roundtrip() {
        let node = NodeId::new(0x21).unwrap();
        let frame = build_emcy_frame(node, 0x2310, error_register::CURRENT, &[0x01, 0x02]);
        let msg = EmcyMessage::parse(&frame).unwrap();
        assert_eq!(msg.node, node);
        assert_eq!(msg.error_code, 0x2310);
        assert_eq!(msg.error_register, error_register::CURRENT);
        assert_eq!(msg.vendor_data, [0x01, 0x02, 0x00, 0x00, 0x00]);
        assert!(!msg.is_error_reset());

        // Not an EMCY COB-ID
        let hb = CanFrame::new(0x721, &[0x05]).unwrap();
        assert!(EmcyMessage::parse(&hb).is_none());
    }

    #[test]
    fn emcy_producer_set_clear() {
        let node = NodeId::new(1).unwrap();
        let mut emcy = EmcyProducer::new(node);
        assert_eq!(emcy.error_register(), 0);

        // Set error
        emcy.set_error(0x3000, error_register::VOLTAGE, &[]);
        assert_eq!(
            emcy.error_register(),
            error_register::VOLTAGE | error_register::GENERIC
        );
        let frame = emcy.take_pending().unwrap();
        assert_eq!(frame.raw_id(), 0x081);
        assert_eq!(frame.data()[0..2], [0x00, 0x30]); // 0x3000 LE

        // Clear specific bit
        emcy.clear_error(error_register::VOLTAGE);
        // Generic bit still set since error_register != 0 if we cleared voltage but generic remains
        // Actually: error_register = GENERIC (since we clear VOLTAGE but not GENERIC)
        assert_eq!(emcy.error_register(), error_register::GENERIC);
        assert!(emcy.take_pending().is_none()); // not zero yet

        // Clear all
        emcy.clear_all();
        assert_eq!(emcy.error_register(), 0);
        let frame = emcy.take_pending().unwrap();
        assert_eq!(frame.data()[0..2], [0x00, 0x00]); // NoError
    }

    #[test]
    fn emcy_burst_queues_multiple_frames() {
        let node = NodeId::new(1).unwrap();
        let mut emcy = EmcyProducer::new(node);

        emcy.set_error(0x2000, error_register::CURRENT, &[]);
        emcy.set_error(0x3000, error_register::VOLTAGE, &[]);
        emcy.set_error(0x4000, error_register::TEMPERATURE, &[]);

        // Drained in order, each carrying the register state at queue time.
        let f1 = emcy.take_pending().unwrap();
        assert_eq!(f1.data()[0..2], [0x00, 0x20]);
        let f2 = emcy.take_pending().unwrap();
        assert_eq!(f2.data()[0..2], [0x00, 0x30]);
        let f3 = emcy.take_pending().unwrap();
        assert_eq!(f3.data()[0..2], [0x00, 0x40]);
        assert_eq!(
            f3.data()[2],
            error_register::CURRENT
                | error_register::VOLTAGE
                | error_register::TEMPERATURE
                | error_register::GENERIC
        );
        assert!(emcy.take_pending().is_none());
    }

    #[test]
    fn emcy_overflow_drops_oldest() {
        let node = NodeId::new(1).unwrap();
        let mut emcy = EmcyProducer::new(node);

        for code in [0x1000u16, 0x2000, 0x3000, 0x4000, 0x5000] {
            emcy.set_error(code, error_register::GENERIC, &[]);
        }

        // Queue holds 4; the oldest (0x1000) was dropped.
        let f = emcy.take_pending().unwrap();
        assert_eq!(f.data()[0..2], [0x00, 0x20]);
        assert!(emcy.take_pending().is_some());
        assert!(emcy.take_pending().is_some());
        let last = emcy.take_pending().unwrap();
        assert_eq!(last.data()[0..2], [0x00, 0x50]);
        assert!(emcy.take_pending().is_none());
    }
}
