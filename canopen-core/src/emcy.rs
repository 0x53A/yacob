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
