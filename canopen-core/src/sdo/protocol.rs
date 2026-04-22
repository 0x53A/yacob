/// SDO abort codes (CiA 301 Table 22).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum AbortCode {
    ToggleBitNotAlternated = 0x0503_0000,
    SdoProtocolTimeout = 0x0504_0000,
    InvalidCommandSpecifier = 0x0504_0001,
    InvalidBlockSize = 0x0504_0002,
    InvalidSequenceNumber = 0x0504_0003,
    OutOfMemory = 0x0504_0005,
    UnsupportedAccess = 0x0601_0000,
    WriteOnlyObject = 0x0601_0001,
    ReadOnlyObject = 0x0601_0002,
    ObjectNotFound = 0x0602_0000,
    ObjectCannotBeMapped = 0x0604_0041,
    PdoLengthExceeded = 0x0604_0042,
    ParameterIncompatibility = 0x0604_0043,
    SubindexNotFound = 0x0609_0011,
    ValueRangeExceeded = 0x0609_0030,
    ValueTooHigh = 0x0609_0031,
    ValueTooLow = 0x0609_0032,
    DataTypeMismatch = 0x0609_0043,
    GeneralError = 0x0800_0000,
    DataTransferError = 0x0800_0020,
    DataTransferLocalControl = 0x0800_0021,
    DataTransferDeviceState = 0x0800_0022,
}

impl AbortCode {
    pub const fn to_u32(self) -> u32 {
        self as u32
    }

    pub fn to_le_bytes(self) -> [u8; 4] {
        (self as u32).to_le_bytes()
    }
}

/// Client Command Specifier (CCS) - upper 3 bits of byte 0 in client→server SDO frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ccs {
    /// Initiate download (write) request
    InitiateDownload = 1,
    /// Download segment
    DownloadSegment = 0,
    /// Initiate upload (read) request
    InitiateUpload = 2,
    /// Upload segment
    UploadSegment = 3,
    /// Abort transfer
    AbortTransfer = 4,
    /// Block upload (client initiates)
    BlockUpload = 5,
    /// Block download (client initiates)
    BlockDownload = 6,
}

/// Server Command Specifier (SCS) - upper 3 bits of byte 0 in server→client SDO frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scs {
    /// Initiate download response
    InitiateDownloadResponse = 3,
    /// Download segment response
    DownloadSegmentResponse = 1,
    /// Initiate upload response
    InitiateUploadResponse = 2,
    /// Upload segment response
    UploadSegmentResponse = 0,
    /// Abort transfer
    AbortTransfer = 4,
}

/// Extract the command specifier (upper 3 bits) from SDO byte 0.
pub const fn command_specifier(byte0: u8) -> u8 {
    (byte0 >> 5) & 0x07
}

/// Encode an SDO abort frame.
pub fn encode_abort(index: u16, subindex: u8, code: AbortCode) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0] = (Scs::AbortTransfer as u8) << 5; // 0x80
    data[1] = (index & 0xFF) as u8;
    data[2] = (index >> 8) as u8;
    data[3] = subindex;
    let abort_bytes = code.to_le_bytes();
    data[4..8].copy_from_slice(&abort_bytes);
    data
}

/// Encode an initiate upload response (expedited, ≤4 bytes).
pub fn encode_upload_response_expedited(
    index: u16,
    subindex: u8,
    value: &[u8],
) -> Option<[u8; 8]> {
    if value.len() > 4 {
        return None;
    }
    let n = 4 - value.len();
    let mut data = [0u8; 8];
    // SCS=2, n=unused bytes, e=1 (expedited), s=1 (size indicated)
    data[0] = (Scs::InitiateUploadResponse as u8) << 5 | (n as u8) << 2 | 0x02 | 0x01;
    data[1] = (index & 0xFF) as u8;
    data[2] = (index >> 8) as u8;
    data[3] = subindex;
    data[4..4 + value.len()].copy_from_slice(value);
    Some(data)
}

/// Encode an initiate upload response for segmented transfer (size indicated).
pub fn encode_upload_response_segmented(
    index: u16,
    subindex: u8,
    total_size: u32,
) -> [u8; 8] {
    let mut data = [0u8; 8];
    // SCS=2, e=0, s=1 (size indicated)
    data[0] = (Scs::InitiateUploadResponse as u8) << 5 | 0x01;
    data[1] = (index & 0xFF) as u8;
    data[2] = (index >> 8) as u8;
    data[3] = subindex;
    data[4..8].copy_from_slice(&total_size.to_le_bytes());
    data
}

/// Encode an upload segment response.
pub fn encode_upload_segment_response(
    toggle: bool,
    segment_data: &[u8],
    last: bool,
) -> [u8; 8] {
    let n = 7 - segment_data.len();
    let mut data = [0u8; 8];
    data[0] = (Scs::UploadSegmentResponse as u8) << 5
        | if toggle { 0x10 } else { 0 }
        | (n as u8) << 1
        | if last { 0x01 } else { 0 };
    data[1..1 + segment_data.len()].copy_from_slice(segment_data);
    data
}

/// Encode an initiate download response.
pub fn encode_download_response(index: u16, subindex: u8) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0] = (Scs::InitiateDownloadResponse as u8) << 5;
    data[1] = (index & 0xFF) as u8;
    data[2] = (index >> 8) as u8;
    data[3] = subindex;
    data
}

/// Encode a download segment response.
pub fn encode_download_segment_response(toggle: bool) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0] = (Scs::DownloadSegmentResponse as u8) << 5 | if toggle { 0x10 } else { 0 };
    data
}

/// Parse index and subindex from SDO frame bytes 1-3.
pub const fn parse_index_sub(data: &[u8; 8]) -> (u16, u8) {
    let index = data[1] as u16 | (data[2] as u16) << 8;
    let subindex = data[3];
    (index, subindex)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn abort_frame_encoding() {
        let data = encode_abort(0x1000, 0, AbortCode::ObjectNotFound);
        assert_eq!(data[0], 0x80); // CCS/SCS = 4
        assert_eq!(data[1], 0x00); // index low
        assert_eq!(data[2], 0x10); // index high
        assert_eq!(data[3], 0x00); // subindex
        let code = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        assert_eq!(code, 0x0602_0000);
    }

    #[test]
    fn expedited_upload_response() {
        let data =
            encode_upload_response_expedited(0x1000, 0, &[0x91, 0x01, 0x00, 0x00]).unwrap();
        let cs = command_specifier(data[0]);
        assert_eq!(cs, Scs::InitiateUploadResponse as u8);
        // e=1, s=1, n=0 (4 bytes used)
        assert_eq!(data[0] & 0x03, 0x03); // e=1, s=1
        assert_eq!((data[0] >> 2) & 0x03, 0); // n=0
        assert_eq!(data[4], 0x91);
    }

    #[test]
    fn segmented_upload_response() {
        let data = encode_upload_response_segmented(0x2000, 1, 100);
        let cs = command_specifier(data[0]);
        assert_eq!(cs, Scs::InitiateUploadResponse as u8);
        assert_eq!(data[0] & 0x02, 0); // e=0
        assert_eq!(data[0] & 0x01, 1); // s=1
        let size = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        assert_eq!(size, 100);
    }
}
