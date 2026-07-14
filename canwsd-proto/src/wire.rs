//! Binary wire format for CAN frames over WebSocket.
//!
//! Exactly one CAN frame per WebSocket binary message. Variable length,
//! little-endian:
//!
//! ```text
//! [0..4]  u32  id word (bit 31 = EFF, bit 30 = RTR, bit 29 = ERR,
//!              bits 0-28 = CAN ID) — matches the Linux socketCAN convention
//! [4]     u8   DLC (0-8, classic CAN)
//! [5..]   [u8] data, exactly DLC bytes
//! ```
//!
//! Total message size is `5 + DLC` bytes; decoding rejects any other length.
//! TCP/WebSocket already provide integrity and message delimiting — the DLC
//! is a structural sanity check and preserves the DLC of RTR frames.

/// Extended frame format flag (29-bit ID) in the id word.
pub const CAN_EFF_FLAG: u32 = 0x8000_0000;
/// Remote transmission request flag in the id word.
pub const CAN_RTR_FLAG: u32 = 0x4000_0000;
/// Error frame flag in the id word.
pub const CAN_ERR_FLAG: u32 = 0x2000_0000;
/// Valid ID bits of an extended frame.
pub const CAN_EFF_MASK: u32 = 0x1FFF_FFFF;
/// Valid ID bits of a standard frame.
pub const CAN_SFF_MASK: u32 = 0x0000_07FF;

/// Bytes preceding the data: id word + DLC.
pub const WIRE_HEADER_SIZE: usize = 5;
/// Largest possible encoded frame (DLC = 8).
pub const MAX_WIRE_FRAME_SIZE: usize = WIRE_HEADER_SIZE + 8;

/// A CAN frame as carried on the WebSocket, id word flags included.
///
/// This is transport-layer: it does not decide what flags a consumer accepts
/// (e.g. a CANopen client will reject EFF/RTR/ERR frames, a raw CAN bridge
/// forwards them).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WireFrame {
    id_word: u32,
    dlc: u8,
    data: [u8; 8],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// Shorter than the 5-byte header.
    TooShort { actual: usize },
    /// DLC byte above 8.
    BadDlc { dlc: u8 },
    /// Message length is not `5 + DLC`.
    LengthMismatch { expected: usize, actual: usize },
}

impl core::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::TooShort { actual } => write!(f, "message too short: {actual} bytes"),
            Self::BadDlc { dlc } => write!(f, "invalid DLC: {dlc}"),
            Self::LengthMismatch { expected, actual } => {
                write!(
                    f,
                    "length mismatch: expected {expected} bytes, got {actual}"
                )
            }
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for DecodeError {}

impl WireFrame {
    /// Create a frame from a raw id word and data. Returns `None` if data
    /// exceeds 8 bytes.
    pub fn new(id_word: u32, data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            return None;
        }
        let mut buf = [0u8; 8];
        buf[..data.len()].copy_from_slice(data);
        Some(Self {
            id_word,
            dlc: data.len() as u8,
            data: buf,
        })
    }

    /// Raw id word including EFF/RTR/ERR flags.
    pub const fn id_word(&self) -> u32 {
        self.id_word
    }

    /// CAN ID with flag bits masked off (29-bit for EFF, 11-bit otherwise).
    pub const fn id(&self) -> u32 {
        if self.is_extended() {
            self.id_word & CAN_EFF_MASK
        } else {
            self.id_word & CAN_SFF_MASK
        }
    }

    pub const fn is_extended(&self) -> bool {
        self.id_word & CAN_EFF_FLAG != 0
    }

    pub const fn is_rtr(&self) -> bool {
        self.id_word & CAN_RTR_FLAG != 0
    }

    pub const fn is_error(&self) -> bool {
        self.id_word & CAN_ERR_FLAG != 0
    }

    pub const fn dlc(&self) -> u8 {
        self.dlc
    }

    pub fn data(&self) -> &[u8] {
        &self.data[..self.dlc as usize]
    }

    /// Encode into a fixed buffer; the valid prefix is `.1` bytes long.
    pub fn encode(&self) -> ([u8; MAX_WIRE_FRAME_SIZE], usize) {
        let mut buf = [0u8; MAX_WIRE_FRAME_SIZE];
        buf[0..4].copy_from_slice(&self.id_word.to_le_bytes());
        buf[4] = self.dlc;
        buf[WIRE_HEADER_SIZE..WIRE_HEADER_SIZE + self.dlc as usize].copy_from_slice(self.data());
        (buf, WIRE_HEADER_SIZE + self.dlc as usize)
    }

    /// Decode one WebSocket binary message. The message must contain exactly
    /// one frame (`buf.len() == 5 + DLC`).
    pub fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        if buf.len() < WIRE_HEADER_SIZE {
            return Err(DecodeError::TooShort { actual: buf.len() });
        }
        let id_word = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        let dlc = buf[4];
        if dlc > 8 {
            return Err(DecodeError::BadDlc { dlc });
        }
        let expected = WIRE_HEADER_SIZE + dlc as usize;
        if buf.len() != expected {
            return Err(DecodeError::LengthMismatch {
                expected,
                actual: buf.len(),
            });
        }
        let mut data = [0u8; 8];
        data[..dlc as usize].copy_from_slice(&buf[WIRE_HEADER_SIZE..expected]);
        Ok(Self { id_word, dlc, data })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_standard_frame() {
        let frame = WireFrame::new(0x123, &[1, 2, 3]).unwrap();
        let (buf, len) = frame.encode();
        assert_eq!(len, 8);
        let decoded = WireFrame::decode(&buf[..len]).unwrap();
        assert_eq!(decoded, frame);
        assert_eq!(decoded.id(), 0x123);
        assert_eq!(decoded.data(), &[1, 2, 3]);
        assert!(!decoded.is_extended());
    }

    #[test]
    fn roundtrip_empty_frame() {
        let frame = WireFrame::new(0x7FF, &[]).unwrap();
        let (buf, len) = frame.encode();
        assert_eq!(len, WIRE_HEADER_SIZE);
        assert_eq!(WireFrame::decode(&buf[..len]).unwrap(), frame);
    }

    #[test]
    fn roundtrip_extended_frame() {
        let id_word = CAN_EFF_FLAG | 0x1234_5678;
        let frame = WireFrame::new(id_word, &[0xAA; 8]).unwrap();
        let (buf, len) = frame.encode();
        assert_eq!(len, MAX_WIRE_FRAME_SIZE);
        let decoded = WireFrame::decode(&buf[..len]).unwrap();
        assert!(decoded.is_extended());
        assert_eq!(decoded.id(), 0x1234_5678 & CAN_EFF_MASK);
    }

    #[test]
    fn new_rejects_oversized_payload() {
        assert_eq!(WireFrame::new(0x123, &[0; 9]), None);
    }

    #[test]
    fn decode_rejects_short_header() {
        assert_eq!(
            WireFrame::decode(&[0; 4]),
            Err(DecodeError::TooShort { actual: 4 })
        );
    }

    #[test]
    fn decode_rejects_bad_dlc() {
        let mut buf = [0u8; WIRE_HEADER_SIZE + 9];
        buf[4] = 9;
        assert_eq!(WireFrame::decode(&buf), Err(DecodeError::BadDlc { dlc: 9 }));
    }

    #[test]
    fn decode_rejects_length_mismatch() {
        // dlc says 3 but message carries 8 data bytes (e.g. old fixed-size format)
        let mut buf = [0u8; 13];
        buf[4] = 3;
        assert_eq!(
            WireFrame::decode(&buf),
            Err(DecodeError::LengthMismatch {
                expected: 8,
                actual: 13
            })
        );
        // truncated data
        let mut buf = [0u8; 6];
        buf[4] = 3;
        assert_eq!(
            WireFrame::decode(&buf),
            Err(DecodeError::LengthMismatch {
                expected: 8,
                actual: 6
            })
        );
    }
}
