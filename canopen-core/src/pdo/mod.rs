pub mod engine;

pub use engine::{
    PdoConfigSource, PdoMapping, PdoNumber, RpdoConfig, RpdoEngine, TpdoConfig, TpdoEngine,
    TransmissionType, PDO_MAX_MAPPINGS, PDO_MAX_PAYLOAD_BITS,
};

/// Error returned by EDS-generated remote PDO decoders.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PdoDecodeError {
    WrongCobId { expected: u16, actual: u16 },
    PayloadTooShort { expected: usize, actual: usize },
}

impl core::fmt::Display for PdoDecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::WrongCobId { expected, actual } => {
                write!(
                    f,
                    "expected PDO COB-ID 0x{expected:03X}, got 0x{actual:03X}"
                )
            }
            Self::PayloadTooShort { expected, actual } => {
                write!(f, "expected at least {expected} PDO bytes, got {actual}")
            }
        }
    }
}

/// Copy CANopen PDO bits least-significant-bit first.
///
/// This is public for proc-macro-generated remote PDO codecs. The caller must
/// ensure both bit ranges fit their respective slices.
#[doc(hidden)]
pub fn copy_pdo_bits(
    src: &[u8],
    src_bit_offset: usize,
    dst: &mut [u8],
    dst_bit_offset: usize,
    bit_len: usize,
) {
    if src_bit_offset % 8 == 0 && dst_bit_offset % 8 == 0 && bit_len % 8 == 0 {
        let byte_len = bit_len / 8;
        let src_start = src_bit_offset / 8;
        let dst_start = dst_bit_offset / 8;
        dst[dst_start..dst_start + byte_len].copy_from_slice(&src[src_start..src_start + byte_len]);
        return;
    }

    for bit in 0..bit_len {
        let src_bit = src_bit_offset + bit;
        let dst_bit = dst_bit_offset + bit;
        let value = (src[src_bit / 8] >> (src_bit % 8)) & 1;
        if value != 0 {
            dst[dst_bit / 8] |= 1 << (dst_bit % 8);
        } else {
            dst[dst_bit / 8] &= !(1 << (dst_bit % 8));
        }
    }
}

/// Helper for generated code: create an empty Vec (avoids generated code importing heapless).
pub fn heapless_vec_new<T, const N: usize>() -> heapless::Vec<T, N> {
    heapless::Vec::new()
}

/// Helper for generated code: push to a Vec.
pub fn heapless_vec_push<T, const N: usize>(v: &mut heapless::Vec<T, N>, val: T) -> Result<(), T> {
    v.push(val)
}
