/// CANopen data type codes (CiA 301).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum DataType {
    Boolean = 0x0001,
    I8 = 0x0002,
    I16 = 0x0003,
    I32 = 0x0004,
    U8 = 0x0005,
    U16 = 0x0006,
    U32 = 0x0007,
    Real32 = 0x0008,
    VisibleString = 0x0009,
    OctetString = 0x000A,
    Domain = 0x000F,
    Real64 = 0x0011,
    I64 = 0x0015,
    U64 = 0x001B,
}

impl DataType {
    /// Size in bytes for fixed-size types. Returns None for variable-length types.
    pub const fn size(self) -> Option<usize> {
        match self {
            Self::Boolean | Self::U8 | Self::I8 => Some(1),
            Self::U16 | Self::I16 => Some(2),
            Self::U32 | Self::I32 | Self::Real32 => Some(4),
            Self::U64 | Self::I64 | Self::Real64 => Some(8),
            Self::VisibleString | Self::OctetString | Self::Domain => None,
        }
    }

    /// Parse from the u16 code used in EDS files.
    pub const fn from_code(code: u16) -> Option<Self> {
        match code {
            0x0001 => Some(Self::Boolean),
            0x0002 => Some(Self::I8),
            0x0003 => Some(Self::I16),
            0x0004 => Some(Self::I32),
            0x0005 => Some(Self::U8),
            0x0006 => Some(Self::U16),
            0x0007 => Some(Self::U32),
            0x0008 => Some(Self::Real32),
            0x0009 => Some(Self::VisibleString),
            0x000A => Some(Self::OctetString),
            0x000F => Some(Self::Domain),
            0x0011 => Some(Self::Real64),
            0x0015 => Some(Self::I64),
            0x001B => Some(Self::U64),
            _ => None,
        }
    }
}
