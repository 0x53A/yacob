//! SLCAN (Serial Line CAN) protocol support.
//!
//! The pure Lawicel/SLCAN line codec lives in [`logic`]. The synchronous
//! [`SerialPort`]-based transport lives in [`serial`] and is suitable for
//! native serial ports, embedded UART adapters, or tests. Browser Web Serial
//! uses the same line codec directly because its API is promise-based.

pub mod logic;
pub mod serial;

pub use logic::{encode_slcan_frame, has_slcan_frame, parse_slcan_frame, SlcanBitrate};
pub use serial::{SerialPort, SlcanTransport};

#[cfg(feature = "std")]
pub use serial::IoPort;
