//! Browser transports for canopen-rs.
//!
//! The transports in this crate are single-threaded and event-driven. They use
//! browser WebSocket callbacks and Web Serial promises, then expose received CAN
//! frames through a small pollable queue for UI/event-loop integration.

#[cfg(target_arch = "wasm32")]
pub mod transport;

#[cfg(target_arch = "wasm32")]
pub use transport::{
    canwsd::{fetch_canwsd_networks, CanwsdTransport},
    slcan::SlcanTransport,
    CanEvent, CanTransport,
};
