//! Shared definition of the canwsd public interface.
//!
//! Any server exposing CAN networks over WebSocket (the Linux `canwsd` daemon,
//! embedded devices hosting a virtual bus, ...) and any client (browser via
//! `canopen-web`, native tools) should implement this interface:
//!
//! - REST: `GET /api/networks` → JSON array of [`NetworkInfo`]
//! - WS: `GET /api/networks/<name>?filter=id:mask,...` → WebSocket upgrade
//! - Binary WS messages: exactly one CAN frame per WebSocket message, encoded
//!   with the variable-length format in [`wire`]
//! - Text WS messages: client → server carries JSON commands
//!   ([`ClientCommand`]); server → client carries JSON status messages
//!   ([`ServerStatus`])
//! - CAN error frames (local controller error reports) are only delivered to
//!   clients that opt in with [`ERRORS_QUERY_PARAM`] (`?errors=1`)
//! - When the underlying bus becomes unusable, the server sends one
//!   [`ServerStatus::BusError`] status message and then closes the WebSocket
//!   with close code [`close_code::BUS_ERROR`]; reconnecting is the client's
//!   responsibility (a connect attempt while the bus is down is answered with
//!   HTTP 503)
//!
//! This crate is `no_std` by default (disable default features); it contains
//! the interface only, no transport implementation.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod filter;
pub mod wire;

pub use filter::CanFilter;
pub use wire::{DecodeError, WireFrame, MAX_WIRE_FRAME_SIZE};

/// REST path listing available networks.
pub const NETWORKS_PATH: &str = "/api/networks";

/// WebSocket route pattern (axum-style `{name}` placeholder).
pub const NETWORK_WS_ROUTE: &str = "/api/networks/{name}";

/// Query parameter carrying the initial receive filter (`id:mask,id:mask,...`).
pub const FILTER_QUERY_PARAM: &str = "filter";

/// Query parameter opting in to CAN error frames (`?errors=1` or
/// `?errors=true`). Fixed for the lifetime of the connection.
pub const ERRORS_QUERY_PARAM: &str = "errors";

/// Application WebSocket close codes (RFC 6455 reserves 4000-4999 for these).
pub mod close_code {
    /// The underlying CAN bus became unusable (interface down, device gone).
    /// Preceded by a [`ServerStatus::BusError`](crate::ServerStatus) status
    /// message.
    pub const BUS_ERROR: u16 = 4000;
}

/// Network availability as reported by `GET /api/networks`.
#[cfg(feature = "serde")]
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NetworkStatus {
    Available,
    Unavailable,
}

/// One entry of the `GET /api/networks` response.
#[cfg(all(feature = "serde", feature = "alloc"))]
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct NetworkInfo {
    pub name: alloc::string::String,
    pub interface: alloc::string::String,
    /// Bitrate in bit/s. `0` means unknown or not applicable.
    pub bitrate: u32,
    pub status: NetworkStatus,
    /// Empty when there is no current error.
    pub error: alloc::string::String,
}

/// Borrowed variant of [`NetworkInfo`] for serialization without allocation
/// (e.g. `serde-json-core` on embedded servers).
#[cfg(feature = "serde")]
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
pub struct NetworkInfoRef<'a> {
    pub name: &'a str,
    pub interface: &'a str,
    pub bitrate: u32,
    pub status: NetworkStatus,
    pub error: &'a str,
}

/// Server → client status message, sent as a JSON text frame.
#[cfg(all(feature = "serde", feature = "alloc"))]
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "status")]
pub enum ServerStatus {
    /// The bus became unusable; the server closes the connection with
    /// [`close_code::BUS_ERROR`] right after this message.
    #[serde(rename = "bus_error")]
    BusError { error: alloc::string::String },
    /// The server-side receive buffer for this client overflowed and was
    /// cleared completely: `dropped` frames were lost, everything after this
    /// message is a fresh start.
    #[serde(rename = "overflow")]
    Overflow { dropped: u64 },
}

/// Borrowed variant of [`ServerStatus`] for serialization without allocation.
#[cfg(feature = "serde")]
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
#[serde(tag = "status")]
pub enum ServerStatusRef<'a> {
    #[serde(rename = "bus_error")]
    BusError { error: &'a str },
    #[serde(rename = "overflow")]
    Overflow { dropped: u64 },
}
