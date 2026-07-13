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
//! - Text WS messages: JSON commands ([`ClientCommand`]) and server status
//!   messages
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

/// One entry of the `GET /api/networks` response.
#[cfg(all(feature = "serde", feature = "alloc"))]
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct NetworkInfo {
    pub name: alloc::string::String,
}

/// Borrowed variant of [`NetworkInfo`] for serialization without allocation
/// (e.g. `serde-json-core` on embedded servers).
#[cfg(feature = "serde")]
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
pub struct NetworkInfoRef<'a> {
    pub name: &'a str,
}
