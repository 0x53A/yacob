//! High-level SDO client helpers for Linux.
//!
//! These wrap the low-level `SdoClient` state machine into blocking
//! read/write calls, suitable for test harnesses and CLI tools.

use canopen_core::cobid::{CobId, NodeId};
use canopen_core::sdo::client::{SdoClient, SdoClientResult};
use canopen_core::sdo::AbortCode;
use canopen_core::transport::{CanFrame, Transport};
use std::time::{Duration, Instant};

/// Error from a high-level SDO operation.
#[derive(Debug)]
pub enum SdoError {
    Aborted(AbortCode),
    Timeout,
    ProtocolError,
    TransportError,
}

impl std::fmt::Display for SdoError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Aborted(code) => write!(f, "SDO aborted: {:?}", code),
            Self::Timeout => write!(f, "SDO timeout"),
            Self::ProtocolError => write!(f, "SDO protocol error"),
            Self::TransportError => write!(f, "CAN transport error"),
        }
    }
}

impl std::error::Error for SdoError {}

/// Perform a blocking SDO upload (read) from a remote node.
///
/// Returns the data read from the object dictionary entry.
pub fn sdo_upload(
    transport: &mut impl Transport,
    target: NodeId,
    index: u16,
    subindex: u8,
    timeout: Duration,
) -> Result<Vec<u8>, SdoError> {
    let mut client = SdoClient::new(target);
    let req = client.start_upload(index, subindex);
    transport
        .send(&req)
        .map_err(|_| SdoError::TransportError)?;

    let response_cob = CobId::sdo_tx(target).raw();
    let deadline = Instant::now() + timeout;

    loop {
        if Instant::now() > deadline {
            return Err(SdoError::Timeout);
        }

        if let Some(frame) = transport.recv() {
            if frame.id() == response_cob && frame.dlc() == 8 {
                let data: [u8; 8] = frame.data().try_into().unwrap();
                match client.process_response(&data) {
                    SdoClientResult::UploadComplete { data_len } => {
                        return Ok(client.data()[..data_len].to_vec());
                    }
                    SdoClientResult::SendNext(next) => {
                        transport
                            .send(&next)
                            .map_err(|_| SdoError::TransportError)?;
                    }
                    SdoClientResult::Aborted(code) => return Err(SdoError::Aborted(code)),
                    SdoClientResult::DownloadComplete | SdoClientResult::Error => {
                        return Err(SdoError::ProtocolError)
                    }
                }
            }
        }

        std::thread::sleep(Duration::from_micros(100));
    }
}

/// Perform a blocking SDO download (write) to a remote node.
pub fn sdo_download(
    transport: &mut impl Transport,
    target: NodeId,
    index: u16,
    subindex: u8,
    data: &[u8],
    timeout: Duration,
) -> Result<(), SdoError> {
    let mut client = SdoClient::new(target);
    let req = client.start_download(index, subindex, data);
    transport
        .send(&req)
        .map_err(|_| SdoError::TransportError)?;

    let response_cob = CobId::sdo_tx(target).raw();
    let deadline = Instant::now() + timeout;

    loop {
        if Instant::now() > deadline {
            return Err(SdoError::Timeout);
        }

        if let Some(frame) = transport.recv() {
            if frame.id() == response_cob && frame.dlc() == 8 {
                let resp: [u8; 8] = frame.data().try_into().unwrap();
                match client.process_response(&resp) {
                    SdoClientResult::DownloadComplete => return Ok(()),
                    SdoClientResult::SendNext(next) => {
                        transport
                            .send(&next)
                            .map_err(|_| SdoError::TransportError)?;
                    }
                    SdoClientResult::Aborted(code) => return Err(SdoError::Aborted(code)),
                    SdoClientResult::UploadComplete { .. } | SdoClientResult::Error => {
                        return Err(SdoError::ProtocolError)
                    }
                }
            }
        }

        std::thread::sleep(Duration::from_micros(100));
    }
}

/// Send an NMT command to a node (or broadcast with node_id=0).
pub fn nmt_command(
    transport: &mut impl Transport,
    command: u8,
    target_node: u8,
) -> Result<(), SdoError> {
    let frame = CanFrame::new(0x000, &[command, target_node]).unwrap();
    transport
        .send(&frame)
        .map_err(|_| SdoError::TransportError)
}

/// Wait for a heartbeat from a specific node. Returns the NMT state byte.
pub fn wait_heartbeat(
    transport: &mut impl Transport,
    target: NodeId,
    timeout: Duration,
) -> Result<u8, SdoError> {
    let hb_cob = CobId::heartbeat(target).raw();
    let deadline = Instant::now() + timeout;

    loop {
        if Instant::now() > deadline {
            return Err(SdoError::Timeout);
        }
        if let Some(frame) = transport.recv() {
            if frame.id() == hb_cob && frame.dlc() >= 1 {
                return Ok(frame.data()[0]);
            }
        }
        std::thread::sleep(Duration::from_micros(100));
    }
}
