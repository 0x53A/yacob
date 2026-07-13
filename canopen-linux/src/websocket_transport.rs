//! WebSocket CAN transport for `canwsd`.
//!
//! This connects to the raw CAN WebSocket interface defined by the
//! `canwsd-proto` crate: one variable-length CAN frame per binary message.

use canopen_core::transport::{CanError, CanFrame};
use canwsd_proto::wire::{CAN_EFF_FLAG, CAN_ERR_FLAG, CAN_RTR_FLAG};
use canwsd_proto::WireFrame;
use futures_util::{SinkExt, StreamExt};
use std::sync::mpsc;
use std::thread;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

/// Raw CAN-over-WebSocket transport.
///
/// Connect to a `canwsd` interface endpoint, for example:
///
/// ```ignore
/// let mut can = WebSocketTransport::connect("ws://127.0.0.1:8080/api/networks/vcan0")?;
/// ```
pub struct WebSocketTransport {
    tx: tokio::sync::mpsc::UnboundedSender<CanFrame>,
    rx: mpsc::Receiver<CanFrame>,
    _thread: thread::JoinHandle<()>,
}

#[derive(Debug)]
pub enum WebSocketTransportError {
    Runtime(std::io::Error),
    Connect(String),
}

impl std::fmt::Display for WebSocketTransportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Runtime(e) => write!(f, "failed to create tokio runtime: {e}"),
            Self::Connect(e) => write!(f, "websocket connection failed: {e}"),
        }
    }
}

impl std::error::Error for WebSocketTransportError {}

impl WebSocketTransport {
    /// Connect to a `canwsd` websocket endpoint.
    pub fn connect(url: impl Into<String>) -> Result<Self, WebSocketTransportError> {
        let url = url.into();
        let (tx, mut tx_rx) = tokio::sync::mpsc::unbounded_channel::<CanFrame>();
        let (rx_tx, rx) = mpsc::channel::<CanFrame>();
        let runtime = tokio::runtime::Runtime::new().map_err(WebSocketTransportError::Runtime)?;

        let (ready_tx, ready_rx) = mpsc::sync_channel(1);
        let thread = thread::spawn(move || {
            runtime.block_on(async move {
                let (mut ws_tx, mut ws_rx) = match connect_async(&url).await {
                    Ok((stream, _)) => {
                        let _ = ready_tx.send(Ok(()));
                        stream.split()
                    }
                    Err(e) => {
                        let _ = ready_tx.send(Err(e.to_string()));
                        return;
                    }
                };

                loop {
                    tokio::select! {
                        Some(frame) = tx_rx.recv() => {
                            let Some(wire) = WireFrame::new(frame.raw_id() as u32, frame.data()) else {
                                continue;
                            };
                            let (buf, len) = wire.encode();
                            if ws_tx.send(Message::Binary(buf[..len].to_vec().into())).await.is_err() {
                                break;
                            }
                        }
                        msg = ws_rx.next() => {
                            match msg {
                                Some(Ok(Message::Binary(data))) => {
                                    if let Ok(frame) = decode_frame(&data) {
                                        let _ = rx_tx.send(frame);
                                    }
                                }
                                Some(Ok(Message::Close(_))) | None => break,
                                Some(Ok(_)) => {}
                                Some(Err(_)) => break,
                            }
                        }
                    }
                }
            });
        });

        match ready_rx.recv() {
            Ok(Ok(())) => Ok(Self {
                tx,
                rx,
                _thread: thread,
            }),
            Ok(Err(e)) => Err(WebSocketTransportError::Connect(e)),
            Err(e) => Err(WebSocketTransportError::Connect(e.to_string())),
        }
    }
}

impl embedded_can::nb::Can for WebSocketTransport {
    type Frame = CanFrame;
    type Error = CanError;

    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        self.tx
            .send(*frame)
            .map_err(|_| nb::Error::Other(CanError::BusError))?;
        Ok(None)
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        match self.rx.try_recv() {
            Ok(frame) => Ok(frame),
            Err(mpsc::TryRecvError::Empty) => Err(nb::Error::WouldBlock),
            Err(mpsc::TryRecvError::Disconnected) => Err(nb::Error::Other(CanError::BusError)),
        }
    }
}

/// Decode one WebSocket message into a CANopen frame. Frames with EFF/RTR/ERR
/// flags are valid on the wire but have no meaning for CANopen and are
/// rejected here.
fn decode_frame(buf: &[u8]) -> Result<CanFrame, ()> {
    let wire = WireFrame::decode(buf).map_err(|_| ())?;
    if wire.id_word() & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG) != 0 {
        return Err(());
    }
    CanFrame::new(wire.id_word() as u16, wire.data()).ok_or(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_frame() {
        let (buf, len) = WireFrame::new(0x123, &[1, 2, 3]).unwrap().encode();
        assert_eq!(decode_frame(&buf[..len]).unwrap().raw_id(), 0x123);
        assert_eq!(decode_frame(&buf[..len]).unwrap().data(), &[1, 2, 3]);
    }

    #[test]
    fn rejects_non_canopen_wire_frames() {
        // truncated message
        assert!(decode_frame(&[0; 4]).is_err());

        // extended-id frame: valid on the wire, not valid for CANopen
        let (buf, len) = WireFrame::new(CAN_EFF_FLAG | 0x123, &[1])
            .unwrap()
            .encode();
        assert!(decode_frame(&buf[..len]).is_err());
    }
}
