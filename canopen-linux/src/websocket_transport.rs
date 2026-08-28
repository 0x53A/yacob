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
use std::time::{Duration, Instant};
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

/// Defaults for the bounded WebSocket transmit queue.
///
/// A frame that cannot complete its WebSocket write within
/// [`Self::max_transmit_age`] is abandoned and the transport is disconnected.
/// This prevents a cancelled, internally buffered write from escaping later.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct WebSocketTransportConfig {
    /// Maximum number of frames waiting to be written to the WebSocket.
    pub transmit_queue_capacity: usize,
    /// Maximum time from [`embedded_can::nb::Can::transmit`] accepting a frame
    /// until the WebSocket write completes.
    pub max_transmit_age: Duration,
}

impl Default for WebSocketTransportConfig {
    fn default() -> Self {
        Self {
            transmit_queue_capacity: 32,
            max_transmit_age: Duration::from_millis(250),
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct QueuedFrame {
    frame: CanFrame,
    enqueued_at: Instant,
}

/// Raw CAN-over-WebSocket transport.
///
/// Connect to a `canwsd` interface endpoint, for example:
///
/// ```ignore
/// let mut can = WebSocketTransport::connect("ws://127.0.0.1:8080/api/networks/vcan0")?;
/// ```
pub struct WebSocketTransport {
    tx: tokio::sync::mpsc::Sender<QueuedFrame>,
    rx: mpsc::Receiver<CanFrame>,
    _thread: thread::JoinHandle<()>,
}

#[derive(Debug)]
pub enum WebSocketTransportError {
    Runtime(std::io::Error),
    Connect(String),
    InvalidConfig(&'static str),
}

impl std::fmt::Display for WebSocketTransportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Runtime(e) => write!(f, "failed to create tokio runtime: {e}"),
            Self::Connect(e) => write!(f, "websocket connection failed: {e}"),
            Self::InvalidConfig(e) => write!(f, "invalid websocket transport config: {e}"),
        }
    }
}

impl std::error::Error for WebSocketTransportError {}

impl WebSocketTransport {
    /// Connect to a `canwsd` websocket endpoint with bounded, short-lived
    /// transmit buffering.
    pub fn connect(url: impl Into<String>) -> Result<Self, WebSocketTransportError> {
        Self::connect_with_config(url, WebSocketTransportConfig::default())
    }

    /// Connect with explicit transmit queue limits.
    pub fn connect_with_config(
        url: impl Into<String>,
        config: WebSocketTransportConfig,
    ) -> Result<Self, WebSocketTransportError> {
        if config.transmit_queue_capacity == 0 {
            return Err(WebSocketTransportError::InvalidConfig(
                "transmit_queue_capacity must be nonzero",
            ));
        }
        if config.max_transmit_age.is_zero() {
            return Err(WebSocketTransportError::InvalidConfig(
                "max_transmit_age must be nonzero",
            ));
        }

        let url = url.into();
        let (tx, mut tx_rx) =
            tokio::sync::mpsc::channel::<QueuedFrame>(config.transmit_queue_capacity);
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
                        queued = tx_rx.recv() => {
                            let Some(queued) = queued else {
                                break;
                            };
                            match send_before_expiry(&mut ws_tx, queued, config.max_transmit_age).await {
                                SendOutcome::Sent | SendOutcome::Expired => {}
                                SendOutcome::Failed | SendOutcome::TimedOut => break,
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
        enqueue_frame(&self.tx, frame)
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        match self.rx.try_recv() {
            Ok(frame) => Ok(frame),
            Err(mpsc::TryRecvError::Empty) => Err(nb::Error::WouldBlock),
            Err(mpsc::TryRecvError::Disconnected) => Err(nb::Error::Other(CanError::BusError)),
        }
    }
}

fn enqueue_frame(
    sender: &tokio::sync::mpsc::Sender<QueuedFrame>,
    frame: &CanFrame,
) -> nb::Result<Option<CanFrame>, CanError> {
    let queued = QueuedFrame {
        frame: *frame,
        enqueued_at: Instant::now(),
    };
    match sender.try_send(queued) {
        Ok(()) => Ok(None),
        Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => Err(nb::Error::WouldBlock),
        Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => {
            Err(nb::Error::Other(CanError::BusError))
        }
    }
}

#[derive(Debug, Eq, PartialEq)]
enum SendOutcome {
    Sent,
    Expired,
    Failed,
    TimedOut,
}

/// Write a frame only while its transmit lifetime remains. If a WebSocket
/// write itself times out, the caller must drop the socket: a cancelled sink
/// flush may still hold bytes internally, so continuing could release a stale
/// frame during a later write.
async fn send_before_expiry<S>(
    sink: &mut S,
    queued: QueuedFrame,
    max_transmit_age: Duration,
) -> SendOutcome
where
    S: futures_util::Sink<Message> + Unpin,
{
    let elapsed = queued.enqueued_at.elapsed();
    let Some(remaining) = max_transmit_age.checked_sub(elapsed) else {
        return SendOutcome::Expired;
    };

    let Some(wire) = WireFrame::new(queued.frame.raw_id() as u32, queued.frame.data()) else {
        return SendOutcome::Failed;
    };
    let (buf, len) = wire.encode();
    let message = Message::Binary(buf[..len].to_vec().into());

    match tokio::time::timeout(remaining, sink.send(message)).await {
        Ok(Ok(())) => SendOutcome::Sent,
        Ok(Err(_)) => SendOutcome::Failed,
        Err(_) => SendOutcome::TimedOut,
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
    use core::pin::Pin;
    use core::task::{Context, Poll};

    struct StalledSink;

    impl futures_util::Sink<Message> for StalledSink {
        type Error = ();

        fn poll_ready(
            self: Pin<&mut Self>,
            _cx: &mut Context<'_>,
        ) -> Poll<Result<(), Self::Error>> {
            Poll::Pending
        }

        fn start_send(self: Pin<&mut Self>, _item: Message) -> Result<(), Self::Error> {
            Ok(())
        }

        fn poll_flush(
            self: Pin<&mut Self>,
            _cx: &mut Context<'_>,
        ) -> Poll<Result<(), Self::Error>> {
            Poll::Pending
        }

        fn poll_close(
            self: Pin<&mut Self>,
            _cx: &mut Context<'_>,
        ) -> Poll<Result<(), Self::Error>> {
            Poll::Ready(Ok(()))
        }
    }

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
        let (buf, len) = WireFrame::new(CAN_EFF_FLAG | 0x123, &[1]).unwrap().encode();
        assert!(decode_frame(&buf[..len]).is_err());
    }

    #[test]
    fn full_transmit_queue_reports_would_block() {
        let (sender, _receiver) = tokio::sync::mpsc::channel(1);
        let frame = CanFrame::new(0x123, &[1]).unwrap();

        assert!(enqueue_frame(&sender, &frame).is_ok());
        assert!(matches!(
            enqueue_frame(&sender, &frame),
            Err(nb::Error::WouldBlock)
        ));
    }

    #[tokio::test]
    async fn drops_a_frame_that_expired_in_the_queue() {
        let frame = CanFrame::new(0x123, &[1]).unwrap();
        let queued = QueuedFrame {
            frame,
            enqueued_at: Instant::now() - Duration::from_millis(2),
        };
        let mut sink = futures_util::sink::drain();

        assert_eq!(
            send_before_expiry(&mut sink, queued, Duration::from_millis(1)).await,
            SendOutcome::Expired
        );
    }

    #[tokio::test]
    async fn abandons_a_socket_whose_write_outlives_the_frame() {
        let frame = CanFrame::new(0x123, &[1]).unwrap();
        let queued = QueuedFrame {
            frame,
            enqueued_at: Instant::now(),
        };

        assert_eq!(
            send_before_expiry(&mut StalledSink, queued, Duration::from_millis(1)).await,
            SendOutcome::TimedOut
        );
    }
}
