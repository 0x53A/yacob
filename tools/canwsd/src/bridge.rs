use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use axum::extract::ws::{self, CloseFrame, WebSocket};
use futures_util::{SinkExt, StreamExt};
use socketcan::tokio::CanSocket;
use socketcan::{
    CanFrame, CanRemoteFrame, EmbeddedFrame, ExtendedId, Frame, Id, SocketOptions, StandardId,
};
use tokio::sync::mpsc;
use tokio::time::{Instant, MissedTickBehavior};

use canwsd_proto::filter::ClientCommand;
use canwsd_proto::{CanFilter, ServerStatusRef, WireFrame, close_code};

/// Per-client receive buffer between the CAN socket and the WebSocket:
/// 16384 frames × 16 B = 256 KiB, roughly 3 s of a saturated 500 kbps bus.
/// It absorbs short WS delivery hiccups; if it fills anyway the whole buffer
/// is cleared and the client is told how much it lost (ServerStatus::Overflow)
/// — a client that far behind wants a fresh start, not a stale replay.
const FRAME_BUFFER: usize = 16384;
/// CAN_RAW sockets accept at most this many kernel filters. The kernel filter
/// is an optimization only — every failure path degrades to accept-all plus
/// userspace filtering, never to over-filtering.
const MAX_KERNEL_FILTERS: usize = 512;
/// WS keepalive: ping cadence and how long a client may stay silent (no pong,
/// no traffic) before the connection is considered half-open and dropped.
const PING_INTERVAL: Duration = Duration::from_secs(10);
const CLIENT_TIMEOUT: Duration = Duration::from_secs(30);

static NEXT_CLIENT_ID: AtomicU64 = AtomicU64::new(0);

pub fn next_client_id() -> u64 {
    NEXT_CLIENT_ID.fetch_add(1, Ordering::Relaxed)
}

/// Maps exposed network names to socketcan interface names. Each WS client
/// gets its own CAN socket, so the kernel provides filtering and cross-client
/// echo (loopback delivers a frame to the other sockets once it was actually
/// transmitted; RECV_OWN_MSGS stays off, so a client never sees its own).
pub struct BridgeHub {
    interfaces: HashMap<String, String>,
}

impl BridgeHub {
    pub fn new(specs: &[(String, String)]) -> Self {
        let interfaces = specs
            .iter()
            .map(|(can_name, exposed)| (exposed.clone(), can_name.clone()))
            .collect();
        BridgeHub { interfaces }
    }

    pub fn resolve(&self, exposed: &str) -> Option<&str> {
        self.interfaces.get(exposed).map(String::as_str)
    }

    pub fn interface_names(&self) -> Vec<String> {
        let mut names: Vec<_> = self.interfaces.keys().cloned().collect();
        names.sort();
        names
    }
}

/// Open and configure the CAN socket for one client. Returns the socket and
/// whether frames must additionally be filtered in userspace (kernel filter
/// could not be applied exactly).
pub fn open_client_socket(
    can_name: &str,
    filter: Option<&[CanFilter]>,
    want_errors: bool,
    log_name: &str,
) -> std::io::Result<(CanSocket, bool)> {
    let sock = CanSocket::open(can_name)?;
    let userspace_filter = apply_filter(&sock, filter, log_name);
    if want_errors
        && let Err(e) = sock.set_error_filter_accept_all() {
            log::warn!("{log_name}: failed to enable error frames: {e}");
        }
    Ok((sock, userspace_filter))
}

struct RxState {
    frames: VecDeque<WireFrame>,
    /// Frames discarded by buffer clears since the last overflow report.
    dropped: u64,
    /// Set once by the reader when the CAN read fails; the reader exits after.
    bus_error: Option<String>,
}

enum RxEvent {
    Frame(WireFrame),
    Overflow(u64),
    BusError(String),
}

/// Wait for the next receive event. Overflow is reported before the frames
/// that came after the clear; bus death is reported after everything that was
/// buffered before it.
async fn next_rx(state: &Mutex<RxState>, signal: &mut mpsc::Receiver<()>) -> RxEvent {
    loop {
        {
            let mut s = state.lock().unwrap();
            if s.dropped > 0 {
                let n = s.dropped;
                s.dropped = 0;
                return RxEvent::Overflow(n);
            }
            if let Some(frame) = s.frames.pop_front() {
                return RxEvent::Frame(frame);
            }
            if let Some(error) = s.bus_error.take() {
                return RxEvent::BusError(error);
            }
        }
        if signal.recv().await.is_none() {
            // Reader gone without setting bus_error — only after abort, where
            // nobody awaits us anymore. Report it anyway rather than spin.
            return RxEvent::BusError("CAN reader task ended".into());
        }
    }
}

/// Drains the CAN socket into the buffer so the kernel-side queue (only
/// ~200 frames at default rcvbuf) never fills while the WS is slow.
async fn reader_task(sock: Arc<CanSocket>, state: Arc<Mutex<RxState>>, signal: mpsc::Sender<()>) {
    loop {
        match sock.read_frame().await {
            Ok(frame) => {
                // RTR frames: socketcan's data() yields DLC zero bytes, so
                // the wire frame keeps the DLC.
                let Some(wf) = WireFrame::new(frame.id_word(), frame.data()) else {
                    continue;
                };
                {
                    let mut s = state.lock().unwrap();
                    if s.frames.len() >= FRAME_BUFFER {
                        s.dropped += s.frames.len() as u64;
                        s.frames.clear();
                    }
                    s.frames.push_back(wf);
                }
                let _ = signal.try_send(());
            }
            Err(e) => {
                state.lock().unwrap().bus_error = Some(e.to_string());
                let _ = signal.try_send(());
                return;
            }
        }
    }
}

pub async fn run_client(
    ws: WebSocket,
    sock: CanSocket,
    log_name: String,
    mut filter: Option<Vec<CanFilter>>,
    mut userspace_filter: bool,
) {
    log::info!("{log_name}: connected");

    let sock = Arc::new(sock);
    let state = Arc::new(Mutex::new(RxState {
        frames: VecDeque::with_capacity(FRAME_BUFFER),
        dropped: 0,
        bus_error: None,
    }));
    let (signal_tx, mut signal_rx) = mpsc::channel::<()>(1);
    let reader = tokio::spawn(reader_task(sock.clone(), state.clone(), signal_tx));

    let (mut ws_tx, mut ws_rx) = ws.split();
    let mut ping = tokio::time::interval(PING_INTERVAL);
    ping.set_missed_tick_behavior(MissedTickBehavior::Delay);
    let mut last_rx = Instant::now();

    loop {
        tokio::select! {
            event = next_rx(&state, &mut signal_rx) => match event {
                RxEvent::Frame(frame) => {
                    // Error frames are only delivered if this client enabled
                    // the error mask; they bypass id filters (kernel semantics).
                    let pass = frame.is_error()
                        || !userspace_filter
                        || match &filter {
                            None => true,
                            Some(filters) => filters.iter().any(|flt| flt.matches(frame.id_word())),
                        };
                    if pass {
                        let (buf, len) = frame.encode();
                        if ws_tx
                            .send(ws::Message::binary(buf[..len].to_vec()))
                            .await
                            .is_err()
                        {
                            break;
                        }
                    }
                }
                RxEvent::Overflow(dropped) => {
                    log::warn!("{log_name}: rx buffer overflow, {dropped} frames dropped");
                    let status =
                        serde_json::to_string(&ServerStatusRef::Overflow { dropped }).unwrap();
                    if ws_tx.send(ws::Message::text(status)).await.is_err() {
                        break;
                    }
                }
                RxEvent::BusError(error) => {
                    log::warn!("{log_name}: bus error: {error}; disconnecting");
                    let status =
                        serde_json::to_string(&ServerStatusRef::BusError { error: &error })
                            .unwrap();
                    let _ = ws_tx.send(ws::Message::text(status)).await;
                    let _ = ws_tx
                        .send(ws::Message::Close(Some(CloseFrame {
                            code: close_code::BUS_ERROR,
                            reason: error.into(),
                        })))
                        .await;
                    break;
                }
            },

            msg = ws_rx.next() => {
                let Some(Ok(msg)) = msg else { break };
                last_rx = Instant::now();
                match msg {
                    ws::Message::Binary(data) => match WireFrame::decode(&data) {
                        Ok(frame) => match can_frame_from_wire(&frame) {
                            Some(out) => {
                                if let Err(e) = sock.write_frame(out).await {
                                    log::warn!("{log_name}: CAN write error: {e}");
                                }
                            }
                            None => {
                                log::warn!(
                                    "{log_name}: refusing to send frame: id_word={:#x} dlc={}",
                                    frame.id_word(),
                                    frame.dlc()
                                );
                            }
                        },
                        Err(e) => {
                            log::debug!("{log_name}: ignoring invalid binary frame: {e}");
                        }
                    },
                    ws::Message::Text(text) => match serde_json::from_str::<ClientCommand>(&text) {
                        Ok(cmd) => {
                            filter = match &cmd {
                                ClientCommand::SetFilter { filter } => {
                                    Some(filter.iter().map(CanFilter::from).collect())
                                }
                                ClientCommand::ClearFilter => None,
                            };
                            userspace_filter = apply_filter(&sock, filter.as_deref(), &log_name);
                        }
                        Err(e) => {
                            log::debug!("{log_name}: invalid client command: {e}");
                        }
                    },
                    ws::Message::Close(_) => break,
                    // Pings are answered by the WS layer; Pongs only matter
                    // for last_rx, updated above.
                    _ => {}
                }
            },

            _ = ping.tick() => {
                if last_rx.elapsed() > CLIENT_TIMEOUT {
                    log::info!(
                        "{log_name}: timed out ({}s without traffic)",
                        CLIENT_TIMEOUT.as_secs()
                    );
                    break;
                }
                if ws_tx.send(ws::Message::Ping(Vec::new().into())).await.is_err() {
                    break;
                }
            }
        }
    }

    reader.abort();
    log::info!("{log_name}: disconnected");
}

/// Apply a client filter to the socket's kernel filter. Returns whether
/// frames must additionally be checked in userspace because the kernel could
/// not be configured exactly (the kernel side is then accept-all).
fn apply_filter(sock: &CanSocket, filter: Option<&[CanFilter]>, log_name: &str) -> bool {
    match filter {
        None => {
            if let Err(e) = sock.set_filter_accept_all() {
                log::warn!("{log_name}: failed to set accept-all filter: {e}");
            }
            false
        }
        Some([]) => {
            if let Err(e) = sock.set_filter_drop_all() {
                log::warn!("{log_name}: failed to set drop-all filter: {e}");
                return true;
            }
            false
        }
        Some(filters) if filters.len() > MAX_KERNEL_FILTERS => {
            log::info!(
                "{log_name}: {} filters exceed the kernel limit ({MAX_KERNEL_FILTERS}); filtering in userspace",
                filters.len()
            );
            let _ = sock.set_filter_accept_all();
            true
        }
        Some(filters) => {
            let pairs: Vec<(u32, u32)> = filters.iter().map(|f| (f.id, f.mask)).collect();
            match sock.set_filters(&pairs) {
                Ok(()) => false,
                Err(e) => {
                    log::warn!(
                        "{log_name}: failed to set {} kernel filters: {e}; falling back to accept-all",
                        pairs.len()
                    );
                    let _ = sock.set_filter_accept_all();
                    true
                }
            }
        }
    }
}

/// Build a socketcan frame from a decoded wire frame. `None` for frames that
/// cannot be transmitted (error frames, out-of-range IDs).
///
/// Not `CanFrame::from_raw_id`: that feeds the whole id word (flags included)
/// into the ID range check, so every EFF or RTR frame would be rejected.
fn can_frame_from_wire(wf: &WireFrame) -> Option<CanFrame> {
    if wf.is_error() {
        return None;
    }
    let id: Id = if wf.is_extended() {
        ExtendedId::new(wf.id())?.into()
    } else {
        StandardId::new(wf.id() as u16)?.into()
    };
    if wf.is_rtr() {
        CanRemoteFrame::new_remote(id, wf.dlc() as usize).map(CanFrame::Remote)
    } else {
        CanFrame::new(id, wf.data())
    }
}
