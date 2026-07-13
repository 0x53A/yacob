use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use axum::extract::ws::{self, WebSocket};
use futures_util::{SinkExt, StreamExt};
use socketcan::tokio::CanSocket;
use socketcan::{CanFrame, EmbeddedFrame, Frame, SocketOptions};
use tokio::sync::{broadcast, mpsc};

use canwsd_proto::filter::ClientCommand;
use canwsd_proto::{CanFilter, WireFrame};

const BROADCAST_CAPACITY: usize = 256;
const WRITE_CHANNEL_CAPACITY: usize = 64;

enum KernelFilterUpdate {
    DropAll,
    AcceptAll,
    Set(Vec<(u32, u32)>),
}

struct FilterManager {
    clients: HashMap<u64, Option<Vec<CanFilter>>>,
    next_id: u64,
    filter_tx: mpsc::UnboundedSender<KernelFilterUpdate>,
}

impl FilterManager {
    fn new(filter_tx: mpsc::UnboundedSender<KernelFilterUpdate>) -> Self {
        Self {
            clients: HashMap::new(),
            next_id: 0,
            filter_tx,
        }
    }

    fn register(&mut self, initial: Option<Vec<CanFilter>>) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        self.clients.insert(id, initial);
        self.recompute();
        id
    }

    fn unregister(&mut self, id: u64) {
        self.clients.remove(&id);
        self.recompute();
    }

    fn update(&mut self, id: u64, filter: Option<Vec<CanFilter>>) {
        if let Some(entry) = self.clients.get_mut(&id) {
            *entry = filter;
            self.recompute();
        }
    }

    fn recompute(&self) {
        let update = if self.clients.is_empty() {
            KernelFilterUpdate::DropAll
        } else if self.clients.values().any(|f| f.is_none()) {
            KernelFilterUpdate::AcceptAll
        } else {
            let filters: Vec<(u32, u32)> = self
                .clients
                .values()
                .flat_map(|f| f.as_ref().unwrap().iter())
                .map(|f| (f.id, f.mask))
                .collect();
            if filters.is_empty() {
                KernelFilterUpdate::DropAll
            } else {
                KernelFilterUpdate::Set(filters)
            }
        };
        let _ = self.filter_tx.send(update);
    }
}

struct Interface {
    rx: broadcast::Sender<(u32, Vec<u8>)>,
    write_tx: mpsc::Sender<(u32, Vec<u8>)>,
    filters: Arc<Mutex<FilterManager>>,
}

pub struct BridgeHub {
    interfaces: HashMap<String, Arc<Interface>>,
}

impl BridgeHub {
    pub fn new(specs: &[(String, String)]) -> Self {
        let mut interfaces = HashMap::new();
        for (can_name, exposed_name) in specs {
            let (bc_tx, _) = broadcast::channel(BROADCAST_CAPACITY);
            let (write_tx, write_rx) = mpsc::channel(WRITE_CHANNEL_CAPACITY);
            let (filter_tx, filter_rx) = mpsc::unbounded_channel();

            let fm = Arc::new(Mutex::new(FilterManager::new(filter_tx)));

            let iface = Arc::new(Interface {
                rx: bc_tx.clone(),
                write_tx,
                filters: fm,
            });

            let can_name = can_name.clone();
            let log_name = if *exposed_name != can_name {
                format!("{can_name} (as {exposed_name})")
            } else {
                can_name.clone()
            };
            tokio::spawn(can_task(can_name, log_name, bc_tx, write_rx, filter_rx));

            interfaces.insert(exposed_name.clone(), iface);
        }
        BridgeHub { interfaces }
    }

    pub fn interface_names(&self) -> Vec<String> {
        let mut names: Vec<_> = self.interfaces.keys().cloned().collect();
        names.sort();
        names
    }

    pub async fn serve_client(
        &self,
        socket: WebSocket,
        interface: &str,
        initial_filter: Option<Vec<CanFilter>>,
    ) -> Result<(), String> {
        let iface = self
            .interfaces
            .get(interface)
            .ok_or_else(|| format!("unknown interface: {interface}"))?;

        let client_id = iface
            .filters
            .lock()
            .unwrap()
            .register(initial_filter.clone());
        log::info!("{interface}: client {client_id} connected");

        let client_filters: Arc<Mutex<Option<Vec<CanFilter>>>> =
            Arc::new(Mutex::new(initial_filter));
        let mut can_rx = iface.rx.subscribe();
        let write_tx = iface.write_tx.clone();

        let (mut ws_tx, mut ws_rx) = socket.split();
        let client_filters_recv = client_filters.clone();
        let fm_for_send = iface.filters.clone();

        let recv_task = tokio::spawn(async move {
            loop {
                let (id, data) = match can_rx.recv().await {
                    Ok(frame) => frame,
                    Err(broadcast::error::RecvError::Lagged(n)) => {
                        let msg = serde_json::json!({"status": "lagged", "dropped": n});
                        let _ = ws_tx.send(ws::Message::text(msg.to_string())).await;
                        continue;
                    }
                    Err(broadcast::error::RecvError::Closed) => break,
                };

                let pass = {
                    let f = client_filters_recv.lock().unwrap();
                    match &*f {
                        None => true,
                        Some(filters) => filters.iter().any(|flt| flt.matches(id)),
                    }
                };

                if pass {
                    let Some(frame) = WireFrame::new(id, &data) else {
                        log::warn!(
                            "dropping oversized CAN frame from socketcan: id={id:#x} len={}",
                            data.len()
                        );
                        continue;
                    };
                    let (buf, len) = frame.encode();
                    if ws_tx.send(ws::Message::binary(buf[..len].to_vec())).await.is_err() {
                        break;
                    }
                }
            }
        });

        let send_task = tokio::spawn(async move {
            while let Some(Ok(msg)) = ws_rx.next().await {
                match msg {
                    ws::Message::Binary(data) => match WireFrame::decode(&data) {
                        Ok(frame) => {
                            let _ = write_tx.send((frame.id_word(), frame.data().to_vec())).await;
                        }
                        Err(e) => {
                            log::debug!("ignoring invalid binary frame: {e}");
                        }
                    },
                    ws::Message::Text(text) => match serde_json::from_str::<ClientCommand>(&text) {
                        Ok(cmd) => {
                            let new_filter = match &cmd {
                                ClientCommand::SetFilter { filter } => {
                                    Some(filter.iter().map(CanFilter::from).collect())
                                }
                                ClientCommand::ClearFilter => None,
                            };
                            *client_filters.lock().unwrap() = new_filter.clone();
                            fm_for_send.lock().unwrap().update(client_id, new_filter);
                        }
                        Err(e) => {
                            log::debug!("invalid client command: {e}");
                        }
                    },
                    ws::Message::Close(_) => break,
                    _ => {}
                }
            }
        });

        tokio::select! {
            _ = recv_task => {},
            _ = send_task => {},
        }

        iface.filters.lock().unwrap().unregister(client_id);
        log::info!("{interface}: client {client_id} disconnected");

        Ok(())
    }
}

async fn can_task(
    can_name: String,
    log_name: String,
    bc_tx: broadcast::Sender<(u32, Vec<u8>)>,
    mut write_rx: mpsc::Receiver<(u32, Vec<u8>)>,
    mut filter_rx: mpsc::UnboundedReceiver<KernelFilterUpdate>,
) {
    let sock = match CanSocket::open(&can_name) {
        Ok(s) => {
            log::info!("{log_name}: opened");
            s
        }
        Err(e) => {
            log::error!("{log_name}: failed to open: {e}");
            return;
        }
    };

    if let Err(e) = sock.set_filter_drop_all() {
        log::warn!("{log_name}: failed to set initial drop-all filter: {e}");
    }
    log::info!("{log_name}: kernel filter set to drop-all (no clients)");

    loop {
        tokio::select! {
            result = sock.read_frame() => {
                match result {
                    Ok(frame) => {
                        let id = frame.id_word();
                        let data = frame.data();
                        let _ = bc_tx.send((id, data.to_vec()));
                    }
                    Err(e) => {
                        log::error!("{log_name}: CAN read error: {e}");
                    }
                }
            }

            Some((id, data)) = write_rx.recv() => {
                match CanFrame::from_raw_id(id, &data) {
                    Some(frame) => {
                        if let Err(e) = sock.write_frame(frame).await {
                            log::warn!("{log_name}: CAN write error: {e}");
                        }
                    }
                    None => {
                        log::warn!("{log_name}: invalid CAN frame: id={id:#x} len={}", data.len());
                    }
                }
            }

            Some(update) = filter_rx.recv() => {
                apply_kernel_filter(&sock, &log_name, update);
            }
        }
    }
}

fn apply_kernel_filter(sock: &CanSocket, log_name: &str, update: KernelFilterUpdate) {
    match update {
        KernelFilterUpdate::DropAll => {
            if let Err(e) = sock.set_filter_drop_all() {
                log::warn!("{log_name}: failed to set drop-all filter: {e}");
            } else {
                log::info!("{log_name}: kernel filter set to drop-all");
            }
        }
        KernelFilterUpdate::AcceptAll => {
            if let Err(e) = sock.set_filter_accept_all() {
                log::warn!("{log_name}: failed to set accept-all filter: {e}");
            } else {
                log::info!("{log_name}: kernel filter set to accept-all");
            }
        }
        KernelFilterUpdate::Set(filters) => {
            if let Err(e) = sock.set_filters(&filters) {
                log::warn!("{log_name}: failed to set kernel filters: {e}");
            } else {
                log::info!(
                    "{log_name}: kernel filter updated ({} rules)",
                    filters.len()
                );
            }
        }
    }
}
