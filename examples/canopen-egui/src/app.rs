use crate::can_bus::{Backend, CanConnection, Command, ConnectionTarget, Event};
use canopen_core::cobid::{CobId, ParsedCobId};
use canopen_core::emcy::EmcyMessage;
use canopen_core::nmt::NmtState;
use canopen_core::transport::CanFrame;
use eframe::egui;
use std::collections::BTreeMap;
#[cfg(not(target_arch = "wasm32"))]
use std::io::{Read, Write};
use web_time::{Duration, Instant};

pub struct App {
    connection: Option<CanConnection>,

    backend: Backend,
    slcan_device: String,
    socketcan_interface: String,
    canwsd_base_url: String,
    canwsd_networks: Vec<String>,
    canwsd_selected_network: String,
    canwsd_fetch_error: Option<String>,
    canwsd_fetching: bool,

    connected: bool,
    connection_error: Option<String>,
    nodes: BTreeMap<u8, NodeInfo>,
    messages: BTreeMap<u16, MessageEntry>,
    selected_node: Option<u8>,
}

#[derive(Default)]
struct NodeInfo {
    state: Option<NmtState>,
    last_seen: Option<Instant>,
    last_heartbeat: Option<Instant>,
    heartbeat_count: u64,
    emcy_count: u64,
    tpdo_count: [u64; 4],
    rpdo_count: [u64; 4],
    last_emcy: Option<EmcyMessage>,
}

impl NodeInfo {
    fn is_online(&self) -> bool {
        self.last_heartbeat
            .map(|t| t.elapsed() < Duration::from_secs(3))
            .unwrap_or(false)
    }
}

struct MessageEntry {
    can_id: u16,
    data: Vec<u8>,
    count: u64,
    last_seen: Instant,
}

impl App {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Self {
            connection: None,
            backend: Backend::Slcan,
            slcan_device: "/dev/ttyACM0".into(),
            socketcan_interface: "can0".into(),
            canwsd_base_url: "http://127.0.0.1:8080".into(),
            canwsd_networks: Vec::new(),
            canwsd_selected_network: String::new(),
            canwsd_fetch_error: None,
            canwsd_fetching: false,
            connected: false,
            connection_error: None,
            nodes: BTreeMap::new(),
            messages: BTreeMap::new(),
            selected_node: None,
        }
    }

    fn connect(&mut self) {
        let target = match self.connection_target() {
            Ok(target) => target,
            Err(e) => {
                self.connection_error = Some(e);
                return;
            }
        };

        self.connection = Some(CanConnection::connect(target));
        self.connection_error = None;
    }

    fn disconnect(&mut self) {
        if let Some(connection) = &mut self.connection {
            connection.disconnect();
        }
        self.connection = None;
        self.connected = false;
    }

    fn connection_target(&self) -> Result<ConnectionTarget, String> {
        match self.backend {
            Backend::Slcan => Ok(ConnectionTarget::Slcan {
                device: self.slcan_device.clone(),
            }),
            Backend::Socketcan => Ok(ConnectionTarget::Socketcan {
                interface: self.socketcan_interface.clone(),
            }),
            Backend::Canwsd => {
                if self.canwsd_selected_network.is_empty() {
                    return Err("select a canwsd network".into());
                }
                Ok(ConnectionTarget::Canwsd {
                    url: canwsd_ws_url(&self.canwsd_base_url, &self.canwsd_selected_network),
                })
            }
        }
    }

    fn process_events(&mut self) {
        while let Some(event) = self
            .connection
            .as_mut()
            .and_then(CanConnection::try_recv_event)
        {
            match event {
                Event::Connected => {
                    self.connected = true;
                    self.connection_error = None;
                }
                Event::Disconnected => {
                    self.connected = false;
                }
                Event::Error(e) => {
                    self.connection_error = Some(e);
                    self.connected = false;
                }
                Event::Frame(frame) => self.process_frame(frame),
            }
        }

        #[cfg(target_arch = "wasm32")]
        while let Some(result) = crate::can_bus::try_recv_canwsd_networks() {
            self.canwsd_fetching = false;
            match result {
                Ok(networks) => {
                    self.canwsd_fetch_error = None;
                    self.canwsd_networks = networks;
                    if !self
                        .canwsd_networks
                        .iter()
                        .any(|n| n == &self.canwsd_selected_network)
                    {
                        self.canwsd_selected_network =
                            self.canwsd_networks.first().cloned().unwrap_or_default();
                    }
                }
                Err(e) => self.canwsd_fetch_error = Some(e),
            }
        }
    }

    fn process_frame(&mut self, frame: CanFrame) {
        let now = Instant::now();
        let data = frame.data().to_vec();
        let entry = self.messages.entry(frame.raw_id()).or_insert(MessageEntry {
            can_id: frame.raw_id(),
            data: Vec::new(),
            count: 0,
            last_seen: now,
        });
        entry.data = data;
        entry.count += 1;
        entry.last_seen = now;

        let Some(cob) = CobId::new(frame.raw_id()) else {
            return;
        };
        match cob.parse() {
            ParsedCobId::Heartbeat(node) => {
                let info = self.nodes.entry(node.raw()).or_default();
                info.last_seen = Some(now);
                info.last_heartbeat = Some(now);
                info.heartbeat_count += 1;
                if let Some(&state) = frame.data().first() {
                    info.state = NmtState::from_heartbeat_byte(state);
                }
            }
            ParsedCobId::Emergency(node) => {
                let info = self.nodes.entry(node.raw()).or_default();
                info.last_seen = Some(now);
                info.emcy_count += 1;
                info.last_emcy = EmcyMessage::parse(&frame);
            }
            ParsedCobId::Tpdo { pdo_num, node } => {
                let info = self.nodes.entry(node.raw()).or_default();
                info.last_seen = Some(now);
                if let Some(count) = info.tpdo_count.get_mut(pdo_num as usize) {
                    *count += 1;
                }
            }
            ParsedCobId::Rpdo { pdo_num, node } => {
                let info = self.nodes.entry(node.raw()).or_default();
                info.last_seen = Some(now);
                if let Some(count) = info.rpdo_count.get_mut(pdo_num as usize) {
                    *count += 1;
                }
            }
            ParsedCobId::SdoRequest(node) | ParsedCobId::SdoResponse(node) => {
                let info = self.nodes.entry(node.raw()).or_default();
                info.last_seen = Some(now);
            }
            ParsedCobId::Nmt | ParsedCobId::Sync | ParsedCobId::Unknown(_) => {}
        }
    }

    fn send_nmt(&self, node_id: u8, command: u8) {
        if let Some(connection) = &self.connection {
            connection.send_cmd(Command::Nmt { node_id, command });
        }
    }
}

impl eframe::App for App {
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.process_events();

        if self.connected || self.canwsd_fetching {
            ctx.request_repaint_after(Duration::from_millis(50));
        }
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        style_ui(ui.ctx());

        egui::Panel::top("connection").show_inside(ui, |ui| {
            self.render_connection(ui);
        });

        egui::Panel::left("nodes")
            .default_size(260.0)
            .show_inside(ui, |ui| self.render_nodes(ui));

        egui::CentralPanel::default().show_inside(ui, |ui| {
            self.render_bus(ui);
        });
    }
}

impl App {
    fn render_connection(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.label("Backend");
            ui.add_enabled_ui(!self.connected, |ui| {
                egui::ComboBox::from_id_salt("backend")
                    .selected_text(self.backend.label())
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.backend, Backend::Slcan, "SLCAN");
                        ui.selectable_value(&mut self.backend, Backend::Socketcan, "SocketCAN");
                        ui.selectable_value(&mut self.backend, Backend::Canwsd, "canwsd");
                    });
            });

            match self.backend {
                Backend::Slcan => {
                    ui.label("Device");
                    ui.add_enabled_ui(!self.connected, |ui| {
                        let devices = enumerate_slcan_devices();
                        device_combo(ui, "slcan_device", &mut self.slcan_device, &devices);
                    });
                }
                Backend::Socketcan => {
                    ui.label("Interface");
                    ui.add_enabled_ui(!self.connected, |ui| {
                        let interfaces = enumerate_socketcan_interfaces();
                        device_combo(
                            ui,
                            "socketcan_interface",
                            &mut self.socketcan_interface,
                            &interfaces,
                        );
                    });
                }
                Backend::Canwsd => {
                    ui.label("Base URL");
                    ui.add_enabled_ui(!self.connected, |ui| {
                        ui.add(
                            egui::TextEdit::singleline(&mut self.canwsd_base_url)
                                .desired_width(230.0),
                        );
                        if ui.button("Discover").clicked() {
                            self.discover_canwsd_networks();
                        }
                        egui::ComboBox::from_id_salt("canwsd_network")
                            .selected_text(if self.canwsd_selected_network.is_empty() {
                                "network"
                            } else {
                                &self.canwsd_selected_network
                            })
                            .show_ui(ui, |ui| {
                                for network in &self.canwsd_networks {
                                    ui.selectable_value(
                                        &mut self.canwsd_selected_network,
                                        network.clone(),
                                        network,
                                    );
                                }
                            });
                    });
                }
            }

            ui.separator();
            if self.connected {
                if ui.button("Disconnect").clicked() {
                    self.disconnect();
                }
                ui.colored_label(egui::Color32::from_rgb(80, 210, 120), "Connected");
            } else if ui.button("Open").clicked() {
                self.connect();
            }

            if let Some(err) = self.connection_error.as_ref() {
                ui.colored_label(egui::Color32::from_rgb(255, 90, 90), err);
            }
            if let Some(err) = self.canwsd_fetch_error.as_ref() {
                ui.colored_label(egui::Color32::from_rgb(255, 180, 80), err);
            }
            if self.canwsd_fetching {
                ui.colored_label(egui::Color32::from_rgb(180, 180, 180), "Discovering");
            }
        });
    }

    fn discover_canwsd_networks(&mut self) {
        self.canwsd_fetch_error = None;
        self.canwsd_fetching = true;

        #[cfg(not(target_arch = "wasm32"))]
        {
            match fetch_canwsd_networks(&self.canwsd_base_url) {
                Ok(networks) => {
                    self.canwsd_fetching = false;
                    self.canwsd_networks = networks;
                    if !self
                        .canwsd_networks
                        .iter()
                        .any(|n| n == &self.canwsd_selected_network)
                    {
                        self.canwsd_selected_network =
                            self.canwsd_networks.first().cloned().unwrap_or_default();
                    }
                }
                Err(e) => {
                    self.canwsd_fetching = false;
                    self.canwsd_fetch_error = Some(e);
                }
            }
        }

        #[cfg(target_arch = "wasm32")]
        crate::can_bus::fetch_canwsd_networks_async(self.canwsd_base_url.clone());
    }

    fn render_nodes(&mut self, ui: &mut egui::Ui) {
        ui.heading("Nodes");
        ui.separator();

        if self.nodes.is_empty() {
            ui.colored_label(egui::Color32::GRAY, "No nodes observed");
        }

        for (&node_id, info) in &self.nodes {
            ui.horizontal(|ui| {
                let color = if info.is_online() {
                    egui::Color32::from_rgb(80, 210, 120)
                } else {
                    egui::Color32::GRAY
                };
                ui.colored_label(color, if info.is_online() { "up" } else { "--" });
                let selected = self.selected_node == Some(node_id);
                if ui
                    .selectable_label(selected, format!("Node 0x{node_id:02X}"))
                    .clicked()
                {
                    self.selected_node = Some(node_id);
                }
            });

            ui.indent(format!("node_{node_id}"), |ui| {
                ui.small(format!(
                    "{}  HB {}  EMCY {}",
                    state_label(info.state),
                    info.heartbeat_count,
                    info.emcy_count
                ));
            });
        }

        ui.separator();
        let Some(node_id) = self.selected_node else {
            ui.colored_label(egui::Color32::GRAY, "Select a node for NMT commands");
            return;
        };

        ui.label(format!("Selected 0x{node_id:02X}"));
        ui.horizontal(|ui| {
            if ui.button("Start").clicked() {
                self.send_nmt(node_id, 0x01);
            }
            if ui.button("Pre-Op").clicked() {
                self.send_nmt(node_id, 0x80);
            }
        });
        ui.horizontal(|ui| {
            if ui.button("Stop").clicked() {
                self.send_nmt(node_id, 0x02);
            }
            if ui.button("Reset").clicked() {
                self.send_nmt(node_id, 0x81);
            }
        });
        if ui.button("Reset Comm").clicked() {
            self.send_nmt(node_id, 0x82);
        }
    }

    fn render_bus(&mut self, ui: &mut egui::Ui) {
        ui.heading("CANopen Bus");
        ui.separator();

        ui.columns(4, |columns| {
            columns[0].group(|ui| self.render_node_summary(ui, SummaryKind::Heartbeat));
            columns[1].group(|ui| self.render_node_summary(ui, SummaryKind::Emcy));
            columns[2].group(|ui| self.render_node_summary(ui, SummaryKind::Tpdo));
            columns[3].group(|ui| self.render_node_summary(ui, SummaryKind::Rpdo));
        });

        ui.add_space(8.0);
        self.render_message_table(ui);
    }

    fn render_node_summary(&self, ui: &mut egui::Ui, kind: SummaryKind) {
        ui.heading(kind.title());
        ui.separator();
        egui::ScrollArea::vertical()
            .max_height(160.0)
            .show(ui, |ui| {
                for (&node_id, info) in &self.nodes {
                    match kind {
                        SummaryKind::Heartbeat => {
                            if info.heartbeat_count > 0 {
                                ui.monospace(format!(
                                    "0x{node_id:02X} {:<14} {}",
                                    state_label(info.state),
                                    age_label(info.last_heartbeat)
                                ));
                            }
                        }
                        SummaryKind::Emcy => {
                            if info.emcy_count > 0 {
                                let code = info
                                    .last_emcy
                                    .map(|e| format!("0x{:04X}", e.error_code))
                                    .unwrap_or_else(|| "-".into());
                                ui.monospace(format!(
                                    "0x{node_id:02X} {:>4} last {code}",
                                    info.emcy_count
                                ));
                            }
                        }
                        SummaryKind::Tpdo => {
                            for (idx, count) in info.tpdo_count.iter().enumerate() {
                                if *count > 0 {
                                    ui.monospace(format!(
                                        "0x{node_id:02X} TPDO{} {count}",
                                        idx + 1
                                    ));
                                }
                            }
                        }
                        SummaryKind::Rpdo => {
                            for (idx, count) in info.rpdo_count.iter().enumerate() {
                                if *count > 0 {
                                    ui.monospace(format!(
                                        "0x{node_id:02X} RPDO{} {count}",
                                        idx + 1
                                    ));
                                }
                            }
                        }
                    }
                }
            });
    }

    fn render_message_table(&self, ui: &mut egui::Ui) {
        ui.heading("Latest Frames");
        ui.separator();

        let mut entries: Vec<_> = self.messages.values().collect();
        entries.sort_by_key(|entry| {
            let (node, prio) = message_sort_key(entry.can_id);
            (node, prio, entry.can_id)
        });

        let row_height = ui.text_style_height(&egui::TextStyle::Body) + 3.0;
        egui_extras::TableBuilder::new(ui)
            .striped(true)
            .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
            .column(egui_extras::Column::exact(70.0))
            .column(egui_extras::Column::exact(70.0))
            .column(egui_extras::Column::exact(55.0))
            .column(egui_extras::Column::exact(40.0))
            .column(egui_extras::Column::remainder())
            .column(egui_extras::Column::exact(70.0))
            .column(egui_extras::Column::exact(70.0))
            .header(row_height, |mut header| {
                header.col(|ui| {
                    ui.strong("Type");
                });
                header.col(|ui| {
                    ui.strong("CAN ID");
                });
                header.col(|ui| {
                    ui.strong("Node");
                });
                header.col(|ui| {
                    ui.strong("DLC");
                });
                header.col(|ui| {
                    ui.strong("Data");
                });
                header.col(|ui| {
                    ui.strong("Count");
                });
                header.col(|ui| {
                    ui.strong("Age");
                });
            })
            .body(|body| {
                body.rows(row_height, entries.len(), |mut row| {
                    let entry = entries[row.index()];
                    let (kind, node) = classify_message(entry.can_id);
                    row.col(|ui| {
                        ui.colored_label(message_color(kind), kind);
                    });
                    row.col(|ui| {
                        ui.monospace(format!("0x{:03X}", entry.can_id));
                    });
                    row.col(|ui| {
                        if let Some(node) = node {
                            ui.monospace(format!("0x{node:02X}"));
                        } else {
                            ui.monospace("-");
                        }
                    });
                    row.col(|ui| {
                        ui.monospace(entry.data.len().to_string());
                    });
                    row.col(|ui| {
                        ui.monospace(hex_bytes(&entry.data));
                    });
                    row.col(|ui| {
                        ui.monospace(entry.count.to_string());
                    });
                    row.col(|ui| {
                        ui.monospace(age_label(Some(entry.last_seen)));
                    });
                });
            });
    }
}

#[derive(Clone, Copy)]
enum SummaryKind {
    Heartbeat,
    Emcy,
    Tpdo,
    Rpdo,
}

impl SummaryKind {
    fn title(self) -> &'static str {
        match self {
            Self::Heartbeat => "Heartbeat",
            Self::Emcy => "EMCY",
            Self::Tpdo => "TPDO",
            Self::Rpdo => "RPDO",
        }
    }
}

fn device_combo(ui: &mut egui::Ui, id: &'static str, value: &mut String, choices: &[String]) {
    let selected = if value.is_empty() {
        "device".to_string()
    } else {
        value.clone()
    };
    egui::ComboBox::from_id_salt(id)
        .width(180.0)
        .selected_text(selected)
        .show_ui(ui, |ui| {
            for choice in choices {
                ui.selectable_value(value, choice.clone(), choice);
            }
            ui.separator();
            ui.horizontal(|ui| {
                ui.label("Custom");
                ui.text_edit_singleline(value);
            });
        });
}

#[cfg(not(target_arch = "wasm32"))]
fn enumerate_slcan_devices() -> Vec<String> {
    let mut out = Vec::new();
    if let Ok(entries) = std::fs::read_dir("/dev") {
        for entry in entries.flatten() {
            let name = entry.file_name().to_string_lossy().to_string();
            if name.starts_with("ttyACM") || name.starts_with("ttyUSB") {
                out.push(format!("/dev/{name}"));
            }
        }
    }
    out.sort();
    out
}

#[cfg(target_arch = "wasm32")]
fn enumerate_slcan_devices() -> Vec<String> {
    Vec::new()
}

#[cfg(not(target_arch = "wasm32"))]
fn enumerate_socketcan_interfaces() -> Vec<String> {
    let mut out = Vec::new();
    if let Ok(entries) = std::fs::read_dir("/sys/class/net") {
        for entry in entries.flatten() {
            let name = entry.file_name().to_string_lossy().to_string();
            if name.starts_with("can") || name.starts_with("vcan") {
                out.push(name);
            }
        }
    }
    out.sort();
    out
}

#[cfg(target_arch = "wasm32")]
fn enumerate_socketcan_interfaces() -> Vec<String> {
    Vec::new()
}

#[cfg(not(target_arch = "wasm32"))]
fn fetch_canwsd_networks(base_url: &str) -> Result<Vec<String>, String> {
    let url = format!(
        "{}{}",
        base_url.trim_end_matches('/'),
        canwsd_proto::NETWORKS_PATH
    );
    let (host, port, path) = parse_http_url(&url)?;
    let mut stream = std::net::TcpStream::connect((host.as_str(), port))
        .map_err(|e| format!("connect {host}:{port}: {e}"))?;
    stream
        .set_read_timeout(Some(Duration::from_secs(2)))
        .map_err(|e| e.to_string())?;

    let request = format!(
        "GET {path} HTTP/1.1\r\nHost: {host}\r\nConnection: close\r\nAccept: application/json\r\n\r\n"
    );
    stream
        .write_all(request.as_bytes())
        .map_err(|e| e.to_string())?;

    let mut response = String::new();
    stream
        .read_to_string(&mut response)
        .map_err(|e| e.to_string())?;

    let (_, body) = response
        .split_once("\r\n\r\n")
        .ok_or_else(|| "bad HTTP response".to_string())?;
    let networks: Vec<canwsd_proto::NetworkInfo> =
        serde_json::from_str(body).map_err(|e| e.to_string())?;
    Ok(networks.into_iter().map(|n| n.name).collect())
}

#[cfg(not(target_arch = "wasm32"))]
fn parse_http_url(url: &str) -> Result<(String, u16, String), String> {
    let rest = url
        .strip_prefix("http://")
        .ok_or_else(|| "only http:// canwsd URLs are supported".to_string())?;
    let (authority, path) = rest.split_once('/').unwrap_or((rest, ""));
    let (host, port) = match authority.rsplit_once(':') {
        Some((host, port)) => (
            host.to_string(),
            port.parse::<u16>().map_err(|e| e.to_string())?,
        ),
        None => (authority.to_string(), 80),
    };
    Ok((host, port, format!("/{path}")))
}

fn canwsd_ws_url(base_url: &str, network: &str) -> String {
    let base = base_url.trim_end_matches('/');
    let ws_base = if let Some(rest) = base.strip_prefix("http://") {
        format!("ws://{rest}")
    } else if let Some(rest) = base.strip_prefix("https://") {
        format!("wss://{rest}")
    } else {
        base.to_string()
    };
    format!("{ws_base}{}/{network}", canwsd_proto::NETWORKS_PATH)
}

fn classify_message(can_id: u16) -> (&'static str, Option<u8>) {
    if let Some(cob) = CobId::new(can_id) {
        match cob.parse() {
            ParsedCobId::Nmt => ("NMT", None),
            ParsedCobId::Sync => ("SYNC", None),
            ParsedCobId::Emergency(node) => ("EMCY", Some(node.raw())),
            ParsedCobId::Tpdo { pdo_num, node } => (pdo_label("TPDO", pdo_num), Some(node.raw())),
            ParsedCobId::Rpdo { pdo_num, node } => (pdo_label("RPDO", pdo_num), Some(node.raw())),
            ParsedCobId::SdoResponse(node) => ("SDO_TX", Some(node.raw())),
            ParsedCobId::SdoRequest(node) => ("SDO_RX", Some(node.raw())),
            ParsedCobId::Heartbeat(node) => ("HB", Some(node.raw())),
            ParsedCobId::Unknown(_) => ("OTHER", None),
        }
    } else {
        ("OTHER", None)
    }
}

fn pdo_label(prefix: &'static str, pdo_num: u8) -> &'static str {
    match (prefix, pdo_num) {
        ("TPDO", 0) => "TPDO1",
        ("TPDO", 1) => "TPDO2",
        ("TPDO", 2) => "TPDO3",
        ("TPDO", 3) => "TPDO4",
        ("RPDO", 0) => "RPDO1",
        ("RPDO", 1) => "RPDO2",
        ("RPDO", 2) => "RPDO3",
        ("RPDO", 3) => "RPDO4",
        _ => "PDO?",
    }
}

fn message_sort_key(can_id: u16) -> (u16, u8) {
    let (_, node) = classify_message(can_id);
    let node = node.map(u16::from).unwrap_or(999);
    let prio = match can_id {
        0x700..=0x77F => 0,
        0x080..=0x0FF => 1,
        0x180..=0x1FF => 2,
        0x200..=0x27F => 3,
        0x280..=0x2FF => 4,
        0x300..=0x37F => 5,
        0x380..=0x3FF => 6,
        0x400..=0x47F => 7,
        0x480..=0x4FF => 8,
        0x500..=0x57F => 9,
        0x580..=0x67F => 10,
        _ => 99,
    };
    (node, prio)
}

fn state_label(state: Option<NmtState>) -> &'static str {
    match state {
        Some(NmtState::Initializing) => "Boot-up",
        Some(NmtState::PreOperational) => "Pre-op",
        Some(NmtState::Operational) => "Operational",
        Some(NmtState::Stopped) => "Stopped",
        None => "-",
    }
}

fn message_color(kind: &str) -> egui::Color32 {
    match kind {
        "HB" => egui::Color32::from_rgb(105, 150, 240),
        "TPDO1" | "TPDO2" | "TPDO3" | "TPDO4" => egui::Color32::from_rgb(80, 220, 140),
        "RPDO1" | "RPDO2" | "RPDO3" | "RPDO4" => egui::Color32::from_rgb(100, 200, 210),
        "EMCY" => egui::Color32::from_rgb(255, 90, 90),
        "SDO_TX" | "SDO_RX" => egui::Color32::from_rgb(245, 195, 80),
        _ => egui::Color32::from_rgb(190, 190, 190),
    }
}

fn hex_bytes(data: &[u8]) -> String {
    data.iter()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn age_label(time: Option<Instant>) -> String {
    match time {
        Some(t) => {
            let elapsed = t.elapsed();
            if elapsed.as_secs() >= 1 {
                format!("{:.1}s", elapsed.as_secs_f32())
            } else {
                format!("{}ms", elapsed.as_millis())
            }
        }
        None => "-".into(),
    }
}

fn style_ui(ctx: &egui::Context) {
    use egui::{Color32, CornerRadius, Visuals};

    let mut visuals = Visuals::dark();
    visuals.window_corner_radius = CornerRadius::same(4);
    visuals.menu_corner_radius = CornerRadius::same(4);
    visuals.widgets.noninteractive.corner_radius = CornerRadius::same(3);
    visuals.widgets.inactive.corner_radius = CornerRadius::same(3);
    visuals.widgets.hovered.corner_radius = CornerRadius::same(3);
    visuals.widgets.active.corner_radius = CornerRadius::same(3);
    visuals.widgets.open.corner_radius = CornerRadius::same(3);
    visuals.selection.bg_fill = Color32::from_rgb(70, 120, 160);
    visuals.hyperlink_color = Color32::from_rgb(100, 180, 220);
    ctx.set_visuals(visuals);
}
