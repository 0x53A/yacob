mod bridge;
mod socketcan_wire;

use axum::{
    Router,
    extract::{Path, Query, State, WebSocketUpgrade},
    http::StatusCode,
    response::Html,
    response::IntoResponse,
    routing::get,
};
use canwsd_proto::{
    ERRORS_QUERY_PARAM, FILTER_QUERY_PARAM, NETWORK_WS_ROUTE, NETWORKS_PATH, NetworkInfo,
    NetworkStatus, WireFrame,
};
use clap::{Parser, Subcommand};
use futures_util::{SinkExt, StreamExt};
use serde::Deserialize;
use socketcan::tokio::CanSocket;
use std::sync::Arc;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

use bridge::BridgeHub;
use socketcan_wire::{can_frame_from_wire, wire_from_can_frame};

#[derive(Parser)]
#[command(about = "CAN WebSocket daemon — bridges socketCAN interfaces over WebSocket")]
struct Args {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Expose one or more local SocketCAN interfaces over HTTP/WebSocket.
    Expose(ExposeArgs),
    /// Attach one remote CANWSD network to one local SocketCAN interface.
    Attach(AttachArgs),
    /// List networks exposed by a CANWSD server.
    List(ListArgs),
}

#[derive(Parser)]
struct ExposeArgs {
    /// TCP listen address
    #[arg(short, long, default_value = "0.0.0.0:8080")]
    listen: String,

    /// CAN interfaces to expose. Use socketcan_name:alias to rename
    /// (e.g. can_arm:arm). Without alias, the socketcan name is used as-is.
    #[arg(required = true, value_parser = parse_interface_spec)]
    interfaces: Vec<(String, String)>,
}

#[derive(Parser)]
struct AttachArgs {
    /// Remote CANWSD websocket URL, e.g. ws://host:8080/api/networks/can0.
    #[arg(long)]
    remote: String,

    /// Initial remote receive filter (`id:mask,id:mask,...`).
    #[arg(long)]
    filter: Option<String>,

    /// Request CAN error frames from the remote server.
    #[arg(long)]
    errors: bool,

    /// Local SocketCAN interface to bridge to, usually a vcan interface.
    local_interface: String,
}

#[derive(Parser)]
struct ListArgs {
    /// Print raw JSON instead of a table.
    #[arg(long)]
    json: bool,

    /// CANWSD HTTP(S) base URL, e.g. http://host:8080.
    base_url: String,
}

fn parse_interface_spec(s: &str) -> Result<(String, String), String> {
    match s.split_once(':') {
        Some((host, alias)) if !host.is_empty() && !alias.is_empty() => {
            Ok((host.to_string(), alias.to_string()))
        }
        Some(_) => Err(format!("expected SOCKETCAN_NAME:ALIAS, got '{s}'")),
        None => Ok((s.to_string(), s.to_string())),
    }
}

#[derive(Deserialize)]
struct WsParams {
    filter: Option<String>,
    errors: Option<String>,
}

async fn list_networks(State(hub): State<Arc<BridgeHub>>) -> impl IntoResponse {
    let networks = hub.networks();
    axum::Json(networks)
}

async fn index(State(hub): State<Arc<BridgeHub>>) -> impl IntoResponse {
    let networks = hub.networks();
    let mut html = String::from(
        "<!doctype html><meta charset=\"utf-8\"><title>canwsd</title>\
         <style>body{font-family:sans-serif;margin:2rem}table{border-collapse:collapse}\
         td,th{padding:.3rem .7rem;text-align:left;border-bottom:1px solid #ddd}\
         code{font-family:ui-monospace,monospace}</style><h1>canwsd</h1><table>\
         <thead><tr><th>Name</th><th>Status</th><th>Interface</th><th>Bitrate</th><th>Error</th></tr></thead><tbody>",
    );
    for network in networks {
        html.push_str(&format!(
            "<tr><td><code>{}</code></td><td>{}</td><td><code>{}</code></td><td>{}</td><td>{}</td></tr>",
            escape_html(&network.name),
            status_text(network.status),
            escape_html(&network.interface),
            format_bitrate(network.bitrate),
            escape_html(&network.error),
        ));
    }
    html.push_str("</tbody></table><p>JSON: <a href=\"/api/networks\">/api/networks</a></p>");
    Html(html)
}

async fn ws_handler(
    Path(name): Path<String>,
    Query(params): Query<WsParams>,
    State(hub): State<Arc<BridgeHub>>,
    ws: WebSocketUpgrade,
) -> impl IntoResponse {
    let Some(can_name) = hub.resolve(&name) else {
        return (StatusCode::NOT_FOUND, format!("unknown network: {name}")).into_response();
    };

    let initial_filter = match params.filter.as_deref() {
        Some(filter) => match canwsd_proto::filter::parse_filter_param(filter) {
            Ok(filter) => Some(filter),
            Err(e) => return (StatusCode::BAD_REQUEST, e).into_response(),
        },
        None => None,
    };
    let want_errors = matches!(params.errors.as_deref(), Some("1" | "true"));

    let client_id = bridge::next_client_id();
    let log_name = if can_name == name {
        format!("{name}: client {client_id}")
    } else {
        format!("{can_name} (as {name}): client {client_id}")
    };

    let (sock, userspace_filter) = match bridge::open_client_socket(
        can_name,
        initial_filter.as_deref(),
        want_errors,
        &log_name,
    ) {
        Ok(v) => v,
        Err(e) => {
            log::warn!("{log_name}: cannot open: {e}");
            return (
                StatusCode::SERVICE_UNAVAILABLE,
                format!("cannot open {name}: {e}"),
            )
                .into_response();
        }
    };

    ws.on_upgrade(move |socket| {
        bridge::run_client(socket, sock, log_name, initial_filter, userspace_filter)
    })
    .into_response()
}

async fn expose(args: ExposeArgs) -> Result<(), String> {
    let hub = Arc::new(BridgeHub::new(&args.interfaces));
    if hub.interface_names().len() != args.interfaces.len() {
        return Err("duplicate network names in interface list".into());
    }
    for network in hub.networks() {
        let shown = if network.interface == network.name {
            network.name.clone()
        } else {
            format!("{} (as {})", network.interface, network.name)
        };
        match network.status {
            NetworkStatus::Available => log::info!("{shown}: available"),
            NetworkStatus::Unavailable => {
                log::warn!(
                    "{shown}: not available: {} (clients get 503 until it is)",
                    network.error
                )
            }
        }
    }

    let app = Router::new()
        .route("/", get(index))
        .route(NETWORKS_PATH, get(list_networks))
        .route(NETWORK_WS_ROUTE, get(ws_handler))
        .with_state(hub);

    let listener = tokio::net::TcpListener::bind(&args.listen)
        .await
        .map_err(|e| format!("bind {}: {e}", args.listen))?;
    log::info!("canwsd listening on {}", args.listen);
    axum::serve(listener, app)
        .await
        .map_err(|e| format!("serve: {e}"))
}

async fn attach(args: AttachArgs) -> Result<(), String> {
    let remote = remote_url_with_params(&args.remote, args.filter.as_deref(), args.errors)?;
    let local = CanSocket::open(&args.local_interface)
        .map_err(|e| format!("open {}: {e}", args.local_interface))?;
    let (ws, _) = connect_async(&remote)
        .await
        .map_err(|e| format!("connect {remote}: {e}"))?;
    log::info!("attached {remote} to {}", args.local_interface);

    let (mut ws_tx, mut ws_rx) = ws.split();
    loop {
        tokio::select! {
            frame = local.read_frame() => {
                let frame = frame.map_err(|e| format!("read {}: {e}", args.local_interface))?;
                let Some(wire) = wire_from_can_frame(&frame) else {
                    continue;
                };
                let (buf, len) = wire.encode();
                ws_tx
                    .send(Message::Binary(buf[..len].to_vec().into()))
                    .await
                    .map_err(|e| format!("send websocket frame: {e}"))?;
            }

            msg = ws_rx.next() => {
                let Some(msg) = msg else {
                    return Err("remote websocket closed".into());
                };
                match msg.map_err(|e| format!("receive websocket frame: {e}"))? {
                    Message::Binary(data) => {
                        let wire = WireFrame::decode(&data)
                            .map_err(|e| format!("decode websocket frame: {e}"))?;
                        let Some(frame) = can_frame_from_wire(&wire) else {
                            log::debug!(
                                "ignoring non-transmittable remote frame: id_word={:#x} dlc={}",
                                wire.id_word(),
                                wire.dlc()
                            );
                            continue;
                        };
                        local
                            .write_frame(frame)
                            .await
                            .map_err(|e| format!("write {}: {e}", args.local_interface))?;
                    }
                    Message::Text(text) => {
                        log::info!("remote status: {text}");
                    }
                    Message::Close(close) => {
                        return Err(format!("remote websocket closed: {close:?}"));
                    }
                    Message::Ping(_) | Message::Pong(_) | Message::Frame(_) => {}
                }
            }
        }
    }
}

async fn list(args: ListArgs) -> Result<(), String> {
    let url = format!("{}{NETWORKS_PATH}", args.base_url.trim_end_matches('/'));
    let response = reqwest::get(&url)
        .await
        .map_err(|e| format!("GET {url}: {e}"))?;
    let status = response.status();
    if !status.is_success() {
        return Err(format!("GET {url}: HTTP {status}"));
    }
    let networks: Vec<NetworkInfo> = response
        .json()
        .await
        .map_err(|e| format!("parse {url}: {e}"))?;

    if args.json {
        println!(
            "{}",
            serde_json::to_string_pretty(&networks).map_err(|e| e.to_string())?
        );
    } else {
        print_network_table(&networks);
    }
    Ok(())
}

#[tokio::main]
async fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args = Args::parse();
    let result = match args.command {
        Command::Expose(args) => expose(args).await,
        Command::Attach(args) => attach(args).await,
        Command::List(args) => list(args).await,
    };
    if let Err(e) = result {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

fn remote_url_with_params(
    remote: &str,
    filter: Option<&str>,
    errors: bool,
) -> Result<String, String> {
    if !remote.starts_with("ws://") && !remote.starts_with("wss://") {
        return Err("--remote must start with ws:// or wss://".into());
    }
    if query_has_key(remote, FILTER_QUERY_PARAM) && filter.is_some() {
        return Err(format!(
            "--remote already contains `{FILTER_QUERY_PARAM}`; do not also pass --filter"
        ));
    }
    if query_has_key(remote, ERRORS_QUERY_PARAM) && errors {
        return Err(format!(
            "--remote already contains `{ERRORS_QUERY_PARAM}`; do not also pass --errors"
        ));
    }

    let mut url = remote.to_string();
    let mut sep = if remote.contains('?') { '&' } else { '?' };
    if let Some(filter) = filter {
        url.push(sep);
        sep = '&';
        url.push_str(FILTER_QUERY_PARAM);
        url.push('=');
        url.push_str(filter);
    }
    if errors {
        url.push(sep);
        url.push_str(ERRORS_QUERY_PARAM);
        url.push_str("=1");
    }
    Ok(url)
}

fn query_has_key(url: &str, key: &str) -> bool {
    let Some((_, query)) = url.split_once('?') else {
        return false;
    };
    query.split('&').any(|part| {
        let candidate = part.split_once('=').map_or(part, |(name, _)| name);
        candidate == key
    })
}

fn print_network_table(networks: &[NetworkInfo]) {
    println!("NAME\tSTATUS\tINTERFACE\tBITRATE\tERROR");
    for network in networks {
        println!(
            "{}\t{}\t{}\t{}\t{}",
            network.name,
            status_text(network.status),
            network.interface,
            format_bitrate(network.bitrate),
            network.error
        );
    }
}

fn format_bitrate(bitrate: u32) -> String {
    if bitrate == 0 {
        "-".into()
    } else {
        bitrate.to_string()
    }
}

fn status_text(status: NetworkStatus) -> &'static str {
    match status {
        NetworkStatus::Available => "available",
        NetworkStatus::Unavailable => "unavailable",
    }
}

fn escape_html(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}
