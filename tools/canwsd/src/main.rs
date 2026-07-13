mod bridge;

use axum::{
    Router,
    extract::{Path, Query, State, WebSocketUpgrade, ws},
    http::StatusCode,
    response::IntoResponse,
    routing::get,
};
use canwsd_proto::{CanFilter, NETWORK_WS_ROUTE, NETWORKS_PATH, NetworkInfo};
use clap::Parser;
use serde::Deserialize;
use std::sync::Arc;

use bridge::BridgeHub;

#[derive(Parser)]
#[command(about = "CAN WebSocket daemon — bridges socketCAN interfaces over WebSocket")]
struct Args {
    /// TCP listen address
    #[arg(short, long, default_value = "0.0.0.0:8080")]
    listen: String,

    /// CAN interfaces to expose. Use socketcan_name:alias to rename
    /// (e.g. can_arm:arm). Without alias, the socketcan name is used as-is.
    #[arg(required = true, value_parser = parse_interface_spec)]
    interfaces: Vec<(String, String)>,
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
}

async fn list_networks(State(hub): State<Arc<BridgeHub>>) -> impl IntoResponse {
    let networks: Vec<NetworkInfo> = hub
        .interface_names()
        .into_iter()
        .map(|name| NetworkInfo { name })
        .collect();
    axum::Json(networks)
}

async fn ws_handler(
    Path(name): Path<String>,
    Query(params): Query<WsParams>,
    State(hub): State<Arc<BridgeHub>>,
    ws: WebSocketUpgrade,
) -> impl IntoResponse {
    let initial_filter = match params.filter.as_deref() {
        Some(filter) => match canwsd_proto::filter::parse_filter_param(filter) {
            Ok(filter) => Some(filter),
            Err(e) => return (StatusCode::BAD_REQUEST, e).into_response(),
        },
        None => None,
    };

    ws.on_upgrade(move |socket| handle_ws(socket, name, hub, initial_filter))
        .into_response()
}

async fn handle_ws(
    socket: ws::WebSocket,
    interface: String,
    hub: Arc<BridgeHub>,
    initial_filter: Option<Vec<CanFilter>>,
) {
    if let Err(e) = hub.serve_client(socket, &interface, initial_filter).await {
        log::warn!("WS client for {interface} disconnected: {e}");
    }
}

#[tokio::main]
async fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args = Args::parse();
    for (can_name, exposed) in &args.interfaces {
        if can_name != exposed {
            log::info!("Interface {can_name} exposed as {exposed}");
        }
    }
    let hub = Arc::new(BridgeHub::new(&args.interfaces));

    let app = Router::new()
        .route(NETWORKS_PATH, get(list_networks))
        .route(NETWORK_WS_ROUTE, get(ws_handler))
        .with_state(hub);

    let listener = tokio::net::TcpListener::bind(&args.listen).await.unwrap();
    log::info!("canwsd listening on {}", args.listen);
    axum::serve(listener, app).await.unwrap();
}
