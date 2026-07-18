mod bridge;

use axum::{
    Router,
    extract::{Path, Query, State, WebSocketUpgrade},
    http::StatusCode,
    response::IntoResponse,
    routing::get,
};
use canwsd_proto::{NETWORK_WS_ROUTE, NETWORKS_PATH, NetworkInfo};
use clap::Parser;
use serde::Deserialize;
use socketcan::Socket;
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
    errors: Option<String>,
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

#[tokio::main]
async fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args = Args::parse();
    let hub = Arc::new(BridgeHub::new(&args.interfaces));
    if hub.interface_names().len() != args.interfaces.len() {
        eprintln!("duplicate network names in interface list");
        std::process::exit(1);
    }
    for (can_name, exposed) in &args.interfaces {
        let shown = if can_name == exposed {
            can_name.clone()
        } else {
            format!("{can_name} (as {exposed})")
        };
        match socketcan::CanSocket::open(can_name) {
            Ok(_) => log::info!("{shown}: available"),
            Err(e) => log::warn!("{shown}: not available: {e} (clients get 503 until it is)"),
        }
    }

    let app = Router::new()
        .route(NETWORKS_PATH, get(list_networks))
        .route(NETWORK_WS_ROUTE, get(ws_handler))
        .with_state(hub);

    let listener = tokio::net::TcpListener::bind(&args.listen).await.unwrap();
    log::info!("canwsd listening on {}", args.listen);
    axum::serve(listener, app).await.unwrap();
}
