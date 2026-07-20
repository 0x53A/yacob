//! `canwsd-dump` — a minimal `candump` for canwsd WebSocket networks.
//!
//! Connects to exactly one canwsd network over WebSocket and prints received
//! CAN frames in candump's format. Unlike `canwsd attach`, it needs no vcan
//! interface and no root — it only listens and prints, so it is the quick way
//! to check whether frames are flowing on a remote bus.

mod format;

use std::io::Write;
use std::time::{SystemTime, UNIX_EPOCH};

use clap::Parser;
use futures_util::StreamExt;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

use canwsd_proto::filter::parse_filter_param;
use canwsd_proto::{FILTER_QUERY_PARAM, WireFrame};

use format::{TimestampMode, Timestamps};

#[derive(Parser)]
#[command(
    about = "Dump CAN traffic from a canwsd WebSocket network (a candump for canwsd)",
    long_about = "Dump CAN traffic from a canwsd WebSocket network.\n\n\
        A simplified candump that listens to exactly one remote canwsd network \
        over WebSocket — no vcan, no root. Frames carry no bus timestamp over \
        the wire, so -t values are client-side arrival times (subject to \
        network latency), not kernel/bus RX times."
)]
struct Args {
    /// Remote canwsd WebSocket URL, e.g. ws://host:8080/api/networks/can0.
    #[arg(long)]
    remote: String,

    /// Timestamp column: a=absolute, d=delta, z=zero (since first), A=absolute
    /// with date. Reflects client-side arrival time (see above).
    #[arg(short = 't', value_name = "a|d|z|A", conflicts_with = "jsonl")]
    timestamp: Option<TimestampMode>,

    /// Print data bytes as binary (bits) instead of hex.
    #[arg(short = 'i', conflicts_with = "jsonl")]
    binary: bool,

    /// Append an ASCII rendering of the data bytes.
    #[arg(short = 'a', conflicts_with = "jsonl")]
    ascii: bool,

    /// Terminate after receiving COUNT frames.
    #[arg(short = 'n', value_name = "COUNT")]
    count: Option<u64>,

    /// Initial receive filter (`id:mask,id:mask,...`), applied server-side.
    #[arg(long)]
    filter: Option<String>,

    /// Emit one JSON object per frame (JSONL) instead of candump text.
    #[arg(long)]
    jsonl: bool,
}

#[tokio::main]
async fn main() {
    let args = Args::parse();
    if let Err(e) = run(args).await {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

async fn run(args: Args) -> Result<(), String> {
    let url = build_url(&args.remote, args.filter.as_deref())?;
    let iface = interface_name(&args.remote);

    // The query param above asks the server to filter; enforce it client-side
    // too so `--filter` is honored even against servers that ignore it (and to
    // match candump's guarantee that you only see what you asked for).
    let filters = match args.filter.as_deref() {
        Some(s) => Some(parse_filter_param(s).map_err(|e| format!("--filter: {e}"))?),
        None => None,
    };

    let (ws, _) = connect_async(&url)
        .await
        .map_err(|e| format!("connect {url}: {e}"))?;
    eprintln!("connected to {}", args.remote);

    let (_ws_tx, mut ws_rx) = ws.split();

    let mut timestamps = Timestamps::new(args.timestamp);
    let mut remaining = args.count;
    let stdout = std::io::stdout();
    let mut out = stdout.lock();

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => break,
            msg = ws_rx.next() => {
                let Some(msg) = msg else {
                    return Err("remote websocket closed".into());
                };
                match msg.map_err(|e| format!("receive websocket frame: {e}"))? {
                    Message::Binary(data) => {
                        let wf = WireFrame::decode(&data)
                            .map_err(|e| format!("decode websocket frame: {e}"))?;
                        // Error frames bypass id filters (kernel/candump semantics).
                        if let Some(filters) = &filters
                            && !wf.is_error()
                            && !filters.iter().any(|f| f.matches(wf.id_word()))
                        {
                            continue;
                        }
                        let now = SystemTime::now()
                            .duration_since(UNIX_EPOCH)
                            .unwrap_or_default();
                        let line = if args.jsonl {
                            format::format_jsonl(now, &wf)
                        } else {
                            let ts = timestamps.render(now);
                            format::format_line(&ts, &iface, &wf, args.binary, args.ascii)
                        };
                        // Broken pipe etc.: the reader (e.g. `head`) is gone — stop quietly.
                        if writeln!(out, "{line}").and_then(|()| out.flush()).is_err() {
                            break;
                        }
                        if let Some(n) = remaining.as_mut() {
                            *n -= 1;
                            if *n == 0 {
                                break;
                            }
                        }
                    }
                    // Server status (overflow, bus_error) arrives as JSON text.
                    Message::Text(text) => eprintln!("canwsd: {text}"),
                    Message::Close(close) => {
                        return match close {
                            Some(c) => {
                                Err(format!("remote websocket closed: {} {}", c.code, c.reason))
                            }
                            None => Err("remote websocket closed".into()),
                        };
                    }
                    Message::Ping(_) | Message::Pong(_) | Message::Frame(_) => {}
                }
            }
        }
    }
    Ok(())
}

/// Append `--filter` to the URL as the canwsd `filter` query parameter, or
/// reject `--filter` if the URL already carries one.
fn build_url(remote: &str, filter: Option<&str>) -> Result<String, String> {
    if !remote.starts_with("ws://") && !remote.starts_with("wss://") {
        return Err("--remote must start with ws:// or wss://".into());
    }
    let Some(filter) = filter else {
        return Ok(remote.to_string());
    };
    if query_has_key(remote, FILTER_QUERY_PARAM) {
        return Err(format!(
            "--remote already contains `{FILTER_QUERY_PARAM}`; do not also pass --filter"
        ));
    }
    let sep = if remote.contains('?') { '&' } else { '?' };
    Ok(format!("{remote}{sep}{FILTER_QUERY_PARAM}={filter}"))
}

fn query_has_key(url: &str, key: &str) -> bool {
    let Some((_, query)) = url.split_once('?') else {
        return false;
    };
    query
        .split('&')
        .any(|part| part.split_once('=').map_or(part, |(name, _)| name) == key)
}

/// The interface/network name shown in the output column: the last path
/// segment of the URL (before the query). Falls back to "can" for a URL with
/// no path (so a host:port is never mistaken for a network name).
fn interface_name(remote: &str) -> String {
    let after_scheme = remote
        .strip_prefix("ws://")
        .or_else(|| remote.strip_prefix("wss://"))
        .unwrap_or(remote);
    let path = after_scheme.split(['?', '#']).next().unwrap_or(after_scheme);
    let path = path.trim_end_matches('/');
    match path.rsplit_once('/') {
        Some((_, name)) if !name.is_empty() => name.to_string(),
        _ => "can".to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_url_rejects_non_ws() {
        assert!(build_url("http://x/api/networks/can0", None).is_err());
    }

    #[test]
    fn build_url_appends_filter() {
        assert_eq!(
            build_url("ws://h/api/networks/can0", Some("0x181:0x7ff")).unwrap(),
            "ws://h/api/networks/can0?filter=0x181:0x7ff"
        );
        assert_eq!(
            build_url("ws://h/api/networks/can0?errors=1", Some("1:2")).unwrap(),
            "ws://h/api/networks/can0?errors=1&filter=1:2"
        );
    }

    #[test]
    fn build_url_rejects_double_filter() {
        assert!(build_url("ws://h/x?filter=1:2", Some("3:4")).is_err());
    }

    #[test]
    fn interface_name_from_url() {
        assert_eq!(interface_name("ws://h:8080/api/networks/can0"), "can0");
        assert_eq!(interface_name("ws://h:8080/api/networks/arm?filter=1:2"), "arm");
        assert_eq!(interface_name("ws://h:8080/"), "can");
    }
}
