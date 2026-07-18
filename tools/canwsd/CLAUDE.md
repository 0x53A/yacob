# canwsd

CAN WebSocket daemon — bridges socketCAN interfaces over WebSocket.

The public interface (REST paths, wire format, filter syntax, JSON command
types) is defined in the `canwsd-proto` crate, shared with all clients and
alternative servers (e.g. embedded devices exposing a virtual bus). This tool
is the Linux/socketCAN implementation of that interface.

## Architecture

- REST: `GET /api/networks` → list available CAN interfaces
- WS: `GET /api/networks/<name>?filter=id:mask,id:mask&errors=1` → upgrade to
  WebSocket. Connect errors before upgrade: 404 unknown name, 400 bad filter,
  503 interface cannot be opened (down/absent)
- Binary WS messages: exactly one CAN frame each, variable length (see below)
- Text WS frames: client → server JSON commands (`set_filter`, `clear_filter`),
  server → client JSON status messages (`ServerStatus` in canwsd-proto)
- **One CAN socket per WS client** (no shared socket, no fan-out): the kernel
  does the filtering (`CAN_RAW_FILTER` = exactly the client's filter; on any
  failure or >512 filters it degrades to accept-all + userspace filtering,
  never to over-filtering) and provides cross-client echo — kernel loopback
  delivers a client's TX to the other clients once it was actually transmitted
  on the wire; `RECV_OWN_MSGS` stays off, so a sender never sees its own frames
  (same semantics as a physical CAN node)
- Per-client reader task drains the socket into a 16384-frame (256 KiB, ~3 s
  at saturated 500 kbps) buffer so the small kernel rcvbuf (~200 frames) never
  overflows on WS hiccups. If the buffer fills anyway it is cleared completely
  and the client gets `{"status":"overflow","dropped":N}` — fresh start, not a
  stale replay. A client too slow to keep up long-term loses data audibly, not
  silently
- Bus death mid-session (read error, e.g. interface down): the client gets one
  `{"status":"bus_error","error":...}` text message, then a WS Close with
  application close code 4000 (`close_code::BUS_ERROR`). Reconnecting is the
  client's job; while the bus is down, connects are answered with 503. canwsd
  itself never exits over interface trouble
- Keepalive: server pings every 10s, drops clients silent for 30s (half-open
  connections would otherwise hold a socket with a wide filter open)
- Generic CAN bridge: EFF/RTR frames are forwarded in both directions (TX
  builds frames from decoded flags — `CanFrame::from_raw_id` chokes on flag
  bits and would reject them all); RTR DLC is preserved. CAN error frames
  (controller error reports) are opt-in per client via `?errors=1`
  (per-socket error mask; they bypass id filters, kernel semantics).
  Malformed WS messages are logged and ignored, not a disconnect reason

## Binary wire format (canwsd-proto `wire`)

Little-endian, `5 + DLC` bytes, one frame per WebSocket message:
- `[0..4]` u32: CAN ID (bit 31=EFF, bit 30=RTR, bit 29=ERR, bits 0-28=ID) — matches Linux socketCAN convention
- `[4]` u8: DLC (0-8)
- `[5..]` exactly DLC data bytes

## Build & Run

```sh
cargo build
cargo run -- --listen 0.0.0.0:8080 can0 can_arm
```

## Dependencies

- canwsd-proto (shared interface definition)
- axum (HTTP + WS)
- socketcan (Linux CAN)
- tokio (async runtime)
