# canwsd

CAN WebSocket daemon — bridges socketCAN interfaces over WebSocket.

The public interface (REST paths, wire format, filter syntax, JSON command
types) is defined in the `canwsd-proto` crate, shared with all clients and
alternative servers (e.g. embedded devices exposing a virtual bus). This tool
is the Linux/socketCAN implementation of that interface.

## Architecture

- REST: `GET /api/networks` → list available CAN interfaces
- WS: `GET /api/networks/<name>?filter=id:mask,id:mask` → upgrade to WebSocket
- Binary WS messages: exactly one CAN frame each, variable length (see below)
- Text WS frames: JSON commands (`set_filter`, `clear_filter`) and status messages
- Per-interface dedicated CAN reader thread, tokio broadcast to WS clients
- Per-client userspace filter + kernel `CAN_RAW_FILTER` set to union of all clients
- No clients connected → kernel filter drops all frames
- Generic CAN bridge: EFF/RTR/ERR frames are forwarded as-is; malformed WS
  messages are logged and ignored, not a disconnect reason

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
