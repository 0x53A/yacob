# canwsd-dump

A minimal `candump` for canwsd WebSocket networks: connects to exactly one
remote canwsd network over WebSocket and prints received CAN frames in
candump's format. No vcan, no root — it only listens and prints, so it is the
quick way to check whether frames are flowing on a remote bus (unlike
`canwsd attach`, which maps a remote bus onto a local vcan and needs sudo).

The public interface (WS URL, wire format, filter syntax) is defined in the
`canwsd-proto` crate. This is a pure client — it never transmits.

## Usage

```sh
canwsd-dump --remote ws://host:8080/api/networks/can0
canwsd-dump --remote ws://host/api/networks/ddu -t d -n 100
canwsd-dump --remote wss://host/api/networks/ddu -t A -a
canwsd-dump --remote ws://host/api/networks/ddu -i
canwsd-dump --remote ws://host/api/networks/ddu --filter 0x181:0x7ff
canwsd-dump --remote ws://host/api/networks/ddu --jsonl > frames.jsonl
```

Flags mirror `candump` semantics for the subset implemented:

- `-t <a|d|z|A>` — timestamp column: absolute / delta / zero (since first) /
  absolute-with-date
- `-i` — data bytes as binary (bits) instead of hex
- `-a` — append an ASCII rendering of the data bytes
- `-n <COUNT>` — terminate after COUNT frames
- `--filter <id:mask,...>` — receive filter (socketCAN semantics)
- `--jsonl` — one JSON object per frame instead of candump text (mutually
  exclusive with `-t`/`-i`/`-a`)

The interface column shows the network name parsed from the URL path.

## Notable behavior

- **Timestamps are client-side arrival times.** The canwsd wire format carries
  no bus timestamp, so every `-t` value is when the frame came off the
  WebSocket — subject to network latency/buffering, not a kernel/bus RX time.
  Fine for "are frames flowing", not for bus-accurate timing. Documented in
  the `-t` help and `format.rs`.
- **`--filter` is enforced client-side as well as sent to the server.** The
  `?filter=` query param asks the server to filter (saves bandwidth on capable
  servers), but the frames are also filtered locally so the output is correct
  even against servers that ignore the param (some embedded canwsd servers do).
  Error frames bypass id filters (kernel/candump semantics).
- Server status text frames (`overflow`, `bus_error`) are printed to **stderr**;
  a WS close makes the tool exit non-zero (like candump without `-D`). No
  auto-reconnect in v1.
- Output is flushed per line so redirected/piped output stays timely; a broken
  pipe (e.g. `| head`) exits quietly.

## Format fidelity

`format.rs` is a line-by-line port of can-utils `snprintf_long_canframe`
(`lib.c`) restricted to classic CAN (the only thing canwsd carries), plus the
surrounding line assembly and `sprint_timestamp` from `candump.c`. Unit tests
pin the exact byte output (id padding, spacing, `[len]`, RTR/error/ASCII/binary
views).

## Not implemented (candump features deliberately left out)

- `--file`/`-l`/`-f` (log to file): use `> file` redirection instead. A
  canplayer-compatible `-L` log format could be added later.
- `-e` human-readable error-frame decode, `-D` reconnect, `-S`/`-8`/`-x`/`-c`,
  `-H` hardware timestamps (impossible over the wire), multiple interfaces.

## Dependencies

- canwsd-proto (shared interface definition)
- tokio, tokio-tungstenite (async WS client)
- clap (CLI), chrono (local time for `-t A`)
