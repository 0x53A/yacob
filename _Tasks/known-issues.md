# Known Issues

## Open

### `SharedCanBus::publish` runs subscriber filters under the subscriber-list lock

(2026-07-07 code review.) `publish()` holds the global subscriber-list `Mutex`
while invoking every subscription's `FrameFilter` closure and pushing into the
per-subscription queues. Two consequences:

1. A **panicking filter** (e.g. out-of-bounds index into `frame.data()`)
   unwinds inside the pump thread with the list mutex poisoned. `run_pump`
   never returns `Err`, so the application's designed teardown path
   (`PumpError` → supervisor restart) is skipped; every later bus access
   panics on the poisoned lock in whatever thread touches it first.
2. Subscribe/unsubscribe (frequent in the fresh-subscription-per-SDO-op
   pattern) contends with the RX hot path for the same lock.

**Fix:** snapshot the live `Arc<SubscriptionShared>`s under the lock, run
filters and pushes on the snapshot outside it (prune on the next pass), and
either `catch_unwind` around filters or document the panic-free contract on
`FrameFilter`.

### `TxQueue::send` blocks forever once the pump is gone

(2026-07-07 code review.) If the pump thread has exited (transport error —
the exact no-lifecycle teardown scenario the bus design prescribes) and the
queue is full, `send()` waits on the `space` condvar forever: `try_pop` is
never called again. `Subscription::recv` documents its equivalent hazard;
`send` does not, and there is no non-polling escape.

**Fix:** give `TxQueue` a `close()` (set by the pump on exit, `notify_all`)
and make `send` return `Result<(), Closed>`; or at minimum add a
`send_timeout` and a doc warning matching `recv`'s.

### Sync-type RPDOs are applied immediately instead of buffered until SYNC

`RpdoEngine::process()` writes mapped values to the OD at frame reception
regardless of transmission type. Per CiA 301, RPDOs with transmission type
0..=240 must buffer received data and apply it on the next SYNC (coordinated
updates). The stm32-node DSL comment describing buffer-until-SYNC is intent,
not implementation; `on_sync()` exists only on the TPDO side.

**Fix:** pending buffer per sync-type RPDO in `RpdoEngine`, applied in SYNC
handling. Planned as part of the application-models work — see
`application-models.md` ("SYNC-based RPDOs across the models").

### `EmcyProducer` latches the GENERIC bit forever

(2026-07-14, noticed during RPDO deadline design.) `set_error()` unconditionally
ORs in `error_register::GENERIC`, but `clear_error(bits)` only clears the bits
it is given — so after any error, GENERIC stays set unless the app explicitly
clears it (or calls `clear_all()`). Consequence: the register never returns to
0 through per-bit clears, and the "error reset" EMCY (0x0000) is never emitted
on recovery. CiA 301 intends GENERIC as "set while any error condition is
present", i.e. derived, not latched.

**Fix (later):** derive GENERIC (set while any other bit is set, or when an
error with no specific bit is reported; cleared when the last one clears) so
`clear_error(COMMUNICATION)` after e.g. an RPDO-timeout recovery can reach
register 0 and emit the standard error-reset frame. Touches
`emcy_producer_set_clear` test semantics.

### Low-level SdoClient has no transfer timeout

The raw `SdoClient` state machine has no built-in timeout. This is largely mitigated by `SdoDriver` (the async high-level driver) which delegates timeout to the caller via `embassy_time::with_timeout` or similar, and `sdo_helpers` on Linux uses `block_on_with_timeout`. But embedded users calling `SdoClient` directly have no protection.

**Note:** The SDO *server* does have a 5s timeout. This issue is about the low-level *client* only. Prefer `SdoDriver` for new code.

### LSS FastScan not implemented

The LSS slave supports Switch Mode Global, Switch Mode Selective, Configure Node ID, Store Configuration, and identity inquiries. FastScan (command 0x51) is not implemented.

### `SdoServer` busy-channel behavior is underspecified

If a second initiate request arrives while a segmented or block transfer is active on the same SDO channel, the server should reject the second request without clearing the active transfer when possible. This matters for multiple diagnostic clients sharing the default SDO channel.

**Fix:** Add explicit busy-state handling for initiate upload/download/block requests during active transfers. Abort the second request using its requested index/subindex. Preserve the active transfer unless the incoming frame is a valid-looking continuation or a true channel-level error.

### SDO protocol/state aborts often use `0x0000:00`

Several SDO server protocol/state error paths respond with abort index/subindex `0x0000:00`. That is appropriate for some channel-level errors, but it makes multi-client diagnostic tools cancel unrelated transfers unnecessarily.

**Fix:** Prefer the active transfer's index/subindex or the incoming request's index/subindex whenever known. Reserve `0x0000:00` for genuinely channel-level or unparseable errors. (2026-07-04: block download paths now use the active transfer's index/subindex; other paths still pending.)

### SDO timed transfers on non-embassy wake-driven executors

`SdoDriver::upload_timed`/`download_timed` check the clock deadline on every poll and, with the `embassy` feature, arm an `embassy_time::Timer` so the timeout fires on a silent bus. On a wake-driven executor *without* the `embassy` feature there is still no wakeup source for the deadline; the executor must re-poll periodically (the canopen-linux blocking helpers do).

### SDO collision behavior lacks focused tests

There are tests for ordinary transfers and some block sequence errors, but not for multi-client same-channel collision behavior.

**Fix:** Add tests for:
- unrelated initiate request while a transfer is busy,
- same-register initiate request while a transfer is busy,
- ~~abort filtering in `SdoClient`~~ (done 2026-07-06, see Fixed section),
- valid-looking continuation frame collision behavior.

## Ergonomic Improvements (backlog)

### No DSL syntax for dummy PDO mappings

(2026-07-16.) The PDO engines support dummy mappings (padding via static data
type indices 0x0001–0x0007: TPDOs emit zeros, RPDOs skip) and EDS
import/export round-trips them, but the hand-written `object_dictionary!` DSL
cannot declare one — useful for alignment padding now that bools map as
1 bit. Would need e.g. `dummy<bits>` in the PDO mapping list;
`PdoMappingDef.raw_mapping` is already plumbed through codegen, so only the
parser needs the syntax.

### `TryFrom<u8> for NodeId`

`NodeId::new(x).unwrap()` is the only way to create a `NodeId`. A `TryFrom<u8>` impl would be more idiomatic and enable `let node: NodeId = 5u8.try_into()?;`.

### `NodeConfig` has no `Default`

Users must always specify every field including `identity: LssIdentity::default()` even when LSS is unused. A `Default` impl or builder would reduce boilerplate.

### PDO config requires verbose heapless Vec construction

Setting up TPDO/RPDO configs requires manually creating `heapless::Vec`, calling `.push()` with `.unwrap()`. Helper methods or a small builder would improve ergonomics.

### Configurable and additional SDO channels

The stack only supports the predefined default SDO channel (`0x600 + node_id` / `0x580 + node_id`). Classic CANopen supports additional SDO server/client channels via communication parameter objects such as `0x1201+` and `0x1280+`.

**Improvement:** Make SDO client/server COB-IDs configurable, support multiple independent `SdoServer` state machines on a node, and expose the corresponding OD communication parameter entries.

### Expose SDO server busy state for diagnostics

`SdoServer` does not expose whether a segmented/block transfer is active or which object it targets.

**Improvement:** Add `is_busy()` and active transfer metadata accessors so applications and diagnostic tooling can avoid starting optional SDO work while a local server channel is occupied.

## Fixed (2026-07-06)

### `SlcanTransport` is unix-only but not gated

**Fixed in:** `canopen-linux/src/lib.rs` — module declaration and re-export
are now behind `#[cfg(unix)]`.

Follow-up refactor: the SLCAN protocol driver moved to
`canopen_core::slcan::SlcanTransport<P: SerialPort>` (sans-IO, `no_std`,
tested against an in-memory fake port). Only the unix termios/DTR backend
(`UnixSerialPort` + the `open()`/`open_raw()` init helpers) remains in
`canopen-linux` behind `#[cfg(unix)]`. `IoPort<T>` (core, `std` feature)
adapts any `std::io::Read + Write` — e.g. the `serialport` crate on
Windows (`COM1`) or macOS. The `!HUPCL` fast-reopen trick stays a
unix-backend detail.

### `AccessKind::Const` exists but was rejected by the parser

The DSL now accepts `const` as an access type (parsed via `parse_any` since
`const` is a Rust keyword). It behaves like `ro` for reads/writes; the meta
entry carries `AccessType::Const` and the exported EDS declares
`AccessType=const` (valid per CiA 306; the EDS importer already mapped
`const` back to read-only).

**Fixed in:** `canopen-derive/src/dsl.rs`, `canopen-derive/src/eds_export.rs`.

### `EmcyProducer` only queued one pending EMCY frame

`set_error()` overwrote the pending frame, dropping diagnostics in
burst-error scenarios. Now a `heapless::Deque<CanFrame, 4>` queues frames
(oldest dropped on overflow — later frames carry the accumulated error
register) and `Node::process()` drains all pending frames.

**Fixed in:** `canopen-core/src/emcy.rs`, `canopen-core/src/node.rs`.

### `SdoClient` treated every abort response as fatal

While a transfer is active, aborts whose index/subindex match neither the
active object nor channel-level `0x0000:00` now return the new
`SdoClientResult::IgnoredAbort` and leave the transfer running; `SdoDriver`
keeps waiting. Covered by unit tests (unrelated abort ignored + transfer
completes, matching abort cancels, `0x0000:00` cancels).

**Fixed in:** `canopen-core/src/sdo/client.rs`, `canopen-core/src/sdo/driver.rs`.

## Fixed (2026-07-04 code review)

### Block download broke at the End frame and on seqno ≥ 32

The dispatch condition for routing block-download segments sniffed byte-pattern
ranges of byte 0. The End Block Download frame (0xC1) was misrouted into the
segment handler (aborting every block download at completion), and non-final
segments with seqno 32..127 (whose top bits alias other command specifiers)
fell through to normal command dispatch, corrupting the transfer.

**Fixed in:** `canopen-core/src/sdo/server.rs` — `BlockDownloadState` now
tracks an explicit `awaiting_end` phase: while receiving sub-blocks, every
frame except an exact `0x80` client abort is a segment; after the final
segment is acked, normal dispatch resumes so End Block Download is routed
correctly. Covered by roundtrip and >31-segment unit tests plus the
python-canopen interop test.

### Block download sub-block ACK used the wrong command specifier

The ACK was sent as `0x40` (bare `2 << 5`); CiA 301 specifies SCS=5 with
ss=2, i.e. `0xA2`. python-canopen aborted every block download on the first
sub-block ACK.

**Fixed in:** `canopen-core/src/sdo/server.rs`.

### Block upload sub-block ACK was never accepted

The client's ACK arrives with sub-command 2 (`0xA2 & 3`), but the handler
only matched sub-commands 0/1/3 (ACK handling was wrongly folded into the
"start" branch), so every block upload died with InvalidCommandSpecifier
after the first sub-block.

**Fixed in:** `canopen-core/src/sdo/server.rs` — sub-command 2 (ACK) and 3
(start) are now separate branches.

### Block SDO CRC used the wrong init value and was never verified

CRC ran with init 0xFFFF; CiA 301 (and python-canopen) use CRC-16/XMODEM with
init 0. Block uploads failed the client's CRC check. On download the CRC also
wrongly included last-segment padding and the client's CRC was never checked.

**Fixed in:** `canopen-core/src/sdo/server.rs` — init 0 everywhere; download
CRC is computed over the trimmed data at End Block Download and verified when
the client negotiated CRC (aborts with new `AbortCode::CrcError`).

### `BlockDownloadState::total_len` is now validated

Declared size vs. received size mismatch aborts with `DataTransferError`
per CiA 301.

### Multi-sub-block downloads aborted on the second sub-block

`state.seqno` was never reset after a sub-block ACK, so the next sub-block's
first segment (seqno 1) failed sequence validation. Currently only reachable
if `MAX_BLKSIZE` or the 889-byte buffer changes, but fixed defensively; a
non-final segment that no longer fits the buffer now aborts with
`OutOfMemory` instead of being silently truncated.

### Expedited download without size indication rejected small objects

The generated OD write arms were tightened to exact-length checks, but the
server passed all 4 expedited data bytes when the size bit was clear, so
writes to u8/u16/bool objects from masters that omit the size bit failed with
DataTypeMismatch. The server now clamps to the object's size from OD metadata
(CiA 301: server uses the object's known length). Array element write arms
were also aligned to the same exact-length policy as scalar/record arms.

**Fixed in:** `canopen-core/src/sdo/server.rs`, `canopen-derive/src/codegen.rs`.

### Oversized `SdoClient::start_download` hung the async driver

When the payload exceeded `BUF`, `start_download` returned a fabricated abort
frame; `SdoDriver::download` transmitted it and waited forever for a response
(or returned a misleading `Timeout`). `start_download` now returns
`Result<CanFrame, ()>` and the driver surfaces `SdoError::TooLarge`
(mirrored in canopen-linux's `SdoError`).

### SDO timed transfers hung on silent bus under wake-driven executors

`receive_response_timed` only checked the deadline when polled and never
armed a wakeup, so `upload_timed`/`download_timed` hung forever on Embassy if
the peer never answered. With the `embassy` feature (which now pulls in
`embassy-time`), a timer is armed alongside the receive future. See the Open
note for the non-embassy caveat.

### Block SDO interop test enabled

`vcan_node` gained a `[0x2001] blob: octet_string<64>` object and the
python-canopen block download/upload roundtrip test now runs (27 interop
tests). The PDO with >8 mappings case is now a compile error in the DSL, and
generated EDS index lists are sorted again.

## Fixed (2026-04-04 code review)

### `NbCanAsync::transmit` hung forever on `WouldBlock`

The receive side correctly yielded with `wake_by_ref()` then retried, but the transmit side awaited `core::future::poll_fn(|_cx| Poll::Pending)` — a future that never completes. If the CAN TX buffer was temporarily full, the entire async SDO transfer would hang.

**Fixed in:** `canopen-core/src/sdo/driver.rs` — transmit now uses the same yield-then-retry pattern as receive.

### `SdoClient` doubled memory usage with two `[u8; BUF]` buffers

Upload and download each had their own `[u8; BUF]` array, but only one transfer direction is active at a time. At default BUF=256, this wasted 256 bytes per client — significant on MCU targets.

**Fixed in:** `canopen-core/src/sdo/client.rs` — merged into a single shared buffer.

### Dead `pending_response` field in `LssSlave`

Field was declared and initialized but never read or written. Leftover from an earlier design.

**Fixed in:** `canopen-core/src/lss.rs` — removed.

### Redundant position lookup in `DemuxSdoPort::receive`

Code computed position via `.position()` then discarded it and iterated the deque again linearly.

**Fixed in:** `canopen-core/src/can_router.rs` — simplified to a single pass.
