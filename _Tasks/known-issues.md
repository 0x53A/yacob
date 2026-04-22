# Known Issues

## Open

### Low-level SdoClient has no transfer timeout

The raw `SdoClient` state machine has no built-in timeout. This is largely mitigated by `SdoDriver` (the async high-level driver) which delegates timeout to the caller via `embassy_time::with_timeout` or similar, and `sdo_helpers` on Linux uses `block_on_with_timeout`. But embedded users calling `SdoClient` directly have no protection.

**Note:** The SDO *server* does have a 5s timeout. This issue is about the low-level *client* only. Prefer `SdoDriver` for new code.

### `SlcanTransport` is unix-only but not gated

The SLCAN transport uses `libc` ioctls, `/dev/tty` paths, and unix fd operations, but the module has no `#[cfg(unix)]` gate. It compiles on Linux only.

**Fix:** Add `#[cfg(unix)]` to the module declaration in `lib.rs`, or gate the whole `slcan` module behind a feature flag.

### `AccessKind::Const` exists but is rejected by the parser

The `dsl.rs` parser rejects `const` as an access type with an error message saying to use `ro`. But the `AccessKind::Const` variant still exists.

**Fix:** Either remove the variant or support `const` (which in CANopen means read-only and the value never changes — semantically the same as `ro` for our purposes).

### Block SDO not yet tested via interop tests

Block upload and download are implemented in the server but the interop test suite only exercises expedited and segmented transfers. python-canopen supports block transfers, so adding interop tests would be straightforward.

### LSS FastScan not implemented

The LSS slave supports Switch Mode Global, Switch Mode Selective, Configure Node ID, Store Configuration, and identity inquiries. FastScan (command 0x51) is not implemented.

### `EmcyProducer` only queues one pending EMCY frame

`EmcyProducer::set_error()` overwrites the pending frame. If called twice before the next `Node::process()`, the first EMCY is silently lost. In burst-error scenarios (e.g. multiple sensors failing simultaneously), this drops diagnostics.

**Fix:** Replace `pending: Option<CanFrame>` with a small `heapless::Deque<CanFrame, 4>` and drain all pending frames in `Node::process()`.

### `BlockDownloadState::total_len` is stored but never validated

The server stores the expected download size from the block initiate frame but never validates that the actual received data matches it. Per CiA 301 the server should abort if the sizes don't match.

## Ergonomic Improvements (backlog)

### `TryFrom<u8> for NodeId`

`NodeId::new(x).unwrap()` is the only way to create a `NodeId`. A `TryFrom<u8>` impl would be more idiomatic and enable `let node: NodeId = 5u8.try_into()?;`.

### `NodeConfig` has no `Default`

Users must always specify every field including `identity: LssIdentity::default()` even when LSS is unused. A `Default` impl or builder would reduce boilerplate.

### PDO config requires verbose heapless Vec construction

Setting up TPDO/RPDO configs requires manually creating `heapless::Vec`, calling `.push()` with `.unwrap()`. Helper methods or a small builder would improve ergonomics.

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
