# API Surface Evaluation

Findings from implementing three realistic scenarios against the canopen-rs API.
Several gaps were identified and **all 8 have been implemented**.

## Scenarios Implemented

1. **STM32 firmware-update node** (`examples/stm32-fwupdate/`) — CiA 302 programming objects, Embassy, flash programming over SDO, `#[validate_write]`, `request_reset()`
2. **Linux firmware uploader** (`canopen-linux/examples/firmware_upload.rs`) — Master-side tool, chunked SDO downloads, `NmtCommand::to_frame()`, CRC verification
3. **Multi-sensor hub** (`canopen-linux/examples/sensor_hub.rs`) — 4 temp + 2 ADC sensors, 2 TPDOs at different rates, EMCY alarms, `#[validate_write]` for threshold validation, `events_dropped()`
4. **Async master** (`canopen-linux/examples/async_master.rs`) — `CanDemux` for frame routing, `SdoDriver::upload_timed()` with timeout, heartbeat monitoring, `NmtState::from_heartbeat_byte()`

## What Works Well

### OdGuard auto-diff (sensor_hub)
Writing sensor values via `od_mut()` automatically detects changes and marks TPDO-mapped fields dirty. No manual `notify_changed()` needed. This is the single best ergonomic in the API — it makes event-driven TPDOs "just work" for the common case of periodically-updated sensor data.

### Per-TPDO event timers (sensor_hub)
Different TPDOs can have different event_timer values (100ms for temperatures, 250ms for analog). The `TpdoConfig` struct handles this cleanly. Inhibit time prevents flooding.

### object_dictionary! macro DSL (all scenarios)
The inline DSL is concise and readable. PDO mapping declarations (`tpdo[1](...) { field1, field2 }`) are well-designed — fields map by name, not by index, so refactoring is safe.

### Domain type for firmware chunks (stm32-fwupdate)
`domain<256>` backed by `heapless::Vec<u8, 256>` works well for receiving firmware chunks. Each SDO write replaces the domain content, which is exactly right for a chunked protocol.

### Blocking SDO helpers (firmware_upload)
`sdo_upload`/`sdo_download` with timeout wrapping the async driver is the right abstraction for CLI tools and test harnesses.

### Embassy integration (stm32-fwupdate)
`OdEventSignal` + `select()` gives a clean event-driven architecture. The node sleeps in WFI between events.

## API Gaps Found

### P1: SDO pre-write hook [DONE]

**Impact**: High — affects any application that needs input validation.

**What was done**:
- Added `ObjectDictionary::validate_write(&self, index, subindex, data) -> Result<(), OdError>` with a default impl that accepts all writes.
- SDO server calls `validate_write()` before `write()` at all three write sites (expedited, segmented, block). On `Err`, sends an SDO abort.
- Added `#[validate_write(fn_name)]` attribute to the `object_dictionary!` macro. The user defines a method on the struct; the macro wires it into the trait impl.
- Fixed a pre-existing bug: `SdoClient` was only matching 7 of 21 abort codes (rest mapped to `GeneralError`). All abort codes are now correctly parsed.

**Used by**: `stm32-fwupdate` (reject writes in error state), `sensor_hub` (reject `temp_high < temp_low`).

### P2: Self-initiated NMT reset [DONE]

**Impact**: Medium — affects firmware update, watchdog recovery, config-driven resets.

**What was done**: Added `Node::request_reset(ResetType)` and `ResetType` enum (`Application`, `Communication`). Aborts any in-progress SDO transfer, resets NMT state to Initializing, re-syncs PDO config. On the next `process()` call, the boot sequence runs (boot heartbeat + transition per `auto_start`).

**Used by**: `stm32-fwupdate` (reset after firmware validation).

### P3: SdoDriver timeout [DONE]

**Impact**: Medium — any async SDO caller can hang forever.

**What was done**: Added `SdoDriver::upload_timed()` and `download_timed()` that accept `timeout_us: u64` and `clock: &impl Clock`. Checks deadline before each `receive()` call. Returns new `SdoError::Timeout` variant. Uses the no_std `Clock` trait so it works on embedded (Embassy, bare metal) and Linux alike.

The original `upload()`/`download()` are unchanged for callers who handle timeout externally (e.g., `embassy_time::with_timeout`).

**Used by**: `async_master` example.

### P4: NMT command builder in canopen-core [DONE]

**Impact**: Medium — embedded masters can't send NMT commands without hand-crafting frames.

**What was done**: Added `NmtCommand::to_frame(target_node: u8) -> CanFrame`. Also added `NmtState::from_heartbeat_byte(u8) -> Option<NmtState>` as the decoder counterpart to `heartbeat_byte()`.

**Used by**: `firmware_upload` (NMT Pre-Op + Reset), `async_master` (NMT Start + heartbeat decoding).

### P5: SdoClient const generic buffer [DONE]

**Impact**: Low-Medium — limits firmware chunk size.

**What was done**: Changed `SdoClient` to `SdoClient<const BUF: usize = 256>`. Both `buf` and `download_buf` use `[u8; BUF]`. The overflow check uses `BUF` instead of hardcoded 256. Default stays 256 for backwards compatibility. `SdoDriver` uses the default internally.

### P6: Event queue overflow tracking [DONE]

**Impact**: Low — hard to debug when events are lost.

**What was done**: Added `event_overflow_count: u32` to `Node` and `events_dropped() -> u32` accessor. The count is incremented when the SDO server or RPDO engine pushes an event while the queue is already full (detected by checking `is_full()` around dispatch calls).

**Used by**: `sensor_hub` (prints warning in periodic status line).

### P7: Async heartbeat with timeout [DONE]

**Impact**: Low — only affects async masters.

**What was done**: Added `CanDemux::recv_heartbeat_timed(node, timeout_us, clock)` that returns `Ok(None)` on timeout instead of hanging forever. Extracted `take_buffered_heartbeat()` helper shared with the existing `recv_heartbeat()`.

**Used by**: `async_master` example (initial heartbeat wait with 5s timeout).

### P8: PDO mapping error messages [DONE]

**Impact**: Low — ergonomic improvement for macro users.

**What was done**: Improved all three PDO mapping error messages to include:
- Which PDO (e.g., `tpdo[1]` or `rpdo[1]`) references the field
- The OD index and subindex of the field
- A concrete fix suggestion with the exact syntax to add (e.g., `[1] temp_high: i16 = ..., rw, pdo;`)

## Summary Priority Matrix

| # | Gap | Impact | Effort | Status |
|---|-----|--------|--------|--------|
| P1 | SDO pre-write hook | High | Medium | **DONE** — `ObjectDictionary::validate_write()` |
| P2 | Self-initiated NMT reset | Medium | Low | **DONE** — `Node::request_reset(ResetType)` |
| P3 | SdoDriver timeout | Medium | Low | **DONE** — `upload_timed()` / `download_timed()` |
| P4 | NMT command builder in core | Medium | Low | **DONE** — `NmtCommand::to_frame(target)` |
| P5 | SdoClient const generic buffer | Low-Med | Medium | **DONE** — `SdoClient<const BUF: usize = 256>` |
| P6 | Event queue overflow counter | Low | Low | **DONE** — `Node::events_dropped()` |
| P7 | Async heartbeat waiting | Low | Low | **DONE** — `CanDemux::recv_heartbeat_timed()` |
| P8 | PDO mapping error message | Low | Low | **DONE** — include PDO name, index, fix suggestion |

## Bonus fixes

- **SdoClient abort code matching**: The client-side abort code parser was missing most abort codes (only 7 of 21 were matched). All codes are now correctly parsed. Pre-existing bug found during P1 testing.
- **`NmtState::from_heartbeat_byte()`**: Decoder for heartbeat state bytes (counterpart to `heartbeat_byte()`). Found missing while writing `async_master` example.
- **`#[validate_write(fn)]` macro attribute**: Wires user-defined validation into proc-macro-generated ODs. Discovered during example updates that the trait method alone wasn't enough — macro-generated ODs need first-class support.

## All items complete

All 8 identified API gaps have been addressed, plus 3 bonus fixes discovered during implementation.
