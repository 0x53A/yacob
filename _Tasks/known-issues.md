# Known Issues

This is the canonical index of open defects and debt in `canopen_rs`.
It describes the current state only. It was last verified against the working
tree on **2026-11-04** (HEAD `f3da028` plus uncommitted changes).
Section 1.1 and its sub-sections come from four review passes on that day
(K01–K31); everything else was spot-checked then.
The 2026-09-07/08 fix sessions (`review-2026-09-0*.md`) were reviewed and
committed on 2026-11-05.

Fix records and history live in the dated review files and in git
(`git log -p -- _Tasks/known-issues.md`):
- [review-2026-09-07](review-2026-09-07.md)
- [review-2026-09-08](review-2026-09-08.md)
- [review-iterations-2026-09-08](review-iterations-2026-09-08.md)
- the drive-simulator findings ([09-07](drive-simulator-findings-2026-09-07.md), [09-08](drive-simulator-findings-2026-09-08.md))

The pre-2026-11 version of this file, including its "Fixed" sections, is in
`git show f3da028:_Tasks/known-issues.md`.

Feature and conformance omissions are tracked in
[`conformance-matrix.md`](conformance-matrix.md). Stateful protocol designs are
expanded in [`protocol-reliability-followups.md`](protocol-reliability-followups.md)
and [`sync-feature-audit.md`](sync-feature-audit.md).
[Architecture notes](architecture-review-2026-09-07.md) are proposals, not
decisions.

Downstream users (TurnGrip, RES gateway) pin older revisions. Infer their
behavior from their pin, not from this file.

Severity: **high** means protocol work or user data can be lost silently, or the
feature does not work at all. **medium** means a spec deviation or a reliability
gap with a workaround. **low** means a footgun or cosmetic issue.

---

## 1. Open defects

Suggested fix order (proposal, not a decision):
1. Silent data loss with small, local fixes: K20, K27, K12, K16, K13.
2. Generator correctness: K21, K28, K29, K14, K15, K22.
3. Protocol behavior: K04, K24, K23, K01, K09, R05.
4. Larger design work: K02 (streaming OD access), R14/application models,
   A-items.

### 1.1 Found in the 2026-11-04 architecture review

**K01 — high — OD 0x1001 is not wired to the EMCY error register.**
`EmcyProducer` keeps its own `error_register`. `set_error`/`clear_error` never
update the OD's 0x1001 field, so SDO reads and PDO mappings of 0x1001 return the
static default. Only `node.error_register()` is correct. 0x1003 (pre-defined
error field) is not maintained by the stack, and 0x1014/0x1015 (EMCY COB-ID and
inhibit time) are not supported. The 2026-09-07 review described this as an
"ownership boundary". It is still a conformance defect: CiA 301 requires
0x1001 to reflect the device's error state.

**K02 — high — 0x1021 Store EDS cannot be served for realistic devices.**
- The SDO server transfers through a fixed `[u8; 889]` buffer, and the generated
  0x1021 read returns `ValueTooLong` above that.
  `interop-tests/vcan_node.eds` deflates to about 1.9 kB, so the flagship
  example cannot serve its own EDS.
- `SdoDriver` hardcodes `SdoClient::<256>` (`sdo/driver.rs`), so the host side
  cannot read more than 256 B regardless of the server.
- The same ceilings apply to every large `domain<N>`/`octet_string<N>`. The
  CLAUDE.md example `domain<1024>` is not fully transferable.

A streaming (chunked) OD read/write interface is needed rather than a bigger
buffer.

**K03 — low — 0x1022 Store Format value 1 is in the CiA-reserved range.**
Values 1–127 are reserved by CiA 301. A manufacturer-specific "deflate" format
must use 128–255.

**K04 — high — `SdoDriver` never sends an abort.**
On a protocol error, toggle mismatch, `TooLarge`, or timeout, the driver
returns an error without sending an SDO abort. `SdoClient` has no abort-frame
builder. The server channel stays busy until its 5 s timeout, so an immediate
retry fails or interleaves with the stale transfer.

**K05 — high — `Dcf::apply_remote` fails on any real EDS/DCF and skips the important parts.**
- `Dcf::parse` collects every entry with a `ParameterValue` **or**
  `DefaultValue`, including read-only and const objects (0x1000, 0x1018, …).
  Access type is not checked. `apply_remote` stops at the first ReadOnly abort.
  `apply` (local) silently skips such entries. The two methods have
  opposite failure policies (skip vs. stop).
- PDO communication and mapping objects are skipped entirely.
- Every `$NODEID` value is skipped, although the DCF node ID is parsed.
- EDS default values are applied as if they were configured values.

**K06 — medium — Reset semantics deviate from CiA 301.**
- Reset Communication re-reads the *current* OD (`sync_pdo_from_od`,
  `sync_heartbeat_from_od`), so SDO-modified communication parameters survive
  the reset. The spec restores power-on or stored values.
- Reset Application is identical to Reset Communication apart from the event
  type: no application OD values are reset. In `request_reset` the two match
  arms are identical.
- The reset sequence is copied in three places (`request_reset` and the two
  NMT arms in `dispatch_frame`).
- 0x1010/0x1011 (store/restore) and a default/stored/current value model do
  not exist. A proper fix needs that design first.

**K07 — medium — RTR transmission types 252/253 are accepted but dead.**
The DSL (`rtr_sync`, `rtr_event`), EDS import, and `TransmissionType` accept
them. `CanFrame` cannot represent RTR (`new_remote` returns `None`), the
transports drop RTR frames, and the TPDO engine has no RTR path. A TPDO with
one of these types silently never transmits. Either reject them at all
configuration boundaries or implement RTR.

**K08 — medium — The EMCY pending queue drops silently.**
`EmcyProducer::pending` holds 4 frames and pops the oldest when full. It can
drop an error-reset EMCY (0x0000) or the first error of a burst. The drop is
not counted (`events_dropped` does not include it).

**K09 — medium — PDO config write policy does not follow the valid bit.**
The SDO server rejects all 0x1400–0x1BFF writes while Operational
(`server.rs`, `is_pdo_config_index`). CiA 301 instead allows changes to a
disabled PDO (bit 31 set) in any state, and rejects changing the COB-ID of an
*enabled* PDO (abort 0x0609_0030) even in Pre-Operational. The current rule is
both too strict and too loose.

**K10 — low — Smaller defects.**
- **Block upload end confirmation** (`handle_block_upload`, sub-command 1)
  clears `self.transfer` unconditionally. A stray frame ends any active
  transfer on the channel, including a segmented download, without a state
  check.
- **Block upload CRC:** the initiate response always claims CRC support and
  ignores the client's CRC flag.
- **`CobId::tpdo`/`rpdo`:** PDO numbers above 3 silently fall back to the
  PDO1 base.
- **`ParsedCobId::pdo_num`** is 0-based, while `PdoNumber` is 1-based.
- **`sync_pdo_from_od`:**
  - It is a `pub fn` that can be called mid-operation.
  - It accepts reserved transmission types 241–251 and ignores comm-param
    bit 30.
  - It silently skips mapping entries it cannot read.
- **RPDO COB-IDs:** two enabled RPDOs on the same COB-ID are not rejected; the
  first match wins.

**K11 — suspected — UDP multicast transport self-echo.**
`UdpMulticastTransport` enables `multicast_loop` and does not filter its own
datagrams. A SYNC-producing node applies its pulse locally (by design) and then
again from the echo. Unverified: depends on python-can's sender behavior and
the interop test setup.

### 1.1b Found in the 2026-11-04 second review pass

**K12 — high — DSL records have no sub0.**
For `record { ... }` entries the generated OD has no `(index, 0)` read arm and
no meta entry: `read(0x1018, 0)` returns `NotFound`, `lookup` returns `None`,
and SDO aborts with 0x0602_0000. Arrays do have sub0. The exported EDS still
declares `[1018sub0]` (DefaultValue = highest sub), so the device contradicts
its own EDS. Generic masters and python-canopen commonly read sub0 first. This
affects every record, including the mandatory 0x1018 identity object.
Minor related issue: for sparse records `sub_count` returns the highest
subindex, and loops such as the 0x1016 duplicate check iterate `1..=count`
over holes.

**K13 — medium — OD error → SDO abort code mapping is wrong.**
(`od_error_to_abort`, `sdo/protocol.rs`)
- `OdError::ValueTooLong` maps to 0x0504_0005 (out of memory). For writes it
  should be 0x0607_0012 (data type length too high).
- `AbortCode::DataTypeMismatch` is 0x0609_0043, which is not a CiA 301 code.
  It should be 0x0607_0010 (length mismatch), or 0x0607_0012/0x0607_0013.
- `OdError::HardwareError` maps to 0x0800_0000 (general) instead of
  0x0606_0000 (hardware error).
- A segmented/block download whose length disagrees with the declared size
  aborts with 0x0800_0020 (general transfer error). CiA 301 has 0x0607_0012
  (too long) and 0x0607_0013 (too short) for this.
- 0x0609_0011 (subindex does not exist) is never emitted: a missing subindex
  of an existing object aborts as "object does not exist".
- `AbortCode` lacks several standard codes, including 0x0607_0010/12/13,
  0x0606_0000, 0x0604_0047, 0x0609_0036 and 0x0800_0023/24.

**K14 — medium — The typed EDS SDO client has wrong signatures and gaps.**
(`sdo_client_codegen`, extends A09)
- `i8`/`i16` objects are read and written as `u8`/`u16`.
- `i32` reads return `i32` but writes take a `u32` (`write_u32`).
- `i64`/`u64`/`f64`/strings/domains get no typed methods at all; they are
  skipped silently. `SdoDriver` has no `read_i8/i16/i64/u64/f64`/bool helpers.

**K15 — medium — EDS import fails open.**
(`canopen-derive/src/eds_parser.rs`)
- An unparseable `DefaultValue` becomes `0`.
- `$NODEID+…` defaults outside the PDO records (e.g. 0x1014, 0x1200) become
  `0` silently.
- An unknown `AccessType` becomes `ro`; `const` imports as `ro`, so the
  re-export loses `const`.
- Entries with an unsupported `DataType` or no `ParameterName` are dropped
  without a diagnostic.
- Record vs. array is decided by a heuristic (≥4 elements, all-zero
  defaults, string token compare), not by `ObjectType`/the sub data types.
- `sanitize_name` collisions are possible, and `capacity` overrides are
  per index, not per subindex.

A proc macro can `compile_error!` on every one of these; silently
generating a different device is the worst option.

**K16 — medium — `SdoPort` overflow neither drains nor counts.**
(`can_router.rs`)
- `drain_overflow()` returns `&self.overflow` and does not clear it. Code
  written like the CLAUDE.md example (`for frame in port.drain_overflow()`)
  re-processes old frames on every call.
- The buffer is 16 frames. Once full, every newer frame is dropped with no
  counter, and it stays full until `clear_overflow()` is called. On a bus
  with cyclic PDOs one segmented transfer fills it.
- `SdoPort` only knows the default SDO channel (`0x580 + id`), not
  additional servers.

**K17 — low — `WebSocketTransport` resource shape.**
(`canopen-linux/src/websocket_transport.rs`)
- The RX path is an unbounded `std::sync::mpsc` channel. A consumer that
  stops polling grows memory without limit; the TX side is bounded.
- Each transport creates a full multi-threaded tokio runtime (one worker
  per core) plus a thread, for a single socket. A `current_thread` runtime
  is enough.
- Undecodable messages (EFF/RTR/ERR or malformed) are dropped without a
  counter.

**K18 — low — `pdo_client_from_eds!` decodes partial signed mappings
without sign extension.** An `i8`/`i16`/`i32` object mapped with fewer
bits than its width decodes as a positive number (4-bit `-1` → `15`). Only
`i24` with a full 24-bit mapping is sign-extended. Whether the OD side
(`copy_pdo_bits` into the RPDO target) behaves the same way is untested;
the two sides should share one rule.

**K19 — low — Shared bus does not see its own transmissions.**
`run_pump` publishes received frames only. Frames sent through the
`TxQueue` are not published locally, so a monitor or logger subscribed to
the same `SharedCanBus` does not see the master's own NMT/SDO/SYNC traffic.
SocketCAN loops these back to other sockets, so the bus behaves differently
from a second socket on the same interface. This should be a documented
choice or an option.

### 1.1c Found in the 2026-11-04 third review pass

**K20 — high — A too-long write wipes a variable-length object.**
Generated `write()` for `visible_string<N>`/`octet_string<N>`/`domain<N>`
(heapless) calls `clear()` first and then `push_str`/`extend_from_slice`.
When that fails, it returns `ValueTooLong` (SDO aborts), but the field is
already empty. Verified: write `"abc"`, then write 10 bytes into a
`visible_string<8>`. The result is `Err(ValueTooLong)` and the stored value
is `""`. The SDO server never checks `max_size` up front, so any oversized
SDO download destroys the current value. Writes must check the length first
(or build into a temporary).

**K21 — medium — The exported EDS has invalid `DefaultValue`s.**
`eds_export` writes the default expression's token stream verbatim. Verified:
`-5` → `DefaultValue=- 5`, `true` → `DefaultValue=true`,
`1 << 4` → `DefaultValue=1 << 4`. None of these is a CiA 306 value. Our own
importer turns them into `0` (K15), so export → import round trips silently
change defaults. Evaluate the literal at macro time, or reject non-literal
expressions.

**K22 — medium — The exported EDS is incomplete.**
- The auto-registered 0x1021/0x1022 exist in the OD but are not in the EDS.
  (The implicit default SDO server has no 0x1200 OD record either, which is
  allowed but means tools can't discover it from the OD.)
- `[DeviceInfo]` has only `VendorName=` and `ProductName`. `VendorNumber`,
  `ProductNumber`, `RevisionNumber`, `BaudRate_*`, `SimpleBootUpSlave`,
  `Granularity`, `NrOfRXPDO`/`NrOfTXPDO` and `LSS_Supported` are missing.
  Strict tools (CANeds, some master configurators) reject the file or guess.
- Every user index below 0x2000 is listed under `[MandatoryObjects]`.
  CiA 306 reserves that list for 0x1000, 0x1001 and 0x1018; the rest belong
  in `[OptionalObjects]`.
- Array sub-entries all get the array's name as `ParameterName` and
  `DefaultValue=0`.

**K23 — medium — LSS identity is not taken from 0x1018.**
`LssSlave` uses `NodeConfig::identity`. `NodeConfig::from_od` sets it to
`LssIdentity::default()` (all zeros) even when the OD declares 0x1018. By
default, every node therefore answers an LSS selective switch for
0/0/0/0, and the LSS identity can disagree with the identity readable via
SDO. This is one more instance of the dual-source pattern (A07).

**K24 — medium — RPDOs longer than their mapping are dropped silently.**
`RpdoEngine::process_with_drop_count` requires `DLC == mapped length`
exactly. A frame with extra bytes is consumed and ignored. CiA 301 (and
CANopenNode) process a longer PDO using the leading bytes, and use EMCY
0x8210 (PDO not processed, length error) / 0x8220 (length exceeded) as
diagnostics. Masters that pad PDOs to 8 bytes do exist. In either direction
the application gets no event or counter, so it cannot report 0x8210/0x8220
itself. Check the exact CiA 301 wording when fixing.

**K25 — low — LSS selective-switch state is not sequence-bound.**
Vendor/product/revision matches are latched independently and cleared only
by a serial request or a global switch. An interrupted selection sequence
leaves stale partial matches that a later sequence can complete. CiA 305
expects the four requests as one ordered sequence.

**K26 — low — `sdo_helpers` discard foreign traffic and busy-poll.**
`sdo_upload`/`sdo_download` wrap the transport in a fresh `SdoPort` per call
and drop its overflow, so every non-SDO frame received during the transfer
is lost to the caller. `block_on_with_timeout` polls in a 100 µs sleep loop
(a no-op waker). `sdo_upload` allocates 889 B although the driver is capped
at 256 B (K02).

### 1.1d Found in the 2026-11-04 fourth review pass (derive crate focus)

**K27 — high — RPDO partial-width mappings never reach the OD.**
`apply_mapped` writes `bytes_for_bits(bit_length)` bytes, but generated
`write()` requires the object's exact size. Any RPDO mapping whose byte
length is shorter than the object is rejected with `DataTypeMismatch`, and
the error is swallowed: no OD write, no event, no counter. Verified: a
`u32` mapped with 16 bits and a `u16` mapped with 8 bits stay 0 after a
matching frame (`consumed = true`, 0 events). Such mappings come from EDS
import and runtime remapping (`validate_mapping_target` does not check the
width). Bools (1 bit → 1 byte) and full-width mappings work, so the test
suite does not notice. The TPDO side works (it reads the whole object and
copies the leading bits), so the two directions disagree. CLAUDE.md
documents partial mappings as supported.

**K28 — medium — EDS-driven macros do not track their input file.**
`object_dictionary_from_eds!`, `sdo_client_from_eds!`, `pdo_client_from_eds!`
and `device_from_eds!` read the EDS with `std::fs::read_to_string` and emit
no `include_bytes!`/`include_str!`. Cargo therefore does not know the
dependency: editing the EDS does not rebuild the crate, and the generated
OD/client stays stale until something else triggers a rebuild. The usual fix
is to emit `const _: &[u8] = include_bytes!("<abs path>");` in the expansion.

**K29 — medium — The DSL accepts conflicting and impossible declarations.**
All verified to compile without a diagnostic:
- The same index declared twice (`[0x2000] a` and `[0x2000] b`). The first
  match arm wins; `b` is unreachable over SDO.
- A user entry at a generated index (e.g. `[0x1800] c: u8` next to
  `tpdo[1]`, or `[0x1021]`). The user entry shadows sub0 of the generated
  record or the Store-EDS object; reading 0x1800:00 then returns the user
  value instead of the highest subindex.
- An RPDO mapping a `ro` object and a TPDO mapping a `wo` object. The runtime
  remapping validator (`validate_mapping_target`) rejects exactly these, so
  the static and dynamic rules disagree. At runtime the RPDO write is
  silently refused and the TPDO frame cannot be built ("retried forever",
  §1.2).
- A record may declare its own `[0]` entry, which works around K12 but can
  disagree with the real highest subindex.

Most macro errors are also `panic!`s without a span, rather than
`syn::Error`s pointing at the offending token.

**K30 — withdrawn.** Reported as "fixed-size `read()` panics on a short
buffer". The uncommitted 2026-09-08 change already guards this (a metadata
lookup for `buf.len() < 8`, regression
`generated_od_reads_reject_short_buffers_without_partial_writes`). The only
remaining gap is objects without metadata, i.e. record sub0 (K12).

**K31 — low — `#[export_eds(path)]` is a silent build-time side effect.**
The proc macro writes into the source tree on every expansion (including
rust-analyzer runs) and ignores write errors (`let _ = fs::write`). Using
`MyOd::EDS` from a build script or a test is the conventional alternative.

### 1.2 Carried over from earlier reviews (spot-checked 2026-11-04, all still open)

**R05 — high — Transmit acceptance and displacement.**
- Boot-up (`process()` ignores the `transmit` result) and LSS replies still
  consume state before the transport accepts the frame. SDO, heartbeat, EMCY,
  async TPDO and SYNC paths are two-phase.
- `Ok(Some(displaced))` is discarded by `Node`, `NbCanAsync` and `run_pump`.

See [protocol-reliability-followups §1](protocol-reliability-followups.md#1-protocol-state-is-consumed-before-transmit-acceptance).

**R10 — high — `process()`/pump work is unbounded.**
- `Node::process` drains RX until idle before timers, and stamps every drained
  frame with one timestamp.
- `run_pump` drains RX, then all TX. While retrying one stuck TX it services
  neither RX nor stop, for up to 500 ms (`TX_STUCK_TIMEOUT`).

**DSIM-02 — medium — Late RX can conceal an elapsed heartbeat timeout.**
RX is drained before heartbeat expiry checks, so a queued heartbeat overwrites
`last_seen_us` of an already expired lease. Details:
[drive-simulator-findings-2026-09-08](drive-simulator-findings-2026-09-08.md).

**R11 — high — Shared-bus teardown and SDO channel ownership.**
- `TxQueue`/`Subscription` have no closed state. `TxQueue::send` blocks forever
  once the pump has exited, and `SubscriptionPort::Error` is `Infallible`.
- `publish` runs subscriber filters under the subscriber-list mutex. Wakeups
  were moved out, but a panicking filter still poisons the lock.
- Fanout is not SDO channel arbitration. Two clients on one server channel
  receive each other's segments.

**R12 — medium — Browser transport delivery.**
`canopen-web` `send` returns `()` and swallows missing-socket and JS errors.
The RX `VecDeque` is unbounded, and there is no TX buffered-amount guard.

**R13 — medium — Integrated LSS is not a complete commissioning path.**
`Node` neither drains nor exposes `LssSlave` events. Configure Node ID changes
only the LSS object's ID. Store Configuration is acknowledged without
persistence. FastScan (0x51) is not implemented.

**R14 — high for commands — OD notifications are not an ordered command stream.**
`next_change()` reads values at drain time: writes 1, 2, 3 can be seen as
3, 3, 3, and overflow drops events. RPDO writes bypass `validate_write`.
The replacement design is in [application-models](application-models.md).

**R15 — medium — Configuration validation is fragmented.**
Manual config, generated config, EDS import, DCF and SDO writes do not share
one validator. Public engine mutation (`sync_pdo_from_od`, `config_mut`) can
bypass the runtime guards. This overlaps K05, K09, K10 and the
runtime-reconfiguration item below.

**Runtime reconfiguration keeps stale phase and pending state.**
Live PDO/SYNC changes do not reset counters, pending frames, latches, deadlines
or inhibit timestamps. See
[protocol-reliability-followups §4](protocol-reliability-followups.md#4-runtime-reconfiguration-needs-transition-semantics).

**An invalid TPDO configuration is retried forever.**
If `build_pdo_frame()` fails permanently, the event latch and timer stay armed
and construction is retried on every poll. There is no disable or diagnostic
policy.

**Block download does not recover a sequence gap.**
The download aborts with 0x0504_0003 instead of ACKing the last contiguous
seqno. Upload does recover. See
[protocol-reliability-followups §5](protocol-reliability-followups.md#5-block-download-aborts-where-the-spec-asks-for-retransmission).

**SYNC gaps (postponed).**
Missing: TPDO start value, type-0 event timer, missing-SYNC monitoring and
extended-frame SYNC. Details:
[sync-feature-audit](sync-feature-audit.md).

**SDO server busy-channel semantics are unspecified.**
- An initiate replaces an active transfer on the same channel.
- Block-download data aliases command bytes.
- Eight protocol/state abort paths still answer with `0x0000:00`, which cancels
  unrelated client transfers.
- There are no multi-client collision tests.

**SDO client timeouts.**
- Raw `SdoClient` has no timeout.
- `SdoDriver::*_timed` has a wake source only with the `embassy` feature;
  other executors must re-poll.
- `canopen_linux::sdo_helpers::block_on_with_timeout` is a whole-transfer
  deadline, not a progress (silence) timeout. TurnGrip/EE GW-08 depends on this.

**Firmware updater post-erase status.**
After erase, the updater sends one long-lived status read (30 s/120 s). A
single lost response fails the update. Retrying is deliberately avoided
because late responses could be mistaken for new ones; this needs SDO
transfer correlation first. See
[protocol-reliability-followups §6](protocol-reliability-followups.md#6-the-updaters-post-erase-status-read-has-no-retry).

---

## 2. Architecture and complexity debt

These items are not defects. They raise the cost and risk of every future
change. They were found in the 2026-11-04 review.

**A01 — The legacy `dirty_set` / `DIRTY_SET` path.**
`notify_changed` sets the per-TPDO latch *and* pushes into `dirty_set`.
`latch_sync_events`/`poll_with` re-latch from `dirty_set`. The latch is
authoritative, so the set, the const generic, and the `DIRTY` parameter on the
engine can be removed.

**A02 — Const-generic explosion.**
`Node<OD, TPDO, RPDO, SDO, EVT_QUEUE, DIRTY_SET>` is repeated across three
`OdGuard` impls, `SharedNode` and the tests. Remove A01, and move queue sizes
into one config type or associated consts on the OD.

**A03 — `od_mut()` clones the whole OD.**
`OdGuard` requires `OD: Clone` and copies the entire OD, including strings and
domains, to diff the TPDO-mapped fields on drop. This costs RAM and stack on
MCUs. Snapshot only the mapped bytes, or rely on `notify_changed`.

**A04 — COB-ID validation is scattered and layered upside down.**
The checks live in:
- `sdo_cob_ids_are_unique`
- `reserved_cob_ids` (capacity 32, with a "truncated" flag)
- `sync_cob_id_is_unique`
- the `SdoCobIds`/`ReservedCobIds`/`SdoContext` types passed into `SdoServer`

The SDO server thereby validates node-level PDO, heartbeat and SYNC
semantics. A single node-owned COB-ID plan table, plus one node-level
pre-commit hook for SDO writes, would replace these and fix K09 and the
duplicate-RPDO case.

**A05 — SDO server memory.**
Every `TransferState` variant holds `[u8; 889]`, multiplied by each additional
server. Initiate upload, block upload and download commit make further
889-byte stack copies. See K02 for the functional side; streaming OD access
would remove both.

**A06 — The derive macro unrolls PDO records.**
Each PDO generates separate read, write and metadata arms per sub-entry: up to
64 mapping subs × 512 PDOs. The validation logic is copied into every
generated OD. A generic PDO record type in core, with table-driven access,
would cut code size and compile time.

**A07 — Dual PDO identity.**
There is a CiA PDO number (`od_number`) and an internal dense slot
(`pdo_od_offset`, `pdo_number_for_slot`, an `is_placeholder` heuristic).
Using the explicit number everywhere removes the conversion code.

**A08 — Five event channels.**
`event_queue`, `hb_event_queue`, `reset_event`, LSS events (not exposed) and
the embassy signal. `events_dropped` mixes two of them. Address this together
with R14 / the application-models redesign.

**A09 — `SdoDriver` duplication and gaps.**
- `upload`/`download` duplicate the `*_timed` variants; the plain ones could
  delegate.
- There is no block transfer and no way to target additional server channels.
- `NbCanAsync` busy-yields via `wake_by_ref`, which burns CPU on std.

**A10 — Three INI/EDS implementations.**
`canopen-derive/eds_parser.rs`, `canopen-core/dcf.rs` and `eds_export.rs` each
carry their own type and `$NODEID` handling. A shared `canopen-eds` crate
(usable from both the proc macro and runtime code) would back R15 and K05.

**A11 — `canopen-core` mixes device and host code.**
The no_std device stack sits next to std/host modules: `bus`, `slcan`, `dcf`,
the deprecated `can_router`/`CanDemux` (still about 660 lines) and the
heuristic `events` decoder. Split it into device, master and host crates, and
delete the deprecated router instead of keeping it.

**A12 — Documentation and test organization carry history.**
- `CLAUDE.md` is long and mixes reference with status.
- `_Tasks` is about 5.7k lines; `sync-aware-transmit-scheduling.md` is about
  1.1k lines for an unimplemented feature.
- Doc comments narrate history ("legacy", "no longer").
- Tests are grouped by review session (`tests/review_20260907.rs`) instead of
  by module.

Future agents read this history as requirements.

**A13 — Small leftovers.**
- `unreachable!()` after an `assert!` in `sync_config_from_od`.
- `HeartbeatMonitorState::Disabled` is never assigned to a monitor.
- Redundant `unsafe impl Send/Sync` on `MailboxTransport`.
- Strict `cargo clippy --workspace --all-targets -D warnings` fails with about
  30 mechanical lints in `canopen-core` and `canopen-derive` (same-type casts,
  manual `contains`/`div_ceil`, missing `Default`, `Result<_, ()>`).

---

## 3. Ergonomic backlog

- **Dummy PDO mappings in the DSL.** The engines and EDS import/export support
  dummy mappings (0x0001–0x0007), but `object_dictionary!` has no syntax for
  them (e.g. `dummy<bits>`).
- **`TryFrom<u8> for NodeId`.** The only constructor is `NodeId::new(x)`.
- **`NodeConfig` has no `Default` or builder.**
- **Building PDO configs needs verbose `heapless::Vec` push/unwrap.**
- **Additional SDO channels.** EDS import of 0x1201+ server records, and
  OD-managed SDO client channels (0x1280+), are missing.
- **Arrays cannot declare non-zero per-element defaults.**
- **No `SdoServer::is_busy()` or active-transfer accessors** for diagnostics.
