# Known Issues

This is the canonical index of open defects and debt in `canopen_rs`.
It describes the current state only. It was last verified against the working
tree on **2026-11-05**, after the fix pass recorded in
[review-2026-11-05-fixes](review-2026-11-05-fixes.md). Section 1.1 comes
from four review passes on 2026-11-04 (K01–K31; fixed numbers are not
reused); everything else was spot-checked then.
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

The 2026-11-05 fix pass closed K03, K04, K07, K08, K13, K17, K18, K20, K21,
K23, K24, K25, K27, K28 and K31, most of K12, K14, K15, K16, K22, K26 and
K29, the K01 0x1001 mirror, and the R05 boot-up case. What is left below either
needs a design decision or is a small remainder. Fix records:
[review-2026-11-05-fixes](review-2026-11-05-fixes.md).

Suggested order (proposal, not a decision):
1. Needs a design first: K02 (streaming OD access), K06 (default/stored value
   model), K05 (DCF semantics), K09 (PDO valid-bit write policy, ties into
   A04), R14/application models, R10/R11 (pump and bus lifecycle).
2. Decide, then small: K19 (bus self-publish), K10 duplicate RPDO COB-IDs.
3. Verify: K11.

### 1.1 Found in the 2026-11-04 review passes

**K01 — medium — EMCY objects beyond 0x1001 are not maintained.**
0x1001 now mirrors the EMCY error register: when the OD declares it as a
`u8` VAR, `set_error`/`clear_error`/`clear_all_errors` write it and notify a
change (so a TPDO mapping it fires). Still missing: 0x1003 (pre-defined error
field) is not maintained by the stack, and 0x1014/0x1015 (EMCY COB-ID and
inhibit time) are not supported.

**K02 — high — 0x1021 Store EDS cannot be served for realistic devices.**
- The SDO server transfers through a fixed `[u8; 889]` buffer, and the generated
  0x1021 read returns `ValueTooLong` above that (abort 0x0504_0005).
  `interop-tests/vcan_node.eds` deflates to about 2 kB, so the flagship
  example cannot serve its own EDS.
- `SdoDriver` uses `SdoClient::<256>` (`SdoDriver::MAX_TRANSFER`), so the host
  side cannot read more than 256 B regardless of the server.
- The same ceilings apply to every large `domain<N>`/`octet_string<N>`. The
  CLAUDE.md example `domain<1024>` is not fully transferable.

A streaming (chunked) OD read/write interface is needed rather than a bigger
buffer.

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

**K09 — medium — PDO config write policy does not follow the valid bit.**
The SDO server rejects all 0x1400–0x1BFF writes while Operational
(`server.rs`, `is_pdo_config_index`). CiA 301 instead allows changes to a
disabled PDO (bit 31 set) in any state, and rejects changing the COB-ID of an
*enabled* PDO (abort 0x0609_0030) even in Pre-Operational. The current rule is
both too strict and too loose.

**K10 — low — Smaller defects.**
- **`ParsedCobId::pdo_num`** is 0-based, while `PdoNumber` is 1-based. Only
  the monitoring classifier and the egui monitor use it.
- **`sync_pdo_from_od`:**
  - It is a `pub fn` that can be called mid-operation.
  - It ignores comm-param bit 30 and silently skips mapping entries it cannot
    read. (Reserved transmission types can no longer reach the OD: the DSL,
    EDS import and generated SDO writes reject 241–253.)
- **RPDO COB-IDs:** two enabled RPDOs on the same COB-ID are not rejected; the
  first match wins. Whether to forbid it (CiA 301 does not explicitly) belongs
  to the A04 COB-ID plan.

**K11 — suspected — UDP multicast transport self-echo.**
`UdpMulticastTransport` enables `multicast_loop` and does not filter its own
datagrams. A SYNC-producing node applies its pulse locally (by design) and then
again from the echo. Unverified: depends on python-can's sender behavior and
the interop test setup.

**K12 — low — Sparse records iterate holes.**
Records now have a generated sub0 (highest declared subindex). For sparse
records `sub_count` returns that highest subindex, and loops such as the
0x1016 duplicate check iterate `1..=count` over holes.

**K14 — low — No typed EDS client methods for strings and domains.**
`sdo_client_from_eds!` generates typed methods for every numeric type and
bool; `visible_string`/`octet_string`/`domain` objects only have the raw
upload/download escape hatch. Bounded by K02 on the driver side.

**K15 — low — EDS import remainder.**
- Entries with an unsupported `DataType` or no `ParameterName` are still
  skipped without a diagnostic (deliberately for now: vendor EDS files use
  types this stack cannot represent; a warning mechanism is missing).
- `$NODEID` defaults outside the PDO records (e.g. 0x1014, 0x1200) become 0.
- Record vs. array is decided by a heuristic (≥4 elements, all-zero
  defaults, string token compare), not by `ObjectType`/the sub data types.
- `sanitize_name` collisions are possible, and `capacity` overrides are
  per index, not per subindex.

**K16 — low — Router remainder.**
`SdoPort::drain_overflow()` now takes the retained frames and
`overflow_dropped()` counts losses. `SdoPort` still only knows the default SDO
channel (`0x580 + id`), and the deprecated `HeartbeatPort` overflow is
uncounted.

**K19 — low — Shared bus does not see its own transmissions.**
`run_pump` publishes received frames only. Frames sent through the
`TxQueue` are not published locally, so a monitor or logger subscribed to
the same `SharedCanBus` does not see the master's own NMT/SDO/SYNC traffic.
SocketCAN loops these back to other sockets, so the bus behaves differently
from a second socket on the same interface. This should be a documented
choice or an option.

**K22 — low — Exported EDS remainder.**
The implicit default SDO server has no 0x1200 OD record, so tools cannot
discover it from the EDS (allowed by CiA 301). `[DeviceInfo]` claims every
standard bit rate, since the stack does not know the driver's.

**K26 — low — `sdo_helpers` discard foreign traffic and busy-poll.**
`sdo_upload`/`sdo_download` wrap the transport in a fresh `SdoPort` per call
and drop its overflow, so every non-SDO frame received during the transfer
is lost to the caller. `block_on_with_timeout` polls in a 100 µs sleep loop
(a no-op waker).

**K29 — low — Macro errors without spans.**
Conflicting declarations are now rejected (duplicate indices, user entries on
generated indices, record `[0]`/duplicate subindices, RPDO mapping of
`ro`/`const` and TPDO mapping of `wo` objects, RTR transmission types). Many
other macro errors are still `panic!`s without a span rather than
`syn::Error`s pointing at the offending token.

### 1.2 Carried over from earlier reviews (spot-checked 2026-11-04, all still open)

**R05 — high — Transmit acceptance and displacement.**
- LSS replies still consume state before the transport accepts the frame.
  Boot-up, SDO, heartbeat, EMCY, async TPDO and SYNC paths are two-phase
  (boot-up retries on `WouldBlock` since 2026-11-05).
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

**A09 — `SdoDriver` gaps.**
(The plain/`*_timed` duplication was folded into one implementation with an
optional deadline on 2026-11-05.)
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
