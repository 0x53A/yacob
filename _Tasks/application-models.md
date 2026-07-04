# Application Models — API Design

> **Status: design agreed (2026-07-04), not yet implemented.**
> Supersedes the event-queue/`next_change()` design direction explored earlier
> the same day (see api-evaluation.md Round 2 — the DSL/config/const work from
> that round stays; the typed event-drain APIs are replaced by this design).

## Problem

A single event API cannot serve both natures of OD entries:

- **State entries** (setpoints, outputs, process data): only the latest value
  matters. Intermediate values in a write burst are noise.
- **Command entries** (write-signature-to-trigger, program control): every
  write is a trigger; coalescing loses real events.

Both are standardized, not vendor quirks:
- CiA 301 0x1010/0x1011: write ASCII `"save"`/`"load"` to trigger; the value
  is never stored (reads return capability flags). A keyhole, not a variable.
- CiA 302 0x1F51 Program Control: command register with a state machine.
- CiA 402 controlword fault-reset: rising-edge sensitive by spec.

Key insight: the standards' mental model is that triggers fire **in the write
handler**, synchronously with the transfer — never by the application later
observing stored state. Commands therefore belong in the synchronous path;
state observation never needs to represent write history.

Instead of encoding state-vs-command per entry (DSL flags) or per source
(heuristics), the distinction is expressed by **which application model the
firmware chooses**. CAN classic at 500 kbit/s tops out at ~4000 frames/s
bus-wide, so even the "see every frame" model is trivial for an STM32G4.

## The three models

| # | Model | Topology | Semantics | Typical device |
|---|-------|----------|-----------|----------------|
| 1 | **Inline** | app owns the loop, no protocol task | edge — every frame, exact order, pre-commit introspection | command-style (fwupdate), gateways, sniffers |
| 2 | **Reactive** | protocol task + wake-on-change | level — consistent OD view + modified mask | low-power event-response nodes (stm32-node) |
| 3 | **Scan** | protocol task + app polls on own ticker | level — OD as shared process image | fixed-rate control loops, CiA 401-style I/O |

Models 2 and 3 share the same init path (node in `SharedNode`, spawn protocol
task) and compose freely (one task waits, another scans). Model 1 excludes the
background task; the app loop *is* the protocol loop.

## Model 1: Inline

```rust
loop {
    // Keeps the node alive while parked: services heartbeat TX, SDO
    // timeouts, event-timer TPDOs, SYNC. Protocol starves only while the
    // app computes between next_frame() and handle().
    let frame = node.next_frame(&mut can, &clock).await;

    match node.decode(frame) {
        Some(Msg::SdoWrite(w)) => {
            // Pre-commit: node still in the PREVIOUS state.
            // w: index, subindex, data; w.change::<NodeOdChange>() for typed view.
            if bad(&w) {
                node.abort(w, Abort::ValueRange);   // consumes w, sends SDO abort
            } else {
                node.handle(w.into());              // consumes w, commits + responds
            }
        }
        Some(msg) => node.handle(msg),              // Heartbeat, Sync, Emcy, Rpdo, Nmt, SdoTransfer…
        None => {}                                  // not ours; raw frame available for sniffing
    }
}
```

Decisions:
- **The app sees every frame** (including other nodes' heartbeats — enables
  app-level heartbeat consumers). Filtering can come later at transport level.
- **`decode` is an advisory view, `handle` is authoritative.** `decode` is
  `&self` (it must peek SDO server state — segment frames are meaningless
  without it). The decoded message *owns a copy of the raw frame* (CanFrame is
  Copy, 16 B); `handle` re-derives from the frame against current state, so a
  stale/reordered message cannot corrupt the protocol. No borrows → no
  conflict between holding the message and calling `&mut node` methods.
- **`SdoWrite` materializes only on the completing frame** (expedited, final
  segment, block end) — the first moment the full data exists; same veto point
  as `validate_write` today. Mid-transfer frames decode to a protocol-chatter
  variant the app forwards to `handle` unchanged.
- **Veto = choosing `abort` instead of `handle`.** `handle` is infallible.
  `abort` lives on the node and takes the `SdoWrite` payload (not the enum), so
  aborting a heartbeat is unrepresentable. Node is always the receiver:
  `node.handle(msg)` / `node.abort(w, code)`.
- **`#[must_use]` on the decoded message** — silently dropping an `SdoWrite`
  means a client timeout; the compiler should flag it. Explicit discard stays
  possible (`let _ =`).
- The SDO download **response is sent by `handle()`** (commit-then-confirm);
  deferred-response latency is bounded because the loop handles immediately.
- RPDO frames may carry several mapped fields → the decoded/typed view is
  plural (small fixed array or iterator of changes per frame). Applied
  atomically by `handle` (one frame = one apply; per-field ordering would be
  an invention, it doesn't exist on the wire).
- `validate_write` **stays** — it is the veto for models 2/3 where no app sits
  in the frame path.

## Model 2: Reactive

```rust
// Protocol task (unchanged): NODE.with(|n| n.process(&mut transport, &clock))
loop {
    NODE.wait_for_change(|od, changed| {
        if changed.led()      { led.set_level((od.led != 0).into()); }
        if changed.echo_in()  { /* … */ }
    }).await;
}
```

Decisions:
- **Closure access, no clone.** Target is STM32G-class RAM; ODs can contain
  `domain<1024>`+ fields, so `{ od: OD, modified }` by value is out. The
  closure runs under the lock (critical section — keep it short); the view is
  atomically consistent; no holding it across an await.
- **Modified mask, not a queue.** Generated `${Name}Mask` bitset over writable
  entries, maintained by protocol writes, cleared on delivery. Cannot
  overflow, cannot misorder, cannot replay — the honest data structure for
  "what changed since you last looked". No sequence claims at all.
- Wakeup via the existing `OdEventSignal` machinery, internalized (the app no
  longer wires the static signal by hand — `SharedNode` owns it).
- Commands cannot ride this model (two identical writes may be one wakeup).
  Not a flaw; documented rule: commands → model 1 or `validate_write`.

## Model 3: Scan

```rust
let mut ticker = Ticker::every(Duration::from_millis(10));
loop {
    ticker.next().await;
    let led = NODE.read(|od| od.led);                 // short critical section
    gpio.set_level((led != 0).into());
    NODE.update(|od| od.button = btn.is_low() as u8); // OdGuard diff → TPDO trigger
}
```

Decisions:
- Just two shorthands on `SharedNode`: `read(|od| …)` and `update(|od| …)`
  (routing through `od()` / `od_mut()` so the OdGuard TPDO auto-diff keeps
  working). Everything else already exists.
- Consistency rule (docs): one closure = atomic view; between closures an
  RPDO may land. Multi-field coherence → one closure.

## SYNC-based RPDOs across the models

General rule: **visibility and notifications follow OD commit time, not frame
arrival time.** Event-driven RPDOs (254/255) commit at reception; sync-type
RPDOs (0..=240) are buffered at reception and commit at the next SYNC. Hence:

- **Model 1**: the RPDO frame decodes on arrival (pending values are
  introspectable); `handle` buffers it. On `Msg::Sync`, `handle(sync)` applies
  all buffered RPDOs (and samples/sends sync TPDOs); the app reads the
  coherent post-SYNC OD *after* `handle`. Pre/post-SYNC comparison = read
  before/after `handle`.
- **Model 2**: mask bits are set at apply time → one SYNC applying several
  buffered RPDOs yields **one** wakeup whose closure sees all changes
  atomically — the coordinated-update semantics SYNC exists for.
- **Model 3**: scans between reception and SYNC see old values; the buffer is
  invisible. A loop that must run SYNC-aligned is model 2 with SYNC as the
  wake source (or model 1 on `Msg::Sync`), not scan mode.

**Implementation gap**: `RpdoEngine::process` currently applies values to the
OD immediately regardless of transmission type — buffer-until-SYNC does not
exist yet (the stm32-node DSL comment describes intent). Needs a pending
buffer per sync-type RPDO, applied in SYNC handling. Work item for the
implementation phase.

## Removals

The event queue and its drain APIs are the hybrid none of the models wants
(async access + edge-ish valueless events → the "five 5s" problem):

- `Node::next_event()`, `Node::next_change()`, `OdEvent`, `OdEventSource`,
  event queue + `events_dropped()`, `EVT_QUEUE` const generic.
- `Node::set_event_signal()` (internalized into SharedNode/model 2).

Kept from the 2026-07-04 ergonomics round:
- DSL: transmission-type keywords, unit-suffixed time literals.
- Address consts (`MyOd::LED: (u16, u8)`), `${Name}Node` alias.
- `NodeConfig::from_od` / `PdoConfigSource`, `tpdo_cob_id()`/`rpdo_cob_id()`.
- `SharedNode` (grows `read`/`update`/`wait_for_change`).
- `${Name}Change` enum — repurposed: pre-commit typed view in model 1
  (`SdoWrite::change()`, RPDO change iteration), and naming basis for the
  model-2 mask bits. `OdChanges` trait adapts accordingly.

## Implementation notes (code pointers & gotchas, captured 2026-07-05)

Recorded while the implementation context was fresh — line numbers approximate.

### Hidden coupling: PDO resync depends on the event queue

`Node::process()` (node.rs, "Check if any SDO write targeted PDO parameter
entries") detects runtime PDO reconfiguration by scanning **newly pushed
events** for index 0x1400..=0x1BFF with source Sdo, then calls
`sync_pdo_from_od()`. Removing the event queue silently breaks this unless
replaced. Cleanest fix: see "change sink" below.

### Suggested refactor: change sink instead of event deque

`SdoServer::process(...)` (three commit sites: expedited ~line 340, segmented
~440, block ~690, all via `push_event()`) and
`RpdoEngine::process_with_drop_count()` currently take
`&mut Deque<OdEvent, N>`. Replace with a small sink trait
(`fn note_write(&mut self, index: u16, subindex: u8, source)`) implemented by:
- the generated `${Name}Mask` (models 2/3) — Node holds mask, sets bit via
  generated `fn mask_bit(index, sub) -> Option<u16>`;
- a PDO-resync flag (folds the 0x1400..=0x1BFF check into the sink);
- a no-op/passthrough for model 1 (the app is in the frame path already).

### SharedNode signal internalization

`SharedNode` should own the wake signal as a field (drop the user-visible
`static EVENT_SIGNAL` + `set_event_signal()` wiring). The `&'static` problem
solves itself by taking `init(&'static self, node)` — SharedNode lives in a
static anyway, and `&'static self` methods can hand `&self.signal` to the
node. `wait_for_change(f)` must **loop while the mask is empty**: the signal
is a binary latch (`embassy_sync::Signal<CS, ()>`) and `OdGuard::drop` also
signals on app-side writes (kept — it hastens `process()`), so spurious
wakes are normal.

### Model 1 machinery

- `Node::process()` already splits internally: per-frame work is
  `dispatch_frame()`; the rest is time-driven (SDO timeout via
  `sdo_server.check_timeout()`, block-upload polling, heartbeat, EMCY, TPDO
  poll + `dirty_set` clear). Model 1 needs that split public-ish:
  `next_frame()` loops the time-driven part while awaiting RX.
- Async RX: the `AsyncCan` trait (sdo/driver.rs) already exists — model 1's
  `next_frame` can take `impl AsyncCan` + a Clock, embassy-time for the tick.
- `decode` needs read access to SDO transfer state (segment frames are
  meaningless without it) → add a peek accessor on `SdoServer` for the active
  transfer's (index, subindex, bytes so far).
- `abort(w, code)`: `SdoServer::abort_transfer()` already exists (used by
  `request_reset()`); needs a variant that takes an abort code and emits the
  abort frame (encode_abort is already there).
- `validate_write` call sites (the 3 commit sites above) stay — they are the
  veto for models 2/3.

### Sync-RPDO buffering (the known gap)

`RpdoEngine` has no `on_sync()` (only `TpdoEngine` does — called from the
SYNC dispatch in Node). Add `pending: [Option<(u8, [u8; 8])>; N]` (len +
frame data) to `RpdoEngine`; `process()` stores instead of applying when
transmission_type ≤ 240; new `on_sync(od, sink)` applies + notes writes.
Also recorded in known-issues.md.

### Mask codegen

The writable-entry iteration needed for bit assignment already exists in
codegen.rs — the loop that builds `change_variants`/`decode_arms` (search
"Generate typed change enum"). Reuse it so mask bits, change variants, and
address consts stay aligned by construction. Decide: one bit per array
*entry* vs per element (lean per entry — subindex granularity rarely matters
for wakeups, and it keeps the mask word-sized for typical ODs).

### Consumers to migrate when the queue APIs go

- `examples/stm32-node/src/main.rs` — `EVENT_SIGNAL` static,
  `set_event_signal`, `next_change` loop → model 2.
- `examples/stm32-fwupdate/src/main.rs` — same wiring + `next_event` +
  `events_dropped` → model 1 (it is the command-style device).
- `canopen-linux/examples/sensor_hub.rs` — `next_event` + `events_dropped`
  (doc comment references the queue) → model 3 + `validate_write`.
- `canopen-linux/examples/vcan_node.rs` — `next_event` → model 2 or 3.
- `canopen-core/tests/macro_test.rs` — `change_enum_decodes_events`,
  `node_next_change_drains_typed` rewrite against the new surface; node.rs
  unit tests around the event queue (`event_queue_overflow_drops_oldest`,
  drop-count tests) become mask/sink tests.
- CLAUDE.md documents `next_change()` and the event bullets — update alongside.

## Open details (decide during implementation)

- Exact `Msg` variant list and naming (`CanOpenMessage`?); shape of the RPDO
  multi-change view.
- `${Name}Mask` representation (u32/u64 vs `[u8; N]`) and generated accessors
  (`changed.led()` + iterator).
- Whether `next_frame` takes the transport/clock or `Node` holds them in
  model 1 (it owns the loop, so borrowing per-call like `process()` is fine).
- Migration of examples: stm32-node → model 2, stm32-fwupdate → model 1,
  sensor-hub style → model 3.
