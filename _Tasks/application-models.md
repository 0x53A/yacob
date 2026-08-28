# Application Models — API Design

> **Status: core design agreed (2026-07-04); public model APIs not yet
> implemented. Deferred SDO requirements added 2026-08-28; exact API open.**
> Supersedes the event-queue/`next_change()` design direction explored earlier
> the same day (see api-evaluation.md Round 2 — the DSL/config/const work from
> that round stays; the typed event-drain APIs are replaced by this design).
> Some prerequisites have since landed: synchronous RPDO buffering,
> queue-independent SDO config resynchronization, and an internal wake signal
> in `SharedNode`. The transitional event queue and drain APIs still remain.

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

Key insight: the standards' mental model is that triggers are **captured in the
write handler**, at the completing SDO/RPDO frame — never by the application
later observing stored state. Execution may finish immediately or be deferred,
but command delivery itself must preserve every write. State observation never
needs to represent write history.

The three application models decide how ordinary OD state is observed. They do
not turn a coalescing state mask into a command queue. Commands handled in the
frame path remain natural in model 1; commands that must finish later use the
separate per-SDO-server deferred-operation path defined below, which is
available in every model. This avoids encoding state-vs-command in DSL flags or
guessing it from the write source. CAN classic at 500 kbit/s tops out at ~4000
frames/s bus-wide, so even the "see every frame" model is trivial for an
STM32G4.

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
            } else if must_finish_later(&w) {
                let request = node.defer(w);        // consumes w, keeps channel busy
                jobs.start(request);                // do not await here
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
- On the immediate path, the SDO download **response is sent by `handle()`**
  (commit-then-confirm). On the deferred path, `defer()` takes ownership of the
  channel operation and the app must promptly return to the protocol loop;
  completion is reported later through the cross-model API below.
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
- Commands cannot ride the modified mask (two identical writes may be one
  wakeup). Immediate side effects may use `validate_write`; operations that
  finish later use the edge-preserving deferred-SDO path below. Neither is
  reconstructed from the OD state mask.

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

## Deferred SDO operations across the models

Some SDO accesses start work whose result is not available in the frame
handler: flash/EEPROM persistence, firmware operations, hardware acquisition,
or application-mediated uploads. CiA 301 defines no universal server response
deadline and no "still working" response; the client chooses its timeout. A
server may therefore answer later, but the SDO channel remains occupied and a
client may time out or abort before the application finishes. See the
[CiA SDO overview](https://can-cia.org/can-knowledge/sdo-protocol) and CiA's
[interoperability note](https://www.can-cia.org/fileadmin/cia/documents/publications/cnlm/previous_issues/17-4_cnlm.pdf),
which specifically calls out slow flash/EEPROM-backed SDO writes and matching
the client timeout to the server.

The application models must obey this invariant:

> Each SDO server channel may own one in-flight protocol transfer or deferred
> application operation, while the node continues servicing PDO, SYNC, NMT,
> heartbeat, EMCY, LSS, deadlines, timers, and every other SDO server channel.

The deferred operation is **protocol state, never blocking control flow**. A
model-1 handler must not await flash or other application work before returning
to `next_frame()`. Models 2/3 already have a protocol task; deferral hands work
to the application without stopping that task. With the default server plus
`SDO` additional servers, up to `SDO + 1` operations may be pending in parallel,
but never more than one on the same channel.

Conceptual per-server state:

```text
Idle
  -> ReceivingSegmentedOrBlock
  -> DeferredToApplication
  -> ResponseReady
  -> Idle
```

Decisions:

- Model 1 chooses reject/immediate/deferred handling in its frame path. Models
  2/3 need a synchronous OD access hook that can return an outcome equivalent
  to `Abort`, `Commit`, `Handled` (success without storing the request bytes),
  or `Deferred`. The existing `validate_write` remains useful for pure vetoes
  but cannot by itself express successful keyhole commands or deferral. Exact
  hook registration/codegen is still open.
- A download becomes deferrable only at the existing commit/veto point, when
  the complete payload is available (expedited request, final segment, or block
  end). No successful final response is sent yet. A deferred upload similarly
  occupies the channel until the application supplies data or an abort.
- `defer(...)` returns an opaque, copyable token identifying the server channel
  and a transfer generation. `complete_sdo(token, result)` accepts success or
  an SDO abort code. A generation check makes completion idempotently reject or
  ignore a stale token rather than answering a newer transfer.
- Completion queues the response in stack-owned state. Normal non-blocking
  transmit/retry processing sends it; an application completion never needs to
  spin on a full CAN driver.
- A client abort or node reset cancels the deferred protocol operation and
  invalidates its token. Application work may be physically irreversible and
  continue, but its late completion cannot emit a response. Cancellation
  observability for the worker is desirable, not required for protocol safety.
- A new initiate request on a busy channel must not replace or corrupt its
  active operation. Exact rejection behavior remains an API/protocol detail;
  other configured SDO server channels are unaffected.
- The existing five-second `SdoServer` timeout is inactivity protection for
  segmented/block exchanges. It is not automatically the deadline for
  application-deferred work. Any server-side deferred-operation deadline is a
  separate configurable policy; the effective interoperability limit remains
  the remote client's timeout.
- Model 1 receives the request inline and starts work before immediately
  returning to its frame loop. Models 2/3 expose pending deferred requests via
  a dedicated edge-preserving surface (one occupied slot per server), not the
  coalescing OD modified mask. The application takes a request exactly once and
  later completes its token.
- Payload access must not hold a borrow of `Node` across an await. The concrete
  API may copy small command data into application-owned storage or provide a
  short closure over stack-owned data before work is started; this is still an
  open API choice.

For operations expected to exceed common client timeouts, prefer a regular OD
command that responds once the job is accepted plus separate status/result OD
entries (optionally announced by TPDO/EMCY). Deferred confirmation is for cases
where SDO success genuinely means that the operation itself completed.

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

**Implementation status** (2026-08): buffer-until-SYNC now exists in the
engine, so the general rule above holds today for models built on `Node`.
`RpdoEngine` parks the frame of a synchronous RPDO at reception and
`RpdoEngine::on_sync` commits it; `Node::handle_sync` runs that before
sampling synchronous TPDOs. What is still model-dependent is the *surface*:
model 1's `Msg::Sync`/`handle` and model 2's single coalesced wakeup are the
parts not yet built — the OD-commit-time semantics they rely on are.

## Removals

The event queue and its drain APIs are the hybrid none of the models wants
(async access + edge-ish valueless events → the "five 5s" problem):

- `Node::next_event()`, `Node::next_change()`, `OdEvent`, `OdEventSource`,
  event queue + `events_dropped()`, `EVT_QUEUE` const generic.
- `Node::set_event_signal()` (internalized into SharedNode/model 2).

The per-server deferred-SDO slots are not a replacement event queue and are not
removed: their bounded cardinality follows the number of protocol channels,
and each occupied slot is authoritative SDO state that must eventually be
completed, aborted, or reset.

Kept from the 2026-07-04 ergonomics round:
- DSL: transmission-type keywords, unit-suffixed time literals.
- Address consts (`MyOd::LED: (u16, u8)`), `${Name}Node` alias.
- `NodeConfig::from_od` / `PdoConfigSource`, `tpdo_cob_id()`/`rpdo_cob_id()`.
- `SharedNode` (grows `read`/`update`/`wait_for_change`).
- `${Name}Change` enum — repurposed: pre-commit typed view in model 1
  (`SdoWrite::change()`, RPDO change iteration), and naming basis for the
  model-2 mask bits. `OdChanges` trait adapts accordingly.

## Implementation notes (code pointers & gotchas)

Originally captured 2026-07-05 and reconciled with the implementation on
2026-08-28. Exact line numbers are intentionally omitted because this refactor
cross-cuts `node.rs`, the SDO server, the PDO engines, and macro code generation.

### PDO resync is already decoupled from the event queue

This was a hidden coupling in the original implementation: runtime PDO
reconfiguration was detected by scanning newly pushed SDO events. It has been
fixed independently of the application-model refactor. Every successful SDO
download now records its committed `(index, subindex)` in `SdoServer`, and
`Node::handle_sdo_request()` consumes that marker to resynchronize PDO,
heartbeat, or SYNC configuration. Queue overflow therefore cannot suppress a
configuration update.

When the event queue is removed, preserve this direct committed-write path (or
fold it into the change sink below); do not reintroduce event-queue inspection.

### Suggested refactor: change sink instead of event deque

`SdoServer::process(...)` (three commit sites: expedited, segmented, and block,
all currently via `push_event()`) and
`RpdoEngine::process_with_drop_count()` currently take
`&mut Deque<OdEvent, N>`. Replace with a small sink trait
(`fn note_write(&mut self, index: u16, subindex: u8, source)`) implemented by:
- the generated `${Name}Mask` (models 2/3) — Node holds mask, sets bit via
  generated `fn mask_bit(index, sub) -> Option<u16>`;
- optionally, a configuration-write collector (or keep the existing direct
  `committed_write` marker alongside the sink);
- a no-op/passthrough for model 1 (the app is in the frame path already).

The PDO-resync bullet is no longer required to fix correctness—the direct
`committed_write` marker already does that—but a composite sink may absorb it
later if doing so keeps the control flow clearer.

### SharedNode signal internalization (partly complete)

`SharedNode` now owns the wake signal as a field, and
`init(&'static self, node)` installs it in the node. New firmware using
`SharedNode` no longer needs a user-visible `static EVENT_SIGNAL` or a direct
`set_event_signal()` call.

The current `SharedNode::wait_for_change()` is still the transitional bare
signal: callers wake and drain `Node::next_change()` under `with()`. Model 2
must replace that with `wait_for_change(f)`, backed by the generated modified
mask. It must **loop while the mask is empty**: the signal is a binary latch
(`embassy_sync::Signal<CS, ()>`) and `OdGuard::drop` also signals on app-side
writes (kept—it hastens `process()`), so spurious wakes are normal.

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

### Deferred-SDO machinery

- Add the OD access disposition needed by models 2/3 before replacing the
  event queue. It must select deferral synchronously at the completing frame;
  observing a later modified-mask wake is too late and loses repeated commands.
- Today each configured `SdoServer` already has independent segmented/block
  transfer state, and `Node` continues dispatching other protocols while those
  transfers are active. Extend that same per-server state machine with
  `DeferredToApplication` and `ResponseReady`; do not park `Node::process()` or
  the model-1 frame loop in an application future.
- The default and additional servers need a stable internal server number plus
  a wrapping generation counter for opaque completion tokens.
- The current response path attempts transmit immediately. Deferred completion
  needs one retained response per server so CAN backpressure cannot lose the
  confirmation and cannot force application code to retry it manually.
- Models 2/3 need a wakeable request-taking API independent of
  `${Name}Mask`. Since a busy server cannot accept a second deferred operation,
  one slot per server preserves every accepted request without an arbitrary
  queue capacity or overflow policy.
- Decide whether application cancellation is polled through the token, emitted
  on the same wake source, or only observed when `complete_sdo` reports a stale
  token. Protocol correctness requires only the last option.

### Sync-RPDO buffering (complete prerequisite)

This former gap is implemented. `RpdoEngine` holds one pending `CanFrame` per
slot; `process()` stores the latest frame instead of applying it when the
transmission type is 0..=240, and `on_sync()` commits all pending frames before
`TpdoEngine::on_sync()` samples the OD. Deadline monitoring is stamped at frame
reception. Leaving Operational and reset discard pending frames. Unit tests,
the node-level integration test, and python-canopen interop cover the behavior.

The application-model work still needs to replace the event deque passed to
`on_sync()` with the change sink/mask so model 2 emits one coalesced wake for a
multi-RPDO SYNC commit.

### Mask codegen

The writable-entry iteration needed for bit assignment already exists in
codegen.rs — the loop that builds `change_variants`/`decode_arms` (search
"Generate typed change enum"). Reuse it so mask bits, change variants, and
address consts stay aligned by construction. Decide: one bit per array
*entry* vs per element (lean per entry — subindex granularity rarely matters
for wakeups, and it keeps the mask word-sized for typical ODs).

### RPDO deadline monitoring rides the queue transitionally (added 2026-07-14)

RPDO deadline expiry (CiA 301 event timer, `rpdo[N](deadline = ...)`) pushes
an `OdEventSource::RpdoDeadline` event (index = comm param index, sub 0) —
deliberately the *secondary* surface. The durable primitive is the per-slot
level flag (`Node::rpdo_deadline_expired(n)` / `RpdoEngine::deadline_expired`),
which maps onto all three models: model 1 — `next_frame()`'s time-driven part
synthesizes a deadline message; model 2 — expiry signals the wake source, the
closure checks a flags word (a deadline is not a write, so it is not a mask
bit); model 3 — poll the flag. When the queue is removed, delete the event
push and wire those three surfaces instead; nothing else about the feature
changes (engine state, OD sub 5, EDS round-trip are queue-independent).

### Consumers to migrate when the queue APIs go

- `examples/stm32-node/src/main.rs` — already uses `SharedNode`'s internal
  signal, but its `wait_for_change` + `next_change` drain loop → model 2 mask
  closure.
- `examples/stm32-fwupdate/src/main.rs` — manual `EVENT_SIGNAL` wiring and a
  `next_event` loop (its module docs also advertise `events_dropped`) → model 1
  because it is a command-style device.
- `canopen-linux/examples/sensor_hub.rs` — `next_event` + `events_dropped`
  (doc comment references the queue) → model 3; its existing
  `#[validate_write(check_thresholds)]` remains the synchronous veto.
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
- Deferred-SDO API names and ownership: request/payload representation,
  `complete_sdo` result for stale tokens, worker-visible cancellation, busy
  channel abort behavior, optional server-side deadline, and whether deferred
  requests share a wake signal with model-2 state changes.
- Models-2/3 OD access-hook shape and codegen/registration: how an entry returns
  `Abort`/`Commit`/`Handled`/`Deferred` without requiring every node to use the
  inline model.
- Migration of examples: stm32-node → model 2, stm32-fwupdate → model 1,
  sensor-hub style → model 3.
