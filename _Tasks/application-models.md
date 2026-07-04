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

## Open details (decide during implementation)

- Exact `Msg` variant list and naming (`CanOpenMessage`?); shape of the RPDO
  multi-change view.
- `${Name}Mask` representation (u32/u64 vs `[u8; N]`) and generated accessors
  (`changed.led()` + iterator).
- Whether `next_frame` takes the transport/clock or `Node` holds them in
  model 1 (it owns the loop, so borrowing per-call like `process()` is fine).
- Migration of examples: stm32-node → model 2, stm32-fwupdate → model 1,
  sensor-hub style → model 3.
