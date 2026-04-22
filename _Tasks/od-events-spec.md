# OD Events — Reactive Object Dictionary

> **Status: Fully implemented.** All features described below are in canopen-core.
> The `OdGuard` RAII pattern (not in original spec) was also added for automatic
> change detection without manual `notify_changed` calls. Embassy async signals
> are implemented behind the `embassy` feature flag.

## Problem

The current OD model is shared memory: the protocol stack and the application both read/write struct fields directly. This means:

- The app has no way to know when a remote SDO write or RPDO changed a value (except polling)
- When the app changes a value, there's no trigger to send an event-driven TPDO
- No hooks for validation, side effects, or async notification

## Design

Two mechanisms, layered:

### 1. Event Queue (core, `no_std`, always available)

A `heapless::Deque<OdEvent, N>` inside the `Node`, populated automatically by the protocol stack. The app drains it in the main loop.

```rust
/// Emitted when the protocol stack modifies an OD entry.
#[derive(Clone, Copy, Debug)]
pub struct OdEvent {
    pub index: u16,
    pub subindex: u8,
    pub source: OdEventSource,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OdEventSource {
    Sdo,   // remote SDO download wrote this entry
    Rpdo,  // incoming RPDO mapped to this entry
}
```

**Where events are generated:**

- `SdoServer::process()` — after a successful expedited or segmented download, push an event
- `RpdoEngine::process()` — after writing mapped values from an incoming PDO frame, push one event per mapped entry

**How the app consumes them:**

```rust
// In the main loop, after node.process():
while let Some(event) = node.next_event() {
    match (event.index, event.subindex) {
        (0x6200, 1) => set_led(node.od().output1),
        (0x6200, 2) => set_pwm(node.od().output2),
        _ => {}
    }
}
```

**Event queue overflow:** if the queue is full, oldest events are silently dropped. The app should drain every loop iteration. The queue size is a const generic on `Node`, defaulting to 16.

### 2. App-to-protocol notification (outbound)

When the app changes an input value, it can notify the protocol stack to trigger event-driven TPDOs:

```rust
node.od_mut().input1 = new_sensor_value;
node.notify_changed(0x6000, 1);  // marks this entry as changed
```

On the next `node.process()` call, if a TPDO (type 254/255) maps this entry and the inhibit time has elapsed, the TPDO is sent immediately rather than waiting for the event timer.

**Implementation:** a small bitset or `heapless::FnvIndexSet` of `(index, subindex)` pairs that were marked dirty. `TpdoEngine::poll()` checks if any of its mapped entries are in the dirty set, and if so, sends the PDO (respecting inhibit time). The dirty set is cleared after `poll()`.

### 3. Optional async signals (behind `embassy` feature, future work)

For Embassy applications that want to `await` changes instead of polling:

```rust
// In an async task:
loop {
    node.od().output1_signal().changed().await;
    set_led(node.od().output1);
}
```

This is NOT part of the initial implementation. It can be added later by having the event queue also wake an `embassy_sync::Signal` when events are pushed. The proc macro would generate the signal fields.

## Changes to existing code

### `canopen-core/src/node.rs`

- Add `event_queue: heapless::Deque<OdEvent, EVT_QUEUE>` field to `Node`
- Add `dirty_set: heapless::FnvIndexSet<(u16, u8), DIRTY_SET>` field to `Node`
- Add const generic `EVT_QUEUE` (default 16) and `DIRTY_SET` (default 8) to `Node`
- Add `pub fn next_event(&mut self) -> Option<OdEvent>`
- Add `pub fn notify_changed(&mut self, index: u16, subindex: u8)`
- Pass `&mut event_queue` to `sdo_server.process()` and `rpdo.process()`

### `canopen-core/src/sdo/server.rs`

- `SdoServer::process()` gains an `events: &mut heapless::Deque<OdEvent, N>` parameter
- After a successful download (expedited or segmented), push an event

### `canopen-core/src/pdo/engine.rs`

- `RpdoEngine::process()` gains an `events: &mut heapless::Deque<OdEvent, N>` parameter
- After writing each mapped entry, push an event
- `TpdoEngine::poll()` gains a `dirty: &heapless::FnvIndexSet<(u16, u8), N>` parameter
- If any mapped entry is in the dirty set and inhibit time has elapsed, send immediately

### `canopen-core/src/od.rs`

- Add `OdEvent` and `OdEventSource` types

### Proc macro (`canopen-derive`)

- No changes needed for the event queue (it's index/subindex based, not field-name based)
- Future: generate signal fields for the async layer

## Testing

### Unit tests

- SDO download generates an event with `source: Sdo`
- RPDO write generates events with `source: Rpdo`, one per mapped entry
- Event queue overflow drops oldest
- `notify_changed` + TPDO type 255 triggers immediate send
- `notify_changed` respects inhibit time
- `notify_changed` does NOT trigger sync-type TPDOs (types 1-240)

### HIL tests

- Write `output1` via SDO, verify the event is emitted and LED changes reactively
- Change `input1` on STM32, call `notify_changed`, verify TPDO is sent immediately

## Non-goals

- No per-field callbacks (closure borrowing issues in `no_std`)
- No filtering (app gets all events, matches on index/subindex)
- No async signals in initial implementation
- No event coalescing (same entry written twice → two events)
