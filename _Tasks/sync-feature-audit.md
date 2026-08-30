# SYNC and synchronous PDO feature audit

Date: 2026-08-30

Status: implementation audit and backlog. SYNC is not currently used by the
project, so the gaps below are documented for later rather than scheduled for
immediate fixes. This document describes the current working tree; it is not a
claim of full CiA 301 conformance.

## Scope and source basis

This audit covers the classic CANopen SYNC service, synchronous TPDO/RPDO
behavior, and the PDO transmission types which interact with SYNC. It does not
cover the separate SYNC-aware background-traffic scheduler proposed in
`sync-aware-transmit-scheduling.md`.

CiA 301 itself is not publicly reproduced here. The comparison uses these
public primary and implementation sources:

- [CiA: SYNC protocol](https://www.can-cia.org/can-knowledge/sync-protocol) —
  counter, overflow, and TPDO SYNC start value.
- [CiA: PDO protocol](https://www.can-cia.org/can-knowledge/pdo-protocol) —
  event/timer, remote-requested, synchronous cyclic, and synchronous acyclic
  trigger families.
- [CiA 301 poster (2024)](https://member.can-cia.org/files/canopen_poster_2024.pdf)
  — OD layout, including TPDO communication-parameter subindex 6.
- [CiA configuration guideline](https://www.can-cia.org/fileadmin/cia/documents/proceedings/2003_rostan.pdf)
  — the classic-CAN synchronous process-image model and transmission-type
  descriptions.
- [CANopenNode `CO_SYNC.c`](https://github.com/CANopenNode/CANopenNode/blob/master/301/CO_SYNC.c)
  and [`CO_PDO.c`](https://github.com/CANopenNode/CANopenNode/blob/master/301/CO_PDO.c)
  — a conformance-tested reference implementation used to resolve operational
  details such as type-0 event timers, consumer timeout monitoring, payload
  length checks, and SYNC start-value handling.
- [Beckhoff PDO communication types](https://infosys.beckhoff.com/content/1033/tcsystemmanager/1092841483.html)
  — a useful description of the legacy RTR types 252/253. Current CANopenNode
  instead treats 241..=253 as unused, so RTR support must be tied to the exact
  CiA 301 edition/profile the project intends to target.

Before implementing a disputed edge case, especially RTR or dynamic-write
restrictions, check it against the project's licensed target revision of CiA
301. The major missing items below do not depend on the RTR-version question.

## Mental model

A synchronous cycle is more than "send a PDO periodically":

```text
between SYNCs                         at the next SYNC
-------------------------------      ---------------------------------
receive synchronous RPDO frames      commit the latest RPDO per PDO
latch type-0 TPDO events         ->  sample synchronous TPDO inputs
                                      transmit due type-0 / cyclic TPDOs
                                      start the 0x1007 transmit window
```

This gives two useful properties:

1. Several RPDO setpoints become visible together at one edge.
2. Synchronous TPDOs sample one coherent post-RPDO process image.

The current implementation has both properties. `Node::handle_sync()` commits
all buffered RPDOs before calling `TpdoEngine::on_sync()`, and TPDO frames are
built at that edge. A refused synchronous TPDO retains that sampled frame for
retry inside the current 0x1007 window.

## PDO transmission types

The table uses the classic transmission-type definitions exposed by this
crate. The status describes behavior, not merely whether the numeric value can
be parsed.

| Value | Meaning | TPDO behavior | RPDO behavior | Current status |
|---:|---|---|---|---|
| 0 | Synchronous acyclic | A device-specific/application event is latched; sample and send once at the next SYNC. An event timer is another event source where supported. | Keep the latest received frame and apply it at the next SYNC. | **Partial.** Change-trigger latching works; the event timer is ignored. |
| 1 | Synchronous cyclic | Sample and send at every SYNC. | Same receive-side staging as all values 0..=240. | **Implemented.** |
| 2..=240 | Synchronous cyclic every N edges | Count SYNCs and sample/send every Nth edge. Optional TPDO subindex 6 aligns the phase to a counter value. | Receivers generally do not distinguish these values from 0/1; apply the latest frame at the next SYNC. | **Partial.** Local divide-by-N works; counter-based phase alignment does not. |
| 241..=251 | Reserved | No transmission. Configuration should be rejected. | No processing mode is defined. | **Partial validation.** DSL and EDS import reject them, but static OD and SDO paths can still install them. |
| 252 | Legacy synchronous RTR-only TPDO | Sample on SYNC, transmit only when an RTR for the PDO arrives. | Not applicable. | **Not implemented.** The enum/DSL name exists, but frames cannot represent RTR. Confirm target CiA revision before implementing. |
| 253 | Legacy asynchronous RTR-only TPDO | Sample independently of SYNC and transmit only on RTR. | Not applicable. | **Not implemented.** Same caveat as 252. |
| 254 | Asynchronous, manufacturer-specific event | Transmit on the manufacturer-defined event or event timer, subject to inhibit time. | Apply immediately; subindex 5 is receive deadline monitoring. | Implemented as dirty/change-driven plus timer. |
| 255 | Asynchronous, device/application-profile event | Transmit on the profile/application event or event timer, subject to inhibit time. | Apply immediately; subindex 5 is receive deadline monitoring. | Implemented as dirty/change-driven plus timer. |

Important distinctions:

- Type 1 is the common "one TPDO after every SYNC" mode.
- Type N is a divider, not a period in time. Its time period is `N * 0x1006`
  only while the producer actually maintains that SYNC period.
- Type 0 is "event-triggered, SYNC-released". It does not transmit at every
  edge, and the payload should be sampled at the release edge rather than at
  the earlier application event.
- On the receive side, types 0..=240 all have the same useful behavior: retain
  data received after one SYNC and make the latest frame valid at the next.
- 0x1007 bounds synchronous TPDO transmission, not RPDO reception and not
  asynchronous traffic.

## SYNC configuration objects

| Object | Function | Current implementation | Gap |
|---|---|---|---|
| 0x1005 | SYNC COB-ID, standard/extended frame selection, producer bit | Live 11-bit COB-ID and producer bit; relocated dispatch; local collision checks | 29-bit SYNC unsupported; CiA restricted-ID validation needs audit |
| 0x1006 | Communication cycle period in microseconds | Drives the local producer when 0x1005 producer bit is set | Consumer-side missing-SYNC monitoring is absent |
| 0x1007 | Synchronous window length in microseconds; 0 means unlimited | Live value; stack-owned refused TPDOs retry through the inclusive boundary and are then discarded | Accepted frames already inside a driver/hardware queue cannot be recalled through `embedded_can` |
| 0x1019 | SYNC counter overflow; 0 means no counter, 2..=240 wraps 1..=N | Producer emits and wraps the counter; static and SDO range checks exist | Consumer does not enforce the configured payload shape; received counter is not used by TPDO scheduling |
| 0x1800..=0x19ff sub 6 | TPDO SYNC start value; 0 ignores the bus counter, 1..=240 chooses the first matching SYNC | Absent from generated OD records, EDS import/export, `TpdoConfig`, and engine behavior | Missing end to end |

## What is already implemented well

### SYNC producer and consumer

- Default consume COB-ID 0x080 and live relocation through 0x1005.
- Producer selection through 0x1005 bit 30 plus a nonzero 0x1006 period.
- Producer operates in Pre-operational and Operational, but not Stopped.
- A locally produced pulse is applied only after the transport accepts the
  SYNC frame, so the local process image cannot advance ahead of the bus.
- Refused producer frames retain their counter and phase for retry.
- 0x1019 producer counters run `1..=overflow` and wrap to 1.
- Static and SDO-written 0x1019 values reject 1 and values above 240.

### Synchronous PDO data path

- TPDO types 1..=240 use independent per-PDO divisors.
- Type-0 application changes are latched across `Node::process()` calls until
  a later SYNC consumes the trigger.
- All synchronous RPDO types 0..=240 buffer rather than changing the OD at
  frame reception.
- The latest frame per RPDO wins between two SYNCs.
- RPDO deadline reception timestamps are still updated on reception, rather
  than delayed until OD commit.
- On SYNC, RPDO commit precedes TPDO sampling.
- Leaving Operational and communication reset discard pending RPDO frames,
  type-0 triggers, synchronous TPDO retries, and window state as appropriate.

### Synchronous transmission window

- A nonzero 0x1007 opens at each accepted/received SYNC.
- A synchronous TPDO refused by `transmit()` retains its already sampled
  payload and retries only inside that window.
- The exact end timestamp is included; the frame expires when elapsed time is
  greater than the configured window.
- A new SYNC supersedes stale pending work from the preceding cycle.

The unavoidable limitation is below the stack: after a CAN driver accepts a
frame into a private software or hardware queue, `embedded_can::nb::Can` has no
generic cancellation operation. The stack can enforce 0x1007 only while it
still owns the pending frame. This should remain a documented transport
contract rather than be presented as fully enforceable at the wire.

## Missing features

### 1. TPDO SYNC start value and counter-aligned phase

Priority: high.

The producer-side counter is implemented, but its main consumer-side use is
not. A cyclic TPDO may expose subindex 6 in its communication record:

- `0`: do not use the SYNC counter to establish the phase;
- `1..=240`: wait until a received SYNC counter equals the start value, treat
  that pulse as the first pulse, then continue with the configured type-N
  cadence.

Current gaps:

- Generated TPDO records stop at highest subindex 5.
- The DSL has no `sync_start = ...` parameter.
- EDS import ignores subindex 6 and export omits it.
- `TpdoConfig` has no start-value field.
- `SyncConsumer::process()` returns a counter, but `Node::dispatch_frame()`
  throws it away and `TpdoEngine::on_sync()` accepts no counter.
- Locally produced pulses likewise call `handle_sync()` without their counter.

Implementation shape:

1. Add `sync_start_value: u8` to `TpdoConfig` and generated OD storage.
2. Add TPDO communication subindex 6 to metadata/read/write/EDS paths and
   `sync_start = N` to the DSL.
3. Validate `0..=240`; a nonzero start value is meaningful only with a
   counter-enabled SYNC network and a cyclic type.
4. Preserve whether the frame carried a counter. Do not turn a counterless
   pulse into a synthetic counter for start-value matching.
5. Pass `Option<u8>` (or a richer pulse object) through
   `dispatch_frame -> handle_sync -> TpdoEngine::on_sync`.
6. Maintain per-TPDO `waiting_for_start`/phase state. Reset it on communication
   reset and when relevant PDO/SYNC configuration changes.
7. Apply exactly the same counter to a producer's accepted local pulse as was
   placed in the transmitted frame.

Required tests:

- type 4, start 3: no transmission for counters 1/2, first transmission on 3,
  then every fourth accepted pulse;
- two TPDOs with different start values are phase-separated;
- start 0 retains counter-independent divide-by-N behavior;
- counterless SYNC never accidentally matches a nonzero start value;
- wrap at 0x1019 preserves the configured phase;
- producer local application and external consumers see the same phase;
- SDO writes, EDS round-trip, invalid ranges, reset, and NMT state changes.

### 2. Type-0 event timer

Priority: high.

Type 0 already latches dirty/change events, but `TpdoEngine::poll()` evaluates
the event timer only for 254/255. CANopenNode treats event-timer expiry as a
send request for type 0 too; the request is held until the next SYNC.

Consequences today:

- `tpdo[N](transmission_type = sync_acyclic, event_timer = 100ms)` compiles;
- the timer never triggers a TPDO;
- the declaration therefore promises behavior the engine silently ignores.

The event timer should set the same per-TPDO latch as an application event.
The next SYNC samples and transmits once, then restarts the event timer from
the actual transmission. A refused frame must retain the sampled payload under
the existing 0x1007 retry policy.

Also tighten DSL/runtime semantics:

- `event_timer` is meaningful for type 0, 254, and 255;
- it is not a second trigger for cyclic types 1..=240;
- `inhibit_time` applies to asynchronous event-driven TPDOs, not type 0 or
  cyclic synchronous TPDOs.

Tests should cover timer-only type 0, an application event before timer expiry,
timer expiry several process calls before SYNC, reset/leaving Operational, and
transport refusal at the release edge.

### 3. Consumer-side SYNC timeout monitoring from 0x1006

Priority: high for devices whose safe behavior depends on the control cycle.

0x1006 is not only a producer scheduler. A consumer can use the expected cycle
period to detect a missing SYNC. The public CiA configuration guideline calls
out cycle-time monitoring, and CANopenNode reports a timeout after 1.5 times
the configured period once reception has started, then clears it on recovery.

The crate currently has no SYNC freshness state or event. If the producer
disappears, synchronous RPDO values simply remain unchanged and type-0/cyclic
TPDOs stop without an application-visible reason.

Suggested policy, matching the existing heartbeat/RPDO approach:

- monitoring begins with the first valid received SYNC, so a configured but
  never-seen producer starts in `Waiting` rather than immediately timing out;
- expose a level state such as `Waiting | Alive | Timeout` and edge events
  `Started | Timeout | Recovered`;
- run it in Pre-operational and Operational, but not Stopped;
- keep the reaction application-controlled: safe outputs, NMT action, and
  EMCY production are device/profile policy;
- add the appropriate communication/SYNC EMCY constants so applications do
  not use magic numbers.

Whether the timeout is normatively fixed at 1.5 periods or should be
configurable by this stack must be checked against the selected CiA revision.
At minimum, 0x1006 must become observable as a consumer watchdog rather than
producer-only configuration.

### 4. SYNC payload shape and malformed-SYNC reporting

Priority: high for conformance and diagnostics.

The local 0x1019 setting determines the expected SYNC payload:

- `0`: a valid SYNC has no data byte;
- `2..=240`: a valid SYNC has one counter byte.

`SyncConsumer` currently accepts either DLC 0 or DLC 1 regardless of 0x1019,
and rejects only DLC greater than 1. Existing interop tests intentionally
codify that permissive behavior. The reference implementation instead checks
the configured shape and reports unexpected SYNC data length (EMCY 0x8240).

Needed work:

- configure the consumer with the local 0x1019 value;
- distinguish `Valid(Pulse)` from `WrongLength` rather than silently treating
  the latter as an unrelated frame;
- expose a diagnostic/event and application-selectable EMCY path;
- decide whether an invalid counter value (0 or greater than the configured
  overflow) is rejected, reported, or merely unusable for start-value phase;
- replace the permissive interop test with matched and mismatched 0x1019 cases.

Malformed frames on the owned SYNC COB-ID should remain consumed so they
cannot fall through into a colliding heuristic protocol path.

### 5. Legacy RTR transmission types 252/253

Priority: low unless interoperability requires them.

The public API currently over-promises support:

- `TransmissionType::{RtrSync, RtrEvent}` exist;
- the DSL accepts `rtr_sync` and `rtr_event` for TPDOs;
- neither PDO engine implements their behavior;
- `CanFrame::new_remote()` always returns `None`, and transports discard or
  cannot preserve remote frames.

If the selected CiA revision/profile requires them, support is cross-cutting:

1. Represent data versus remote frames in `CanFrame`.
2. Preserve RTR through SocketCAN, SLCAN, UDP, canwsd, mailbox, and bus APIs.
3. Route an RTR by exact configured TPDO COB-ID.
4. For 252, refresh/sample the reply image on SYNC and answer RTR with that
   snapshot; for 253, answer with an asynchronous/current snapshot according
   to the chosen controller model.
5. Implement PDO COB-ID bit 30 (RTR allowed/forbidden) and collision rules.
6. Add interop tests using a transport which actually supports CAN remote
   frames.

If RTR is not in scope, remove or deprecate the enum variants and DSL keywords
and reject 252/253 explicitly. Keeping configuration syntax which produces a
permanently silent TPDO is the worst outcome.

### 6. Extended-frame SYNC

Priority: low; broader than SYNC.

0x1005 can select a 29-bit frame, but this stack's `CanFrame` supports only
11-bit identifiers. Static and runtime 0x1005 values with the extended-frame
bit are correctly rejected rather than misinterpreted.

Supporting this requires an extended-ID-capable frame model and every
transport/protocol collision check to preserve frame format as part of the
identity. It should be planned as a stack-wide CAN-ID feature, not a local
change to `sync.rs`.

### 7. Configuration validation and phase lifecycle

Priority: medium.

Several semantic checks exist only in the hand-written DSL, not at every
configuration ingress:

- SDO writes to PDO transmission type accept reserved values;
- EDS import now rejects out-of-width, reserved, and direction-invalid
  transmission types, but equivalent validation is not yet shared with all
  other configuration paths;
- changing a transmission type at runtime updates the config but does not
  deliberately reset or preserve its per-PDO synchronous phase;
- `event_timer` and `inhibit_time` can be attached to types which ignore them;
- local COB-ID collision checks do not implement the full CiA restricted
  CAN-ID rule. For example, the current relocation tests deliberately accept
  0x123, while CANopenNode's CiA restricted-ID check rejects that range.

Replace the currently duplicated DSL/EDS checks with one shared validator used
by codegen assertions, EDS import, `Node::new`, and SDO downloads. Separately
define the lifecycle rule for each runtime change: preserve phase, restart its
divisor, wait for SYNC start value, or discard pending work. Tests should make
those choices explicit.

## API/documentation gaps

`Node::notify_changed()` currently documents only types 254/255, although the
same dirty set is also the application-event source for type 0. Its docs should
say that:

- asynchronous types 254/255 send when eligible;
- type 0 latches the event and sends at the next SYNC;
- cyclic types 1..=240 ignore application change notifications.

It may also be clearer to expose an explicit `request_tpdo(PdoNumber)` API.
Change detection is a convenient default event source, but CiA's application
event can be a threshold, state transition, hardware interrupt, or
profile-defined condition which does not correspond to an OD byte changing.
An explicit request avoids manufacturing a fake mapped-object change and maps
directly to the type-0/254/255 send-request model.

## Recommended implementation order

1. Add the type-0 event timer and fix the DSL/API documentation; this is small
   and closes a currently silent configuration trap.
2. Introduce a structured SYNC pulse carrying `Option<counter>`, then implement
   TPDO subindex 6 end to end.
3. Add strict 0x1019 payload-shape diagnostics and SYNC timeout state/events.
   Design these together so one coherent consumer monitor owns timing, frame
   validity, and recovery.
4. Centralize transmission-type/config validation and phase reset rules.
5. Decide explicitly whether legacy RTR and extended identifiers are in scope;
   either implement them across the transport boundary or reject their
   configuration surfaces.

## Completion criteria

The SYNC feature can be called complete for the selected classic-CANopen scope
when:

- every supported transmission type has executable behavior, not just syntax;
- type 0 supports both explicit/application events and its configured event
  timer;
- cyclic TPDOs can align to the network counter through subindex 6;
- 0x1006 loss and recovery are observable to the application;
- 0x1019 controls both produced and accepted SYNC payload shape;
- reset, NMT transitions, runtime reconfiguration, producer-local pulses, and
  externally received pulses share documented phase semantics;
- EDS import/export, DSL, static construction, and SDO writes enforce the same
  value rules;
- interop tests cover counter alignment, type-0 timer release, timeout and
  recovery, malformed payloads, and the chosen RTR/extended-ID policy.
