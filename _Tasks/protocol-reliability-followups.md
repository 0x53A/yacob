# Protocol reliability follow-ups

Date: 2026-08-30, revised 2026-08-31 after a review pass over the fixes.

Status: findings for discussion before implementation. The local, low-risk
items found during the SYNC audit have been fixed; the stateful items below
need an agreed design. Sections 5 and 6 came out of reviewing the first round
of fixes rather than the original audit.

## Classification

| Finding | Classification | Disposition |
|---|---|---|
| EDS PDO numeric values were narrowed with unchecked casts | Easy/local | Fixed with checked parsing and regression tests |
| `notify_changed()` omitted type-0 behavior from its docs | Easy/local | Fixed documentation |
| `SyncConsumer` called a wrapping `u8` count monotonic | Easy/local | Fixed documentation |
| Block-upload ACK sequence was ignored | Easy/local protocol fix | Fixed with partial-ACK retransmission and block-size validation |
| Nonblocking transmit refusal loses protocol work | Stateful/systemic | Discuss before implementation |
| Event-driven TPDO event is lost during inhibit time | Local PDO state | Fixed with a per-TPDO pending-event latch |
| Dirty-change tracking silently exceeds fixed capacities | Local PDO state | Fixed by coalescing directly per TPDO |
| Inhibited TPDO event survived an NMT reset | Local PDO state | Fixed by clearing the async latch on communication reset |
| Empty `DefaultValue=` became a hard EDS import error | Easy/local | Fixed by treating empty as unspecified |
| Block download aborts instead of asking for retransmission | Stateful protocol fix | Discuss before implementation (section 5) |
| Runtime communication changes inherit stale phase/pending state | Lifecycle semantics | Discuss before implementation |
| Updater makes one post-erase status request | Stateful retry policy | Discuss before implementation (section 6) |
| Broken TPDO mapping leaves its trigger armed forever | Configuration-fault policy | Discuss with runtime reconfiguration (section 4) |

The unchecked-conversion fix intentionally covers the PDO transmission type,
inhibit time, event timer/deadline, and mapping count identified in the audit.
It is not a claim that every EDS integer conversion has been audited; that is
one of the recurring patterns below.

Review follow-up on the checked-parsing fix: the first version rejected an
empty `DefaultValue=` as unparsable. Vendor EDS files routinely write a bare
key for a parameter left at its default, so that turned ordinary files into
compile errors. Absent and empty now both mean "unspecified" and take the CiA
301 default; anything non-empty must still parse and fit.

## 1. Protocol state is consumed before transmit acceptance

Severity: high.

`embedded_can::nb::Can::transmit()` may return `WouldBlock`, meaning the frame
was not accepted. Many `Node` paths discard that result after the protocol has
already consumed or advanced the work:

- ordinary and additional SDO responses;
- SDO timeout abort frames;
- block-upload segments returned by `poll_block_upload()`;
- EMCYs removed by `take_pending()`;
- heartbeat frames whose producer timer has advanced;
- asynchronous/event-driven TPDOs whose event/timer state has advanced.

SYNC production and synchronous TPDO retry are the exceptions: they retain
work and commit producer state only after acceptance. That two-phase behavior
exposes the more general problem.

Block upload now also recovers at its protocol boundary: when a client ACKs
only the last contiguous segment it received, the server rewinds to the first
unacknowledged byte and retransmits the tail with a recomputed CRC. This makes
individual missing block segments recoverable, but does not solve the general
advance-before-acceptance problem for initiate/end responses or other SDO
transfer modes.

### Why a local retry is unsafe

Each protocol has different commit semantics:

- An SDO server must not consume the next request or advance a block sequence
  while its preceding response is still pending.
- An EMCY is a queued diagnostic and must retain ordering across refusal.
- A heartbeat can usually coalesce to the newest state, but its schedule must
  distinguish an attempted frame from an accepted frame.
- An event TPDO may coalesce repeated events, but inhibit time and event-timer
  restart belong to the accepted transmission, not the attempt.
- Some frames have protocol priority over others; one pending frame per
  producer is not enough to define cross-protocol ordering.

Adding `if is_err() { retry later }` independently at each call site risks
duplicates, reordered SDO segments, starvation, and inconsistent timer state.

### Design questions

1. Should `Node` own one bounded priority TX outbox, or should each protocol
   expose a two-phase `peek/confirm` interface and let `Node` arbitrate them?
2. Which traffic is non-lossy? SDO responses/segments and accepted EMCYs
   should be; heartbeat and repeated timer PDOs may have explicit coalescing.
3. Is payload sampling done when work becomes eligible or when the transport
   accepts it? Synchronous TPDOs require the former; asynchronous PDO policy
   needs to be explicit.
4. How does this combine with `_Tasks/sync-aware-transmit-scheduling.md`, which
   also needs a bounded priority scheduler and non-lossy SDO queues?
5. What happens when the configured pending capacity itself fills?

### Required tests

Use a transport which predictably refuses the first N attempts and records
accepted frames. Cover expedited and segmented SDO, block upload, multiple
EMCYs, heartbeat, event TPDO, synchronous TPDO, and simultaneous traffic from
several classes. Assert both wire order and internal state after each refusal.

## 2. Resolved: retain event-driven TPDO changes through inhibit time

Resolution: fixed on 2026-08-30.

Types 254/255 now latch an application event per TPDO before testing inhibit
time. Repeated changes coalesce, and the PDO samples the latest process image
once it becomes eligible. Leaving Operational discards inhibited work so it
cannot appear as a stale event after re-entry.

Review follow-up, fixed on 2026-08-31: the Operational boundary was not the
only one that needed the discard. `TpdoEngine::reset_sync_state()` cleared the
synchronous latch but not the new asynchronous one, and a node with
`auto_start` re-enters Operational inside the same `process()` call that
performs the reset — so the boundary path never ran and a pre-reset event was
transmitted once its inhibit time expired. The method is now
`reset_communication_state()` (the old name is what hid the omission) and
clears both. Regression tests at engine and `Node` level.

Still open in the same area: `last_send_us` is not reset by a communication
reset either, so the first post-reset transmission of an event TPDO is not
subject to a fresh inhibit window. That is a lifecycle question rather than a
lost trigger — folded into section 4.

The implemented path is:

```text
application event -> latch pending -> wait for inhibit expiry
                  -> prepare/sample PDO
                  -> clear pending and restart inhibit/event timer
                  -> attempt transport transmission
```

The final transition is still preparation rather than transport acceptance.
If `transmit()` refuses the prepared frame, the broader issue in section 1
still loses it. Fixing that requires the shared two-phase transmit design.

Regression tests cover expiry without a second notification and sampling the
latest of several inhibited changes. Timer/event coincidence, clock wrap,
refusal at inhibit expiry, and runtime reconfiguration remain tests for the
eventual scheduler/lifecycle work.

## 3. Resolved: dirty-change bookkeeping no longer controls TPDO correctness

Resolution: fixed on 2026-08-30.

There are two independent caps:

- `Node` defaults `DIRTY_SET` to 8 and ignores a failed `push()` in
  `notify_changed()`.
- `OdGuard::drop()` first collects changes into a fixed 32-element array and
  silently ignores the remainder.

PDO mapping supports 64 one-bit entries per PDO and a node may declare many
TPDOs, so both old caps could be exceeded by valid configurations. The engine
now converts `notify_changed()` directly into per-TPDO pending bits. `OdGuard`
similarly compares all mappings and coalesces the result into `[bool; TPDO]`,
without collecting addresses in a fixed scratch array. The legacy dirty set
is no longer authoritative for transmission, and a zero-capacity regression
test verifies that an `OdGuard` change still triggers its TPDO.

## 4. Runtime reconfiguration needs transition semantics

Severity: medium.

`sync_pdo_from_od()` updates communication and mapping fields in place, while
per-PDO counters, event latches, deadline state, and pending frames live in
separate arrays. `sync_config_from_od()` likewise updates 0x1005/0x1006/
0x1007/0x1019 without consistently reconciling an open window or pending TPDO.

The SDO server prevents ordinary PDO-parameter writes in Operational, which
reduces the common exposure, but the resync method is public and SYNC objects
are live. Direct engine mutation is also public.

For every changed field, specify one of:

- preserve phase/state;
- reset the cyclic divisor;
- discard a pending event/frame;
- retain the old sampled frame under the old configuration;
- wait for a SYNC start value before becoming active.

The transition should be centralized rather than inferred from whichever
array happens to retain its previous value. This can be designed alongside
the SYNC start-value work in `_Tasks/sync-feature-audit.md`.

A concrete instance of the same gap, from the section 2 review: NMT reset
clears the TPDO event latches and cyclic divisors but leaves `last_send_us`
untouched, so the first post-reset transmission of an event-driven TPDO is not
subject to a fresh inhibit window. Whether a reset restarts inhibit and event
timers, or deliberately preserves them so a reset cannot be used to bypass an
inhibit time, is exactly the kind of per-field decision this section is meant
to settle.

## 5. Block download aborts where the spec asks for retransmission

Severity: medium. Found reviewing the block-upload partial-ACK fix.

The upload side now recovers from a lost sub-block segment. The download side
does not: `handle_block_download_segment()` answers any sequence gap with abort
0x0504_0003 and drops the transfer. CiA 301 §7.2.4.3.7 instead expects the
server to confirm the sub-block with `ackseq` set to the last correctly
received sequence number and let the client retransmit from there. One lost
frame therefore kills a whole download rather than costing one sub-block.

Reach is limited today: this stack's own `SdoDriver` never initiates block
transfers, so only third-party clients (python-canopen, generic CANopen tools)
take this path. That is also why it is not urgent enough to implement blind.

Why it is not a two-line fix, and why it sits here rather than being done with
the upload fix: after detecting a gap the server must keep consuming the rest
of the sub-block still in flight without treating those segments as data,
recognise the end of the sub-block (`seqno >= blksize` or the last-segment
bit), and only then send the rewinding ACK. If the segment carrying that
terminator is itself the lost one, nothing closes the sub-block and the
transfer falls to the SDO timeout — acceptable, and what CANopenNode does, but
it needs to be a decision rather than an accident.

### Design questions

1. Confirm "discard until end of sub-block, then ACK the last good seqno" as
   the intended recovery, with the timeout as the backstop for a lost
   terminator.
2. Is there a retransmission budget, or does an endlessly re-failing client
   just keep the transfer alive until it stops?
3. Does the existing `block_download_rejects_out_of_sequence_segment` test
   become a retransmission test, or do both behaviors need to be reachable?

## 6. The updater's post-erase status read has no retry

Severity: medium, host side only (`tools/canopen-fwupdate`).

Replacing the `await_status()` poll loop with a single long-lived read was the
right call — retrying short requests while the node blocks in its erase leaves
late responses queued, where a later transfer can mistake one for its own.

But `sdo_upload()` sends the request exactly once and never retransmits, and
the erase timeout is 30 s (firmware) or 120 s (web image). If that single
request frame is lost, the tool waits the full timeout and then fails a device
whose flash is already erased. The current design depends on the node's CAN/WS
receive path buffering the request across a blocking erase.

Options, in rough order of preference:

1. Give `SdoDriver` an explicit "expect a slow server" mode that retransmits
   the initiate request on a bounded schedule while refusing to accept a
   response that does not match the current request — this is the general fix
   and would serve any long-running SDO operation.
2. Have the updater re-request once after a first timeout, accepting the
   duplicate-response hazard because the only transfer in flight is this one.
3. Leave as is and document the buffering assumption in the bootloader.

## Recurring bug patterns to audit

### A. Advance-before-ack at fallible sinks

Pattern: pop a queue, clear a latch, restart a timer, or advance a protocol
sequence before a nonblocking `transmit()` is known to have accepted the
frame.

Search starting points:

```sh
rg -n 'let _ = .*transmit|\.transmit\([^)]*\)' canopen-core canopen-linux
rg -n 'take_pending|pop_front|poll_block_upload|last_send|confirm' canopen-core/src
```

Prevention:

- prefer two-phase `peek/prepare` plus `confirm_accepted` APIs;
- centralize ignored/coalesced traffic policies;
- add a reusable refusing-transport test fixture;
- do not allow a bare ignored `transmit()` result in protocol state machines.

### B. Ignored bounded-container insertion

Pattern: `let _ = queue.push(...)` or a fixed scratch array whose overflow is
silently skipped.

Search starting point:

```sh
rg -n 'let _ = .*\.push|is_full\(\)|< changed\.len\(\)' --glob '*.rs'
```

Prevention:

- every bounded insertion chooses `reject`, `drop oldest`, `coalesce`, or
  `fail safe`, and documents that policy;
- every deliberate loss increments an appropriate diagnostic counter;
- capacities derived from the same protocol limit should share a constant or
  a compile-time equality assertion.

### C. Unchecked numeric narrowing at configuration boundaries

Pattern: EDS/DCF/wire/config values parsed into a wide integer and narrowed
with `as`, potentially turning invalid data into a different valid mode.

Search starting points:

```sh
rg -n 'parse_int.*| as u8| as u16| as u32' canopen-derive/src canopen-core/src/dcf.rs
```

Prevention:

- use `TryFrom` with the OD address and raw value in the error;
- validate semantic enums/ranges after width validation;
- consider enabling Clippy's truncation lints in parser/configuration modules;
- test one-above-maximum and malformed values for every imported wire type.

The just-applied PDO parser fix is the template, not the completion of this
broader audit. Mapping values, COB-ID expressions, data-type codes, ordinary
EDS defaults, and DCF parsing still need a deliberate pass.

### D. Validation implemented at only one ingress

Pattern: inline DSL rejects a value while EDS import, manual configuration, or
SDO download accepts it.

Prevention:

- define shared semantic validators in the owning core module;
- invoke them from DSL, EDS, construction, and runtime writes;
- use table-driven tests which feed the same boundary values through every
  supported ingress.

PDO transmission types are a current example: the EDS path is now aligned
with the DSL for reserved values and RPDO-only direction errors, but SDO and
manual `TpdoConfig`/`RpdoConfig` construction still need the shared-validator
design described in the SYNC audit.
