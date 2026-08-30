# SYNC-aware transmit scheduling

Status: design/specification for later implementation. No implementation has
been started.

## Summary

The end effector (EE) shares a CAN bus with a six-axis arm. On the rover, the
arm runs a known, configured SYNC cycle and its real-time traffic follows this
shape:

```text
RPDO traffic -> SYNC -> TPDO traffic -> background/SDO traffic -> RPDO traffic
```

The EE does not need to make its application or event-driven PDO logic
synchronous. It should, however, opportunistically avoid adding lower-priority
traffic to the arm's protected RPDO/SYNC/TPDO part of the cycle. The same
scheduling logic is needed in two places:

- The STM32 EE node, principally for SDO-server responses and optionally for
  event-driven TPDOs and EMCYs.
- The Linux gateway, principally for SDO-client requests sent through a USB
  CAN adapter.

The same firmware must also work on a testbench where no SYNC producer exists.
Consequently, absence of SYNC must never suppress traffic: before SYNC is
observed, and after an active schedule has expired, all timing policies fail
open to immediate scheduling.

The SYNC period is configuration, not an inferred property. Observed SYNC
pulses activate and confirm a theoretical schedule; they do not continuously
move it and do not change its configured period.

SDO scheduling is the primary goal. A user action such as "read all
registers" can create a large SDO backlog, especially on Linux. Optional
event-driven TPDO deferral is secondary and must never change a PDO's sampled
payload.

## Goals

1. Keep SDO requests and responses out of configured RPDO/SYNC/TPDO protected
   intervals whenever SYNC is active.
2. Send normally when the same device is used on a bus with no SYNC.
3. Continue following the configured theoretical schedule briefly if SYNC
   disappears, then fail open after a configured holdover.
4. Prevent an unexpectedly fast or malformed-frequency SYNC source from
   continually restarting the protected interval and starving all traffic.
5. Provide explicit timing and software-priority policies for EMCY,
   event-driven TPDO, and SDO traffic.
6. Never drop an SDO protocol frame because of scheduling or queue overflow.
7. Optionally delay an event-driven TPDO by a small configured maximum, while
   retaining exactly the payload sampled at the event.
8. Bound the effect of an overflowing event-TPDO queue using the agreed
   `CollapseAndBypassOncePerCycle` behavior.
9. Remain `no_std`, `no_alloc`, and statically bounded on the embedded side.
10. Preserve all existing behavior when the scheduler is not configured.

## Non-goals

- This is not another implementation of CiA 301 synchronous PDO transmission.
  Existing transmission types 0..=240 and OD 0x1007 retain their current
  meaning and implementation.
- The scheduler does not infer the SYNC period from history.
- The scheduler does not make an asynchronous/event-driven TPDO synchronous.
- The scheduler does not reinterpret PDO contents or resample delayed PDOs.
- The scheduler cannot recall a frame after a CAN controller, kernel queue,
  USB adapter, or adapter firmware has accepted it.
- It does not guarantee bus timing under a misconfigured period or windows.
  Misconfiguration should remain fail-safe against permanent starvation, but
  the configured schedule is authoritative.
- It is not a generic replacement for CAN identifier arbitration. Software
  priority only decides which eligible frame is offered to a transport first.

## Terminology

- **Theoretical period (`P`)**: the configured SYNC period in microseconds.
- **Epoch (`E`)**: the first accepted SYNC timestamp after the scheduler was
  inactive.
- **Theoretical boundary (`B[n]`)**: `E + n * P`, independent of later observed
  SYNC timestamps.
- **Post-SYNC protected interval (`A`)**: the configured time after a
  theoretical boundary reserved for SYNC and arm TPDO traffic.
- **Pre-SYNC protected interval (`R`)**: the configured time before the next
  theoretical boundary reserved for arm RPDO traffic.
- **Open/background interval**: the interval between the post-SYNC and
  pre-SYNC protected intervals. SDO traffic normally belongs here.
- **Confirmation**: observing one eligible SYNC pulse around a theoretical
  boundary. Confirmation refreshes schedule liveness but never changes the
  epoch or period.
- **Holdover**: the number of configured theoretical cycles for which the
  scheduler remains active without a new confirmation.
- **Immediately eligible**: the scheduler imposes no intentional delay. The
  frame can still wait for higher software priority, CAN arbitration, or
  transport availability.

## Configured timing model

The base configuration is conceptually:

```rust
pub struct SyncScheduleConfig {
    /// Configured period; for the rover's 100 Hz bus this is 10_000 us.
    pub period_us: u64,

    /// Protected interval beginning at each theoretical SYNC boundary.
    pub protected_after_sync_us: u64,

    /// Protected interval ending at the next theoretical SYNC boundary.
    pub protected_before_sync_us: u64,

    /// Continue the theoretical schedule this many cycles without confirmation.
    pub holdover_cycles: u16,
}
```

For one cycle beginning at `B[n]`:

```text
B[n]                                                       B[n+1]
 |---- protected after ----|------ open ------|---- protected before ----|
 |      SYNC + TPDO        |  SDO/background |          RPDO             |
```

Using half-open intervals avoids ambiguous double ownership of boundaries:

```text
post-SYNC protected: [B[n], B[n] + A)
open/background:     [B[n] + A, B[n+1] - R)
pre-SYNC protected:  [B[n+1] - R, B[n+1])
```

At exactly `B[n]`, traffic is protected. At exactly `B[n] + A`, background
traffic becomes eligible. At exactly `B[n+1] - R`, the pre-SYNC protected
interval begins.

Configuration must require:

```text
P > 0
A + R < P
holdover_cycles > 0
```

An invalid configuration should be rejected when installed rather than
silently creating a schedule with no open interval.

For the rover, `P = 10_000 us` (100 Hz). The exact `A` and `R` values remain
tunable. The desired open interval is only a few milliseconds, less than
5 ms, so the two protected intervals will consume more than half of the 10 ms
cycle.

## Activation, confirmation, and loss of SYNC

### Inactive behavior

At boot, or whenever the schedule has expired, the scheduler is inactive.
Every timing policy behaves as immediate while inactive. This is the required
testbench behavior: a computer connected directly to the EE need not generate
SYNC to use SDO or receive event-driven PDOs.

### First SYNC

The first valid SYNC observed while inactive:

1. Becomes epoch `E`.
2. Establishes theoretical boundary `B[0] = E`.
3. Confirms cycle/boundary zero.
4. Activates protected-window scheduling immediately.

No period inference or second pulse is required. Requiring a second pulse can
be revisited if spurious valid frames on the configured SYNC COB-ID become a
real concern, but it is not part of the current design.

### Confirming later theoretical boundaries

While active, observed SYNC pulses do not move `E` and do not alter `P`.
Instead, a pulse can confirm at most one theoretical boundary.

A pulse in the protected region surrounding a theoretical boundary qualifies
as confirmation for that boundary:

```text
[B[n] - R, B[n] + A)
```

This includes a pulse arriving somewhat before the theoretical boundary (in
the RPDO/pre-SYNC interval) or somewhat after it (in the TPDO/post-SYNC
interval). Only the first qualifying pulse for a boundary confirms it.
Additional pulses associated with the same boundary are ignored.

A pulse in the open/background interval is ignored completely:

- It does not change the epoch.
- It does not change the theoretical period.
- It does not restart a protected interval.
- It does not refresh holdover/liveness.

This is essential to prevent an unexpectedly fast SYNC source from permanently
stalling background traffic.

### Mismatched-frequency example

Suppose the device is incorrectly configured for a 10 Hz schedule:

```text
P = 100 ms
A = 10 ms
R = 10 ms
open interval = SYNC + 10 ms through SYNC + 90 ms
```

but the actual bus produces SYNC at 100 Hz (every 10 ms).

After the first pulse establishes the epoch:

- Pulses at +10, +20, ..., +80 ms are in the theoretical open interval and
  are ignored.
- A pulse at +90 ms is in the pre-SYNC protected interval and may confirm the
  upcoming theoretical boundary at +100 ms.
- That confirmation does not move the theoretical boundary to +90 ms.
- The theoretical schedule remains a configured 100 ms schedule.

The device therefore retains a large open interval instead of restarting a
10 ms protected interval on every actual pulse and starving all outgoing
traffic.

### Holdover and fail-open

After a schedule has been activated, it continues for `holdover_cycles`
theoretical cycles after the most recently confirmed boundary. The initial
target is 10 cycles.

For the rover's 100 Hz configuration, ten cycles means approximately 100 ms
of holdover. This absorbs a few lost or delayed SYNC frames without allowing a
long-absent SYNC source to control testbench behavior indefinitely.

Once holdover expires:

- The scheduler becomes inactive.
- Every queued policy is evaluated as immediate.
- `OutsideOnly` no longer blocks SDO traffic.
- The next valid SYNC establishes a new epoch and immediately reactivates the
  schedule.

Ignored pulses in the open interval do not extend holdover.

## Relationship to CiA 301 SYNC support

The new scheduler must remain distinct from the current SYNC protocol state:

- OD 0x1005 continues to define the exact SYNC COB-ID and producer bit.
- OD 0x1006 continues to define the CiA communication cycle period used by the
  existing SYNC producer. The new schedule may use the same numeric period,
  but it is a separate application/bus policy configuration unless an explicit
  API decision later wires them together.
- OD 0x1007 continues to define the allowed transmission window for genuinely
  synchronous TPDOs. It is not the new pre/post background-traffic policy.
- Synchronous RPDO commit and synchronous TPDO sampling remain unchanged.

The schedule observer should use the same exact configured SYNC COB-ID and the
same frame validity rules as `SyncConsumer`: an 11-bit frame on that COB-ID
with DLC 0 or 1.

## Traffic policy model

Timing, software priority, and optional rate/budget controls are separate
concepts. A conceptual public model is:

```rust
pub struct TxPolicy {
    pub timing: TxTiming,
    pub priority: TxPriority,
    pub budget: TxBudget,
}

pub enum TxTiming {
    /// Never delay intentionally for the theoretical SYNC schedule.
    Immediate,

    /// Prefer the open interval, but only when it can be reached within the
    /// configured delay from this frame's original eligibility time.
    PreferOutside { max_delay_us: u64 },

    /// While the schedule is active, transmit only in the open interval.
    OutsideOnly,
}

pub enum TxBudget {
    Unlimited,

    /// At most this many accepted frames in one theoretical cycle while the
    /// schedule is active. Excess work remains queued for later cycles.
    PerSyncCycle(u16),
}
```

Names and representation are not final API commitments, but these semantics
are the design target.

### Timing decisions

When the scheduler is inactive, all three timing variants are immediately
eligible.

When active:

- `Immediate` is immediately eligible in both protected and open intervals.
- `OutsideOnly` is eligible only in the open interval.
- `PreferOutside { max_delay_us }` behaves as follows:
  - If already in the open interval, send immediately.
  - If protected, calculate the next opening time.
  - If that opening is no later than the frame's original eligibility time
    plus `max_delay_us`, defer to the opening.
  - Otherwise make the frame immediately eligible, even though the schedule
    is protected.

The maximum delay is measured from the moment the protocol producer emits the
frame to the scheduling layer. Repeated policy evaluation must not extend the
deadline.

### Priorities

Software priority chooses among pending frames that are currently eligible.
It exists because generic `embedded_can::nb::Can`, kernel queues, and USB
adapters do not promise that a later high-priority frame can displace an older
low-priority frame already accepted into a FIFO.

The scheduler should use priority first and the raw CAN identifier second.
Lower CAN identifiers retain their normal higher CAN arbitration priority when
otherwise equal. Equal-priority queues should not allow one noisy PDO or SDO
channel to starve all peers indefinitely; stable FIFO or lightweight
round-robin behavior should be used where necessary.

Hard protocol traffic remains outside the user policy surface:

- Locally produced SYNC is hard immediate and highest priority.
- Genuinely synchronous TPDOs generated by a SYNC pulse retain their current
  immediate/window-bounded behavior.
- Boot-up/error-control traffic, heartbeat, and LSS remain immediate unless a
  later design explicitly broadens the policy surface.

User-configurable policies are required for:

- EMCY
- Event-driven TPDOs, individually per TPDO
- SDO, at least as a class and potentially per SDO channel

Recommended defaults are:

| Traffic | Timing | Budget | Software priority |
|---|---|---|---|
| EMCY | `Immediate` | Unlimited | Critical |
| Event-driven TPDO | `Immediate` | Unlimited | Realtime |
| SDO | `OutsideOnly` | Configurable per cycle | Background |

These defaults preserve immediate EMCY behavior and preserve existing
event-driven PDO behavior unless a PDO explicitly opts into bounded deferral.

## SDO scheduling

SDO is the primary use case and has different reliability requirements from
PDO telemetry.

### Never drop SDO protocol frames

An SDO transfer is a stateful request/response protocol. Dropping, replacing,
or collapsing a queued SDO frame corrupts the transfer and normally ends as a
timeout or protocol error. Consequently:

- SDO queues are non-lossy.
- Queue saturation applies backpressure to the producer.
- A queued SDO request, response, segment, ACK, end frame, or abort is retained
  until accepted by the next layer or until the owning protocol transfer is
  explicitly aborted/reset.
- SDO frames never use PDO-style overflow collapse.

### SDO timing policy

The normal rover policy is `OutsideOnly`: client requests and server responses
are admitted only in the theoretical open interval while scheduling is active.
On a no-SYNC testbench, the schedule remains inactive and SDO is immediate.

The policy remains user-configurable. For example, a diagnostic or recovery
channel could use `PreferOutside` with a finite maximum delay, or an application
could deliberately use `Immediate`.

### SDO storm control

A window alone is insufficient. If the Linux side queues a large "read all
registers" operation, immediately draining the entire backlog at the start of
the open interval can fill the kernel/USB/adapter queues and spill traffic into
the next protected interval.

The scheduled SDO path therefore needs an explicit per-cycle admission budget.
The exact initial value remains to be selected from bus measurements, but its
semantics are:

- Count frames when the physical transport accepts them, not when an
  application merely enqueues them.
- Stop admitting budgeted SDO frames once the configured count for the
  theoretical cycle is reached.
- Carry remaining frames into later open intervals without dropping them.
- Reset the budget at each theoretical cycle boundary.
- Do not apply the SYNC-cycle budget while the scheduler is inactive unless a
  separate non-SYNC rate limiter is later requested.

The Linux gateway controls SDO client requests. The EE controls SDO server
responses. Each endpoint accounts for its own transmitted frames; the two
budgets need not be numerically identical.

### SDO backpressure and completion

On the Linux shared bus, `AsyncCan::transmit().await` should not report
completion merely because a scheduled frame joined a backlog. It should, where
practical, complete when the scheduled pump has admitted the frame to the
physical adapter. This has two benefits:

1. It naturally backpressures many concurrent SDO operations.
2. `SdoDriver` does not begin waiting for a response to a request that has not
   yet left the scheduled queue.

An implementation may use a small per-entry completion object/waker on the
`std` side. Adapter acceptance still does not prove on-wire transmission; that
limitation is unavoidable at the generic transport boundary.

### Embedded SDO-server channels

Each SDO-server channel already has an independent protocol state machine.
Scheduling must preserve that independence.

Each channel needs bounded, non-lossy pending output state:

- The default 0x1200 server has its own pending response slot/state.
- Every additional 0x1201+ server has its own pending response slot/state.
- A normal request/response channel cannot validly receive the next protocol
  request until its response is sent.
- Block upload is special because the server produces multiple segments. It
  must produce or retain one segment at a time and must not advance through a
  whole sub-block while merely filling a lower-level adapter queue.
- SDO timeout abort frames are also SDO traffic and follow the configured SDO
  timing policy unless the user selects an immediate policy.

## Event-driven TPDO scheduling

Event-driven TPDO deferral is optional and secondary to SDO scheduling.
Existing event-driven behavior remains the default.

### Payload snapshot is immutable

The PDO engine samples the OD and builds a `CanFrame` at the original event.
That exact frame is the scheduling unit.

If a signal spikes and returns while the spike frame is delayed, the spike
frame must still be transmitted. The scheduling layer must not rebuild the
frame from current OD values at release time and must not replace it with the
return value.

Normal, non-overflow operation is FIFO and preserves every queued snapshot in
order:

```text
spike snapshot -> return snapshot -> later snapshots
```

The scheduler only changes transmission time. It does not change producer
sampling, event detection, mapping, event timer, or inhibit-time semantics.

### Bounded deferral

An event-driven TPDO may opt into:

```rust
TxTiming::PreferOutside {
    max_delay_us: 10_000,
}
```

If the PDO is produced during a protected interval and the next open interval
can be reached within that maximum delay, the original frame is queued until
the opening. If not, the original frame becomes immediately eligible.

For a periodic schedule, the worst intentional wait from the beginning of the
pre-SYNC protected interval through the end of the post-SYNC interval is
approximately:

```text
protected_before_sync_us + protected_after_sync_us
```

On the rover's 10 ms cycle, a 10 ms maximum delay should therefore almost
always be sufficient to move an opted-in event TPDO out of the protected
interval.

### Per-PDO queues

Deferred snapshots are stored in a separate bounded FIFO per TPDO. Per-PDO
queues ensure that one noisy PDO cannot erase another PDO's history or consume
all of a shared telemetry queue.

Queue capacity is a static resource and remains an open tuning/API choice. The
natural TPDO event rate and inhibit time should normally keep these queues
small. Nevertheless, finite memory requires explicit overload behavior.

### Agreed overflow policy: CollapseAndBypassOncePerCycle

The selected overflow behavior is `CollapseAndBypassOncePerCycle`.

On the first overflow of a particular PDO queue in a theoretical cycle:

1. Drop the entire queued history for that PDO.
2. Increment that PDO's overflow event count and add the number of discarded
   snapshots to its dropped-snapshot count.
3. Retain the newly produced/current frame unchanged.
4. Mark that current frame immediately eligible, bypassing the protected
   window for this cycle.
5. Mark the per-PDO bypass as consumed for the current theoretical cycle.

If the transport refuses the immediately eligible frame, the frame remains
pending and immediate; `Immediate` never means "drop on `WouldBlock`".

On subsequent overflows of the same PDO queue in the same theoretical cycle:

1. Collapse the queue again to the newest/current immutable frame.
2. Count the discarded snapshots.
3. Do not grant another protected-window bypass in that cycle.
4. Apply the frame's normal timing policy.

The once-per-cycle bound prevents a pathological high-rate PDO from defeating
the arm's protected interval by overflowing and bypassing repeatedly.

When the schedule is inactive, all PDO timing is already immediate, so the
bypass distinction has no additional effect. Queue collapse and diagnostics
still apply if transport backpressure alone causes an overflow.

The alternative `DropNewest` behavior is deliberately rejected: retaining a
stale backlog while discarding the most recent state is a poor PDO overload
policy. Flushing the whole stale queue immediately is also rejected because it
would turn overload into a burst inside the protected interval.

### PDO diagnostics

At minimum, expose per-PDO or PDO-number-addressed diagnostics for:

- Current queued snapshot count
- Queue overflow count
- Dropped snapshot count
- Number of once-per-cycle bypasses
- Whether an immediate bypass frame remains pending

Diagnostics must use public CANopen `PdoNumber`, not dense engine slots.

## EMCY scheduling

EMCY is user-policy-controlled but defaults to immediate, critical traffic.
The common case must allow an error to bypass held SDO and deferred PDO frames.

The existing EMCY producer has a bounded FIFO. Scheduling must not pop an EMCY
before transport acceptance. It needs peek/confirm behavior or an equivalent
pending state so a transport refusal does not silently discard the emergency.

The precise configurable EMCY queue-overflow behavior remains open. Existing
behavior drops the oldest of four pending EMCYs. That should be reviewed
separately from PDO overflow because EMCY messages are event/history records,
not ordinary latest-state PDO telemetry.

## Priority and arbitration

### Hard ordering

The following traffic should remain hard protocol priority and should not be
delayed by the new scheduler:

1. A due locally produced SYNC.
2. Synchronous TPDOs created by a SYNC pulse, subject to the existing OD 0x1007
   retry/window rules.
3. Boot-up and required control/error-control responses that are not part of
   the user policy surface.

### Policy-managed ordering

Among eligible policy-managed frames, the initial default ordering is:

1. EMCY
2. Event-driven TPDO
3. SDO

Users may configure the relative software priorities of EMCY, event-driven
TPDO, and SDO. Lower CAN ID remains the secondary ordering key. The arbiter
must not allow a held `OutsideOnly` SDO at the head of a FIFO to block a newly
eligible EMCY or PDO.

### Meaning of transport acceptance

`embedded_can::nb::Can::transmit` only says whether the driver accepted a
frame. The stack cannot know whether it is already on the wire, sitting in a
hardware mailbox, in a kernel queue, or buffered by USB adapter firmware.

Therefore:

- Policy is enforced for stack-owned pending frames before acceptance.
- Accepted frames cannot be recalled when a window closes.
- The Linux `protected_before_sync_us` value must include a conservative
  allowance for application scheduling, USB transfer, adapter buffering, and
  any timestamp latency.
- Keeping lower-level queues shallow is part of making the policy effective.

## Embedded insertion points

### New timing/policy module

Add a transport-independent module such as:

```text
canopen-core/src/tx_schedule.rs
```

It should contain the configured-period timing state machine and pure policy
decision types. It must be `no_std`, allocation-free, deterministic, and testable
with explicit `u64` microsecond timestamps.

It must not depend on `Node`, an OD, or a particular transport.

### `Node` state

`canopen-core/src/node.rs` needs opt-in scheduler state plus bounded pending
state for:

- Default SDO-server output
- Each additional SDO-server output
- Per-TPDO deferred snapshot queues and per-cycle bypass markers
- Policy configuration for EMCY, SDO, and individual event-driven TPDOs
- Scheduling and drop diagnostics

The scheduler defaults to disabled/inactive so existing nodes behave exactly
as before.

The public configuration shape remains to be chosen. A post-construction
method such as `node.configure_tx_schedule(...)` avoids immediately breaking
all existing `NodeConfig` struct literals. Per-PDO policy APIs must address
PDOs by `PdoNumber`.

### SYNC observation

The exact received-SYNC insertion point is in `Node::dispatch_frame`, where
the existing `SyncConsumer` has already matched the configured COB-ID and
validated the DLC. Feed that accepted pulse to the new schedule before calling
the existing `handle_sync`.

A locally produced SYNC should also be fed to the schedule only after the
transport accepts it, matching the existing rule that local synchronous PDO
effects occur only after successful SYNC transmission.

### Main transmit orchestration

`Node::process` currently sends from each producer directly. Policy-managed
traffic needs a common eligibility/priority step after RX has been drained and
after protocol producers have created pending work.

The conceptual order is:

1. Boot/control work that is hard immediate.
2. Drain RX, observing SYNC before making scheduling decisions.
3. Produce a due local SYNC and handle accepted local pulse semantics.
4. Retry current synchronous TPDOs within OD 0x1007.
5. Update protocol timers and create pending EMCY, event-TPDO, and SDO work.
6. Repeatedly select the highest-priority currently eligible frame.
7. Remove/confirm it only after `transport.transmit` accepts it.
8. Stop attempting lower-priority work when the transport reports it cannot
   accept more.

This refactor also fixes an existing weakness relevant to the feature: several
current paths discard a frame after ignoring a failed `transmit` result.
Policy-managed frames cannot do that.

### SDO-server response path

`Node::handle_sdo_request` currently builds and transmits a response directly.
Change it to place the response in the selected channel's pending state.

The timeout and block-upload polling in `Node::process` must likewise feed the
same per-channel output state. Do not call `poll_block_upload` repeatedly to
advance and enqueue an entire sub-block without physical admission. Generate
or retain one segment at a time, then advance pacing as frames are accepted.

### Event-driven TPDO producer path

`TpdoEngine::poll` currently returns only `CanFrame`s. The scheduling layer
also needs the originating PDO number/slot so it can locate the per-PDO policy
and queue without guessing from COB-ID ranges.

Add an internal emitted-frame type containing at least:

```rust
struct TpdoEmission {
    slot: usize,       // crate-internal only
    number: PdoNumber,
    frame: CanFrame,   // immutable event-time snapshot
}
```

The public PDO-number rule remains intact; dense slots never escape public
APIs.

The producer should otherwise retain current event-driven sampling behavior.
The scheduler queues the already-built frame; it does not read the OD.

### EMCY producer path

Add non-destructive access to the oldest pending EMCY and remove it only after
transport acceptance. Preserve existing `take_pending` if needed for public
compatibility, but `Node` must use the lossless path.

### Reset behavior

NMT/application/communication reset behavior for the new application-level
schedule is not fully decided. The safe initial recommendation is:

- Clear pending scheduled SDO/PDO work consistently with the owning protocol's
  existing reset behavior.
- Deactivate the observed schedule on communication reset.
- Allow the next bus SYNC to establish a fresh epoch.

This avoids retaining stale phase across a communication reset, but should be
confirmed during implementation review.

## Linux/shared-bus insertion points

### TX queue metadata

`canopen-core/src/bus.rs::TxQueue` currently stores plain `CanFrame`s in one
strict FIFO. A scheduled path needs metadata similar to:

```rust
struct QueuedTx {
    frame: CanFrame,
    policy: TxPolicy,
    eligible_at: Instant,
    sequence: u64,
    completion: Option<...>,
}
```

Existing `send`/`send_async` methods should preserve immediate behavior.
Scheduled/classified submission can be added without forcing monitoring and
other existing clients to use the scheduler.

The queue must be able to select an eligible high-priority entry without an
ineligible SDO at its head blocking EMCY or PDO traffic. It must preserve
protocol ordering within each SDO flow/channel.

### Scheduled CAN pump

The existing `run_pump` drains all RX and then drains the entire TX queue. On
`WouldBlock`, it sleeps and retries the same TX frame for up to 500 ms without
returning to RX. That behavior is incompatible with precise SYNC observation
and priority bypass.

Add a scheduled pump variant or refactor around a shared internal pump that:

1. Drains available RX.
2. Observes a valid exact-COB-ID SYNC before publishing the frame.
3. Updates theoretical cycle/budget state.
4. Selects one highest-priority eligible TX frame.
5. Attempts it once.
6. Confirms and completes the queue entry only on physical transport
   acceptance.
7. Retains it recoverably on `WouldBlock`.
8. Returns to RX observation between TX attempts.

This prevents a queued SDO storm from making the pump blind to subsequent
SYNC pulses and prevents a refused low-priority frame from indefinitely
blocking newly arrived urgent work.

### `SubscriptionPort` and SDO client requests

`SubscriptionPort` is the natural point to label every `SdoDriver` transmit as
SDO traffic. It should accept/configure an SDO `TxPolicy` or have a scheduled
variant. `SdoDriver` itself should remain protocol-focused and need no
SYNC-specific logic.

A completion notification from the physical pump allows
`SubscriptionPort::transmit().await` to apply real backpressure rather than
returning as soon as the frame joins an internal queue.

### Legacy/direct Linux helpers

The direct blocking SDO helpers and `NbCanAsync` bypass `SharedCanBus` and the
central pump. They cannot receive full priority and queue scheduling merely by
changing `SdoDriver`.

The preferred gateway integration is the shared-bus scheduled pump. A later
`AsyncCan` scheduling decorator may support direct helpers, but it is not the
first implementation target and cannot recall frames already accepted by the
adapter.

## Configuration and DSL considerations

The SYNC-aware scheduler is local behavior, not standard CANopen OD data. It
must not invent EDS entries or reinterpret standard PDO communication
parameters.

For hand-written object dictionaries, an optional local DSL parameter could
eventually express per-TPDO behavior, for example:

```rust
tpdo[1](
    transmission_type = event_driven,
    sync_max_delay = 10ms,
) {
    status,
};
```

The exact name is open. Such metadata would configure generated Rust state but
would not be exported to EDS, because there is no corresponding CiA 301
communication parameter.

Runtime setters are still required for EDS-imported ODs and applications that
choose policy dynamically.

Priority and overflow policy syntax should not be added to the DSL until the
Rust policy API is stable.

## Diagnostics and observability

The feature should be measurable rather than opaque. Suggested diagnostics:

### Schedule

- Active/inactive state
- Configured period and protected intervals
- Epoch and current theoretical cycle
- Last confirmed boundary
- Holdover cycles remaining
- Accepted confirmation count
- Pulses ignored in the open interval
- Duplicate pulses ignored for an already-confirmed boundary
- Schedule expirations and reactivations

### SDO

- Current queued request/response count
- Frames admitted in the current theoretical cycle
- Frames deferred by the window
- Frames deferred by the per-cycle budget
- Maximum observed queue depth
- Backpressure wait time/high-water mark
- No dropped-frame counter should be necessary; any nonzero SDO drop is a bug

### PDO

- Per-PDO queue depth
- Per-PDO delayed-frame count
- Per-PDO overflow count
- Per-PDO dropped-snapshot count
- Per-PDO once-per-cycle bypass count
- Per-PDO transport retry count

### EMCY

- Deferred count when a non-immediate policy is selected
- Transport retry count
- Queue overflow/drop count according to the eventual EMCY overflow policy

Counters should saturate rather than wrap where practical.

## Implementation plan

### Phase 1: Pure schedule and policy state machine

1. Add the new `no_std` timing/policy module.
2. Implement configuration validation.
3. Implement inactive activation on first valid pulse.
4. Implement configured theoretical phase calculation.
5. Implement protected/open interval decisions.
6. Implement boundary confirmation without epoch movement.
7. Ignore pulses in the theoretical open interval.
8. Implement holdover and fail-open expiration.
9. Implement `Immediate`, `PreferOutside`, and `OutsideOnly` decisions.
10. Implement per-cycle budget accounting primitives.
11. Test clock wrapping using wrapping microsecond arithmetic.

This phase should be independent of `Node`, `TpdoEngine`, and Linux `std`
types.

### Phase 2: Embedded SDO-server scheduling

1. Add opt-in schedule configuration/state to `Node`.
2. Observe received and locally accepted SYNC pulses at the identified points.
3. Add non-lossy pending output per SDO-server channel.
4. Route immediate SDO responses into pending output rather than directly to
   the transport.
5. Route timeout aborts and block-upload segments through the same path.
6. Ensure block upload produces/admits incrementally.
7. Apply configured SDO timing, priority, and budget.
8. Retain every frame across `WouldBlock`.
9. Clear/abort pending channel state consistently on NMT reset.
10. Add SDO queue/admission diagnostics.

This is the first protocol integration because avoiding SDO storms is the main
feature goal.

### Phase 3: Linux scheduled TX path

1. Add classified/policy-carrying TX queue entries while preserving existing
   immediate APIs.
2. Add eligibility-aware selection without head-of-line blocking.
3. Add a scheduled pump that observes exact SYNC traffic before TX selection.
4. Return to RX between TX attempts.
5. Retain refused frames rather than spinning while blind to RX.
6. Add physical-admission completion signaling/backpressure.
7. Configure `SubscriptionPort` or a scheduled variant to submit SDO policy.
8. Apply the SDO frames-per-cycle budget.
9. Test concurrent SDO clients and priority bypass by immediate traffic.
10. Document USB/adapter latency margins and the fact that accepted frames
    cannot be recalled.

### Phase 4: EMCY policy integration

1. Add peek/confirm semantics to the EMCY producer.
2. Route EMCY through the policy-managed arbiter.
3. Default to immediate critical priority.
4. Retain on `WouldBlock`.
5. Add tests proving held SDO traffic cannot block an immediate EMCY.
6. Decide and document configurable EMCY queue-overflow behavior.

### Phase 5: Optional event-driven TPDO deferral

1. Extend the internal TPDO emission path with source PDO metadata.
2. Add one bounded FIFO per TPDO for immutable `CanFrame` snapshots.
3. Add per-PDO timing and priority policy configuration.
4. Implement bounded `PreferOutside` deferral from original frame eligibility.
5. Preserve FIFO snapshots and payload bytes under normal load.
6. Implement `CollapseAndBypassOncePerCycle` exactly as specified.
7. Make transport refusal retain the current snapshot.
8. Add public `PdoNumber`-based diagnostics.
9. Consider the local DSL parameter only after the runtime API is stable.

### Phase 6: Documentation, examples, and tuning

1. Add an embedded-node example configuration for the rover's 100 Hz bus.
2. Add a Linux shared-bus/gateway example with SDO budget and USB margin.
3. Document testbench/no-SYNC behavior.
4. Measure actual arm RPDO and TPDO burst timing.
5. Choose production `protected_after_sync_us` and
   `protected_before_sync_us` values from measurements.
6. Choose initial Linux and STM32 SDO frame budgets independently.
7. Document diagnostics used to tune queue depth and margins.

## Test plan

### Pure timing tests

- No SYNC means immediate eligibility forever.
- First SYNC establishes epoch and activates protection immediately.
- Exact half-open boundaries produce the documented decisions.
- A pulse in the open interval is ignored and does not refresh holdover.
- A pre-boundary pulse confirms the upcoming theoretical boundary without
  moving it.
- A post-boundary pulse confirms the current boundary without moving it.
- Duplicate pulses cannot confirm the same boundary twice.
- Configured 10 Hz / actual 100 Hz example retains the 100 ms theoretical
  period and a usable open interval.
- Ten-cycle holdover expires at the documented theoretical boundary.
- The next pulse after expiration establishes a new epoch.
- Invalid zero/overlapping-window configurations are rejected.
- Wrapping timestamps behave correctly.

### Policy tests

- `Immediate` is eligible in protected and open intervals.
- `OutsideOnly` waits while active and fails open while inactive.
- `PreferOutside` delays when the next opening fits the maximum delay.
- `PreferOutside` becomes immediate when the opening is too late.
- Re-evaluation never extends a frame's original maximum-delay deadline.
- Priority selection bypasses an ineligible high/backlogged SDO.
- CAN ID is the secondary ordering key.
- Per-cycle budget carries excess work forward without dropping it.

### Embedded SDO tests

- Expedited response is held during protection and sent in the open interval.
- No-SYNC expedited response is immediate.
- Holdover expiry releases a held SDO response.
- Segmented request/response ordering survives several theoretical cycles.
- Block upload advances incrementally and never drops a segment on
  `WouldBlock`.
- Timeout abort follows the configured SDO policy.
- Default and additional SDO channels remain independent.
- A full transport retains pending SDO output.
- NMT reset clears/aborts scheduled transfer state consistently.

### Linux tests

- A queued "read all" storm is limited to the configured frames per cycle.
- Immediate EMCY/PDO traffic bypasses held SDOs.
- Multiple SDO flows retain their protocol ordering.
- Backpressure suspends producers instead of dropping frames.
- `AsyncCan::transmit` completion follows physical pump admission in the
  scheduled variant.
- `WouldBlock` does not prevent RX/SYNC observation.
- Accepted-frame accounting resets at theoretical cycle boundaries.

### Event TPDO tests

- A delayed spike frame retains the spike bytes after the OD returns.
- Spike and return snapshots are transmitted in FIFO order without overflow.
- The next opening inside `max_delay_us` delays the exact original frame.
- An opening beyond `max_delay_us` makes the exact original frame immediate.
- First per-cycle overflow clears that PDO queue, counts every dropped
  snapshot, retains the newest frame, and grants one immediate bypass.
- A second overflow in the same cycle does not grant another bypass.
- Another PDO's queue and counters are unaffected.
- A new theoretical cycle re-arms the one-time bypass.
- `WouldBlock` never discards the bypass frame.
- Inactive scheduling remains immediate.

### EMCY tests

- Default immediate EMCY bypasses SDO backlog.
- A configured deferred EMCY retains exact ordering.
- `WouldBlock` retains rather than drops the oldest EMCY.
- Queue-overflow behavior matches the later explicit EMCY decision.

## Compatibility and resource constraints

- Scheduler configuration must be opt-in; unconfigured behavior remains the
  current immediate behavior.
- Embedded queues use `heapless` storage only.
- No dynamic dispatch, allocation, threads, or timers are required by the
  pure/embedded timing engine.
- Per-PDO queue capacity materially affects `Node` memory. It should be an
  explicit static choice or a carefully documented fixed capacity.
- Adding another `Node` const generic has broad type-alias/API consequences;
  prefer a design that does not expose unnecessary scheduler internals in the
  main `Node` type unless configurability clearly requires it.
- New local scheduling metadata must not alter EDS conformance or standard OD
  wire behavior.
- Custom PDO and additional-SDO COB-IDs must continue to be routed by exact
  configuration, never by predefined-range heuristics.

## Open decisions and tuning parameters

The behavior above is sufficiently specified to resume implementation later,
but these values/API choices remain intentionally open:

1. Production `protected_after_sync_us` and `protected_before_sync_us` for the
   rover's 10 ms cycle.
2. Initial Linux SDO request budget per theoretical cycle.
3. Initial EE SDO response budget per theoretical cycle.
4. Whether a minimum inter-SDO-frame gap is useful in addition to a frame
   count budget.
5. Per-PDO queue capacity and whether it is fixed, generated, or a const
   generic.
6. Exact `TxPriority` representation and default numeric values.
7. Whether SDO policy is one node-wide policy or independently configurable
   per server channel.
8. Exact post-construction versus `NodeConfig` API shape.
9. Whether communication reset always deactivates the observed schedule.
10. EMCY queue-overflow policy.
11. Whether heartbeat/LSS should ever join the user-configurable policy
    surface; current recommendation is no.
12. Exact optional TPDO DSL spelling (`sync_max_delay`, `tx_policy`, etc.).
13. Linux timestamp source and the conservative USB/adapter latency margin.

## Acceptance criteria

The feature is complete when all of the following hold:

- An unconfigured node and an inactive/no-SYNC configured node transmit as
  before.
- A configured node activates on the first valid SYNC and follows the fixed
  configured period rather than measured pulse frequency.
- Unexpected high-frequency SYNC cannot permanently close the background
  window.
- Missing SYNC holds the theoretical schedule for the configured number of
  cycles, then fails open.
- Linux SDO request storms and embedded SDO responses are admitted only by
  policy and budget, with no SDO frame loss.
- Immediate higher-priority traffic can bypass held SDO work.
- Event-driven TPDO deferral preserves original payload snapshots exactly.
- Per-PDO overflow performs `CollapseAndBypassOncePerCycle` and exposes loss
  diagnostics.
- EMCY defaults to immediate and is retained across transport backpressure.
- All embedded state remains statically allocated and `no_std`/`no_alloc`.
- Relevant unit, integration, and interop tests pass without changing standard
  CANopen wire semantics.
