# Architecture discussion notes — 2026-09-07

These are the reviewer's recommendations, not user decisions. The user wants
larger architectural changes discussed before implementation. The evidence and
local correctness fixes are in [review-2026-09-07.md](review-2026-09-07.md).

## 1. Keep the layers; tighten the contracts between them

Current structure:

```text
inline DSL / EDS -> canopen-derive -> OD + typed device codecs
                                      |              |
                                Node / servers   SDO client/driver
                                      |              |
                         nb::Can / AsyncCan / shared-bus ports
                                      |
                         platform adapters and physical CAN

browser apps -> CanTransport callbacks (a separate delivery contract)
```

The portable state-machine core and generated wire APIs are worthwhile. A
single generic abstraction for everything is not necessary. The contracts
that should be consistent are frame identity, acceptance/refusal, disconnect,
ordering, timeout, cancellation and queue bounds.

My recommendation: retain `embedded-can` at hardware-adapter boundaries and
make its displacement behavior explicit. Share conformance scenarios across
adapters even where their execution APIs differ. Forcing browser callbacks
through a thread-oriented bus would not itself improve correctness.

## 2. Reliable TX first; SYNC scheduling is a separate optional policy

The existing `protocol-reliability-followups.md` identifies the real problem.
Its implication that every repair requires one large priority scheduler goes
too far. A server can own a pending response and commit on acceptance without
also deciding the entire rover's background-traffic schedule.

Two plausible designs:

| Option | Benefit | Cost / question |
|---|---|---|
| Protocol-owned pending work, Node arbitration | Ownership and retry/commit state stay close; natural SDO per-channel backpressure | Every producer needs a pending/accepted interface; Node still needs fair selection |
| Bounded Node outbox with explicit admission | Central ordering, capacity and transport handling | Must retain protocol ownership/generation and define what happens before a response can be admitted |

I lean toward protocol-owned pending work plus a small bounded dispatcher.
Application commands, EMCYs, heartbeats and periodic process images do not all
share the same loss/coalescing semantics. Neither design should claim that
driver acceptance equals a physical bus acknowledgement.

Keep the proposed SYNC-aware protected-window scheduler optional and separate.
Its hardcoded policy choices, such as `CollapseAndBypassOncePerCycle`, are
proposals to justify against measured bus behavior, not required foundations
for fixing a lost SDO reply. Do not resample an already committed synchronous
PDO just because it is retried.

## 3. State observation and command execution need different semantics

The useful part of `application-models.md` is its distinction between latest
state and commands whose individual occurrences matter. The three named models
are possible wrappers around those semantics; they need not become three
separate frameworks introduced in one migration.

My recommendation is an incremental order:

1. Specify immediate accepted-write/command observation, including what the
   application knows before SDO success is sent.
2. Provide level-based OD observation separately; make `next_change`'s current
   read-at-drain behavior explicit during migration.
3. Add per-channel deferred SDO completion only for operations whose success
   must mean the application work completed. Tokens need lifecycle generations.
4. Add ergonomic inline/reactive/scan wrappers if real consumers benefit.

A command that means “start work” can often acknowledge acceptance and expose
status separately. That should not be confused with acknowledging durable
flash completion. The choice belongs to the specific object contract.

RPDO structural validity and whole-command validity also differ. Preserve the
ability to express ordinary independent process data, while offering a clear
application acceptance boundary for coupled commands. Do not claim that a
heartbeat or structurally valid RPDO proves fresh operator intent.

## 4. Lifecycle must be observable without forcing one process architecture

“The application owns supervision” is reasonable. “Therefore library queues
never close” does not follow. A restartable in-process subsystem and a process
that exits on failure both need bounded teardown and observable failure.

I recommend explicit close/wakeup behavior for bus ports and one transfer
owner per SDO channel. Multiple subscriptions are useful for observation, but
do not grant concurrent transaction authority. Define how cancellation,
timeout and reset affect queued requests and delayed replies before allowing
channel reuse. Merely dropping an async future cannot retract bytes already
queued in a USB adapter or browser socket.

Similarly, define bounded work per Node/pump turn and where timestamps are
taken. An offline test can establish bounded service and prevent replay across
state transitions. Hardware tests later establish real latency and bus timing.

## 5. One schema model, several front ends

Today validation is distributed between DSL parsing, EDS parsing, generated
OD code, Node construction, SDO writes and public mutable engine configs. The
runtime DCF parser is separate again. This permits differences in which values
are accepted and what they mean.

I recommend a neutral OD/schema representation independent of `syn`, with
explicit support/validation rules, then separate DSL/EDS/code-generation
adapters. This would also support the runtime EDS browser already proposed in
the notes. Do it incrementally: first stop unchecked conversions and silent
mapping loss; don't require a new crate before correcting a bad value.

Separate supported protocol features from offered API names. A type that can
name RTR or an EDS that advertises an object is not proof the runtime executes
it. Conformance should be stated as tested scenarios and known omissions,
with the target CiA edition identified when an edge case depends on it.

## Follow-up proposal: cancellation tickets and stale-command buffers

Lukas raised the possibility of enqueue returning a cancellation ticket, and
asked whether whole-buffer removal would usually be more useful. This is a
question/proposal, not an approved API or a decision to add buffering.

The useful distinction is not simply buffered versus sent:

```text
producer intent -> library queue -> driver/OS/USB/browser queue -> CAN wire
                   cancellable     cancellation depends on adapter support
```

The generic `embedded-can` interface reports acceptance into a transmit
buffer, not completion on the physical bus. Therefore an honest ticket needs
outcomes such as removed-before-handoff, already-handed-off, or expired, rather
than promising removal until physical transmission. If a target adapter can
abort pending hardware frames, expose and test that stronger capability
explicitly. Removing a sent command never undoes its physical effect.

My recommendation for discussion:

- Tag command work with an owner/session generation; invalidate that generation
  on authority loss, disconnect or reset. Reject late enqueue attempts from
  that generation as well as purging its queued frames.
- Make bulk cancellation by generation the main operation for stale commands.
  A shared bus may also hold another node's SDO response, an EMCY, or a newer
  stop command; a blanket FIFO clear can erase unrelated necessary work.
- Use latest-value replacement for continuous setpoints where the application
  contract permits it. Enqueueing every intermediate setpoint and cancelling
  old tickets later is more complicated and easier to get wrong.
- Keep individual tickets as an optional means of cancellation/correlation
  for not-yet-started requests and diagnostics. Ticket identifiers need a
  generation so slot reuse cannot cancel a different frame.
- Use a whole transport-buffer purge for teardown when the buffer is genuinely
  dedicated to that connection. Pair it with producer invalidation and an
  explicit policy for downstream queues that cannot be recalled.

Once an SDO transfer has started, cancellation is a protocol operation as well
as a queue operation. It may require an abort and channel cleanup; dropping
an arbitrary segment does not safely cancel the transfer. A Node server also
needs to know if a response it believes pending has been cancelled, rather
than advancing its state behind the queue's back.

This still leaves three distinct safeguards: queue residence/expiry, session
ownership, and a device-enforced command deadline. A ticket alone proves none
of them. The concrete API and buffer ownership remain open for discussion.

## Suggested next discussion

Start with TX ownership and lifecycle (notes 2 and 4). They constrain both the
node APIs and the master apps, and explain several otherwise unrelated bugs.
Then decide the minimum command/deferred-operation surface needed by DDU,
TurnGrip and RES. Consistent semantics matter more than making their UIs or
task layouts look identical.

The existing architecture notes are useful historical reasoning. “Agreed,”
“must stay this way,” and similar language should be read alongside current
code, demonstrated needs and explicit user instructions. This document does
not promote its own preferences to requirements.
