# Code-first gateway abstractions

> Prototype status (2026-08-30): implemented as `canopen-gateway`, with unit
> tests and an integration test using `device_from_eds!`. It deliberately has
> no ROS dependency.

## What was repeated in the existing gateways

A shallow review covered the RES, DDU, and TurnGrip/end-effector gateways. The
protocol details differ, but their reliable parts converge:

- TPDOs are decoded with the EDS-generated exact-COB-ID codec, converted to a
  ROS message, published once, and forgotten.
- A separate last-observation cache exists only for aggregate gateway status;
  it has an explicit per-PDO freshness timeout.
- RPDO topic traffic has one capacity-one slot per PDO. A new topic sample
  replaces an older unsent one rather than building a command backlog.
- RPDO slots need fair selection. A continuously refreshed low-numbered PDO
  must not starve another PDO.
- Live commands have a short finite lifetime. A frame that sat behind adapter
  backpressure must not execute later as a stale actuator command.
- A selected frame is not gone until the transport accepts it. On
  `WouldBlock`, it remains pending; a newer topic sample may supersede it.
- SDO requests are serialized per SDO channel. The gateways either reject a
  concurrent service or use a bounded queue. Configuration/control work may
  take priority over background inventory.
- SDO queue wait and active protocol-response timeout are different clocks.
- Every service caller must receive a terminal result on timeout, rejection,
  disconnect, or shutdown. Raw SDO abort values are part of that result.

Those are library policies. Topic names, ROS QoS, message field conversion,
read-only mode, commissioning restrictions, reconnect supervision, and special
multi-step operations (such as DDU emergency stop) remain application policy.

## Chosen API shape

The prototype uses small typed route values rather than a central dynamic
registry:

```text
CAN frame -> PdoToTopic<decode, map> -> owned ROS message -> publish now

ROS message -> TopicToPdo<encode, validity> -> LatestPdoSet[slot]
            -> prepare -> CAN transport -> confirm

ROS request -> ServiceToSdo<start, finish> -> ServiceQueue[channel]
            -> SdoCall::execute -> exact SdoReply/SdoFailure -> ROS response
```

Reasons:

- ROS topics have different generated Rust message types. A single
  `HashMap<Pdo, Topic>` would need type erasure and would hide useful compiler
  errors in downcasts or boxed callbacks.
- Route values still read like a mapping table at the declaration site, while
  closures provide explicit field conversion and validation without YAML.
- Buffering belongs to direction-specific primitives. PDO-to-topic delivery is
  stateless; topic-to-PDO delivery is latest-only; SDO services preserve edges
  and serialize. A generic channel abstraction would blur these semantics.
- The transport acceptance boundary stays visible through `prepare`/`confirm`.
  An API that merely returns or sends a frame cannot distinguish queued from
  accepted and tends to lose data on `WouldBlock`.

## Alternatives worth keeping in mind

### Dynamic route registry

Store boxed `Fn(&CanFrame)` handlers and boxed publishers in a `Gateway`.
Declaration is compact and central dispatch is convenient, but ROS types and
publish failures disappear behind type erasure. It is most attractive only if
a future ROS adapter crate owns the concrete middleware types.

### Declarative macro

A `gateway! { tpdo1 => topic(...), ... }` macro could generate the route values
and dispatch boilerplate. It should be layered on the current primitives, not
replace them. Starting with a macro would freeze syntax before the RES/DDU
migrations reveal which special cases are real.

### Async stream/sink traits

Model topics and CAN as streams/sinks and compose them. This reads elegantly,
but executor traits do not encode latest-only replacement, finite command age,
transport acceptance, or one-active-SDO-per-channel. Those policies would
still need the state machines now provided explicitly.

## Exact semantics

### `PdoToTopic`

- `Ok(None)`: unrelated frame; no state changes and no replay.
- `Ok(Some(message))`: one matching, valid PDO and one converted message.
- `Decode(error)`: the route matched but the wire payload was invalid.
- `Map(error)`: wire payload was valid but application validation failed.
- No internal queue or retained previous message.

`FreshValue` is deliberately separate. At exactly `max_age`, a value is stale.
`get_fresh` never returns stale data; `last` is an explicit diagnostic opt-in.

### `TopicToPdo` / `LatestPdoSet`

- Each slot has capacity one; replacement is counted.
- Every route chooses `Validity::For(duration)` or the conspicuous
  `Validity::Forever`; there is no default.
- A finite sample is usable only while `age < duration`. Equality is expired.
- Selection is round-robin after each confirmed frame.
- Only one frame is prepared at once. Repeated preparation retries it.
- A newer value in the prepared slot supersedes the unaccepted frame.
- Confirmation is generation checked, so confirming an older selection never
  clears a newer sample.
- Transport failure may abandon the selection for later fair reselection; its
  original age is retained.

### `ServiceToSdo` / `ServiceQueue`

- One queue instance represents one SDO server channel and permits one active
  transfer.
- Discipline is explicit: reject concurrent, bounded FIFO, or bounded priority
  with FIFO order within equal priority.
- Queue capacity counts waiting requests, not the active transfer.
- Queue age applies only before start and expires at equality.
- Expired, cancelled, and drained requests are returned to the adapter; they
  are never silently discarded.
- The queue cannot time out an active transfer. `SdoCall::execute` owns that
  protocol timeout, because only the SDO runner can safely cancel its state.
- Completion uses a generation token; a late completion cannot release a newer
  transfer.
- Upload/download size is currently exactly bounded by `SdoDriver`'s 256-byte
  client buffer.
- `SdoAbort` retains every raw 32-bit abort code. `known()` classifies standard
  CiA 301 values without rewriting unknown/profile-specific values.

## Migration order

1. Migrate the TurnGrip gateway first: it is one node with three TPDOs, two
   RPDOs, and raw read/write SDO services, so it exercises the primitives with
   little unrelated orchestration.
2. Migrate DDU next: retain its emergency-stop transaction above the generic
   queues, and use cancellation to terminally respond to displaced services.
3. Migrate RES last: it validates multi-node routes, priority SDO scheduling
   (configuration over inventory), and `FreshValue` status snapshots.

Do not move message field conversions into this crate. They are the readable,
code-reviewed link between firmware OD names and ROS interface names.
