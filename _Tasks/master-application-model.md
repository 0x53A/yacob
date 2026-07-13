# Master Application Model — API Design

> **Status: std flavor implemented (2026-07-06)** — see "Implementation
> status" at the end. The no-alloc intrusive flavor remains a sketch.
> Complements `application-models.md`, which is primarily about local
> `Node<OD>` / slave-device application structure. This note covers
> master-side and mixed-role applications: one CAN bus, independent protocol
> consumers, local nodes, and SDO clients that do not own the receive loop.

## Problem

The current `CanDemux` model is a workaround around a single-owner receive
stream:

- An SDO operation temporarily owns RX through `DemuxSdoPort`.
- Non-SDO frames are routed into side buffers while the SDO operation is
  receiving.
- EMCY/heartbeat/PDO consumers only see frames that happened to arrive while
  some other demux method was polling the bus.

That is not a real master application model. A master or mixed-role node
usually wants:

- one central task owning the physical CAN RX stream;
- several independent consumers observing the same frames;
- optionally one or more local `Node<OD>` instances handling frames addressed
  to this device;
- an SDO client running as an independent async utility, not inside the
  application event loop;
- one general CANopen event stream for application logic, rather than separate
  library-owned EMCY/PDO/heartbeat streams.

CAN frames are events. If multiple consumers care about a frame, each consumer
must get its own copy. Dropping from one consumer must not steal the frame from
another.

## Proposed model

Treat the bus as a role-agnostic no-alloc SPMC broadcast of raw `CanFrame`s:

```text
CAN RX task
    -> SharedCanBus::publish(frame)
        -> Subscription<A>::queue
        -> Subscription<B>::queue
        -> Subscription<C>::queue
```

Consumers subscribe to raw frames and apply their own filters:

```rust
let mut events = NODE_EVENTS.receiver();
let sdo = SdoClient::new(node_id, CAN_BUS.sender(), SDO_RX.receiver());

loop {
    let frame = events.recv().await;
    let Some(event) = CanOpenEvent::decode(frame) else {
        continue;
    };
    if !event.matches_node(node_id) {
        continue;
    }

    match event {
        CanOpenEvent::Heartbeat(hb) => { /* update monitor */ }
        CanOpenEvent::Emcy(emcy) => { /* record fault */ }
        CanOpenEvent::Tpdo(pdo) => { /* update process image */ }
        CanOpenEvent::SdoResponse(_) => { /* ignored here; SDO task has its own subscription */ }
        _ => {}
    }
}
```

The same bus is used whether the application is acting as a pure master, a
pure node, or both. A local `Node<OD>` is another consumer of the shared RX
stream plus a producer on the shared TX path. That allows one device to:

- serve its own OD as a CANopen node;
- consume TPDOs from other nodes;
- monitor EMCY/heartbeat traffic;
- initiate SDO transfers to other nodes.

The bus does not decide which frames are "node" frames and which are "master"
frames. Consumers decide by filtering their subscription. This is necessary
for gateways and controller devices that are both CANopen slaves and masters.

## Alternative: lowest-level CAN fanout

The raw-broadcast model is not the only viable shape. We could keep local-node
handling and master-side observation as parallel concepts and fan out raw
frames twice in the lowest-level CAN task:

```text
CAN RX task
    -> local node raw inbox
    -> master/application raw bus
```

The CAN task still does not classify. It only forwards the same raw frame into
two parallel raw domains:

- the local node side, where `Node<OD>` can filter/handle frames addressed to
  it;
- the master/application side, where SDO clients and application event streams
  subscribe independently.

This keeps node and master concepts parallel without asking one abstraction to
serve both APIs. It may be a better mental model for devices that are primarily
CANopen nodes but also consume TPDOs or send SDOs to other nodes.

The tradeoff is RAM/copy cost: every received frame is copied into both raw
domains, and then the master/application side may copy again to its own
subscribers. For CAN classic frame rates this is probably acceptable on the
targets we care about, but it should be an explicit design choice.

The conservative split is:

- lowest-level CAN task always owns physical RX/TX;
- no protocol classification happens in that task;
- either one shared raw bus feeds all consumers, or the task forwards each
  frame into two raw domains: local-node and master/application;
- optional filters/classifier adapters can be layered on top of either raw
  domain.

The SDO client is just another subscriber. It filters for SDO responses from
its node and ignores everything else from its own subscription queue:

```rust
impl SdoClient {
    async fn read_u16(&mut self, index: u16, subindex: u8) -> Result<u16, SdoError> {
        self.tx.send(build_upload_request(self.node, index, subindex)).await;

        loop {
            let frame = self.rx.recv().await;
            match CanOpenEvent::decode(frame) {
                Some(CanOpenEvent::SdoResponse { node, data }) if node == self.node => {
                    return self.state.on_response(data);
                }
                _ => {}
            }
        }
    }
}
```

## No-alloc subscription shape

The bus should not carry a queue-capacity generic. Each subscriber owns its own
queue and chooses its own capacity:

```rust
static CAN_BUS: SharedCanBus<CanFrame> = SharedCanBus::new();

static NODE_EVENTS: Subscription<CanFrame, 16> = Subscription::new();
static SDO_RX: Subscription<CanFrame, 4> = Subscription::new();
static LOGGER: Subscription<CanFrame, 32> = Subscription::new();
```

Use intrusive static registration instead of a bus-level subscriber cap:

```rust
CAN_BUS.subscribe(&NODE_EVENTS)?;
CAN_BUS.subscribe(&SDO_RX)?;
CAN_BUS.subscribe(&LOGGER)?;
```

Internally, each `Subscription<T, N>` is a static list node:

```rust
struct SharedCanBus<T> {
    head: Mutex<Option<&'static dyn SubscriberNode<T>>>,
    tx: Sender<T>,
}

struct Subscription<T, const N: usize> {
    queue: Mutex<Deque<T, N>>,
    signal: Signal<()>,
    next: Mutex<Option<&'static dyn SubscriberNode<T>>>,
}
```

Publishing walks the list and copies the frame into each subscriber queue:

```rust
fn publish(&self, frame: T)
where
    T: Copy,
{
    let mut node = self.head();
    while let Some(sub) = node {
        sub.push(frame);
        node = sub.next();
    }
}
```

Constraints:

- subscriptions are `'static`;
- registration is startup-only;
- unregistering is not supported initially;
- `subscribe()` should reject double-registration of the same subscription;
- each subscriber tracks its own overflow count;
- overflow policy is per-subscriber, initially drop-oldest or drop-newest by
  explicit choice.

This avoids both allocation and an artificial `MAX_SUBSCRIBERS` generic on the
bus. RAM usage is explicit at each subscription site.

## Embassy sketch

```rust
static CAN_BUS: SharedCanBus<CanFrame> = SharedCanBus::new();
static NODE_EVENTS: Subscription<CanFrame, 16> = Subscription::new();
static SDO_RX: Subscription<CanFrame, 4> = Subscription::new();

#[embassy_executor::task]
async fn can_rx_task(mut can_rx: CanRx) -> ! {
    loop {
        let frame = can_rx.receive().await;
        CAN_BUS.publish(frame);
    }
}

#[embassy_executor::task]
async fn can_tx_task(mut can_tx: CanTx) -> ! {
    loop {
        let frame = CAN_BUS.next_tx().await;
        can_tx.transmit(&frame).await;
    }
}

#[embassy_executor::task]
async fn node_events_task(node: NodeId) -> ! {
    let mut rx = NODE_EVENTS.receiver();

    loop {
        let frame = rx.recv().await;
        let Some(event) = CanOpenEvent::decode(frame) else {
            continue;
        };
        if !event.matches_node(node) {
            continue;
        }

        match event {
            CanOpenEvent::Heartbeat(hb) => HEARTBEAT_STATE.update(hb),
            CanOpenEvent::Emcy(emcy) => ERROR_STATE.record(emcy),
            CanOpenEvent::Tpdo(pdo) => PROCESS_IMAGE.apply_tpdo(pdo),
            _ => {}
        }
    }
}

#[embassy_executor::task]
async fn control_task(node: NodeId) -> ! {
    let mut sdo = SdoClient::new(node, CAN_BUS.sender(), SDO_RX.receiver());

    loop {
        match sdo.read_u16(0x6041, 0).await {
            Ok(statusword) => CONTROL_STATE.set_statusword(statusword),
            Err(_) => CONTROL_STATE.set_sdo_fault(),
        }
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    CAN_BUS.subscribe(&NODE_EVENTS).unwrap();
    CAN_BUS.subscribe(&SDO_RX).unwrap();

    let (can_rx, can_tx) = init_can().split();
    spawner.spawn(can_rx_task(can_rx)).unwrap();
    spawner.spawn(can_tx_task(can_tx)).unwrap();

    let drive = NodeId::new(1).unwrap();
    spawner.spawn(node_events_task(drive)).unwrap();
    spawner.spawn(control_task(drive)).unwrap();
}
```

## Decoding layer

Raw bus subscription and CANopen decoding should be separate layers.

```rust
enum CanOpenEvent {
    Nmt { command: u8, target: Option<NodeId> },
    Sync,
    Emcy(EmcyMessage),
    Heartbeat { node: NodeId, state: NmtState },
    Tpdo { pdo_num: u8, node: NodeId, data: CanFrame },
    Rpdo { pdo_num: u8, node: NodeId, data: CanFrame },
    SdoResponse { node: NodeId, data: [u8; 8] },
    SdoRequest { node: NodeId, data: [u8; 8] },
    Unknown(CanFrame),
}
```

Filtering can be built as helper methods, not as separate streams owned by the
bus:

```rust
impl CanOpenEvent {
    fn decode(frame: CanFrame) -> Option<Self>;
    fn node(&self) -> Option<NodeId>;
    fn matches_node(&self, node: NodeId) -> bool;
}
```

Keep protocol-specific helpers small and optional:

- `HeartbeatMonitor` consumes decoded heartbeat events.
- `EmcyMessage::parse` remains a useful core parser.
- PDO mapping helpers may consume decoded PDO events.
- `SdoClient` consumes raw or decoded SDO response events from its own
  subscription.

## std affordances

The no-alloc core should not require `std`, but a `std` master API can be much
more ergonomic:

```rust
let master = CanOpenMaster::socketcan("can0")?;
let node = master.node(NodeId::new(1).unwrap());

let mut events = node.events();
std::thread::spawn(move || {
    while let Ok(event) = events.recv() {
        match event {
            CanOpenEvent::Emcy(emcy) => eprintln!("EMCY: {emcy:?}"),
            CanOpenEvent::Heartbeat(hb) => eprintln!("heartbeat: {hb:?}"),
            _ => {}
        }
    }
});

let status = node.sdo().read_u16(0x6041, 0)?;
```

An async-std/tokio flavor can wrap the same core idea:

```rust
let master = CanOpenMaster::new(transport).spawn();
let mut events = master.node(node).events();

tokio::spawn(async move {
    while let Some(event) = events.recv().await {
        handle_event(event);
    }
});

let status = master.node(node).sdo().read_u16(0x6041, 0).await?;
```

## Relationship to `CanDemux`

`CanDemux` should not grow into the master abstraction. It routes frames only
when some demux method happens to be receiving from the underlying transport,
which makes EMCY and heartbeat monitoring incidental.

Likely path:

1. Keep `EmcyMessage::parse` and other protocol parsers.
2. Stop adding protocol-specific buffers to `CanDemux`.
3. Implement the shared bus/subscription core.
4. Rebuild master examples around one event subscription plus independent SDO
   clients.
5. Deprecate or remove `CanDemux` once examples and users have migrated.

## Open questions

- Overflow policy: drop-oldest, drop-newest, or configurable per subscription?
- Should `Subscription::recv()` return an overflow marker/count with the next
  frame?
- Does `SdoClient` subscribe to raw frames and decode internally, or subscribe
  to a decoded event adapter?
- Should `SharedCanBus` own TX queueing, or should TX be a separate
  `CanTxHandle` type?
- Exact trait bounds for `T`: probably `Copy` initially, matching `CanFrame`.
- Whether a node-filtered receiver helper is worth providing:
  `subscription.decode().for_node(node)`.

## Implementation status (2026-07-06)

The std flavor is implemented in `canopen-core/src/bus.rs` (behind the `std`
feature — cross-platform, not in canopen-linux) and the decode layer in
`canopen-core/src/events.rs` (no_std). Decisions made during review:

- **Publish-side filters**: subscriptions may carry a
  `Box<dyn Fn(&CanFrame) -> bool + Send + Sync>`. The filter runs against a
  *reference* in the publish path; the copy happens only on accept. This keeps
  small queues semantically meaningful (an SDO subscription only ever holds
  SDO responses) and largely dissolves the overflow question for SDO.
  Codegen note: a boxed closure is an indirect call the optimizer cannot
  inline like an ID mask, but at CAN frame rates the difference is noise; the
  closure wins on expressiveness. The future no-alloc flavor will take plain
  `fn(&CanFrame) -> bool` pointers (no captures in `'static` storage).
- **TX is not part of the bus**: `TxQueue` is a separate bounded queue with
  blocking, non-lossy `send` (a silently dropped SDO request becomes an
  inscrutable timeout). The pump gives up on a `WouldBlock`ing transmit after
  500ms (`PumpError::TxStuck`).
- **No transport lifecycle handling**: a dead adapter ends `run_pump` with an
  error; the application tears down and is restarted by its supervisor
  (systemd etc.). No `TransportDown/Up` events on the bus.
- **SDO client reuse**: `SubscriptionPort` implements `AsyncCan` over
  `(TxQueue, Subscription)`, so `SdoDriver` and generated EDS clients run
  unchanged on the bus. Answering the open question: the SDO client
  subscribes to raw frames (ideally publish-side filtered), no decoded-event
  adapter.
- **Overflow**: per-subscription `OverflowPolicy` (`DropOldest` default,
  right for monitoring; `DropNewest` opt-in), overflow counter via
  `take_overflow_count()`; `recv()` stays clean (no overflow marker).
- **Ordering invariant**: per-subscriber FIFO in publish order — documented
  API guarantee; SDO correctness depends on it.
- **Decode layer**: `CanOpenEvent::decode(&CanFrame) -> Option<_>` replaces
  the sketched `Unknown(CanFrame)` variant. Documented assumption: the
  pre-defined connection set. PDO `pdo_num`/`node` values are COB-ID range
  inferences for lightweight monitoring, not ground truth; remapped PDOs inside
  a pre-defined range are classified as whatever range they land in, while
  remapped PDOs outside those ranges decode as `None`. Correct remapped-PDO
  interpretation belongs in a later OD/config-aware decoder layered above the
  raw bus, not in `CanDemux`.
  Broadcast NMT `matches_node()` = true for every node; `Sync` matches none.
- **Two-domain fanout alternative**: rejected — a local `Node<OD>` is just
  another subscriber on the general bus.
- **CanDemux**: unchanged (no new protocol buffers, per "Relationship to
  CanDemux"); deprecation deferred until users (hil-tests, sdo_helpers)
  migrate to the bus.

First consumer: frost-arm-can-bridge (pump thread owns the transport;
control loop and SDO ports are subscribers).
