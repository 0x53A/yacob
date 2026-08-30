# canopen-gateway

Code-first, middleware-neutral primitives for mapping a remote CANopen object
dictionary onto topics and services.

> **Prototype:** this crate is not ready to move to `main`. The known API and
> behavioral blockers are summarized below and tracked with acceptance criteria
> in [`_Tasks/gateway-abstractions.md`](../_Tasks/gateway-abstractions.md#pre-main-integration-blockers).

The crate intentionally stops below ROS. Generated message types, publishers,
subscriptions, and service responders stay in the application, while the hard
behavioral contracts live here:

| Direction | Primitive | Semantics |
|---|---|---|
| PDO -> topic | `PdoToTopic` | Stateless; exactly one message per matching valid frame; unrelated frames and malformed frames are distinct. |
| topic -> PDO | `TopicToPdo` + `LatestPdoSet` | One latest-only slot per PDO; mandatory explicit validity; fair across PDOs; data clears only after transmit acceptance. |
| service -> SDO | `ServiceToSdo` + `ServiceQueue` | Code-defined conversion; one active transfer per SDO channel; explicit reject, bounded FIFO, or bounded priority policy; queued requests expire and are returned to their caller. |

`FreshValue` is separate from delivery. It supports gateway status and snapshot
topics without accidentally returning stale state.

## Intended ROS shape

```rust,ignore
let tpdo1 = PdoToTopic::new(
    move |frame| match device.decode(frame)? {
        Some(DeviceTpdo::Tpdo1(payload)) => Ok(Some(payload)),
        Some(_) | None => Ok(None),
    },
    |pdo| Ok::<_, Infallible>(StatusMsg::from(pdo)),
);

// In the CAN receive loop: no intermediate channel and no replay.
if let Some(message) = tpdo1.on_frame(&frame)? {
    status_publisher.publish(message)?;
}

let rpdo1_slot = PdoSlot::<3>::new(0)?;
let rpdo1 = TopicToPdo::new(
    rpdo1_slot,
    Validity::For(Duration::from_millis(100)),
    move |msg: CommandMsg| Ok::<_, ValidationError>(
        device.rpdo1(DeviceRpdo1::try_from(msg)?)
    ),
);

// In the ROS subscription task: replaces an older unsent RPDO1 command.
rpdo1.on_message(&mut outgoing_pdos, message, Instant::now())?;

// In the sole CAN writer: confirm only after the adapter accepts the frame.
if let Some(prepared) = outgoing_pdos.prepare_next(Instant::now()) {
    match can.transmit(&prepared.frame) {
        Ok(_) => { outgoing_pdos.confirm(prepared); }
        Err(nb::Error::WouldBlock) => {} // selection remains pending
        Err(nb::Error::Other(_)) => { outgoing_pdos.abandon(prepared); }
    }
}
```

An SDO service uses a `ServiceQueue` to serialize calls. `SdoCall::execute`
runs the selected operation through the existing `SdoDriver` and a
`SharedCanBus` filtered subscription. SDO aborts retain the exact remote
32-bit value, including manufacturer-specific codes.

## Known prototype limitations

- `LatestPdoSet::abandon` retains the sample but does not advance the
  round-robin cursor. A repeatedly failing slot can therefore be selected
  again immediately and delay another ready slot. Do not rely on fair
  reselection after transport errors yet.
- `PreparedPdo` exposes its slot, generation, and timestamps as public fields.
  This lets external callers construct confirmation values that were not
  issued by `LatestPdoSet`; those invariant-bearing fields must become private
  before the API is considered stable.
- A direct generated `decode_tpdoN` call reports an unrelated frame as
  `WrongCobId`. A `PdoToTopic` route must first dispatch with `device.decode()`,
  as above, or otherwise prefilter the exact COB-ID. Treating
  `decode_tpdoN(frame).map(Some)` as a general bus route incorrectly reports
  unrelated traffic as malformed.
- Although multiple `ServiceQueue` instances can serialize separate logical
  channels, `SdoCall::execute` currently constructs `SdoDriver::new(target)`
  and therefore uses only the default SDO request/response COB-IDs. Additional
  configured SDO server channels are not supported by the execution API yet.
- The `timeout_us` passed to `SdoCall::execute` is one deadline for the entire
  transfer. It is not an inactivity timeout reset by segmented-transfer
  progress. Gateways must size it for the complete transfer until this policy
  is changed.
- `SdoFailure::Transport` intentionally has a middleware-stable shape, but
  conversion currently discards the adapter's underlying error. The runner
  must log or otherwise retain that diagnostic before mapping a failure into a
  service response.
