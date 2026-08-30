# canopen-gateway

Code-first, middleware-neutral primitives for mapping a remote CANopen object
dictionary onto topics and services.

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
    move |frame| device.decode_tpdo1(frame).map(Some),
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

An SDO service uses one `ServiceQueue` per configured SDO server channel. The
queue serializes calls; `SdoCall::execute` runs the selected operation through
the existing `SdoDriver` and a `SharedCanBus` filtered subscription. Active
timeouts belong to that runner, because only it can safely cancel a live
protocol transfer. SDO aborts retain the exact remote 32-bit value, including
manufacturer-specific codes.
