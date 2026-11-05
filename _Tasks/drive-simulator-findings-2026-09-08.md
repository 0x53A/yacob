# Arm simulator adversarial review findings (2026-09-08)

These are confirmed against the arm simulator's dependency pin
`f3da0281caa9d1594d0304720bd3afca6f0cb507`. This note does not change upstream
source or claim that the arm's local workarounds fix other CANopen consumers.
The same relevant dispatch/check ordering remains in the local working tree
on 2026-09-08, alongside unrelated in-progress fixes.

## DSIM-02: late RX can conceal an elapsed heartbeat timeout

`canopen-core/src/node.rs::Node::process` drains incoming frames before its
heartbeat timeout checks. `heartbeat.rs::HeartbeatMonitor::process` replaces
`last_seen_us` when receiving a valid state byte without first checking the
previous lease. Therefore a heartbeat already queued when a delayed poll runs
can erase evidence of a timeout.

Concrete reproduction with a 200 ms consumer for master node 1:

1. Deliver CAN `701#05` to the node at time 0 microseconds.
2. Do not poll between arrivals.
3. Queue another `701#05`, call `Node::process` at 200001 microseconds.
4. Actual: monitor remains Alive; no Timeout event is emitted.
5. Expected for a lease enforced on local processing time: previous lease
   expires, emitting Timeout, then the new heartbeat may emit Recovered.

The arm-level regression `late_heartbeat_cannot_hide_an_elapsed_consumer_timeout`
failed with the drive still OperationEnabled. The application now services an
empty-RX protocol tick at the supplied time before enqueueing the next frame,
so expiration latches a drive fault before reception can recover the monitor.

This is a polling-order limitation, not a claim that software knows a frame's
physical receive time: CanFrame carries no hardware acquisition timestamp.
An upstream change should explicitly specify whether heartbeat leases use
processing or transport acquisition time and test delayed polling/queued RX.
Frequent normal polling makes this less likely, but cannot prove it absent.

## DSIM-03: overlong NMT and heartbeat frames are accepted

`canopen-core/src/node.rs::Node::dispatch_frame` checks `raw_dlc() >= 2` for NMT
and `raw_dlc() >= 1` before heartbeat dispatch. Trailing bytes are silently
ignored. These services have fixed payloads (two bytes and one byte respectively).

Confirmed arm-level reproductions:

- Enable node 33, then send `000#022100` (three-byte NMT Stop). Actual: node
  stops and the drive disables. Expected: malformed NMT frame is ignored.
- Configure node 1 heartbeat consumer at 200 ms, send `701#05` at time 0, then
  malformed `701#0500` at 190000 us, then poll at 200001 us. Actual: no timeout,
  because the malformed packet refreshed the heartbeat lease. Expected: the
  valid time-0 heartbeat expires and the drive faults.

The arm regressions `malformed_nmt_and_rpdo_do_not_change_command_state` and
`malformed_heartbeat_cannot_extend_consumer_lease` failed before local ingress
length checks were added. Short and overlong RPDOs were already rejected by
this pin's exact mapped-payload-length validation; no new RPDO length bug was
found. The arm's check follows the configured heartbeat consumer ID, so it
also covers remapped consumer-node configurations.

Upstream follow-up: validate exact DLC before changing NMT state or refreshing
a heartbeat monitor, with raw-frame regressions for both undersized and
oversized packets. Keep these findings open until fixed in canopen_rs and
adopted by dependent applications; the simulator currently retains its pin.
