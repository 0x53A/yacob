# Findings from the Rust arm drive simulator (2026-09-07)

## DSIM-01: mutable PDO mapping activation accepts nonexistent OD objects

**Status: open in canopen_rs.** Confirmed against pinned revision
`f3da0281caa9d1594d0304720bd3afca6f0cb507`, also the local checkout HEAD at the
time of this investigation. The working tree has unrelated review fixes;
those are not reverted or represented as this finding's fix.

Discovered while implementing `../frost/frost-arm-drive-sim`, using the real
CANopen `Node`, generated object dictionary and SDO server. The simulator test
`actual_pdo_remapping_changes_wire_layout_and_rejects_invalid_mapping` failed
before adding an application validation hook: the final SDO response was
successful (`0x60`) instead of an abort (`0x80`).

### Reproduction

Use node 33 in Pre-operational with a mutable TPDO2 dictionary and no OD entry
at `0xffff:00`. Send these expedited SDO downloads on CAN ID `0x621`, receiving
responses on `0x5a1`:

| Step | Object | Value | Request bytes |
|---|---|---|---|
| Disable TPDO2 | `1801:01` | `800002a1` (u32) | `23 01 18 01 a1 02 00 80` |
| Clear mapping count | `1a01:00` | `00` (u8) | `2f 01 1a 00 00 00 00 00` |
| Stage nonexistent object | `1a01:01` | `ffff0020` (u32) | `23 01 1a 01 20 00 ff ff` |
| Activate one mapping | `1a01:00` | `01` (u8) | `2f 01 1a 00 01 00 00 00` |

**Actual final response:** successful SDO download, command byte `0x60`.
**Expected:** reject the unusable mapping no later than activation, using an
appropriate mapping abort (e.g. `0x06040041`, object cannot be PDO-mapped),
leaving the mapping count uncommitted. The exact acceptance stage/error code
should be included in the eventual conformance tests.

### Source and related validation gap

In `canopen-derive/src/codegen.rs`, mutable TPDO mapping-count writes around
lines 830–850 and RPDO mapping-count writes around lines 972–992 validate
count capacity and sum the descriptors' low-byte bit lengths. They then commit
the new count. Individual descriptor writes require count zero and bound the
bit length, but do not establish that the mapped index/subindex exists or is
eligible for that direction. `canopen-core/src/sdo/server.rs::validate_download`
does not supply these missing object-mapping checks.

Source inspection shows the same path also lacks validation of PDO-mappability,
readability for TPDO, writability for RPDO, and compatibility between mapped
width and the referenced OD type. The nonexistent-object case above is the
concrete observed regression; test the related cases explicitly when fixing
the general stack behavior. Define valid dummy/partial mappings deliberately
rather than assuming every application permits only full scalar mappings.

### Impact

A configuration client can receive successful write/readback confirmation for
a mapping that cannot subsequently produce/consume useful PDO data. This can
hide configuration bugs in downstream drive simulation and defer failure until
runtime PDO processing. See the related existing known issue about invalid
TPDO construction being retried indefinitely.

### Downstream workaround, not an upstream fix

`../frost/frost-arm-drive-sim/src/od.rs::DriveOd::validate` adds validation for
its supported full-width scalar mapping subset: existence, PDO eligibility,
direction/access, width and disable-before-remap. The simulator's tests pass
with that hook, including valid gateway configuration and changed wire layouts.

The workaround **does not fix canopen_rs** or protect other dictionaries. Keep
this issue open until generated/runtime validation is fixed with stack-level
regressions and the downstream canopen dependency pin is updated. At that
point, review which application-specific restrictions should remain in the
simulator rather than deleting its hook indiscriminately.

### Follow-up checks

- TPDO and RPDO nonexistent index and nonexistent subindex.
- Non-PDO-mappable objects and wrong-direction access.
- Invalid widths, zero/oversized mappings, and supported dummy entries.
- Failed activation leaves the prior disabled state/count intact.
- Valid remapping changes the actual wire format.
- Exercise both direct OD writes and real SDO requests through `Node`.

## Downstream confirmation of existing pump lifecycle findings

The arm gateway review also encountered the consequences of existing
[R10 and R11](review-2026-09-07.md): unbounded pump work and queues without a
closed state. These are existing library findings, not additional PDO defects.

The gateway serializes controller state and CAN enqueue operations to prevent
older motion transactions from interleaving with stops. If its bounded TxQueue
is full after the pump exits, `send_async` can wait indefinitely while holding
that controller mutex. A pump-exit handler that then acquires the same mutex to
mark CAN unhealthy cannot finish. The gateway now has a local failure signal
that cancels queue waits, a 100 ms enqueue deadline with fatal handling of
incomplete transactions, and regressions for pump death/full queue and partial
enqueue timeout. This does not add close/error propagation to the upstream queue
API. Some frames can already have been transmitted before a failure is detected;
the local mitigation does not make physical CAN transactions atomic.

Source inspection also confirms that `run_pump` checks its stop flag only around
the outer loop, draining RX and then TX without per-turn limits. Continuous RX
can therefore prevent TX work and shutdown progress; setting the stop flag alone
is not a bounded shutdown guarantee. Track stack-level fixes and downstream
dependency updates under R10/R11 rather than treating the successful normal-load
ROS/Gazebo tests as coverage of these failure modes.

The gateway also wraps each nonblocking transport call with a stop check, so a
fatal shutdown interrupts the pinned pump's inner drain loops. Continuous-RX and
mid-TX stop regressions exercise this local wrapper. A separate real private-vcan
link-down test after motion observed process exit in 8.17 ms; that one runtime
measurement is not a worst-case bound or a physical drive stop-time measurement.
