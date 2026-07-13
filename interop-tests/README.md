# CANopen Interop Tests

Protocol interoperability tests using [python-canopen](https://github.com/canopen-python/canopen) as the test driver against our `vcan_node` example.

Uses UDP multicast on loopback by default — no kernel modules, no root, works everywhere.

## What's tested

- **Heartbeat**: reception, PreOp state, ~500ms interval
- **NMT**: Start, Stop, PreOperational, Reset Node, Reset Communication
- **SDO expedited**: read device type/error reg/identity, write+readback u8/u16, read-only rejection, object-not-found
- **SDO identity record**: all 4 subindices of 0x1018
- **PDO config protection**: write to 0x1800 rejected in Operational
- **PDO data exchange**: RPDO→OD via SDO readback, RPDO→mirror→TPDO echo
- **Extended PDOs >4**: comm params at 0x1804/0x1404 via SDO (incl. resolved default COB-IDs), RPDO5→OD, RPDO5→mirror→TPDO5 echo on explicit COB-IDs, python-canopen PDO map parsing from EDS

## Run

```sh
cd interop-tests
uv run pytest -v
```

## How it works

The test harness:
1. Builds and spawns `vcan_node` with `CAN_TRANSPORT=udp`
2. Both sides communicate via UDP multicast (239.74.163.2:43113) on loopback
3. python-canopen acts as SDO client / NMT master / PDO producer
4. Wire format is msgpack, compatible with python-can's `udp_multicast` interface

## Alternative: socketcan

If you have vcan available:

```sh
sudo modprobe vcan
sudo ip link add dev vcan_test0 type vcan
sudo ip link set up vcan_test0
CAN_TRANSPORT=socketcan uv run pytest -v
```
