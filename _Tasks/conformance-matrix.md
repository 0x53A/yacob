# CANopen Conformance Matrix

Status legend:

- **Yes**: implemented and covered by focused unit or interop tests.
- **Partial**: implemented for common cases, but missing spec behavior, edge cases, or integration.
- **No**: not implemented.
- **Unknown**: not audited deeply enough yet.
- **N/A**: intentionally outside the current scope.

This matrix is based on repository source/tests plus public summaries of CiA
301/CiA 305/CiA 306 feature families. The normative CiA specifications should
remain the final reference for exact behavior.

## Summary

The existing `known-issues.md` is not a complete conformance checklist. It is a
bug backlog. The crate already covers useful CANopen node functionality, but
the missing/partial areas below should be tracked explicitly if the goal is
"conforms to CANopen" rather than "works for current examples".

Big gaps not fully captured by the current known-issues list:

- CiA 301 mandatory/predefined communication objects are not tracked as a set.
- SYNC support is split: TPDO sync has logic, RPDO sync buffering is known
  missing, and SYNC producer/consumer objects are not integrated via OD 0x1005
  / 0x1006 / 0x1007.
- TIME object/service is absent.
- EMCY producer exists, but pre-defined error field 0x1003 and inhibit time
  0x1015 are absent or not integrated.
- Additional SDO channels (0x1201+) are implemented via the DSL; EDS *import*
  of them is still pending.
- Node guarding/life guarding is absent.
- LSS slave is partial; LSS master and FastScan are absent.
- CiA 302 features are mostly out of scope except application examples.
- Device profile conformance (CiA 401/402/etc.) is out of scope unless a
  profile-specific crate/module is added.

## CiA 301 Application Layer / Communication Profile

| Area | Feature / object | Status | Evidence / notes |
|---|---:|---|---|
| CAN frame model | Classic 11-bit CAN data frames | Yes | `CanFrame` rejects extended IDs and >8-byte payloads. |
| CANopen FD | CANopen FD frame sizes / CiA 1301 | No | Classic CAN only. |
| COB-ID parsing | NMT, EMCY, SDO, PDO, heartbeat, SYNC | Yes | `cobid`, `events`, tests. |
| OD core | Variable/array/record lookup | Yes | `ObjectDictionary`, macro and EDS tests. |
| OD access | `ro`, `wo`, `rw`, `const` | Partial | Implemented, but full CiA access-attribute semantics need audit. |
| OD data types | bool, integer widths, float32, strings, octet/domain | Partial | Good common coverage; not full CiA type universe. |
| OD validation | Application pre-write validation | Yes | `validate_write`, macro attribute, SDO tests. |
| OD metadata | Names, data types, max variable length, PDO mappable | Yes | `OdEntryMeta`. |
| OD extensions | Per-object read/write extension callbacks | Partial | `validate_write` only; no full CANopenNode-style OD extension IO layer. |
| Mandatory object | 0x1000 Device type | User-provided | DSL/EDS can declare it; stack does not force presence. |
| Mandatory object | 0x1001 Error register | Partial | EMCY producer tracks register, but OD synchronization is application/OD dependent. |
| Mandatory object | 0x1017 Producer heartbeat time | Yes | OD value wins over `NodeConfig` at init; SDO writes change the runtime period, 0 disables; re-synced on reset. Unit + interop tests. |
| Mandatory object | 0x1018 Identity object | User-provided | DSL/EDS can declare it; LSS identity is configured separately. |
| Error history | 0x1003 Pre-defined error field | No | EMCY does not maintain 0x1003. |
| SYNC config | 0x1005 COB-ID SYNC | No | No OD-integrated SYNC config. |
| SYNC config | 0x1006 Communication cycle period | No | `SyncProducer` exists but not integrated. |
| SYNC config | 0x1007 Synchronous window length | No | Not implemented. |
| Guarding | 0x100C Guard time | No | Node guarding/life guarding absent. |
| Guarding | 0x100D Life time factor | No | Node guarding/life guarding absent. |
| TIME config | 0x1012 COB-ID TIME | No | TIME service absent. |
| EMCY config | 0x1014 COB-ID EMCY | No | EMCY COB-ID fixed to predefined default. |
| EMCY config | 0x1015 Inhibit time EMCY | No | No EMCY inhibit time. |
| Heartbeat consumer | 0x1016 Consumer heartbeat time | Yes | OD-integrated `HeartbeatMonitor` per entry (up to 8); typed events (started/state-change/remote-reset/timeout/recovery); SDO validation (invalid node id 0x0609_0030, duplicate producer 0x0604_0043). Unit + interop tests. |
| Store params | 0x1010 Store parameters | No | No generic storage object. |
| Restore params | 0x1011 Restore default parameters | No | No generic restore/default storage. |
| SDO server params | 0x1200 default SDO server | Partial | Default channel implemented; 0x1200 record kept implicit (predefined COB-IDs, not modelled as an OD object). |
| Extra SDO server | 0x1201+ additional SDO servers | Yes | DSL `sdo_server[N](cob_rx=, cob_tx=)`; independent transfer state per channel; OD records read-only (const, non-remappable — CiA 302 SDO Manager remap intentionally out of scope). Unit + interop tests. EDS export yes; EDS import pending. See `_Tasks/additional-sdo-servers.md`. |
| SDO client params | 0x1280+ SDO client objects | No | Client exists, but no OD-managed SDO client channels. |

## NMT / Boot-up / Error Control

| Feature | Status | Evidence / notes |
|---|---|---|
| Boot-up heartbeat | Yes | `Node::process` sends boot frame; interop tests. |
| NMT start/stop/pre-op/reset command parsing | Yes | `NmtHandler`, interop tests. |
| Reset node vs reset communication distinction | Yes | `NmtTransition`, `Node::request_reset`. |
| NMT master command frame builder | Yes | `NmtCommand::to_frame`. |
| Automatic startup / auto operational | Yes | `NodeConfig::auto_start`. |
| Heartbeat producer | Yes | `HeartbeatProducer`, interop tests. |
| Heartbeat consumer | Yes | OD/Node-integrated via 0x1016 (`next_heartbeat_event`, `heartbeat_status`); monitoring starts on first heartbeat per CiA 301. No auto-EMCY (app policy; `vcan_node.rs` shows the 0x8130 pattern). |
| Node guarding / life guarding | No | Not implemented. |
| Boot-up sequencing vs complete CiA 301 state rules | Partial | Useful behavior exists; needs conformance audit against spec. |

## SDO

| Feature | Status | Evidence / notes |
|---|---|---|
| Default SDO server | Yes | `SdoServer`, node dispatch, interop tests. |
| Expedited upload/download | Yes | Unit + interop tests. |
| Segmented upload/download | Yes | Unit tests. |
| Block upload/download | Yes | Unit tests and python-canopen block roundtrip. |
| CRC for block transfers | Yes | Fixed and tested. |
| SDO server timeout | Yes | 5s timeout test. |
| SDO client expedited/segmented | Yes | `SdoClient`, `SdoDriver`, typed client tests. |
| SDO client block transfer | No | Client supports expedited/segmented only. |
| SDO client timeout | Partial | `SdoDriver` timed APIs; low-level `SdoClient` has no timeout. |
| Abort-code mapping | Partial | Much improved; audit all state/protocol paths. |
| Busy-channel / collision behavior | Partial | Known issue; focused tests still missing. |
| Multiple SDO channels | Yes | Additional SDO servers (0x1201+) via DSL `sdo_server[N]`; independent transfer state, dispatched by configured COB-ID. |
| SDO via non-CAN mappings (CiA 309/J1939 mapping) | N/A | Outside core scope. |

## PDO / SYNC

| Feature | Status | Evidence / notes |
|---|---|---|
| TPDO mapping from DSL/EDS | Yes | Macro/EDS tests. |
| RPDO mapping from DSL/EDS | Yes | Macro/EDS and interop tests. |
| Event-driven TPDO | Yes | `OdGuard` dirty tracking, tests. |
| TPDO event timer | Yes | Unit tests and examples. |
| TPDO inhibit time | Partial | Implemented for event sends; needs spec edge-case audit. |
| TPDO synchronous cyclic | Partial | `TpdoEngine::on_sync`; tests. |
| TPDO synchronous acyclic | Partial | Code notes missing trigger mechanism. |
| RPDO event-driven | Yes | Unit + interop tests. |
| RPDO synchronous buffering | No | Known issue: sync-type RPDOs applied immediately. |
| RPDO deadline/event timer monitoring | Yes | Unit + interop tests. |
| Dynamic PDO remapping in PreOperational | Partial | Mapping lock protocol exists; needs broader conformance tests. |
| PDO config write rejected in Operational | Yes | Unit + interop tests. |
| Extended PDO count >4 | Yes | Interop tests cover PDO5. |
| RTR PDO transmission types 252/253 | No | DSL recognizes values, but behavior appears absent. |
| MPDO | No | Not implemented. |
| SYNC consumer dispatch | Partial | Node handles SYNC for TPDO; consumer object not integrated. |
| SYNC producer | Partial | `SyncProducer` exists, not integrated into `Node`/OD. |
| SYNC counter overflow/value rules | Unknown | Needs audit. |
| Synchronous window length | No | Not implemented. |

## EMCY

| Feature | Status | Evidence / notes |
|---|---|---|
| EMCY frame producer | Yes | `EmcyProducer`, interop tests. |
| EMCY frame parser/consumer side | Yes | `EmcyMessage::parse`, event decoding tests. |
| Multiple pending EMCY frames | Yes | Fixed, tested. |
| Error register bit handling | Partial | Known issue: GENERIC bit latching semantics wrong. |
| Error reset frame | Partial | Blocked by GENERIC-bit issue in some recovery paths. |
| Pre-defined error field 0x1003 | No | Not maintained. |
| EMCY inhibit time 0x1015 | No | Not implemented. |
| Configurable EMCY COB-ID 0x1014 | No | Not implemented. |

## LSS / CiA 305

| Feature | Status | Evidence / notes |
|---|---|---|
| LSS slave switch mode global | Yes | Unit tests. |
| LSS slave selective switch | Yes | Unit + interop tests. |
| LSS configure node ID | Partial | State updates and event; application persistence/reset semantics need integration. |
| LSS store configuration | Partial | Emits event only. |
| LSS identity inquiries | Yes | Unit + interop tests. |
| LSS configure bit timing | No | Not implemented. |
| LSS FastScan | No | Known issue. |
| LSS master | No | Not implemented. |

## Device Description / Configuration Files

| Feature | Status | Evidence / notes |
|---|---|---|
| EDS import | Partial | `canopen-derive` parser handles useful subset, tests. |
| EDS export | Partial | Macro export tests; needs compatibility testing against more tools. |
| DCF parsing | Partial | `dcf.rs` exists; scope likely limited. |
| DCF generation | Unknown | Needs audit. |
| XDD/XDC | No | Not implemented. |
| Embedded EDS store object | Yes | Compressed EDS object tests. |
| CANopenEditor/CANopenNode XDD workflow compatibility | No | Not implemented. |

## CiA 302 / Higher-level Functions

| Feature | Status | Evidence / notes |
|---|---|---|
| Network manager / NMT master | Partial | Can send NMT frames; no full manager. |
| SDO manager / dynamic SDO management | No | Not implemented. |
| Program download objects | Application-specific | Example firmware update node, not generic CiA 302 implementation. |
| Network variables / process image | No | Not implemented. |
| Redundancy / flying master | No | Not implemented. |

## Device Profiles / Safety

| Feature | Status | Evidence / notes |
|---|---|---|
| CiA 401 generic I/O | N/A | Can model profile objects manually; no profile implementation. |
| CiA 402 drives | N/A | Can talk to drives as master via SDO/PDO; no DS402 state-machine crate. |
| Other CiA 4xx profiles | N/A | Out of core scope. |
| SRDO / CiA 304 safety | No | Not implemented. |

## Test Coverage Checklist

| Test class | Status | Current coverage |
|---|---|---|
| Unit tests | Strong | 112 `canopen-core` unit tests plus macro/EDS/typed-client tests. |
| Python interop | Good but narrow | Heartbeat (incl. 0x1017 runtime config + 0x1016 consumer/EMCY policy), NMT, SDO expedited/block, PDO, EMCY, RPDO deadline, LSS subset. |
| CANopenNode cross-tests | No | Not yet used as oracle. |
| Lely cross-tests | No | Not yet used as oracle. |
| Conformance test vectors from CiA | No | Not available in repo. |
| Hardware HIL | Partial | `hil-tests` exists; not audited in this matrix. |
| Fuzz/property tests | No | Useful for SDO/PDO frame parsers. |

## External Test Sources

### Lely / ESA ECSS test suite

The N7 Space `lely-core` `ecss` branch contains an Apache-2.0 CANopen library
unit test suite developed under an ESA-funded programme, plus Lely's existing
integration tests. It is useful, but not drop-in:

- `unit-tests/co/` is the richest source of expected behavior. These tests
  cover SDO, SDO-managed communication objects, PDO, SYNC, EMCY, NMT,
  heartbeat, object dictionary behavior, types, and CRC. They are tightly
  coupled to Lely internals (`co_dev_t`, `co_ssdo_t`, `co_rpdo_t`,
  `can_net_t`, CppUTest), so the practical path is to port test intent and
  expected abort codes/state transitions into Rust tests.
- `test/` contains smaller integration-style examples using Lely devices,
  DCF files, virtual CAN networks, TAP output, and client/server services. These
  are useful as protocol scenarios and fixtures, but still assume Lely's test
  harness rather than a generic CANopen conformance runner.
- If any source or fixture is copied verbatim, retain the Apache-2.0 license
  header and `NOTICE` attribution. Prefer ported Rust tests for most cases.

Best first targets to port from Lely are SDO abort-code edge cases, SDO object
0x1200 behavior, PDO communication/mapping object writes through SDO, SYNC
configuration behavior, heartbeat consumer behavior, EMCY 0x1003/0x1015
behavior, and LSS/FastScan once LSS work resumes.

## Suggested Next Tracking Issues

Promote these from matrix items into `known-issues.md` if the project wants a
bug-level backlog:

1. ~~Integrate heartbeat consumer with `Node` and OD 0x1016.~~ Done 2026-07.
2. ~~Integrate producer heartbeat time 0x1017 runtime writes with the heartbeat
   producer.~~ Done 2026-07.
3. Add 0x1003 pre-defined error field maintenance.
4. Add EMCY COB-ID/inhibit-time support (0x1014/0x1015).
5. Add SYNC OD integration (0x1005/0x1006/0x1007), including synchronous window.
6. Implement synchronous RPDO buffering.
7. Decide whether node guarding/life guarding is in scope; if not, document it
   as intentionally unsupported.
8. Add TIME object/service or document as unsupported.
9. ~~Add additional/configurable SDO channels.~~ Done 2026-07 (0x1201+ via DSL,
   const/non-remappable; EDS import still pending). See
   `_Tasks/additional-sdo-servers.md`.
10. Add LSS FastScan and/or explicitly mark unsupported.
11. Add SDO client block transfer if master-side large transfers matter.
12. Build a CANopenNode/Lely interop test harness for behavioral comparison.
