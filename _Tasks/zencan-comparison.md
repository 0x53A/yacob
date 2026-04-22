# canopen-rs vs Zencan: Architecture Comparison

## Overall Philosophy

| | **canopen-rs (ours)** | **Zencan** |
|---|---|---|
| OD definition | Proc macro DSL / EDS parsing at compile time | TOML config + build.rs code generation |
| Static allocation | `heapless` collections, const generics for sizes | `AtomicCell`, `critical_section`, static references |
| Transport abstraction | `embedded_can::nb::Can` trait directly | `NodeMbox` — shared mailbox with atomic cells |
| Threading model | `MailboxTransport` is `Send+Sync` via `critical_section` | `Send + Sync` everywhere, lock-free atomics |
| Concurrency | Embassy signal for async wakeup (optional) | Callbacks for process/transmit notify |
| Extra protocols | EMCY, LSS slave | LSS slave+master, persistent storage, bootloader |

## NMT State Machine

**Ours** is cleaner and more explicit:
- Separate `NmtHandler` struct with `process_command()` returning `NmtTransition` enum
- Distinguishes `ResetApplication` vs `ResetCommunication` transitions
- State transitions are all in one place (`NmtHandler`), the `Node` just calls it

**Zencan** distributes NMT logic across `Node`:
- State transitions are individual methods on `Node` with callback hooks
- Each transition fires a callback — more extensible but harder to reason about

## SDO Server

**Ours**: Expedited + segmented + **block transfers** (upload + download) with CRC-16/CCITT. 889-byte buffer (127 segments x 7 bytes). SDO timeout for stale transfers. PDO config protection in Operational state.

**Zencan**: Same feature set. Configurable buffer size.

## Things Zencan Has That We Don't

1. **Persistent storage** — save/restore object values to flash (application-level concern)
2. **Bootloader support**
3. **LSS master** (we have slave only)
4. **FastScan** LSS discovery

## Things We Have That Zencan Doesn't

1. **EDS file parsing** at compile time + **EDS export** from macro
2. **Inline OD DSL** — more concise than TOML + build.rs
3. **OdGuard RAII pattern** for automatic TPDO triggering
4. **OdEvent queue** — structured event notification to application
5. **UDP multicast transport** — cross-process testing without root
6. **SdoClient** for talking to remote nodes (Zencan only has this for Linux)
7. **python-canopen interop test suite** — 26 tests validating protocol compliance

## Resolved Items (previously actionable)

All items from the original comparison have been addressed:

1. ~~ResetNode vs ResetCommunication~~ — **Done.** `NmtTransition` enum distinguishes them.
2. ~~PDO config protection~~ — **Done.** SDO writes to 0x1400-0x1BFF rejected in Operational.
3. ~~SDO timeout~~ — **Done.** 5s timeout aborts stale segmented/block transfers.
4. ~~Send + Sync for mailbox~~ — **Done.** `MailboxTransport` uses `critical_section::Mutex`.
