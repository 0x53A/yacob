# Additional SDO servers (0x1201–0x127F)

Status: **implemented** 2026-07-21 (design agreed same day).

Landed: DSL `sdo_server[N]` parsing, codegen (0x1201+ records, `SDO_COUNT`,
`SdoServerConfigSource`, EDS export), `Node`/`NodeConfig` `const SDO` generic
with independent per-channel transfer state, COB-ID dispatch, `Node::new`
collision debug-assert, unit tests (macro_test.rs) + interop tests
(TestAdditionalSdoServer). `vcan_node` example carries a diagnostics channel.
**Still pending**: EDS *import* of 0x1201+ (`object_dictionary_from_eds!` fills
`sdo_servers: []`).

## Goal

Support more than one SDO server per node. Typical use: a firmware defines two
channels — one for normal operations, one for a diagnostics tool — each with its
own COB-ID pair, both serving the same OD, with **independent** transfer state so
a diagnostics transfer never disturbs an in-flight normal-ops transfer.

## Design decisions

- **Const / non-reconfigurable.** SDO server COB-IDs are a device design
  decision baked into the firmware, not something a master renegotiates at
  runtime. All SDO server parameter entries export as `ro`/`const`; COB-IDs are
  fixed at `Node::new`. Mirrors the "PDO mappings immutable by default"
  precedent. No NMT-state gating, no write→dispatcher resync, no runtime
  COB-ID-collision validation.
  - The **only** legitimate reason the spec makes 0x1200+ COB-IDs writable is
    the CiA 302 **SDO Manager** (dynamic SDO connection allocation on managed
    networks) — exotic, out of scope, and would be its own large feature, not a
    reuse of a `reconfigurable` flag. So we do not build a speculative opt-in.
- **Default server (0x1200) stays implicit.** It always exists at slot 0 with
  predefined COB-IDs (`0x600/0x580 + node_id`); no OD entry is generated for it.
  Only `0x1201+` extras get generated records.
- **No client-node-id restriction (sub 3)** for now — unnecessary for a fixed
  firmware talking to a known tool.
- **Uniform ~900-byte buffers** per server (block-transfer sized). ~1 KB RAM per
  extra server, accepted. Per-server buffer sizing can be parametrized later.
- **Codegen-time uniqueness check.** Because COB-IDs are compile-time known,
  reject at build time if any SDO-server rx/tx COB-ID collides with another
  SDO-server or (ideally) a PDO COB-ID, instead of silently shadowing a channel.

## DSL

```rust
object_dictionary! {
    pub struct MyOd {
        // default server 0x1200 implicit, predefined COB-IDs
        sdo_server[2](cob_rx = 0x640, cob_tx = 0x5C0);        // fixed
        // node-relative also supported, resolved against the real node id:
        // sdo_server[2](cob_rx = node_id + 0x40, cob_tx = node_id + 0x40);
    }
}
```

Server number `n` ↔ OD record `0x1200 + (n - 1)`. Number 1 is the implicit
default; extras are 2..=128.

## Implementation sketch

1. **DSL** (`canopen-derive/src/dsl.rs`): parse `sdo_server[N](cob_rx=, cob_tx=)`,
   fixed or node-relative expressions. Direction/uniqueness checks.
2. **Codegen** (`canopen-derive/src/codegen.rs`): emit `0x1201+` OD records
   (sub0 highest-sub, sub1 cob_rx, sub2 cob_tx; `ro`/`const`), an `SDO_COUNT`
   const, `SdoServerNode` alias, and a config source for `NodeConfig::from_od`.
   EDS export of the records.
3. **Node** (`canopen-core/src/node.rs`): add `const SDO: usize = 1`; store
   `[SdoServer; SDO]` (slot 0 = default) plus a small resolved rx/tx COB-ID
   table. Resolve node-relative COB-IDs once at `Node::new`.
4. **Dispatch**: match the raw frame id against each server's rx COB-ID before
   the `ParsedCobId` match; run that server's state machine; respond on its tx
   COB-ID. `check_timeout` / `poll_block_upload` iterate all servers.
5. **Tests**: unit (two servers, independent transfers, no cross-talk) + interop
   (python-canopen talking to the second channel over UDP multicast).

## Future compliance document

`conformance-matrix.md` is the seed of the "mandatory vs optional CiA feature"
compliance document the project wants once the stack has grown. When this lands,
update rows `0x1200` / `0x1201+` and SDO "Multiple SDO channels", and note the
SDO-Manager remap path as intentionally unsupported.
