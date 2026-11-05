# Fix pass 2026-11-05

Fixes for the "obvious" findings of the 2026-11-04 review passes
(`known-issues.md` K/R numbers). Items that need design work were left open
and are listed in `known-issues.md`.

## Fixed

| Item | Fix | Tests |
|---|---|---|
| K01 (part) | 0x1001 mirrors the EMCY error register via the generated `store_error_register`, with change notification. 0x1003/0x1014/0x1015 remain open. | `od_error_register_follows_emcy_state` |
| K03 | 0x1022 Store Format = `od::STORE_FORMAT_DEFLATE` (0x80). | `macro_test` |
| K04, A09 (part) | `SdoDriver`: one upload/download implementation with an optional deadline; best-effort abort on timeout (0x0504_0000) and protocol error (0x0504_0001); typed helpers for every numeric type and bool. | `sdo_driver_test` |
| K07 | RTR transmission types 252/253 rejected by the DSL, EDS import (`OdDefinition::validate`) and generated SDO writes; reserved 241–251 also rejected on SDO writes (abort 0x0609_0030). | `rtr_transmission_types_are_rejected`, `pdo_transmission_type_rejects_reserved_and_rtr_values` |
| K08 | `EmcyProducer::dropped()` / `Node::emcy_dropped()`. | `overflow_evicts_oldest_and_counts` |
| K10 (part) | Block-upload end confirmation only closes a block upload. `CobId::tpdo`/`rpdo` take a `PdoNumber` and return `None` above 4. The "CRC flag ignored" point was withdrawn: the server's cc bit only states support, CRC is used when both sides set it. | cobid unit tests (end-confirmation guard untested) |
| K12 (part) | Records get a generated sub0 (read + metadata, read-only). | `record_sub0_reports_highest_subindex` |
| K13 | CiA 301 abort codes: subindex vs object not found, length too high/low (also for segmented/block size disagreeing with the declared size), hardware error, `DataTypeMismatch` = 0x0607_0010, missing standard codes added. | `sdo_server_abort_test` |
| K14 (part) | EDS SDO client: signed and 64-bit types, bool, f64. | build of existing `sdo_client_from_eds!` tests; driver helpers in `sdo_driver_test` |
| K15 (part) | EDS import: invalid DefaultValue, unknown AccessType are compile errors; `const` preserved. | `eds_parser` unit tests |
| K16 (part) | `SdoPort::drain_overflow()` takes the frames; `overflow_dropped()` counter. | `sdo_port_counts_frames_beyond_overflow_capacity` |
| K17 | `WebSocketTransport`: `current_thread` runtime, bounded RX queue (`receive_queue_capacity`, default 4096) with `rx_dropped()`, `rx_undecodable()` counter. | build only |
| K18 | `pdo_client_from_eds!` sign-extends partial signed mappings. | `pdo_client_test` |
| K20 | Heapless string/byte writes check capacity before clearing. | `rejected_variable_length_write_keeps_previous_value` |
| K21 | EDS export evaluates default literals and constant integer expressions. | `macro_test`, regenerated `vcan_node.eds` |
| K22 (part) | EDS export: DeviceInfo keys, CiA 306 Mandatory/Optional split, 0x1021/0x1022 sections, per-element array names. | `macro_test`, interop suite |
| K23 | `NodeConfig::from_od` takes the LSS identity from 0x1018 (bound now includes `ObjectDictionary`). | `node_config_takes_lss_identity_from_1018` |
| K24 | Longer RPDOs are processed using the leading bytes; shorter ones are counted (`RpdoEngine::length_errors`). | `long_rpdo_is_processed_using_the_leading_bytes`, `short_rpdo_changes_nothing_and_does_not_refresh_deadline` |
| K25 | LSS selective switch is one ordered sequence. | `selection_must_be_one_ordered_sequence` |
| K26 (part) | `sdo_upload` allocates `SdoDriver::MAX_TRANSFER` instead of 889 B. | — |
| K27 | RPDO partial-width mappings widen to the object size and sign-extend; TPDO reads the full object. | `pdo_mapping_test` |
| K28 | EDS macros emit `include_bytes!` of the input file. | — |
| K29 (part) | `OdDefinition::validate`: duplicate indices, generated-index shadowing, record `[0]`/duplicate subs, mapping access direction. | `dsl::validation_tests` |
| K31 | `#[export_eds]` writes only on change and is a compile error on failure. | — |
| R05 (part) | Boot-up is retried on `WouldBlock` instead of being lost. | `boot_waits_for_transmit_acceptance` |

## Breaking API changes

- `SdoPort::drain_overflow()` returns `heapless::Vec<CanFrame, 16>` (owned).
- `NodeConfig::from_od` requires `OD: ObjectDictionary`.
- `AbortCode::DataTypeMismatch` is 0x0607_0010.
- `CobId::tpdo`/`rpdo(PdoNumber, NodeId) -> Option<CobId>`.
- `WebSocketTransportConfig` has a new `receive_queue_capacity` field.
- DSL/EDS: RTR transmission types and the conflicting declarations above are
  compile errors.

## Verification

`cargo test --workspace --exclude hil-tests`, `cargo check -p canopen-core`
with `--no-default-features` and `--features alloc,embassy,defmt`,
`cargo check` of the four excluded firmware workspaces, and the
python-canopen interop suite (`uv run pytest`) all pass.
