This is **Y**et **A**nother **C**an**O**pen li**B**rary for Rust. It was developed with the goal of providing _full_ functionality, including SDO server and client, for both embedded devices (no_std) and desktop platforms.

Implementation was heavily LLM-assisted, but verified on real hardware, using both embassy on STM32 and ESP32, and Linux with a USB-CAN adapter.

The OD (object dictionary) can either be imported from an EDS file, or specified in-code through a dsl:

```rs
object_dictionary! {
    pub struct NodeOd {
        [0x1000] device_type: u32 = 0x0000_0191, ro;
        [0x1001] error_register: u8 = 0x00, ro;
        [0x1018] identity: record {
            [1] vendor_id: u32 = 0x0000_CAFE, ro;
            [2] product_code: u32 = 0x0001, ro;
            [3] revision: u32 = 0x0001_0000, ro;
            [4] serial_number: u32 = 0x0000_0001, ro;
        };
        // CiA 401-style process I/O. Perspective is the physical process,
        // not the bus: an "input" is read from the world and published on
        // the bus (TPDO); an "output" is commanded from the bus (RPDO)
        // and driven into the world.
        [0x6000] inputs: record {
            [1] button: u8 = 0, ro, pdo;      // PB7 (0=released, 1=pressed)
        };
        [0x6200] outputs: record {
            [1] led: u8 = 0, rw, pdo;         // PB8 (0=off, 1=on)
        };

        // Bus-loopback test object. It has no physical-world meaning, so it
        // lives in the manufacturer-specific area (0x2000..=0x5FFF) instead
        // of the device-profile area. Names are from the device's view:
        // echo_in arrives from the bus, echo_out is sent back.
        [0x2000] echo: record {
            [1] echo_in: u16 = 0, rw, pdo;    // written by remote
            [2] echo_out: u16 = 0, ro, pdo;   // node mirrors echo_in here
        };

        // TPDO1: data this node sends (0x181 for node 1).
        // - event_driven: send on change, not tied to SYNC. Other options:
        //   sync_acyclic, sync_cyclic(N), or a raw CiA 301 value (e.g. 255).
        // - inhibit_time: minimum spacing between sends. event_timer: periodic
        //   fallback — send even if nothing changed (omit to disable). Both
        //   take unit suffixes (50ms, 0.1s, 500us) or raw CiA 301 values.
        // - Fields are packed into one CAN frame: [button (1 byte) | echo_out (2 bytes)]
        tpdo[1](transmission_type = event_driven, inhibit_time = 50ms, event_timer = 1s) {
            button,
            echo_out,
        };

        // RPDO1: data this node receives (0x201 for node 1).
        // - event_driven: apply values to the OD immediately on arrival. With
        //   sync_acyclic, values would be buffered until the next SYNC pulse
        //   (useful for coordinated updates).
        // - Fields are unpacked from the CAN frame: [led (1 byte) | echo_in (2 bytes)]
        // - Writing to these emits a typed NodeOdChange, which wakes main via
        //   EVENT_SIGNAL.
        rpdo[1](transmission_type = event_driven) {
            led,
            echo_in,
        };
    }
}
```


PDOs and SDOs mostly work, but the applicaton model(s) aren't exactly specified yet (sync, async, ...), and canopen SYNC functionality isn't implemented yet.

Nevertheless, I am already using it for a few private projects, both implementating CANopen nodes, or interacting with existing hardware like DS402 motor drivers.

## PDO mapping details

CANopen PDO mapping entries encode three things:

```text
index << 16 | subindex << 8 | bit_length
```

There is no source bit-offset in a standard mapping entry. Partial mappings
therefore always start at bit 0 of the mapped OD subobject. For example, an
`i32` can be mapped as its low 12 bits, but bits 6 through 12 cannot be mapped
directly. Expose a separate shifted/masked OD object when that layout is
needed.

yacob does not insert implicit padding between mapped fields. Mappings are
packed back-to-back at bit granularity, so `{ bool, i32 }` is a 33-bit PDO
payload with the `i32` starting at bit offset 1. If byte alignment becomes
useful for compatibility, add it explicitly at the OD level for now (for
example by packing flags into a `u8`). Future DSL conveniences could expose
CANopen dummy mappings as `pad<N>` or `align<N>`, but that is not implemented
today.

The OD data type and the PDO mapping length are separate. A `bool` OD entry is
CANopen `BOOLEAN` (`DataType=0x0001`), but the PDO mapping length can be 1 bit
or 8 bits. Hand-written DSL mappings use 1 bit for `bool`. Imported EDS files
preserve the declared mapping length, so a `BOOLEAN` mapped as 8 bits remains
an 8-bit wire field.

Classic CANopen PDOs carry at most 64 payload bits. yacob defaults PDO mapping
storage to 64 entries, which is the worst case for bit-granular mapping
(64 one-bit objects). The generated OD rejects static or dynamic mapping
configurations whose total mapped length exceeds 64 bits.

PDO mappings are immutable by default. Immutable mapping records are exported
to EDS with only the active mapping entries, because there is no remapping
capacity to advertise. Opt into dynamic remapping with `mapping = mutable`;
mutable mapping records export all 64 writable mapping-entry subindices and
the mapping count (`sub0`) includes `LowLimit=0` and `HighLimit=64`.

For mutable PDOs, the runtime source of truth is the OD mapping record
(`0x1600..0x17FF` for RPDO, `0x1A00..0x1BFF` for TPDO):

- sub 0 is the active mapping count
- sub 1..N are mapping entries
- remapping follows the standard unlock sequence: write sub 0 to 0, write the
  mapping entries, then write the new active count

Generic EDS-driven tools normally need the target mapping-entry subindices to
exist in the EDS/OD model before they can write them. `sub0 DefaultValue` alone
describes the current/default active mapping count, not the maximum remapping
capacity.

This is intentionally correctness-first. If a very small MCU target needs to
save RAM, the planned escape hatch is an OD-level capacity attribute, e.g.
`#[pdo_max_mappings = 8]`, with the same 64-bit payload validation and with
generated EDS/mapping subindices reduced to that capacity. Until such memory
pressure is real, the public default stays compatible with the full classic
CANopen bit-mapping range.
