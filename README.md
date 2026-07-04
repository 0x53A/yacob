This is **Y**et **A**nother **C**an**O**pen li**B**rary for Rust. It was developed with the goal of providing _full_ functionality, including SDO server and client for both embedded devices (no_std) and desktop platforms.

Implementation was heavily LLM-assisted, but verified on real hardware, using both embassy on STM32 and ESP32 and Linux with a USB-CAN adapter.

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
        [0x6000] inputs: record {
            [1] button: u8 = 0, ro, pdo;      // PB7 (0=released, 1=pressed)
            [2] echo_in: u16 = 0, rw, pdo;    // written by remote, echoed to echo_out
        };
        [0x6200] outputs: record {
            [1] led: u8 = 0, rw, pdo;         // PB8 (0=off, 1=on)
            [2] echo_out: u16 = 0, ro, pdo;   // mirrors echo_in
        };

        // TPDO1: data this node sends (0x181 for node 1)
        // - transmission_type: EVENT_DRIVEN (255) = send on change,
        //   not tied to SYNC. Other options: SYNC_ACYCLIC (0), sync_cyclic(N).
        // - inhibit_time: minimum 50ms between sends (in 100μs units)
        // - event_timer: periodic fallback — send at least every 1000ms even
        //   if nothing changed. Set to 0 to only send on explicit triggers.
        // - Fields are packed into one CAN frame: [button (1 byte) | echo_out (2 bytes)]
        tpdo[1](transmission_type = 255, inhibit_time = 500, event_timer = 1000) {
            button,
            echo_out,
        };

        // RPDO1: data this node receives (0x201 for node 1)
        // - transmission_type: EVENT_DRIVEN (255) = apply values to OD immediately
        //   when the frame arrives. With SYNC_ACYCLIC (0), values would be buffered
        //   and only applied on the next SYNC pulse (useful for coordinated updates).
        // - Fields are unpacked from the CAN frame: [led (1 byte) | echo_in (2 bytes)]
        // - Writing to these triggers an OdEvent, which wakes main via EVENT_SIGNAL.
        rpdo[1](transmission_type = 255) {
            led,
            echo_in,
        };
    }
}
```
