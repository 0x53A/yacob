# Hardware-in-the-Loop Tests

These tests verify the full CANopen stack end-to-end: a Rust SDO client on Linux talks to the STM32 CANopen node firmware over a real CAN bus via an SLCAN adapter.

## Hardware Setup

```
Linux PC                              STM32 (Nucleo-G431KB)
┌──────────────┐                      ┌──────────────┐
│ canopen-rs   │                      │ CANopen node │
│ HIL tests    │                      │ (Embassy)    │
│              │                      │              │
│  /dev/ttyACM1│    CAN bus (500kbps) │ PA11 (RX)    │
│  (SLCAN)     │◄────────────────────►│ PA12 (TX)    │
└──────┬───────┘                      └──────┬───────┘
       │ USB                                 │ USB (ST-Link)
       │                                     │
  ESP32 SLCAN                          /dev/ttyACM0
  adapter
```

### Required hardware

- **STM32 Nucleo-G431KB** (or similar G4 with FDCAN)
- **CAN transceiver** (e.g., SN65HVD230, MCP2551) connected to PA11/PA12
- **ESP32 with SLCAN firmware** ("doggy" or similar) as USB-CAN adapter
- Both devices on the same CAN bus with proper termination (120Ω)

### Wiring

| STM32 (Nucleo-G431KB) | CAN Transceiver |
|------------------------|-----------------|
| PA11                   | RXD             |
| PA12                   | TXD             |
| 3.3V                   | VCC             |
| GND                    | GND             |

| CAN Transceiver | CAN Bus |
|------------------|---------|
| CANH             | CANH    |
| CANL             | CANL    |

The ESP32 SLCAN adapter connects to the same CANH/CANL bus.

## Software Setup

### Prerequisites

- Rust with `thumbv7em-none-eabihf` target: `rustup target add thumbv7em-none-eabihf`
- `probe-rs` for flashing: `cargo install probe-rs-tools`
- `nix-shell` or `direnv` (the project has a `shell.nix` with `udev` and `can-utils`)

### 1. Build and flash the STM32 firmware

```sh
cd examples/stm32-node
cargo build --release
probe-rs download --probe 0483:374e --chip STM32G431KBTx \
  target/thumbv7em-none-eabihf/release/stm32-canopen-node
probe-rs reset --probe 0483:374e --chip STM32G431KBTx
```

If you have multiple probes, find yours with `probe-rs list` and use the
`--probe VID:PID` or `--probe VID:PID:SERIAL` flag.

### 2. Run the HIL tests

```sh
SLCAN_PORT=/dev/ttyACM1 cargo test -p hil-tests -- --test-threads=1 --ignored
```

The SLCAN adapter is auto-initialized on first open (DTR reset + S6/O).
Subsequent runs reuse the existing session and are fast (~1s).

If `SLCAN_PORT` is not set, it defaults to `/dev/ttyACM1`.

### 3. Verify CAN traffic (optional)

```sh
cargo run -p canopen-linux --example slcan_listen -- /dev/ttyACM1
```

You should see heartbeat frames: `ID=0x701 DLC=1 data=[05]` every 500ms.

## Test descriptions

| Test | What it does |
|------|-------------|
| `t01_heartbeat` | Waits for a heartbeat frame from the node |
| `t02_nmt_start` | Sends NMT Start, verifies node enters Operational |
| `t03_sdo_read_device_type` | SDO upload of 0x1000:0, expects 0x00000191 |
| `t04_sdo_read_identity` | SDO upload of vendor ID (0xCAFE) and product code |
| `t05_sdo_write_readback` | SDO download then upload of output values |
| `t06_sdo_read_only_reject` | Verifies write to read-only object is rejected |
| `t07_sdo_not_found` | Verifies read of non-existent object is rejected |
| `t08_nmt_stop_preop` | Tests NMT state transitions: Op → Stop → PreOp |

## Troubleshooting

### "No heartbeat received"

- Check that the STM32 firmware is flashed and running (`probe-rs reset`)
- Check CAN bus wiring and termination
- Verify the SLCAN adapter works: `cargo run -p canopen-linux --example slcan_listen`

### "Timeout" on SDO tests

- The STM32 may need re-flashing after dependency upgrades (`cargo build --release` + `probe-rs download`)
- Ensure tests run single-threaded (`--test-threads=1`)

### SLCAN adapter not responding

- First open after USB plug takes ~3s (device reset + boot + init)
- If stuck, unplug and replug the ESP32 adapter
- Check the serial port: `ls /dev/ttyACM*`
- The ESP32 USB JTAG port is typically the higher-numbered ttyACM device

### Finding the right serial ports

```sh
# List USB serial devices
ls /dev/ttyACM*

# Identify which is which
udevadm info -q property /dev/ttyACM0 | grep ID_MODEL
udevadm info -q property /dev/ttyACM1 | grep ID_MODEL
```

The ST-Link shows as `STLINK-V3`, the ESP32 as `USB_JTAG_serial_debug_unit`.
