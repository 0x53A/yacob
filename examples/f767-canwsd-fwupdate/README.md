# F767 canwsd firmware-update PoC

This is an isolated three-crate project for a NUCLEO-F767ZI. It proves the
intended DDU update architecture without changing the DDU firmware:

```text
canopen-fwupdate -> WebSocket/canwsd -> virtual CAN -> CANopen SDO -> flash
```

WebSocket is only the CAN transport. All discovery, target selection, transfer,
status, validation, and reset operations are CANopen object-dictionary accesses,
so the same `canopen-fwupdate` executable works over SocketCAN (`--iface`) and
Ethernet (`--canwsd`). The embedded server accepts one connection at a time.

## Flash map

The project deliberately uses the F767's default **single-bank** geometry
(`nDBANK=1`), as found on the test Nucleo. No option-byte change is required.

| Region | Address | Size | Erase sectors |
|---|---:|---:|---|
| Immutable bootloader | `0x0800_0000` | 96 KiB | 3 x 32 KiB |
| Config/metadata journal | `0x0801_8000` | 32 KiB | 1 x 32 KiB |
| Firmware | `0x0802_0000` | 384 KiB | 128 KiB + 256 KiB |
| Web image | `0x0808_0000` | 1536 KiB | 6 x 256 KiB |

The current release bootloader occupies about 91 KiB in flash. The metadata
sector contains 512 append-only 64-byte records and is never erased at runtime.
That gives hundreds of rare calibration/update state transitions without
dedicating a 256 KiB sector to configuration. A production design should define
a service procedure for journal exhaustion; this PoC fails closed when full.

Firmware and web images have independent length, CRC, and validity fields. A
web update therefore cannot invalidate the firmware, and a firmware update does
not touch the web partition.

## Object dictionary

The updater is node 42 and exposes the same CiA 302-style objects as the existing
EMDM PoC:

- `0x1F50:1` image data domain
- `0x1F51:1` control: validate `1`, reset `2`, clear `3`
- `0x1F56:1` expected CRC-32/ISO-HDLC
- `0x1F57:1` status: idle `0`, programming `1`, valid `2`, error `0xFF`
- `0x2000:1..4` declared length, received length, base, capacity
- `0x2000:5` target: firmware `1`, web `2`

The demo application exposes `0x2F00:0`; writing `0x54504455` records a one-shot
request and resets into the bootloader.

## Build and provision

```sh
cd examples/f767-canwsd-fwupdate
cargo build --release
probe-rs download --chip STM32F767ZITx \
  target/thumbv7em-none-eabi/release/f767-canwsd-fwupdate-bootloader
```

If the metadata sector is not erased (for example, when repurposing a board),
erase the chip once before that first download:

```sh
probe-rs erase --chip STM32F767ZITx --connect-under-reset
```

That command destroys all existing flash contents. Routine bootloader updates
and all in-band image updates do not erase the metadata sector.

The board uses DHCP and advertises the DHCP hostname `f767-updater` while in the
bootloader and `f767-demo` in the application. If the LAN does not resolve DHCP
hostnames, obtain its address from the DHCP lease table. The canwsd endpoint is:

```text
ws://BOARD_IP:8080/api/networks/update
```

Create the raw relocatable application image with an ARM-capable objcopy, then
use the repository's one uploader:

```sh
llvm-objcopy -O binary \
  target/thumbv7em-none-eabi/release/f767-canwsd-fwupdate-app app.bin

cargo run -p canopen-fwupdate -- \
  --canwsd ws://BOARD_IP:8080/api/networks/update \
  --node 42 flash app.bin

cargo run -p canopen-fwupdate -- \
  --canwsd ws://BOARD_IP:8080/api/networks/update \
  --node 42 flash --target web web-image.bin
```

`GET http://BOARD_IP:8080/api/networks` returns the usual canwsd network list.

## Recovery and security boundary

Before erasing a target, the bootloader appends an invalid record. It marks the
target valid only after reading it back from flash, checking its CRC, and (for
firmware) validating its vector table. Power loss during erase or programming
therefore returns to the immutable updater. There is no A/B rollback: recovery
means uploading again through the bootloader.

CRC detects corruption but does **not** authenticate an image or a client. This
PoC must not be exposed to an untrusted network; signed manifests and an update
authorization policy are production follow-ups.
