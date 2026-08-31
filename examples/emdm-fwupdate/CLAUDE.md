# EMDM CANopen firmware update PoC

Power-fail-safe firmware update over CANopen SDO, on the FROST EMDM board
(STM32G431CB, 128 KiB flash, 2 KiB pages, 24 MHz HSE).

Two images that share one crate:

- `bootloader/` — the immutable updater. Flashed once over SWD, never rewritten
  over CAN.
- `app/` — a deliberately trivial demo: one RPDO bit drives the green LED.
- `common/` — flash map, metadata record, board bring-up. Both images link it,
  so the layout cannot drift between them.

This is a **separate cargo workspace** (excluded from the root one) because it
cross-compiles to `thumbv7em-none-eabihf`.

## Build, flash, test

```sh
cargo build --release

# Host-side unit tests for the metadata record (no target needed)
cargo test -p emdm-fwupdate-common --target x86_64-unknown-linux-gnu

# Bootloader goes on over SWD, once.
probe-rs download --chip STM32G431CBTx --connect-under-reset \
  target/thumbv7em-none-eabihf/release/emdm-fwupdate-bootloader

# The application goes on over CAN.
arm-none-eabi-objcopy -O binary \
  target/thumbv7em-none-eabihf/release/emdm-fwupdate-app app.bin
cargo run --release -p canopen-fwupdate -- --iface can0 --node 42 flash app.bin
```

`--connect-under-reset` is used above out of habit, not necessity: plain
attaches work. See "Debug attach" below before concluding a board is bricked.

## Flash map

Single source of truth: `common/src/lib.rs`. The two `memory.x` files must
agree with it; `const _: () = assert!(...)` catches the arithmetic, not the
linker scripts, so change all three together.

| Region | Offset | Address | Size |
|---|---|---|---|
| Bootloader | `0x00000` | `0x08000000` | 56 KiB |
| Metadata log | `0x0E000` | `0x0800E000` | 2 KiB |
| Application | `0x0E800` | `0x0800E800` | 70 KiB |

`0x0800E800` is 512-byte aligned, which VTOR requires: the table has 118
entries (472 bytes), so it must sit on the next power of two up.

Measured: bootloader 48,760 B of 57,344; application 46,144 B of 71,680.

## Why the boot decision happens before `embassy_stm32::init()`

The bootloader is a plain `#[cortex_m_rt::entry]`, not
`#[embassy_executor::main]`. It reads the metadata log and CRCs the application
straight out of the memory-mapped flash window — no clock, no DMA, no
interrupt, no peripheral. Only if it decides to *stay* does it build an
executor and bring up the PLL and FDCAN.

So the jump into the application happens from a state indistinguishable from
reset: HSI, no NVIC interrupt enabled, RCC at defaults. The application's
`embassy_stm32::init()` then behaves exactly as if it had been flashed at
`0x0800_0000`.

Jumping *after* bringing up the PLL, FDCAN and the time driver is the usual way
this goes wrong — the application then has to tear down a clock tree it knows
nothing about. Do not move the decision later.

The bootloader sets VTOR before `cortex_m::asm::bootload()`, which does not.

## Metadata log

One page, 64 append-only 32-byte records. Newest valid record wins using
wrap-safe sequence ordering; a record whose own CRC fails is skipped, so a
write torn by a power loss is ignored rather than misread. The page is only
erased when no erased slot is left — the one window where metadata can be lost,
and it fails safe, because a device with no valid record stays in the updater.
See "Installing over some other firmware" for why the free slot is found by
scanning for `0xFF` rather than by counting records.

Records are 32 bytes because that is a multiple of the STM32G4 64-bit write
granularity (`embassy_stm32::flash::WRITE_SIZE == 8`, asserted at compile time).

Two independent flags:

- `app_valid` — the slot holds a complete, verified image.
- `update_requested` — one-shot, set by the application, cleared by the
  bootloader the moment it enters the updater.

Keeping them separate is what makes "clear the request immediately" safe: after
the request is cleared the record still says `app_valid`, so an external reset
boots the application again. A stray "enter update mode" cannot strand the node.

## Update sequence and what each step guarantees

| Step | Object | Effect |
|---|---|---|
| declare length | `0x2000:1` | sizes the erase; writable in any state so a retry works |
| expected CRC | `0x1F56:1` | CRC-32 ISO-HDLC over the whole image |
| clear | `0x1F51:1` = 3 | appends `!app_valid`, **then** erases |
| chunks | `0x1F50:1` | domain writes, streamed into flash |
| validate | `0x1F51:1` = 1 | CRC read back **from flash**, then appends `app_valid` |
| reset | `0x1F51:1` = 2 | `SCB::sys_reset()` |

Read `0x1F57:1` for status (0 idle, 1 programming, 2 valid, 0xFF error) and
`0x2000:2` for bytes accepted. That is the resume point a host *could* use
after a timeout; `canopen-fwupdate` does not — it re-issues `clear` and starts
over. Resuming only works while the node has not reset in between, because the
`Programmer`'s progress lives in RAM.

The invalidate-before-erase ordering is the whole point. From the first erased
page until validation succeeds, a reset lands in the updater rather than in a
half-written image.

**Both destructive commands are state-gated, and both gates matter.** `clear`
is rejected unless a length has been declared, and `validate` unless the node
is already `PROGRAMMING`. Without the second gate a single stray write to
`0x1F51:1` destroyed a perfectly good application: with no session running the
declared length is 0, so `validate` reads "0 of 0 bytes received", clears the
completeness check, fails on the empty image, and `fail` invalidates the slot
on the way out. Note the command values here are *not* CiA 302's — the spec
reads 1 as "start program", so a generic tool trying to start the application
sends exactly that byte.

Neither image authenticates anything. The CRC proves the image arrived intact,
not that it was meant for this node, and `ENTER_UPDATE_MAGIC` is a fixed
constant in a public repository. Anyone on the bus can reflash the board. That
is acceptable for a PoC on a closed bus and would not be for a product.

Validation CRCs the memory-mapped slot, not the bytes as they arrived, so a
write that failed silently is caught before the image can ever be booted.
The accepted-byte counter is kept separately from the number of bytes committed
to flash, so padding the final partial doubleword with `0xFF` does not make a
valid, non-8-byte-aligned image look longer than the host declared.

Application side: write `ENTER_UPDATE_MAGIC` (`"UPDT"`) to `0x2F00:0`. Any
other value is rejected with an SDO value-range abort.

## Verified on hardware

Node 42 on `can0` at 500 kbit/s, against a real EMDM board:

- Full update: erase 0.5 s, 46,128 B in 7.5 s, verify, reboot — 8.7 s total.
- Application runs: RPDO1 `0x22A` bit 0 drives the LED, TPDO1 `0x1AA` echoes
  `led_state` in bit 0 and `uptime_s` in bits 1..32.
- `enter-update` round trip, and a plain reset afterwards boots the
  application — confirming the one-shot request really is cleared.
- **Interrupted update**: host killed mid-download, then hard reset. Node came
  back in the updater, still reachable, and reflashed cleanly.
- **Corrupted image** (`--corrupt` fault injection): validation failed, status
  went to error, a reset did *not* boot it, and a clean reflash recovered.
- Stray write of a wrong value to `0x2F00:0` aborts `0x06090030` and the node
  stays on the bus.
- **Dirty metadata page**: bootloader installed over a firmware whose image
  covered `0x0800E000`. First `clear` erased the page and the update proceeded
  normally.

## Timing notes

- Page erase ~24 ms, doubleword program ~83 µs. A 46 KiB image is 23 pages, so
  the erase stalls the executor for about 0.5 s and each 256-byte chunk for
  about 2.5 ms. Control writes therefore need a seconds-scale SDO timeout on
  the host; `canopen-fwupdate` uses 5 s.
- **No flash operation may run inside `NODE.with()`.** `SharedNode::with` holds
  a `CriticalSectionRawMutex`, so its closure runs with interrupts disabled;
  parking a 2.5 ms chunk program (9 ms at the full 889 bytes) in there stalls
  the FDCAN interrupt and the embassy time driver for the whole download.
  `handle_changes` therefore copies the chunk out of the OD first and programs
  it outside the lock, and `clear`/`append_meta` only read the OD under the
  lock. The erase and every metadata write were already outside it.
- The erase is sized from the declared length rather than always covering the
  whole slot, which is why the length must be declared first.
- `RX_CHANNEL` is 64 deep, not 16. An SDO block transfer delivers up to 127
  back-to-back frames — roughly 7.5 per millisecond at 500 kbit/s — while the
  protocol task drains it once per millisecond.

## Installing over some other firmware

The metadata page must be **erased** before the bootloader can use it. Flashing
the bootloader alone does not do that: `probe-rs download` only erases the
pages the bootloader itself occupies, so if the board previously ran an image
that spanned `0x0800E000`, that page still holds its code.

`scan_meta` therefore picks the next slot by looking for one that is entirely
`0xFF`, not by counting valid records. A page full of foreign bytes reports "no
free slot", which makes `append_meta` erase it and start over. Choosing the
slot by record count instead programs a doubleword that was never erased, which
STM32G4 flash rejects — the symptom is a flash error on the very first `clear`,
with the node otherwise healthy. This is covered by
`page_full_of_foreign_data_reports_no_free_slot` and verified on hardware.

`probe-rs erase --chip STM32G431CBTx` before installing avoids the question
entirely.

## Debug attach

Do not assume a board is dead because an attach reports:

```
Debug Port: DPv1, Designer: ARM Ltd
└── No access ports found on this chip.
```

The DP runs off the probe's SWCLK and always answers; failing to enumerate the
AP behind it is a *connect* failure, not evidence about the firmware. It was
seen once here and has not reproduced since — including 5/5 clean attaches
against the same firmware that was running at the time, with and without
`--chip`, and with JTAG probed first. Treat it as intermittent.

Two things it is **not**:

- Not sleep-mode clock gating. `embassy_stm32::init()` sets
  `DBGMCU_CR.DBG_SLEEP`/`DBG_STOP`/`DBG_STANDBY` by default
  (`Config::enable_debug_during_sleep`, `true`); the register reads `0x7` on a
  running board here.
- Not the `defmt-rtt` hazard documented in the statuslight repo. That one hangs
  the firmware *after* a debugger attaches and stops draining RTT in blocking
  mode. It does not stop an AP from enumerating.

`--connect-under-reset` is a good reflex when an attach misbehaves, and is
required if a firmware ever does disable the debug interface.

## Known limitation: 256-byte chunks

`0x1F50:1` is a `domain<889>` and the node's SDO server buffers 889 bytes, but
`SdoDriver::download` builds its client as `SdoClient::<256>` with the size
hardcoded. So the host cannot push more than 256 bytes per transfer, and the
889-byte capacity is unreachable until that gains a size parameter. Uploads are
not affected — `canopen_linux::sdo_helpers::sdo_upload` allocates the full 889.

This costs roughly 3.5x in wall-clock time on a full image.
