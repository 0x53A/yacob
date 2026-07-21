# Timing & Cost Characterization

> **Status: idea / design note (2026-07-21)** — nothing implemented. Captured
> so the approach is not re-derived later.

## Goal

Two distinct things that are easy to conflate:

1. **Regression gating (CI).** Detect the commit that makes `process()` 30%
   more expensive. Needs determinism and reproducibility, not physical
   accuracy. A unitless budget is fine.
2. **A defensible cost claim.** "On a 100 MHz Cortex-M4, this stack costs
   under X% CPU at 50% bus load." Needs real cycles on real silicon, or at
   least a calibrated proxy.

These want different tools. Building only (1) and stating (2) from it is the
main trap.

## Approach: two tiers

### Tier 1 — instruction counting under emulation

Run the firmware in an emulator, count retired instructions between two
markers. Deterministic, no hardware, runs in CI.

Candidate hosts:

- **Unicorn Engine** — easiest to drive from a Rust test harness: map the
  ELF, install a per-instruction hook, count. No device models, which is fine
  because the node under test is driven over `MailboxTransport` with canned
  frames — no CAN peripheral involved.
- **Renode** — full platform models, scriptable, already has instruction
  counting. Heavier, but gives a path to modelling the CAN peripheral later.
- **QEMU with a TCG plugin** — works, but the most plumbing for the least
  benefit here.

Preference: Unicorn for the CI harness. The node never touches a peripheral,
so a full platform model buys nothing.

**Must be pinned in the harness or the numbers drift meaninglessly:**
target triple, opt-level, LTO setting, `codegen-units`, panic strategy,
toolchain version. Treat a toolchain bump as a deliberate rebaseline, not a
regression.

### Tier 2 — DWT CYCCNT on real hardware

The Nucleo-G431KB in `hil-tests/` already gives ground truth. Enable
`DEMCR.TRCENA` + `DWT.CYCCNT`, read it around the call under test. This is
cheap to add and is what the public claim should be based on.

It also yields a measured instructions→cycles ratio for the exact code paths
we care about, which makes the Tier 1 numbers interpretable rather than
merely comparable.

## Why instruction count is not cycles

- M0/M3/M4 executing from zero-wait-state SRAM: CPI ≈ 1.1–1.5, proxy is
  decent (order ±30%).
- Real firmware runs from flash. STM32G4 at 170 MHz has 4–5 wait states with
  ART papering over it; prefetch does badly on branch-dense code, and the SDO
  command-byte dispatch is exactly that.
- M7/M33 with I/D cache: instruction count says very little about time.

Also: a single input path gives a *typical* cost, not a WCET. Adequate for a
duty-cycle statement, not for a real-time guarantee.

## What to measure

The duty cycle is dominated by the boring path, not the interesting one.
Measure per-operation, then build a small linear model over bus load and poll
rate:

- `process()` with an empty RX queue — at a 1 kHz poll rate this is most of
  the budget
- RPDO reception + unpack, per frame (vary mapping count; bit-granular vs
  byte-aligned should differ measurably)
- TPDO transmit on SYNC, per frame
- heartbeat / timer tick, producer and consumer
- SDO expedited round trip; per-segment cost for segmented and block transfer
- OD lookup cost vs. entry count (the macro's dispatch is a match tree —
  worth knowing how it scales)

Report as e.g. "at 500 kbps, 50% bus load, 1 kHz polling: under 4% of a
100 MHz M4", not as a single number. The linear model is both more defensible
and more useful to us for finding where the cost is.

**Scope caveat that must ship with any published figure:** emulation excludes
the CAN peripheral ISR and driver. Label it "stack cost, excluding CAN
driver" or the number is wrong by a margin we do not control.

## Related, cheap, worth doing anyway

- **Static stack depth** via `cargo-call-stack` — the other resource an
  embedded integrator asks about, and there is no recursion in the stack, so
  it should give clean bounds.
- **Flash/RAM footprint per feature** — `cargo-bloat` / `size` across feature
  combinations and OD sizes. Complements the CPU story and is nearly free.

