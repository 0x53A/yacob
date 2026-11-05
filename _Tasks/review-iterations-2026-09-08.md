# Three further review iterations — 2026-09-08

Scope: bounded upstream fixes on top of the existing uncommitted reviews,
plus read-only tracing of the active TurnGrip firmware/gateway. No downstream
edits or dependency updates, hardware actions, staging, commits or pushes.
Recommendations below are not user decisions.

## Fixed: receive framing and error ownership

`canopen-core/src/slcan/serial.rs` now consumes complete buffered lines before
reading again. ACKs and malformed lines cannot hide a following valid frame;
a later serial error cannot prevent delivery of a frame already received.
An overlong unterminated line is discarded through its next delimiter, so a
valid-looking suffix cannot be interpreted as an independent CAN command.
Memory stays bounded at 256 bytes; each receive performs at most one port read.

`canopen-core/src/can_router.rs`: `SdoPort` forwards all frames on its target
response COB-ID to the driver, including malformed DLC. The driver owns DLC
validation and returns ProtocolError; the wrapper previously hid the frame
and could turn the failure into a timeout or transport error.

Four focused regressions failed before the fixes and passed afterwards.
The core unit suite then passed all 223 tests.

## Fixed: EDS wire-value narrowing

`canopen-derive/src/eds_parser.rs` rejects malformed or overflowing nonempty
PDO COB-ID defaults, UNSIGNED32 mapping descriptors and UNSIGNED16 DataType
codes. Previously these could wrap into different valid values, silently
fall back to a default COB-ID, or ignore extra expression terms.

COB-ID expressions support a checked numeric value, `$NODEID`, and either
ordering of `$NODEID + numeric-base`. Missing/empty defaults retain their
previous fallback behavior. Unsupported but in-range DataType codes and
unresolved mapping targets retain their existing handling; this is not a
complete strict EDS validator.

Three new regressions failed before the fix; all 15 derive unit tests passed
afterwards. Mapping overflow is exercised for both PDO directions.

## Fixed: SDO availability across NMT transitions

`canopen-core/src/node.rs` accepts SDO requests only in Pre-Operational or
Operational, on both default and additional server channels. This also
rejects requests behind Reset in the same receive batch, before boot runs.
Entering Stopped aborts all local SDO transfers and pending responses before
the next frame is dispatched. Stop/Start in one batch cannot resurrect them.
Entering Pre-Operational intentionally preserves an active SDO transfer.

Three regressions failed before implementation: writes while Stopped,
refused-response replay across Stop/Start, and stale segmented-download
completion. They now pass, with checks for both reset commands and additional
channels, plus a positive Pre-Operational segmented-transfer case.

This service-state behavior is consistent with the explicit
`NMTisPreOrOperational` guard in
[CANopenNode's SDO server](https://github.com/CANopenNode/CANopenNode/blob/master/301/CO_SDOserver.c).
The change is at Node orchestration, not a redesign of the standalone server.
It does not retract already accepted transport frames or undo committed OD
writes/application events. Other services' state gating is not certified by
this pass; boot/LSS refusal and other previously recorded gaps remain open.

## TurnGrip interaction findings

Paths below are relative to the `frost-pcb-endeffector` repository.

1. **The local fixes are not deployed downstream.** Both
   `firmware/firmware_v2/Cargo.toml` and `gateway/Cargo.toml` pin yacob
   `7955e951ab18d7907cd50908a2e93d163b29827b`, not this working tree.
2. **Acceptance is not wire transmission.** In
   `firmware/firmware_v2/src/can.rs`, `ChannelTransport::transmit` returns
   success when a frame enters the 32-entry TX channel. `tx_task` removes a
   frame and then awaits FDCAN admission. There is no protocol-reset purge in
   that transport path: clearing yacob's pending work cannot remove these
   accepted frames or the task-held frame. The SDO diagnostic is logged before
   FDCAN admission, so it is not proof of completed bus delivery.
3. **Stopped-SDO expectations lacked an assertion.**
   `firmware/firmware_v2/core/src/protocol.rs`'s
   `nmt_filter_prevents_state_changes_and_reset_side_effects` queues a secondary
   SDO after NMT and comments on Stopped. It checks SDO delivery when NMT is
   filtered out, but only checks state/reset when NMT is accepted. It never
   asserts absence of an SDO response after accepted Stop. This led to the
   upstream reproductions above.
4. **Stop/Start can be lost between protocol and motor-control sampling.**
   `firmware/firmware_v2/src/control.rs` snapshots `node.state()` and parks the
   motor/clears stored motion when that snapshot is non-Operational. Node can
   drain Stop and Start in one `process()` call, leaving an Operational final
   state. Consequently the control task can miss that stop boundary. This is
   a source-level integration gap, not a hardware reproduction. A latched
   transition/stop notification or equivalent explicit handoff needs agreement;
   no such architecture change was made here. Library RPDO/SDO cleanup alone
   is not a physical-stop guarantee.
5. **There really are two concurrent SDO channels in this gateway.**
   `gateway/src/can.rs` uses ordinary traffic on fixed request/response
   0x641/0x640 and a separate emergency worker on the node's default channel.
   Each has a one-frame TxQueue. Its `SecondarySdoPort` already validates DLC,
   so the upstream SdoPort fix is not a new fix to this wrapper. The previous
   stale-segment issue remains: a two-second transfer deadline and a detached
   frame's separate 500 ms retry age are not one cancellation lifetime.
   The new Stopped-state SDO gate also means the emergency SDO is unavailable
   while Stopped; control already has a non-Operational stage-off path, but
   that must not be confused with acknowledgement of an emergency request.

## Verification

- `cargo test --workspace --exclude hil-tests --locked`: **418 passed**,
  zero failures, 17 existing ignored documentation examples.
- `uv run pytest -q` in `interop-tests`: **57 passed**, 73.86 seconds.
- Core checks with `--no-default-features`, then
  `--no-default-features --features alloc,embassy,defmt`: passed.
- Ten new regression tests across these three iterations. No new ignores.
- `git diff --check`: clean. Index unchanged/empty.

Full-run logs: `/tmp/yacob-iterations-workspace.log`,
`/tmp/yacob-iterations-interop.log`, `/tmp/yacob-iterations-features.log`.
The dependency's existing proc-macro-error2 future-incompatibility warning
remains. No hardware or downstream integration build was claimed.

## Subsequent lifecycle iteration — 2026-09-08

Following the Stop/Start finding through the adjacent lifecycle paths found
three more bounded upstream defects:

1. **Stopped nodes still transmitted EMCY.** Node now defers its existing
   four-frame EMCY queue until Pre-Operational/Operational, while heartbeat
   production and local error-register updates continue. Error reports and
   error-reset announcements retain their order and existing bounded overflow
   behavior. This is diagnostic history, not a queue of motion commands.
   Both Start and Enter Pre-Operational resume transmission. This agrees with
   [CANopenNode's emergency processing guard](https://github.com/CANopenNode/CANopenNode/blob/master/301/CO_Emergency.c).
2. **Reset could produce two apparent boot-up announcements.** When a reset
   arrived on a due-heartbeat tick, the periodic producer sent state zero
   while initialization was pending; the next call sent the real boot-up.
   Node now suppresses periodic heartbeat production while `booted` is false.
   Pending EMCY also waits until boot has completed. The regression exercises
   Reset Application and Reset Communication, then ordinary heartbeat resumption.
3. **Start queued behind Reset bypassed initialization.** NmtHandler accepted
   Start/Stop/Enter Pre-Operational while Initializing. Reset + Start + SDO or
   RPDO/SYNC in one Node receive batch could therefore mutate the OD before
   the next boot sequence. Commands during initialization are now ignored;
   only `boot_complete()` leaves that state. Auto-start after boot still works,
   as do fresh subsequent SDO requests. This also changes standalone handler
   behavior: callers must explicitly complete boot before sending state commands.

Four new regressions failed before their respective fixes: two Node lifecycle
cases, a standalone NMT initialization test, and a Node Reset/Start/SDO/RPDO
batch test. The workspace run now passes **422 tests/doctests**, with the same
17 ignored documentation examples. Both no-default-feature checks still pass.
The Python interoperability suite also passed **57 tests** in 72.52 seconds
(`/tmp/yacob-lifecycle-interop.log`). No new ignores; diff whitespace checks
pass and the index remains empty.
Logs: `/tmp/yacob-lifecycle-{before,after,workspace,features}.log`,
`/tmp/yacob-resetstart-{before,node-before}.log`.

Further TurnGrip tracing (read-only) confirms another event/level boundary:
`firmware/firmware_v2/src/control.rs::read_snapshot_and_events` deliberately
drains and discards heartbeat events, then samples `heartbeat_status().is_alive()`.
A timeout followed by recovery between control samples can thus be invisible
to the application. That is distinct from DSIM-02, where delayed Node polling
can prevent the Timeout event itself from being produced. Whether *every*
observed lease lapse must latch a stop/fault, or only current freshness matters,
is an application policy question. No heartbeat timing policy or control
contract was silently changed. The Stop/Start handoff, downstream TX queues,
pins, and initial boot-up TX refusal remain unresolved.

## LSS follow-up — 2026-09-08

**Fixed: selective switching was still serviced in Configuration mode.**
A successfully selected slave retained its identity-match flags and replied
again to a repeated serial-number tail. A globally configured slave also
acknowledged a subsequent full selective exchange. These are spurious 0x44
selection responses, not fresh Waiting-to-Configuration transitions.

All four selective commands now require Waiting mode, and successful selection
clears its temporary match flags. Returning to Waiting permits a fresh full
selection as before. No queues, public API changes or new ordering/timeout
policy for incomplete selections in Waiting were introduced.
This mode restriction agrees with
[CANopenNode's LSS slave](https://github.com/CANopenNode/CANopenNode/blob/master/305/CO_LSSslave.c).

Two new regressions failed before the fix: repeated serial after selection,
and full selective exchange after a global switch to Configuration. They
also check the exact positive response and fresh selection after returning
to Waiting. Module documentation now uses the correct request/response IDs
(0x7E5/0x7E4); the code constants were already correct. It also accurately
describes the pending-event API rather than a nonexistent callback.

R13 remains open: Node does not expose/apply LSS configuration events, and
Store Configuration success does not mean persistence has completed. The
module documentation now states that limitation explicitly. This pass is
not evidence of a working integrated commissioning path, nor a fix for LSS
TX refusal. No downstream or hardware changes were made.

Workspace verification: **424 tests/doctests passed**, zero failures, the same
17 ignored documentation examples. Core checks with no default features and
with `alloc,embassy,defmt` passed. Logs: `/tmp/yacob-lss-before.log`,
`/tmp/yacob-lss-workspace.log`, `/tmp/yacob-lss-features.log`.
Python interoperability: **57 passed** in 71.15 seconds
(`/tmp/yacob-lss-interop.log`). Diff whitespace checks pass; index unchanged.

## Async delivery and deadline passes — 2026-09-08

Three focused passes produced the following bounded fixes:

1. **Shared-bus subscriber wakeups ran under the subscriber-list mutex.**
   Inline executor code reentering `subscriber_count`, subscribe or publish
   could deadlock. Publish now completes the entire fanout, collects its
   wakeups, releases the list lock, then wakes subscribers. The regression
   fails with a try-lock assertion rather than hanging on the old code; its
   wake callback then actually publishes a second frame and verifies both
   subscribers see the original frame before the reentrant one. Queue capacity,
   overflow policy and publication serialization are unchanged. Temporary
   waker collection is std-only; no persistent CAN-frame queue was added.
   **Only the wakeup part of R11 is fixed:** arbitrary filters still run under
   the list lock and must not reenter the bus. Close/cancellation and channel
   ownership issues remain open.
2. **`CanDemux::recv_heartbeat_timed` did not time out inside pending receive.**
   It checked the deadline outside `receive().await`, so a silent transport
   could remain pending even when the caller kept polling beyond the deadline.
   Each poll now checks elapsed time; Embassy builds also arm a timer. Without
   Embassy the caller must arrange polling/external timer wakeups, as documented
   for SdoDriver. A fake-clock regression exercises exact expiry and epoch wrap;
   a positive test checks node filtering, preservation of unrelated PDO/heartbeat
   frames, and the existing buffered-first behavior with zero timeout.
3. **Timed SDO operations mixed wrapping deadline construction with ordinary
   absolute comparisons.** A transfer starting near u64::MAX could immediately
   time out. A private elapsed-time budget now spans all TX/RX waits and segments.
   The regression covers upload/download, blocked TX and blocked RX, and exact
   expiry across wrap. This is chiefly arbitrary-epoch/synthetic-clock robustness,
   not a likely ordinary uptime failure. No public signature or cancellation
   ownership changed; accepted transport frames still cannot be recalled.

The three defect regressions failed before implementation. Four tests were
added in total (including the positive heartbeat routing case). Workspace:
**428 tests/doctests passed**, zero failures, 17 existing ignored documentation
examples. Both core feature checks passed (no defaults; alloc,embassy,defmt).
The Embassy path was compile-checked, not exercised on an embedded executor.
Logs: `/tmp/yacob-{waker,hbwait,sdowrap}-{before,after}.log`,
`/tmp/yacob-async-workspace.log`, `/tmp/yacob-async-features.log`.
Python interoperability: **57 passed** in 71.31 seconds
(`/tmp/yacob-async-interop.log`). Diff whitespace checks pass; index unchanged.
No hardware actions, downstream pin/code changes, staging, commits or pushes.

## Encoder and serial-boundary passes — 2026-09-08

Three further passes fixed narrower boundary defects:

1. **Public upload-segment encoding panicked for >7 data bytes.**
   `encode_upload_segment_response` now returns `Option<[u8; 8]>`, like the
   expedited encoder: None rejects oversized input, including in release
   builds. The single workspace caller in SdoServer already limits segments
   to seven bytes and now explicitly unwraps that invariant. A source search
   found no direct callers in the FROST Rust repositories. This is a small
   public API change, not a discovered remote-triggered server panic.
   Regression checks 8..=16-byte rejection and valid 0..=7-byte encoding/toggles.
2. **UnixSerialPort ignored terminal-setup errors.** Opening `/dev/null`
   previously reported success despite tcgetattr/tcsetattr failing. Termios
   read, speed and apply errors now propagate with operation/path context.
   File ownership begins immediately after successful open, closing the
   descriptor on every setup error. Besides the failing-before regression,
   a pseudoterminal test checks successful raw/nonblocking 115200 setup,
   disabled HUPCL and zero VMIN/VTIME. No physical serial device was opened.
   Adapter command acknowledgements and DTR/RTS ioctl reporting are separate
   unresolved paths; this does not certify successful CAN adapter initialization.
3. **SLCAN accepted arbitrary trailing bytes after a valid frame prefix.**
   Parsing now consumes the whole line, accepting only the data implied by
   DLC plus an optional four-hex-digit timestamp and optional trailing CR.
   This preserves timestamped frames described in the
   [LAWICEL CANUSB manual](https://www.canusb.com/files/canusb_manual.pdf).
   Garbage, malformed timestamp tails, embedded delimiters and concatenated
   frames are rejected. The earlier all-256-DLC-byte regression now supplies
   matching payload lengths for valid DLCs instead of relying on ignored
   trailing data. Timestamp metadata is still discarded, not exposed to apps.

The three defect reproductions failed before the fixes. Four tests were added
including the successful pseudoterminal case. **432 workspace tests/doctests
passed**, zero failures, the same 17 ignored documentation examples. Both core
feature checks passed. Logs: `/tmp/yacob-{encoder,serialopen,slcanline}-before.log`,
`/tmp/yacob-boundaries-workspace.log`, `/tmp/yacob-boundaries-features.log`.
Python interoperability: **57 passed** in 73.72 seconds
(`/tmp/yacob-boundaries-interop.log`). Diff whitespace checks pass; index unchanged.
Existing pending-frame ownership, lifecycle handoff and commissioning issues
remain open. No downstream code/pin changes, staging, commits or pushes.

## Convergence pass — 2026-09-08

Stopping criterion: after each actionable nontrivial fix, conduct another
fresh higher-risk pass. Stop when that pass finds no further such defect,
excluding the explicitly deferred design questions. This is not a proof that
the repository is bug-free, nor a claim to have re-reviewed every workspace
tool, example, or downstream firmware during this pass.

Two more defects were reproduced and fixed:

1. **Block download could acknowledge and discard a final overflow segment.**
   After 127 non-final segments filled the 889-byte buffer, another final
   segment copied zero bytes but received an ACK. The end request could then
   commit the truncated payload, including when the declared length was 889.
   The server now aborts with OutOfMemory before acknowledging any segment
   lacking a complete seven-byte slot. Regression covers indicated/unspecified
   sizes, rejection without OD mutation/events, and a valid exactly-full
   889-byte transfer. The regression failed before the fix.
2. **SDO PDO COB-ID writes silently discarded unsupported identifier bits.**
   The server accepted bits 11..=29 although the runtime engine uses only
   the low 11 identifier bits; an extended-ID request became a different
   standard frame. Such writes now abort with ValueRangeExceeded before
   changing the OD. Regression tests every unsupported bit for TPDO/RPDO,
   enabled/disabled, and verifies unchanged configuration and no event.
   Existing validity/RTR flag handling is preserved and positively tested;
   this does not implement RTR support. The regression failed before the fix.
   Direct application configuration and shared validation across all ingress
   paths remain part of the recorded configuration-design follow-up.

A systematic client/server regression also round-trips every supported
segmented-client length 0..=889, downloading then uploading each payload and
checking exact bytes and write-event count. This covers zero-length,
expedited, segmented, and every seven-byte boundary; it found no further bug.

The final post-fix pass revisited SDO length/abort/commit boundaries, block
completion, node reset/Stop and pending-response handoff, PDO dispatch and
mapping bounds, heartbeat lifecycle, async driver response validation,
SLCAN framing and SocketCAN acceptance boundaries. TurnGrip gateway pending
SDO slots and firmware event/state consumption were rechecked read-only.
No additional actionable nontrivial defect was established in that pass.

Remaining concerns are still explicitly open: cancellation beyond transport
acceptance, detached gateway frames, displaced hardware TX, boot/LSS refusal,
unbounded poll work, live reconfiguration phase/validation, and firmware
sampling of Stop/Start and heartbeat timeout/recovery. TurnGrip still pins
an older yacob revision. No new architecture or downstream pin change was made.

Verification: **435 workspace tests/doctests passed**, zero failures, the same
17 ignored documentation examples. Both core checks passed (no defaults;
alloc,embassy,defmt). The existing proc-macro-error2 future-compatibility warning
remains. Logs: `/tmp/yacob-convergence-{before,after}.log`,
`/tmp/yacob-convergence-pdo-{before,after}.log`,
`/tmp/yacob-convergence-matrix.log`, `/tmp/yacob-convergence-workspace.log`.
Python interoperability: **57 passed** in 75.36 seconds
(`/tmp/yacob-convergence-interop.log`). Diff whitespace checks pass; index
remains unchanged.
Hardware verification remains deferred. No staging, commits or pushes.

## Final review and small cleanups — 2026-09-08

Reviewed the accumulated protocol/transport/generator changes and their notes
once more; no new actionable nontrivial defect was established. Deferred
architecture and downstream/hardware boundaries remain unchanged.

- Removed the redundant application-trigger inhibit check: application and
  timer triggers still share the same inhibit gate and acceptance bookkeeping.
- Replaced the review fixture's allocated no-op waker with `Waker::noop()`.
  Tightened the block-overflow regression to assert the actual OutOfMemory
  abort code, not just the presence of an abort.
- Changed the decimal float-parser fixture from approximate-PI test data to
  exactly representable 1.25, asserting the exact wire bytes. This removes
  Clippy's deny-by-default approximate-constant error without suppressing it.
- Clarified public comments for heartbeat polling, queue acceptance versus
  transmission, blocked sends after consumer exit, filter panic constraints,
  segmented-buffer limits, and TX/RX timeout scope. Corrected SDO no-response
  documentation, the DCF apply example, and two broken rustdoc links.
- Consolidated CLAUDE.md's accumulated review preamble; corrected fixed/open
  statuses and stale timeout descriptions in the issue index. Busy-channel
  handling is explicitly a design discussion, not a presumed user decision.

Verification: **435 workspace tests/doctests passed**, zero failures, 17 existing
ignored examples. Clippy completed successfully for core/linux/derive with
all targets and no dependency linting; advisory style warnings remain (this
was not a repository-wide lint/format rewrite). Core/Linux API documentation
builds without warnings. Both core feature checks passed (no defaults;
alloc,embassy,defmt). Logs: `/tmp/yacob-final-workspace.log`,
`/tmp/yacob-final-clippy-after.log`, `/tmp/yacob-final-doc-after.log`,
`/tmp/yacob-final-minimal.log`, `/tmp/yacob-final-features.log`.
Python interop was not rerun for these behavior-preserving cleanups; the
preceding convergence run passed 57 tests. No hardware or downstream changes,
staging, commits or pushes.
