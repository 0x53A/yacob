//! CANopen firmware updater for the EMDM board (STM32G431CB).
//!
//! This is the immutable half of the pair. It is flashed once over SWD and is
//! never rewritten over CAN, which is what makes an interrupted update
//! recoverable: whatever happens to the application slot, this code still runs
//! after the next reset and can be talked to over the bus.
//!
//! # Boot decision
//!
//! The decision to run the application is taken *before* any peripheral is
//! touched — flash is memory-mapped, so reading the metadata log and CRC-ing
//! the image needs no clock, no DMA and no interrupts. The jump therefore
//! happens from a state that is indistinguishable from reset (HSI, no NVIC
//! interrupt enabled), so the application's `embassy_stm32::init()` behaves
//! exactly as it would if it had been flashed at 0x0800_0000.
//!
//! Trying to jump *after* bringing up the PLL, FDCAN and the time driver is
//! the usual way this goes wrong: the application then has to undo a clock
//! tree it knows nothing about.
//!
//! # Update protocol
//!
//! CiA 302 programming objects over SDO, all on the default server:
//!
//! - `0x2000:1` image length — host declares the total size first
//! - `0x1F51:1` program control — `3` clear, `1` validate, `2` reset
//! - `0x1F50:1` program data — firmware chunks, up to 889 bytes per transfer
//! - `0x1F56:1` expected CRC-32 (ISO-HDLC) over the whole image
//! - `0x1F57:1` flash status — 0 idle, 1 programming, 2 valid, 0xFF error
//! - `0x2000:2` bytes accepted so far, so a host can resume after a timeout
//!
//! # Power-fail behaviour
//!
//! Every transition is a new record appended to the metadata log:
//!
//! | Interrupted during | Metadata in force | Next boot |
//! |---|---|---|
//! | download, before "clear" | `app_valid` | old application |
//! | erase or programming | `!app_valid` | updater |
//! | after "validate" | `app_valid`, new CRC | new application |
//!
//! The application is only marked valid after its CRC has been read back out
//! of flash, so a write that silently failed is caught before it can be booted.

#![no_std]
#![no_main]

use canopen_core::cobid::NodeId;
use canopen_core::node::{Node, NodeConfig};
use canopen_derive::object_dictionary;
use cortex_m::peripheral::SCB;
use cortex_m_rt::entry;
use defmt::{error, info, warn};
use embassy_executor::{Executor, Spawner};
use embassy_futures::select::{select, Either};
use embassy_stm32::flash::WRITE_SIZE;
use embassy_time::{Duration, Ticker, Timer};
use static_cell::StaticCell;

use emdm_fwupdate_common as fw;
use fw::hal::{ChannelTransport, EmbassyClock, Nvm, StatusLed};
use fw::{flash_status, program_control, FlashOps, MetaRecord};

use {defmt_rtt as _, panic_probe as _};

/// Doublewords are the STM32G4 programming granularity; a chunk that is not a
/// multiple of it has to be carried over into the next write.
const _: () = assert!(WRITE_SIZE == 8);

// ---------------------------------------------------------------------------
// Object dictionary
// ---------------------------------------------------------------------------

object_dictionary! {
    #[validate_write(check_write)]
    pub struct BootOd {
        [0x1000] device_type: u32 = 0, ro;
        [0x1001] error_register: u8 = 0, ro;
        [0x1008] device_name: visible_string<32>, ro;
        [0x1018] identity: record {
            [1] vendor_id: u32 = 0, ro;
            [2] product_code: u32 = 0, ro;
            [3] revision: u32 = 1, ro;
            [4] serial_number: u32 = 0, ro;
        };

        // CiA 302 programming objects.
        [0x1F50] program_data: record {
            // 889 is `fw::MAX_CHUNK`, which sizes the copy in `handle_changes`.
            // The DSL only takes a literal here, so the two go together by
            // hand — keep them in step.
            [1] firmware_chunk: domain<889>, rw;
        };
        [0x1F51] program_control: record {
            [1] command: u8 = 0, rw;
        };
        [0x1F56] program_sw_id: record {
            [1] expected_crc: u32 = 0, rw;
        };
        [0x1F57] flash_status_id: record {
            [1] flash_status: u32 = 0, ro, pdo;
        };

        // Manufacturer-specific: everything the host needs to size a transfer
        // and to resume one.
        [0x2000] update_info: record {
            [1] image_length: u32 = 0, rw;
            [2] bytes_written: u32 = 0, ro, pdo;
            [3] app_base: u32 = 0, ro;
            [4] app_capacity: u32 = 0, ro;
        };

        tpdo[1](transmission_type = event_driven, inhibit_time = 50ms, event_timer = 1s) {
            flash_status,
            bytes_written,
        };
    }
}

impl BootOd {
    /// Reject writes that the programming state machine could not honour, so
    /// the host gets an SDO abort instead of a silently ignored transfer.
    fn check_write(
        &self,
        index: u16,
        subindex: u8,
        data: &[u8],
    ) -> Result<(), canopen_core::od::OdError> {
        use canopen_core::od::OdError;

        match (index, subindex) {
            // Chunks are only meaningful between "clear" and "validate".
            (0x1F50, 1) => {
                if self.flash_status != flash_status::PROGRAMMING {
                    return Err(OdError::HardwareError);
                }
                let received = self.bytes_written as usize + data.len();
                if received > self.image_length as usize {
                    return Err(OdError::ValueRange);
                }
                Ok(())
            }
            // The declared length only has to fit the slot. It is deliberately
            // writable in every state: a download that failed part way through
            // leaves the node in PROGRAMMING, and re-declaring the length is
            // the first thing a retry does. It takes effect at the next
            // "clear", which is what sizes the erase.
            (0x2000, 1) => {
                let len = u32::from_le_bytes(data.try_into().map_err(|_| OdError::ValueRange)?);
                if len == 0 || len > fw::APP_SIZE {
                    return Err(OdError::ValueRange);
                }
                Ok(())
            }
            (0x1F51, 1) => {
                let cmd = *data.first().ok_or(OdError::ValueRange)?;
                match cmd {
                    program_control::CLEAR => {
                        // "clear" sizes the erase from the declared length.
                        if self.image_length == 0 {
                            Err(OdError::ValueRange)
                        } else {
                            Ok(())
                        }
                    }
                    // "validate" only means anything for a download that is
                    // actually in progress. Accepting it in every state let a
                    // single stray write destroy a good application: with no
                    // session running the declared length is 0, which
                    // `validate` reads as "0 of 0 bytes received", so it gets
                    // past the completeness check and then fails on the empty
                    // image — and `fail` invalidates the slot on the way out.
                    //
                    // Not a hypothetical: 1 is CiA 302's "start program", so a
                    // generic tool trying to *start* the application sends
                    // exactly the byte that would have wiped it.
                    program_control::VALIDATE => {
                        if self.flash_status != flash_status::PROGRAMMING {
                            Err(OdError::HardwareError)
                        } else {
                            Ok(())
                        }
                    }
                    program_control::RESET => Ok(()),
                    _ => Err(OdError::ValueRange),
                }
            }
            _ => Ok(()),
        }
    }
}

// ---------------------------------------------------------------------------
// Programmer
// ---------------------------------------------------------------------------

/// Streams received chunks into the application slot.
///
/// SDO transfers are capped at 889 bytes, which is not a multiple of the 8-byte
/// flash write granularity, so the tail of each chunk is carried over into the
/// next write rather than padded in place.
struct Programmer {
    /// Logical image bytes accepted from the host. Unlike `flushed`, this
    /// deliberately excludes the erased-value padding added by `finish`.
    accepted: u32,
    /// Bytes already committed to flash. Always a multiple of `WRITE_SIZE`.
    flushed: u32,
    pending: [u8; WRITE_SIZE],
    pending_len: usize,
}

impl Programmer {
    const fn new() -> Self {
        Self {
            accepted: 0,
            flushed: 0,
            pending: [0xFF; WRITE_SIZE],
            pending_len: 0,
        }
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    /// Logical number of image bytes accepted so far.
    fn received(&self) -> u32 {
        self.accepted
    }

    fn push(&mut self, nvm: &mut Nvm<'_>, mut data: &[u8]) -> Result<(), ProgramError> {
        let accepted = data.len() as u32;
        if self.received() as usize + data.len() > fw::APP_SIZE as usize {
            return Err(ProgramError::TooLarge);
        }

        // Top the carried tail back up to a full doubleword first.
        if self.pending_len > 0 {
            let take = core::cmp::min(WRITE_SIZE - self.pending_len, data.len());
            self.pending[self.pending_len..self.pending_len + take].copy_from_slice(&data[..take]);
            self.pending_len += take;
            data = &data[take..];
            if self.pending_len == WRITE_SIZE {
                let word = self.pending;
                self.commit(nvm, &word)?;
                self.pending_len = 0;
            }
        }

        // Bulk of the chunk goes straight to flash.
        let aligned = data.len() - data.len() % WRITE_SIZE;
        if aligned > 0 {
            self.commit(nvm, &data[..aligned])?;
        }

        // Carry whatever does not fill a doubleword.
        let tail = &data[aligned..];
        self.pending[..tail.len()].copy_from_slice(tail);
        self.pending_len = tail.len();
        self.accepted += accepted;
        Ok(())
    }

    /// Flush the carried tail, padding to a full doubleword with the erased
    /// value so the padding is indistinguishable from untouched flash.
    fn finish(&mut self, nvm: &mut Nvm<'_>) -> Result<(), ProgramError> {
        if self.pending_len > 0 {
            let mut word = self.pending;
            word[self.pending_len..].fill(0xFF);
            self.pending_len = 0;
            self.commit(nvm, &word)?;
        }
        Ok(())
    }

    fn commit(&mut self, nvm: &mut Nvm<'_>, data: &[u8]) -> Result<(), ProgramError> {
        nvm.write(fw::APP_OFFSET + self.flushed, data)
            .map_err(|_| ProgramError::Flash)?;
        self.flushed += data.len() as u32;
        Ok(())
    }
}

#[derive(Clone, Copy, defmt::Format)]
enum ProgramError {
    TooLarge,
    Flash,
}

// ---------------------------------------------------------------------------
// Entry — boot decision before any peripheral is initialised
// ---------------------------------------------------------------------------

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    let (meta, slots_used) = fw::load_meta();

    if meta.update_requested {
        info!("update requested by the application, staying in the updater");
    } else if !meta.app_valid {
        info!("no valid application recorded, staying in the updater");
    } else if !fw::app_image_verifies(&meta) {
        // Should not happen — the image is verified before it is marked valid.
        // If it does, the slot decayed or was corrupted, and booting it would
        // be worse than sitting in the updater.
        error!(
            "application marked valid but failed verification (len={}), staying in the updater",
            meta.app_len
        );
    } else {
        info!(
            "booting application: {} bytes, crc=0x{:08x}, meta seq={}",
            meta.app_len, meta.app_crc, meta.seq
        );
        // SAFETY: the image has just been verified — its CRC matches and its
        // vector table points into its own slot with a plausible stack
        // pointer. Nothing has been initialised yet, so no interrupt can be
        // pending and the clock tree is still at its reset defaults.
        unsafe {
            (*SCB::PTR).vtor.write(fw::APP_BASE);
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
            cortex_m::asm::bootload(fw::APP_BASE as *const u32);
        }
    }

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(updater(spawner, meta, slots_used).expect("allocate the updater task"))
    })
}

// ---------------------------------------------------------------------------
// Updater
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn updater(spawner: Spawner, boot_meta: MetaRecord, slots_used: usize) -> ! {
    let p = embassy_stm32::init(fw::hal::clock_config());
    info!(
        "EMDM CANopen updater: node {}, app slot 0x{:08x}+{} B, metadata slot {}/{}",
        fw::NODE_ID,
        fw::APP_BASE,
        fw::APP_SIZE,
        slots_used,
        fw::META_SLOTS
    );

    let mut led = StatusLed::new(p.PB9, p.PC13, p.PA15);
    let mut nvm = Nvm::new(p.FLASH);

    // Clear the one-shot request immediately, exactly as the design calls for:
    // from here on an external reset boots the application again, so a stray
    // "enter update mode" cannot strand the node in the updater.
    let mut meta = boot_meta;
    if meta.update_requested {
        meta = meta.next(|r| r.update_requested = false);
        match fw::append_meta(&mut nvm, &meta) {
            Ok(()) => info!("cleared the update request (meta seq={})", meta.seq),
            // Not fatal: the updater still works, it just is not one-shot.
            Err(e) => warn!("could not clear the update request: {:?}", e),
        }
    }

    fw::hal::init_can(&spawner, p.FDCAN1, p.PA11, p.PA12);

    let node_id = NodeId::new(fw::NODE_ID).expect("node id in range");
    let mut od = BootOd::new();
    od.vendor_id = fw::VENDOR_ID;
    od.product_code = fw::PRODUCT_CODE_BOOTLOADER;
    od.serial_number = fw::hal::serial_from_uid();
    od.app_base = fw::APP_BASE;
    od.app_capacity = fw::APP_SIZE;
    od.flash_status = flash_status::IDLE;
    od.device_name
        .push_str("EMDM CANopen updater")
        .expect("device name fits");

    let node: BootOdNode = Node::new(
        NodeConfig {
            heartbeat_interval_ms: 500,
            auto_start: true,
            ..NodeConfig::from_od(&od, node_id)
        },
        od,
    );
    NODE.init(node);

    spawner.spawn(protocol_task().expect("allocate the protocol task"));
    info!("updater ready on CAN, waiting for a host");

    let mut programmer = Programmer::new();
    let mut blink = Ticker::every(Duration::from_millis(250));
    let mut phase = false;

    loop {
        match select(NODE.wait_for_change(), blink.next()).await {
            Either::First(()) => {
                handle_changes(&mut nvm, &mut programmer, &mut meta);
            }
            Either::Second(()) => {
                phase = !phase;
                let status = NODE.with(|n| n.od().flash_status);
                match status {
                    // Blue heartbeat: idle, nothing programmed yet.
                    flash_status::IDLE => led.set(false, false, phase),
                    // Violet flicker while chunks are landing.
                    flash_status::PROGRAMMING => led.set(phase, false, true),
                    // Green: a bootable image is in the slot.
                    flash_status::VALID => led.set(false, true, false),
                    _ => led.set(true, false, false),
                }
            }
        }
    }
}

/// Drain the object dictionary changes the SDO server produced.
///
/// Flash work happens here rather than inside the SDO write itself, so the
/// host's transfer is acknowledged before an erase stalls the core for a
/// second. Failures surface asynchronously through 0x1F57:1.
fn handle_changes(nvm: &mut Nvm<'_>, programmer: &mut Programmer, meta: &mut MetaRecord) {
    while let Some(change) = NODE.with(|n| n.next_change()) {
        match change {
            BootOdChange::FirmwareChunk => {
                // Copy the chunk out of the OD before touching flash.
                // `SharedNode::with` runs its closure inside a critical
                // section, and programming 256 bytes takes about 2.5 ms (a
                // full 889-byte chunk, about 9) — far too long to hold
                // interrupts off, which would stall the CAN driver and the
                // time driver for the whole download.
                let mut chunk = [0u8; fw::MAX_CHUNK];
                let len = NODE.with(|n| {
                    let data = n.od().firmware_chunk.as_slice();
                    chunk[..data.len()].copy_from_slice(data);
                    data.len()
                });
                match programmer.push(nvm, &chunk[..len]) {
                    Ok(()) => {
                        NODE.with(|n| n.od_mut().bytes_written = programmer.received());
                        defmt::trace!("chunk {} B, {} B total", len, programmer.received());
                    }
                    Err(e) => {
                        error!("programming failed: {:?}", e);
                        fail(nvm, meta);
                    }
                }
            }

            BootOdChange::Command(cmd) => match cmd {
                program_control::CLEAR => clear(nvm, programmer, meta),
                program_control::VALIDATE => validate(nvm, programmer, meta),
                program_control::RESET => {
                    info!("resetting on host request");
                    SCB::sys_reset();
                }
                // `check_write` rejects everything else.
                other => warn!("ignoring program control 0x{:02x}", other),
            },

            // Declared length and expected CRC are just stored; they are read
            // back when "clear" and "validate" run.
            BootOdChange::ImageLength(len) => info!("host declared image length {} B", len),
            BootOdChange::ExpectedCrc(crc) => info!("host declared image crc 0x{:08x}", crc),
        }
    }
}

/// Invalidate the application, then erase exactly as many pages as the
/// declared image needs.
///
/// The metadata write comes first and is what makes this safe: from the moment
/// the first page is erased until "validate" succeeds, a reset lands back in
/// the updater rather than in a half-written image.
fn clear(nvm: &mut Nvm<'_>, programmer: &mut Programmer, meta: &mut MetaRecord) {
    let declared = NODE.with(|n| n.od().image_length);

    let invalidated = meta.next(|r| {
        r.app_valid = false;
        r.app_len = 0;
        r.app_crc = 0;
    });
    if let Err(e) = fw::append_meta(nvm, &invalidated) {
        error!("could not invalidate the application: {:?}", e);
        return fail(nvm, meta);
    }
    *meta = invalidated;

    let pages = declared.div_ceil(fw::PAGE_SIZE);
    let end = fw::APP_OFFSET + pages * fw::PAGE_SIZE;
    info!(
        "erasing {} page(s) for a {} B image (0x{:08x}..0x{:08x})",
        pages,
        declared,
        fw::FLASH_BASE + fw::APP_OFFSET,
        fw::FLASH_BASE + end
    );

    // ~24 ms per page: a 78 KiB image stalls the core for about a second.
    // Nothing here is time-critical, and the CAN driver's FIFO absorbs the
    // host's next request, but this is why the erase is sized to the image.
    if let Err(e) = nvm.erase(fw::APP_OFFSET, end) {
        error!("erase failed: {:?}", e);
        return fail(nvm, meta);
    }

    programmer.reset();
    NODE.with(|n| {
        n.od_mut().bytes_written = 0;
        n.od_mut().flash_status = flash_status::PROGRAMMING;
    });
    info!("application slot erased, ready for chunks");
}

/// Verify what is actually in flash and, only then, mark it bootable.
///
/// The CRC is taken over the memory-mapped slot rather than over the bytes as
/// they were received, so a write that failed silently is caught here instead
/// of at the next boot.
fn validate(nvm: &mut Nvm<'_>, programmer: &mut Programmer, meta: &mut MetaRecord) {
    if let Err(e) = programmer.finish(nvm) {
        error!("could not flush the final doubleword: {:?}", e);
        return fail(nvm, meta);
    }

    let (declared, expected_crc) = NODE.with(|n| (n.od().image_length, n.od().expected_crc));
    let received = programmer.received();

    if received != declared {
        error!(
            "incomplete image: {} B received, {} B declared",
            received, declared
        );
        return fail(nvm, meta);
    }

    let candidate = meta.next(|r| {
        r.app_valid = true;
        r.app_len = declared;
        r.app_crc = expected_crc;
    });

    let actual = match fw::app_image(declared) {
        Some(image) => fw::CRC32.checksum(image),
        None => return fail(nvm, meta),
    };
    if actual != expected_crc {
        error!(
            "crc mismatch: flash has 0x{:08x}, host declared 0x{:08x}",
            actual, expected_crc
        );
        return fail(nvm, meta);
    }
    if !fw::app_vector_table_plausible(declared) {
        error!("image has no plausible vector table");
        return fail(nvm, meta);
    }

    if let Err(e) = fw::append_meta(nvm, &candidate) {
        error!("could not mark the application valid: {:?}", e);
        return fail(nvm, meta);
    }
    *meta = candidate;

    NODE.with(|n| n.od_mut().flash_status = flash_status::VALID);
    info!(
        "image accepted: {} B, crc=0x{:08x}, meta seq={} — reset to run it",
        declared, expected_crc, candidate.seq
    );
}

/// Put the node into the error state and make sure the slot stays marked
/// invalid, so a reset cannot boot a half-written image.
fn fail(nvm: &mut Nvm<'_>, meta: &mut MetaRecord) {
    if meta.app_valid {
        let invalidated = meta.next(|r| {
            r.app_valid = false;
            r.app_len = 0;
            r.app_crc = 0;
        });
        if fw::append_meta(nvm, &invalidated).is_ok() {
            *meta = invalidated;
        }
    }
    NODE.with(|n| {
        n.od_mut().flash_status = flash_status::ERROR;
        n.set_error(0x5000, canopen_core::error_register::GENERIC, &[]);
    });
}

// ---------------------------------------------------------------------------
// Plumbing
// ---------------------------------------------------------------------------

static NODE: canopen_core::node::SharedNode<
    BootOd,
    { BootOd::TPDO_COUNT },
    { BootOd::RPDO_COUNT },
> = canopen_core::node::SharedNode::new();

#[embassy_executor::task]
async fn protocol_task() -> ! {
    let mut transport = ChannelTransport;
    let clock = EmbassyClock;
    loop {
        NODE.with(|node| node.process(&mut transport, &clock));
        Timer::after(Duration::from_millis(1)).await;
    }
}
