//! Shared flash layout and update metadata for the EMDM CANopen update PoC.
//!
//! The bootloader and the application both link this crate, so the flash map
//! and the metadata record format cannot drift apart between the two images.
//! Drift there is exactly the bug class that bricks a device in the field.

#![no_std]

use crc::{Crc, CRC_32_ISO_HDLC};

#[cfg(feature = "hal")]
pub mod hal;

/// CRC used for both the application image and the metadata records.
pub const CRC32: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

// ---------------------------------------------------------------------------
// Flash map — STM32G431CB, 128 KiB of flash in one bank, 2 KiB pages.
// ---------------------------------------------------------------------------

pub const FLASH_BASE: u32 = 0x0800_0000;
pub const FLASH_SIZE: u32 = 128 * 1024;
pub const PAGE_SIZE: u32 = 2 * 1024;

/// The bootloader is never rewritten over CAN. It is the immutable root that
/// makes an interrupted update recoverable.
pub const BOOTLOADER_OFFSET: u32 = 0;
pub const BOOTLOADER_SIZE: u32 = 56 * 1024;

/// One page holding an append-only log of [`MetaRecord`]s.
pub const META_OFFSET: u32 = BOOTLOADER_OFFSET + BOOTLOADER_SIZE;
pub const META_SIZE: u32 = PAGE_SIZE;

/// Everything after the metadata page is the application slot.
pub const APP_OFFSET: u32 = META_OFFSET + META_SIZE;
pub const APP_SIZE: u32 = FLASH_SIZE - APP_OFFSET;

/// Absolute address the application is linked at, and where its vector table
/// lives. VTOR requires the table to be aligned to the next power of two at or
/// above its byte size; this part has 118 entries (472 bytes), so 512.
pub const APP_BASE: u32 = FLASH_BASE + APP_OFFSET;

const _: () = assert!(APP_OFFSET % PAGE_SIZE == 0);
const _: () = assert!(META_OFFSET % PAGE_SIZE == 0);
const _: () = assert!(APP_BASE % 512 == 0);
const _: () = assert!(APP_OFFSET + APP_SIZE == FLASH_SIZE);

/// SRAM window used to sanity-check an image's initial stack pointer.
pub const RAM_BASE: u32 = 0x2000_0000;
pub const RAM_SIZE: u32 = 32 * 1024;

// ---------------------------------------------------------------------------
// Update metadata
// ---------------------------------------------------------------------------

/// `"EMDM"` little-endian.
pub const META_MAGIC: u32 = 0x4D44_4D45;

/// 32 bytes: a multiple of the STM32G4 64-bit write granularity, and small
/// enough that one page holds 64 generations before it has to be erased.
pub const META_RECORD_SIZE: usize = 32;
pub const META_SLOTS: usize = META_SIZE as usize / META_RECORD_SIZE;

/// One generation of update state.
///
/// Records are appended, never modified: the newest valid record wins, with
/// wrapping sequence-number ordering. A record is only considered at all once
/// its own CRC checks out, so a write torn by a power loss is ignored rather
/// than misread.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MetaRecord {
    pub seq: u32,
    /// Length of the application image in bytes.
    pub app_len: u32,
    /// CRC-32 (ISO-HDLC) over exactly `app_len` bytes at [`APP_BASE`].
    pub app_crc: u32,
    /// The application slot holds a complete, verified image.
    pub app_valid: bool,
    /// One-shot request from the application: stay in the updater after the
    /// next reset. The bootloader clears this before doing anything else.
    pub update_requested: bool,
}

impl MetaRecord {
    /// The state a device is in before it has ever been programmed.
    pub const BLANK: Self = Self {
        seq: 0,
        app_len: 0,
        app_crc: 0,
        app_valid: false,
        update_requested: false,
    };

    pub fn encode(&self) -> [u8; META_RECORD_SIZE] {
        let mut out = [0u8; META_RECORD_SIZE];
        out[0..4].copy_from_slice(&META_MAGIC.to_le_bytes());
        out[4..8].copy_from_slice(&self.seq.to_le_bytes());
        out[8..12].copy_from_slice(&self.app_len.to_le_bytes());
        out[12..16].copy_from_slice(&self.app_crc.to_le_bytes());
        out[16] = self.app_valid as u8;
        out[17] = self.update_requested as u8;
        // 18..28 stays zero: reserved.
        let crc = CRC32.checksum(&out[..28]);
        out[28..32].copy_from_slice(&crc.to_le_bytes());
        out
    }

    pub fn decode(raw: &[u8]) -> Option<Self> {
        if raw.len() != META_RECORD_SIZE {
            return None;
        }
        if u32::from_le_bytes(raw[0..4].try_into().ok()?) != META_MAGIC {
            return None;
        }
        let stored = u32::from_le_bytes(raw[28..32].try_into().ok()?);
        if CRC32.checksum(&raw[..28]) != stored {
            return None;
        }
        Some(Self {
            seq: u32::from_le_bytes(raw[4..8].try_into().ok()?),
            app_len: u32::from_le_bytes(raw[8..12].try_into().ok()?),
            app_crc: u32::from_le_bytes(raw[12..16].try_into().ok()?),
            app_valid: raw[16] != 0,
            update_requested: raw[17] != 0,
        })
    }

    /// Next generation of this record, with the mutations applied by `f`.
    pub fn next(&self, f: impl FnOnce(&mut Self)) -> Self {
        let mut out = *self;
        out.seq = self.seq.wrapping_add(1);
        f(&mut out);
        out
    }
}

/// Read the metadata page straight out of the memory-mapped flash window.
///
/// This deliberately touches no peripheral, so the bootloader can call it
/// before it has configured anything at all.
pub fn load_meta() -> (MetaRecord, usize) {
    scan_meta(meta_page())
}

/// Pure form of [`load_meta`]: the record in force, and the slot the next
/// record may be written into (`META_SLOTS` if the page is full).
///
/// The free slot is the first one that is **entirely erased**, not the one
/// after the last valid record. Those differ whenever the page holds bytes
/// that are neither 0xFF nor a valid record — which is exactly the state a
/// board is in when this bootloader is installed over some other firmware that
/// happened to span this page. Programming a non-erased doubleword fails on
/// STM32G4, so choosing the slot by counting records would fail on first use
/// and leave the node unable to record anything.
fn scan_meta(page: &[u8]) -> (MetaRecord, usize) {
    let mut best: Option<MetaRecord> = None;
    let mut next_free = META_SLOTS;
    for (slot, raw) in page.chunks_exact(META_RECORD_SIZE).enumerate() {
        if next_free == META_SLOTS && raw.iter().all(|&b| b == 0xFF) {
            next_free = slot;
        }
        if let Some(rec) = MetaRecord::decode(raw) {
            if best.is_none_or(|b| rec.seq != b.seq && rec.seq.wrapping_sub(b.seq) < (1 << 31)) {
                best = Some(rec);
            }
        }
    }
    (best.unwrap_or(MetaRecord::BLANK), next_free)
}

/// The subset of flash operations this crate needs.
///
/// Keeping it a trait is what lets the metadata log be unit-tested on the host
/// against a plain byte array, instead of only on the target.
pub trait FlashOps {
    type Error;
    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error>;
    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error>;
}

/// Append the next generation of update state to the metadata log.
///
/// The common case is a single 32-byte write into an erased slot, which is
/// atomic enough for our purposes: a record torn by a power loss fails its own
/// CRC and is skipped, leaving the previous generation in force.
///
/// Only when no erased slot is left does this erase the page and start over.
/// That is the one window where a power loss loses the metadata entirely — and
/// it fails safe, because a device with no valid record stays in the updater.
/// It is also what recovers a page left full of some previous firmware's code.
pub fn append_meta<F: FlashOps>(flash: &mut F, rec: &MetaRecord) -> Result<(), F::Error> {
    let (_, next_free) = load_meta();
    let slot = if next_free >= META_SLOTS {
        flash.erase(META_OFFSET, META_OFFSET + META_SIZE)?;
        0
    } else {
        next_free
    };
    flash.write(
        META_OFFSET + (slot * META_RECORD_SIZE) as u32,
        &rec.encode(),
    )
}

fn meta_page() -> &'static [u8] {
    // SAFETY: the metadata page is inside the device's flash window and is
    // excluded from every linker script in this workspace, so nothing else
    // owns it. Reads of erased flash return 0xFF and are well-defined.
    unsafe {
        core::slice::from_raw_parts(
            (FLASH_BASE + META_OFFSET) as *const u8,
            META_SIZE as usize,
        )
    }
}

// ---------------------------------------------------------------------------
// Application image checks
// ---------------------------------------------------------------------------

/// Borrow `len` bytes of the application slot from memory-mapped flash.
pub fn app_image(len: u32) -> Option<&'static [u8]> {
    if len == 0 || len > APP_SIZE {
        return None;
    }
    // SAFETY: bounds-checked against the application slot, which no linker
    // script in this workspace places anything else into.
    Some(unsafe { core::slice::from_raw_parts(APP_BASE as *const u8, len as usize) })
}

/// A programmed image must at minimum carry a vector table that could
/// plausibly be executed: a stack pointer inside SRAM and a reset vector
/// inside its own slot with the Thumb bit set.
///
/// This is cheap insurance against jumping into erased flash. It is not a
/// substitute for the CRC, which is what actually proves the image is intact.
pub fn app_vector_table_plausible(len: u32) -> bool {
    let Some(image) = app_image(len) else {
        return false;
    };
    if image.len() < 8 {
        return false;
    }
    let sp = u32::from_le_bytes(image[0..4].try_into().unwrap());
    let reset = u32::from_le_bytes(image[4..8].try_into().unwrap());

    let sp_ok = sp > RAM_BASE && sp <= RAM_BASE + RAM_SIZE && sp % 8 == 0;
    let reset_ok =
        reset & 1 == 1 && reset > APP_BASE && reset < APP_BASE + len;
    sp_ok && reset_ok
}

/// Full verification of the programmed slot against a record.
pub fn app_image_verifies(rec: &MetaRecord) -> bool {
    if !app_vector_table_plausible(rec.app_len) {
        return false;
    }
    match app_image(rec.app_len) {
        Some(image) => CRC32.checksum(image) == rec.app_crc,
        None => false,
    }
}

// ---------------------------------------------------------------------------
// CANopen interface constants shared by both images
// ---------------------------------------------------------------------------

pub const NODE_ID: u8 = 42;
pub const CAN_BITRATE: u32 = 500_000;

pub const VENDOR_ID: u32 = 0x0000_053A;
/// `"EMDA"` — the demo application.
pub const PRODUCT_CODE_APP: u32 = 0x454D_4441;
/// `"EMDB"` — the bootloader/updater.
pub const PRODUCT_CODE_BOOTLOADER: u32 = 0x454D_4442;

/// Magic the application requires at object 0x2F00:0 before it will reboot
/// into the updater, so a stray write cannot take the node offline.
pub const ENTER_UPDATE_MAGIC: u32 = 0x5450_4455; // "UPDT" little-endian

/// Largest single SDO transfer the server can buffer, and therefore the
/// largest firmware chunk the host may push in one write to 0x1F50:1. It also
/// sizes the buffer the updater stages a chunk in before programming it.
///
/// The host currently caps its own transfers well below this — see "Known
/// limitation: 256-byte chunks".
pub const MAX_CHUNK: usize = 889;

/// Values of 0x1F57:1, "flash status identification".
pub mod flash_status {
    /// Nothing programmed since reset; the slot has not been erased.
    pub const IDLE: u32 = 0;
    /// The slot is erased and accepting chunks.
    pub const PROGRAMMING: u32 = 1;
    /// The slot holds a verified image; the node may be reset into it.
    pub const VALID: u32 = 2;
    /// A step failed. The host must re-issue "clear" to start over.
    pub const ERROR: u32 = 0xFF;
}

/// Commands accepted at 0x1F51:1, "program control".
pub mod program_control {
    /// Erase the application slot and begin a new download.
    pub const CLEAR: u8 = 3;
    /// Verify what was programmed and mark it bootable.
    pub const VALIDATE: u8 = 1;
    /// Reset the MCU.
    pub const RESET: u8 = 2;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn record_round_trips() {
        let rec = MetaRecord {
            seq: 7,
            app_len: 1234,
            app_crc: 0xDEAD_BEEF,
            app_valid: true,
            update_requested: false,
        };
        assert_eq!(MetaRecord::decode(&rec.encode()), Some(rec));
    }

    #[test]
    fn erased_flash_is_not_a_record() {
        assert_eq!(MetaRecord::decode(&[0xFF; META_RECORD_SIZE]), None);
    }

    #[test]
    fn corrupted_record_is_rejected() {
        let mut raw = MetaRecord::BLANK.encode();
        raw[10] ^= 0x01;
        assert_eq!(MetaRecord::decode(&raw), None);
    }

    #[test]
    fn next_bumps_sequence() {
        let a = MetaRecord::BLANK;
        let b = a.next(|r| r.app_valid = true);
        assert_eq!(b.seq, 1);
        assert!(b.app_valid);
    }

    fn page_with(records: &[MetaRecord]) -> [u8; META_SIZE as usize] {
        let mut page = [0xFFu8; META_SIZE as usize];
        for (slot, rec) in records.iter().enumerate() {
            let at = slot * META_RECORD_SIZE;
            page[at..at + META_RECORD_SIZE].copy_from_slice(&rec.encode());
        }
        page
    }

    #[test]
    fn newest_record_wins_and_next_slot_follows_it() {
        let a = MetaRecord::BLANK.next(|r| r.app_valid = true);
        let b = a.next(|r| r.app_len = 99);
        let (found, next_free) = scan_meta(&page_with(&[a, b]));
        assert_eq!(found, b);
        assert_eq!(next_free, 2);
    }

    #[test]
    fn sequence_order_survives_wraparound() {
        let before_wrap = MetaRecord {
            seq: u32::MAX,
            ..MetaRecord::BLANK
        };
        let after_wrap = before_wrap.next(|r| r.app_len = 99);
        let (found, next_free) = scan_meta(&page_with(&[before_wrap, after_wrap]));
        assert_eq!(found, after_wrap);
        assert_eq!(next_free, 2);
    }

    #[test]
    fn erased_page_starts_at_slot_zero() {
        let (found, next_free) = scan_meta(&[0xFF; META_SIZE as usize]);
        assert_eq!(found, MetaRecord::BLANK);
        assert_eq!(next_free, 0);
    }

    /// A page carrying some other firmware's code is not erased, so no slot in
    /// it may be programmed. Reporting the page as full is what makes
    /// `append_meta` erase it instead of failing to program a used doubleword.
    #[test]
    fn page_full_of_foreign_data_reports_no_free_slot() {
        let (found, next_free) = scan_meta(&[0x5A; META_SIZE as usize]);
        assert_eq!(found, MetaRecord::BLANK);
        assert_eq!(next_free, META_SLOTS);
    }

    /// Garbage in early slots must not be mistaken for space to write into,
    /// but a genuinely erased slot after it is still usable.
    #[test]
    fn free_slot_skips_leading_garbage() {
        let mut page = [0xFFu8; META_SIZE as usize];
        page[..META_RECORD_SIZE].fill(0x5A);
        let (_, next_free) = scan_meta(&page);
        assert_eq!(next_free, 1);
    }

    #[test]
    fn layout_matches_the_device() {
        assert_eq!(APP_BASE, 0x0800_E800);
        assert_eq!(APP_SIZE, 70 * 1024);
        assert_eq!(META_SLOTS, 64);
    }
}
