//! Flash layout and power-fail-safe metadata shared by the F767 updater demo.

#![no_std]

use crc::{Crc, CRC_32_ISO_HDLC};

#[cfg(feature = "hal")]
pub mod hal;
#[cfg(feature = "hal")]
pub mod ws;

pub const CRC32: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

// STM32F767ZI in its factory/default single-bank mode. The first four erase
// sectors are 32 KiB, followed by one 128 KiB sector and seven 256 KiB
// sectors. Keeping metadata in the fourth small sector avoids spending a
// 256 KiB sector on rarely-written configuration.
pub const FLASH_BASE: u32 = 0x0800_0000;
pub const FLASH_SIZE: u32 = 2 * 1024 * 1024;
pub const BOOTLOADER_OFFSET: u32 = 0;
pub const BOOTLOADER_SIZE: u32 = 96 * 1024;
pub const CONFIG_OFFSET: u32 = BOOTLOADER_OFFSET + BOOTLOADER_SIZE;
pub const CONFIG_SIZE: u32 = 32 * 1024;
pub const FIRMWARE_OFFSET: u32 = CONFIG_OFFSET + CONFIG_SIZE;
pub const FIRMWARE_SIZE: u32 = 384 * 1024;
pub const WEB_OFFSET: u32 = FIRMWARE_OFFSET + FIRMWARE_SIZE;
pub const WEB_SIZE: u32 = 1536 * 1024;

pub const FIRMWARE_BASE: u32 = FLASH_BASE + FIRMWARE_OFFSET;
pub const WEB_BASE: u32 = FLASH_BASE + WEB_OFFSET;
pub const CONFIG_BASE: u32 = FLASH_BASE + CONFIG_OFFSET;

pub const RAM_BASE: u32 = 0x2000_0000;
pub const RAM_SIZE: u32 = 512 * 1024;

const _: () = assert!(BOOTLOADER_SIZE + CONFIG_SIZE + FIRMWARE_SIZE + WEB_SIZE == FLASH_SIZE);
const _: () = assert!(FIRMWARE_BASE % 512 == 0);
const _: () = assert!(WEB_OFFSET + WEB_SIZE == FLASH_SIZE);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ImageTarget {
    Firmware = 1,
    Web = 2,
}

impl ImageTarget {
    pub const fn from_raw(value: u8) -> Option<Self> {
        match value {
            1 => Some(Self::Firmware),
            2 => Some(Self::Web),
            _ => None,
        }
    }

    pub const fn offset(self) -> u32 {
        match self {
            Self::Firmware => FIRMWARE_OFFSET,
            Self::Web => WEB_OFFSET,
        }
    }

    pub const fn base(self) -> u32 {
        FLASH_BASE + self.offset()
    }

    pub const fn capacity(self) -> u32 {
        match self {
            Self::Firmware => FIRMWARE_SIZE,
            Self::Web => WEB_SIZE,
        }
    }
}

pub const META_MAGIC: u32 = 0x3755_4446; // "FDU7"
pub const META_FORMAT: u16 = 1;
pub const META_RECORD_SIZE: usize = 64;
pub const META_SLOTS: usize = CONFIG_SIZE as usize / META_RECORD_SIZE;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MetaRecord {
    pub seq: u32,
    pub firmware_len: u32,
    pub firmware_crc: u32,
    pub web_len: u32,
    pub web_crc: u32,
    pub firmware_valid: bool,
    pub web_valid: bool,
    pub update_requested: bool,
}

impl MetaRecord {
    pub const BLANK: Self = Self {
        seq: 0,
        firmware_len: 0,
        firmware_crc: 0,
        web_len: 0,
        web_crc: 0,
        firmware_valid: false,
        web_valid: false,
        update_requested: false,
    };

    pub fn encode(self) -> [u8; META_RECORD_SIZE] {
        let mut out = [0u8; META_RECORD_SIZE];
        out[0..4].copy_from_slice(&META_MAGIC.to_le_bytes());
        out[4..6].copy_from_slice(&META_FORMAT.to_le_bytes());
        out[8..12].copy_from_slice(&self.seq.to_le_bytes());
        out[12..16].copy_from_slice(&self.firmware_len.to_le_bytes());
        out[16..20].copy_from_slice(&self.firmware_crc.to_le_bytes());
        out[20..24].copy_from_slice(&self.web_len.to_le_bytes());
        out[24..28].copy_from_slice(&self.web_crc.to_le_bytes());
        out[28] = self.firmware_valid as u8;
        out[29] = self.web_valid as u8;
        out[30] = self.update_requested as u8;
        let crc = CRC32.checksum(&out[..60]);
        out[60..64].copy_from_slice(&crc.to_le_bytes());
        out
    }

    pub fn decode(raw: &[u8]) -> Option<Self> {
        if raw.len() != META_RECORD_SIZE
            || u32::from_le_bytes(raw[0..4].try_into().ok()?) != META_MAGIC
            || u16::from_le_bytes(raw[4..6].try_into().ok()?) != META_FORMAT
            || u32::from_le_bytes(raw[60..64].try_into().ok()?) != CRC32.checksum(&raw[..60])
        {
            return None;
        }
        Some(Self {
            seq: u32::from_le_bytes(raw[8..12].try_into().ok()?),
            firmware_len: u32::from_le_bytes(raw[12..16].try_into().ok()?),
            firmware_crc: u32::from_le_bytes(raw[16..20].try_into().ok()?),
            web_len: u32::from_le_bytes(raw[20..24].try_into().ok()?),
            web_crc: u32::from_le_bytes(raw[24..28].try_into().ok()?),
            firmware_valid: raw[28] == 1,
            web_valid: raw[29] == 1,
            update_requested: raw[30] == 1,
        })
    }

    pub fn next(self, f: impl FnOnce(&mut Self)) -> Self {
        let mut next = self;
        next.seq = self.seq.wrapping_add(1);
        f(&mut next);
        next
    }

    pub const fn image_len(self, target: ImageTarget) -> u32 {
        match target {
            ImageTarget::Firmware => self.firmware_len,
            ImageTarget::Web => self.web_len,
        }
    }

    pub const fn image_crc(self, target: ImageTarget) -> u32 {
        match target {
            ImageTarget::Firmware => self.firmware_crc,
            ImageTarget::Web => self.web_crc,
        }
    }

    pub const fn image_valid(self, target: ImageTarget) -> bool {
        match target {
            ImageTarget::Firmware => self.firmware_valid,
            ImageTarget::Web => self.web_valid,
        }
    }

    pub fn set_image(&mut self, target: ImageTarget, len: u32, crc: u32, valid: bool) {
        match target {
            ImageTarget::Firmware => {
                self.firmware_len = len;
                self.firmware_crc = crc;
                self.firmware_valid = valid;
            }
            ImageTarget::Web => {
                self.web_len = len;
                self.web_crc = crc;
                self.web_valid = valid;
            }
        }
    }
}

fn config_bytes() -> &'static [u8] {
    unsafe { core::slice::from_raw_parts(CONFIG_BASE as *const u8, CONFIG_SIZE as usize) }
}

pub fn scan_meta(page: &[u8]) -> (MetaRecord, usize) {
    let mut newest = None;
    let mut first_free = META_SLOTS;
    for (slot, raw) in page.chunks_exact(META_RECORD_SIZE).enumerate() {
        if first_free == META_SLOTS && raw.iter().all(|byte| *byte == 0xFF) {
            first_free = slot;
        }
        if let Some(candidate) = MetaRecord::decode(raw) {
            if newest
                .map(|old: MetaRecord| {
                    candidate.seq != old.seq && candidate.seq.wrapping_sub(old.seq) < (1 << 31)
                })
                .unwrap_or(true)
            {
                newest = Some(candidate);
            }
        }
    }
    (newest.unwrap_or(MetaRecord::BLANK), first_free)
}

pub fn load_meta() -> (MetaRecord, usize) {
    scan_meta(config_bytes())
}

pub trait FlashOps {
    type Error;
    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error>;
    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error>;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AppendMetaError<E> {
    Full,
    Flash(E),
}

pub fn append_meta<F: FlashOps>(
    flash: &mut F,
    record: MetaRecord,
) -> Result<(), AppendMetaError<F::Error>> {
    let (_, slot) = load_meta();
    if slot >= META_SLOTS {
        return Err(AppendMetaError::Full);
    }
    flash
        .write(
            CONFIG_OFFSET + (slot * META_RECORD_SIZE) as u32,
            &record.encode(),
        )
        .map_err(AppendMetaError::Flash)
}

pub fn image(target: ImageTarget, len: u32) -> Option<&'static [u8]> {
    if len == 0 || len > target.capacity() {
        return None;
    }
    Some(unsafe { core::slice::from_raw_parts(target.base() as *const u8, len as usize) })
}

pub fn firmware_vector_table_plausible(len: u32) -> bool {
    let Some(image) = image(ImageTarget::Firmware, len) else {
        return false;
    };
    if image.len() < 8 {
        return false;
    }
    let sp = u32::from_le_bytes(image[0..4].try_into().unwrap());
    let reset = u32::from_le_bytes(image[4..8].try_into().unwrap());
    sp > RAM_BASE
        && sp <= RAM_BASE + RAM_SIZE
        && sp % 8 == 0
        && reset & 1 == 1
        && reset > FIRMWARE_BASE
        && reset < FIRMWARE_BASE + len
}

pub fn firmware_verifies(meta: MetaRecord) -> bool {
    meta.firmware_valid
        && firmware_vector_table_plausible(meta.firmware_len)
        && image(ImageTarget::Firmware, meta.firmware_len)
            .is_some_and(|bytes| CRC32.checksum(bytes) == meta.firmware_crc)
}

pub const NODE_ID: u8 = 42;
pub const VENDOR_ID: u32 = 0x0000_053A;
pub const PRODUCT_CODE_APP: u32 = 0x4444_5541; // "DDUA"
pub const PRODUCT_CODE_BOOTLOADER: u32 = 0x4444_5542; // "DDUB"
pub const ENTER_UPDATE_MAGIC: u32 = 0x5450_4455; // "UPDT"
pub const MAX_CHUNK: usize = 889;
pub const HTTP_PORT: u16 = 8080;
pub const NETWORK_NAME: &str = "update";

pub mod flash_status {
    pub const IDLE: u32 = 0;
    pub const PROGRAMMING: u32 = 1;
    pub const VALID: u32 = 2;
    pub const ERROR: u32 = 0xFF;
}

pub mod program_control {
    pub const VALIDATE: u8 = 1;
    pub const RESET: u8 = 2;
    pub const CLEAR: u8 = 3;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layout_is_complete_and_sector_aligned() {
        assert_eq!(FIRMWARE_BASE, 0x0802_0000);
        assert_eq!(WEB_BASE, 0x0808_0000);
        assert_eq!(CONFIG_BASE, 0x0801_8000);
        assert_eq!(META_SLOTS, 512);
    }

    #[test]
    fn metadata_round_trips_both_targets() {
        let record = MetaRecord::BLANK.next(|record| {
            record.set_image(ImageTarget::Firmware, 123, 456, true);
            record.set_image(ImageTarget::Web, 789, 1011, true);
            record.update_requested = true;
        });
        assert_eq!(MetaRecord::decode(&record.encode()), Some(record));
    }

    #[test]
    fn torn_record_is_ignored() {
        let mut record = MetaRecord::BLANK.encode();
        record[17] ^= 1;
        assert_eq!(MetaRecord::decode(&record), None);
    }

    #[test]
    fn scan_skips_garbage_and_finds_erased_slot() {
        let mut page = [0xFF; META_RECORD_SIZE * 3];
        page[..META_RECORD_SIZE].fill(0x55);
        let valid = MetaRecord::BLANK.next(|record| record.web_valid = true);
        page[META_RECORD_SIZE..META_RECORD_SIZE * 2].copy_from_slice(&valid.encode());
        let (found, free) = scan_meta(&page);
        assert_eq!(found, valid);
        assert_eq!(free, 2);
    }
}
