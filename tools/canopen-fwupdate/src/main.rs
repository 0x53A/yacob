//! Host side of the CANopen firmware update PoC.
//!
//! Drives the CiA 302 programming objects exposed by
//! `examples/emdm-fwupdate/bootloader` over SDO on a socketcan interface.
//!
//! ```sh
//! canopen-fwupdate --iface can0 --node 42 status
//! canopen-fwupdate --iface can0 --node 42 flash app.bin
//! ```

use std::path::PathBuf;
use std::time::{Duration, Instant};

use canopen_core::cobid::NodeId;
use canopen_linux::sdo_helpers::{sdo_download, sdo_upload};
use canopen_linux::SocketcanTransport;
use clap::{Parser, Subcommand};
use crc::{Crc, CRC_32_ISO_HDLC};

const CRC32: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

/// Bytes per chunk.
///
/// The node's SDO server and its 0x1F50:1 domain both take up to 889 bytes,
/// but `SdoDriver::download` builds its client with a fixed 256-byte buffer,
/// so that is the real ceiling until the driver grows a size parameter.
const MAX_CHUNK: usize = 256;

const VENDOR_ID: u32 = 0x0000_053A;
const PRODUCT_CODE_APP: u32 = 0x454D_4441;
const PRODUCT_CODE_BOOTLOADER: u32 = 0x454D_4442;
const ENTER_UPDATE_MAGIC: u32 = 0x5450_4455;

// Object dictionary addresses.
const IDENTITY: u16 = 0x1018;
const PROGRAM_DATA: u16 = 0x1F50;
const PROGRAM_CONTROL: u16 = 0x1F51;
const PROGRAM_SW_ID: u16 = 0x1F56;
const FLASH_STATUS: u16 = 0x1F57;
const UPDATE_INFO: u16 = 0x2000;
const ENTER_UPDATE: u16 = 0x2F00;

const CMD_VALIDATE: u8 = 1;
const CMD_RESET: u8 = 2;
const CMD_CLEAR: u8 = 3;

const ST_IDLE: u32 = 0;
const ST_PROGRAMMING: u32 = 1;
const ST_VALID: u32 = 2;
const ST_ERROR: u32 = 0xFF;

/// Ordinary request/response. The node answers in well under a millisecond.
const T_SHORT: Duration = Duration::from_millis(500);
/// Writes whose handler blocks the node's executor on flash: a chunk stalls it
/// for about 9 ms, a full-slot erase for the better part of a second.
const T_FLASH: Duration = Duration::from_secs(5);

#[derive(Parser)]
#[command(about = "CiA 302 firmware download over CANopen SDO", version)]
struct Cli {
    /// SocketCAN interface.
    #[arg(long, default_value = "can0")]
    iface: String,

    /// Target node ID.
    #[arg(long, default_value_t = 42)]
    node: u8,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Report which image is running and, in the updater, the flash state.
    Status,
    /// Ask a running application to reboot into the updater.
    EnterUpdate,
    /// Reset the node.
    Reset,
    /// Download an image and, unless told otherwise, boot it.
    Flash {
        /// Raw binary, as produced by `arm-none-eabi-objcopy -O binary`.
        image: PathBuf,
        /// Verify and mark bootable, but leave the node in the updater.
        #[arg(long)]
        no_reset: bool,
        /// Fault injection: corrupt one byte *after* computing the CRC the
        /// node is told to expect, so that what lands in flash cannot match.
        /// Used to check that the node's read-back verification actually
        /// rejects a bad image instead of booting it.
        #[arg(long, hide = true)]
        corrupt: bool,
    },
}

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

fn main() -> Result<()> {
    let cli = Cli::parse();
    let node = NodeId::new(cli.node).ok_or("node id must be 1..=127")?;
    let mut can = SocketcanTransport::open(&cli.iface)?;

    match cli.command {
        Command::Status => status(&mut can, node)?,
        Command::EnterUpdate => {
            enter_update(&mut can, node)?;
        }
        Command::Reset => {
            // Only the updater implements 0x1F51. Against the application the
            // write just aborts, and reporting "reset requested" for a node
            // that did nothing is worse than saying so.
            if identify(&mut can, node)? != Role::Bootloader {
                return Err("node is running the application, which has no program \
                            control object — use `enter-update` first"
                    .into());
            }
            // The node resets while the response is still in flight, so a
            // timeout here is the expected outcome rather than a failure.
            let _ = write_u8(&mut can, node, PROGRAM_CONTROL, 1, CMD_RESET, T_SHORT);
            println!("reset requested");
        }
        Command::Flash {
            image,
            no_reset,
            corrupt,
        } => flash(&mut can, node, &image, no_reset, corrupt)?,
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

fn status(can: &mut SocketcanTransport, node: NodeId) -> Result<()> {
    let role = identify(can, node)?;
    println!("node {}: {}", node.raw(), role.describe());

    if role == Role::Bootloader {
        let status = read_u32(can, node, FLASH_STATUS, 1)?;
        let declared = read_u32(can, node, UPDATE_INFO, 1)?;
        let written = read_u32(can, node, UPDATE_INFO, 2)?;
        let base = read_u32(can, node, UPDATE_INFO, 3)?;
        let capacity = read_u32(can, node, UPDATE_INFO, 4)?;
        println!("  flash status:  {}", describe_status(status));
        println!("  app slot:      0x{base:08X}, {capacity} bytes");
        println!("  declared:      {declared} bytes");
        println!("  received:      {written} bytes");
    }
    Ok(())
}

/// Put the node into the updater, whichever image is currently running.
///
/// Returns once the bootloader has been positively identified on the bus.
fn enter_update(can: &mut SocketcanTransport, node: NodeId) -> Result<()> {
    match identify(can, node)? {
        Role::Bootloader => {
            println!("already in the updater");
            return Ok(());
        }
        Role::Application => {}
    }

    println!("requesting update mode...");
    // The node records the request and resets immediately afterwards, so the
    // SDO response may be lost. A timeout here is expected, not an error.
    let _ = write_u32(can, node, ENTER_UPDATE, 0, ENTER_UPDATE_MAGIC, T_SHORT);

    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(200));
        if let Ok(Role::Bootloader) = identify(can, node) {
            println!("updater is up");
            return Ok(());
        }
    }
    Err("node did not come back as the updater".into())
}

fn flash(
    can: &mut SocketcanTransport,
    node: NodeId,
    path: &PathBuf,
    no_reset: bool,
    corrupt: bool,
) -> Result<()> {
    let mut image = std::fs::read(path)?;
    if image.len() < 8 {
        return Err("image is too small to contain a vector table".into());
    }
    let crc = CRC32.checksum(&image);
    let sp = u32::from_le_bytes(image[0..4].try_into().unwrap());
    let reset = u32::from_le_bytes(image[4..8].try_into().unwrap());
    println!(
        "image {}: {} bytes, crc32=0x{:08X}, SP=0x{:08X}, reset=0x{:08X}",
        path.display(),
        image.len(),
        crc,
        sp,
        reset
    );

    if corrupt {
        let victim = image.len() / 2;
        image[victim] ^= 0xFF;
        println!("fault injection: byte {victim} corrupted after the CRC was taken");
    }

    enter_update(can, node)?;

    let capacity = read_u32(can, node, UPDATE_INFO, 4)?;
    let base = read_u32(can, node, UPDATE_INFO, 3)?;
    if image.len() as u32 > capacity {
        return Err(format!(
            "image is {} bytes but the slot holds {capacity}",
            image.len()
        )
        .into());
    }
    // Catch an image linked for the wrong address before erasing anything.
    if reset & 1 == 0 || reset < base || reset >= base + image.len() as u32 {
        return Err(format!(
            "image reset vector 0x{reset:08X} is not inside the slot at 0x{base:08X} — \
             it was probably linked for a different address"
        )
        .into());
    }

    write_u32(can, node, UPDATE_INFO, 1, image.len() as u32, T_SHORT)?;
    write_u32(can, node, PROGRAM_SW_ID, 1, crc, T_SHORT)?;

    println!("erasing...");
    let started = Instant::now();
    write_u8(can, node, PROGRAM_CONTROL, 1, CMD_CLEAR, T_FLASH)?;
    // The erase runs after the SDO response, so wait for the node to report
    // that it is actually ready for chunks rather than assuming it.
    await_status(can, node, ST_PROGRAMMING, Duration::from_secs(10))?;
    println!("erased in {:.1} s", started.elapsed().as_secs_f32());

    let started = Instant::now();
    let total = image.len();
    for (n, chunk) in image.chunks(MAX_CHUNK).enumerate() {
        sdo_download(can, node, PROGRAM_DATA, 1, chunk, T_FLASH)
            .map_err(|e| format!("chunk {n} failed: {e:?}"))?;
        let done = (n * MAX_CHUNK + chunk.len()) * 100 / total;
        print!("\rprogramming... {done}%");
        use std::io::Write;
        std::io::stdout().flush().ok();
    }
    println!(
        "\rprogrammed {total} bytes in {:.1} s",
        started.elapsed().as_secs_f32()
    );

    let written = read_u32(can, node, UPDATE_INFO, 2)?;
    if written != total as u32 {
        return Err(format!("node received {written} of {total} bytes").into());
    }

    println!("validating...");
    write_u8(can, node, PROGRAM_CONTROL, 1, CMD_VALIDATE, T_FLASH)?;
    let status = read_u32(can, node, FLASH_STATUS, 1)?;
    if status != ST_VALID {
        return Err(format!("validation failed: {}", describe_status(status)).into());
    }
    println!("image verified against flash and marked bootable");

    if no_reset {
        println!("leaving the node in the updater (--no-reset)");
        return Ok(());
    }

    println!("resetting into the new image...");
    // The node resets while answering, so a lost response is expected.
    let _ = write_u8(can, node, PROGRAM_CONTROL, 1, CMD_RESET, T_SHORT);

    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(200));
        if let Ok(Role::Application) = identify(can, node) {
            println!("application is running");
            return Ok(());
        }
    }
    Err("node did not come back as the application".into())
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq)]
enum Role {
    Application,
    Bootloader,
}

impl Role {
    fn describe(self) -> &'static str {
        match self {
            Role::Application => "application",
            Role::Bootloader => "updater",
        }
    }
}

/// Distinguish the two images by their identity object.
fn identify(can: &mut SocketcanTransport, node: NodeId) -> Result<Role> {
    let vendor = read_u32(can, node, IDENTITY, 1)?;
    if vendor != VENDOR_ID {
        return Err(format!("unexpected vendor id 0x{vendor:08X}").into());
    }
    match read_u32(can, node, IDENTITY, 2)? {
        PRODUCT_CODE_APP => Ok(Role::Application),
        PRODUCT_CODE_BOOTLOADER => Ok(Role::Bootloader),
        other => Err(format!("unexpected product code 0x{other:08X}").into()),
    }
}

fn await_status(
    can: &mut SocketcanTransport,
    node: NodeId,
    want: u32,
    timeout: Duration,
) -> Result<()> {
    let deadline = Instant::now() + timeout;
    loop {
        match read_u32(can, node, FLASH_STATUS, 1) {
            Ok(s) if s == want => return Ok(()),
            Ok(ST_ERROR) => return Err("node reported a flash error".into()),
            // The node stalls the bus-facing task while erasing, so a timeout
            // here just means "still busy".
            Ok(_) | Err(_) if Instant::now() < deadline => {
                std::thread::sleep(Duration::from_millis(100));
            }
            Ok(s) => return Err(format!("timed out waiting for status {want}, saw {s}").into()),
            Err(e) => return Err(e),
        }
    }
}

fn describe_status(status: u32) -> String {
    match status {
        ST_IDLE => "idle".into(),
        ST_PROGRAMMING => "programming".into(),
        ST_VALID => "valid".into(),
        ST_ERROR => "error".into(),
        other => format!("unknown ({other})"),
    }
}

fn read_u32(can: &mut SocketcanTransport, node: NodeId, index: u16, sub: u8) -> Result<u32> {
    let data = sdo_upload(can, node, index, sub, T_SHORT)
        .map_err(|e| format!("read 0x{index:04X}:{sub} failed: {e:?}"))?;
    let bytes: [u8; 4] = data
        .get(..4)
        .ok_or_else(|| format!("read 0x{index:04X}:{sub} returned {} bytes", data.len()))?
        .try_into()
        .unwrap();
    Ok(u32::from_le_bytes(bytes))
}

fn write_u32(
    can: &mut SocketcanTransport,
    node: NodeId,
    index: u16,
    sub: u8,
    value: u32,
    timeout: Duration,
) -> Result<()> {
    sdo_download(can, node, index, sub, &value.to_le_bytes(), timeout)
        .map_err(|e| format!("write 0x{index:04X}:{sub} failed: {e:?}").into())
}

fn write_u8(
    can: &mut SocketcanTransport,
    node: NodeId,
    index: u16,
    sub: u8,
    value: u8,
    timeout: Duration,
) -> Result<()> {
    sdo_download(can, node, index, sub, &[value], timeout)
        .map_err(|e| format!("write 0x{index:04X}:{sub} failed: {e:?}").into())
}
