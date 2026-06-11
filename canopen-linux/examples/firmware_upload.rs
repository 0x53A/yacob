//! Firmware uploader — master-side tool for CiA 302 firmware update.
//!
//! Uploads a binary file to a remote CANopen node using the CiA 302
//! programming protocol:
//!
//!   0x1F50 sub1: Program Data (domain, receives chunks)
//!   0x1F51 sub1: Program Control (u8: 3=clear, 1=validate, 0x80=reset)
//!   0x1F56 sub1: Program Software ID (u32: CRC32)
//!   0x1F57 sub1: Flash Status (u32: 0=idle, 1=prog, 2=valid, 0xFF=error)
//!
//! Usage:
//!   # Start the firmware-update node first (STM32 or vcan simulation)
//!   CAN_TRANSPORT=udp cargo run --example firmware_upload -- firmware.bin
//!
//! ## API observations
//!
//! Building this uploader revealed several API gaps:
//!
//! 1. **No timeout on SdoDriver**: The async `upload()` and `download()` methods
//!    have no timeout parameter. If the target node dies mid-transfer, the driver
//!    hangs forever waiting for a response. We work around this by using the
//!    blocking `sdo_helpers` which wrap the async driver with a timeout, but
//!    a native `SdoDriver::with_timeout()` would be cleaner.
//!
//! 2. **NMT command builder now in canopen-core**: `NmtCommand::to_frame(target)`
//!    builds the CAN frame directly, usable from no_std embedded masters.
//!    (Previously this was only available via canopen-linux's `nmt_command()`.)
//!
//! 3. **No progress/streaming support for SDO downloads**: Each chunk is a
//!    separate SDO transfer (connect → transfer → disconnect). For firmware
//!    update this means N full SDO handshakes. A multi-segment streaming
//!    download (write once, send all data) would be more efficient, but the
//!    889-byte server buffer limits this anyway.
//!
//! 4. **SdoClient hardcoded to 256-byte buffer**: The low-level `SdoClient`
//!    has `download_buf: [u8; 256]`. Larger chunks would reduce the number of
//!    SDO transfers needed. Making this a const generic (`SdoClient<const BUF: usize>`)
//!    would help — though the server's 889-byte limit is the real ceiling.
//!
//! 5. **HeartbeatConsumer not async-friendly**: `HeartbeatConsumer` tracks state
//!    and timeouts but requires polling. For an async master that awaits heartbeats
//!    before starting operations, an `async fn wait_heartbeat()` would help.
//!    The `wait_heartbeat()` in sdo_helpers is blocking, not truly async.
//!
//! 6. **No typed SDO client for CiA 302**: The `sdo_client_from_eds!` macro
//!    generates typed clients from EDS files. A similar pattern for standard
//!    CiA profiles (302 programming, 402 drives) would reduce boilerplate.

use canopen_core::cobid::NodeId;
use canopen_core::nmt::NmtCommand;
use canopen_core::transport::CanFrame;
use canopen_linux::sdo_helpers::*;
use canopen_linux::{SocketcanTransport, UdpMulticastTransport};
use std::time::Duration;

const CHUNK_SIZE: usize = 128; // Conservative chunk size (fits in one SDO segmented transfer)

fn upload_firmware(
    transport: &mut impl embedded_can::nb::Can<Frame = CanFrame, Error: core::fmt::Debug>,
    target: NodeId,
    firmware: &[u8],
) -> Result<(), Box<dyn std::error::Error>> {
    let timeout = Duration::from_secs(5);

    // Step 1: Wait for heartbeat
    println!("[1/7] Waiting for heartbeat from node {}...", target.raw());
    let state = wait_heartbeat(transport, target, timeout)?;
    println!("       Node alive, NMT state: 0x{:02X}", state);

    // Step 2: Read current firmware version
    println!("[2/7] Reading current firmware info...");
    match sdo_upload(transport, target, 0x1F56, 1, timeout) {
        Ok(data) if data.len() >= 4 => {
            let crc = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
            println!("       Current Software ID (CRC): 0x{:08X}", crc);
        }
        Ok(_) => println!("       Software ID: (unexpected size)"),
        Err(e) => println!("       Could not read Software ID: {} (continuing)", e),
    }

    // Read flash status
    match sdo_upload(transport, target, 0x1F57, 1, timeout) {
        Ok(data) if data.len() >= 4 => {
            let status = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
            println!(
                "       Flash status: {}",
                match status {
                    0 => "idle",
                    1 => "programming",
                    2 => "valid",
                    0xFF => "error",
                    _ => "unknown",
                }
            );
        }
        _ => {}
    }

    // Step 3: Send NMT Pre-Operational (stop PDOs during update)
    println!("[3/7] Sending NMT Pre-Operational...");
    let frame = NmtCommand::EnterPreOperational.to_frame(target.raw());
    transport
        .transmit(&frame)
        .map_err(|_| SdoError::TransportError)?;
    std::thread::sleep(Duration::from_millis(100));

    // Step 4: Clear flash (write 3 to Program Control)
    println!("[4/7] Clearing flash region...");
    sdo_download(transport, target, 0x1F51, 1, &[3], timeout)?;
    std::thread::sleep(Duration::from_millis(500)); // Give flash erase time

    // Verify status is idle after clear
    let status_data = sdo_upload(transport, target, 0x1F57, 1, timeout)?;
    let status = u32::from_le_bytes([
        status_data[0],
        status_data[1],
        status_data[2],
        status_data[3],
    ]);
    if status != 0 {
        return Err(format!("Flash not idle after clear (status={})", status).into());
    }
    println!("       Flash cleared successfully");

    // Step 5: Upload firmware in chunks
    let total_chunks = (firmware.len() + CHUNK_SIZE - 1) / CHUNK_SIZE;
    println!(
        "[5/7] Uploading {} bytes in {} chunks of {} bytes...",
        firmware.len(),
        total_chunks,
        CHUNK_SIZE
    );

    // Compute CRC32 of firmware
    let crc = crc32(firmware);

    for (i, chunk) in firmware.chunks(CHUNK_SIZE).enumerate() {
        // API OBSERVATION: Each chunk is a separate SDO transfer.
        // This means: initiate download → send segments → complete → repeat.
        // For 100KB firmware with 128-byte chunks, that's 800 SDO handshakes.
        // A streaming mode would be significantly faster.
        sdo_download(transport, target, 0x1F50, 1, chunk, timeout)?;

        let pct = (i + 1) * 100 / total_chunks;
        let bytes_sent = (i + 1) * CHUNK_SIZE;
        print!(
            "\r       [{:3}%] {}/{} bytes",
            pct,
            bytes_sent.min(firmware.len()),
            firmware.len()
        );
    }
    println!();

    // Step 6: Write expected CRC and validate
    println!("[6/7] Validating firmware (CRC32: 0x{:08X})...", crc);
    sdo_download(transport, target, 0x1F56, 1, &crc.to_le_bytes(), timeout)?;
    sdo_download(transport, target, 0x1F51, 1, &[1], timeout)?; // validate command

    // Read status
    std::thread::sleep(Duration::from_millis(100));
    let status_data = sdo_upload(transport, target, 0x1F57, 1, timeout)?;
    let status = u32::from_le_bytes([
        status_data[0],
        status_data[1],
        status_data[2],
        status_data[3],
    ]);

    match status {
        2 => println!("       Firmware validated successfully!"),
        0xFF => return Err("Firmware validation failed (CRC mismatch)".into()),
        other => return Err(format!("Unexpected flash status after validate: {}", other).into()),
    }

    // Step 7: Reset node into new firmware
    println!("[7/7] Resetting node into new firmware...");
    sdo_download(transport, target, 0x1F51, 1, &[0x80], timeout)?;

    // Also send an NMT Reset Node as a fallback
    let reset_frame = NmtCommand::ResetNode.to_frame(target.raw());
    let _ = transport.transmit(&reset_frame);

    // Wait for node to come back
    println!("       Waiting for node to reboot...");
    std::thread::sleep(Duration::from_secs(2));
    match wait_heartbeat(transport, target, Duration::from_secs(10)) {
        Ok(state) => println!("       Node back online! NMT state: 0x{:02X}", state),
        Err(_) => println!("       Node did not respond (may need manual check)"),
    }

    println!("\nFirmware update complete!");
    Ok(())
}

/// Simple CRC32 (ISO-HDLC / "standard" CRC-32).
fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320;
            } else {
                crc >>= 1;
            }
        }
    }
    !crc
}

fn main() {
    let args: Vec<String> = std::env::args().collect();

    // Load firmware file (or generate test data)
    let firmware = if args.len() > 1 {
        std::fs::read(&args[1]).unwrap_or_else(|e| {
            eprintln!("Failed to read {}: {}", args[1], e);
            std::process::exit(1);
        })
    } else {
        // Generate test firmware data (1KB of pattern data)
        eprintln!("No firmware file specified, using 1KB test pattern");
        (0..1024).map(|i| (i & 0xFF) as u8).collect()
    };

    let target = NodeId::new(2).unwrap(); // firmware-update node is node 2

    let transport_type = std::env::var("CAN_TRANSPORT").unwrap_or("socketcan".into());

    println!("=== CANopen Firmware Uploader ===");
    println!("Target: node {}", target.raw());
    println!("Firmware: {} bytes", firmware.len());
    println!("Transport: {}\n", transport_type);

    let result = match transport_type.as_str() {
        "udp" => {
            let mut transport = UdpMulticastTransport::new(None, None)
                .expect("Failed to create UDP multicast transport");
            upload_firmware(&mut transport, target, &firmware)
        }
        _ => {
            let iface = std::env::var("CAN_IFACE").unwrap_or("vcan0".into());
            let mut transport = SocketcanTransport::open(&iface).unwrap_or_else(|e| {
                eprintln!("Failed to open {}: {}", iface, e);
                std::process::exit(1);
            });
            upload_firmware(&mut transport, target, &firmware)
        }
    };

    if let Err(e) = result {
        eprintln!("\nFirmware update failed: {}", e);
        std::process::exit(1);
    }
}
