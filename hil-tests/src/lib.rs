//! Hardware-in-the-loop test utilities.
//!
//! This crate provides helpers for testing a CANopen node running on an
//! STM32 MCU via a USB-CAN adapter on the Linux host.

use std::process::Command;

/// Flash firmware to the STM32 using probe-rs.
pub fn flash_firmware(elf_path: &str) -> Result<(), String> {
    let output = Command::new("probe-rs")
        .args(["run", "--chip", "STM32G431KBTx", elf_path])
        .output()
        .map_err(|e| format!("Failed to run probe-rs: {e}"))?;

    if !output.status.success() {
        return Err(format!(
            "probe-rs failed:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr),
        ));
    }
    Ok(())
}

/// Flash firmware in the background (non-blocking).
/// Returns the child process handle.
pub fn flash_firmware_background(elf_path: &str) -> Result<std::process::Child, String> {
    Command::new("probe-rs")
        .args(["run", "--chip", "STM32G431KBTx", elf_path])
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .map_err(|e| format!("Failed to spawn probe-rs: {e}"))
}

/// Reset the STM32 using probe-rs.
pub fn reset_target() -> Result<(), String> {
    let output = Command::new("probe-rs")
        .args(["reset", "--chip", "STM32G431KBTx"])
        .output()
        .map_err(|e| format!("Failed to run probe-rs reset: {e}"))?;

    if !output.status.success() {
        return Err(format!("probe-rs reset failed"));
    }
    Ok(())
}

/// Check if the CAN interface exists and is up.
pub fn check_can_interface(ifname: &str) -> bool {
    let output = Command::new("ip")
        .args(["link", "show", ifname])
        .output();

    match output {
        Ok(out) => {
            let s = String::from_utf8_lossy(&out.stdout);
            s.contains("UP") || s.contains("state UP")
        }
        Err(_) => false,
    }
}

/// Build the STM32 firmware (release mode).
pub fn build_firmware(project_dir: &str) -> Result<String, String> {
    let output = Command::new("cargo")
        .args(["build", "--release"])
        .current_dir(project_dir)
        .output()
        .map_err(|e| format!("Failed to run cargo build: {e}"))?;

    if !output.status.success() {
        return Err(format!(
            "Firmware build failed:\n{}",
            String::from_utf8_lossy(&output.stderr),
        ));
    }

    // Return path to the built ELF
    Ok(format!(
        "{}/target/thumbv7em-none-eabihf/release/stm32-canopen-node",
        project_dir
    ))
}
