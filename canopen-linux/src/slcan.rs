//! SLCAN (Serial Line CAN) transport.
//!
//! Talks directly to an SLCAN adapter over a serial port.
//! No `slcand` or kernel socketcan needed.
//!
//! SLCAN protocol (Lawicel):
//! - `S6\r`         — set 500 kbps
//! - `O\r`          — open CAN channel
//! - `C\r`          — close CAN channel
//! - `tIIILDD..\r`  — transmit standard frame (III=3 hex digits ID, L=DLC, DD=data hex)
//! - Received frames arrive as `tIIILDD..\r`
//!
//! Note on USB CDC-ACM devices (ESP32, Arduino, etc.):
//! The Linux kernel sets DTR when the port is opened, which resets the device.
//! This is unavoidable. We handle it by waiting for boot after open and then
//! sending the SLCAN init commands. We set `!HUPCL` so that closing the port
//! does NOT reset the device, allowing subsequent opens to find it already
//! streaming and skip the slow init.

use canopen_core::transport::{CanError, CanFrame};
use std::io::{Read, Write};
use std::time::Duration;

/// SLCAN bitrate codes.
#[derive(Clone, Copy, Debug)]
pub enum SlcanBitrate {
    S0 = 0, // 10 kbps
    S1 = 1, // 20 kbps
    S2 = 2, // 50 kbps
    S3 = 3, // 100 kbps
    S4 = 4, // 125 kbps
    S5 = 5, // 250 kbps
    S6 = 6, // 500 kbps
    S7 = 7, // 800 kbps
    S8 = 8, // 1000 kbps
}

pub struct SlcanTransport {
    port: std::fs::File,
    rx_buf: [u8; 256],
    rx_pos: usize,
}

impl SlcanTransport {
    /// Open an SLCAN adapter on the given serial port.
    ///
    /// On first open (or after USB replug), the device resets via DTR and
    /// the SLCAN init sequence is sent (~2.5s). On subsequent opens, if the
    /// adapter is already streaming frames, it is reused immediately.
    pub fn open(path: &str, bitrate: SlcanBitrate) -> Result<Self, String> {
        Self::open_opts(path, Some(bitrate))
    }

    /// Open without sending any SLCAN commands.
    pub fn open_raw(path: &str) -> Result<Self, String> {
        Self::open_opts(path, None)
    }

    fn open_opts(path: &str, bitrate: Option<SlcanBitrate>) -> Result<Self, String> {
        // Open with same flags as pyserial: O_RDWR | O_NOCTTY | O_NONBLOCK
        // O_NONBLOCK at open time is critical — it prevents the kernel CDC-ACM
        // driver from blocking on carrier detect and may affect DTR behavior.
        #[cfg(unix)]
        let port = {
            use std::os::unix::io::FromRawFd;
            let c_path = std::ffi::CString::new(path).map_err(|e| format!("{e}"))?;
            let fd = unsafe {
                libc::open(
                    c_path.as_ptr(),
                    libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK,
                )
            };
            if fd < 0 {
                return Err(format!(
                    "Failed to open {path}: {}",
                    std::io::Error::last_os_error()
                ));
            }

            // Configure terminal: raw, 115200, no hupcl
            unsafe {
                let mut tio: libc::termios = std::mem::zeroed();
                libc::tcgetattr(fd, &mut tio);
                libc::cfmakeraw(&mut tio);
                tio.c_cflag &= !libc::HUPCL;
                libc::cfsetispeed(&mut tio, libc::B115200);
                libc::cfsetospeed(&mut tio, libc::B115200);
                tio.c_cc[libc::VMIN] = 0;
                tio.c_cc[libc::VTIME] = 0;
                libc::tcsetattr(fd, libc::TCSANOW, &tio);
            }

            unsafe { std::fs::File::from_raw_fd(fd) }
        };

        let mut this = Self {
            port,
            rx_buf: [0; 256],
            rx_pos: 0,
        };

        if let Some(br) = bitrate {
            let mut drain = [0u8; 512];

            // Check if the adapter is already streaming CAN frames
            // (from a previous session where !HUPCL kept it alive).
            // Poll for up to 1.5s to detect if the adapter is already
            // streaming frames (previous session with !HUPCL).
            let mut detected = false;
            let probe_start = std::time::Instant::now();
            while probe_start.elapsed() < Duration::from_millis(1500) {
                let n = this.port.read(&mut drain).unwrap_or(0);
                if n > 0 && has_slcan_frame(&drain[..n]) {
                    detected = true;
                    break;
                }
                std::thread::sleep(Duration::from_millis(50));
            }

            if detected {
                // Already open — drain remaining data
                while this.port.read(&mut drain).unwrap_or(0) > 0 {}
            } else {
                // Not streaming. Set DTR+RTS to activate the CDC-ACM port,
                // which resets the device on first open after USB plug.
                #[cfg(unix)]
                {
                    use std::os::unix::io::AsRawFd;
                    let fd = this.port.as_raw_fd();
                    let bits: libc::c_int = libc::TIOCM_DTR | libc::TIOCM_RTS;
                    unsafe { libc::ioctl(fd, libc::TIOCMBIS, &bits) };
                }

                // Wait for device to boot
                std::thread::sleep(Duration::from_secs(2));
                while this.port.read(&mut drain).unwrap_or(0) > 0 {}

                // SLCAN init: close (in case already open) → set bitrate → open
                let _ = this.port.write_all(b"C\r");
                let _ = this.port.flush();
                std::thread::sleep(Duration::from_millis(100));
                while this.port.read(&mut drain).unwrap_or(0) > 0 {}

                let cmd = format!("S{}\r", br as u8);
                let _ = this.port.write_all(cmd.as_bytes());
                let _ = this.port.flush();
                std::thread::sleep(Duration::from_millis(100));
                while this.port.read(&mut drain).unwrap_or(0) > 0 {}

                let _ = this.port.write_all(b"O\r");
                let _ = this.port.flush();
                std::thread::sleep(Duration::from_millis(100));
                while this.port.read(&mut drain).unwrap_or(0) > 0 {}
            }
        }

        Ok(this)
    }

    /// Close the CAN channel.
    pub fn close(&mut self) {
        let _ = self.port.write_all(b"C\r");
    }

    fn try_recv(&mut self) -> Option<CanFrame> {
        let space = self.rx_buf.len() - self.rx_pos;
        if space > 0 {
            match self.port.read(&mut self.rx_buf[self.rx_pos..]) {
                Ok(n) => self.rx_pos += n,
                Err(_) => {}
            }
        }

        let buf = &self.rx_buf[..self.rx_pos];
        if let Some(cr_pos) = buf.iter().position(|&b| b == b'\r') {
            let line = &buf[..cr_pos];
            let frame = parse_slcan_frame(line);

            let remaining = self.rx_pos - cr_pos - 1;
            self.rx_buf.copy_within(cr_pos + 1..self.rx_pos, 0);
            self.rx_pos = remaining;

            frame
        } else {
            if self.rx_pos > 200 {
                self.rx_pos = 0;
            }
            None
        }
    }
}

impl Drop for SlcanTransport {
    fn drop(&mut self) {
        // Don't send 'C' on drop. Combined with !HUPCL, this keeps the
        // adapter open so the next open() can reuse it without the 2s boot wait.
    }
}

/// Check if a byte buffer contains what looks like a valid SLCAN frame.
fn has_slcan_frame(buf: &[u8]) -> bool {
    for window in buf.windows(5) {
        if window[0] == b't'
            && parse_hex(window[1]).is_some()
            && parse_hex(window[2]).is_some()
            && parse_hex(window[3]).is_some()
            && window[4] >= b'0'
            && window[4] <= b'8'
        {
            return true;
        }
    }
    false
}

impl embedded_can::nb::Can for SlcanTransport {
    type Frame = CanFrame;
    type Error = CanError;

    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        let mut cmd = [0u8; 32];
        let id = frame.raw_id();
        let data = frame.data();

        let mut pos = 0;
        cmd[pos] = b't';
        pos += 1;
        cmd[pos] = hex_digit((id >> 8) as u8 & 0x0F);
        pos += 1;
        cmd[pos] = hex_digit((id >> 4) as u8 & 0x0F);
        pos += 1;
        cmd[pos] = hex_digit(id as u8 & 0x0F);
        pos += 1;
        cmd[pos] = b'0' + frame.raw_dlc();
        pos += 1;
        for &b in data {
            cmd[pos] = hex_digit(b >> 4);
            pos += 1;
            cmd[pos] = hex_digit(b & 0x0F);
            pos += 1;
        }
        cmd[pos] = b'\r';
        pos += 1;

        self.port
            .write_all(&cmd[..pos])
            .map_err(|_| nb::Error::Other(CanError::BusError))?;
        Ok(None)
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        self.try_recv().ok_or(nb::Error::WouldBlock)
    }
}

fn parse_slcan_frame(line: &[u8]) -> Option<CanFrame> {
    if line.is_empty() {
        return None;
    }
    match line[0] {
        b't' => {
            if line.len() < 5 {
                return None;
            }
            let id = (parse_hex(line[1])? as u16) << 8
                | (parse_hex(line[2])? as u16) << 4
                | parse_hex(line[3])? as u16;
            let dlc = (line[4] - b'0') as usize;
            if line.len() < 5 + dlc * 2 {
                return None;
            }
            let mut data = [0u8; 8];
            for i in 0..dlc {
                data[i] = (parse_hex(line[5 + i * 2])? << 4) | parse_hex(line[6 + i * 2])?;
            }
            CanFrame::new(id, &data[..dlc])
        }
        _ => None,
    }
}

fn hex_digit(val: u8) -> u8 {
    match val & 0x0F {
        0..=9 => b'0' + val,
        10..=15 => b'A' + (val - 10),
        _ => b'0',
    }
}

fn parse_hex(ch: u8) -> Option<u8> {
    match ch {
        b'0'..=b'9' => Some(ch - b'0'),
        b'a'..=b'f' => Some(ch - b'a' + 10),
        b'A'..=b'F' => Some(ch - b'A' + 10),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_standard_frame() {
        let line = b"t1FF8DEADBEEFCAFEBABE";
        let frame = parse_slcan_frame(line).unwrap();
        assert_eq!(frame.raw_id(), 0x1FF);
        assert_eq!(frame.raw_dlc(), 8);
        assert_eq!(
            frame.data(),
            &[0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE]
        );
    }

    #[test]
    fn parse_heartbeat_frame() {
        let line = b"t701105";
        let frame = parse_slcan_frame(line).unwrap();
        assert_eq!(frame.raw_id(), 0x701);
        assert_eq!(frame.raw_dlc(), 1);
        assert_eq!(frame.data(), &[0x05]);
    }

    #[test]
    fn parse_empty() {
        assert!(parse_slcan_frame(b"").is_none());
        assert!(parse_slcan_frame(b"\x07").is_none());
    }
}
