//! Unix serial port backend for the SLCAN transport.
//!
//! The SLCAN protocol driver itself lives in [`canopen_core::slcan`] and is
//! generic over a [`SerialPort`]. This module provides [`UnixSerialPort`]
//! (raw termios serial port) and the [`open`]/[`open_raw`] helpers that
//! handle the adapter init sequence.
//!
//! Note on USB CDC-ACM devices (ESP32, Arduino, etc.):
//! The Linux kernel sets DTR when the port is opened, which resets the device.
//! This is unavoidable. We handle it by waiting for boot after open and then
//! sending the SLCAN init commands. We set `!HUPCL` so that closing the port
//! does NOT reset the device, allowing subsequent opens to find it already
//! streaming and skip the slow init.

use std::io::Read;
use std::time::Duration;

pub use canopen_core::slcan::{has_slcan_frame, SerialPort, SlcanBitrate};

/// SLCAN transport over a [`UnixSerialPort`].
pub type SlcanTransport = canopen_core::slcan::SlcanTransport<UnixSerialPort>;

/// Raw non-blocking serial port (115200 8N1, raw mode, `!HUPCL`).
pub struct UnixSerialPort {
    file: std::fs::File,
}

impl UnixSerialPort {
    /// Open and configure the serial port.
    pub fn open(path: &str) -> Result<Self, String> {
        // Open with same flags as pyserial: O_RDWR | O_NOCTTY | O_NONBLOCK
        // O_NONBLOCK at open time is critical — it prevents the kernel CDC-ACM
        // driver from blocking on carrier detect and may affect DTR behavior.
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
        // Own the descriptor before any fallible setup, so errors close it.
        let file = unsafe { std::fs::File::from_raw_fd(fd) };
        let setup_error = |step| {
            format!(
                "Failed to {step} for {path}: {}",
                std::io::Error::last_os_error()
            )
        };

        // Configure terminal: raw, 115200, no hupcl
        unsafe {
            let mut tio: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(fd, &mut tio) != 0 {
                return Err(setup_error("read terminal settings"));
            }
            libc::cfmakeraw(&mut tio);
            tio.c_cflag &= !libc::HUPCL;
            if libc::cfsetispeed(&mut tio, libc::B115200) != 0
                || libc::cfsetospeed(&mut tio, libc::B115200) != 0
            {
                return Err(setup_error("set terminal speed"));
            }
            tio.c_cc[libc::VMIN] = 0;
            tio.c_cc[libc::VTIME] = 0;
            if libc::tcsetattr(fd, libc::TCSANOW, &tio) != 0 {
                return Err(setup_error("apply terminal settings"));
            }
        }

        Ok(Self { file })
    }

    /// Assert DTR and RTS. On USB CDC-ACM devices this activates the port
    /// (and resets the device on first open after USB plug).
    pub fn set_dtr_rts(&mut self) {
        use std::os::unix::io::AsRawFd;
        let fd = self.file.as_raw_fd();
        let bits: libc::c_int = libc::TIOCM_DTR | libc::TIOCM_RTS;
        unsafe { libc::ioctl(fd, libc::TIOCMBIS, &bits) };
    }
}

impl SerialPort for UnixSerialPort {
    type Error = std::io::Error;

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // VMIN=0/VTIME=0 + O_NONBLOCK: no data comes back as Ok(0) or EAGAIN.
        match self.file.read(buf) {
            Ok(n) => Ok(n),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(0),
            Err(e) => Err(e),
        }
    }

    fn write_all(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        use std::io::Write;
        self.file.write_all(data)?;
        self.file.flush()
    }
}

/// Open an SLCAN adapter on the given serial port.
///
/// On first open (or after USB replug), the device resets via DTR and
/// the SLCAN init sequence is sent (~2.5s). On subsequent opens, if the
/// adapter is already streaming frames, it is reused immediately.
pub fn open(path: &str, bitrate: SlcanBitrate) -> Result<SlcanTransport, String> {
    let mut port = UnixSerialPort::open(path)?;
    let mut drain = [0u8; 512];

    // Check if the adapter is already streaming CAN frames
    // (from a previous session where !HUPCL kept it alive).
    // Poll for up to 1.5s.
    let mut detected = false;
    let probe_start = std::time::Instant::now();
    while probe_start.elapsed() < Duration::from_millis(1500) {
        let n = port.read(&mut drain).unwrap_or(0);
        if n > 0 && has_slcan_frame(&drain[..n]) {
            detected = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(50));
    }

    let mut slcan = SlcanTransport::new(port);

    if detected {
        // Already open — drain remaining data
        slcan.drain_rx();
    } else {
        // Not streaming. Set DTR+RTS to activate the CDC-ACM port,
        // which resets the device on first open after USB plug.
        slcan.port_mut().set_dtr_rts();

        // Wait for device to boot
        std::thread::sleep(Duration::from_secs(2));
        slcan.drain_rx();

        // SLCAN init: close (in case already open) → set bitrate → open
        let _ = slcan.send_close();
        std::thread::sleep(Duration::from_millis(100));
        slcan.drain_rx();

        let _ = slcan.send_bitrate(bitrate);
        std::thread::sleep(Duration::from_millis(100));
        slcan.drain_rx();

        let _ = slcan.send_open();
        std::thread::sleep(Duration::from_millis(100));
        slcan.drain_rx();
    }

    Ok(slcan)
}

/// Open without sending any SLCAN commands.
pub fn open_raw(path: &str) -> Result<SlcanTransport, String> {
    Ok(SlcanTransport::new(UnixSerialPort::open(path)?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::fd::{AsRawFd, FromRawFd};

    #[test]
    fn serial_open_configures_a_pseudoterminal() {
        let (mut master, mut slave) = (-1, -1);
        let mut name = [0 as libc::c_char; 128];
        assert_eq!(
            unsafe {
                libc::openpty(
                    &mut master,
                    &mut slave,
                    name.as_mut_ptr(),
                    std::ptr::null(),
                    std::ptr::null(),
                )
            },
            0
        );
        let _master = unsafe { std::fs::File::from_raw_fd(master) };
        let slave = unsafe { std::fs::File::from_raw_fd(slave) };
        let path = unsafe { std::ffi::CStr::from_ptr(name.as_ptr()) }
            .to_str()
            .unwrap();
        let port = UnixSerialPort::open(path).unwrap();
        let mut settings = unsafe { std::mem::zeroed::<libc::termios>() };
        assert_eq!(
            unsafe { libc::tcgetattr(slave.as_raw_fd(), &mut settings) },
            0
        );
        assert_eq!(settings.c_lflag & (libc::ICANON | libc::ECHO), 0);
        assert_eq!(settings.c_cflag & libc::HUPCL, 0);
        assert_eq!(settings.c_cc[libc::VMIN], 0);
        assert_eq!(settings.c_cc[libc::VTIME], 0);
        assert_eq!(unsafe { libc::cfgetispeed(&settings) }, libc::B115200);
        assert_eq!(unsafe { libc::cfgetospeed(&settings) }, libc::B115200);
        assert!(
            unsafe { libc::fcntl(port.file.as_raw_fd(), libc::F_GETFL) } & libc::O_NONBLOCK != 0
        );
    }
}
