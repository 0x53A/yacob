//! SLCAN (Serial Line CAN) protocol driver.
//!
//! Talks to an SLCAN adapter over any byte stream via the [`SerialPort`]
//! trait — a unix serial port (`canopen-linux`), a `serialport` crate port
//! on Windows/macOS (via [`IoPort`], `std` feature), or a bare MCU UART.
//! No `slcand` or kernel socketcan needed.
//!
//! SLCAN protocol (Lawicel):
//! - `S6\r`         — set 500 kbps
//! - `O\r`          — open CAN channel
//! - `C\r`          — close CAN channel
//! - `tIIILDD..\r`  — transmit standard frame (III=3 hex digits ID, L=DLC, DD=data hex)
//! - Received frames arrive as `tIIILDD..\r`
//!
//! Opening the port, configuring baud rate/raw mode, and the adapter init
//! sequence (which needs delays) are platform concerns and live with the
//! `SerialPort` implementation. The command helpers [`send_close`],
//! [`send_bitrate`], and [`send_open`] emit the init commands; the caller
//! sequences them with appropriate delays.
//!
//! [`send_close`]: SlcanTransport::send_close
//! [`send_bitrate`]: SlcanTransport::send_bitrate
//! [`send_open`]: SlcanTransport::send_open

use crate::transport::{CanError, CanFrame};

/// Byte-stream access to an SLCAN adapter.
///
/// Implementations must provide *non-blocking* reads: `read` returns
/// `Ok(0)` when no data is currently available.
pub trait SerialPort {
    type Error;

    /// Non-blocking read. Returns the number of bytes read; `Ok(0)` means
    /// no data available right now.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error>;

    /// Write all bytes.
    fn write_all(&mut self, data: &[u8]) -> Result<(), Self::Error>;
}

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

/// SLCAN protocol driver over a [`SerialPort`].
///
/// Implements `embedded_can::nb::Can`, so it plugs in anywhere the other
/// transports do.
pub struct SlcanTransport<P: SerialPort> {
    port: P,
    rx_buf: [u8; 256],
    rx_pos: usize,
}

impl<P: SerialPort> SlcanTransport<P> {
    /// Wrap an already-opened and configured serial port.
    ///
    /// Does not send any SLCAN commands; use the `send_*` helpers if the
    /// adapter still needs its init sequence.
    pub fn new(port: P) -> Self {
        Self {
            port,
            rx_buf: [0; 256],
            rx_pos: 0,
        }
    }

    /// Access the underlying port (e.g. for platform-specific control lines).
    pub fn port_mut(&mut self) -> &mut P {
        &mut self.port
    }

    /// Consume the transport and return the underlying port.
    pub fn into_inner(self) -> P {
        self.port
    }

    /// Send `C\r` — close the CAN channel.
    pub fn send_close(&mut self) -> Result<(), P::Error> {
        self.port.write_all(b"C\r")
    }

    /// Send `Sn\r` — set the CAN bitrate. Only valid while closed.
    pub fn send_bitrate(&mut self, bitrate: SlcanBitrate) -> Result<(), P::Error> {
        let cmd = [b'S', b'0' + bitrate as u8, b'\r'];
        self.port.write_all(&cmd)
    }

    /// Send `O\r` — open the CAN channel.
    pub fn send_open(&mut self) -> Result<(), P::Error> {
        self.port.write_all(b"O\r")
    }

    /// Discard any buffered RX bytes (driver-side and whatever the port
    /// currently has pending).
    pub fn drain_rx(&mut self) {
        self.rx_pos = 0;
        let mut drain = [0u8; 64];
        while matches!(self.port.read(&mut drain), Ok(n) if n > 0) {}
    }

    fn try_recv(&mut self) -> Option<CanFrame> {
        let space = self.rx_buf.len() - self.rx_pos;
        if space > 0 {
            if let Ok(n) = self.port.read(&mut self.rx_buf[self.rx_pos..]) {
                self.rx_pos += n;
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

impl<P: SerialPort> embedded_can::nb::Can for SlcanTransport<P> {
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

/// Check if a byte buffer contains what looks like a valid SLCAN frame.
///
/// Useful for probing whether an adapter is already streaming.
pub fn has_slcan_frame(buf: &[u8]) -> bool {
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

/// Adapter implementing [`SerialPort`] for any `std::io::Read + Write`
/// stream, e.g. a port from the `serialport` crate (cross-platform, opens
/// `COM1` on Windows) or a `std::fs::File` opened non-blocking.
///
/// `WouldBlock`, `TimedOut`, and `Interrupted` read errors are mapped to
/// "no data" (`Ok(0)`); configure the stream for non-blocking reads or a
/// short read timeout.
#[cfg(feature = "std")]
pub struct IoPort<T>(pub T);

#[cfg(feature = "std")]
impl<T: std::io::Read + std::io::Write> SerialPort for IoPort<T> {
    type Error = std::io::Error;

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        use std::io::ErrorKind;
        match self.0.read(buf) {
            Ok(n) => Ok(n),
            Err(e)
                if matches!(
                    e.kind(),
                    ErrorKind::WouldBlock | ErrorKind::TimedOut | ErrorKind::Interrupted
                ) =>
            {
                Ok(0)
            }
            Err(e) => Err(e),
        }
    }

    fn write_all(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        std::io::Write::write_all(&mut self.0, data)?;
        self.0.flush()
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

    /// In-memory SerialPort for driver tests.
    struct FakePort {
        rx: heapless::Deque<u8, 512>,
        tx: heapless::Vec<u8, 512>,
    }

    impl FakePort {
        fn new() -> Self {
            Self {
                rx: heapless::Deque::new(),
                tx: heapless::Vec::new(),
            }
        }

        fn feed(&mut self, data: &[u8]) {
            for &b in data {
                self.rx.push_back(b).unwrap();
            }
        }
    }

    impl SerialPort for FakePort {
        type Error = ();

        fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
            let mut n = 0;
            while n < buf.len() {
                match self.rx.pop_front() {
                    Some(b) => {
                        buf[n] = b;
                        n += 1;
                    }
                    None => break,
                }
            }
            Ok(n)
        }

        fn write_all(&mut self, data: &[u8]) -> Result<(), ()> {
            self.tx.extend_from_slice(data).map_err(|_| ())
        }
    }

    #[test]
    fn transport_receives_frames_split_across_reads() {
        use embedded_can::nb::Can;

        let mut slcan = SlcanTransport::new(FakePort::new());
        slcan.port_mut().feed(b"t70110");
        assert!(slcan.receive().is_err()); // incomplete line
        slcan.port_mut().feed(b"5\rt1FF");

        let frame = slcan.receive().unwrap();
        assert_eq!(frame.raw_id(), 0x701);
        assert_eq!(frame.data(), &[0x05]);
        assert!(slcan.receive().is_err()); // next line still incomplete
    }

    #[test]
    fn transport_transmit_encodes_frame() {
        use embedded_can::nb::Can;

        let mut slcan = SlcanTransport::new(FakePort::new());
        let frame = CanFrame::new(0x181, &[0x12, 0xAB]).unwrap();
        slcan.transmit(&frame).unwrap();
        assert_eq!(&slcan.port_mut().tx[..], b"t181212AB\r");
    }

    #[test]
    fn init_commands() {
        let mut slcan = SlcanTransport::new(FakePort::new());
        slcan.send_close().unwrap();
        slcan.send_bitrate(SlcanBitrate::S6).unwrap();
        slcan.send_open().unwrap();
        assert_eq!(&slcan.port_mut().tx[..], b"C\rS6\rO\r");
    }
}
