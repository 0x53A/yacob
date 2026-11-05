//! SerialPort-backed SLCAN transport.

use super::logic::{encode_slcan_frame, parse_slcan_frame, SlcanBitrate};
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

/// SLCAN protocol driver over a [`SerialPort`].
///
/// Implements `embedded_can::nb::Can`, so it plugs in anywhere the other
/// transports do.
pub struct SlcanTransport<P: SerialPort> {
    port: P,
    rx_buf: [u8; 256],
    rx_pos: usize,
    discarding_line: bool,
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
            discarding_line: false,
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
        self.discarding_line = false;
        let mut drain = [0u8; 64];
        while matches!(self.port.read(&mut drain), Ok(n) if n > 0) {}
    }

    fn try_recv(&mut self) -> Result<Option<CanFrame>, P::Error> {
        if let Some(frame) = self.take_buffered_frame() {
            return Ok(Some(frame));
        }
        let space = self.rx_buf.len() - self.rx_pos;
        if space > 0 {
            let n = self.port.read(&mut self.rx_buf[self.rx_pos..])?;
            self.rx_pos += n;
        }
        Ok(self.take_buffered_frame())
    }

    /// Drain only already-buffered lines: ACKs and malformed lines must not
    /// report idle while a complete frame is sitting behind them.
    fn take_buffered_frame(&mut self) -> Option<CanFrame> {
        while let Some(cr_pos) = self.rx_buf[..self.rx_pos].iter().position(|&b| b == b'\r') {
            let frame = if self.discarding_line {
                self.discarding_line = false;
                None
            } else {
                parse_slcan_frame(&self.rx_buf[..cr_pos])
            };
            let remaining = self.rx_pos - cr_pos - 1;
            self.rx_buf.copy_within(cr_pos + 1..self.rx_pos, 0);
            self.rx_pos = remaining;
            if frame.is_some() {
                return frame;
            }
        }
        if self.rx_pos == self.rx_buf.len() {
            // Discard through the next delimiter, not merely this chunk:
            // otherwise a command-shaped tail would become a new CAN frame.
            self.rx_pos = 0;
            self.discarding_line = true;
        }
        None
    }
}

impl<P: SerialPort> embedded_can::nb::Can for SlcanTransport<P> {
    type Frame = CanFrame;
    type Error = CanError;

    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        let mut cmd = [0u8; 32];
        let pos =
            encode_slcan_frame(frame, &mut cmd).ok_or(nb::Error::Other(CanError::BusError))?;

        self.port
            .write_all(&cmd[..pos])
            .map_err(|_| nb::Error::Other(CanError::BusError))?;
        Ok(None)
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        self.try_recv()
            .map_err(|_| nb::Error::Other(CanError::BusError))?
            .ok_or(nb::Error::WouldBlock)
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

#[cfg(test)]
mod framing_regressions {
    use super::*;
    use embedded_can::nb::Can;

    struct Bytes {
        data: &'static [u8],
        reads: usize,
        fail_after: usize,
    }
    impl SerialPort for Bytes {
        type Error = ();
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
            if self.reads >= self.fail_after {
                return Err(());
            }
            self.reads += 1;
            let len = buf.len().min(self.data.len());
            buf[..len].copy_from_slice(&self.data[..len]);
            self.data = &self.data[len..];
            Ok(len)
        }
        fn write_all(&mut self, _: &[u8]) -> Result<(), ()> {
            Ok(())
        }
    }

    #[test]
    fn ack_and_invalid_lines_do_not_hide_a_buffered_frame() {
        let mut can = SlcanTransport::new(Bytes {
            data: b"\rF00\rt20112A\r",
            reads: 0,
            fail_after: 1,
        });
        assert_eq!(can.receive().unwrap().data(), &[42]);
    }

    #[test]
    fn buffered_frames_are_delivered_before_another_serial_read() {
        let mut can = SlcanTransport::new(Bytes {
            data: b"t20112A\rt20112B\r",
            reads: 0,
            fail_after: 1,
        });
        assert_eq!(can.receive().unwrap().data(), &[42]);
        assert_eq!(can.receive().unwrap().data(), &[43]);
        assert!(matches!(can.receive(), Err(nb::Error::Other(_))));
    }

    #[test]
    fn overlong_line_tail_cannot_become_a_command() {
        let mut can = SlcanTransport::new(Bytes {
            data: b"t20112A\rt20112B\r",
            reads: 0,
            fail_after: 2,
        });
        // Already filled with an unterminated malformed line. The next serial
        // chunk begins with a valid-looking tail of that same line.
        can.rx_buf.fill(b'x');
        can.rx_pos = can.rx_buf.len();
        let frame = can.receive().or_else(|_| can.receive()).unwrap();
        assert_eq!(frame.data(), &[43]);
    }
}

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
