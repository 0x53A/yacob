use canopen_core::transport::{CanError, CanFrame};
use socketcan::{CanDataFrame, CanSocket, EmbeddedFrame, Frame as ScFrame, Socket, StandardId};

/// CAN transport using Linux SocketCAN, implementing `embedded_can::nb::Can`.
///
/// Uses non-blocking mode internally so `receive()` returns `WouldBlock`
/// when no frame is available.
pub struct SocketcanTransport {
    socket: CanSocket,
}

fn sc_frame_to_canframe(frame: &socketcan::CanFrame) -> Option<CanFrame> {
    match frame {
        socketcan::CanFrame::Data(df) => {
            let id = (df.raw_id() & 0x7FF) as u16;
            CanFrame::new(id, df.data())
        }
        _ => None,
    }
}

impl SocketcanTransport {
    /// Open a CAN interface (e.g., "can0", "vcan0").
    pub fn open(ifname: &str) -> std::io::Result<Self> {
        let socket = CanSocket::open(ifname)?;
        socket.set_nonblocking(true)?;
        Ok(Self { socket })
    }

    /// Get a reference to the underlying socket.
    pub fn socket(&self) -> &CanSocket {
        &self.socket
    }

    /// Set a receive timeout. Useful for blocking reads in test harnesses.
    pub fn set_read_timeout(&self, timeout: std::time::Duration) -> std::io::Result<()> {
        self.socket.set_read_timeout(timeout)
    }

    /// Blocking receive with timeout. Returns None on timeout.
    pub fn recv_blocking(&self, timeout: std::time::Duration) -> Option<CanFrame> {
        self.set_read_timeout(timeout).ok()?;
        match self.socket.read_frame() {
            Ok(frame) => sc_frame_to_canframe(&frame),
            Err(_) => None,
        }
    }
}

impl embedded_can::nb::Can for SocketcanTransport {
    type Frame = CanFrame;
    type Error = CanError;

    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        let id = StandardId::new(frame.raw_id()).ok_or(nb::Error::Other(CanError::BusError))?;
        let sc_frame =
            CanDataFrame::new(id, frame.data()).ok_or(nb::Error::Other(CanError::BusError))?;
        self.socket
            .write_frame(&sc_frame)
            .map_err(|_| nb::Error::Other(CanError::BusError))?;
        Ok(None)
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        match self.socket.read_frame() {
            Ok(frame) => sc_frame_to_canframe(&frame).ok_or(nb::Error::WouldBlock),
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => Err(nb::Error::WouldBlock),
            Err(_) => Err(nb::Error::Other(CanError::BusError)),
        }
    }
}
