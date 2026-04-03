use canopen_core::transport::{CanFrame, Transport, TransportError};
use socketcan::{CanDataFrame, CanSocket, EmbeddedFrame, Frame as ScFrame, Socket, StandardId};

/// Transport implementation using Linux SocketCAN.
///
/// Uses non-blocking mode internally so `recv()` returns `None` immediately
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

impl Transport for SocketcanTransport {
    fn send(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let id = StandardId::new(frame.id()).ok_or(TransportError::BusError)?;
        let sc_frame =
            CanDataFrame::new(id, frame.data()).ok_or(TransportError::BusError)?;
        self.socket
            .write_frame(&sc_frame)
            .map_err(|_| TransportError::BusError)?;
        Ok(())
    }

    fn recv(&mut self) -> Option<CanFrame> {
        match self.socket.read_frame() {
            Ok(frame) => sc_frame_to_canframe(&frame),
            Err(_) => None,
        }
    }
}
