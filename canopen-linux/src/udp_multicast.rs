//! UDP multicast CAN transport, compatible with python-can's `udp_multicast` interface.
//!
//! This allows cross-process virtual CAN communication without kernel modules
//! or root privileges. Both sides just need to join the same multicast group.
//!
//! Wire format: msgpack-encoded dict matching python-can's schema.

use canopen_core::transport::{CanError, CanFrame};
use serde::{Deserialize, Serialize};
use socket2::{Domain, Protocol, SockAddr, Socket, Type};
use std::io;
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};

/// Default multicast group (matches python-can's default for IPv4).
pub const DEFAULT_MULTICAST_ADDR: Ipv4Addr = Ipv4Addr::new(239, 74, 163, 2);
/// Default port (matches python-can).
pub const DEFAULT_PORT: u16 = 43113;

/// Wire format matching python-can's udp_multicast interface.
#[derive(Serialize, Deserialize)]
struct WireMessage {
    timestamp: f64,
    arbitration_id: u32,
    is_extended_id: bool,
    is_remote_frame: bool,
    is_error_frame: bool,
    #[serde(with = "serde_bytes")]
    data: Vec<u8>,
    dlc: u8,
    is_fd: bool,
    bitrate_switch: bool,
    error_state_indicator: bool,
    channel: Option<String>,
}

/// Outcome of decoding one datagram.
#[derive(Debug)]
enum Decoded {
    Frame(CanFrame),
    /// Valid python-can message without CANopen meaning (extended, RTR,
    /// error or FD frame).
    Ignored,
    /// Inconsistent metadata or undecodable payload.
    Malformed,
}

fn decode_message(msg: WireMessage) -> Decoded {
    if msg.is_extended_id || msg.is_error_frame || msg.is_remote_frame || msg.is_fd {
        return Decoded::Ignored;
    }
    if msg.arbitration_id > 0x7ff || msg.dlc as usize != msg.data.len() {
        return Decoded::Malformed;
    }
    match CanFrame::new(msg.arbitration_id as u16, &msg.data) {
        Some(frame) => Decoded::Frame(frame),
        None => Decoded::Malformed,
    }
}

/// UDP multicast CAN transport.
///
/// Compatible with python-can's `udp_multicast` interface. Create with:
/// ```ignore
/// let transport = UdpMulticastTransport::new(None, None)?;
/// ```
pub struct UdpMulticastTransport {
    socket: UdpSocket,
    dest: SocketAddr,
    buf: [u8; 1024],
    malformed: u64,
}

impl UdpMulticastTransport {
    /// Number of received datagrams skipped because they could not be decoded
    /// as a classic CAN frame (bad msgpack, ID > 0x7FF, DLC/data mismatch).
    ///
    /// Any process can send to the multicast group, so such datagrams are
    /// counted and skipped rather than reported as a transport failure.
    pub fn malformed_count(&self) -> u64 {
        self.malformed
    }

    /// Create a new transport, joining the multicast group.
    ///
    /// - `addr`: multicast IPv4 address (default: 239.74.163.2)
    /// - `port`: UDP port (default: 43113)
    pub fn new(addr: Option<Ipv4Addr>, port: Option<u16>) -> io::Result<Self> {
        let mcast_addr = addr.unwrap_or(DEFAULT_MULTICAST_ADDR);
        let port = port.unwrap_or(DEFAULT_PORT);
        let bind_addr = SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, port);
        let dest = SocketAddr::V4(SocketAddrV4::new(mcast_addr, port));

        // Use socket2 to set SO_REUSEADDR before bind
        let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
        socket.set_reuse_address(true)?;
        // SO_REUSEPORT allows multiple processes to bind to the same port
        #[cfg(all(unix, not(target_os = "solaris")))]
        {
            use std::os::unix::io::AsRawFd;
            let fd = socket.as_raw_fd();
            let val: libc::c_int = 1;
            unsafe {
                libc::setsockopt(
                    fd,
                    libc::SOL_SOCKET,
                    libc::SO_REUSEPORT,
                    &val as *const _ as *const libc::c_void,
                    std::mem::size_of::<libc::c_int>() as libc::socklen_t,
                );
            }
        }
        socket.bind(&SockAddr::from(bind_addr))?;
        socket.set_nonblocking(true)?;
        socket.set_multicast_ttl_v4(1)?;
        socket.set_multicast_loop_v4(true)?;
        // Bind multicast to loopback so it works without network config
        socket.set_multicast_if_v4(&Ipv4Addr::LOCALHOST)?;

        let socket: UdpSocket = socket.into();
        socket.join_multicast_v4(&mcast_addr, &Ipv4Addr::LOCALHOST)?;

        Ok(Self {
            socket,
            dest,
            buf: [0u8; 1024],
            malformed: 0,
        })
    }
}

impl embedded_can::nb::Can for UdpMulticastTransport {
    type Frame = CanFrame;
    type Error = CanError;

    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        let msg = WireMessage {
            timestamp: 0.0, // python-can replaces this on receive anyway
            arbitration_id: frame.raw_id() as u32,
            is_extended_id: false,
            is_remote_frame: false,
            is_error_frame: false,
            data: frame.data().to_vec(),
            dlc: frame.raw_dlc(),
            is_fd: false,
            bitrate_switch: false,
            error_state_indicator: false,
            channel: None,
        };

        let encoded =
            rmp_serde::to_vec_named(&msg).map_err(|_| nb::Error::Other(CanError::BusError))?;

        self.socket
            .send_to(&encoded, self.dest)
            .map_err(|e| match e.kind() {
                io::ErrorKind::WouldBlock => nb::Error::WouldBlock,
                _ => nb::Error::Other(CanError::BusError),
            })?;

        Ok(None)
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        loop {
            match self.socket.recv_from(&mut self.buf) {
                Ok((len, _src)) => {
                    let decoded = match rmp_serde::from_slice::<WireMessage>(&self.buf[..len]) {
                        Ok(msg) => decode_message(msg),
                        Err(_) => Decoded::Malformed,
                    };
                    match decoded {
                        Decoded::Frame(frame) => return Ok(frame),
                        Decoded::Ignored => {}
                        Decoded::Malformed => self.malformed = self.malformed.saturating_add(1),
                    }
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                    return Err(nb::Error::WouldBlock)
                }
                Err(_) => return Err(nb::Error::Other(CanError::BusError)),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn message() -> WireMessage {
        WireMessage {
            timestamp: 0.0,
            arbitration_id: 0x201,
            is_extended_id: false,
            is_remote_frame: false,
            is_error_frame: false,
            data: vec![42],
            dlc: 1,
            is_fd: false,
            bitrate_switch: false,
            error_state_indicator: false,
            channel: None,
        }
    }

    #[test]
    fn wire_metadata_cannot_alias_a_classic_command() {
        assert!(matches!(decode_message(message()), Decoded::Frame(f) if f.data() == [42]));
        for variant in 0..7 {
            let mut msg = message();
            match variant {
                0 => msg.arbitration_id = 0x1201,
                1 => msg.arbitration_id = u32::MAX,
                2 => msg.dlc = 0,
                3 => msg.is_fd = true,
                4 => msg.is_extended_id = true,
                5 => msg.is_remote_frame = true,
                _ => msg.is_error_frame = true,
            }
            let expected_malformed = variant < 3;
            match decode_message(msg) {
                Decoded::Malformed => assert!(expected_malformed, "variant {variant}"),
                Decoded::Ignored => assert!(!expected_malformed, "variant {variant}"),
                Decoded::Frame(_) => panic!("variant {variant} decoded as a frame"),
            }
        }
    }
}

/// Convenience module for serde_bytes to handle `Vec<u8>` as msgpack binary.
mod serde_bytes {
    use serde::{self, Deserialize, Deserializer, Serializer};

    pub fn serialize<S>(bytes: &Vec<u8>, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_bytes(bytes)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Vec<u8>, D::Error>
    where
        D: Deserializer<'de>,
    {
        let bytes: &[u8] = Deserialize::deserialize(deserializer)?;
        Ok(bytes.to_vec())
    }
}
