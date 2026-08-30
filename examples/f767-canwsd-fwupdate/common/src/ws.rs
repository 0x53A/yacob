//! A deliberately small, single-client canwsd server.
//!
//! WebSocket is only a carrier for classic 11-bit CAN frames. The update
//! protocol remains ordinary CANopen SDO against the object dictionary.

use canopen_core::transport::CanFrame;
use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_time::{with_timeout, Duration};
use sha1::{Digest, Sha1};

use crate::hal::{NODE_RX, NODE_TX};
use crate::{HTTP_PORT, NETWORK_NAME};

const WS_GUID: &str = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

#[embassy_executor::task]
pub async fn canwsd_task(stack: Stack<'static>) -> ! {
    stack.wait_config_up().await;
    defmt::info!("network up: {:?}", defmt::Debug2Format(&stack.config_v4()));
    let mut rx = [0u8; 2048];
    let mut tx = [0u8; 2048];
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx, &mut tx);
        socket.set_timeout(Some(Duration::from_secs(30)));
        if socket.accept(HTTP_PORT).await.is_err() {
            continue;
        }
        if serve_connection(&mut socket).await == Ok(ConnectionKind::Http) {
            socket.close();
            // HTTP has a response queued. Give the runner a short flush window
            // before forcibly releasing the one reusable socket.
            embassy_time::Timer::after_millis(50).await;
        }
        // A completed WebSocket session has no response pending. Abort it
        // directly; graceful TCP closing states otherwise keep this single
        // listener from rebinding port 8080 for the next uploader connection.
        socket.abort();
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum ConnectionKind {
    Http,
    WebSocket,
}

async fn serve_connection(socket: &mut TcpSocket<'_>) -> Result<ConnectionKind, ()> {
    let mut request = [0u8; 1024];
    let mut len = 0;
    loop {
        if len == request.len() {
            return Err(());
        }
        let n = socket.read(&mut request[len..]).await.map_err(|_| ())?;
        if n == 0 {
            return Err(());
        }
        len += n;
        if request[..len].windows(4).any(|w| w == b"\r\n\r\n") {
            break;
        }
    }

    if request_line(&request[..len]) == Some("GET /api/networks HTTP/1.1")
        || request_line(&request[..len]) == Some("GET /api/networks HTTP/1.0")
    {
        serve_networks(socket).await?;
        return Ok(ConnectionKind::Http);
    }
    handshake(socket, &request[..len]).await?;
    session(socket).await?;
    Ok(ConnectionKind::WebSocket)
}

fn request_line(request: &[u8]) -> Option<&str> {
    core::str::from_utf8(request).ok()?.split("\r\n").next()
}

async fn serve_networks(socket: &mut TcpSocket<'_>) -> Result<(), ()> {
    let networks = [canwsd_proto::NetworkInfoRef {
        name: NETWORK_NAME,
        interface: "virtual",
        bitrate: 0,
        status: canwsd_proto::NetworkStatus::Available,
        error: "",
    }];
    let mut body = [0u8; 160];
    let len = serde_json_core::to_slice(&networks, &mut body).map_err(|_| ())?;
    let mut header = heapless::String::<192>::new();
    use core::fmt::Write as _;
    write!(
        header,
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        len
    )
    .map_err(|_| ())?;
    write_all(socket, header.as_bytes()).await?;
    write_all(socket, &body[..len]).await
}

async fn handshake(socket: &mut TcpSocket<'_>, request: &[u8]) -> Result<(), ()> {
    let text = core::str::from_utf8(request).map_err(|_| ())?;
    let line = request_line(request).ok_or(())?;
    if line != "GET /api/networks/update HTTP/1.1" && line != "GET /api/networks/update HTTP/1.0" {
        return Err(());
    }
    if header(text, "Upgrade").is_none_or(|v| !v.eq_ignore_ascii_case("websocket"))
        || header(text, "Sec-WebSocket-Version") != Some("13")
    {
        return Err(());
    }
    let key = header(text, "Sec-WebSocket-Key")
        .filter(|v| !v.is_empty())
        .ok_or(())?;
    let mut hasher = Sha1::new();
    hasher.update(key.as_bytes());
    hasher.update(WS_GUID.as_bytes());
    let hash = hasher.finalize();
    let mut accept = [0u8; 28];
    base64(&hash, &mut accept);
    let accept = core::str::from_utf8(&accept).map_err(|_| ())?;
    let mut response = heapless::String::<256>::new();
    use core::fmt::Write as _;
    write!(
        response,
        "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: {}\r\n\r\n",
        accept
    )
    .map_err(|_| ())?;
    write_all(socket, response.as_bytes()).await
}

fn header<'a>(request: &'a str, name: &str) -> Option<&'a str> {
    request.split("\r\n").skip(1).find_map(|line| {
        let (candidate, value) = line.split_once(':')?;
        candidate
            .trim()
            .eq_ignore_ascii_case(name)
            .then_some(value.trim())
    })
}

async fn session(socket: &mut TcpSocket<'_>) -> Result<(), ()> {
    // Responses belong to the client that caused them. If that client vanished
    // mid-SDO, never deliver its queued tail to the next connection.
    while NODE_TX.try_receive().is_ok() {}
    let mut header = [0u8; 2];
    let mut header_len = 0;
    let mut payload = [0u8; canwsd_proto::MAX_WIRE_FRAME_SIZE];
    loop {
        while let Ok(frame) = NODE_TX.try_receive() {
            let wire =
                canwsd_proto::WireFrame::new(frame.raw_id() as u32, frame.data()).ok_or(())?;
            let (encoded, len) = wire.encode();
            send_frame(socket, 2, &encoded[..len]).await?;
        }

        match with_timeout(
            Duration::from_millis(2),
            read_header(socket, &mut header, &mut header_len),
        )
        .await
        {
            Err(_) => continue,
            Ok(Err(())) => return Err(()),
            Ok(Ok(())) => {}
        }
        header_len = 0;
        let Some((opcode, len)) = read_body(socket, header, &mut payload).await? else {
            return Ok(());
        };
        match opcode {
            2 => {
                let wire = canwsd_proto::WireFrame::decode(&payload[..len]).map_err(|_| ())?;
                if wire.is_extended() || wire.is_rtr() || wire.is_error() {
                    continue;
                }
                if let Some(frame) = CanFrame::new(wire.id() as u16, wire.data()) {
                    NODE_RX.try_send(frame).map_err(|_| ())?;
                }
            }
            9 => send_frame(socket, 10, &payload[..len]).await?,
            _ => {}
        }
    }
}

async fn read_header(
    socket: &mut TcpSocket<'_>,
    header: &mut [u8; 2],
    pos: &mut usize,
) -> Result<(), ()> {
    while *pos < 2 {
        let n = socket.read(&mut header[*pos..]).await.map_err(|_| ())?;
        if n == 0 {
            return Err(());
        }
        *pos += n;
    }
    Ok(())
}

async fn read_body(
    socket: &mut TcpSocket<'_>,
    header: [u8; 2],
    payload: &mut [u8],
) -> Result<Option<(u8, usize)>, ()> {
    if header[0] & 0xf0 != 0x80 || header[1] & 0x80 == 0 {
        return Err(());
    }
    let opcode = header[0] & 0x0f;
    let len = (header[1] & 0x7f) as usize;
    if len > 125 || len > payload.len() {
        return Err(());
    }
    let mut mask = [0u8; 4];
    read_exact(socket, &mut mask).await?;
    read_exact(socket, &mut payload[..len]).await?;
    for (i, byte) in payload[..len].iter_mut().enumerate() {
        *byte ^= mask[i % 4];
    }
    if opcode == 8 {
        return Ok(None);
    }
    Ok(Some((opcode, len)))
}

async fn send_frame(socket: &mut TcpSocket<'_>, opcode: u8, payload: &[u8]) -> Result<(), ()> {
    if payload.len() > 125 {
        return Err(());
    }
    write_all(socket, &[0x80 | opcode, payload.len() as u8]).await?;
    write_all(socket, payload).await
}

async fn read_exact(socket: &mut TcpSocket<'_>, mut bytes: &mut [u8]) -> Result<(), ()> {
    while !bytes.is_empty() {
        let n = socket.read(bytes).await.map_err(|_| ())?;
        if n == 0 {
            return Err(());
        }
        bytes = &mut bytes[n..];
    }
    Ok(())
}

async fn write_all(socket: &mut TcpSocket<'_>, mut bytes: &[u8]) -> Result<(), ()> {
    while !bytes.is_empty() {
        let n = socket.write(bytes).await.map_err(|_| ())?;
        if n == 0 {
            return Err(());
        }
        bytes = &bytes[n..];
    }
    Ok(())
}

fn base64(input: &[u8], output: &mut [u8; 28]) {
    const TABLE: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (chunk, out) in input.chunks(3).zip(output.chunks_mut(4)) {
        let a = chunk[0];
        let b = chunk.get(1).copied().unwrap_or(0);
        let c = chunk.get(2).copied().unwrap_or(0);
        out[0] = TABLE[(a >> 2) as usize];
        out[1] = TABLE[(((a & 3) << 4) | (b >> 4)) as usize];
        out[2] = if chunk.len() > 1 {
            TABLE[(((b & 15) << 2) | (c >> 6)) as usize]
        } else {
            b'='
        };
        out[3] = if chunk.len() > 2 {
            TABLE[(c & 63) as usize]
        } else {
            b'='
        };
    }
}
