//! candump-compatible frame rendering.
//!
//! The text output mirrors can-utils `candump`'s "long" frame format
//! (`snprintf_long_canframe` in `lib.c`) plus the surrounding line assembly
//! in `candump.c`, restricted to classic CAN (the only thing canwsd carries):
//!
//! ```text
//!   can0  123   [8]  DE AD BE EF DE AD BE EF
//!  (0000000000.000000)  can0  123   [8]  11 22 33
//! ```
//!
//! **Timestamp caveat:** the canwsd wire format carries no timestamp, so every
//! `-t` value is the *client-side arrival time* (when the frame came off the
//! WebSocket), not a kernel/bus RX time. It is subject to network latency and
//! buffering — good for "are messages flowing", not for bus-accurate timing.

use std::time::{Duration, UNIX_EPOCH};

use canwsd_proto::WireFrame;

const HEX: &[u8; 16] = b"0123456789ABCDEF";

/// candump `CAN_ERR_MASK | CAN_ERR_FLAG` — the bits shown for an error frame's
/// "id" (`lib.c`).
const ERR_ID_BITS: u32 = 0x3FFF_FFFF;

/// Which timestamp column to print, mirroring candump's `-t` argument.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum TimestampMode {
    /// `-t a`: absolute Unix time, `(<secs>.<usecs>)`.
    Absolute,
    /// `-t d`: delta since the previous frame.
    Delta,
    /// `-t z`: elapsed since the first frame.
    Zero,
    /// `-t A`: absolute local date+time, `(YYYY-MM-DD HH:MM:SS.<usecs>)`.
    AbsoluteDate,
}

impl std::str::FromStr for TimestampMode {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "a" => Ok(Self::Absolute),
            "d" => Ok(Self::Delta),
            "z" => Ok(Self::Zero),
            "A" => Ok(Self::AbsoluteDate),
            other => Err(format!("invalid timestamp type '{other}' (expected a, d, z or A)")),
        }
    }
}

/// Stateful timestamp renderer. Delta/zero need the previous/first reception
/// time, so this is not a pure function — call [`Timestamps::render`] once per
/// frame in arrival order.
pub struct Timestamps {
    mode: Option<TimestampMode>,
    /// Reference time for delta/zero; `None` until the first frame, matching
    /// candump's `last_ts.tv_sec == 0` first-init.
    last: Option<Duration>,
}

impl Timestamps {
    pub fn new(mode: Option<TimestampMode>) -> Self {
        Self { mode, last: None }
    }

    /// Render the timestamp column (including the trailing space candump adds)
    /// for a frame received at `now`. Empty string when no `-t` was given.
    pub fn render(&mut self, now: Duration) -> String {
        let Some(mode) = self.mode else {
            return String::new();
        };
        match mode {
            TimestampMode::Absolute => {
                format!("({:010}.{:06}) ", now.as_secs(), now.subsec_micros())
            }
            TimestampMode::AbsoluteDate => {
                let dt: chrono::DateTime<chrono::Local> = (UNIX_EPOCH + now).into();
                format!("({}.{:06}) ", dt.format("%Y-%m-%d %H:%M:%S"), now.subsec_micros())
            }
            TimestampMode::Delta | TimestampMode::Zero => {
                // candump: first frame initialises the reference to itself, so
                // the first delta/zero reads (000.000000).
                let reference = *self.last.get_or_insert(now);
                let diff = now.saturating_sub(reference);
                if mode == TimestampMode::Delta {
                    self.last = Some(now);
                }
                format!("({:03}.{:06}) ", diff.as_secs(), diff.subsec_micros())
            }
        }
    }
}

/// Assemble the full candump line (without the trailing newline) for one frame:
/// `[timestamp] <iface>  <frame body>`, matching candump's `-t`/device-name
/// layout for a single interface.
pub fn format_line(ts: &str, iface: &str, wf: &WireFrame, binary: bool, ascii: bool) -> String {
    // candump.c: " " + timestamp + " " + devname + "  " + frame body.
    format!(" {ts} {iface}  {}", frame_body(wf, binary, ascii))
}

/// Render one frame in candump's long format (the part after the device name).
///
/// Direct port of the classic-CAN path of `snprintf_long_canframe`; the wire
/// format guarantees DLC ≤ 8, so the CAN FD / XL branches are omitted.
fn frame_body(wf: &WireFrame, binary: bool, ascii: bool) -> String {
    let len = wf.dlc() as usize;
    let data = wf.data();
    let dlen = if binary { 9 } else { 3 };

    // candump reserves and space-fills the id + length field, then overwrites.
    let mut buf: Vec<u8> = vec![b' '; 15];

    let is_err = wf.is_error();
    let offset = if is_err {
        put_id(&mut buf, 7, wf.id_word() & ERR_ID_BITS);
        10
    } else if wf.is_extended() {
        put_id(&mut buf, 7, wf.id());
        10
    } else {
        put_id(&mut buf, 2, wf.id());
        5
    };

    // Classic-CAN length: "[<len>]".
    buf[offset + 1] = b'[';
    buf[offset + 2] = b'0' + len as u8;
    buf[offset + 3] = b']';

    // RTR frames show the request marker instead of any data.
    if wf.is_rtr() {
        let mut out = buf;
        out.truncate(offset + 5);
        out.extend_from_slice(b" remote request");
        return String::from_utf8(out).unwrap();
    }

    let mut offset = offset + 5;
    for &byte in &data[..len] {
        set_at(&mut buf, &mut offset, b' ');
        if binary {
            for j in (0..8).rev() {
                set_at(&mut buf, &mut offset, if byte & (1 << j) != 0 { b'1' } else { b'0' });
            }
        } else {
            set_at(&mut buf, &mut offset, HEX[(byte >> 4) as usize]);
            set_at(&mut buf, &mut offset, HEX[(byte & 0x0F) as usize]);
        }
    }
    buf.truncate(offset);

    // Fixed-column suffix behind the data (candump aligns to 8 bytes).
    if is_err {
        append_right_justified(&mut buf, dlen * (8 - len) + 13, "ERRORFRAME");
    } else if ascii {
        append_right_justified(&mut buf, dlen * (8 - len) + 4, "'");
        for &byte in &data[..len] {
            buf.push(if (0x20..0x7F).contains(&byte) { byte } else { b'.' });
        }
        buf.push(b'\'');
    }

    String::from_utf8(buf).unwrap()
}

/// Render one frame as a single JSON object (one line of JSONL output).
///
/// `t` is the client arrival time in seconds (see the timestamp caveat above),
/// `id` the numeric CAN id, with `ext`/`rtr`/`err` breaking out the flag bits,
/// and `data` the uppercase-hex payload (empty for RTR frames).
pub fn format_jsonl(now: Duration, wf: &WireFrame) -> String {
    let id = if wf.is_error() {
        wf.id_word() & ERR_ID_BITS
    } else {
        wf.id()
    };
    let data = if wf.is_rtr() {
        String::new()
    } else {
        wf.data().iter().fold(String::new(), |mut s, b| {
            s.push(HEX[(b >> 4) as usize] as char);
            s.push(HEX[(b & 0x0F) as usize] as char);
            s
        })
    };
    format!(
        "{{\"t\":{}.{:06},\"id\":{},\"ext\":{},\"rtr\":{},\"err\":{},\"dlc\":{},\"data\":\"{}\"}}",
        now.as_secs(),
        now.subsec_micros(),
        id,
        wf.is_extended(),
        wf.is_rtr(),
        wf.is_error(),
        wf.dlc(),
        data,
    )
}

/// Write a zero-padded uppercase-hex id ending at `end_offset` (2 → 3 SFF
/// digits, 7 → 8 EFF digits), returning the byte offset just past the id field
/// (candump uses 5 for SFF, 10 for EFF). Port of `_put_id`.
fn put_id(buf: &mut [u8], end_offset: usize, mut id: u32) -> usize {
    let mut i = end_offset as isize;
    while i >= 0 {
        buf[i as usize] = HEX[(id & 0x0F) as usize];
        id >>= 4;
        i -= 1;
    }
    if end_offset == 7 { 10 } else { 5 }
}

/// Overwrite the space-filled reservation while it lasts, then append. Mirrors
/// candump indexing `buf[offset++]` into its pre-filled buffer.
fn set_at(buf: &mut Vec<u8>, offset: &mut usize, byte: u8) {
    if *offset < buf.len() {
        buf[*offset] = byte;
    } else {
        buf.push(byte);
    }
    *offset += 1;
}

/// `sprintf("%*s", width, s)`: right-justify `s` in a field of `width`.
fn append_right_justified(buf: &mut Vec<u8>, width: usize, s: &str) {
    for _ in 0..width.saturating_sub(s.len()) {
        buf.push(b' ');
    }
    buf.extend_from_slice(s.as_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;
    use canwsd_proto::wire::{CAN_EFF_FLAG, CAN_ERR_FLAG, CAN_RTR_FLAG};

    fn sff(id: u32, data: &[u8]) -> WireFrame {
        WireFrame::new(id, data).unwrap()
    }

    #[test]
    fn standard_frame_hex() {
        let f = sff(0x123, &[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]);
        assert_eq!(frame_body(&f, false, false), "123   [8]  11 22 33 44 55 66 77 88");
    }

    #[test]
    fn short_frame_and_padding() {
        let f = sff(0x7, &[0xAB]);
        // id zero-padded to 3 digits, "[1]" then two spaces before the byte.
        assert_eq!(frame_body(&f, false, false), "007   [1]  AB");
    }

    #[test]
    fn ascii_view_aligns_and_dots_nonprintable() {
        let f = sff(0x123, &[0x41, 0x42, 0x00, 0x7F]);
        // dlen*(8-len)+4 = 3*4+4 = 16-wide right-justified "'" behind the data.
        let body = frame_body(&f, false, true);
        assert_eq!(body, "123   [4]  41 42 00 7F               'AB..'");
    }

    #[test]
    fn binary_view() {
        let f = sff(0x1, &[0xAA, 0x0F]);
        assert_eq!(frame_body(&f, true, false), "001   [2]  10101010 00001111");
    }

    #[test]
    fn extended_frame() {
        let f = WireFrame::new(CAN_EFF_FLAG | 0x1234_5678, &[0xDE, 0xAD]).unwrap();
        assert_eq!(frame_body(&f, false, false), "12345678   [2]  DE AD");
    }

    #[test]
    fn rtr_frame() {
        let f = WireFrame::new(CAN_RTR_FLAG | 0x123, &[0, 0, 0]).unwrap();
        assert_eq!(frame_body(&f, false, false), "123   [3]  remote request");
    }

    #[test]
    fn error_frame_suffix() {
        let f = WireFrame::new(CAN_ERR_FLAG | 0x40, &[0, 0, 0, 0, 0, 0, 0, 0]).unwrap();
        let body = frame_body(&f, false, false);
        assert!(body.starts_with("20000040   [8]  00 00 00 00 00 00 00 00"));
        assert!(body.ends_with("ERRORFRAME"));
    }

    #[test]
    fn line_layout_without_timestamp() {
        let f = sff(0x123, &[0x11]);
        assert_eq!(format_line("", "can0", &f, false, false), "  can0  123   [1]  11");
    }

    #[test]
    fn line_layout_with_timestamp() {
        let f = sff(0x123, &[0x11]);
        let line = format_line("(0000000001.000002) ", "can0", &f, false, false);
        assert_eq!(line, " (0000000001.000002)  can0  123   [1]  11");
    }

    #[test]
    fn delta_first_frame_is_zero_then_counts() {
        let mut ts = Timestamps::new(Some(TimestampMode::Delta));
        assert_eq!(ts.render(Duration::from_micros(5_000_000)), "(000.000000) ");
        assert_eq!(ts.render(Duration::from_micros(5_250_000)), "(000.250000) ");
    }

    #[test]
    fn zero_is_relative_to_first() {
        let mut ts = Timestamps::new(Some(TimestampMode::Zero));
        assert_eq!(ts.render(Duration::from_micros(5_000_000)), "(000.000000) ");
        assert_eq!(ts.render(Duration::from_micros(6_500_000)), "(001.500000) ");
    }

    #[test]
    fn jsonl_record() {
        let f = sff(0x123, &[0xDE, 0xAD]);
        assert_eq!(
            format_jsonl(Duration::from_micros(1_500_000_000_123_456), &f),
            "{\"t\":1500000000.123456,\"id\":291,\"ext\":false,\"rtr\":false,\"err\":false,\"dlc\":2,\"data\":\"DEAD\"}"
        );
    }
}
