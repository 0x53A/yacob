use crate::transport::CanFrame;

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

/// Check if a byte buffer contains what looks like a valid SLCAN frame.
///
/// Useful for probing whether an adapter is already streaming.
pub fn has_slcan_frame(buf: &[u8]) -> bool {
    for window in buf.windows(5) {
        if window[0] == b't'
            && parse_hex_digit(window[1]).is_some()
            && parse_hex_digit(window[2]).is_some()
            && parse_hex_digit(window[3]).is_some()
            && window[4] >= b'0'
            && window[4] <= b'8'
        {
            return true;
        }
    }
    false
}

/// Encode a classic CANopen frame as a standard SLCAN `tIIILDD..\r` line.
///
/// Returns the number of bytes written into `out`.
pub fn encode_slcan_frame(frame: &CanFrame, out: &mut [u8]) -> Option<usize> {
    let needed = 6 + frame.data().len() * 2;
    if out.len() < needed {
        return None;
    }

    let id = frame.raw_id();
    let data = frame.data();

    let mut pos = 0;
    out[pos] = b't';
    pos += 1;
    out[pos] = hex_digit((id >> 8) as u8 & 0x0F);
    pos += 1;
    out[pos] = hex_digit((id >> 4) as u8 & 0x0F);
    pos += 1;
    out[pos] = hex_digit(id as u8 & 0x0F);
    pos += 1;
    out[pos] = b'0' + frame.raw_dlc();
    pos += 1;
    for &b in data {
        out[pos] = hex_digit(b >> 4);
        pos += 1;
        out[pos] = hex_digit(b & 0x0F);
        pos += 1;
    }
    out[pos] = b'\r';
    pos += 1;

    Some(pos)
}

/// Parse a standard SLCAN `tIIILDD..` line.
///
/// The trailing `\r` is optional. Extended and remote frames are ignored
/// because CANopen uses standard data frames.
pub fn parse_slcan_frame(line: &[u8]) -> Option<CanFrame> {
    if line.is_empty() {
        return None;
    }
    match line[0] {
        b't' => {
            if line.len() < 5 {
                return None;
            }
            let id = (parse_hex_digit(line[1])? as u16) << 8
                | (parse_hex_digit(line[2])? as u16) << 4
                | parse_hex_digit(line[3])? as u16;
            let dlc = (line[4] - b'0') as usize;
            if dlc > 8 {
                return None;
            }
            if line.len() < 5 + dlc * 2 {
                return None;
            }
            let mut data = [0u8; 8];
            for i in 0..dlc {
                data[i] =
                    (parse_hex_digit(line[5 + i * 2])? << 4) | parse_hex_digit(line[6 + i * 2])?;
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

fn parse_hex_digit(ch: u8) -> Option<u8> {
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

    #[test]
    fn encode_standard_frame() {
        let frame = CanFrame::new(0x181, &[0x12, 0xAB]).unwrap();
        let mut out = [0u8; 32];
        let len = encode_slcan_frame(&frame, &mut out).unwrap();
        assert_eq!(&out[..len], b"t181212AB\r");
    }
}
