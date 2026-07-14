//! Receive filters: `id:mask` pairs, applied to the raw id word.
//!
//! Filters arrive either as the `filter` query parameter on the WebSocket URL
//! (`?filter=0x181:0x7FF,512:1792`) or at runtime as a JSON text message
//! ([`ClientCommand`]).

/// A single `id:mask` receive filter (socketCAN semantics: a frame passes if
/// `frame_id & mask == id & mask`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFilter {
    pub id: u32,
    pub mask: u32,
}

impl CanFilter {
    pub fn matches(&self, frame_id: u32) -> bool {
        (frame_id & self.mask) == (self.id & self.mask)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterParseErrorKind {
    /// Entry is not of the form `id:mask`.
    MissingColon,
    /// The id or mask is not a valid decimal or `0x` hex number.
    BadNumber,
}

/// Parse error, carrying the offending `id:mask` entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FilterParseError<'a> {
    pub entry: &'a str,
    pub kind: FilterParseErrorKind,
}

impl core::fmt::Display for FilterParseError<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.kind {
            FilterParseErrorKind::MissingColon => {
                write!(f, "expected id:mask, got '{}'", self.entry)
            }
            FilterParseErrorKind::BadNumber => {
                write!(f, "invalid number in '{}'", self.entry)
            }
        }
    }
}

/// Parse a `id:mask,id:mask,...` filter string without allocating.
///
/// An empty string yields no filters. Numbers are decimal or `0x` hex.
pub fn parse_filters(s: &str) -> impl Iterator<Item = Result<CanFilter, FilterParseError<'_>>> {
    s.split(',').filter(|e| !e.is_empty()).map(|entry| {
        let (id_s, mask_s) = entry.split_once(':').ok_or(FilterParseError {
            entry,
            kind: FilterParseErrorKind::MissingColon,
        })?;
        let bad_number = FilterParseError {
            entry,
            kind: FilterParseErrorKind::BadNumber,
        };
        Ok(CanFilter {
            id: parse_hex_or_dec(id_s).ok_or(bad_number)?,
            mask: parse_hex_or_dec(mask_s).ok_or(bad_number)?,
        })
    })
}

/// Convenience wrapper collecting [`parse_filters`] into a `Vec`, with the
/// error rendered as a `String`.
#[cfg(feature = "alloc")]
pub fn parse_filter_param(s: &str) -> Result<alloc::vec::Vec<CanFilter>, alloc::string::String> {
    use alloc::string::ToString;
    parse_filters(s)
        .collect::<Result<_, _>>()
        .map_err(|e| e.to_string())
}

fn parse_hex_or_dec(s: &str) -> Option<u32> {
    let s = s.trim();
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u32::from_str_radix(hex, 16).ok()
    } else {
        s.parse::<u32>().ok()
    }
}

/// Text-frame command for changing the receive filter at runtime.
#[cfg(all(feature = "serde", feature = "alloc"))]
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "cmd")]
pub enum ClientCommand {
    #[serde(rename = "set_filter")]
    SetFilter {
        filter: alloc::vec::Vec<FilterEntry>,
    },
    #[serde(rename = "clear_filter")]
    ClearFilter,
}

/// JSON representation of one filter in a [`ClientCommand`].
#[cfg(feature = "serde")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct FilterEntry {
    pub id: u32,
    pub mask: u32,
}

#[cfg(feature = "serde")]
impl From<&FilterEntry> for CanFilter {
    fn from(e: &FilterEntry) -> Self {
        CanFilter {
            id: e.id,
            mask: e.mask,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_hex_and_decimal_filters() {
        let filters = parse_filter_param("0x181:0x7ff,512:1792").unwrap();
        assert_eq!(
            filters,
            [
                CanFilter {
                    id: 0x181,
                    mask: 0x7ff
                },
                CanFilter {
                    id: 512,
                    mask: 1792
                }
            ]
        );
    }

    #[test]
    fn empty_string_yields_no_filters() {
        assert_eq!(parse_filter_param("").unwrap(), []);
    }

    #[test]
    fn rejects_malformed_filters() {
        assert_eq!(
            parse_filter_param("181").unwrap_err(),
            "expected id:mask, got '181'"
        );
        assert_eq!(
            parse_filter_param("0xgg:0x7ff").unwrap_err(),
            "invalid number in '0xgg:0x7ff'"
        );
    }

    #[test]
    fn matches_uses_mask() {
        let f = CanFilter {
            id: 0x181,
            mask: 0x7FF,
        };
        assert!(f.matches(0x181));
        assert!(!f.matches(0x182));
        let any_node_tpdo1 = CanFilter {
            id: 0x180,
            mask: 0x780,
        };
        assert!(any_node_tpdo1.matches(0x1A5));
    }
}
