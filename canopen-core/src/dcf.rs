//! Runtime DCF/EDS parser — load configuration from text at runtime.
//!
//! A DCF (Device Configuration File) is an EDS with actual configured values.
//! This module parses DCF/EDS text and can apply the configured values to:
//! - A local `ObjectDictionary` (e.g. at startup)
//! - A remote node via `SdoDriver` (e.g. during network configuration)
//!
//! Requires the `alloc` feature.
//!
//! ```ignore
//! let dcf = Dcf::parse(include_str!("config.dcf"))?;
//! dcf.apply(&mut od)?;
//! ```

use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::od::{ObjectDictionary, OdError};
use crate::sdo::driver::{AsyncCan, SdoDriver, SdoError};

/// A single configured OD entry from a DCF file.
#[derive(Debug, Clone)]
pub struct DcfEntry {
    pub index: u16,
    pub subindex: u8,
    pub data: Vec<u8>,
    pub name: String,
}

/// Parsed DCF/EDS configuration.
#[derive(Debug, Clone)]
pub struct Dcf {
    /// Node ID from [DeviceComissioning], if present.
    pub node_id: Option<u8>,
    /// Baud rate index from [DeviceComissioning], if present.
    pub baud_rate: Option<u32>,
    /// Configured entries (index, subindex, serialized value).
    pub entries: Vec<DcfEntry>,
}

/// Errors from DCF parsing or application.
#[derive(Debug)]
pub enum DcfError {
    /// Failed to parse a value in the DCF.
    ParseError(String),
    /// OD write failed for an entry.
    OdError {
        index: u16,
        subindex: u8,
        error: OdError,
    },
}

impl core::fmt::Display for DcfError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::ParseError(msg) => write!(f, "DCF parse error: {}", msg),
            Self::OdError {
                index,
                subindex,
                error,
            } => {
                write!(
                    f,
                    "OD write failed at 0x{:04X}:{}: {:?}",
                    index, subindex, error
                )
            }
        }
    }
}

impl Dcf {
    /// Parse a DCF or EDS file content.
    ///
    /// For DCF files, uses `ParameterValue` for configured values.
    /// For EDS files, falls back to `DefaultValue`.
    pub fn parse(content: &str) -> Result<Self, DcfError> {
        let sections = parse_ini_sections(content);

        // Read DeviceComissioning (DCF-specific)
        let node_id = find_section(&sections, "DeviceComissioning")
            .and_then(|props| get_prop(props, "NodeID"))
            .and_then(|v| parse_int_val(&v).map(|n| n as u8));

        let baud_rate = find_section(&sections, "DeviceComissioning")
            .and_then(|props| get_prop(props, "BaudRate"))
            .and_then(|v| parse_int_val(&v).map(|n| n as u32));

        let mut entries = Vec::new();

        for (section_name, props) in &sections {
            let Some((index, subindex)) = parse_section_index(section_name) else {
                continue;
            };

            // Skip PDO comm/mapping indices — those are handled by the stack
            if is_pdo_index(index) {
                continue;
            }

            // Skip sub0 (number of entries) for records/arrays
            if subindex.is_some() && subindex == Some(0) {
                continue;
            }

            let sub = subindex.unwrap_or(0);

            // Get the data type
            let Some(data_type) = get_prop(props, "DataType").and_then(|v| parse_int_val(&v))
            else {
                continue;
            };

            // Get the value: prefer ParameterValue (DCF), fall back to DefaultValue (EDS)
            let value_str =
                get_prop(props, "ParameterValue").or_else(|| get_prop(props, "DefaultValue"));

            let Some(value_str) = value_str else {
                continue;
            };

            // Skip $NODEID expressions and empty values
            let trimmed = value_str.trim();
            if trimmed.is_empty() || trimmed.contains("$NODEID") || trimmed.contains("$nodeid") {
                continue;
            }

            // Serialize the value to bytes based on data type
            if let Some(data) = serialize_value(trimmed, data_type as u16) {
                let name = get_prop(props, "ParameterName").unwrap_or_default();
                entries.push(DcfEntry {
                    index,
                    subindex: sub,
                    data,
                    name,
                });
            }
        }

        Ok(Dcf {
            node_id,
            baud_rate,
            entries,
        })
    }

    /// Apply configured values to a local ObjectDictionary.
    ///
    /// Skips entries that fail to write (e.g. read-only). Returns the count of
    /// entries successfully applied.
    pub fn apply<OD: ObjectDictionary>(&self, od: &mut OD) -> usize {
        let mut count = 0;
        for entry in &self.entries {
            if od.write(entry.index, entry.subindex, &entry.data).is_ok() {
                count += 1;
            }
        }
        count
    }

    /// Apply configured values to a local ObjectDictionary, failing on first error.
    pub fn apply_strict<OD: ObjectDictionary>(&self, od: &mut OD) -> Result<usize, DcfError> {
        let mut count = 0;
        for entry in &self.entries {
            od.write(entry.index, entry.subindex, &entry.data)
                .map_err(|e| DcfError::OdError {
                    index: entry.index,
                    subindex: entry.subindex,
                    error: e,
                })?;
            count += 1;
        }
        Ok(count)
    }

    /// Write configured values to a remote node via SDO.
    ///
    /// Skips entries that fail. Returns the count of entries successfully written.
    pub async fn apply_remote<E: core::fmt::Debug>(
        &self,
        driver: &SdoDriver,
        can: &mut impl AsyncCan<Error = E>,
    ) -> Result<usize, SdoError<E>> {
        let mut count = 0;
        for entry in &self.entries {
            driver
                .download(entry.index, entry.subindex, &entry.data, can)
                .await?;
            count += 1;
        }
        Ok(count)
    }
}

// ---- INI parsing helpers ----

fn parse_ini_sections(content: &str) -> Vec<(String, Vec<(String, String)>)> {
    let mut sections = Vec::new();
    let mut current_section = String::new();
    let mut section_props: Vec<(String, String)> = Vec::new();

    for line in content.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with(';') {
            continue;
        }
        if line.starts_with('[') && line.ends_with(']') {
            if !current_section.is_empty() {
                sections.push((current_section.clone(), section_props.clone()));
                section_props.clear();
            }
            current_section = line[1..line.len() - 1].to_string();
        } else if let Some((key, value)) = line.split_once('=') {
            section_props.push((key.trim().to_string(), value.trim().to_string()));
        }
    }
    if !current_section.is_empty() {
        sections.push((current_section, section_props));
    }
    sections
}

fn find_section<'a>(
    sections: &'a [(String, Vec<(String, String)>)],
    name: &str,
) -> Option<&'a Vec<(String, String)>> {
    sections
        .iter()
        .find(|(n, _)| n.eq_ignore_ascii_case(name))
        .map(|(_, props)| props)
}

fn get_prop(props: &[(String, String)], key: &str) -> Option<String> {
    props
        .iter()
        .find(|(k, _)| k.eq_ignore_ascii_case(key))
        .map(|(_, v)| v.clone())
}

fn parse_int_val(s: &str) -> Option<u64> {
    let s = s.trim();
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(hex, 16).ok()
    } else {
        s.parse().ok()
    }
}

fn parse_section_index(section: &str) -> Option<(u16, Option<u8>)> {
    let lower = section.to_lowercase();
    if let Some((idx_str, sub_str)) = lower.split_once("sub") {
        let index = u16::from_str_radix(idx_str, 16).ok()?;
        let subindex = u8::from_str_radix(sub_str, 16)
            .ok()
            .or_else(|| sub_str.parse().ok())?;
        Some((index, Some(subindex)))
    } else {
        let index = u16::from_str_radix(&lower, 16).ok()?;
        if index >= 0x1000 {
            Some((index, None))
        } else {
            None
        }
    }
}

fn is_pdo_index(index: u16) -> bool {
    matches!(index, 0x1400..=0x15FF | 0x1600..=0x17FF | 0x1800..=0x19FF | 0x1A00..=0x1BFF)
}

/// Serialize a text value to LE bytes based on CiA 301 data type code.
fn serialize_value(value: &str, data_type: u16) -> Option<Vec<u8>> {
    let v = value.trim().trim_matches('"');
    match data_type {
        // Boolean
        0x0001 => {
            let b = match v {
                "0" | "false" => 0u8,
                _ => 1u8,
            };
            Some(alloc::vec![b])
        }
        // Integer8 / Unsigned8
        0x0002 | 0x0005 => parse_int_val(v).map(|n| alloc::vec![n as u8]),
        // Integer16 / Unsigned16
        0x0003 | 0x0006 => parse_int_val(v).map(|n| (n as u16).to_le_bytes().to_vec()),
        // Integer32 / Unsigned32
        0x0004 | 0x0007 => parse_int_val(v).map(|n| (n as u32).to_le_bytes().to_vec()),
        // Real32
        0x0008 => {
            if let Some(hex) = v.strip_prefix("0x").or_else(|| v.strip_prefix("0X")) {
                // Hex = IEEE 754 bit pattern
                u32::from_str_radix(hex, 16)
                    .ok()
                    .map(|bits| bits.to_le_bytes().to_vec())
            } else {
                // Decimal float
                v.parse::<f32>().ok().map(|f| f.to_le_bytes().to_vec())
            }
        }
        // Visible String
        0x0009 => Some(v.as_bytes().to_vec()),
        // Octet String / Domain
        0x000A | 0x000F => {
            // Try hex, otherwise raw bytes
            if let Some(hex) = v.strip_prefix("0x").or_else(|| v.strip_prefix("0X")) {
                let bytes: Option<Vec<u8>> = (0..hex.len())
                    .step_by(2)
                    .map(|i| u8::from_str_radix(&hex[i..i + 2.min(hex.len() - i)], 16).ok())
                    .collect();
                bytes
            } else {
                Some(v.as_bytes().to_vec())
            }
        }
        // Real64
        0x0011 => {
            if let Some(hex) = v.strip_prefix("0x").or_else(|| v.strip_prefix("0X")) {
                u64::from_str_radix(hex, 16)
                    .ok()
                    .map(|bits| bits.to_le_bytes().to_vec())
            } else {
                v.parse::<f64>().ok().map(|f| f.to_le_bytes().to_vec())
            }
        }
        // Integer64 / Unsigned64
        0x0015 | 0x001B => parse_int_val(v).map(|n| n.to_le_bytes().to_vec()),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datatypes::DataType;
    use crate::od::*;

    struct DcfTestOd {
        device_type: u32,
        error_register: u8,
        heartbeat_time: u16,
        controlword: u16,
        statusword: u16,
    }

    impl DcfTestOd {
        fn new() -> Self {
            Self {
                device_type: 0x191,
                error_register: 0,
                heartbeat_time: 100,
                controlword: 0,
                statusword: 0,
            }
        }
    }

    static DCF_TEST_META: &[OdEntryMeta] = &[
        OdEntryMeta {
            index: 0x1000,
            subindex: 0,
            data_type: DataType::U32,
            access: AccessType::Ro,
            pdo_mappable: false,
            name: "device_type",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x1001,
            subindex: 0,
            data_type: DataType::U8,
            access: AccessType::Ro,
            pdo_mappable: false,
            name: "error_register",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x1017,
            subindex: 0,
            data_type: DataType::U16,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "heartbeat_time",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x6040,
            subindex: 0,
            data_type: DataType::U16,
            access: AccessType::Rw,
            pdo_mappable: false,
            name: "controlword",
            max_size: None,
        },
        OdEntryMeta {
            index: 0x6041,
            subindex: 0,
            data_type: DataType::U16,
            access: AccessType::Ro,
            pdo_mappable: false,
            name: "statusword",
            max_size: None,
        },
    ];

    impl ObjectDictionary for DcfTestOd {
        fn lookup(&self, index: u16, subindex: u8) -> Option<&'static OdEntryMeta> {
            DCF_TEST_META
                .iter()
                .find(|e| e.index == index && e.subindex == subindex)
        }
        fn read(&self, index: u16, _sub: u8, buf: &mut [u8]) -> Result<usize, OdError> {
            match index {
                0x1000 => {
                    buf[..4].copy_from_slice(&self.device_type.to_le_bytes());
                    Ok(4)
                }
                0x1001 => {
                    buf[0] = self.error_register;
                    Ok(1)
                }
                0x1017 => {
                    buf[..2].copy_from_slice(&self.heartbeat_time.to_le_bytes());
                    Ok(2)
                }
                0x6040 => {
                    buf[..2].copy_from_slice(&self.controlword.to_le_bytes());
                    Ok(2)
                }
                0x6041 => {
                    buf[..2].copy_from_slice(&self.statusword.to_le_bytes());
                    Ok(2)
                }
                _ => Err(OdError::NotFound),
            }
        }
        fn write(&mut self, index: u16, _sub: u8, data: &[u8]) -> Result<(), OdError> {
            match index {
                0x1000 | 0x1001 | 0x6041 => Err(OdError::ReadOnly),
                0x1017 => {
                    self.heartbeat_time = u16::from_le_bytes([data[0], data[1]]);
                    Ok(())
                }
                0x6040 => {
                    self.controlword = u16::from_le_bytes([data[0], data[1]]);
                    Ok(())
                }
                _ => Err(OdError::NotFound),
            }
        }
        fn sub_count(&self, _: u16) -> Option<u8> {
            Some(0)
        }
    }

    const TEST_DCF: &str = "\
[FileInfo]
FileName=test.dcf
FileVersion=1

[DeviceComissioning]
NodeID=5
BaudRate=5

[MandatoryObjects]
SupportedObjects=2
1=0x1000
2=0x1001

[1000]
ParameterName=Device Type
ObjectType=0x7
DataType=0x0007
AccessType=ro
DefaultValue=0x191

[1001]
ParameterName=Error Register
ObjectType=0x7
DataType=0x0005
AccessType=ro
DefaultValue=0

[1017]
ParameterName=Producer Heartbeat Time
ObjectType=0x7
DataType=0x0006
AccessType=rw
ParameterValue=500

[6040]
ParameterName=Controlword
ObjectType=0x7
DataType=0x0006
AccessType=rw
ParameterValue=0x000F
";

    #[test]
    fn parse_dcf_node_id() {
        let dcf = Dcf::parse(TEST_DCF).unwrap();
        assert_eq!(dcf.node_id, Some(5));
        assert_eq!(dcf.baud_rate, Some(5));
    }

    #[test]
    fn parse_dcf_entries() {
        let dcf = Dcf::parse(TEST_DCF).unwrap();
        // Should have entries for 1000, 1017, 6040
        // (1001 has no ParameterValue and AccessType=ro, but DefaultValue exists)
        assert!(dcf.entries.len() >= 2);

        // Check heartbeat time entry
        let hb = dcf.entries.iter().find(|e| e.index == 0x1017).unwrap();
        assert_eq!(u16::from_le_bytes(hb.data[..2].try_into().unwrap()), 500);

        // Check controlword entry
        let cw = dcf.entries.iter().find(|e| e.index == 0x6040).unwrap();
        assert_eq!(u16::from_le_bytes(cw.data[..2].try_into().unwrap()), 0x000F);
    }

    #[test]
    fn apply_dcf_to_od() {
        let dcf = Dcf::parse(TEST_DCF).unwrap();
        let mut od = DcfTestOd::new();

        assert_eq!(od.heartbeat_time, 100); // original default
        let count = dcf.apply(&mut od);
        assert!(count >= 2);

        assert_eq!(od.heartbeat_time, 500); // DCF value applied
        assert_eq!(od.controlword, 0x000F);
    }

    #[test]
    fn apply_dcf_skips_readonly() {
        let dcf = Dcf::parse(TEST_DCF).unwrap();
        let mut od = DcfTestOd::new();

        let original_device_type = od.device_type;
        dcf.apply(&mut od);
        // device_type is ro — should not change
        assert_eq!(od.device_type, original_device_type);
    }

    #[test]
    fn serialize_float_hex() {
        // 0x447a0000 = 1000.0f32
        let data = serialize_value("0x447a0000", 0x0008).unwrap();
        let f = f32::from_le_bytes(data[..4].try_into().unwrap());
        assert!((f - 1000.0).abs() < 0.01);
    }

    #[test]
    fn serialize_float_decimal() {
        let data = serialize_value("3.14", 0x0008).unwrap();
        let f = f32::from_le_bytes(data[..4].try_into().unwrap());
        assert!((f - 3.14).abs() < 0.01);
    }

    #[test]
    fn serialize_string() {
        let data = serialize_value("\"Hello\"", 0x0009).unwrap();
        assert_eq!(data, b"Hello");
    }
}
