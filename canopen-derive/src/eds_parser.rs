use crate::dsl::{AccessKind, EntryKind, OdDefinition, OdEntry, SubEntry, VarDef};
use proc_macro2::Span;
use syn::parse::{Parse, ParseStream};
use syn::{Ident, LitStr, Result, Token, Visibility};

/// Parsed input for `object_dictionary_from_eds!`.
pub struct EdsDefinition {
    pub vis: Visibility,
    pub name: Ident,
    pub file_path: String,
}

impl Parse for EdsDefinition {
    fn parse(input: ParseStream) -> Result<Self> {
        let vis: Visibility = input.parse()?;
        input.parse::<Token![struct]>()?;
        let name: Ident = input.parse()?;
        input.parse::<Token![=]>()?;
        let path_lit: LitStr = input.parse()?;
        let _ = input.parse::<Token![;]>(); // optional trailing semicolon
        Ok(EdsDefinition {
            vis,
            name,
            file_path: path_lit.value(),
        })
    }
}

/// Parse an EDS file and convert it to an OdDefinition for codegen.
pub fn parse_eds_to_od(def: EdsDefinition) -> std::result::Result<OdDefinition, syn::Error> {
    // Resolve path relative to CARGO_MANIFEST_DIR
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").map_err(|_| {
        syn::Error::new(Span::call_site(), "CARGO_MANIFEST_DIR not set")
    })?;
    let full_path = std::path::Path::new(&manifest_dir).join(&def.file_path);

    let content = std::fs::read_to_string(&full_path).map_err(|e| {
        syn::Error::new(
            Span::call_site(),
            format!("failed to read EDS file `{}`: {}", full_path.display(), e),
        )
    })?;

    let entries = parse_eds_content(&content).map_err(|e| {
        syn::Error::new(Span::call_site(), format!("EDS parse error: {e}"))
    })?;

    Ok(OdDefinition {
        vis: def.vis,
        name: def.name,
        entries,
    })
}

/// Parse EDS file content into OD entries.
fn parse_eds_content(content: &str) -> std::result::Result<Vec<OdEntry>, String> {
    let mut entries: Vec<OdEntry> = Vec::new();
    let mut current_section = String::new();
    let mut section_props: Vec<(String, String)> = Vec::new();

    // Collect all sections and their properties
    let mut sections: Vec<(String, Vec<(String, String)>)> = Vec::new();

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

    // Parse object sections (hex indices like "1000", "1018sub1", etc.)
    // First pass: collect top-level objects
    let mut record_subs: std::collections::HashMap<u16, Vec<(u8, Vec<(String, String)>)>> =
        std::collections::HashMap::new();

    for (section_name, props) in &sections {
        // Try to parse as a hex index
        if let Some((index, subindex)) = parse_section_index(section_name) {
            if let Some(sub) = subindex {
                record_subs
                    .entry(index)
                    .or_default()
                    .push((sub, props.clone()));
            } else {
                // Top-level object
                let obj_type = get_prop(props, "ObjectType")
                    .and_then(|v| parse_int(&v))
                    .unwrap_or(0x07); // default to VAR

                if obj_type == 0x07 {
                    // VAR object
                    if let Some(entry) = parse_var_entry(index, 0, props) {
                        entries.push(OdEntry {
                            index,
                            name: entry.name,
                            kind: EntryKind::Var(entry.var),
                        });
                    }
                }
                // Records/arrays will be assembled from sub-entries
            }
        }
    }

    // Second pass: assemble records from sub-entries
    for (index, subs) in &record_subs {
        let mut sub_entries = Vec::new();
        for (subindex, props) in subs {
            if let Some(entry) = parse_var_entry(*index, *subindex, props) {
                sub_entries.push(entry);
            }
        }
        if !sub_entries.is_empty() {
            sub_entries.sort_by_key(|s| s.subindex);

            // Find name from the main section
            let main_name = sections
                .iter()
                .find(|(name, _)| {
                    parse_section_index(name)
                        .map(|(idx, sub)| idx == *index && sub.is_none())
                        .unwrap_or(false)
                })
                .and_then(|(_, props)| get_prop(props, "ParameterName"))
                .unwrap_or_else(|| format!("obj_{:04x}", index));

            entries.push(OdEntry {
                index: *index,
                name: Ident::new(&sanitize_name(&main_name), Span::call_site()),
                kind: EntryKind::Record(sub_entries),
            });
        }
    }

    entries.sort_by_key(|e| e.index);
    Ok(entries)
}

fn parse_section_index(section: &str) -> Option<(u16, Option<u8>)> {
    let lower = section.to_lowercase();
    if let Some((idx_str, sub_str)) = lower.split_once("sub") {
        let index = u16::from_str_radix(idx_str, 16).ok()?;
        let subindex = u8::from_str_radix(sub_str, 16).ok().or_else(|| sub_str.parse().ok())?;
        Some((index, Some(subindex)))
    } else {
        let index = u16::from_str_radix(&lower, 16).ok()?;
        // Filter out non-object sections
        if index >= 0x1000 {
            Some((index, None))
        } else {
            None
        }
    }
}

fn parse_var_entry(_index: u16, subindex: u8, props: &[(String, String)]) -> Option<SubEntry> {
    let name_str = get_prop(props, "ParameterName")?;
    let data_type_code = get_prop(props, "DataType").and_then(|v| parse_int(&v))? as u16;
    let access_str = get_prop(props, "AccessType").unwrap_or_else(|| "ro".to_string());
    let default_str = get_prop(props, "DefaultValue").unwrap_or_else(|| "0".to_string());
    let pdo_mapping = get_prop(props, "PDOMapping")
        .and_then(|v| parse_int(&v))
        .unwrap_or(0)
        != 0;

    let (rust_type, _) = datatype_to_rust(data_type_code)?;
    let access = match access_str.to_lowercase().as_str() {
        "ro" | "const" => AccessKind::Ro,
        "rw" => AccessKind::Rw,
        "wo" => AccessKind::Wo,
        _ => AccessKind::Ro,
    };

    let default_expr: syn::Expr = syn::parse_str(&normalize_default(&default_str, data_type_code)).ok()?;

    Some(SubEntry {
        subindex,
        name: Ident::new(&sanitize_name(&name_str), Span::call_site()),
        var: VarDef {
            type_name: Ident::new(rust_type, Span::call_site()),
            default_value: default_expr,
            access,
            pdo_mappable: pdo_mapping,
        },
    })
}

fn get_prop(props: &[(String, String)], key: &str) -> Option<String> {
    props
        .iter()
        .find(|(k, _)| k.eq_ignore_ascii_case(key))
        .map(|(_, v)| v.clone())
}

fn parse_int(s: &str) -> Option<u64> {
    let s = s.trim();
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(hex, 16).ok()
    } else {
        s.parse().ok()
    }
}

fn datatype_to_rust(code: u16) -> Option<(&'static str, usize)> {
    match code {
        0x0001 => Some(("bool", 1)),
        0x0002 => Some(("i8", 1)),
        0x0003 => Some(("i16", 2)),
        0x0004 => Some(("i32", 4)),
        0x0005 => Some(("u8", 1)),
        0x0006 => Some(("u16", 2)),
        0x0007 => Some(("u32", 4)),
        0x0008 => Some(("f32", 4)),
        0x0015 => Some(("i64", 8)),
        0x001B => Some(("u64", 8)),
        _ => None, // strings/domains not supported in macro yet
    }
}

fn sanitize_name(name: &str) -> String {
    let s: String = name
        .chars()
        .map(|c| if c.is_alphanumeric() { c.to_ascii_lowercase() } else { '_' })
        .collect();
    // Ensure it starts with a letter or underscore
    if s.starts_with(|c: char| c.is_ascii_digit()) {
        format!("_{s}")
    } else if s.is_empty() {
        "_unnamed".to_string()
    } else {
        s
    }
}

fn normalize_default(value: &str, _data_type: u16) -> String {
    let v = value.trim();
    if v.is_empty() {
        return "0".to_string();
    }
    // Handle hex values
    if let Some(hex) = v.strip_prefix("0x").or_else(|| v.strip_prefix("0X")) {
        return format!("0x{hex}");
    }
    v.to_string()
}
