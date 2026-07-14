use crate::dsl::{
    AccessKind, CobIdSpec, EntryKind, MappingKind, OdDefinition, OdEntry, PdoDef, PdoDirection,
    SubEntry, VarDef,
};
use proc_macro2::Span;
use syn::parse::{Parse, ParseStream};
use syn::{braced, bracketed, Ident, LitInt, LitStr, Result, Token, Visibility};

/// Parsed input for `object_dictionary_from_eds!`.
pub struct EdsDefinition {
    pub vis: Visibility,
    pub name: Ident,
    pub file_path: String,
    /// Per-index capacity overrides for variable-length types.
    pub capacity_overrides: std::collections::HashMap<u16, usize>,
}

impl Parse for EdsDefinition {
    fn parse(input: ParseStream) -> Result<Self> {
        let vis: Visibility = input.parse()?;
        input.parse::<Token![struct]>()?;
        let name: Ident = input.parse()?;
        input.parse::<Token![=]>()?;
        let path_lit: LitStr = input.parse()?;

        // Optional: with { [0x1008] capacity = 32, ... }
        let mut capacity_overrides = std::collections::HashMap::new();
        if input.peek(Ident) {
            let kw: Ident = input.parse()?;
            if kw != "with" {
                return Err(syn::Error::new(kw.span(), "expected `with` or `;`"));
            }
            let content;
            braced!(content in input);
            while !content.is_empty() {
                let idx_content;
                bracketed!(idx_content in content);
                let idx_lit: LitInt = idx_content.parse()?;
                let index: u16 = idx_lit
                    .base10_parse()
                    .map_err(|_| syn::Error::new(idx_lit.span(), "expected u16 index"))?;
                let kw: Ident = content.parse()?;
                if kw != "capacity" {
                    return Err(syn::Error::new(kw.span(), "expected `capacity`"));
                }
                content.parse::<Token![=]>()?;
                let cap_lit: LitInt = content.parse()?;
                let cap: usize = cap_lit
                    .base10_parse()
                    .map_err(|_| syn::Error::new(cap_lit.span(), "expected usize capacity"))?;
                capacity_overrides.insert(index, cap);
                let _ = content.parse::<Token![,]>(); // optional trailing comma
            }
        }

        let _ = input.parse::<Token![;]>(); // optional trailing semicolon
        Ok(EdsDefinition {
            vis,
            name,
            file_path: path_lit.value(),
            capacity_overrides,
        })
    }
}

/// Parse an EDS file and convert it to an OdDefinition for codegen.
pub fn parse_eds_to_od(def: EdsDefinition) -> std::result::Result<OdDefinition, syn::Error> {
    // Resolve path relative to CARGO_MANIFEST_DIR
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR")
        .map_err(|_| syn::Error::new(Span::call_site(), "CARGO_MANIFEST_DIR not set"))?;
    let full_path = std::path::Path::new(&manifest_dir).join(&def.file_path);

    let content = std::fs::read_to_string(&full_path).map_err(|e| {
        syn::Error::new(
            Span::call_site(),
            format!("failed to read EDS file `{}`: {}", full_path.display(), e),
        )
    })?;

    let (entries, pdos) = parse_eds_content_with_overrides(&content, &def.capacity_overrides)
        .map_err(|e| syn::Error::new(Span::call_site(), format!("EDS parse error: {e}")))?;

    Ok(OdDefinition {
        vis: def.vis,
        name: def.name,
        entries,
        pdos,
        export_eds_path: None,
        use_alloc: false,
        validate_write_fn: None,
    })
}

/// Parse EDS file content into OD entries and PDO definitions.
#[allow(dead_code)]
fn parse_eds_content(content: &str) -> std::result::Result<(Vec<OdEntry>, Vec<PdoDef>), String> {
    parse_eds_content_with_overrides(content, &std::collections::HashMap::new())
}

fn parse_eds_content_with_overrides(
    content: &str,
    capacity_overrides: &std::collections::HashMap<u16, usize>,
) -> std::result::Result<(Vec<OdEntry>, Vec<PdoDef>), String> {
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

    validate_unique_object_sections(&sections)?;

    // Parse object sections (hex indices like "1000", "1018sub1", etc.)
    // First pass: collect top-level objects and track their ObjectType
    let mut record_subs: std::collections::HashMap<u16, Vec<(u8, Vec<(String, String)>)>> =
        std::collections::HashMap::new();
    let mut obj_types: std::collections::HashMap<u16, u64> = std::collections::HashMap::new();

    for (section_name, props) in &sections {
        // Try to parse as a hex index
        if let Some((index, subindex)) = parse_section_index(section_name) {
            // Skip PDO comm/mapping indices — handled separately in extract_pdos
            if is_pdo_index(index) {
                continue;
            }
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
                obj_types.insert(index, obj_type);

                if obj_type == 0x07 {
                    // VAR object
                    if let Some(entry) = parse_var_entry(index, 0, props, capacity_overrides) {
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

    // Second pass: assemble records/arrays from sub-entries
    for (index, subs) in &record_subs {
        let mut sub_entries = Vec::new();
        for (subindex, props) in subs {
            // Skip sub0 (number of entries) — codegen generates it automatically
            if *subindex == 0 {
                continue;
            }
            if let Some(entry) = parse_var_entry(*index, *subindex, props, capacity_overrides) {
                sub_entries.push(entry);
            }
        }
        if !sub_entries.is_empty() {
            sub_entries.sort_by_key(|s| s.subindex);

            // Find name from the main section (needed for record name prefix)
            let main_name = sections
                .iter()
                .find(|(name, _)| {
                    parse_section_index(name)
                        .map(|(idx, sub)| idx == *index && sub.is_none())
                        .unwrap_or(false)
                })
                .and_then(|(_, props)| get_prop(props, "ParameterName"))
                .unwrap_or_else(|| format!("obj_{:04x}", index));

            // Prefix sub-entry names with record name to avoid field collisions
            let record_prefix = sanitize_name(&main_name);
            for sub in &mut sub_entries {
                let prefixed = format!("{}_{}", record_prefix, sub.name);
                sub.name = Ident::new(&prefixed, Span::call_site());
            }

            let obj_type = obj_types.get(index).copied().unwrap_or(0x09);

            if obj_type == 0x08 {
                // ARRAY: all non-sub0 entries share the same type.
                // Only use EntryKind::Array for larger homogeneous arrays (4+ elements)
                // where per-element defaults don't matter. Smaller arrays or arrays
                // with non-zero defaults are kept as records to preserve defaults.
                let elements: Vec<&SubEntry> =
                    sub_entries.iter().filter(|s| s.subindex > 0).collect();
                let all_zero_defaults = elements.iter().all(|e| {
                    e.var
                        .default_value
                        .as_ref()
                        .map(|expr| {
                            let s = quote::ToTokens::to_token_stream(expr).to_string();
                            s == "0"
                                || s == "0x00"
                                || s == "0i32"
                                || s == "0u32"
                                || s == "0.0f32"
                                || s == "0.0f64"
                                || s == "false"
                        })
                        .unwrap_or(true) // None = no default = zero
                });
                let use_array = elements.len() >= 4 && all_zero_defaults;

                if use_array {
                    if let Some(first) = elements.first() {
                        let element_type = first.var.type_name.clone();
                        let count = elements.len();
                        let access = first.var.access;
                        let pdo_mappable = first.var.pdo_mappable;
                        let element_capacity = first.var.capacity;

                        entries.push(OdEntry {
                            index: *index,
                            name: Ident::new(&sanitize_name(&main_name), Span::call_site()),
                            kind: EntryKind::Array(crate::dsl::ArrayDef {
                                element_type,
                                element_capacity,
                                count,
                                access,
                                pdo_mappable,
                            }),
                        });
                    }
                } else {
                    // Keep as record to preserve per-element defaults
                    entries.push(OdEntry {
                        index: *index,
                        name: Ident::new(&sanitize_name(&main_name), Span::call_site()),
                        kind: EntryKind::Record(sub_entries),
                    });
                }
            } else {
                entries.push(OdEntry {
                    index: *index,
                    name: Ident::new(&sanitize_name(&main_name), Span::call_site()),
                    kind: EntryKind::Record(sub_entries),
                });
            }
        }
    }

    entries.sort_by_key(|e| e.index);

    // Third pass: extract PDO definitions from comm/mapping indices
    let pdos = extract_pdos(&sections, &mut entries);

    Ok((entries, pdos))
}

fn validate_unique_object_sections(
    sections: &[(String, Vec<(String, String)>)],
) -> std::result::Result<(), String> {
    let mut seen: std::collections::HashMap<(u16, Option<u8>), &str> =
        std::collections::HashMap::new();

    for (section_name, _) in sections {
        if let Some(address) = parse_section_index(section_name) {
            if let Some(previous) = seen.insert(address, section_name.as_str()) {
                let (index, subindex) = address;
                let address = match subindex {
                    Some(sub) => format!("0x{index:04X}:{sub:02X}"),
                    None => format!("0x{index:04X}"),
                };
                return Err(format!(
                    "duplicate EDS object section for OD address {address}: [{previous}] and [{section_name}]"
                ));
            }
        }
    }

    Ok(())
}

/// Check if an index is a PDO communication or mapping parameter.
fn is_pdo_index(index: u16) -> bool {
    matches!(index, 0x1400..=0x15FF | 0x1600..=0x17FF | 0x1800..=0x19FF | 0x1A00..=0x1BFF)
}

/// Extract PDO definitions from EDS PDO comm/mapping sections.
/// Also marks referenced OD entries as pdo_mappable if they aren't already.
fn extract_pdos(
    sections: &[(String, Vec<(String, String)>)],
    entries: &mut [OdEntry],
) -> Vec<PdoDef> {
    let mut pdos = Vec::new();

    // Helper: look up props for a given section like "1800sub2"
    let get_section_props = |section_key: &str| -> Option<&Vec<(String, String)>> {
        sections
            .iter()
            .find(|(name, _)| name.eq_ignore_ascii_case(section_key))
            .map(|(_, props)| props)
    };

    // (resolve_mapping is a free function below)

    // Parse TPDOs (comm: 0x1800+N, mapping: 0x1A00+N); CiA 301 allows up to 512
    for n in 0u16..512 {
        let comm_idx = 0x1800 + n;
        let map_idx = 0x1A00 + n;

        // Check if this TPDO exists (has a comm param section)
        let comm_key = format!("{:04X}", comm_idx);
        if get_section_props(&comm_key).is_none() {
            continue;
        }

        // Read COB-ID from sub1
        let cob_id = get_section_props(&format!("{:04X}sub1", comm_idx))
            .and_then(|props| get_prop(props, "DefaultValue"))
            .and_then(|v| parse_pdo_cob_id(&v));

        // Read transmission type from sub2
        let transmission_type = get_section_props(&format!("{:04X}sub2", comm_idx))
            .and_then(|props| get_prop(props, "DefaultValue"))
            .and_then(|v| parse_int(&v))
            .unwrap_or(255) as u8;

        // Read inhibit time from sub3
        let inhibit_time = get_section_props(&format!("{:04X}sub3", comm_idx))
            .and_then(|props| get_prop(props, "DefaultValue"))
            .and_then(|v| parse_int(&v))
            .unwrap_or(0) as u16;

        // Read event timer from sub5
        let event_timer = get_section_props(&format!("{:04X}sub5", comm_idx))
            .and_then(|props| get_prop(props, "DefaultValue"))
            .and_then(|v| parse_int(&v))
            .unwrap_or(0) as u16;

        // Read mapping count from mapping sub0
        let map_count = get_section_props(&format!("{:04X}sub0", map_idx))
            .and_then(|props| get_prop(props, "DefaultValue"))
            .and_then(|v| parse_int(&v))
            .unwrap_or(0) as u8;

        let mapping = mapping_kind_from_access(
            get_section_props(&format!("{:04X}sub0", map_idx))
                .and_then(|props| get_prop(props, "AccessType")),
        );

        // Read mappings
        let mut mappings = Vec::new();
        for sub in 1..=map_count {
            if let Some(props) = get_section_props(&format!("{:04X}sub{:X}", map_idx, sub)) {
                if let Some(val_str) = get_prop(props, "DefaultValue") {
                    if let Some(val) = parse_int(&val_str) {
                        if val != 0 {
                            if let Some(field_name) = resolve_mapping(val as u32, entries) {
                                mappings.push(field_name);
                            }
                        }
                    }
                }
            }
        }

        // Push even with no mappings: a comm-only PDO (disabled placeholder,
        // or intended for runtime remapping) must keep its comm params.
        pdos.push(PdoDef {
            direction: PdoDirection::Tpdo,
            number: n + 1,
            cob_id,
            transmission_type,
            inhibit_time,
            event_timer,
            mapping,
            mappings,
        });
    }

    // Parse RPDOs (comm: 0x1400+N, mapping: 0x1600+N); CiA 301 allows up to 512
    for n in 0u16..512 {
        let comm_idx = 0x1400 + n;
        let map_idx = 0x1600 + n;

        let comm_key = format!("{:04X}", comm_idx);
        if get_section_props(&comm_key).is_none() {
            continue;
        }

        let cob_id = get_section_props(&format!("{:04X}sub1", comm_idx))
            .and_then(|props| get_prop(props, "DefaultValue"))
            .and_then(|v| parse_pdo_cob_id(&v));

        let transmission_type = get_section_props(&format!("{:04X}sub2", comm_idx))
            .and_then(|props| get_prop(props, "DefaultValue"))
            .and_then(|v| parse_int(&v))
            .unwrap_or(255) as u8;

        // Sub 5 event timer = reception deadline monitoring for RPDOs
        let event_timer = get_section_props(&format!("{:04X}sub5", comm_idx))
            .and_then(|props| get_prop(props, "DefaultValue"))
            .and_then(|v| parse_int(&v))
            .unwrap_or(0) as u16;

        let map_count = get_section_props(&format!("{:04X}sub0", map_idx))
            .and_then(|props| get_prop(props, "DefaultValue"))
            .and_then(|v| parse_int(&v))
            .unwrap_or(0) as u8;

        let mapping = mapping_kind_from_access(
            get_section_props(&format!("{:04X}sub0", map_idx))
                .and_then(|props| get_prop(props, "AccessType")),
        );

        let mut mappings = Vec::new();
        for sub in 1..=map_count {
            if let Some(props) = get_section_props(&format!("{:04X}sub{:X}", map_idx, sub)) {
                if let Some(val_str) = get_prop(props, "DefaultValue") {
                    if let Some(val) = parse_int(&val_str) {
                        if val != 0 {
                            if let Some(field_name) = resolve_mapping(val as u32, entries) {
                                mappings.push(field_name);
                            }
                        }
                    }
                }
            }
        }

        pdos.push(PdoDef {
            direction: PdoDirection::Rpdo,
            number: n + 1,
            cob_id,
            transmission_type,
            inhibit_time: 0,
            event_timer,
            mapping,
            mappings,
        });
    }

    pdos
}

/// Map an EDS mapping-record AccessType to mapping mutability. `ro` and
/// `const` both mean immutable; `rw`/`rww`/`rwr` mean CiA 301 dynamic
/// mapping. A missing mapping section or AccessType stays mutable so
/// comm-only placeholder PDOs remain remappable at runtime.
fn mapping_kind_from_access(access: Option<String>) -> MappingKind {
    match access.as_deref().map(str::to_ascii_lowercase).as_deref() {
        Some("ro") | Some("const") => MappingKind::Immutable,
        _ => MappingKind::Mutable,
    }
}

/// Resolve a PDO mapping value (e.g. 0x60400010) to a field name in the OD,
/// and mark the referenced entry as pdo_mappable.
fn resolve_mapping(mapping_val: u32, entries: &mut [OdEntry]) -> Option<Ident> {
    let mapped_index = (mapping_val >> 16) as u16;
    let mapped_sub = ((mapping_val >> 8) & 0xFF) as u8;
    for entry in entries.iter_mut() {
        match &mut entry.kind {
            EntryKind::Var(ref mut var) if entry.index == mapped_index && mapped_sub == 0 => {
                var.pdo_mappable = true;
                return Some(entry.name.clone());
            }
            EntryKind::Record(ref mut subs) if entry.index == mapped_index => {
                for sub in subs.iter_mut() {
                    if sub.subindex == mapped_sub {
                        sub.var.pdo_mappable = true;
                        return Some(sub.name.clone());
                    }
                }
            }
            _ => {}
        }
    }
    None
}

/// Parse a PDO COB-ID value. `$NODEID+0xNNN` (either operand order) becomes
/// [`CobIdSpec::NodeRelative`] with the numeric base; plain hex/decimal values
/// become [`CobIdSpec::Absolute`]. Returns None if the value is unparseable.
fn parse_pdo_cob_id(value: &str) -> Option<CobIdSpec> {
    let v = value.trim();
    if v.to_ascii_uppercase().contains("$NODEID") {
        // e.g. "$NODEID+0x200" or "0x200+$NODEID"
        let base_str = v
            .split('+')
            .map(str::trim)
            .find(|part| !part.eq_ignore_ascii_case("$NODEID"))?;
        parse_int(base_str).map(|n| CobIdSpec::NodeRelative(n as u32))
    } else {
        parse_int(v).map(|n| CobIdSpec::Absolute(n as u32))
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
        // Filter out non-object sections
        if index >= 0x1000 {
            Some((index, None))
        } else {
            None
        }
    }
}

fn parse_var_entry(
    _index: u16,
    subindex: u8,
    props: &[(String, String)],
    capacity_overrides: &std::collections::HashMap<u16, usize>,
) -> Option<SubEntry> {
    let name_str = get_prop(props, "ParameterName")?;
    let data_type_code = get_prop(props, "DataType").and_then(|v| parse_int(&v))? as u16;
    let access_str = get_prop(props, "AccessType").unwrap_or_else(|| "ro".to_string());
    let default_str = get_prop(props, "DefaultValue").unwrap_or_else(|| "0".to_string());
    let pdo_mapping = get_prop(props, "PDOMapping")
        .and_then(|v| parse_int(&v))
        .unwrap_or(0)
        != 0;

    let (rust_type, _size) = datatype_to_rust(data_type_code)?;
    let is_varlen = crate::dsl::is_variable_length_type(rust_type);
    let access = match access_str.to_lowercase().as_str() {
        "ro" | "const" => AccessKind::Ro,
        // CiA 306: rwr/rww are read-write; the suffix only marks TPDO/RPDO mappability
        "rw" | "rwr" | "rww" => AccessKind::Rw,
        "wo" => AccessKind::Wo,
        _ => AccessKind::Ro,
    };

    let capacity = if is_varlen {
        let default_cap = match rust_type {
            "visible_string" => 64,
            "octet_string" => 256,
            "domain" => 512,
            _ => 256,
        };
        Some(
            capacity_overrides
                .get(&_index)
                .copied()
                .unwrap_or(default_cap),
        )
    } else {
        None
    };

    let default_value = if is_varlen {
        // Preserve string defaults for visible_string
        if rust_type == "visible_string" {
            let stripped = default_str.trim().trim_matches('"');
            if !stripped.is_empty() {
                syn::parse_str::<syn::Expr>(&format!("\"{}\"", stripped)).ok()
            } else {
                None
            }
        } else {
            None
        }
    } else {
        let normalized = normalize_default(&default_str, data_type_code);
        syn::parse_str::<syn::Expr>(&normalized).ok().or_else(|| {
            // Fallback: if we can't parse the default, use 0
            syn::parse_str::<syn::Expr>("0").ok()
        })
    };

    Some(SubEntry {
        subindex,
        name: Ident::new(&sanitize_name(&name_str), Span::call_site()),
        var: VarDef {
            type_name: Ident::new(rust_type, Span::call_site()),
            capacity,
            default_value,
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
        0x0010 => Some(("i24", 3)),
        0x0016 => Some(("u24", 3)),
        0x0009 => Some(("visible_string", 0)),
        0x000A => Some(("octet_string", 0)),
        0x000F => Some(("domain", 0)),
        0x0011 => Some(("f64", 8)),
        0x0015 => Some(("i64", 8)),
        0x001B => Some(("u64", 8)),
        _ => None,
    }
}

fn sanitize_name(name: &str) -> String {
    let s: String = name
        .chars()
        .map(|c| {
            if c.is_alphanumeric() {
                c.to_ascii_lowercase()
            } else {
                '_'
            }
        })
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

fn normalize_default(value: &str, data_type: u16) -> String {
    let v = value.trim();
    if v.is_empty() {
        // Use type-appropriate zero
        return match data_type {
            0x0008 => "0.0f32".to_string(),
            0x0011 => "0.0f64".to_string(),
            _ => "0".to_string(),
        };
    }
    // $NODEID expressions can't be represented as Rust literals — use 0
    if v.contains("$NODEID") || v.contains("$nodeid") {
        return match data_type {
            0x0008 => "0.0f32".to_string(),
            0x0011 => "0.0f64".to_string(),
            _ => "0".to_string(),
        };
    }
    // Strip quotes from string defaults
    if v.starts_with('"') && v.ends_with('"') {
        return v.to_string();
    }
    // Float types: hex values are IEEE 754 bit patterns, convert to f32/f64
    if data_type == 0x0008 {
        if let Some(hex) = v.strip_prefix("0x").or_else(|| v.strip_prefix("0X")) {
            if u32::from_str_radix(hex, 16).is_ok() {
                return format!("f32::from_bits(0x{hex})");
            }
        }
        // Plain numeric — pass through as f32
        return format!("{v}f32");
    }
    if data_type == 0x0011 {
        if let Some(hex) = v.strip_prefix("0x").or_else(|| v.strip_prefix("0X")) {
            if u64::from_str_radix(hex, 16).is_ok() {
                return format!("f64::from_bits(0x{hex})");
            }
        }
        return format!("{v}f64");
    }
    // Handle hex values
    if let Some(hex) = v.strip_prefix("0x").or_else(|| v.strip_prefix("0X")) {
        return format!("0x{hex}");
    }
    v.to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duplicate_top_level_object_section_is_error() {
        let err = parse_eds_content(
            r#"
[1000]
ParameterName=Device Type
ObjectType=0x7
DataType=0x0007

[1000]
ParameterName=Different Name
ObjectType=0x7
DataType=0x0007
"#,
        )
        .unwrap_err();

        assert!(err.contains("duplicate EDS object section"));
        assert!(err.contains("0x1000"));
    }

    #[test]
    fn duplicate_subindex_section_is_error_even_with_different_spelling() {
        let err = parse_eds_content(
            r#"
[1018]
ParameterName=Identity
ObjectType=0x9

[1018sub1]
ParameterName=Vendor ID
ObjectType=0x7
DataType=0x0007

[1018sub01]
ParameterName=Different Name
ObjectType=0x7
DataType=0x0007
"#,
        )
        .unwrap_err();

        assert!(err.contains("duplicate EDS object section"));
        assert!(err.contains("0x1018:01"));
    }

    #[test]
    fn record_top_level_and_subzero_are_distinct_sections() {
        parse_eds_content(
            r#"
[1018]
ParameterName=Identity
ObjectType=0x9

[1018sub0]
ParameterName=Number of Entries
ObjectType=0x7
DataType=0x0005
DefaultValue=1

[1018sub1]
ParameterName=Vendor ID
ObjectType=0x7
DataType=0x0007
"#,
        )
        .unwrap();
    }
}
