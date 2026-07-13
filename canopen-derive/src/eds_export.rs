use crate::dsl::*;
use std::fmt::Write;

/// Map Rust type names to CiA data type codes.
fn type_to_eds_code(ty: &str) -> u16 {
    match ty {
        "bool" => 0x0001,
        "i8" => 0x0002,
        "i16" => 0x0003,
        "i32" => 0x0004,
        "u8" => 0x0005,
        "u16" => 0x0006,
        "u32" => 0x0007,
        "f32" => 0x0008,
        "f64" => 0x0011,
        "visible_string" => 0x0009,
        "octet_string" => 0x000A,
        "domain" => 0x000F,
        "i64" => 0x0015,
        "u64" => 0x001B,
        _ => 0x0007,
    }
}

fn access_to_eds(access: AccessKind) -> &'static str {
    match access {
        AccessKind::Ro => "ro",
        AccessKind::Const => "const",
        AccessKind::Rw => "rw",
        AccessKind::Wo => "wo",
    }
}

struct FlatEdsEntry {
    index: u16,
    subindex: u8,
    name: String,
    type_name: String,
    default_value: String,
    access: AccessKind,
    pdo_mappable: bool,
}

fn flatten_for_eds(entries: &[OdEntry]) -> Vec<FlatEdsEntry> {
    let mut flat = Vec::new();
    for entry in entries {
        match &entry.kind {
            EntryKind::Var(var) => {
                let default_value = var
                    .default_value
                    .as_ref()
                    .map(|expr| quote::ToTokens::to_token_stream(expr).to_string())
                    .unwrap_or_default();
                flat.push(FlatEdsEntry {
                    index: entry.index,
                    subindex: 0,
                    name: entry.name.to_string(),
                    type_name: var.type_name.to_string(),
                    default_value,
                    access: var.access,
                    pdo_mappable: var.pdo_mappable,
                });
            }
            EntryKind::Record(subs) => {
                for sub in subs {
                    let default_value = sub
                        .var
                        .default_value
                        .as_ref()
                        .map(|expr| quote::ToTokens::to_token_stream(expr).to_string())
                        .unwrap_or_default();
                    flat.push(FlatEdsEntry {
                        index: entry.index,
                        subindex: sub.subindex,
                        name: sub.name.to_string(),
                        type_name: sub.var.type_name.to_string(),
                        default_value,
                        access: sub.var.access,
                        pdo_mappable: sub.var.pdo_mappable,
                    });
                }
            }
            EntryKind::Array(arr) => {
                for sub in 1..=arr.count {
                    flat.push(FlatEdsEntry {
                        index: entry.index,
                        subindex: sub as u8,
                        name: entry.name.to_string(),
                        type_name: arr.element_type.to_string(),
                        default_value: "0".to_string(),
                        access: arr.access,
                        pdo_mappable: arr.pdo_mappable,
                    });
                }
            }
        }
    }
    flat
}

/// Generate EDS file content from an OdDefinition.
pub fn generate_eds(od: &OdDefinition) -> String {
    let flat = flatten_for_eds(&od.entries);
    let mut out = String::new();

    // FileInfo
    writeln!(out, "[FileInfo]").unwrap();
    writeln!(out, "FileName={}.eds", od.name).unwrap();
    writeln!(out, "FileVersion=1").unwrap();
    writeln!(out, "CreatedBy=canopen-rs (object_dictionary! macro)").unwrap();
    writeln!(out).unwrap();

    // DeviceInfo
    writeln!(out, "[DeviceInfo]").unwrap();
    writeln!(out, "VendorName=").unwrap();
    writeln!(out, "ProductName={}", od.name).unwrap();
    writeln!(out).unwrap();

    // Categorise indices
    let mut user_indices: Vec<u16> = od.entries.iter().map(|e| e.index).collect();
    user_indices.sort();
    user_indices.dedup();
    let mut pdo_indices = Vec::new();
    for pdo in &od.pdos {
        match pdo.direction {
            PdoDirection::Tpdo => {
                pdo_indices.push(0x1800 + (pdo.number - 1) as u16);
                pdo_indices.push(0x1A00 + (pdo.number - 1) as u16);
            }
            PdoDirection::Rpdo => {
                pdo_indices.push(0x1400 + (pdo.number - 1) as u16);
                pdo_indices.push(0x1600 + (pdo.number - 1) as u16);
            }
        }
    }
    pdo_indices.sort();
    pdo_indices.dedup();

    let mandatory: Vec<u16> = user_indices
        .iter()
        .copied()
        .filter(|i| *i < 0x2000)
        .collect();
    let optional: Vec<u16> = pdo_indices;
    let manufacturer: Vec<u16> = user_indices
        .iter()
        .copied()
        .filter(|i| *i >= 0x2000 && *i < 0x6000)
        .collect();
    let standardised: Vec<u16> = user_indices
        .iter()
        .copied()
        .filter(|i| *i >= 0x6000)
        .collect();

    // Write index lists
    write_index_list(&mut out, "MandatoryObjects", &mandatory);
    write_index_list(&mut out, "OptionalObjects", &optional);
    write_index_list(&mut out, "ManufacturerObjects", &manufacturer);
    if !standardised.is_empty() {
        write_index_list(&mut out, "StandardizedObjects", &standardised);
    }

    // Write each object
    for entry in &od.entries {
        match &entry.kind {
            EntryKind::Var(_) => {
                let fe = flat
                    .iter()
                    .find(|f| f.index == entry.index && f.subindex == 0)
                    .unwrap();
                writeln!(out, "[{:04X}]", entry.index).unwrap();
                write_var_props(&mut out, fe, 0x07);
                writeln!(out).unwrap();
            }
            EntryKind::Record(subs) => {
                let max_sub = subs.iter().map(|s| s.subindex).max().unwrap_or(0);

                // Main record section
                writeln!(out, "[{:04X}]", entry.index).unwrap();
                writeln!(out, "ParameterName={}", entry.name).unwrap();
                writeln!(out, "ObjectType=0x9").unwrap();
                writeln!(out, "SubNumber={}", subs.len() + 1).unwrap();
                writeln!(out).unwrap();

                // Sub 0: number of entries
                writeln!(out, "[{:04X}sub0]", entry.index).unwrap();
                writeln!(out, "ParameterName=Number of Entries").unwrap();
                writeln!(out, "ObjectType=0x7").unwrap();
                writeln!(out, "DataType=0x0005").unwrap();
                writeln!(out, "AccessType=ro").unwrap();
                writeln!(out, "DefaultValue={}", max_sub).unwrap();
                writeln!(out, "PDOMapping=0").unwrap();
                writeln!(out).unwrap();

                // Sub-entries
                for sub in subs {
                    let fe = flat
                        .iter()
                        .find(|f| f.index == entry.index && f.subindex == sub.subindex)
                        .unwrap();
                    writeln!(out, "[{:04X}sub{:X}]", entry.index, sub.subindex).unwrap();
                    write_var_props(&mut out, fe, 0x07);
                    writeln!(out).unwrap();
                }
            }
            EntryKind::Array(arr) => {
                // Main array section
                writeln!(out, "[{:04X}]", entry.index).unwrap();
                writeln!(out, "ParameterName={}", entry.name).unwrap();
                writeln!(out, "ObjectType=0x8").unwrap();
                writeln!(out, "SubNumber={}", arr.count + 1).unwrap();
                writeln!(out).unwrap();

                // Sub 0: number of entries
                writeln!(out, "[{:04X}sub0]", entry.index).unwrap();
                writeln!(out, "ParameterName=Number of Entries").unwrap();
                writeln!(out, "ObjectType=0x7").unwrap();
                writeln!(out, "DataType=0x0005").unwrap();
                writeln!(out, "AccessType=ro").unwrap();
                writeln!(out, "DefaultValue={}", arr.count).unwrap();
                writeln!(out, "PDOMapping=0").unwrap();
                writeln!(out).unwrap();

                // Sub-entries
                for sub in 1..=arr.count {
                    let fe = flat
                        .iter()
                        .find(|f| f.index == entry.index && f.subindex == sub as u8)
                        .unwrap();
                    writeln!(out, "[{:04X}sub{:X}]", entry.index, sub).unwrap();
                    write_var_props(&mut out, fe, 0x07);
                    writeln!(out).unwrap();
                }
            }
        }
    }

    let flat_entries = crate::codegen::flatten(&od.entries);
    for pdo in &od.pdos {
        let mappings = crate::codegen::resolve_pdo_mappings(pdo, &flat_entries);
        write_pdo_eds(&mut out, pdo, &mappings);
    }

    out
}

fn write_index_list(out: &mut String, section: &str, indices: &[u16]) {
    writeln!(out, "[{section}]").unwrap();
    writeln!(out, "SupportedObjects={}", indices.len()).unwrap();
    for (i, idx) in indices.iter().enumerate() {
        writeln!(out, "{}=0x{:04X}", i + 1, idx).unwrap();
    }
    writeln!(out).unwrap();
}

fn normalize_eds_value(val: &str) -> String {
    // Remove Rust-style underscores from numeric literals (e.g. 0x0000_0191 -> 0x00000191)
    val.replace('_', "")
}

fn write_var_props(out: &mut String, fe: &FlatEdsEntry, obj_type: u8) {
    writeln!(out, "ParameterName={}", fe.name).unwrap();
    writeln!(out, "ObjectType=0x{:X}", obj_type).unwrap();
    writeln!(out, "DataType=0x{:04X}", type_to_eds_code(&fe.type_name)).unwrap();
    writeln!(out, "AccessType={}", access_to_eds(fe.access)).unwrap();
    writeln!(
        out,
        "DefaultValue={}",
        normalize_eds_value(&fe.default_value)
    )
    .unwrap();
    writeln!(out, "PDOMapping={}", if fe.pdo_mappable { 1 } else { 0 }).unwrap();
}

fn write_record_header(out: &mut String, index: u16, name: &str, sub_number: u8) {
    writeln!(out, "[{:04X}]", index).unwrap();
    writeln!(out, "ParameterName={name}").unwrap();
    writeln!(out, "ObjectType=0x9").unwrap();
    writeln!(out, "SubNumber={sub_number}").unwrap();
    writeln!(out).unwrap();
}

fn write_pdo_sub(
    out: &mut String,
    index: u16,
    subindex: u8,
    name: &str,
    data_type: u16,
    access: &str,
    default_value: &str,
) {
    writeln!(out, "[{:04X}sub{:X}]", index, subindex).unwrap();
    writeln!(out, "ParameterName={name}").unwrap();
    writeln!(out, "ObjectType=0x7").unwrap();
    writeln!(out, "DataType=0x{data_type:04X}").unwrap();
    writeln!(out, "AccessType={access}").unwrap();
    writeln!(out, "DefaultValue={default_value}").unwrap();
    writeln!(out, "PDOMapping=0").unwrap();
    writeln!(out).unwrap();
}

fn predefined_pdo_cob_id(direction: PdoDirection, number: u16) -> String {
    debug_assert!(number <= 4, "PDOs >4 have no predefined COB-ID");
    let base = match direction {
        PdoDirection::Tpdo => 0x180 + 0x100 * (number - 1),
        PdoDirection::Rpdo => 0x200 + 0x100 * (number - 1),
    };
    format!("$NODEID+0x{base:X}")
}

fn pdo_cob_id_default(pdo: &PdoDef) -> String {
    match pdo.cob_id {
        Some(CobIdSpec::Absolute(cob_id)) => format!("0x{cob_id:X}"),
        Some(CobIdSpec::NodeRelative(base)) => format!("$NODEID+0x{base:X}"),
        None => predefined_pdo_cob_id(pdo.direction, pdo.number),
    }
}

fn write_pdo_eds(out: &mut String, pdo: &PdoDef, mappings: &[crate::codegen::ResolvedMapping]) {
    let (comm_base, map_base, prefix) = match pdo.direction {
        PdoDirection::Tpdo => (0x1800u16, 0x1A00u16, "TPDO"),
        PdoDirection::Rpdo => (0x1400u16, 0x1600u16, "RPDO"),
    };
    let comm_idx = comm_base + (pdo.number - 1) as u16;
    let map_idx = map_base + (pdo.number - 1) as u16;
    let is_tpdo = pdo.direction == PdoDirection::Tpdo;
    // TPDO comm params: sub 0-3 + 5; RPDO: sub 0-2.
    let (sub_number, highest_sub) = if is_tpdo { (5, "5") } else { (3, "2") };

    write_record_header(
        out,
        comm_idx,
        &format!("{prefix}{} Communication", pdo.number),
        sub_number,
    );
    write_pdo_sub(
        out,
        comm_idx,
        0,
        "Number of Entries",
        0x0005,
        "ro",
        highest_sub,
    );
    write_pdo_sub(
        out,
        comm_idx,
        1,
        "COB-ID",
        0x0007,
        "rw",
        &pdo_cob_id_default(pdo),
    );
    write_pdo_sub(
        out,
        comm_idx,
        2,
        "Transmission Type",
        0x0005,
        "rw",
        &format!("0x{:X}", pdo.transmission_type),
    );
    if is_tpdo {
        write_pdo_sub(
            out,
            comm_idx,
            3,
            "Inhibit Time",
            0x0006,
            "rw",
            &format!("0x{:X}", pdo.inhibit_time),
        );
        write_pdo_sub(
            out,
            comm_idx,
            5,
            "Event Timer",
            0x0006,
            "rw",
            &format!("0x{:X}", pdo.event_timer),
        );
    }

    write_pdo_mapping_eds(
        out,
        map_idx,
        &format!("{prefix}{} Mapping", pdo.number),
        mappings,
    );
}

fn write_pdo_mapping_eds(
    out: &mut String,
    index: u16,
    name: &str,
    mappings: &[crate::codegen::ResolvedMapping],
) {
    write_record_header(out, index, name, 9);
    write_pdo_sub(
        out,
        index,
        0,
        "Number of Mapped Objects",
        0x0005,
        "rw",
        &format!("0x{:X}", mappings.len()),
    );

    for sub in 1..=8u8 {
        let default_value = mappings
            .get((sub - 1) as usize)
            .map(|m| format!("0x{:08X}", m.raw()))
            .unwrap_or_else(|| "0x00000000".to_string());
        write_pdo_sub(
            out,
            index,
            sub,
            &format!("Mapping Entry {sub}"),
            0x0007,
            "rw",
            &default_value,
        );
    }
}

/// Write the EDS file to the given path (relative to CARGO_MANIFEST_DIR, or absolute).
pub fn export_eds_file(od: &OdDefinition, path: &str) {
    let path = std::path::Path::new(path);
    let full_path = if path.is_absolute() {
        path.to_path_buf()
    } else if let Ok(manifest_dir) = std::env::var("CARGO_MANIFEST_DIR") {
        std::path::Path::new(&manifest_dir).join(path)
    } else {
        return;
    };
    if let Some(parent) = full_path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    let eds = generate_eds(od);
    let _ = std::fs::write(&full_path, eds);
}
