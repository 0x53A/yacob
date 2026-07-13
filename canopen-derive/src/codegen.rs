use crate::dsl::*;
use proc_macro2::{Literal, TokenStream};
use quote::{format_ident, quote};

/// Flatten an OdDefinition into a list of (index, subindex, name, VarDef).
pub(crate) struct FlatEntry {
    pub(crate) index: u16,
    pub(crate) subindex: u8,
    pub(crate) field_name: syn::Ident,
    pub(crate) var: VarDef,
}

pub(crate) fn flatten(entries: &[OdEntry]) -> Vec<FlatEntry> {
    let mut flat = Vec::new();
    for entry in entries {
        match &entry.kind {
            EntryKind::Var(var) => {
                flat.push(FlatEntry {
                    index: entry.index,
                    subindex: 0,
                    field_name: entry.name.clone(),
                    var: var.clone(),
                });
            }
            EntryKind::Record(subs) => {
                for sub in subs {
                    flat.push(FlatEntry {
                        index: entry.index,
                        subindex: sub.subindex,
                        field_name: sub.name.clone(),
                        var: sub.var.clone(),
                    });
                }
            }
            EntryKind::Array(_) => {
                // Arrays are handled separately in codegen
            }
        }
    }
    flat
}

/// Collect array definitions from entries.
struct ArrayEntry {
    index: u16,
    field_name: syn::Ident,
    def: ArrayDef,
}

fn collect_arrays(entries: &[OdEntry]) -> Vec<ArrayEntry> {
    entries
        .iter()
        .filter_map(|e| {
            if let EntryKind::Array(def) = &e.kind {
                Some(ArrayEntry {
                    index: e.index,
                    field_name: e.name.clone(),
                    def: def.clone(),
                })
            } else {
                None
            }
        })
        .collect()
}

/// Resolved PDO mapping: field name → (index, subindex, bit_length).
pub(crate) struct ResolvedMapping {
    index: u16,
    subindex: u8,
    bit_length: u8,
}

impl ResolvedMapping {
    /// The CiA 301 PDO mapping entry value: index << 16 | subindex << 8 | bits.
    pub(crate) fn raw(&self) -> u32 {
        (self.index as u32) << 16 | (self.subindex as u32) << 8 | self.bit_length as u32
    }
}

/// Resolve PDO field names to (index, subindex, bit_length) using the flat entry list.
pub(crate) fn resolve_pdo_mappings(pdo: &PdoDef, flat: &[FlatEntry]) -> Vec<ResolvedMapping> {
    let pdo_label = format!(
        "{}[{}]",
        match pdo.direction {
            PdoDirection::Tpdo => "tpdo",
            PdoDirection::Rpdo => "rpdo",
        },
        pdo.number,
    );
    let mut resolved = Vec::new();
    for field_name in &pdo.mappings {
        let entry = flat
            .iter()
            .find(|e| e.field_name == *field_name)
            .unwrap_or_else(|| {
                panic!(
                    "{pdo_label} references unknown field `{field_name}`. \
                     Make sure it is defined as a VAR or RECORD sub-entry in the OD.",
                )
            });
        if !entry.var.pdo_mappable {
            panic!(
                "{pdo_label} maps field `{field_name}` (0x{:04X}:{}) which is not PDO-mappable. \
                 Add `, pdo` to its definition, e.g.: `[{}] {field_name}: {} = ..., {}, pdo;`",
                entry.index,
                entry.subindex,
                entry.subindex,
                entry.var.type_name,
                match entry.var.access {
                    AccessKind::Ro => "ro",
                    AccessKind::Wo => "wo",
                    AccessKind::Rw => "rw",
                    AccessKind::Const => "const",
                },
            );
        }
        let ty_str = entry.var.type_name.to_string();
        if is_variable_length_type(&ty_str) {
            panic!(
                "{pdo_label} maps field `{field_name}` which has variable-length type `{ty_str}`. \
                 Only fixed-size types (u8, u16, u32, i8, i16, i32, f32, f64) can be PDO-mapped.",
            );
        }
        let bit_length = type_size(&ty_str).expect("unsupported type") as u8 * 8;
        resolved.push(ResolvedMapping {
            index: entry.index,
            subindex: entry.subindex,
            bit_length,
        });
    }
    resolved
}

pub fn generate(od: OdDefinition) -> TokenStream {
    if let Some(path) = &od.export_eds_path {
        crate::eds_export::export_eds_file(&od, path);
    }
    let eds_content = crate::eds_export::generate_eds(&od);
    let eds_compressed = miniz_oxide::deflate::compress_to_vec(eds_content.as_bytes(), 6);
    let vis = &od.vis;
    let name = &od.name;
    let use_alloc = od.use_alloc;
    let flat = flatten(&od.entries);
    let arrays = collect_arrays(&od.entries);

    // Count TPDOs and RPDOs
    let tpdo_defs: Vec<&PdoDef> = od
        .pdos
        .iter()
        .filter(|p| p.direction == PdoDirection::Tpdo)
        .collect();
    let rpdo_defs: Vec<&PdoDef> = od
        .pdos
        .iter()
        .filter(|p| p.direction == PdoDirection::Rpdo)
        .collect();
    let tpdo_count = tpdo_defs.len();
    let rpdo_count = rpdo_defs.len();
    let has_pdos = tpdo_count > 0 || rpdo_count > 0;

    // Reject duplicate PDO numbers (they would collide on OD comm/mapping indices)
    for (label, defs) in [("tpdo", &tpdo_defs), ("rpdo", &rpdo_defs)] {
        let mut numbers: Vec<u16> = defs.iter().map(|p| p.number).collect();
        numbers.sort_unstable();
        numbers.dedup();
        if numbers.len() != defs.len() {
            panic!("duplicate {label}[N] definition: each PDO number may only be declared once");
        }
    }

    // Resolve PDO mappings (field names → index/subindex/bit_length)
    let tpdo_resolved: Vec<Vec<ResolvedMapping>> = tpdo_defs
        .iter()
        .map(|p| resolve_pdo_mappings(p, &flat))
        .collect();
    let rpdo_resolved: Vec<Vec<ResolvedMapping>> = rpdo_defs
        .iter()
        .map(|p| resolve_pdo_mappings(p, &flat))
        .collect();

    // ---- Generate user-defined struct fields ----
    let struct_fields: Vec<TokenStream> = flat
        .iter()
        .map(|e| {
            let fname = &e.field_name;
            let ty_str = e.var.type_name.to_string();
            if is_variable_length_type(&ty_str) {
                if use_alloc {
                    if ty_str == "visible_string" {
                        quote! { pub #fname: alloc::string::String }
                    } else {
                        quote! { pub #fname: alloc::vec::Vec<u8> }
                    }
                } else {
                    let cap = e.var.capacity.unwrap_or_else(|| {
                        panic!("Variable-length type `{ty_str}` for field `{fname}` requires a capacity (e.g. {ty_str}<64>). Use #[use_alloc] for dynamic allocation.")
                    });
                    let cap_lit = Literal::usize_unsuffixed(cap);
                    if ty_str == "visible_string" {
                        quote! { pub #fname: canopen_core::heapless::String<#cap_lit> }
                    } else {
                        quote! { pub #fname: canopen_core::heapless::Vec<u8, #cap_lit> }
                    }
                }
            } else {
                let ty = &e.var.type_name;
                quote! { pub #fname: #ty }
            }
        })
        .collect();

    // Track whether any field has a non-const default (non-empty string literal)
    let _has_nonconst_default = flat.iter().any(|e| {
        let ty_str = e.var.type_name.to_string();
        if ty_str == "visible_string" {
            // If there's a string default value, it's non-const
            e.var.default_value.is_some()
        } else {
            false
        }
    });

    let field_defaults: Vec<TokenStream> = flat
        .iter()
        .map(|e| {
            let fname = &e.field_name;
            let ty_str = e.var.type_name.to_string();
            if is_variable_length_type(&ty_str) {
                if use_alloc {
                    if ty_str == "visible_string" {
                        if let Some(ref default_expr) = e.var.default_value {
                            quote! { #fname: alloc::string::String::from(&#default_expr.to_string()) }
                        } else {
                            quote! { #fname: alloc::string::String::new() }
                        }
                    } else {
                        quote! { #fname: alloc::vec::Vec::new() }
                    }
                } else if ty_str == "visible_string" {
                    if let Some(ref default_expr) = e.var.default_value {
                        quote! { #fname: {
                            let mut s = canopen_core::heapless::String::new();
                            let _ = s.push_str(&#default_expr.to_string());
                            s
                        }}
                    } else {
                        quote! { #fname: canopen_core::heapless::String::new() }
                    }
                } else {
                    quote! { #fname: canopen_core::heapless::Vec::new() }
                }
            } else if let Some(ref default_expr) = e.var.default_value {
                quote! { #fname: #default_expr }
            } else {
                // No default provided — use zero/default
                let ty = &e.var.type_name;
                if ty_str == "bool" {
                    quote! { #fname: false }
                } else if ty_str == "f32" {
                    quote! { #fname: 0.0f32 }
                } else if ty_str == "f64" {
                    quote! { #fname: 0.0f64 }
                } else {
                    quote! { #fname: 0 as #ty }
                }
            }
        })
        .collect();

    // ---- Generate array struct fields and defaults ----
    let array_struct_fields: Vec<TokenStream> = arrays
        .iter()
        .map(|a| {
            let fname = &a.field_name;
            let ty = &a.def.element_type;
            let count = a.def.count;
            quote! { pub #fname: [#ty; #count] }
        })
        .collect();

    let array_field_defaults: Vec<TokenStream> = arrays
        .iter()
        .map(|a| {
            let fname = &a.field_name;
            let ty = &a.def.element_type;
            let ty_str = ty.to_string();
            let count = a.def.count;
            if ty_str == "bool" {
                quote! { #fname: [false; #count] }
            } else if ty_str == "f32" {
                quote! { #fname: [0.0f32; #count] }
            } else if ty_str == "f64" {
                quote! { #fname: [0.0f64; #count] }
            } else {
                quote! { #fname: [0 as #ty; #count] }
            }
        })
        .collect();

    // ---- Generate array read/write/meta ----
    let mut array_read_arms: Vec<TokenStream> = Vec::new();
    let mut array_write_arms: Vec<TokenStream> = Vec::new();
    let mut array_meta_entries: Vec<TokenStream> = Vec::new();

    for a in &arrays {
        let index = a.index;
        let fname = &a.field_name;
        let ty = &a.def.element_type;
        let ty_str = ty.to_string();
        let count = a.def.count as u8;

        let access_ident = match a.def.access {
            AccessKind::Ro => format_ident!("Ro"),
            AccessKind::Rw => format_ident!("Rw"),
            AccessKind::Wo => format_ident!("Wo"),
            AccessKind::Const => format_ident!("Const"),
        };
        let pdo = a.def.pdo_mappable;
        let dt = type_to_datatype(&ty_str).expect("unsupported array element type");
        let dt_ident = format_ident!("{}", dt);
        let entry_name = fname.to_string();

        // sub0: number of entries (u8, ro)
        array_meta_entries.push(quote! {
            canopen_core::od::OdEntryMeta {
                index: #index, subindex: 0,
                data_type: canopen_core::datatypes::DataType::U8,
                access: canopen_core::od::AccessType::Ro,
                pdo_mappable: false,
                name: "number_of_entries",
                max_size: None,
            }
        });

        array_read_arms.push(quote! {
            (#index, 0) => { buf[0] = #count; Ok(1) }
        });
        array_write_arms.push(quote! {
            (#index, 0) => Err(canopen_core::od::OdError::ReadOnly),
        });

        // sub1..N: array elements
        for sub in 1..=count {
            array_meta_entries.push(quote! {
                canopen_core::od::OdEntryMeta {
                    index: #index, subindex: #sub,
                    data_type: canopen_core::datatypes::DataType::#dt_ident,
                    access: canopen_core::od::AccessType::#access_ident,
                    pdo_mappable: #pdo,
                    name: #entry_name,
                    max_size: None,
                }
            });
        }

        let size = type_size(&ty_str).expect("unsupported array element type");

        // Read arm with range match
        if matches!(a.def.access, AccessKind::Wo) {
            array_read_arms.push(quote! {
                (#index, sub @ 1..=#count) => Err(canopen_core::od::OdError::WriteOnly),
            });
        } else if size == 1 {
            if ty_str == "bool" {
                array_read_arms.push(quote! {
                    (#index, sub @ 1..=#count) => {
                        buf[0] = if self.#fname[(sub as usize) - 1] { 1 } else { 0 };
                        Ok(1)
                    }
                });
            } else {
                array_read_arms.push(quote! {
                    (#index, sub @ 1..=#count) => {
                        buf[0] = self.#fname[(sub as usize) - 1] as u8;
                        Ok(1)
                    }
                });
            }
        } else {
            array_read_arms.push(quote! {
                (#index, sub @ 1..=#count) => {
                    let bytes = self.#fname[(sub as usize) - 1].to_le_bytes();
                    buf[..#size].copy_from_slice(&bytes);
                    Ok(#size)
                }
            });
        }

        // Write arm with range match
        if matches!(a.def.access, AccessKind::Ro | AccessKind::Const) {
            array_write_arms.push(quote! {
                (#index, sub @ 1..=#count) => Err(canopen_core::od::OdError::ReadOnly),
            });
        } else if ty_str == "bool" {
            array_write_arms.push(quote! {
                (#index, sub @ 1..=#count) => {
                    if data.len() != 1 { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                    self.#fname[(sub as usize) - 1] = data[0] != 0;
                    Ok(())
                }
            });
        } else if size == 1 {
            array_write_arms.push(quote! {
                (#index, sub @ 1..=#count) => {
                    if data.len() != 1 { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                    self.#fname[(sub as usize) - 1] = data[0] as #ty;
                    Ok(())
                }
            });
        } else {
            array_write_arms.push(quote! {
                (#index, sub @ 1..=#count) => {
                    if data.len() != #size { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                    let mut arr = [0u8; #size];
                    arr.copy_from_slice(&data[..#size]);
                    self.#fname[(sub as usize) - 1] = #ty::from_le_bytes(arr);
                    Ok(())
                }
            });
        }
    }

    // ---- Generate PDO struct fields and defaults ----
    let pdo_struct_fields = if has_pdos {
        let tc = tpdo_count;
        let rc = rpdo_count;
        let mut fields = Vec::new();
        if tc > 0 {
            fields.push(quote! { pub tpdo_cob_id: [u32; #tc] });
            fields.push(quote! { pub tpdo_transmission_type: [u8; #tc] });
            fields.push(quote! { pub tpdo_inhibit_time: [u16; #tc] });
            fields.push(quote! { pub tpdo_event_timer: [u16; #tc] });
            fields.push(quote! { pub tpdo_mapping_count: [u8; #tc] });
            fields.push(quote! { pub tpdo_mappings: [[u32; 8]; #tc] });
        }
        if rc > 0 {
            fields.push(quote! { pub rpdo_cob_id: [u32; #rc] });
            fields.push(quote! { pub rpdo_transmission_type: [u8; #rc] });
            fields.push(quote! { pub rpdo_mapping_count: [u8; #rc] });
            fields.push(quote! { pub rpdo_mappings: [[u32; 8]; #rc] });
        }
        fields
    } else {
        Vec::new()
    };

    let pdo_field_defaults = if has_pdos {
        let mut defaults = Vec::new();
        if tpdo_count > 0 {
            // Build arrays of default values from the PdoDef list
            let cob_ids: Vec<TokenStream> = tpdo_defs
                .iter()
                .map(|p| {
                    match p.cob_id {
                        Some(CobIdSpec::Absolute(id)) => quote! { #id },
                        // 0 = resolved at runtime with node_id (predefined
                        // default or node-relative base)
                        Some(CobIdSpec::NodeRelative(_)) | None => quote! { 0 },
                    }
                })
                .collect();
            let tt: Vec<u8> = tpdo_defs.iter().map(|p| p.transmission_type).collect();
            let inh: Vec<u16> = tpdo_defs.iter().map(|p| p.inhibit_time).collect();
            let evt: Vec<u16> = tpdo_defs.iter().map(|p| p.event_timer).collect();
            let map_counts: Vec<u8> = tpdo_resolved.iter().map(|m| m.len() as u8).collect();
            let map_arrays: Vec<TokenStream> = tpdo_resolved
                .iter()
                .map(|mappings| {
                    let mut vals = [0u32; 8];
                    for (i, m) in mappings.iter().enumerate() {
                        vals[i] = m.raw();
                    }
                    let v = vals;
                    quote! { [#(#v),*] }
                })
                .collect();

            defaults.push(quote! { tpdo_cob_id: [#(#cob_ids),*] });
            defaults.push(quote! { tpdo_transmission_type: [#(#tt),*] });
            defaults.push(quote! { tpdo_inhibit_time: [#(#inh),*] });
            defaults.push(quote! { tpdo_event_timer: [#(#evt),*] });
            defaults.push(quote! { tpdo_mapping_count: [#(#map_counts),*] });
            defaults.push(quote! { tpdo_mappings: [#(#map_arrays),*] });
        }
        if rpdo_count > 0 {
            let cob_ids: Vec<TokenStream> = rpdo_defs
                .iter()
                .map(|p| match p.cob_id {
                    Some(CobIdSpec::Absolute(id)) => quote! { #id },
                    Some(CobIdSpec::NodeRelative(_)) | None => quote! { 0 },
                })
                .collect();
            let tt: Vec<u8> = rpdo_defs.iter().map(|p| p.transmission_type).collect();
            let map_counts: Vec<u8> = rpdo_resolved.iter().map(|m| m.len() as u8).collect();
            let map_arrays: Vec<TokenStream> = rpdo_resolved
                .iter()
                .map(|mappings| {
                    let mut vals = [0u32; 8];
                    for (i, m) in mappings.iter().enumerate() {
                        vals[i] = m.raw();
                    }
                    let v = vals;
                    quote! { [#(#v),*] }
                })
                .collect();

            defaults.push(quote! { rpdo_cob_id: [#(#cob_ids),*] });
            defaults.push(quote! { rpdo_transmission_type: [#(#tt),*] });
            defaults.push(quote! { rpdo_mapping_count: [#(#map_counts),*] });
            defaults.push(quote! { rpdo_mappings: [#(#map_arrays),*] });
        }
        defaults
    } else {
        Vec::new()
    };

    // ---- Generate metadata table for user entries ----
    let meta_name = format_ident!("{}_META", to_screaming_snake(&name.to_string()));

    let mut all_meta_entries: Vec<TokenStream> = flat
        .iter()
        .map(|e| {
            let index = e.index;
            let subindex = e.subindex;
            let dt = type_to_datatype(&e.var.type_name.to_string()).expect("unsupported type");
            let dt_ident = format_ident!("{}", dt);
            let access_ident = match e.var.access {
                AccessKind::Ro => format_ident!("Ro"),
                AccessKind::Rw => format_ident!("Rw"),
                AccessKind::Wo => format_ident!("Wo"),
                AccessKind::Const => format_ident!("Const"),
            };
            let pdo = e.var.pdo_mappable;
            let entry_name = e.field_name.to_string();
            let max_size = if let Some(cap) = e.var.capacity {
                let cap = cap as u16;
                quote! { Some(#cap) }
            } else {
                quote! { None }
            };
            quote! {
                canopen_core::od::OdEntryMeta {
                    index: #index,
                    subindex: #subindex,
                    data_type: canopen_core::datatypes::DataType::#dt_ident,
                    access: canopen_core::od::AccessType::#access_ident,
                    pdo_mappable: #pdo,
                    name: #entry_name,
                    max_size: #max_size,
                }
            }
        })
        .collect();

    // ---- Generate PDO metadata + read/write arms ----
    let mut pdo_read_arms: Vec<TokenStream> = Vec::new();
    let mut pdo_write_arms: Vec<TokenStream> = Vec::new();
    let mut pdo_sub_counts: std::collections::HashMap<u16, u8> = std::collections::HashMap::new();

    // Helper: generate metadata, read/write for one TPDO communication parameter record
    for (i, pdo) in tpdo_defs.iter().enumerate() {
        let comm_idx = 0x1800u16 + (pdo.number - 1) as u16;
        let map_idx = 0x1A00u16 + (pdo.number - 1) as u16;
        let n = i; // array index

        // Comm params: sub0=highest_subindex(ro), sub1=cob_id(rw), sub2=tt(rw), sub3=inhibit(rw), sub5=event_timer(rw)
        pdo_sub_counts.insert(comm_idx, 5);

        // sub 0: highest subindex = 5
        all_meta_entries.push(gen_pdo_meta(comm_idx, 0, "U8", "Ro", "highest_subindex"));
        pdo_read_arms.push(quote! { (#comm_idx, 0) => { buf[0] = 5; Ok(1) } });
        pdo_write_arms.push(quote! { (#comm_idx, 0) => Err(canopen_core::od::OdError::ReadOnly), });

        // sub 1: cob_id (u32)
        all_meta_entries.push(gen_pdo_meta(comm_idx, 1, "U32", "Rw", "cob_id"));
        pdo_read_arms.push(quote! {
            (#comm_idx, 1) => {
                let bytes = self.tpdo_cob_id[#n].to_le_bytes();
                buf[..4].copy_from_slice(&bytes);
                Ok(4)
            }
        });
        pdo_write_arms.push(quote! {
            (#comm_idx, 1) => {
                if data.len() != 4 { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                let mut arr = [0u8; 4];
                arr.copy_from_slice(&data[..4]);
                self.tpdo_cob_id[#n] = u32::from_le_bytes(arr);
                Ok(())
            }
        });

        // sub 2: transmission_type (u8)
        all_meta_entries.push(gen_pdo_meta(comm_idx, 2, "U8", "Rw", "transmission_type"));
        pdo_read_arms
            .push(quote! { (#comm_idx, 2) => { buf[0] = self.tpdo_transmission_type[#n]; Ok(1) } });
        pdo_write_arms.push(quote! {
            (#comm_idx, 2) => {
                if data.len() != 1 { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                self.tpdo_transmission_type[#n] = data[0];
                Ok(())
            }
        });

        // sub 3: inhibit_time (u16)
        all_meta_entries.push(gen_pdo_meta(comm_idx, 3, "U16", "Rw", "inhibit_time"));
        pdo_read_arms.push(quote! {
            (#comm_idx, 3) => {
                let bytes = self.tpdo_inhibit_time[#n].to_le_bytes();
                buf[..2].copy_from_slice(&bytes);
                Ok(2)
            }
        });
        pdo_write_arms.push(quote! {
            (#comm_idx, 3) => {
                if data.len() != 2 { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                let mut arr = [0u8; 2];
                arr.copy_from_slice(&data[..2]);
                self.tpdo_inhibit_time[#n] = u16::from_le_bytes(arr);
                Ok(())
            }
        });

        // sub 5: event_timer (u16) — sub 4 is reserved
        all_meta_entries.push(gen_pdo_meta(comm_idx, 5, "U16", "Rw", "event_timer"));
        pdo_read_arms.push(quote! {
            (#comm_idx, 5) => {
                let bytes = self.tpdo_event_timer[#n].to_le_bytes();
                buf[..2].copy_from_slice(&bytes);
                Ok(2)
            }
        });
        pdo_write_arms.push(quote! {
            (#comm_idx, 5) => {
                if data.len() != 2 { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                let mut arr = [0u8; 2];
                arr.copy_from_slice(&data[..2]);
                self.tpdo_event_timer[#n] = u16::from_le_bytes(arr);
                Ok(())
            }
        });

        // Mapping params: sub0=count(rw), sub1..8=mapping(rw, guarded by count==0)
        let map_count = tpdo_resolved[i].len() as u8;
        let max_map_sub = if map_count > 0 { map_count } else { 8 };
        pdo_sub_counts.insert(map_idx, max_map_sub);

        // sub 0: mapping count
        all_meta_entries.push(gen_pdo_meta(map_idx, 0, "U8", "Rw", "mapping_count"));
        pdo_read_arms
            .push(quote! { (#map_idx, 0) => { buf[0] = self.tpdo_mapping_count[#n]; Ok(1) } });
        pdo_write_arms.push(quote! {
            (#map_idx, 0) => {
                if data.len() != 1 { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                self.tpdo_mapping_count[#n] = data[0];
                Ok(())
            }
        });

        // sub 1..8: mapping entries
        for sub in 1u8..=8 {
            let arr_idx = (sub - 1) as usize;
            all_meta_entries.push(gen_pdo_meta(map_idx, sub, "U32", "Rw", "mapping_entry"));
            pdo_read_arms.push(quote! {
                (#map_idx, #sub) => {
                    let bytes = self.tpdo_mappings[#n][#arr_idx].to_le_bytes();
                    buf[..4].copy_from_slice(&bytes);
                    Ok(4)
                }
            });
            pdo_write_arms.push(quote! {
                (#map_idx, #sub) => {
                    if self.tpdo_mapping_count[#n] != 0 {
                        return Err(canopen_core::od::OdError::ReadOnly);
                    }
                    if data.len() != 4 { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                    let mut arr = [0u8; 4];
                    arr.copy_from_slice(&data[..4]);
                    self.tpdo_mappings[#n][#arr_idx] = u32::from_le_bytes(arr);
                    Ok(())
                }
            });
        }
    }

    // RPDO: same pattern, fewer params (no inhibit_time, no event_timer)
    for (i, pdo) in rpdo_defs.iter().enumerate() {
        let comm_idx = 0x1400u16 + (pdo.number - 1) as u16;
        let map_idx = 0x1600u16 + (pdo.number - 1) as u16;
        let n = i;

        pdo_sub_counts.insert(comm_idx, 2);

        // sub 0: highest subindex = 2
        all_meta_entries.push(gen_pdo_meta(comm_idx, 0, "U8", "Ro", "highest_subindex"));
        pdo_read_arms.push(quote! { (#comm_idx, 0) => { buf[0] = 2; Ok(1) } });
        pdo_write_arms.push(quote! { (#comm_idx, 0) => Err(canopen_core::od::OdError::ReadOnly), });

        // sub 1: cob_id (u32)
        all_meta_entries.push(gen_pdo_meta(comm_idx, 1, "U32", "Rw", "cob_id"));
        pdo_read_arms.push(quote! {
            (#comm_idx, 1) => {
                let bytes = self.rpdo_cob_id[#n].to_le_bytes();
                buf[..4].copy_from_slice(&bytes);
                Ok(4)
            }
        });
        pdo_write_arms.push(quote! {
            (#comm_idx, 1) => {
                if data.len() != 4 { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                let mut arr = [0u8; 4];
                arr.copy_from_slice(&data[..4]);
                self.rpdo_cob_id[#n] = u32::from_le_bytes(arr);
                Ok(())
            }
        });

        // sub 2: transmission_type (u8)
        all_meta_entries.push(gen_pdo_meta(comm_idx, 2, "U8", "Rw", "transmission_type"));
        pdo_read_arms
            .push(quote! { (#comm_idx, 2) => { buf[0] = self.rpdo_transmission_type[#n]; Ok(1) } });
        pdo_write_arms.push(quote! {
            (#comm_idx, 2) => {
                if data.len() != 1 { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                self.rpdo_transmission_type[#n] = data[0];
                Ok(())
            }
        });

        // Mapping params
        let map_count = rpdo_resolved[i].len() as u8;
        let max_map_sub = if map_count > 0 { map_count } else { 8 };
        pdo_sub_counts.insert(map_idx, max_map_sub);

        all_meta_entries.push(gen_pdo_meta(map_idx, 0, "U8", "Rw", "mapping_count"));
        pdo_read_arms
            .push(quote! { (#map_idx, 0) => { buf[0] = self.rpdo_mapping_count[#n]; Ok(1) } });
        pdo_write_arms.push(quote! {
            (#map_idx, 0) => {
                if data.len() != 1 { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                self.rpdo_mapping_count[#n] = data[0];
                Ok(())
            }
        });

        for sub in 1u8..=8 {
            let arr_idx = (sub - 1) as usize;
            all_meta_entries.push(gen_pdo_meta(map_idx, sub, "U32", "Rw", "mapping_entry"));
            pdo_read_arms.push(quote! {
                (#map_idx, #sub) => {
                    let bytes = self.rpdo_mappings[#n][#arr_idx].to_le_bytes();
                    buf[..4].copy_from_slice(&bytes);
                    Ok(4)
                }
            });
            pdo_write_arms.push(quote! {
                (#map_idx, #sub) => {
                    if self.rpdo_mapping_count[#n] != 0 {
                        return Err(canopen_core::od::OdError::ReadOnly);
                    }
                    if data.len() != 4 { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                    let mut arr = [0u8; 4];
                    arr.copy_from_slice(&data[..4]);
                    self.rpdo_mappings[#n][#arr_idx] = u32::from_le_bytes(arr);
                    Ok(())
                }
            });
        }
    }

    // ---- Generate user-entry read/write arms ----
    let read_arms: Vec<TokenStream> = flat
        .iter()
        .map(|e| {
            let index = e.index;
            let subindex = e.subindex;
            let fname = &e.field_name;
            let ty_str = e.var.type_name.to_string();

            if matches!(e.var.access, AccessKind::Wo) {
                return quote! {
                    (#index, #subindex) => Err(canopen_core::od::OdError::WriteOnly),
                };
            }

            if is_variable_length_type(&ty_str) {
                if ty_str == "visible_string" {
                    return quote! {
                        (#index, #subindex) => {
                            let data = self.#fname.as_bytes();
                            let len = data.len();
                            if buf.len() < len { return Err(canopen_core::od::OdError::ValueTooLong); }
                            buf[..len].copy_from_slice(data);
                            Ok(len)
                        }
                    };
                } else {
                    return quote! {
                        (#index, #subindex) => {
                            let data = self.#fname.as_slice();
                            let len = data.len();
                            if buf.len() < len { return Err(canopen_core::od::OdError::ValueTooLong); }
                            buf[..len].copy_from_slice(data);
                            Ok(len)
                        }
                    };
                }
            }

            let size = type_size(&ty_str).expect("unsupported type");
            if size == 1 {
                if ty_str == "bool" {
                    quote! {
                        (#index, #subindex) => {
                            buf[0] = if self.#fname { 1 } else { 0 };
                            Ok(1)
                        }
                    }
                } else {
                    quote! {
                        (#index, #subindex) => {
                            buf[0] = self.#fname as u8;
                            Ok(1)
                        }
                    }
                }
            } else {
                quote! {
                    (#index, #subindex) => {
                        let bytes = self.#fname.to_le_bytes();
                        buf[..#size].copy_from_slice(&bytes);
                        Ok(#size)
                    }
                }
            }
        })
        .collect();

    let write_arms: Vec<TokenStream> = flat
        .iter()
        .map(|e| {
            let index = e.index;
            let subindex = e.subindex;
            let fname = &e.field_name;
            let ty = &e.var.type_name;
            let ty_str = e.var.type_name.to_string();

            if matches!(e.var.access, AccessKind::Ro | AccessKind::Const) {
                return quote! {
                    (#index, #subindex) => Err(canopen_core::od::OdError::ReadOnly),
                };
            }

            if is_variable_length_type(&ty_str) {
                if use_alloc {
                    if ty_str == "visible_string" {
                        return quote! {
                            (#index, #subindex) => {
                                let s = core::str::from_utf8(data)
                                    .map_err(|_| canopen_core::od::OdError::DataTypeMismatch)?;
                                self.#fname.clear();
                                self.#fname.push_str(s);
                                Ok(())
                            }
                        };
                    } else {
                        return quote! {
                            (#index, #subindex) => {
                                self.#fname.clear();
                                self.#fname.extend_from_slice(data);
                                Ok(())
                            }
                        };
                    }
                } else {
                    if ty_str == "visible_string" {
                        return quote! {
                            (#index, #subindex) => {
                                let s = core::str::from_utf8(data)
                                    .map_err(|_| canopen_core::od::OdError::DataTypeMismatch)?;
                                self.#fname.clear();
                                self.#fname.push_str(s)
                                    .map_err(|_| canopen_core::od::OdError::ValueTooLong)?;
                                Ok(())
                            }
                        };
                    } else {
                        return quote! {
                            (#index, #subindex) => {
                                self.#fname.clear();
                                self.#fname.extend_from_slice(data)
                                    .map_err(|_| canopen_core::od::OdError::ValueTooLong)?;
                                Ok(())
                            }
                        };
                    }
                }
            }

            let size = type_size(&ty_str).expect("unsupported type");
            if ty_str == "bool" {
                quote! {
                    (#index, #subindex) => {
                        if data.len() != 1 { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                        self.#fname = data[0] != 0;
                        Ok(())
                    }
                }
            } else if size == 1 {
                quote! {
                    (#index, #subindex) => {
                        if data.len() != 1 { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                        self.#fname = data[0] as #ty;
                        Ok(())
                    }
                }
            } else {
                quote! {
                    (#index, #subindex) => {
                        if data.len() != #size { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                        let mut arr = [0u8; #size];
                        arr.copy_from_slice(&data[..#size]);
                        self.#fname = #ty::from_le_bytes(arr);
                        Ok(())
                    }
                }
            }
        })
        .collect();

    // ---- Generate sub_count ----
    let mut sub_counts: std::collections::HashMap<u16, u8> = std::collections::HashMap::new();
    for e in &flat {
        let counter = sub_counts.entry(e.index).or_insert(0);
        if e.subindex > *counter {
            *counter = e.subindex;
        }
    }
    // Merge PDO sub counts
    for (idx, max_sub) in &pdo_sub_counts {
        sub_counts.insert(*idx, *max_sub);
    }
    // Merge array sub counts
    for a in &arrays {
        sub_counts.insert(a.index, a.def.count as u8);
    }
    let sub_count_arms: Vec<TokenStream> = sub_counts
        .iter()
        .map(|(index, max_sub)| {
            quote! { #index => Some(#max_sub), }
        })
        .collect();

    // ---- Generate optional validate_write override ----
    let validate_write_impl = if let Some(fn_name) = &od.validate_write_fn {
        quote! {
            fn validate_write(&self, index: u16, subindex: u8, data: &[u8]) -> Result<(), canopen_core::od::OdError> {
                self.#fn_name(index, subindex, data)
            }
        }
    } else {
        quote! {}
    };

    // ---- Generate PDO config helper methods ----
    let pdo_helper_methods = if has_pdos {
        let tc = tpdo_count;
        let rc = rpdo_count;

        // Generate tpdo_configs() method body
        let tpdo_config_items: Vec<TokenStream> = tpdo_defs.iter().enumerate().map(|(i, pdo)| {
            let n = pdo.number;
            // A stored COB-ID of 0 means "resolve at runtime with node_id":
            // node-relative base if declared, else the predefined default
            // (PDO 1-4 only). PDOs >4 without a node-relative base always
            // have an absolute cob_id (enforced by the DSL), so 0 can only
            // mean "disabled" there.
            let default_cob = match pdo.cob_id {
                Some(CobIdSpec::NodeRelative(base)) => {
                    let base = base as u16;
                    quote! { #base + node_id.raw() as u16 }
                }
                _ if n <= 4 => {
                    let base: u16 = 0x180 + (n - 1) * 0x100;
                    quote! { #base + node_id.raw() as u16 }
                }
                _ => quote! { 0u16 },
            };
            quote! {
                {
                    let raw = self.tpdo_cob_id[#i];
                    let cob_id = if raw == 0 {
                        #default_cob
                    } else {
                        (raw & 0x7FF) as u16
                    };
                    let enabled = (raw & 0x8000_0000) == 0 && cob_id != 0;

                    let mut mappings = canopen_core::pdo::heapless_vec_new();
                    let count = self.tpdo_mapping_count[#i] as usize;
                    let mut j = 0;
                    while j < count && j < 8 {
                        let _ = canopen_core::pdo::heapless_vec_push(&mut mappings,
                            canopen_core::pdo::PdoMapping::from_mapping_value(self.tpdo_mappings[#i][j]));
                        j += 1;
                    }

                    canopen_core::pdo::TpdoConfig {
                        od_number: #n,
                        cob_id,
                        transmission_type: self.tpdo_transmission_type[#i],
                        inhibit_time_100us: self.tpdo_inhibit_time[#i],
                        event_timer_ms: self.tpdo_event_timer[#i],
                        mappings,
                        enabled,
                    }
                }
            }
        }).collect();

        let rpdo_config_items: Vec<TokenStream> = rpdo_defs.iter().enumerate().map(|(i, pdo)| {
            let n = pdo.number;
            let default_cob = match pdo.cob_id {
                Some(CobIdSpec::NodeRelative(base)) => {
                    let base = base as u16;
                    quote! { #base + node_id.raw() as u16 }
                }
                _ if n <= 4 => {
                    let base: u16 = 0x200 + (n - 1) * 0x100;
                    quote! { #base + node_id.raw() as u16 }
                }
                _ => quote! { 0u16 },
            };
            quote! {
                {
                    let raw = self.rpdo_cob_id[#i];
                    let cob_id = if raw == 0 {
                        #default_cob
                    } else {
                        (raw & 0x7FF) as u16
                    };
                    let enabled = (raw & 0x8000_0000) == 0 && cob_id != 0;

                    let mut mappings = canopen_core::pdo::heapless_vec_new();
                    let count = self.rpdo_mapping_count[#i] as usize;
                    let mut j = 0;
                    while j < count && j < 8 {
                        let _ = canopen_core::pdo::heapless_vec_push(&mut mappings,
                            canopen_core::pdo::PdoMapping::from_mapping_value(self.rpdo_mappings[#i][j]));
                        j += 1;
                    }

                    canopen_core::pdo::RpdoConfig {
                        od_number: #n,
                        cob_id,
                        transmission_type: self.rpdo_transmission_type[#i],
                        mappings,
                        enabled,
                    }
                }
            }
        }).collect();

        quote! {
            pub const TPDO_COUNT: usize = #tc;
            pub const RPDO_COUNT: usize = #rc;

            /// Build TPDO configs from current OD values.
            /// COB-IDs of 0 are resolved to predefined defaults using `node_id`.
            pub fn tpdo_configs(&self, node_id: canopen_core::cobid::NodeId) -> [canopen_core::pdo::TpdoConfig; #tc] {
                [#(#tpdo_config_items),*]
            }

            /// Build RPDO configs from current OD values.
            pub fn rpdo_configs(&self, node_id: canopen_core::cobid::NodeId) -> [canopen_core::pdo::RpdoConfig; #rc] {
                [#(#rpdo_config_items),*]
            }
        }
    } else {
        quote! {}
    };

    // Add array metadata entries
    all_meta_entries.extend(array_meta_entries);

    // ---- Generate 0x1021 (Store EDS) and 0x1022 (Store Format) ----
    // These auto-entries expose the compressed EDS content via SDO for device self-description.
    let eds_compressed_len = eds_compressed.len() as u16;
    let eds_compressed_bytes = &eds_compressed;
    all_meta_entries.push(quote! {
        canopen_core::od::OdEntryMeta {
            index: 0x1021, subindex: 0,
            data_type: canopen_core::datatypes::DataType::Domain,
            access: canopen_core::od::AccessType::Ro,
            pdo_mappable: false,
            name: "store_eds",
            max_size: Some(#eds_compressed_len),
        }
    });
    all_meta_entries.push(quote! {
        canopen_core::od::OdEntryMeta {
            index: 0x1022, subindex: 0,
            data_type: canopen_core::datatypes::DataType::U8,
            access: canopen_core::od::AccessType::Ro,
            pdo_mappable: false,
            name: "store_format",
            max_size: None,
        }
    });
    pdo_read_arms.push(quote! {
        (0x1021, 0) => {
            let data = Self::EDS_COMPRESSED;
            let len = data.len();
            if buf.len() < len { return Err(canopen_core::od::OdError::ValueTooLong); }
            buf[..len].copy_from_slice(data);
            Ok(len)
        }
    });
    pdo_read_arms.push(quote! {
        (0x1022, 0) => {
            buf[0] = 1; // 1 = zlib/deflate compressed
            Ok(1)
        }
    });
    pdo_write_arms.push(quote! {
        (0x1021, 0) => Err(canopen_core::od::OdError::ReadOnly),
    });
    pdo_write_arms.push(quote! {
        (0x1022, 0) => Err(canopen_core::od::OdError::ReadOnly),
    });

    // ---- Generate OD address constants ----
    // One `pub const FIELD: (u16, u8)` per entry, so application code can
    // refer to OD addresses by name instead of magic numbers.
    let mut addr_consts: Vec<TokenStream> = Vec::new();
    for e in &flat {
        let const_name = format_ident!("{}", to_screaming_snake(&e.field_name.to_string()));
        let index = e.index;
        let sub = e.subindex;
        let doc = format!(
            "OD address of `{}` (0x{:04X}:{}).",
            e.field_name, index, sub
        );
        addr_consts.push(quote! {
            #[doc = #doc]
            pub const #const_name: (u16, u8) = (#index, #sub);
        });
    }
    for a in &arrays {
        let const_name = format_ident!("{}_INDEX", to_screaming_snake(&a.field_name.to_string()));
        let index = a.index;
        let count = a.def.count;
        let doc = format!(
            "OD index of array `{}` (0x{:04X}, subindices 1..={}).",
            a.field_name, index, count
        );
        addr_consts.push(quote! {
            #[doc = #doc]
            pub const #const_name: u16 = #index;
        });
    }

    // ---- Generate typed change enum + OdChanges impl ----
    // One variant per writable entry: the protocol stack can only ever modify
    // rw/wo entries (SDO download, RPDO), so events map onto these variants.
    let change_name = format_ident!("{}Change", name);
    let mut change_variants: Vec<TokenStream> = Vec::new();
    let mut decode_arms: Vec<TokenStream> = Vec::new();
    for e in &flat {
        if !matches!(e.var.access, AccessKind::Rw | AccessKind::Wo) {
            continue;
        }
        let variant = format_ident!("{}", to_camel_case(&e.field_name.to_string()));
        let fname = &e.field_name;
        let index = e.index;
        let sub = e.subindex;
        let ty_str = e.var.type_name.to_string();
        if is_variable_length_type(&ty_str) {
            // Value not carried: read `od().field` if needed.
            let doc = format!("`{}` (0x{:04X}:{}) was written.", fname, index, sub);
            change_variants.push(quote! { #[doc = #doc] #variant });
            decode_arms.push(quote! {
                (#index, #sub) => Some(#change_name::#variant),
            });
        } else {
            let ty = &e.var.type_name;
            let doc = format!(
                "`{}` (0x{:04X}:{}) was written; carries the current value.",
                fname, index, sub
            );
            change_variants.push(quote! { #[doc = #doc] #variant(#ty) });
            decode_arms.push(quote! {
                (#index, #sub) => Some(#change_name::#variant(self.#fname)),
            });
        }
    }
    for a in &arrays {
        if !matches!(a.def.access, AccessKind::Rw | AccessKind::Wo) {
            continue;
        }
        let variant = format_ident!("{}", to_camel_case(&a.field_name.to_string()));
        let fname = &a.field_name;
        let index = a.index;
        let count_u8 = a.def.count as u8;
        let ty = &a.def.element_type;
        let doc = format!(
            "`{}[subindex]` (0x{:04X}) was written; carries (subindex, current value).",
            fname, index
        );
        change_variants.push(quote! { #[doc = #doc] #variant(u8, #ty) });
        decode_arms.push(quote! {
            (#index, sub) if sub >= 1 && sub <= #count_u8 =>
                Some(#change_name::#variant(sub, self.#fname[(sub as usize) - 1])),
        });
    }

    let change_doc = format!(
        "Typed OD change, decoded from an `OdEvent` by [`{name}::decode_event`] \
         (usually via `node.next_change()`). One variant per writable OD entry."
    );

    // ---- Node type alias + PdoConfigSource impl ----
    let tc = tpdo_count;
    let rc = rpdo_count;
    let node_alias = format_ident!("{}Node", name);
    let alias_doc = format!(
        "[`Node`](canopen_core::node::Node) preconfigured for [`{name}`] ({tc} TPDO, {rc} RPDO)."
    );

    let pdo_source_impl = if has_pdos {
        quote! {
            impl canopen_core::pdo::PdoConfigSource<#tc, #rc> for #name {
                fn tpdo_configs(&self, node_id: canopen_core::cobid::NodeId) -> [canopen_core::pdo::TpdoConfig; #tc] {
                    #name::tpdo_configs(self, node_id)
                }
                fn rpdo_configs(&self, node_id: canopen_core::cobid::NodeId) -> [canopen_core::pdo::RpdoConfig; #rc] {
                    #name::rpdo_configs(self, node_id)
                }
            }
        }
    } else {
        quote! {
            impl canopen_core::pdo::PdoConfigSource<0, 0> for #name {
                fn tpdo_configs(&self, _node_id: canopen_core::cobid::NodeId) -> [canopen_core::pdo::TpdoConfig; 0] {
                    []
                }
                fn rpdo_configs(&self, _node_id: canopen_core::cobid::NodeId) -> [canopen_core::pdo::RpdoConfig; 0] {
                    []
                }
            }
        }
    };

    // ---- Assemble output ----
    let meta_len = all_meta_entries.len();

    quote! {
        #[derive(Clone)]
        #vis struct #name {
            #(#struct_fields,)*
            #(#array_struct_fields,)*
            #(#pdo_struct_fields,)*
        }

        static #meta_name: [canopen_core::od::OdEntryMeta; #meta_len] = [
            #(#all_meta_entries,)*
        ];

        impl #name {
            /// EDS (Electronic Data Sheet) content for this object dictionary (uncompressed).
            pub const EDS: &'static str = #eds_content;

            /// Deflate-compressed EDS content, served via 0x1021 (Store EDS).
            pub const EDS_COMPRESSED: &'static [u8] = &[#(#eds_compressed_bytes),*];

            #(#addr_consts)*

            pub fn new() -> Self {
                Self {
                    #(#field_defaults,)*
                    #(#array_field_defaults,)*
                    #(#pdo_field_defaults,)*
                }
            }

            #pdo_helper_methods
        }

        impl canopen_core::od::ObjectDictionary for #name {
            fn lookup(&self, index: u16, subindex: u8) -> Option<&'static canopen_core::od::OdEntryMeta> {
                #meta_name.iter().find(|e| e.index == index && e.subindex == subindex)
            }

            fn read(&self, index: u16, subindex: u8, buf: &mut [u8]) -> Result<usize, canopen_core::od::OdError> {
                match (index, subindex) {
                    #(#read_arms)*
                    #(#array_read_arms)*
                    #(#pdo_read_arms)*
                    _ => Err(canopen_core::od::OdError::NotFound),
                }
            }

            fn write(&mut self, index: u16, subindex: u8, data: &[u8]) -> Result<(), canopen_core::od::OdError> {
                match (index, subindex) {
                    #(#write_arms)*
                    #(#array_write_arms)*
                    #(#pdo_write_arms)*
                    _ => Err(canopen_core::od::OdError::NotFound),
                }
            }

            fn sub_count(&self, index: u16) -> Option<u8> {
                match index {
                    #(#sub_count_arms)*
                    _ => None,
                }
            }

            #validate_write_impl
        }

        #[doc = #change_doc]
        #[derive(Clone, Copy, Debug, PartialEq)]
        #vis enum #change_name {
            #(#change_variants,)*
        }

        impl canopen_core::od::OdChanges for #name {
            type Change = #change_name;

            fn decode_event(&self, event: canopen_core::od::OdEvent) -> Option<#change_name> {
                match (event.index, event.subindex) {
                    #(#decode_arms)*
                    _ => None,
                }
            }
        }

        #pdo_source_impl

        #[doc = #alias_doc]
        #vis type #node_alias = canopen_core::node::Node<#name, #tc, #rc>;
    }
}

fn gen_pdo_meta(index: u16, subindex: u8, dt: &str, access: &str, entry_name: &str) -> TokenStream {
    let dt_ident = format_ident!("{}", dt);
    let access_ident = format_ident!("{}", access);
    quote! {
        canopen_core::od::OdEntryMeta {
            index: #index,
            subindex: #subindex,
            data_type: canopen_core::datatypes::DataType::#dt_ident,
            access: canopen_core::od::AccessType::#access_ident,
            pdo_mappable: false,
            name: #entry_name,
            max_size: None,
        }
    }
}

/// `echo_in` → `EchoIn` (for change enum variant names).
fn to_camel_case(name: &str) -> String {
    name.split('_')
        .filter(|part| !part.is_empty())
        .map(|part| {
            let mut chars = part.chars();
            match chars.next() {
                Some(first) => first.to_ascii_uppercase().to_string() + chars.as_str(),
                None => String::new(),
            }
        })
        .collect()
}

fn to_screaming_snake(name: &str) -> String {
    let mut result = String::new();
    for (i, c) in name.chars().enumerate() {
        if c.is_uppercase() && i > 0 {
            result.push('_');
        }
        result.push(c.to_ascii_uppercase());
    }
    result
}
