use crate::codegen::{flatten, resolve_pdo_mappings, FlatEntry, ResolvedMapping};
use crate::dsl::{CobIdSpec, PdoDef, PdoDirection, VarDef};
use crate::eds_parser;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};

pub fn generate(def: eds_parser::EdsDefinition) -> TokenStream {
    let od = match eds_parser::parse_eds_to_od(def) {
        Ok(od) => od,
        Err(error) => return error.to_compile_error(),
    };
    let vis = &od.vis;
    let name = &od.name;
    let (payload_types, methods) = match generate_parts(&od, quote! { self.node_id.raw() }) {
        Ok(parts) => parts,
        Err(error) => return error,
    };

    quote! {
        #(#payload_types)*

        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        #vis struct #name {
            node_id: canopen_core::cobid::NodeId,
        }

        impl #name {
            pub const fn new(node_id: canopen_core::cobid::NodeId) -> Self {
                Self { node_id }
            }

            pub const fn node_id(&self) -> canopen_core::cobid::NodeId {
                self.node_id
            }

            #(#methods)*
        }
    }
}

pub(crate) fn generate_for_device(od: &crate::dsl::OdDefinition) -> TokenStream {
    let name = &od.name;
    let (payload_types, methods) = match generate_parts(od, quote! { self.target().raw() }) {
        Ok(parts) => parts,
        Err(error) => return error,
    };
    quote! {
        #(#payload_types)*
        impl #name {
            #(#methods)*
        }
    }
}

fn generate_parts(
    od: &crate::dsl::OdDefinition,
    node_id_raw: TokenStream,
) -> Result<(Vec<TokenStream>, Vec<TokenStream>), TokenStream> {
    let vis = &od.vis;
    let name = &od.name;
    let flat = flatten(&od.entries);
    let mut payload_types = Vec::new();
    let mut methods = Vec::new();
    // A disabled-but-mapped PDO still has a complete typed wire contract and
    // may be enabled later by the master, so it keeps its direct codec.
    // Unmapped communication records have no payload shape to generate.
    let mapped_pdos: Vec<&PdoDef> = od
        .pdos
        .iter()
        .filter(|pdo| !pdo.mappings.is_empty())
        .collect();
    // Generic dispatch describes what the EDS says is active at power-up.
    // Disabled PDOs must not claim frames until the caller addresses them
    // explicitly after enabling them.
    let dispatch_pdos: Vec<&PdoDef> = mapped_pdos
        .iter()
        .copied()
        .filter(|pdo| !pdo_is_disabled(pdo))
        .collect();

    for (i, pdo) in dispatch_pdos.iter().enumerate() {
        for other in dispatch_pdos.iter().skip(i + 1) {
            if effective_cob_id_spec(pdo) == effective_cob_id_spec(other) {
                return Err(syn::Error::new(
                    proc_macro2::Span::call_site(),
                    format!(
                        "{}[{}] and {}[{}] resolve to the same COB-ID",
                        method_stem(pdo),
                        pdo.number,
                        method_stem(other),
                        other.number,
                    ),
                )
                .to_compile_error());
            }
        }
    }

    for pdo in mapped_pdos {
        let mappings = resolve_pdo_mappings(pdo, &flat);
        match generate_pdo(vis, name, pdo, &mappings, &flat, &node_id_raw) {
            Ok((payload, generated_methods)) => {
                payload_types.push(payload);
                methods.extend(generated_methods);
            }
            Err(error) => {
                return Err(
                    syn::Error::new(proc_macro2::Span::call_site(), error).to_compile_error()
                );
            }
        }
    }

    let tpdo_enum = format_ident!("{}Tpdo", name);
    let tpdo_variants = dispatch_pdos.iter().filter_map(|pdo| {
        if pdo.direction != PdoDirection::Tpdo {
            return None;
        }
        let variant = format_ident!("Tpdo{}", pdo.number);
        let payload = format_ident!("{}Tpdo{}", name, pdo.number);
        Some(quote! { #variant(#payload) })
    });
    payload_types.push(quote! {
        /// A decoded TPDO produced by the remote device.
        #[derive(Clone, Copy, Debug, PartialEq)]
        #vis enum #tpdo_enum {
            #(#tpdo_variants,)*
        }
    });

    let tpdo_pdos: Vec<&PdoDef> = dispatch_pdos
        .iter()
        .copied()
        .filter(|pdo| pdo.direction == PdoDirection::Tpdo)
        .collect();
    let potentially_ambiguous_pairs = tpdo_pdos.iter().enumerate().flat_map(|(i, pdo)| {
        tpdo_pdos.iter().skip(i + 1).filter_map(move |other| {
            node_id_dependent_collision(pdo, other).then_some((*pdo, *other))
        })
    });
    let ambiguity_checks = potentially_ambiguous_pairs.map(|(pdo, other)| {
        let cob_method = format_ident!("tpdo{}_cob_id", pdo.number);
        let other_cob_method = format_ident!("tpdo{}_cob_id", other.number);
        quote! {
            if frame.raw_id() == self.#cob_method().raw()
                && frame.raw_id() == self.#other_cob_method().raw()
            {
                return Err(canopen_core::pdo::PdoDecodeError::AmbiguousCobId {
                    cob_id: frame.raw_id(),
                });
            }
        }
    });
    let decode_arms = dispatch_pdos.iter().filter_map(|pdo| {
        if pdo.direction != PdoDirection::Tpdo {
            return None;
        }
        let cob_method = format_ident!("tpdo{}_cob_id", pdo.number);
        let decode_method = format_ident!("decode_tpdo{}", pdo.number);
        let variant = format_ident!("Tpdo{}", pdo.number);
        Some(quote! {
            if frame.raw_id() == self.#cob_method().raw() {
                return self
                    .#decode_method(frame)
                    .map(|payload| Some(#tpdo_enum::#variant(payload)));
            }
        })
    });
    methods.push(quote! {
        /// Decode any initially active TPDO configured by the EDS for this remote node.
        /// Unrelated CAN frames return `Ok(None)`.
        pub fn decode(
            &self,
            frame: &canopen_core::transport::CanFrame,
        ) -> Result<Option<#tpdo_enum>, canopen_core::pdo::PdoDecodeError> {
            #(#ambiguity_checks)*
            #(#decode_arms)*
            Ok(None)
        }
    });
    Ok((payload_types, methods))
}

fn generate_pdo(
    vis: &syn::Visibility,
    client_name: &syn::Ident,
    pdo: &PdoDef,
    mappings: &[ResolvedMapping],
    flat: &[FlatEntry],
    node_id_raw: &TokenStream,
) -> Result<(TokenStream, Vec<TokenStream>), String> {
    let direction = match pdo.direction {
        PdoDirection::Tpdo => "Tpdo",
        PdoDirection::Rpdo => "Rpdo",
    };
    let payload_name = format_ident!("{}{}{}", client_name, direction, pdo.number);
    let fields = mapped_fields(pdo, mappings, flat)?;
    let field_defs = fields
        .iter()
        .map(|field| {
            let name = &field.name;
            let ty = rust_type(field.var)?;
            Ok(quote! { pub #name: #ty })
        })
        .collect::<Result<Vec<_>, String>>()?;
    let payload_doc = format!("Typed payload for remote {direction}{}.", pdo.number);
    let payload = quote! {
        #[doc = #payload_doc]
        #[derive(Clone, Copy, Debug, PartialEq, Default)]
        #vis struct #payload_name {
            #(#field_defs,)*
        }
    };

    let cob_expr = cob_id_expr(pdo, node_id_raw)?;
    let cob_method = format_ident!("{}{}_cob_id", method_stem(pdo), pdo.number);
    let cob = quote! {
        pub fn #cob_method(&self) -> canopen_core::cobid::CobId {
            let raw = #cob_expr;
            match canopen_core::cobid::CobId::new(raw) {
                Some(cob_id) => cob_id,
                None => panic!("EDS contains an invalid PDO COB-ID"),
            }
        }
    };

    let payload_bits: usize = mappings
        .iter()
        .map(|mapping| mapping.bit_length as usize)
        .sum();
    let payload_bytes = payload_bits.div_ceil(8);
    let mut generated = vec![cob];
    match pdo.direction {
        PdoDirection::Tpdo => {
            let decode_method = format_ident!("decode_tpdo{}", pdo.number);
            let decode_fields = fields.iter().map(|field| decode_field(field));
            generated.push(quote! {
                pub fn #decode_method(
                    &self,
                    frame: &canopen_core::transport::CanFrame,
                ) -> Result<#payload_name, canopen_core::pdo::PdoDecodeError> {
                    let expected_cob_id = self.#cob_method().raw();
                    if frame.raw_id() != expected_cob_id {
                        return Err(canopen_core::pdo::PdoDecodeError::WrongCobId {
                            expected: expected_cob_id,
                            actual: frame.raw_id(),
                        });
                    }
                    if frame.data().len() < #payload_bytes {
                        return Err(canopen_core::pdo::PdoDecodeError::PayloadTooShort {
                            expected: #payload_bytes,
                            actual: frame.data().len(),
                        });
                    }
                    let data = frame.data();
                    Ok(#payload_name {
                        #(#decode_fields,)*
                    })
                }
            });
        }
        PdoDirection::Rpdo => {
            let encode_method = format_ident!("rpdo{}", pdo.number);
            let encode_fields = fields.iter().map(encode_field);
            generated.push(quote! {
                pub fn #encode_method(&self, payload: #payload_name) -> canopen_core::transport::CanFrame {
                    let mut data = [0u8; 8];
                    #(#encode_fields)*
                    match canopen_core::transport::CanFrame::new(
                        self.#cob_method().raw(),
                        &data[..#payload_bytes],
                    ) {
                        Some(frame) => frame,
                        None => panic!("EDS contains an invalid PDO definition"),
                    }
                }
            });
        }
    }

    Ok((payload, generated))
}

struct MappedField<'a> {
    name: syn::Ident,
    var: &'a VarDef,
    bit_offset: usize,
    bit_length: usize,
}

fn mapped_fields<'a>(
    pdo: &PdoDef,
    mappings: &[ResolvedMapping],
    flat: &'a [FlatEntry],
) -> Result<Vec<MappedField<'a>>, String> {
    let mut fields = Vec::new();
    let mut bit_offset = 0;
    for mapping in mappings {
        let bit_length = mapping.bit_length as usize;
        if matches!(mapping.index, 0x0001..=0x0007) {
            bit_offset += bit_length;
            continue;
        }
        let entry = flat
            .iter()
            .find(|entry| entry.index == mapping.index && entry.subindex == mapping.subindex)
            .ok_or_else(|| {
                format!(
                    "{}[{}] maps unknown OD address 0x{:04X}:{:02X}",
                    method_stem(pdo),
                    pdo.number,
                    mapping.index,
                    mapping.subindex,
                )
            })?;
        let object_bits = wire_bits(&entry.var)?;
        if bit_length > object_bits {
            return Err(format!(
                "{}[{}] maps {} bits from 0x{:04X}:{:02X}, but `{}` is only {} bits",
                method_stem(pdo),
                pdo.number,
                bit_length,
                mapping.index,
                mapping.subindex,
                entry.var.type_name,
                object_bits,
            ));
        }
        if fields
            .iter()
            .any(|field: &MappedField<'_>| field.name == entry.field_name)
        {
            return Err(format!(
                "{}[{}] maps `{}` more than once; generated payload field names must be unique",
                method_stem(pdo),
                pdo.number,
                entry.field_name,
            ));
        }
        fields.push(MappedField {
            name: entry.field_name.clone(),
            var: &entry.var,
            bit_offset,
            bit_length,
        });
        bit_offset += bit_length;
    }
    Ok(fields)
}

fn method_stem(pdo: &PdoDef) -> &'static str {
    match pdo.direction {
        PdoDirection::Tpdo => "tpdo",
        PdoDirection::Rpdo => "rpdo",
    }
}

fn effective_cob_id_spec(pdo: &PdoDef) -> Option<CobIdSpec> {
    pdo.cob_id.map(mask_cob_id_flags).or_else(|| {
        let base = match (pdo.direction, pdo.number) {
            (PdoDirection::Tpdo, 1) => 0x180,
            (PdoDirection::Tpdo, 2) => 0x280,
            (PdoDirection::Tpdo, 3) => 0x380,
            (PdoDirection::Tpdo, 4) => 0x480,
            (PdoDirection::Rpdo, 1) => 0x200,
            (PdoDirection::Rpdo, 2) => 0x300,
            (PdoDirection::Rpdo, 3) => 0x400,
            (PdoDirection::Rpdo, 4) => 0x500,
            _ => return None,
        };
        Some(CobIdSpec::NodeRelative(base))
    })
}

fn mask_cob_id_flags(spec: CobIdSpec) -> CobIdSpec {
    match spec {
        CobIdSpec::Absolute(raw) => CobIdSpec::Absolute(raw & 0x3fff_ffff),
        CobIdSpec::NodeRelative(base) => CobIdSpec::NodeRelative(base & 0x3fff_ffff),
    }
}

fn node_id_dependent_collision(pdo: &PdoDef, other: &PdoDef) -> bool {
    match (effective_cob_id_spec(pdo), effective_cob_id_spec(other)) {
        (Some(CobIdSpec::Absolute(id)), Some(CobIdSpec::NodeRelative(base)))
        | (Some(CobIdSpec::NodeRelative(base)), Some(CobIdSpec::Absolute(id))) => id
            .checked_sub(base)
            .is_some_and(|node_id| (1..=127).contains(&node_id)),
        _ => false,
    }
}

fn pdo_is_disabled(pdo: &PdoDef) -> bool {
    match pdo.cob_id {
        Some(CobIdSpec::Absolute(raw) | CobIdSpec::NodeRelative(raw)) => raw & 0x8000_0000 != 0,
        None => false,
    }
}

fn cob_id_expr(pdo: &PdoDef, node_id_raw: &TokenStream) -> Result<TokenStream, String> {
    let spec = effective_cob_id_spec(pdo);
    let Some(spec) = spec else {
        return Err(format!(
            "{}[{}] has no predefined COB-ID and must declare one",
            method_stem(pdo),
            pdo.number
        ));
    };
    if matches!(
        spec,
        CobIdSpec::Absolute(raw) | CobIdSpec::NodeRelative(raw)
            if raw & 0x2000_0000 != 0
    ) {
        return Err(format!(
            "{}[{}] uses a 29-bit CAN ID; only 11-bit COB-IDs are supported",
            method_stem(pdo),
            pdo.number,
        ));
    }
    match spec {
        CobIdSpec::Absolute(raw) if raw <= 0x7ff => {
            let raw = raw as u16;
            Ok(quote! { #raw })
        }
        CobIdSpec::NodeRelative(base) if base + 127 <= 0x7ff => {
            let base = base as u16;
            Ok(quote! { #base + #node_id_raw as u16 })
        }
        _ => Err(format!(
            "{}[{}] has an invalid COB-ID",
            method_stem(pdo),
            pdo.number
        )),
    }
}

fn wire_bits(var: &VarDef) -> Result<usize, String> {
    let ty = var.type_name.to_string();
    match ty.as_str() {
        "bool" | "u8" | "i8" => Ok(8),
        "u16" | "i16" => Ok(16),
        "u24" | "i24" => Ok(24),
        "u32" | "i32" | "f32" => Ok(32),
        "f64" => Ok(64),
        _ => Err(format!("PDO codec does not support mapped type `{ty}`")),
    }
}

fn rust_type(var: &VarDef) -> Result<TokenStream, String> {
    let ty = var.type_name.to_string();
    Ok(match ty.as_str() {
        "bool" => quote! { bool },
        "u8" => quote! { u8 },
        "u16" => quote! { u16 },
        "u24" | "u32" => quote! { u32 },
        "i8" => quote! { i8 },
        "i16" => quote! { i16 },
        "i24" | "i32" => quote! { i32 },
        "f32" => quote! { f32 },
        "f64" => quote! { f64 },
        _ => return Err(format!("PDO codec does not support mapped type `{ty}`")),
    })
}

fn encode_field(field: &MappedField<'_>) -> TokenStream {
    let name = &field.name;
    let offset = field.bit_offset;
    let bits = field.bit_length;
    let ty = field.var.type_name.to_string();
    let bytes = match ty.as_str() {
        "bool" => quote! { [payload.#name as u8] },
        "u8" | "i8" => quote! { [payload.#name as u8] },
        _ => quote! { payload.#name.to_le_bytes() },
    };
    quote! {
        {
            let object_data = #bytes;
            canopen_core::pdo::copy_pdo_bits(&object_data, 0, &mut data, #offset, #bits);
        }
    }
}

fn decode_field(field: &MappedField<'_>) -> TokenStream {
    let name = &field.name;
    let offset = field.bit_offset;
    let bits = field.bit_length;
    let ty = field.var.type_name.to_string();
    let size: usize = match ty.as_str() {
        "bool" | "u8" | "i8" => 1,
        "u16" | "i16" => 2,
        "u24" | "i24" | "u32" | "i32" | "f32" => 4,
        "f64" => 8,
        _ => unreachable!(),
    };
    let value = match ty.as_str() {
        "bool" => quote! { object_data[0] != 0 },
        "u8" => quote! { object_data[0] },
        "i8" => quote! { object_data[0] as i8 },
        "u16" => quote! { u16::from_le_bytes(object_data) },
        "i16" => quote! { i16::from_le_bytes(object_data) },
        "u24" | "u32" => quote! { u32::from_le_bytes(object_data) },
        "i24" => quote! {{
            if object_data[2] & 0x80 != 0 { object_data[3] = 0xff; }
            i32::from_le_bytes(object_data)
        }},
        "i32" => quote! { i32::from_le_bytes(object_data) },
        "f32" => quote! { f32::from_le_bytes(object_data) },
        "f64" => quote! { f64::from_le_bytes(object_data) },
        _ => unreachable!(),
    };
    quote! {
        #name: {
            let mut object_data = [0u8; #size];
            canopen_core::pdo::copy_pdo_bits(data, #offset, &mut object_data, 0, #bits);
            #value
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::{AccessKind, EntryKind, MappingKind, OdDefinition, OdEntry, PdoMappingDef};
    use quote::quote;
    use syn::parse_quote;

    fn test_od(pdos: Vec<PdoDef>, type_name: syn::Ident) -> OdDefinition {
        OdDefinition {
            vis: parse_quote!(pub),
            name: parse_quote!(Generated),
            entries: vec![OdEntry {
                index: 0x6041,
                name: parse_quote!(statusword),
                kind: EntryKind::Var(VarDef {
                    type_name,
                    capacity: None,
                    default_value: None,
                    access: AccessKind::Ro,
                    pdo_mappable: true,
                }),
            }],
            pdos,
            sdo_servers: Vec::new(),
            export_eds_path: None,
            use_alloc: false,
            validate_write_fn: None,
        }
    }

    fn pdo(
        direction: PdoDirection,
        number: u16,
        cob_id: Option<CobIdSpec>,
        bit_length: Option<u8>,
    ) -> PdoDef {
        PdoDef {
            direction,
            number,
            cob_id,
            transmission_type: 255,
            inhibit_time: 0,
            event_timer: 0,
            mapping: MappingKind::Immutable,
            mappings: bit_length
                .map(|bit_length| {
                    vec![PdoMappingDef {
                        field_name: parse_quote!(statusword),
                        bit_length: Some(bit_length),
                        raw_mapping: None,
                    }]
                })
                .unwrap_or_default(),
        }
    }

    #[test]
    fn over_wide_mapping_is_rejected_during_expansion() {
        let od = test_od(
            vec![pdo(PdoDirection::Tpdo, 1, None, Some(16))],
            parse_quote!(u8),
        );

        let error = match generate_parts(&od, quote!(self.node_id.raw())) {
            Ok(_) => panic!("over-wide mapping unexpectedly generated a codec"),
            Err(error) => error.to_string(),
        };

        assert!(error.contains("maps 16 bits"));
        assert!(error.contains("only 8 bits"));
    }

    #[test]
    fn disabled_mapped_pdo_keeps_direct_codec_but_not_dispatch() {
        let od = test_od(
            vec![
                pdo(
                    PdoDirection::Tpdo,
                    1,
                    Some(CobIdSpec::NodeRelative(0x8000_0180)),
                    Some(16),
                ),
                pdo(PdoDirection::Rpdo, 2, None, None),
                pdo(
                    PdoDirection::Rpdo,
                    3,
                    Some(CobIdSpec::NodeRelative(0x8000_0400)),
                    Some(16),
                ),
            ],
            parse_quote!(u16),
        );

        let (payloads, methods) =
            generate_parts(&od, quote!(self.node_id.raw())).expect("codec generation failed");
        let generated = quote!(#(#payloads)* #(#methods)*).to_string();

        assert!(generated.contains("GeneratedTpdo1"));
        assert!(!generated.contains("GeneratedRpdo2"));
        assert!(generated.contains("GeneratedRpdo3"));
        assert!(generated.contains("tpdo1_cob_id"));
        assert!(generated.contains("decode_tpdo1"));
        assert!(!generated.contains("rpdo2_cob_id"));
        assert!(generated.contains("rpdo3_cob_id"));
        assert!(generated.contains("fn rpdo3"));
        assert!(
            generated.contains("enum GeneratedTpdo { }")
                || generated.contains("enum GeneratedTpdo {}")
        );
    }

    #[test]
    fn duplicate_effective_cob_ids_are_rejected() {
        let od = test_od(
            vec![
                pdo(
                    PdoDirection::Tpdo,
                    1,
                    Some(CobIdSpec::Absolute(0x180)),
                    Some(16),
                ),
                pdo(
                    PdoDirection::Tpdo,
                    2,
                    Some(CobIdSpec::Absolute(0x4000_0180)),
                    Some(16),
                ),
            ],
            parse_quote!(u16),
        );

        let error = match generate_parts(&od, quote!(self.node_id.raw())) {
            Ok(_) => panic!("duplicate COB-IDs unexpectedly generated a codec"),
            Err(error) => error.to_string(),
        };

        assert!(error.contains("resolve to the same COB-ID"));
    }

    #[test]
    fn extended_can_id_is_rejected_explicitly() {
        let od = test_od(
            vec![pdo(
                PdoDirection::Tpdo,
                1,
                Some(CobIdSpec::Absolute(0x2000_0185)),
                Some(16),
            )],
            parse_quote!(u16),
        );

        let error = match generate_parts(&od, quote!(self.node_id.raw())) {
            Ok(_) => panic!("29-bit PDO unexpectedly generated a codec"),
            Err(error) => error.to_string(),
        };

        assert!(error.contains("uses a 29-bit CAN ID"));
        assert!(error.contains("only 11-bit COB-IDs are supported"));
    }

    #[test]
    fn all_node_relative_tpdos_emit_no_runtime_ambiguity_check() {
        let od = test_od(
            vec![
                pdo(PdoDirection::Tpdo, 1, None, Some(16)),
                pdo(PdoDirection::Tpdo, 2, None, Some(16)),
            ],
            parse_quote!(u16),
        );

        let (payloads, methods) =
            generate_parts(&od, quote!(self.node_id.raw())).expect("codec generation failed");
        let generated = quote!(#(#payloads)* #(#methods)*).to_string();

        assert!(!generated.contains("AmbiguousCobId"));
    }
}
