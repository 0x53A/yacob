use crate::dsl::*;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};

/// Flatten an OdDefinition into a list of (index, subindex, name, VarDef).
struct FlatEntry {
    index: u16,
    subindex: u8,
    field_name: syn::Ident,
    var: VarDef,
}

fn flatten(entries: &[OdEntry]) -> Vec<FlatEntry> {
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
        }
    }
    flat
}

pub fn generate(od: OdDefinition) -> TokenStream {
    let vis = &od.vis;
    let name = &od.name;
    let flat = flatten(&od.entries);

    // Generate struct fields
    let struct_fields: Vec<TokenStream> = flat
        .iter()
        .map(|e| {
            let fname = &e.field_name;
            let ty = &e.var.type_name;
            quote! { pub #fname: #ty }
        })
        .collect();

    // Generate default values for new()
    let field_defaults: Vec<TokenStream> = flat
        .iter()
        .map(|e| {
            let fname = &e.field_name;
            let default = &e.var.default_value;
            quote! { #fname: #default }
        })
        .collect();

    // Generate static metadata table
    let meta_name = format_ident!("{}_META", to_screaming_snake(&name.to_string()));
    let meta_entries: Vec<TokenStream> = flat
        .iter()
        .map(|e| {
            let index = e.index;
            let subindex = e.subindex;
            let dt = type_to_datatype(&e.var.type_name.to_string())
                .expect("unsupported type");
            let dt_ident = format_ident!("{}", dt);
            let access_ident = match e.var.access {
                AccessKind::Ro => format_ident!("Ro"),
                AccessKind::Rw => format_ident!("Rw"),
                AccessKind::Wo => format_ident!("Wo"),
                AccessKind::Const => format_ident!("Const"),
            };
            let pdo = e.var.pdo_mappable;
            let entry_name = e.field_name.to_string();
            quote! {
                canopen_core::od::OdEntryMeta {
                    index: #index,
                    subindex: #subindex,
                    data_type: canopen_core::datatypes::DataType::#dt_ident,
                    access: canopen_core::od::AccessType::#access_ident,
                    pdo_mappable: #pdo,
                    name: #entry_name,
                }
            }
        })
        .collect();

    // Generate read match arms
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

            let size = type_size(&ty_str).expect("unsupported type");
            if size == 1 {
                // For u8/i8/bool
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

    // Generate write match arms
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

            let size = type_size(&ty_str).expect("unsupported type");
            if ty_str == "bool" {
                quote! {
                    (#index, #subindex) => {
                        if data.len() < 1 { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                        self.#fname = data[0] != 0;
                        Ok(())
                    }
                }
            } else if size == 1 {
                quote! {
                    (#index, #subindex) => {
                        if data.len() < 1 { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                        self.#fname = data[0] as #ty;
                        Ok(())
                    }
                }
            } else {
                quote! {
                    (#index, #subindex) => {
                        if data.len() < #size { return Err(canopen_core::od::OdError::DataTypeMismatch); }
                        let mut arr = [0u8; #size];
                        arr.copy_from_slice(&data[..#size]);
                        self.#fname = #ty::from_le_bytes(arr);
                        Ok(())
                    }
                }
            }
        })
        .collect();

    // Generate sub_count: for each index that has sub-entries, return the max subindex
    let mut sub_counts: std::collections::HashMap<u16, u8> = std::collections::HashMap::new();
    for e in &flat {
        let counter = sub_counts.entry(e.index).or_insert(0);
        if e.subindex > *counter {
            *counter = e.subindex;
        }
    }
    let sub_count_arms: Vec<TokenStream> = sub_counts
        .iter()
        .map(|(index, max_sub)| {
            quote! { #index => Some(#max_sub), }
        })
        .collect();

    // Generate the output
    let meta_len = flat.len();

    quote! {
        #vis struct #name {
            #(#struct_fields,)*
        }

        static #meta_name: [canopen_core::od::OdEntryMeta; #meta_len] = [
            #(#meta_entries,)*
        ];

        impl #name {
            pub const fn new() -> Self {
                Self {
                    #(#field_defaults,)*
                }
            }
        }

        impl canopen_core::od::ObjectDictionary for #name {
            fn lookup(&self, index: u16, subindex: u8) -> Option<&'static canopen_core::od::OdEntryMeta> {
                #meta_name.iter().find(|e| e.index == index && e.subindex == subindex)
            }

            fn read(&self, index: u16, subindex: u8, buf: &mut [u8]) -> Result<usize, canopen_core::od::OdError> {
                match (index, subindex) {
                    #(#read_arms)*
                    _ => Err(canopen_core::od::OdError::NotFound),
                }
            }

            fn write(&mut self, index: u16, subindex: u8, data: &[u8]) -> Result<(), canopen_core::od::OdError> {
                match (index, subindex) {
                    #(#write_arms)*
                    _ => Err(canopen_core::od::OdError::NotFound),
                }
            }

            fn sub_count(&self, index: u16) -> Option<u8> {
                match index {
                    #(#sub_count_arms)*
                    _ => None,
                }
            }
        }
    }
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
