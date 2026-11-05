use crate::dsl::*;
use crate::eds_parser;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};

pub fn generate(def: eds_parser::EdsDefinition) -> TokenStream {
    let od = match eds_parser::parse_eds_to_od(def) {
        Ok(od) => od,
        Err(e) => return e.to_compile_error(),
    };

    generate_from_od(&od)
}

pub(crate) fn generate_from_od(od: &OdDefinition) -> TokenStream {
    let vis = &od.vis;
    let name = &od.name;

    // Every generated method includes the OD address so names remain stable,
    // callers can cross-reference datasheets directly, and duplicate EDS
    // ParameterNames cannot collide.
    let base_name = |entry: &OdEntry, sub_name: Option<&str>| -> String {
        match sub_name {
            // EDS record sub-entry names are already prefixed with the record
            // name by the parser to avoid object-dictionary field collisions.
            Some(s) => s.to_string(),
            None => entry.name.to_string(),
        }
    };

    let mut methods: Vec<TokenStream> = Vec::new();
    for entry in &od.entries {
        match &entry.kind {
            EntryKind::Var(var) => {
                let ty_str = var.type_name.to_string();
                let base = base_name(entry, None);
                let name = format!("i{:04x}_s00_{base}", entry.index);
                gen_var_methods(&mut methods, entry.index, 0, &name, &ty_str, var.access);
            }
            EntryKind::Record(subs) => {
                for sub in subs {
                    let ty_str = sub.var.type_name.to_string();
                    let base = base_name(entry, Some(&sub.name.to_string()));
                    let method_name = format!("i{:04x}_s{:02x}_{base}", entry.index, sub.subindex);
                    gen_var_methods(
                        &mut methods,
                        entry.index,
                        sub.subindex,
                        &method_name,
                        &ty_str,
                        sub.var.access,
                    );
                }
            }
            EntryKind::Array(arr) => {
                let ty_str = arr.element_type.to_string();
                let base = base_name(entry, None);
                let name = format!("i{:04x}_{base}", entry.index);
                gen_array_methods(
                    &mut methods,
                    entry.index,
                    &name,
                    &ty_str,
                    arr.access,
                    arr.count,
                );
            }
        }
    }

    quote! {
        #vis struct #name {
            driver: canopen_core::sdo::SdoDriver,
        }

        impl #name {
            pub fn new(target: canopen_core::cobid::NodeId) -> Self {
                Self {
                    driver: canopen_core::sdo::SdoDriver::new(target),
                }
            }

            pub fn target(&self) -> canopen_core::cobid::NodeId {
                self.driver.target()
            }

            /// Raw upload (read) by index/subindex.
            pub async fn upload<E: core::fmt::Debug>(
                &self,
                index: u16,
                subindex: u8,
                buf: &mut [u8],
                can: &mut impl canopen_core::sdo::AsyncCan<Error = E>,
            ) -> Result<usize, canopen_core::sdo::SdoError<E>> {
                self.driver.upload(index, subindex, buf, can).await
            }

            /// Raw download (write) by index/subindex.
            pub async fn download<E: core::fmt::Debug>(
                &self,
                index: u16,
                subindex: u8,
                data: &[u8],
                can: &mut impl canopen_core::sdo::AsyncCan<Error = E>,
            ) -> Result<(), canopen_core::sdo::SdoError<E>> {
                self.driver.download(index, subindex, data, can).await
            }

            #(#methods)*
        }
    }
}

fn gen_var_methods(
    methods: &mut Vec<TokenStream>,
    index: u16,
    subindex: u8,
    name: &str,
    ty_str: &str,
    access: AccessKind,
) {
    let is_varlen = is_variable_length_type(ty_str);
    let doc = format!("OD `0x{index:04X}:{subindex:02X}`");

    // Read method
    if !matches!(access, AccessKind::Wo) {
        if is_varlen {
            let read_name = format_ident!("read_{}", name);
            methods.push(quote! {
                #[doc = #doc]
                pub async fn #read_name<E: core::fmt::Debug>(
                    &self,
                    buf: &mut [u8],
                    can: &mut impl canopen_core::sdo::AsyncCan<Error = E>,
                ) -> Result<usize, canopen_core::sdo::SdoError<E>> {
                    self.driver.upload(#index, #subindex, buf, can).await
                }
            });
        } else if let Some((driver_fn, ret_ty)) = typed_read_info(ty_str) {
            let read_name = format_ident!("read_{}", name);
            let driver_fn = format_ident!("{}", driver_fn);
            methods.push(quote! {
                #[doc = #doc]
                pub async fn #read_name<E: core::fmt::Debug>(
                    &self,
                    can: &mut impl canopen_core::sdo::AsyncCan<Error = E>,
                ) -> Result<#ret_ty, canopen_core::sdo::SdoError<E>> {
                    self.driver.#driver_fn(#index, #subindex, can).await
                }
            });
        }
    }

    // Write method
    if matches!(access, AccessKind::Rw | AccessKind::Wo) {
        if is_varlen {
            let write_name = format_ident!("write_{}", name);
            methods.push(quote! {
                #[doc = #doc]
                pub async fn #write_name<E: core::fmt::Debug>(
                    &self,
                    data: &[u8],
                    can: &mut impl canopen_core::sdo::AsyncCan<Error = E>,
                ) -> Result<(), canopen_core::sdo::SdoError<E>> {
                    self.driver.download(#index, #subindex, data, can).await
                }
            });
        } else if let Some((driver_fn, param_ty)) = typed_write_info(ty_str) {
            let write_name = format_ident!("write_{}", name);
            let driver_fn = format_ident!("{}", driver_fn);
            methods.push(quote! {
                #[doc = #doc]
                pub async fn #write_name<E: core::fmt::Debug>(
                    &self,
                    val: #param_ty,
                    can: &mut impl canopen_core::sdo::AsyncCan<Error = E>,
                ) -> Result<(), canopen_core::sdo::SdoError<E>> {
                    self.driver.#driver_fn(#index, #subindex, val, can).await
                }
            });
        }
    }
}

fn gen_array_methods(
    methods: &mut Vec<TokenStream>,
    index: u16,
    name: &str,
    element_ty_str: &str,
    access: AccessKind,
    count: usize,
) {
    let count_u8 = count as u8;
    let doc = format!("OD `0x{index:04X}` (array, sub 1..={count})");

    // Read element
    if !matches!(access, AccessKind::Wo) {
        if let Some((driver_fn, ret_ty)) = typed_read_info(element_ty_str) {
            let read_name = format_ident!("read_{}", name);
            let driver_fn = format_ident!("{}", driver_fn);
            methods.push(quote! {
                #[doc = #doc]
                pub async fn #read_name<E: core::fmt::Debug>(
                    &self,
                    sub: u8,
                    can: &mut impl canopen_core::sdo::AsyncCan<Error = E>,
                ) -> Result<#ret_ty, canopen_core::sdo::SdoError<E>> {
                    assert!((1..=#count_u8).contains(&sub), "array subindex out of range");
                    self.driver.#driver_fn(#index, sub, can).await
                }
            });
        }
    }

    // Write element
    if matches!(access, AccessKind::Rw | AccessKind::Wo) {
        if let Some((driver_fn, param_ty)) = typed_write_info(element_ty_str) {
            let write_name = format_ident!("write_{}", name);
            let driver_fn = format_ident!("{}", driver_fn);
            methods.push(quote! {
                #[doc = #doc]
                pub async fn #write_name<E: core::fmt::Debug>(
                    &self,
                    sub: u8,
                    val: #param_ty,
                    can: &mut impl canopen_core::sdo::AsyncCan<Error = E>,
                ) -> Result<(), canopen_core::sdo::SdoError<E>> {
                    assert!((1..=#count_u8).contains(&sub), "array subindex out of range");
                    self.driver.#driver_fn(#index, sub, val, can).await
                }
            });
        }
    }
}

/// Rust type and `SdoDriver` method suffix for each fixed-width CANopen type.
fn typed_info(ty: &str) -> Option<(&'static str, TokenStream)> {
    Some(match ty {
        "bool" => ("bool", quote! { bool }),
        "u8" => ("u8", quote! { u8 }),
        "i8" => ("i8", quote! { i8 }),
        "u16" => ("u16", quote! { u16 }),
        "i16" => ("i16", quote! { i16 }),
        "u24" => ("u24", quote! { u32 }),
        "i24" => ("i24", quote! { i32 }),
        "u32" => ("u32", quote! { u32 }),
        "i32" => ("i32", quote! { i32 }),
        "f32" => ("f32", quote! { f32 }),
        "u64" => ("u64", quote! { u64 }),
        "i64" => ("i64", quote! { i64 }),
        "f64" => ("f64", quote! { f64 }),
        _ => return None,
    })
}

/// Returns (driver_method_name, return_type_tokens) for a typed read.
fn typed_read_info(ty: &str) -> Option<(String, TokenStream)> {
    typed_info(ty).map(|(suffix, ty)| (format!("read_{suffix}"), ty))
}

/// Returns (driver_method_name, parameter_type_tokens) for a typed write.
fn typed_write_info(ty: &str) -> Option<(String, TokenStream)> {
    typed_info(ty).map(|(suffix, ty)| (format!("write_{suffix}"), ty))
}
