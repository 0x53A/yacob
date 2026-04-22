use crate::dsl::*;
use crate::eds_parser;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};

pub fn generate(def: eds_parser::EdsDefinition) -> TokenStream {
    let od = match eds_parser::parse_eds_to_od(def) {
        Ok(od) => od,
        Err(e) => return e.to_compile_error(),
    };

    let vis = &od.vis;
    let name = &od.name;

    // Generate typed read/write methods for each OD entry
    let mut methods: Vec<TokenStream> = Vec::new();

    for entry in &od.entries {
        match &entry.kind {
            EntryKind::Var(var) => {
                let ty_str = var.type_name.to_string();
                gen_var_methods(
                    &mut methods,
                    entry.index,
                    0,
                    &entry.name.to_string(),
                    &ty_str,
                    var.access,
                );
            }
            EntryKind::Record(subs) => {
                let record_name = entry.name.to_string();
                for sub in subs {
                    let ty_str = sub.var.type_name.to_string();
                    let method_name = format!("{}_{}", record_name, sub.name);
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
                gen_array_methods(
                    &mut methods,
                    entry.index,
                    &entry.name.to_string(),
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

    // Read method
    if !matches!(access, AccessKind::Wo) {
        if is_varlen {
            let read_name = format_ident!("read_{}", name);
            methods.push(quote! {
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

    // Read element
    if !matches!(access, AccessKind::Wo) {
        if let Some((driver_fn, ret_ty)) = typed_read_info(element_ty_str) {
            let read_name = format_ident!("read_{}", name);
            let driver_fn = format_ident!("{}", driver_fn);
            methods.push(quote! {
                pub async fn #read_name<E: core::fmt::Debug>(
                    &self,
                    sub: u8,
                    can: &mut impl canopen_core::sdo::AsyncCan<Error = E>,
                ) -> Result<#ret_ty, canopen_core::sdo::SdoError<E>> {
                    assert!(sub >= 1 && sub <= #count_u8, "array subindex out of range");
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
                pub async fn #write_name<E: core::fmt::Debug>(
                    &self,
                    sub: u8,
                    val: #param_ty,
                    can: &mut impl canopen_core::sdo::AsyncCan<Error = E>,
                ) -> Result<(), canopen_core::sdo::SdoError<E>> {
                    assert!(sub >= 1 && sub <= #count_u8, "array subindex out of range");
                    self.driver.#driver_fn(#index, sub, val, can).await
                }
            });
        }
    }
}

/// Returns (driver_method_name, return_type_tokens) for a typed read.
fn typed_read_info(ty: &str) -> Option<(&'static str, TokenStream)> {
    match ty {
        "bool" | "u8" => Some(("read_u8", quote! { u8 })),
        "u16" => Some(("read_u16", quote! { u16 })),
        "u32" => Some(("read_u32", quote! { u32 })),
        "i32" => Some(("read_i32", quote! { i32 })),
        "f32" => Some(("read_f32", quote! { f32 })),
        // i8/i16 use unsigned reads — caller casts if needed
        "i8" => Some(("read_u8", quote! { u8 })),
        "i16" => Some(("read_u16", quote! { u16 })),
        _ => None,
    }
}

/// Returns (driver_method_name, parameter_type_tokens) for a typed write.
fn typed_write_info(ty: &str) -> Option<(&'static str, TokenStream)> {
    match ty {
        "bool" | "u8" => Some(("write_u8", quote! { u8 })),
        "u16" => Some(("write_u16", quote! { u16 })),
        "u32" => Some(("write_u32", quote! { u32 })),
        "f32" => Some(("write_f32", quote! { f32 })),
        "i8" => Some(("write_u8", quote! { u8 })),
        "i16" => Some(("write_u16", quote! { u16 })),
        "i32" => Some(("write_u32", quote! { u32 })),
        _ => None,
    }
}
