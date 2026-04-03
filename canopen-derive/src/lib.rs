extern crate proc_macro;

mod codegen;
mod dsl;
mod eds_parser;

use proc_macro::TokenStream;

/// Define a CANopen object dictionary using an inline DSL.
///
/// # Example
/// ```ignore
/// object_dictionary! {
///     pub struct MyOd {
///         [0x1000] device_type: u32 = 0x0000_0191, ro;
///         [0x1001] error_register: u8 = 0x00, ro;
///         [0x1018] identity: record {
///             [1] vendor_id: u32 = 0x0000_1234, ro;
///             [2] product_code: u32 = 0x0001, ro;
///             [3] revision: u32 = 0x0001_0000, ro;
///             [4] serial: u32 = 0x0000_0001, ro;
///         };
///         [0x6000] inputs: record {
///             [1] input1: u8 = 0, ro, pdo;
///             [2] input2: u8 = 0, ro, pdo;
///         };
///         [0x6200] outputs: record {
///             [1] output1: u8 = 0, rw, pdo;
///         };
///     }
/// }
/// ```
#[proc_macro]
pub fn object_dictionary(input: TokenStream) -> TokenStream {
    let od_def = syn::parse_macro_input!(input as dsl::OdDefinition);
    codegen::generate(od_def).into()
}

/// Define a CANopen object dictionary from an EDS file.
///
/// # Example
/// ```ignore
/// object_dictionary_from_eds! {
///     pub struct MotorControllerOd = "motor_controller.eds";
/// }
/// ```
#[proc_macro]
pub fn object_dictionary_from_eds(input: TokenStream) -> TokenStream {
    let eds_def = syn::parse_macro_input!(input as eds_parser::EdsDefinition);
    match eds_parser::parse_eds_to_od(eds_def) {
        Ok(od_def) => codegen::generate(od_def).into(),
        Err(e) => e.to_compile_error().into(),
    }
}
