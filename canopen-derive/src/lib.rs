extern crate proc_macro;

mod codegen;
mod dsl;
mod eds_export;
mod eds_parser;
mod sdo_client_codegen;

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

/// Generate a typed async SDO client from an EDS file.
///
/// Creates a struct with typed `read_*` and `write_*` async methods for each
/// OD entry, using `SdoDriver` under the hood.
///
/// # Example
/// ```ignore
/// sdo_client_from_eds! {
///     pub struct MotorClient = "motor_controller.eds";
/// }
///
/// let client = MotorClient::new(node_id);
/// let status: u16 = client.read_statusword(&mut can).await?;
/// client.write_controlword(0x0F, &mut can).await?;
/// ```
#[proc_macro]
pub fn sdo_client_from_eds(input: TokenStream) -> TokenStream {
    let eds_def = syn::parse_macro_input!(input as eds_parser::EdsDefinition);
    sdo_client_codegen::generate(eds_def).into()
}
