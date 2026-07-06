use syn::parse::{Parse, ParseStream};
use syn::{braced, bracketed, parenthesized, Ident, LitInt, LitStr, Result, Token, Visibility};

/// Top-level OD definition parsed from the macro input.
#[derive(Debug)]
pub struct OdDefinition {
    pub vis: Visibility,
    pub name: Ident,
    pub entries: Vec<OdEntry>,
    pub pdos: Vec<PdoDef>,
    /// If set, export the OD as an EDS file to this path (relative to CARGO_MANIFEST_DIR or absolute).
    pub export_eds_path: Option<String>,
    /// If true, use `alloc::string::String` / `alloc::vec::Vec<u8>` instead of heapless for
    /// variable-length types. Capacity parameters become optional.
    pub use_alloc: bool,
    /// If set, name of a user-defined function to call for `validate_write()`.
    /// The function must have signature: `fn(&Self, u16, u8, &[u8]) -> Result<(), canopen_core::od::OdError>`
    pub validate_write_fn: Option<Ident>,
}

/// A single OD entry (either a VAR or a RECORD/ARRAY with sub-entries).
#[derive(Debug, Clone)]
pub struct OdEntry {
    pub index: u16,
    pub name: Ident,
    pub kind: EntryKind,
}

#[derive(Debug, Clone)]
pub enum EntryKind {
    Var(VarDef),
    Record(Vec<SubEntry>),
    Array(ArrayDef),
}

#[derive(Debug, Clone)]
pub struct ArrayDef {
    pub element_type: Ident,
    /// Capacity for variable-length element types.
    #[allow(dead_code)]
    pub element_capacity: Option<usize>,
    pub count: usize,
    pub access: AccessKind,
    pub pdo_mappable: bool,
}

#[derive(Debug, Clone)]
pub struct SubEntry {
    pub subindex: u8,
    pub name: Ident,
    pub var: VarDef,
}

#[derive(Debug, Clone)]
pub struct VarDef {
    pub type_name: Ident,
    /// Capacity for variable-length types (e.g. visible_string<32>).
    /// Required in no_alloc mode, optional with alloc.
    pub capacity: Option<usize>,
    pub default_value: Option<syn::Expr>,
    pub access: AccessKind,
    pub pdo_mappable: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
pub enum AccessKind {
    Ro,
    Rw,
    Wo,
    Const,
}

/// PDO definition parsed from `tpdo[N](...) { field, ... };` or `rpdo[N](...) { ... };`.
#[derive(Debug, Clone)]
pub struct PdoDef {
    pub direction: PdoDirection,
    /// 1-indexed PDO number (1..=4), matching CANopen naming (TPDO1, RPDO1, etc.)
    pub number: u8,
    /// COB-ID override. None = use predefined default (resolved at Node init with node_id).
    pub cob_id: Option<u32>,
    pub transmission_type: u8,
    /// Inhibit time in 100μs units (TPDO only).
    pub inhibit_time: u16,
    /// Event timer in ms (TPDO only).
    pub event_timer: u16,
    /// Field names referencing previously defined OD entries.
    pub mappings: Vec<Ident>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PdoDirection {
    Tpdo,
    Rpdo,
}

impl Parse for OdDefinition {
    fn parse(input: ParseStream) -> Result<Self> {
        // Parse optional attributes: #[export_eds(path = "...")], #[use_alloc]
        let mut export_eds_path = None;
        let mut use_alloc = false;
        let mut validate_write_fn = None;
        while input.peek(Token![#]) {
            input.parse::<Token![#]>()?;
            let attr_content;
            bracketed!(attr_content in input);
            let attr_name: Ident = attr_content.parse()?;
            if attr_name == "export_eds" {
                if !attr_content.peek(syn::token::Paren) {
                    return Err(syn::Error::new(
                        attr_name.span(),
                        "expected `export_eds(path = \"...\")`, e.g. #[export_eds(path = \"./my_device.eds\")]",
                    ));
                }
                let paren_content;
                parenthesized!(paren_content in attr_content);
                let key: Ident = paren_content.parse()?;
                if key != "path" {
                    return Err(syn::Error::new(
                        key.span(),
                        format!("unknown parameter `{key}`, expected `path`"),
                    ));
                }
                paren_content.parse::<Token![=]>()?;
                let path_lit: LitStr = paren_content.parse()?;
                export_eds_path = Some(path_lit.value());
            } else if attr_name == "use_alloc" {
                use_alloc = true;
            } else if attr_name == "validate_write" {
                // #[validate_write(my_function)]
                if !attr_content.peek(syn::token::Paren) {
                    return Err(syn::Error::new(
                        attr_name.span(),
                        "expected `validate_write(fn_name)`, e.g. #[validate_write(my_validate)]",
                    ));
                }
                let paren_content;
                parenthesized!(paren_content in attr_content);
                let fn_name: Ident = paren_content.parse()?;
                validate_write_fn = Some(fn_name);
            } else {
                return Err(syn::Error::new(
                    attr_name.span(),
                    format!("unknown attribute `{attr_name}`, expected `export_eds`, `use_alloc`, or `validate_write`"),
                ));
            }
        }

        let vis: Visibility = input.parse()?;
        input.parse::<Token![struct]>()?;
        let name: Ident = input.parse()?;

        let content;
        braced!(content in input);

        let mut entries = Vec::new();
        let mut pdos = Vec::new();
        while !content.is_empty() {
            // PDO definitions start with an ident (tpdo/rpdo),
            // regular OD entries start with [0xINDEX]
            if content.peek(Ident) {
                pdos.push(content.call(parse_pdo_def)?);
            } else {
                entries.push(content.call(parse_entry)?);
            }
        }

        Ok(OdDefinition {
            vis,
            name,
            entries,
            pdos,
            export_eds_path,
            use_alloc,
            validate_write_fn,
        })
    }
}

fn parse_entry(input: ParseStream) -> Result<OdEntry> {
    // Parse [0xINDEX]
    let index_content;
    bracketed!(index_content in input);
    let index_lit: LitInt = index_content.parse()?;
    let index: u16 = index_lit
        .base10_parse()
        .map_err(|_| syn::Error::new(index_lit.span(), "expected u16 index"))?;

    // Parse name
    let name: Ident = input.parse()?;

    // Check if it's a record/array
    if input.peek(Token![:]) && !input.peek2(Token![:]) {
        input.parse::<Token![:]>()?;

        // Check for "record" or "array" keyword
        if input.peek(Ident) {
            let type_or_kw: Ident = input.fork().parse()?;
            if type_or_kw == "record" {
                input.parse::<Ident>()?; // consume the keyword

                let sub_content;
                braced!(sub_content in input);

                let mut subs = Vec::new();
                while !sub_content.is_empty() {
                    subs.push(sub_content.call(parse_sub_entry)?);
                }

                input.parse::<Token![;]>()?;

                return Ok(OdEntry {
                    index,
                    name,
                    kind: EntryKind::Record(subs),
                });
            } else if type_or_kw == "array" {
                input.parse::<Ident>()?; // consume "array"

                // Parse <Type, Count>
                input.parse::<Token![<]>()?;
                let element_type: Ident = input.parse()?;
                input.parse::<Token![,]>()?;
                let count_lit: LitInt = input.parse()?;
                let count: usize = count_lit.base10_parse().map_err(|_| {
                    syn::Error::new(count_lit.span(), "expected array count (usize)")
                })?;
                input.parse::<Token![>]>()?;

                input.parse::<Token![,]>()?;

                // Parse access type
                let access_ident: Ident = input.parse()?;
                let access = match access_ident.to_string().as_str() {
                    "ro" => AccessKind::Ro,
                    "rw" => AccessKind::Rw,
                    "wo" => AccessKind::Wo,
                    other => {
                        return Err(syn::Error::new(
                            access_ident.span(),
                            format!("unknown access type `{other}`, expected ro, rw, or wo"),
                        ));
                    }
                };

                // Optional: pdo flag
                let pdo_mappable = if input.peek(Token![,]) {
                    input.parse::<Token![,]>()?;
                    let flag: Ident = input.parse()?;
                    if flag == "pdo" {
                        true
                    } else {
                        return Err(syn::Error::new(flag.span(), "expected `pdo`"));
                    }
                } else {
                    false
                };

                input.parse::<Token![;]>()?;

                return Ok(OdEntry {
                    index,
                    name,
                    kind: EntryKind::Array(ArrayDef {
                        element_type,
                        element_capacity: None,
                        count,
                        access,
                        pdo_mappable,
                    }),
                });
            }
        }

        // It's a VAR: type = default, access[, pdo];
        let var = parse_var_def(input)?;
        input.parse::<Token![;]>()?;

        Ok(OdEntry {
            index,
            name,
            kind: EntryKind::Var(var),
        })
    } else {
        return Err(input.error("expected `:` after entry name"));
    }
}

fn parse_sub_entry(input: ParseStream) -> Result<SubEntry> {
    // Parse [SUBINDEX]
    let sub_content;
    bracketed!(sub_content in input);
    let sub_lit: LitInt = sub_content.parse()?;
    let subindex: u8 = sub_lit
        .base10_parse()
        .map_err(|_| syn::Error::new(sub_lit.span(), "expected u8 subindex"))?;

    let name: Ident = input.parse()?;
    input.parse::<Token![:]>()?;
    let var = parse_var_def(input)?;
    input.parse::<Token![;]>()?;

    Ok(SubEntry {
        subindex,
        name,
        var,
    })
}

/// Parse `tpdo[1](transmission_type = 255, inhibit_time = 500) { button, echo_out, };`
fn parse_pdo_def(input: ParseStream) -> Result<PdoDef> {
    let dir_ident: Ident = input.parse()?;
    let direction = match dir_ident.to_string().as_str() {
        "tpdo" => PdoDirection::Tpdo,
        "rpdo" => PdoDirection::Rpdo,
        other => {
            return Err(syn::Error::new(
                dir_ident.span(),
                format!("expected `tpdo` or `rpdo`, got `{other}`"),
            ));
        }
    };

    // Parse [N] — 1-indexed PDO number
    let num_content;
    bracketed!(num_content in input);
    let num_lit: LitInt = num_content.parse()?;
    let number: u8 = num_lit
        .base10_parse()
        .map_err(|_| syn::Error::new(num_lit.span(), "expected PDO number 1-4"))?;
    if number < 1 || number > 4 {
        return Err(syn::Error::new(num_lit.span(), "PDO number must be 1-4"));
    }

    // Parse optional (key = value, ...) parameters
    let mut cob_id = None;
    let mut transmission_type: u8 = 255;
    let mut inhibit_time: u16 = 0;
    let mut event_timer: u16 = 0;

    if input.peek(syn::token::Paren) {
        let params;
        parenthesized!(params in input);
        while !params.is_empty() {
            let key: Ident = params.parse()?;
            params.parse::<Token![=]>()?;
            match key.to_string().as_str() {
                "cob_id" => {
                    let val: LitInt = params.parse()?;
                    cob_id = Some(val.base10_parse()?);
                }
                "transmission_type" => transmission_type = parse_transmission_type(&params)?,
                "inhibit_time" => inhibit_time = parse_time_value(&params, &INHIBIT_TIME_SPEC)?,
                "event_timer" => event_timer = parse_time_value(&params, &EVENT_TIMER_SPEC)?,
                other => {
                    return Err(syn::Error::new(
                        key.span(),
                        format!("unknown PDO parameter `{other}`, expected cob_id, transmission_type, inhibit_time, or event_timer"),
                    ));
                }
            }
            if params.peek(Token![,]) {
                params.parse::<Token![,]>()?;
            }
        }
    }

    // Parse { field, field, ... }
    let mapping_content;
    braced!(mapping_content in input);
    let mut mappings = Vec::new();
    while !mapping_content.is_empty() {
        let field: Ident = mapping_content.parse()?;
        mappings.push(field);
        if mapping_content.peek(Token![,]) {
            mapping_content.parse::<Token![,]>()?;
        }
    }
    if mappings.len() > 8 {
        return Err(syn::Error::new(
            mappings[8].span(),
            "a PDO can map at most 8 objects (CiA 301)",
        ));
    }

    input.parse::<Token![;]>()?;

    Ok(PdoDef {
        direction,
        number,
        cob_id,
        transmission_type,
        inhibit_time,
        event_timer,
        mappings,
    })
}

/// Parse a transmission type: either a raw CiA 301 value (`255`) or a keyword
/// (`event_driven`, `event_driven_manufacturer`, `sync_acyclic`,
/// `sync_cyclic(N)`, `rtr_sync`, `rtr_event`).
fn parse_transmission_type(input: ParseStream) -> Result<u8> {
    if input.peek(LitInt) {
        let lit: LitInt = input.parse()?;
        let raw: u8 = lit.base10_parse()?;
        if (241..=251).contains(&raw) {
            return Err(syn::Error::new(
                lit.span(),
                format!("transmission type {raw} is reserved (CiA 301); valid values are 0 (sync acyclic), 1-240 (sync cyclic), 252/253 (RTR), 254/255 (event-driven)"),
            ));
        }
        return Ok(raw);
    }

    let kw: Ident = input.parse().map_err(|_| {
        input.error("expected a transmission type: raw value (e.g. 255) or keyword (event_driven, sync_acyclic, sync_cyclic(N), rtr_sync, rtr_event)")
    })?;
    match kw.to_string().as_str() {
        "event_driven" => Ok(255),
        "event_driven_manufacturer" => Ok(254),
        "rtr_event" => Ok(253),
        "rtr_sync" => Ok(252),
        "sync_acyclic" => Ok(0),
        "sync_cyclic" => {
            let n_content;
            parenthesized!(n_content in input);
            let n_lit: LitInt = n_content.parse()?;
            let n: u8 = n_lit.base10_parse()?;
            if !(1..=240).contains(&n) {
                return Err(syn::Error::new(
                    n_lit.span(),
                    "sync_cyclic(N): N must be 1..=240 (send every N SYNCs)",
                ));
            }
            Ok(n)
        }
        other => Err(syn::Error::new(
            kw.span(),
            format!("unknown transmission type `{other}`, expected event_driven, event_driven_manufacturer, sync_acyclic, sync_cyclic(N), rtr_sync, rtr_event, or a raw CiA 301 value"),
        )),
    }
}

/// How a DSL time parameter maps to its CiA 301 wire representation.
struct TimeSpec {
    /// Parameter name for error messages.
    param: &'static str,
    /// Size of one wire unit in nanoseconds.
    unit_ns: u64,
    /// Human-readable wire unit for error messages.
    unit_name: &'static str,
}

static INHIBIT_TIME_SPEC: TimeSpec = TimeSpec {
    param: "inhibit_time",
    unit_ns: 100_000, // 100 µs
    unit_name: "100µs",
};

static EVENT_TIMER_SPEC: TimeSpec = TimeSpec {
    param: "event_timer",
    unit_ns: 1_000_000, // 1 ms
    unit_name: "1ms",
};

/// Parse a time parameter value. Accepts either a raw integer in the CiA 301
/// wire unit (spec-exact, quirks included) or a unit-suffixed literal:
/// `500` (raw), `50ms`, `100us`, `0.1s`.
fn parse_time_value(input: ParseStream, spec: &TimeSpec) -> Result<u16> {
    let lit: syn::Lit = input.parse()?;

    let (value_ns, span) = match &lit {
        syn::Lit::Int(li) => {
            match li.suffix() {
                // Raw value: spec wire units, passed through unchanged.
                "" => return li.base10_parse::<u16>(),
                suffix => {
                    let v: u64 = li.base10_parse()?;
                    let ns = v
                        .checked_mul(suffix_to_ns(suffix, li.span(), spec)?)
                        .ok_or_else(|| syn::Error::new(li.span(), "time value overflows"))?;
                    (ns as f64, li.span())
                }
            }
        }
        syn::Lit::Float(lf) => {
            let suffix = lf.suffix();
            if suffix.is_empty() {
                return Err(syn::Error::new(
                    lf.span(),
                    format!(
                        "float {} values need a unit suffix (us, ms, s), e.g. `{} = 0.1s`; \
                         raw {} units must be integers",
                        spec.param, spec.param, spec.unit_name
                    ),
                ));
            }
            let v: f64 = lf.base10_parse()?;
            (v * suffix_to_ns(suffix, lf.span(), spec)? as f64, lf.span())
        }
        other => {
            return Err(syn::Error::new_spanned(
                other,
                format!(
                    "expected a time value: raw integer in {} units, or a suffixed literal like 50ms or 0.1s",
                    spec.unit_name
                ),
            ));
        }
    };

    let units = value_ns / spec.unit_ns as f64;
    let rounded = units.round();
    if (units - rounded).abs() > 1e-6 {
        return Err(syn::Error::new(
            span,
            format!(
                "{} value is not representable: CiA 301 stores {} in {} units",
                spec.param, spec.param, spec.unit_name
            ),
        ));
    }
    if rounded < 0.0 || rounded > u16::MAX as f64 {
        let max_s = u16::MAX as f64 * spec.unit_ns as f64 / 1e9;
        return Err(syn::Error::new(
            span,
            format!(
                "{} value out of range: maximum is {}s (u16 in {} units)",
                spec.param, max_s, spec.unit_name
            ),
        ));
    }
    Ok(rounded as u16)
}

fn suffix_to_ns(suffix: &str, span: proc_macro2::Span, spec: &TimeSpec) -> Result<u64> {
    match suffix {
        "us" => Ok(1_000),
        "ms" => Ok(1_000_000),
        "s" => Ok(1_000_000_000),
        other => Err(syn::Error::new(
            span,
            format!(
                "unknown time unit `{other}` for {}, expected us, ms, or s \
                 (or an unsuffixed integer in raw {} units)",
                spec.param, spec.unit_name
            ),
        )),
    }
}

fn parse_var_def(input: ParseStream) -> Result<VarDef> {
    let type_name: Ident = input.parse()?;

    // Optional capacity: <N> for parameterized types (visible_string<32>, domain<1024>, etc.)
    let capacity = if input.peek(Token![<]) {
        input.parse::<Token![<]>()?;
        let cap_lit: LitInt = input.parse()?;
        let cap: usize = cap_lit
            .base10_parse()
            .map_err(|_| syn::Error::new(cap_lit.span(), "expected capacity (usize)"))?;
        input.parse::<Token![>]>()?;
        Some(cap)
    } else {
        None
    };

    // Optional default value: = expr
    let default_value = if input.peek(Token![=]) {
        input.parse::<Token![=]>()?;
        let expr: syn::Expr = input.parse()?;
        input.parse::<Token![,]>()?;
        Some(expr)
    } else {
        input.parse::<Token![,]>()?;
        None
    };

    // Parse access type (`const` is a Rust keyword, so accept keywords too)
    let access_ident: Ident = input.call(syn::ext::IdentExt::parse_any)?;
    let access = match access_ident.to_string().as_str() {
        "ro" => AccessKind::Ro,
        "rw" => AccessKind::Rw,
        "wo" => AccessKind::Wo,
        "const" => AccessKind::Const,
        other => {
            return Err(syn::Error::new(
                access_ident.span(),
                format!("unknown access type `{other}`, expected ro, rw, wo, or const"),
            ));
        }
    };

    // Optional: pdo flag
    let pdo_mappable = if input.peek(Token![,]) {
        input.parse::<Token![,]>()?;
        let flag: Ident = input.parse()?;
        if flag == "pdo" {
            true
        } else {
            return Err(syn::Error::new(flag.span(), "expected `pdo`"));
        }
    } else {
        false
    };

    Ok(VarDef {
        type_name,
        capacity,
        default_value,
        access,
        pdo_mappable,
    })
}

/// Map DSL type names to canopen DataType variants.
pub fn type_to_datatype(ty: &str) -> Option<&'static str> {
    match ty {
        "bool" => Some("Boolean"),
        "u8" => Some("U8"),
        "u16" => Some("U16"),
        "u32" => Some("U32"),
        "u64" => Some("U64"),
        "i8" => Some("I8"),
        "i16" => Some("I16"),
        "i32" => Some("I32"),
        "i64" => Some("I64"),
        "f32" => Some("Real32"),
        "f64" => Some("Real64"),
        "visible_string" => Some("VisibleString"),
        "octet_string" => Some("OctetString"),
        "domain" => Some("Domain"),
        _ => None,
    }
}

/// Map DSL type names to byte sizes. Returns None for variable-length types.
pub fn type_size(ty: &str) -> Option<usize> {
    match ty {
        "bool" | "u8" | "i8" => Some(1),
        "u16" | "i16" => Some(2),
        "u32" | "i32" | "f32" => Some(4),
        "u64" | "i64" | "f64" => Some(8),
        _ => None,
    }
}

/// Returns true if the type is variable-length (needs capacity parameter).
pub fn is_variable_length_type(ty: &str) -> bool {
    matches!(ty, "visible_string" | "octet_string" | "domain")
}
