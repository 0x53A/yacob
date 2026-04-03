use syn::parse::{Parse, ParseStream};
use syn::{braced, bracketed, Ident, LitInt, Result, Token, Visibility};

/// Top-level OD definition parsed from the macro input.
#[derive(Debug)]
pub struct OdDefinition {
    pub vis: Visibility,
    pub name: Ident,
    pub entries: Vec<OdEntry>,
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
    pub default_value: syn::Expr,
    pub access: AccessKind,
    pub pdo_mappable: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccessKind {
    Ro,
    Rw,
    Wo,
    Const,
}

impl Parse for OdDefinition {
    fn parse(input: ParseStream) -> Result<Self> {
        let vis: Visibility = input.parse()?;
        input.parse::<Token![struct]>()?;
        let name: Ident = input.parse()?;

        let content;
        braced!(content in input);

        let mut entries = Vec::new();
        while !content.is_empty() {
            entries.push(content.call(parse_entry)?);
        }

        Ok(OdDefinition { vis, name, entries })
    }
}

fn parse_entry(input: ParseStream) -> Result<OdEntry> {
    // Parse [0xINDEX]
    let index_content;
    bracketed!(index_content in input);
    let index_lit: LitInt = index_content.parse()?;
    let index: u16 = index_lit.base10_parse().map_err(|_| {
        syn::Error::new(index_lit.span(), "expected u16 index")
    })?;

    // Parse name
    let name: Ident = input.parse()?;

    // Check if it's a record/array
    if input.peek(Token![:]) && !input.peek2(Token![:]) {
        input.parse::<Token![:]>()?;

        // Check for "record" keyword
        if input.peek(Ident) {
            let type_or_kw: Ident = input.fork().parse()?;
            if type_or_kw == "record" || type_or_kw == "array" {
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
    let subindex: u8 = sub_lit.base10_parse().map_err(|_| {
        syn::Error::new(sub_lit.span(), "expected u8 subindex")
    })?;

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

fn parse_var_def(input: ParseStream) -> Result<VarDef> {
    let type_name: Ident = input.parse()?;
    input.parse::<Token![=]>()?;
    let default_value: syn::Expr = input.parse()?;
    input.parse::<Token![,]>()?;

    // Parse access type
    let access_ident: Ident = input.parse()?;
    let access = match access_ident.to_string().as_str() {
        "ro" => AccessKind::Ro,
        "rw" => AccessKind::Rw,
        "wo" => AccessKind::Wo,
        "const" => {
            return Err(syn::Error::new(
                access_ident.span(),
                "use `ro` for const access (use Rust const for compile-time constants)",
            ));
        }
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

    Ok(VarDef {
        type_name,
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
        _ => None,
    }
}

/// Map DSL type names to byte sizes.
pub fn type_size(ty: &str) -> Option<usize> {
    match ty {
        "bool" | "u8" | "i8" => Some(1),
        "u16" | "i16" => Some(2),
        "u32" | "i32" | "f32" => Some(4),
        "u64" | "i64" => Some(8),
        _ => None,
    }
}
