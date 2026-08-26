use std::{env, fs, path::PathBuf};

fn main() {
    // Each image in this workspace has its own linker script, so memory.x has
    // to be published per crate rather than found via the working directory.
    let out = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR is set by Cargo"));
    fs::write(out.join("memory.x"), include_bytes!("memory.x")).expect("write memory.x");
    println!("cargo:rustc-link-search={}", out.display());

    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
