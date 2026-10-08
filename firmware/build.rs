use std::{env, fs, path::PathBuf};

fn main() -> std::io::Result<()> {
    // Cargo supplies OUT_DIR for build scripts. cortex-m-rt finds memory.x
    // through the linker search path, leaving the application layout local.
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("cargo must provide OUT_DIR"));
    fs::copy("memory.x", out.join("memory.x"))?;
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg-bin=firmware=-Tlink.x");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    Ok(())
}
