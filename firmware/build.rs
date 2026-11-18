use std::{env, fs, path::PathBuf};

/// Each binary's memory layout, which cortex-m-rt's `link.x` includes as
/// `memory.x` from the linker search path.
const LAYOUTS: [(&str, &str); 2] = [("firmware", "memory.x"), ("unpacker", "unpacker.x")];
/// The unpacker's layout with the `usb-install` feature.
const USB_UNPACKER: &str = "unpacker-usb.x";

fn main() -> std::io::Result<()> {
    // Cargo supplies OUT_DIR for build scripts.
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("cargo must provide OUT_DIR"));
    let usb = env::var_os("CARGO_FEATURE_USB_INSTALL").is_some();
    for (bin, layout) in LAYOUTS {
        let layout = if usb && bin == "unpacker" {
            USB_UNPACKER
        } else {
            layout
        };
        let dir = out.join(bin);
        fs::create_dir_all(&dir)?;
        fs::copy(layout, dir.join("memory.x"))?;
        println!("cargo:rustc-link-arg-bin={bin}=-L{}", dir.display());
        println!("cargo:rerun-if-changed={layout}");
    }
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rerun-if-changed=build.rs");
    Ok(())
}
