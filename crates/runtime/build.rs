//! The drone's maburd an app carries (`drone_update::embedded`): the file
//! `PAL_MABURD` names at build time, copied into OUT_DIR; without it the
//! copy is empty and the build updates no drone.
fn main() {
    println!("cargo:rerun-if-env-changed=PAL_MABURD");
    let out = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap()).join("maburd");
    let bytes = match std::env::var("PAL_MABURD") {
        Ok(p) if !p.is_empty() => {
            println!("cargo:rerun-if-changed={p}");
            std::fs::read(&p).unwrap_or_else(|e| panic!("PAL_MABURD={p}: {e}"))
        }
        _ => Vec::new(),
    };
    std::fs::write(&out, bytes).unwrap();
}
