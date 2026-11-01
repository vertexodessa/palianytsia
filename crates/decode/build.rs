// Links the system libavcodec/libavutil/libswscale found by pkg-config and
// compiles the small C shim that hides their struct layouts from Rust.
use std::process::Command;

fn pkg(args: &[&str]) -> Vec<String> {
    let out = Command::new("pkg-config")
        .args(args)
        .args(["libavcodec", "libavutil", "libswscale"])
        .output()
        .expect("pkg-config");
    assert!(
        out.status.success(),
        "pkg-config: libavcodec-dev, libavutil-dev, libswscale-dev"
    );
    String::from_utf8_lossy(&out.stdout)
        .split_whitespace()
        .map(str::to_string)
        .collect()
}

fn main() {
    println!("cargo:rerun-if-changed=cpp/dec.c");
    // Android decodes with MediaCodec (src/mediacodec.rs): no libavcodec.
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("android") {
        println!("cargo:rustc-link-lib=dylib=mediandk");
        println!("cargo:rustc-link-lib=dylib=android");
        return;
    }
    let mut b = cc::Build::new();
    b.file("cpp/dec.c").warnings(true).extra_warnings(true);
    for f in pkg(&["--cflags-only-I"]) {
        b.include(f.trim_start_matches("-I"));
    }
    b.compile("paldec");
    for f in pkg(&["--libs"]) {
        if let Some(l) = f.strip_prefix("-l") {
            println!("cargo:rustc-link-lib={l}");
        } else if let Some(p) = f.strip_prefix("-L") {
            println!("cargo:rustc-link-search=native={p}");
        }
    }
}
