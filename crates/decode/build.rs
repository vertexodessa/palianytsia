// Links libavcodec/libavutil/libswscale found by pkg-config and compiles
// the small C shim that hides their struct layouts from Rust. The system
// FFmpeg by default; PAL_FFMPEG_STATIC=<prefix> (tools/ffmpeg-static.sh)
// links that minimal build statically, for a desktop binary that runs on a
// machine with another FFmpeg or none.
use std::process::Command;

fn static_prefix() -> Option<String> {
    std::env::var("PAL_FFMPEG_STATIC")
        .ok()
        .filter(|s| !s.is_empty())
}

fn pkg(args: &[&str]) -> Vec<String> {
    let mut cmd = Command::new("pkg-config");
    if let Some(p) = static_prefix() {
        cmd.env("PKG_CONFIG_PATH", format!("{p}/lib/pkgconfig"));
        cmd.env("PKG_CONFIG_LIBDIR", format!("{p}/lib/pkgconfig"));
        cmd.arg("--static");
    }
    let out = cmd
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
    println!("cargo:rerun-if-env-changed=PAL_FFMPEG_STATIC");
    if let Some(p) = static_prefix() {
        // A rebuilt FFmpeg under the same prefix must relink.
        for l in ["avcodec", "avutil", "swscale"] {
            println!("cargo:rerun-if-changed={p}/lib/lib{l}.a");
        }
    }
    // Android decodes with MediaCodec (src/mediacodec.rs): no libavcodec.
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("android") {
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
            let ff = ["avcodec", "avutil", "swscale"].contains(&l);
            if ff && static_prefix().is_some() {
                println!("cargo:rustc-link-lib=static={l}");
            } else {
                println!("cargo:rustc-link-lib={l}");
            }
        } else if let Some(p) = f.strip_prefix("-L") {
            println!("cargo:rustc-link-search=native={p}");
        }
    }
}
