//! Builds devourer from the pinned submodule with its own CMake, only the
//! chips the GS drives (8812AU Jaguar1, 8822E and 8822C Jaguar3; spec 5.2),
//! then the C shim.
use std::path::PathBuf;

fn main() {
    let root = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap()).join("../..");
    let devourer = root
        .join("third_party/devourer")
        .canonicalize()
        .expect("third_party/devourer missing: git submodule update --init");

    let mut cfg = cmake::Config::new(&devourer);
    for (opt, on) in [
        ("DEVOURER_JAGUAR1", true),
        ("DEVOURER_8814", false),
        ("DEVOURER_JAGUAR2_8822B", false),
        ("DEVOURER_JAGUAR2_8821C", false),
        ("DEVOURER_JAGUAR3_8822C", true),
        ("DEVOURER_JAGUAR3_8822E", true),
        ("DEVOURER_8733B", false),
        ("DEVOURER_KESTREL_8852B", false),
        ("DEVOURER_KESTREL_8852C", false),
        ("DEVOURER_PCIE", false),
        ("DEVOURER_MT7612U", false),
    ] {
        cfg.define(opt, if on { "ON" } else { "OFF" });
    }
    let dst = cfg
        .define("CMAKE_POSITION_INDEPENDENT_CODE", "ON")
        .profile("Release")
        .build_target("devourer")
        .build();
    let b = dst.join("build");

    cc::Build::new()
        .cpp(true)
        .std("c++20")
        .file("cpp/dv.cpp")
        .include("cpp")
        .include(devourer.join("src"))
        .include("/usr/include/libusb-1.0")
        .warnings(true)
        .flag_if_supported("-Wno-missing-field-initializers")
        .compile("dv_shim");

    println!("cargo:rustc-link-search=native={}", b.display());
    println!("cargo:rustc-link-lib=static=devourer");
    println!("cargo:rustc-link-lib=dylib=usb-1.0");
    println!("cargo:rustc-link-lib=dylib=stdc++");
    println!("cargo:rustc-link-lib=dylib=pthread");
    println!("cargo:rerun-if-changed=cpp");
    println!("cargo:rerun-if-changed=build.rs");
}
