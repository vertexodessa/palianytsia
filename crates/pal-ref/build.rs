//! Builds mabur's native web GS core with mabur-ground's own CMake (only the
//! `mabur_web_core` target and what it needs), then the C ABI shim against it.
//! The GS lives in mabur-ground since upstream's split; mabur (common/, the
//! wire formats and FEC) comes in as its subproject from our own pinned
//! `third_party/mabur`, not from mabur-ground's nested submodule, so one mabur
//! pin serves the drone build, the golden and this oracle. Host only: the test
//! oracle and pal-desktop --core ref.
use std::path::PathBuf;

fn main() {
    let root = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap()).join("../..");
    let mabur = root
        .join("third_party/mabur")
        .canonicalize()
        .expect("third_party/mabur missing: git submodule update --init");
    let ground = root
        .join("third_party/mabur-ground")
        .canonicalize()
        .expect("third_party/mabur-ground missing: git submodule update --init");
    let devourer = root
        .join("third_party/devourer")
        .canonicalize()
        .expect("third_party/devourer missing: git submodule update --init");

    let dst = cmake::Config::new(&ground)
        .define("MABUR_DIR", &mabur)
        .define("DEVOURER_DIR", &devourer)
        .define("GROUND_BUILD_TESTS", "OFF")
        .define("GROUND_BUILD_MABURD", "OFF")
        .define("MABUR_BUILD_LINKBENCH", "OFF")
        // Rust links PIE executables, so every archive must be PIC.
        .define("CMAKE_POSITION_INDEPENDENT_CODE", "ON")
        .profile("Release")
        .build_target("mabur_web_core")
        .build();
    let b = dst.join("build");

    cc::Build::new()
        .cpp(true)
        .std("c++20")
        .file("cpp/shim.cpp")
        .file("cpp/pal_card.cpp")
        .file("cpp/chan_trace.cpp")
        .file("cpp/verdict_diff.cpp")
        .file("cpp/control_frame.cpp")
        // The oracle of tests/control_frame.rs. mabur_gs_dot11 is a separate
        // CMake target and would pull the whole devourer library in for
        // this one radiotap builder.
        .file(ground.join("gs/src/dot11.cpp"))
        .file(devourer.join("src/RadiotapBuilder.cpp"))
        .include("cpp")
        .include(ground.join("web/src"))
        .include(ground.join("gs/src"))
        .include(ground.join("gs/player/src"))
        .include(mabur.join("common/include"))
        .include(mabur.join("third_party"))
        .include(devourer.join("src"))
        .warnings(true)
        // mabur headers trip -Wextra; the shim itself stays clean.
        .flag_if_supported("-Wno-missing-field-initializers")
        .compile("pal_ref_shim");

    // Dependency order: shim -> web core -> gs core -> common.
    for (dir, lib) in [
        ("web", "mabur_web_core"),
        ("gs", "mabur_gs_core"),
        ("mabur/common", "mabur_common"),
    ] {
        println!("cargo:rustc-link-search=native={}", b.join(dir).display());
        println!("cargo:rustc-link-lib=static={lib}");
    }
    println!("cargo:rustc-link-lib=dylib=stdc++");
    println!("cargo:rustc-link-lib=dylib=rt");
    println!("cargo:rustc-link-lib=dylib=pthread");

    println!("cargo:rerun-if-changed=cpp");
    println!("cargo:rerun-if-changed=build.rs");
    // Directories, not one header: cargo rescans them by mtime, so a fork
    // commit to web_gs.cpp (or to devourer, which mabur links) rebuilds the
    // oracle instead of linking the stale archive.
    for d in [
        ground.join("web/src"),
        ground.join("gs/src"),
        ground.join("gs/player/src"),
        mabur.join("common"),
        devourer.join("src"),
    ] {
        println!("cargo:rerun-if-changed={}", d.display());
    }
}
