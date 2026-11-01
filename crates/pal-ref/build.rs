//! Builds mabur's native web GS core from the pinned submodule with its own
//! CMake (only the `mabur_web_core` target and what it needs), then the C ABI
//! shim against it. Host (the test oracle) and aarch64 Android, where the
//! C++ core with its ChannelCore drives the card in the phone/Quest apps.
use std::path::PathBuf;

fn main() {
    let root = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap()).join("../..");
    let mabur = root
        .join("third_party/mabur")
        .canonicalize()
        .expect("third_party/mabur missing: git submodule update --init");
    let devourer = root
        .join("third_party/devourer")
        .canonicalize()
        .expect("third_party/devourer missing: git submodule update --init");

    let android = std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("android");
    let mut cfg = cmake::Config::new(&mabur);
    if android {
        // As devourer-sys: the NDK toolchain, and libusb (mabur's configure
        // pulls devourer's CMake in, which asks pkg-config for it) from
        // tools/android-libusb.sh. Nothing of it is linked here.
        let ndk =
            std::env::var("ANDROID_NDK_HOME").expect("ANDROID_NDK_HOME (tools/android-build.sh)");
        let usb = PathBuf::from(
            std::env::var("PAL_ANDROID_LIBUSB")
                .expect("PAL_ANDROID_LIBUSB (tools/android-libusb.sh)"),
        );
        println!("cargo:rerun-if-env-changed=PAL_ANDROID_LIBUSB");
        cfg.env("PKG_CONFIG_LIBDIR", usb.join("lib/pkgconfig"))
            .env("PKG_CONFIG_PATH", "")
            .define("CMAKE_FIND_ROOT_PATH", &usb)
            .define(
                "CMAKE_TOOLCHAIN_FILE",
                format!("{ndk}/build/cmake/android.toolchain.cmake"),
            )
            .define("ANDROID_ABI", "arm64-v8a")
            .define("ANDROID_PLATFORM", "android-29");
    }
    let dst = cfg
        .define("DEVOURER_DIR", &devourer)
        .define("MABUR_BUILD_TESTS", "OFF")
        .define("MABUR_BUILD_DRONE", "OFF")
        .define("MABUR_BUILD_LINKBENCH", "OFF")
        .define("MABUR_BUILD_GS", "ON")
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
        .include("cpp")
        .include(mabur.join("web/src"))
        .include(mabur.join("gs/src"))
        .include(mabur.join("gs/player/src"))
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
        ("common", "mabur_common"),
    ] {
        println!("cargo:rustc-link-search=native={}", b.join(dir).display());
        println!("cargo:rustc-link-lib=static={lib}");
    }
    if android {
        println!("cargo:rustc-link-lib=dylib=c++_shared");
    } else {
        println!("cargo:rustc-link-lib=dylib=stdc++");
        println!("cargo:rustc-link-lib=dylib=rt");
        println!("cargo:rustc-link-lib=dylib=pthread");
    }

    println!("cargo:rerun-if-changed=cpp");
    println!("cargo:rerun-if-changed=build.rs");
    // Directories, not one header: cargo rescans them by mtime, so a fork
    // commit to web_gs.cpp (or to devourer, which mabur links) rebuilds the
    // oracle instead of linking the stale archive.
    for d in [
        mabur.join("web/src"),
        mabur.join("gs/src"),
        mabur.join("gs/player/src"),
        mabur.join("common"),
        devourer.join("src"),
    ] {
        println!("cargo:rerun-if-changed={}", d.display());
    }
}
