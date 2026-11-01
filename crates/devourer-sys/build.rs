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

    let android = std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("android");
    let mut cfg = cmake::Config::new(&devourer);
    // Android: the NDK's CMake toolchain, and libusb from
    // tools/android-libusb.sh found through a pkg-config dir of its own
    // (devourer's CMake asks pkg-config; the host's .pc must not leak in).
    let libusb_inc = if android {
        let ndk =
            std::env::var("ANDROID_NDK_HOME").expect("ANDROID_NDK_HOME (tools/android-build.sh)");
        let usb = PathBuf::from(
            std::env::var("PAL_ANDROID_LIBUSB")
                .expect("PAL_ANDROID_LIBUSB (tools/android-libusb.sh)"),
        );
        println!("cargo:rerun-if-env-changed=PAL_ANDROID_LIBUSB");
        cfg.env("PKG_CONFIG_LIBDIR", usb.join("lib/pkgconfig"))
            .env("PKG_CONFIG_PATH", "")
            // The NDK toolchain confines find_library to the sysroot; the
            // imported target looks libusb up under its own prefix.
            .define("CMAKE_FIND_ROOT_PATH", &usb)
            .define(
                "CMAKE_TOOLCHAIN_FILE",
                format!("{ndk}/build/cmake/android.toolchain.cmake"),
            )
            .define("ANDROID_ABI", "arm64-v8a")
            .define("ANDROID_PLATFORM", "android-29");
        println!(
            "cargo:rustc-link-search=native={}",
            usb.join("lib").display()
        );
        usb.join("include/libusb-1.0")
    } else {
        PathBuf::from("/usr/include/libusb-1.0")
    };
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
        .include(&libusb_inc)
        .warnings(true)
        .flag_if_supported("-Wno-missing-field-initializers")
        .compile("dv_shim");

    println!("cargo:rustc-link-search=native={}", b.display());
    println!("cargo:rustc-link-lib=static=devourer");
    if android {
        println!("cargo:rustc-link-lib=static=usb-1.0");
        println!("cargo:rustc-link-lib=dylib=c++_shared");
        println!("cargo:rustc-link-lib=dylib=log");
    } else {
        println!("cargo:rustc-link-lib=dylib=usb-1.0");
        println!("cargo:rustc-link-lib=dylib=stdc++");
        println!("cargo:rustc-link-lib=dylib=pthread");
    }
    println!("cargo:rerun-if-changed=cpp");
    // Without these a fork commit in the submodule kept the old static lib:
    // a 2026-11-01 live run still printed the per-frame warning the fork
    // had already rate-limited.
    for p in ["src", "CMakeLists.txt"] {
        println!("cargo:rerun-if-changed={}", devourer.join(p).display());
    }
    println!("cargo:rerun-if-changed=build.rs");
}
