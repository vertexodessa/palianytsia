//! Quest 3 ground station (spec 5.6). A NativeActivity; everything is Rust:
//! - `usb`: UsbManager over JNI (find the card, permission, usbfs fd);
//! - `supervisor`: runs `pal_android::Link` on that fd and restarts it
//!   every 3 s when the card leaves or the core gives up; it ignores XR
//!   session state, so lifting the headset keeps the link up;
//! - `xr`: the OpenXR frame loop: MediaCodec renders straight into an
//!   XR_KHR_android_surface_swapchain shown as a head-locked quad, and
//!   `osd` draws pal-osd's HUD into a second, transparent quad above it.
#[cfg(target_os = "android")]
mod input;
#[cfg(target_os = "android")]
mod osd;
#[cfg(target_os = "android")]
mod perf;
// Called from xr (Android); host builds run its tests.
#[cfg_attr(not(target_os = "android"), allow(dead_code))]
mod rate;
#[cfg(target_os = "android")]
mod supervisor;
#[cfg(target_os = "android")]
mod usb;
#[cfg(target_os = "android")]
mod xr;

#[cfg(target_os = "android")]
#[no_mangle]
fn android_main(app: android_activity::AndroidApp) {
    android_logger::init_once(
        android_logger::Config::default()
            .with_max_level(log::LevelFilter::Info)
            .with_tag("palquest"),
    );
    if let Err(e) = xr::run(&app) {
        log::error!("quest: {e}");
    }
    // run returns once the link is stopped and the supervisor joined, so
    // nothing is left to flush. The process must not outlive the activity:
    // after a Quit from the Meta menu it stayed, the next launch reused it,
    // and the menu offered no Quit for that second activity (2026-11-03).
    log::info!("quest: activity gone, process exits");
    std::process::exit(0);
}
