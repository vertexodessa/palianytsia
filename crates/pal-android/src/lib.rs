//! The Android link shared by apps/phone and apps/quest: given the usbfs fd
//! UsbManager handed out, a decoder output window and the GS config, run
//! the core loop (`runtime::run_core_on`) and MediaCodec on their own threads
//! and write the flight log. Drawing (OSD, XR layers) stays in the apps,
//! which read the [`Ports`].
#[cfg(target_os = "android")]
mod link;
#[cfg(target_os = "android")]
pub use link::*;
