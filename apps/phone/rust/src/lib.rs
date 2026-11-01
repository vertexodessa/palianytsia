//! Phone app native side (spec 5.5). The Java Activity owns the USB
//! permission and the Surface; this library gets the usbfs fd, the Surface
//! and the config text, runs `runtime::run_live` on a core thread and
//! MediaCodec on a decode thread, and hands the newest stats line back.
//!
//! JNI (class org.palianytsia.phone.Native):
//!   long start(int fd, Surface video, Surface osd, float density, String configToml,
//!              String filesDir, String logDir)
//!                             0 = failed, see lastError()
//!   String stats(long h)      newest stats JSON ("" before the first)
//!   void touch(long h, int action, float x, float y)  touches for the OSD (gear, panel, tap toggles stats)
//!   boolean running(long h)  false once the core thread returned (card gone)
//!   String lastError()
//!   void stop(long h)
#[cfg(target_os = "android")]
mod android;
#[cfg(target_os = "android")]
mod osd;
