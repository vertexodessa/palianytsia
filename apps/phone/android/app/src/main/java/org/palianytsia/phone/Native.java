package org.palianytsia.phone;

import android.view.Surface;

/** libpalphone (apps/phone/rust): the whole GS behind a few calls. */
final class Native {
    static {
        System.loadLibrary("palphone");
    }

    private Native() {}

    /** Returns a handle, or 0 with the reason in {@link #lastError()}. */
    static native long start(int usbFd, Surface video, Surface osd, float density,
            String configToml, String filesDir, String logDir);

    /** A touch for the Rust-drawn OSD: action 0 down, 1 up, 2 move,
     *  3 cancel; x, y in Surface pixels. */
    static native void touch(long handle, int action, float x, float y);

    /** False once the core thread has returned (card gone, USB error). */
    static native boolean running(long handle);

    /** After running() turned false: true when new link settings ended the
     *  link and the next one started on the same open card; false means
     *  stop and start over. */
    static native boolean relink(long handle);

    static native String lastError();

    static native void stop(long handle);
}
