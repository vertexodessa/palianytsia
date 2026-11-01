package org.palianytsia.phone;

import android.view.Surface;

/** libpalphone (apps/phone/rust): the whole GS behind four calls. */
final class Native {
    static {
        System.loadLibrary("palphone");
    }

    private Native() {}

    /** Returns a handle, or 0 with the reason in {@link #lastError()}. */
    static native long start(int usbFd, Surface video, Surface osd, float density,
            String configToml, String filesDir);

    /** Newest core stats line (JSON), "" before the first. */
    static native String stats(long handle);

    /** A touch for the Rust-drawn OSD: action 0 down, 1 up, 2 move,
     *  3 cancel; x, y in Surface pixels. */
    static native void touch(long handle, int action, float x, float y);

    static native String lastError();

    static native void stop(long handle);
}
