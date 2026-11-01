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
            String configToml);

    /** Newest core stats line (JSON), "" before the first. */
    static native String stats(long handle);

    /** Link statistics over the flight HUD on/off. */
    static native void toggleStats(long handle);

    static native String lastError();

    static native void stop(long handle);
}
