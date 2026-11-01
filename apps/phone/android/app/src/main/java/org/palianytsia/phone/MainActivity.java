package org.palianytsia.phone;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Color;
import android.graphics.PixelFormat;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.Gravity;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.FrameLayout;
import android.widget.TextView;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;

/**
 * The thin shell of spec 5.5: USB permission, the video Surface, and a
 * text OSD until the Rust-drawn OSD surface lands. Everything else is in
 * libpalphone.
 */
public final class MainActivity extends Activity implements SurfaceHolder.Callback {
    private static final String TAG = "palianytsia";
    private static final String ACTION_USB = "org.palianytsia.phone.USB_PERMISSION";
    private static final int REALTEK = 0x0bda;
    /** devourer-sys kScanPids: 8812EU, 8812AU (881a), 8812AU. A hub's
     *  RTL8153 Ethernet (0bda:8153) is Realtek too and must not match. */
    private static final int[] CARD_PIDS = {0xa81a, 0x881a, 0x8812};

    private final Handler ui = new Handler(Looper.getMainLooper());
    private SurfaceView video;
    /** Transparent, above the video: Rust draws the pal-osd widgets here. */
    private SurfaceView osdView;
    /** Status text before the link is up and on errors. */
    private TextView osd;
    private UsbManager usb;
    private UsbDeviceConnection conn;
    private SurfaceHolder holder;
    private SurfaceHolder osdHolder;
    private long handle;

    private final BroadcastReceiver permission = new BroadcastReceiver() {
        @Override
        public void onReceive(Context c, Intent i) {
            if (!ACTION_USB.equals(i.getAction())) return;
            if (i.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                tryStart();
            } else {
                show("USB permission denied");
            }
        }
    };

    @Override
    protected void onCreate(Bundle b) {
        super.onCreate(b);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        usb = (UsbManager) getSystemService(Context.USB_SERVICE);
        FrameLayout root = new FrameLayout(this);
        root.setBackgroundColor(Color.BLACK);
        video = new SurfaceView(this);
        video.getHolder().addCallback(this);
        root.addView(video, new FrameLayout.LayoutParams(-1, -1, Gravity.CENTER));
        osdView = new SurfaceView(this);
        osdView.setZOrderMediaOverlay(true);
        osdView.getHolder().setFormat(PixelFormat.TRANSLUCENT);
        osdView.getHolder().addCallback(this);
        root.addView(osdView, new FrameLayout.LayoutParams(-1, -1, Gravity.CENTER));
        osd = new TextView(this);
        osd.setTextColor(Color.WHITE);
        osd.setShadowLayer(3, 1, 1, Color.BLACK);
        osd.setTextSize(13);
        osd.setPadding(24, 16, 24, 16);
        root.addView(osd, new FrameLayout.LayoutParams(-1, -2, Gravity.TOP));
        setContentView(root);
        IntentFilter f = new IntentFilter(ACTION_USB);
        if (Build.VERSION.SDK_INT >= 33) {
            registerReceiver(permission, f, Context.RECEIVER_NOT_EXPORTED);
        } else {
            registerReceiver(permission, f);
        }
        show("waiting for an RTL8812 card");
    }

    @Override
    protected void onDestroy() {
        stopLink();
        unregisterReceiver(permission);
        super.onDestroy();
    }

    @Override
    protected void onNewIntent(Intent i) {
        super.onNewIntent(i);
        tryStart();
    }

    @Override
    public void surfaceCreated(SurfaceHolder h) {
        if (h == osdView.getHolder()) {
            osdHolder = h;
        } else {
            holder = h;
        }
        tryStart();
    }

    @Override
    public void surfaceChanged(SurfaceHolder h, int format, int w, int hgt) {}

    @Override
    public void surfaceDestroyed(SurfaceHolder h) {
        // MediaCodec and the OSD render into these Surfaces: stop first.
        stopLink();
        if (h == osdView.getHolder()) {
            osdHolder = null;
        } else {
            holder = null;
        }
    }

    private UsbDevice findCard() {
        for (UsbDevice d : usb.getDeviceList().values()) {
            if (d.getVendorId() != REALTEK) continue;
            for (int p : CARD_PIDS) {
                if (d.getProductId() == p) return d;
            }
        }
        return null;
    }

    private void tryStart() {
        if (handle != 0 || holder == null || osdHolder == null) return;
        UsbDevice d = findCard();
        if (d == null) {
            show("no RTL8812 card");
            ui.postDelayed(this::tryStart, 3000);
            return;
        }
        if (!usb.hasPermission(d)) {
            // Android 14: the broadcast must be explicit, and USB extras need
            // a mutable PendingIntent.
            Intent i = new Intent(ACTION_USB).setPackage(getPackageName());
            PendingIntent pi = PendingIntent.getBroadcast(this, 0, i, PendingIntent.FLAG_MUTABLE);
            usb.requestPermission(d, pi);
            show("asking for USB permission");
            return;
        }
        conn = usb.openDevice(d);
        if (conn == null) {
            show("openDevice failed");
            return;
        }
        String cfg;
        try {
            cfg = readAsset("gs.toml");
        } catch (IOException e) {
            show("gs.toml asset: " + e);
            return;
        }
        String name = String.format("%04x:%04x", d.getVendorId(), d.getProductId());
        Log.i(TAG, "starting on " + name + " fd " + conn.getFileDescriptor());
        float density = getResources().getDisplayMetrics().density;
        handle = Native.start(conn.getFileDescriptor(), holder.getSurface(),
                osdHolder.getSurface(), density, cfg);
        if (handle == 0) {
            show("start failed: " + Native.lastError());
            conn.close();
            conn = null;
            return;
        }
        show("card " + name + " up");
        ui.postDelayed(tick, 500);
    }

    private final Runnable tick = new Runnable() {
        @Override
        public void run() {
            if (handle == 0) return;
            // The Rust OSD shows the link; this text is only for errors.
            String err = Native.lastError();
            osd.setText(err.isEmpty() ? "" : "error: " + err);
            ui.postDelayed(this, 500);
        }
    };

    private void stopLink() {
        ui.removeCallbacks(tick);
        if (handle != 0) {
            Native.stop(handle);
            handle = 0;
        }
        if (conn != null) {
            conn.close();
            conn = null;
        }
    }

    private void show(String s) {
        Log.i(TAG, s);
        osd.setText(s);
    }

    private String readAsset(String name) throws IOException {
        try (InputStream in = getAssets().open(name)) {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] buf = new byte[8192];
            int n;
            while ((n = in.read(buf)) > 0) out.write(buf, 0, n);
            return out.toString("UTF-8");
        }
    }
}
