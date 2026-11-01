package org.palianytsia.phone;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Color;
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

import org.json.JSONObject;

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
    private TextView osd;
    private UsbManager usb;
    private UsbDeviceConnection conn;
    private SurfaceHolder holder;
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
        holder = h;
        tryStart();
    }

    @Override
    public void surfaceChanged(SurfaceHolder h, int format, int w, int hgt) {}

    @Override
    public void surfaceDestroyed(SurfaceHolder h) {
        // MediaCodec renders into this Surface: stop before it goes away.
        stopLink();
        holder = null;
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
        if (handle != 0 || holder == null) return;
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
        handle = Native.start(conn.getFileDescriptor(), holder.getSurface(), cfg);
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
            String err = Native.lastError();
            String s = Native.stats(handle);
            osd.setText(err.isEmpty() ? summary(s) : "error: " + err + "\n" + summary(s));
            ui.postDelayed(this, 500);
        }
    };

    private static String summary(String json) {
        if (json.isEmpty()) return "no stats yet";
        try {
            JSONObject j = new JSONObject(json);
            String link = j.optBoolean("session")
                    ? "rung " + j.optInt("rung") + " MCS" + j.optInt("mcs") + " " + j.optInt("bw") + "M"
                    : "beaconing";
            return link
                    + "   rssi " + fmt(j, "rssi_dbm", "%.0f") + " dBm  SNR " + fmt(j, "snr_db", "%.0f")
                    + "   pre " + pct(j, "pre_fec_loss") + "  post " + pct(j, "residual")
                    + "   AUs " + j.optLong("aus") + "  trunc " + j.optLong("trunc")
                    + "   idr " + j.optLong("idr_req")
                    + (j.optBoolean("drone_lp") ? "   low power" : "");
        } catch (Exception e) {
            return json;
        }
    }

    private static String fmt(JSONObject j, String k, String f) {
        return j.isNull(k) ? "-" : String.format(f, j.optDouble(k));
    }

    private static String pct(JSONObject j, String k) {
        return j.isNull(k) ? "-" : String.format("%.1f%%", 100 * j.optDouble(k));
    }

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
