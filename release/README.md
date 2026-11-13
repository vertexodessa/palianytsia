# Palianytsia v0.13.1

A ground station for the mabur FPV link, plus the drone daemon it pairs with.

| File | What it is |
|---|---|
| `gs/palianytsia-quest-0.13.1.apk` | Meta Quest 3 app (OpenXR). |
| `gs/palianytsia-phone-0.13.1.apk` | Android phone app (arm64, Android 10+). |
| `gs/pal-desktop-linux-x86_64` | Linux desktop GS. |
| `gs/gs.toml` | The GS config the APKs carry. Pass it to the desktop app with `-c`. |
| `drone/maburd` | Drone daemon (OpenIPC SigmaStar, armv7 glibc). RC_VERSION 18; supports RTL8812EU/8822E radios. |
| `drone/install.py` | Installs maburd on a stock OpenIPC FPV image, updates or rolls it back, or removes it (see below). |
| `drone/S96mabur` | The init script that runs maburd; `install.py` puts it in place. |
| `drone/mabur.toml` | Drone config: 1920x1080@90 binned (`venc.binning = true`), 4 slices, channels 40/64/112/144. `radio.width = 40` lets the GS pick 20 or 40. |
| `drone/sensor_imx415_star6e.ko` | waybeam's six-mode IMX415 sensor driver (native 1:1 windows up to 100 fps). `install.py` installs it on an imx415 drone. |

## What changed in v0.13.1
- **The desktop app has the channel menu.** Gear, Link, Channels, as in the Quest and phone apps: it scans the 5 GHz channels, shows foreign frames per second and a DFS mark for each, and the ticked ones are where the link may sit. One ticked channel holds the link there; a change moves a live link by a hop. The choice is saved and applied at the next start. Before, the desktop had no way to choose a channel but `--ch`, although this README said it had the menu. `--ch N` still holds one channel with no search and no menu.
- **The desktop applies "Step down on enhance-layer loss".** The switch was shown and saved but only the apps used it.
- **README:** the desktop writes a flight log only when started with `--log-dir DIR`.

maburd, the drone config and the Quest and phone apps are the same as in v0.13 (the APKs only carry the new version number). A v0.13 ground station and drone work with v0.13.1.

## What changed since v0.12
Update both ends together. The drone parameter table has two new rows, so a v0.13 ground station with an older drone (or the other way round) shows "table mismatch" and the new settings without a value. The video itself still flies between v0.12 and v0.13.

- **The picture no longer stalls when the stream runs near the drone's limit.** Above about 18 Mbit/s the drone's video thread waited for the thread that builds the repair packets, up to 167 ms at a time, and stayed in that state: frames were skipped at the camera, the ones that followed were large and arrived late. It now waits only when it must, and that did not happen in any run. When the drone's CPU cannot build every repair packet it sends fewer of them instead of holding frames back.
- **Periodic refresh is a setting.** Picture, Advanced: "Periodic refresh every, s" and "Refresh picture instead of a keyframe". Off by default (the ground station asks for a keyframe when its picture is damaged). With the second one on, the drone sends one all-intra picture per period instead of a keyframe. Each change restarts the video for about 0.7 s.
- **Calibration covers both widths.** One start calibrates TX power and capacity at 20 MHz, then measures the board's own 40 MHz receive-gain floor and writes it (never under the default 53), then calibrates 40 MHz. If the 40 MHz part fails, what 20 MHz applied stays. The floor measurement needs a quiet channel; on a busy one the drone says so and nothing is written.
- **Flight log.** `pal log` says how many frames its latency percentiles leave out (truncated, never emitted). Channel-search events carry the time they happened.

On one bench (RTL8822E drone, 2944x1656 at 60 fps, 40 MHz):
- Bitrate ceiling raised to 20000 kbps, 60 s runs alternating, two each: before, 30 % and 27 % of the camera's frames were missing, 95 % of the frames were complete within 161 and 159 ms, and the ground station asked for a keyframe 64 and 44 times; after, 2 frames of about 3400 were missing, 95 % within 33 ms, 1 request.
- At the shipped ceiling of 18000 kbps, one run of 15 minutes each: before, 26 waits, the longest 105 ms, and a worst frame of 168 ms; after, no wait and a worst frame of 84 ms. The typical frame is the same (median 24 and 25 ms, 95 % within 31 ms), and neither run fell into the stalled state. After the change 0.2 % of the camera's frames were missing, most of them in two stretches of 2 s that are not explained yet (before: 0.04 %).
- Refresh every second, 30 pictures removed from a recording: with a keyframe or with the refresh picture the image is whole again at the next refresh; with the refresh off it stays damaged until a keyframe is asked for.
- Under loss, with the Quest 3 as the ground station, 40 s runs at 20000 kbps: with 200 foreign frames a second on the channel, 8 and 5 damaged base pictures and 10 and 7 keyframe requests before, none and 2 and 2 after; with 8 % of the packets dropped at each receiving card, 6 damaged base pictures and 95 % of the frames within 141 ms before, none and 48 ms after. The missing repair packets cost nothing there. A weak signal at range is not measured.

From the Quest and phone apps, a calibration against a v0.13 drone runs the 40 MHz steps too; those have host tests only.

v0.12 notes: the link stays on its top rung (NACK residual rule), `VENC_MIN_IQP` 42, A-MPDU off by default, the 40 MHz receive-gain floor, the Quest menu panel.

## Ground station

Plug an RTL8812AU/8812EU card into the headset or phone with a USB-C OTG adapter.

1. Install:

   ```
   adb install palianytsia-quest-0.13.1.apk    # or the phone APK
   ```

   The APKs are signed with the Palianytsia release key (SHA-256 `81f44154...b7a6`). A device that has a debug build must uninstall it first:

   ```
   adb uninstall org.palianytsia.quest
   ```

   Uninstalling deletes the app's flight logs.
2. Start the app and accept the USB permission prompt.
3. Wait for the link. The GS listens across channels 40/64/112/144 and finds the drone wherever it is parked. It then moves the link to the cleanest channel among the ones you chose and hops away from interference while the link is up.
4. To choose channels: gear, Link, Channels. The menu scans the channels first (about 10 s for all 25) and shows each one's busy time, foreign frames per second and a DFS mark; tick the ones the link may use. One ticked channel keeps the link on it; several let the GS hop between them. The choice applies to the running link at once (a hop, no reconnect) and is saved.

Flight logs are written to `Android/data/org.palianytsia.<quest|phone>/files/logs/`.

### Desktop

The desktop app was built on Ubuntu 24.04 and needs glibc 2.39 or newer (Ubuntu 24.04, Debian 13, Fedora 40 or later), plus `libusb-1.0`, `libudev`, `libva`, `libvdpau` and `libX11` (present on any desktop with a Mesa or NVIDIA driver; on a minimal Fedora: `sudo dnf install libva libvdpau libusb1`).

The decoder is picked at start and named on the console. `decode: hwaccel vdpau` (or `vaapi`, `cuda`) means the GPU decodes; `decode: no hwaccel opened; software decode` means the CPU does, because no GPU decoder opened on this machine. To pick one by hand: `--hwdec vdpau|vaapi|cuda|sw`. Known: `--hwdec cuda` can crash when the app exits, and vaapi stalls on NVIDIA's VA-API wrapper; `auto` tries vdpau first, which works on NVIDIA.

Run it as:

```
./pal-desktop-linux-x86_64 -c gs.toml --log-dir ~/pal-logs
```

It runs the same core and channel logic as the APKs, including the channel menu: gear, Link, Channels (`--core ref` runs mabur's C++ core instead, without the channel menu). `--ch N` holds one channel instead: no search and no menu, so the drone must already be on it. With `--log-dir` each run leaves a flight log folder `fl-...` there (without it, none); for a report send its `cal.log`, `events.jsonl` and `stats.jsonl` (the `bodies*.bin` files are the recorded air and are large). The card needs USB access, either through a udev rule or by running as root.

## Drone

Run this from a Linux or macOS PC with python3 that can reach the drone over ssh. The drone needs no scp.

```
cd drone
./install.py 192.168.1.10          # or root@<drone-ip>
```

It asks for the drone's root password once. The drone must be a SigmaStar ssc338q camera with an RTL8812EU or RTL8822E card (tested with the imx415 sensor). The script checks the SoC, the card and the free flash first and stops on a mismatch before changing anything.

On a stock OpenIPC FPV image (first install) it:

1. stops majestic and wifibroadcast (and waybeam, if present) and moves their init scripts to `/etc/init.d.off`, since maburd runs the encoder and the radio itself;
2. blacklists the kernel Wi-Fi drivers in `/etc/modprobe.d/palianytsia.conf`, because maburd drives the card from user space;
3. installs `maburd`, `mabur.toml` and the init script `/etc/init.d/S96mabur`, and reboots the drone.

The camera tuning file (`venc.sensor_bin`) is fitted to the drone on the way, on an update too: when the configured file under `/etc/sensors` is not on the drone, the script sets the image's own file for the drone's sensor (`<sensor>_infinity6e.bin`, else `<sensor>.bin`) and says so. With no such file maburd still starts, and the picture runs untuned.

On a drone that already runs maburd (update) it:

1. shows the sha256 of what runs now and of what is about to go in, and stops if they already match;
2. saves the current `maburd` and `mabur.toml` to `./drone-backup-<date>/` on the PC;
3. keeps the drone's `mabur.toml` (`--config` replaces it with the one in this folder);
4. stops the service, swaps the binary and starts it again.

On an imx415 drone, both ways, it also installs the six-mode sensor driver:

1. copies `sensor_imx415_star6e.ko` next to the stock `sensor_imx415_mipi.ko`;
2. patches the drone's `/usr/bin/load_sigmastar` on the PC so that it loads the six-mode driver when present and the stock one otherwise, and keeps the original as `/usr/bin/load_sigmastar.palianytsia`;
3. on an update, swaps the running driver once maburd has stopped. If the swap fails, the stock driver stays loaded and the next boot loads the new one.

`/proc/mi_modules/mi_sensor/mi_sensor0` then lists 6 modes; with native 90 or 100 it shows `Cur 2272x1278@100fps` (the encoder runs 1920x1080 from it).

Both ways it asks for confirmation (`-y` skips it), uploads to `/tmp` and checks the hashes there (a broken upload changes nothing), then checks the files on flash and waits for maburd's build line in `/tmp/mabur.log`. Expected:

   ```
   maburd: build mabur @MABUR_SHA@ devourer @DEVOURER_SHA@ RC_VERSION @RC_VERSION@
   ```

To roll back, run the same script with the backup folder it printed:

```
./install.py 192.168.1.10 ./drone-backup-20261101-181614
```

To go back to the stock image (majestic + wifibroadcast, the original `load_sigmastar` and sensor driver), with a reboot:

```
./install.py --remove 192.168.1.10
```

maburd refuses config keys it does not know, so `install.py` fits the config to the maburd it installs: it removes `fec.repairs_with_frame` when that maburd does not have the key (the original stays in the backup). A v0.8.x maburd takes that key as optional; a v0.7.1 or older one and v0.9 refuse a config that has it (v0.4 added `venc.binning`; a v0.3 binary refuses a config with it). Roll back with the backup folder (it restores the config it saved), not by copying an older maburd by hand.

### What a first install gives

A first install puts `drone/mabur.toml` on the drone as it is (an update keeps the drone's own file). With it and the ground station's defaults:

| | Out of the box | Changed in |
|---|---|---|
| Picture | 1920x1080 at 90 fps, full view (binned) | settings, Picture tab |
| Video bitrate | 1 to 14 Mbit/s, chosen by the link; at most 65 % of the capacity measured for the rate in use | Picture tab, Advanced: `ENC_BITRATE_MAX`, `ENC_AIRTIME` |
| Width | 20 MHz (the ground station's default; the drone follows the ground station) | Link tab |
| Channels | 40, 64, 112, 144, picked by the ground station | Link tab, Channels |
| TX power | the card's own default power; the wall tables in the file are not used (`power_mode = "none"`) | Calibrate |
| Capacity tables | the reference board's, in use from the first flight | Calibrate |
| Error correction | +50 % repair data on the base layer, window 64; frame aggregation off | `mabur.toml` |
| Keyframes | only when the ground station asks for one; quality floor `min_iqp` 42 | Picture tab |
| Flight controller | MSP on `/dev/ttyS2` at 115200 for the OSD and the arm state; while disarmed the video drops to 1 Mbit/s at 30 fps | `mabur.toml`, `[msp]` and `[low_power]` |
| Recording | on the drone's microSD when the ground station asks, 1920x1080 at 36 Mbit/s | Recording tab |
| Pairing | the built-in default key | `/etc/mabur.key` on both ends |

Calibrate (Link tab, drone at a distance and disarmed) measures this drone's card instead of the reference board: the highest clean TX power per rate (the walls) and the delivered capacity per rate, at 20 MHz and, against this maburd, at 40 MHz with the board's receive-gain floor. It writes them into the drone's `mabur.toml` and switches `power_mode` to `"offset"`. From then on each rate is sent `wall_margin_db` (6 dB) under its wall. On the bench drone a margin of 1 dB parked MCS 4 about 2 dB above the card's default power, where frames already drop; with 6 dB the top rung held 585 of 610 s, against 67 of 606 s uncalibrated.

When to raise the bitrate ceiling: 14 Mbit/s fits 1080p90 at 20 MHz. At 40 MHz, or with a native (not binned) camera mode, the link carries more; if the OSD shows the bitrate at the ceiling on the top rung, raise `ENC_BITRATE_MAX` (the bench drone runs 18000 with `ENC_AIRTIME` 0.75 at 40 MHz). Values above the factory ones were run on one bench drone and not flown.

## Known limits in v0.13

- Pairing uses the default link key on both ends.
- The changes of v0.13 have been run on one bench and not flown. The released Quest app has run there (video under interference and simulated loss); its menu was not opened, so the two new settings have not been used in the headset. The phone app was built but not run.
- Calibration: against a v0.13 drone the Calibrate button of the Quest and phone apps, and of a desktop started with the shipped `gs.toml` and no `--ch`, runs five steps: power and capacity at 20 MHz, the receive-gain floor, power and capacity at 40 MHz. The apps have no button for the 20 MHz part alone. A desktop pinned to a channel with `--ch` on a 20 MHz link runs the 20 MHz part only (run once on the bench against this maburd). The floor measurement has been run on one board. The 40 MHz power and capacity steps have host tests only.
- Periodic refresh: with the shipped frame layering only "off" (a keyframe) and "on" (one refresh picture) restore the whole image. A longer refresh stripe (`venc.intra_refresh_frames` above 1 in `mabur.toml`) restores the top of the picture only.
- 40 MHz video is experimental. The gain floor default clears every board measured so far, but its cost for the control link at long range is not measured; a board's own lowest working value is lower (36 on the bench board), and calibration now measures it on a quiet channel. Without a calibration the 40 MHz power table stays the reference board's. Keep `encoder.bitrate_max_kbps` at what your drone can send: above that, frames arrive late and are cut.
- The link does not notice frames that arrive late, only frames that are lost. A bitrate cap set too high for the drone shows as cut frames on the top rung.
- The new FEC placement costs drone CPU: the hot thread goes from about 12 % to 19 % of a core at 120 fps and 13 Mbps.
- The changes of v0.11 have not been flown either.
- The per-frame FEC window of v0.10 has not been flown: host simulation and one bench run only. The worst inline repair batch at high rates is not measured.
- At 30-60 fps under steady random loss, a model of the link loses slightly more whole frames with repairs held to the frame end than without (0.6 % against 0.2 % of frames at 4 % loss, before NACK). On the air the difference was within noise.
- While disarmed, the drone runs low power: 30 fps at 1 Mbps. Full rate starts on arming.
- Above 90 fps and every native mode need waybeam's six-mode driver (`install.py` installs it); on any other sensor driver those modes are refused.
- DFS channels (52-144) are marked in the channel menu but neither end detects radar. Whether you may use them is up to your local rules.
- On one card the channel scan dwells on the link card itself: about 30 ms off the video every 120 ms for the 6 s of a scan. On the bench that lost 19-27 % of the drone's frames and took the link to its lowest rung, back to the top about 8 s after the scan. So with one card the menu does not scan when it opens; press "scan" on the ground, not in flight. With two cards the spare card scans when the menu opens and the video is not touched.
- Calibration with two cards measures the channels on the second card. If the second card does not receive, the link drops during the measure and the run fails with "calibration ack timeout". The OSD cards chip shows such a card as DEAF; replug it before calibrating.
- Calibrate in the field with the drone at a distance, never next to the GS. Next to the GS (RSSI around -30 dBm) the receivers overload and the higher MCS rungs fail, so the bitrate ladder also stays low on the bench.
- The two-card transmit switch, the channel menu and hops between any grid channels are tested on the bench, not yet in flight.

## Sources

- palianytsia: `main` @ @PAL_SHA@.
- mabur: fork branch `release/v0.12` @ @MABUR_SHA@ (`release/MABUR_RELEASE`): `release/v0.11` plus the receiver gain floor key, the new drone defaults and the ground-station fixes from `palianytsia/base-v2`.
- `sensor_imx415_star6e.ko`: waybeam's six-mode IMX415 driver (sha256 `d9b09101...`; waybeam_venc 581804a, as shipped in openipc-builder's mabur image at 49087b5f): native 2944x1656 at 60.57 Hz and 2272x1278 at 100.52 Hz (the build before it, `c639856a...`, had 2304x1296 at 99.63 Hz).
- devourer: @DEVOURER_SHA@.
- FFmpeg 6.1.2 (LGPL), HEVC decoder, its nvdec/vaapi/vdpau hwaccels and swscale, linked into the desktop app (`tools/ffmpeg-static.sh`).
