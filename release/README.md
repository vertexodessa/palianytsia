# Palianytsia v0.14

A ground station for the mabur FPV link, plus the drone daemon it pairs with.

| File | What it is |
|---|---|
| `gs/palianytsia-quest-0.14.0.apk` | Meta Quest 3 app (OpenXR). |
| `gs/palianytsia-phone-0.14.0.apk` | Android phone app (arm64, Android 10+). |
| `gs/pal-desktop-linux-x86_64` | Linux desktop GS. |
| `gs/gs.toml` | The GS config the APKs carry. Pass it to the desktop app with `-c`. |
| `drone/maburd` | Drone daemon (OpenIPC SigmaStar, armv7 glibc). RC_VERSION 18; supports RTL8812EU/8822E radios. |
| `drone/install.py` | Installs maburd on a stock OpenIPC FPV image, updates or rolls it back, or removes it (see below). |
| `drone/S96mabur` | The init script that runs maburd; `install.py` puts it in place. |
| `drone/mabur.toml` | Drone config: 1920x1080@90 binned (`venc.binning = true`), 4 slices, channels 40/64/112/144. `radio.width = 40` lets the GS pick 20 or 40. |
| `drone/sensor_imx415_star6e.ko` | waybeam's six-mode IMX415 sensor driver (native 1:1 windows up to 100 fps). `install.py` installs it on an imx415 drone. |

## What changed in v0.14
- **The ground station updates the drone by itself.** The Quest, phone and desktop apps carry the maburd of their release. When one meets a disarmed drone that runs a different maburd (older or newer, or one on another RC version), it uploads its own over the radio link, runs it from RAM, waits for video from it, then writes it to flash and deletes the old one. It takes about 50 s with the video off; the banner says UPDATING DRONE with the step and a percentage, then DRONE UPDATED or DRONE UPDATE FAILED. Settings, System shows what the drone runs. If the new maburd does not run, or the link is lost before the flash write, the drone starts its old maburd again within 2 minutes by itself. The drone keeps its own `mabur.toml`; the keys a release sets on every drone are set as `install.py` does. Never while the drone is armed, and one try per app start.
- **This one install is still from a PC.** A drone learns the update lane with v0.14's maburd: install it once with `drone/install.py`, later releases arrive from the ground station. A v0.13 drone flies with a v0.14 ground station (same RC version and parameter table), and Settings, System says it cannot be updated over the air.
- **A picture that cannot heal is named.** When four keyframe requests in a row bring no whole keyframe, the ground station asks every 2 s, then 4 s, instead of every second, and the banner says PICTURE CANNOT HEAL until a keyframe comes in whole.
- **A drone on another RC version is named on screen** (DRONE SPEAKS RC N, THIS APP RC M) instead of an endless search.
- **The drone gives way to a neighbour more gently.** While another transmitter keeps filling the drone's queue, the drone lowers its video bitrate a step (at most three, each 25 %) instead of dropping every other frame again and again; the OSD shows a "half fps" chip while it does drop frames.
- **Channels.** With one card the channel scan takes one 12 ms look every 250 ms (5 % of the frames lost during a scan on the bench, against 19 % before), and the menu scans on open only with a spare card. At boot the link moves to the chosen channel only once video flows.
- **Calibration.** One failing cell with three clean cells above it no longer ends a power wall low; replayed over ten calibration logs, 14 of 158 walls move up.
- **install.py.** An update also sets A-MPDU off, the 6 dB wall margin, `venc.min_iqp` 42 and, on an uncalibrated drone, the single-frame capacity tables. The camera tuning file is fitted to the drone's sensor. The shipped config has a wall margin of 6 dB.
- **OSD.** The video chip turns amber while the decoder throws video away; the stats chips sit below the HUD's top line; the drone chip says stale without a session.

On one bench (RTL8822E drone, ch 144): the released Quest app met the drone on an older maburd and had it on this release's maburd, written to flash, 48.5 s later; the desktop app did the same in 49 s and, started again, left the drone alone. A build that exits at once was refused by the trial and the drone stayed on its maburd. The phone app carries the same code and was not run against a drone. A power cut during the flash write (a few seconds) can leave the drone without maburd; `install.py` from a PC recovers it.

v0.13.1 notes: the channel menu in the desktop app. v0.13 notes: the picture no longer stalls near the drone's limit (FEC), periodic refresh as a setting, calibration of both widths.

## Ground station

Plug an RTL8812AU/8812EU card into the headset or phone with a USB-C OTG adapter.

1. Install:

   ```
   adb install palianytsia-quest-0.14.0.apk    # or the phone APK
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
3. keeps the drone's `mabur.toml` (`--config` replaces it with the one in this folder), except for the keys whose shipped value a release requires on every drone, which are set to the values in this folder's `drone/mabur.toml`, each change printed: `venc.min_iqp` (the keyframe size floor; 34 and below make keyframes of 200 kB and more), `ampdu.max_num` (0, A-MPDU off), `radio.wall_margin_db` (6) and, on a drone not calibrated yet (`power_mode` not `"offset"`), the capacity tables `air_clock.efficiency_20/_40`; a calibrated drone keeps its own tables and is told to calibrate again when they were measured with A-MPDU on;
4. checks the kept file against this folder's `drone/mabur.toml` and lists what needs a look (keys this maburd does not know and would not start on, values of the wrong type, keys the file lacks, for which maburd runs its built-in value) and the values set apart from the shipped ones;
5. stops the service, swaps the binary and starts it again.

On an imx415 drone, both ways, it also installs the six-mode sensor driver:

1. copies `sensor_imx415_star6e.ko` next to the stock `sensor_imx415_mipi.ko`;
2. patches the drone's `/usr/bin/load_sigmastar` on the PC so that it loads the six-mode driver when present and the stock one otherwise, and keeps the original as `/usr/bin/load_sigmastar.palianytsia`;
3. on an update, swaps the running driver once maburd has stopped. If the swap fails, the stock driver stays loaded and the next boot loads the new one.

`/proc/mi_modules/mi_sensor/mi_sensor0` then lists 6 modes; with native 90 or 100 it shows `Cur 2272x1278@100fps` (the encoder runs 1920x1080 from it).

Both ways it asks for confirmation (`-y` skips it), uploads to `/tmp` and checks the hashes there (a broken upload changes nothing), then checks the files on flash and waits for maburd's build line in `/tmp/mabur.log`. Expected:

   ```
   maburd: build mabur @MABUR_SHA@ devourer @DEVOURER_SHA@ RC_VERSION @RC_VERSION@
   ```

Once the drone runs this release's maburd, later releases need no PC: a ground station that carries another maburd installs it over the radio link while the drone is disarmed (see "What changed in v0.14"). It keeps the drone's own `mabur.toml` and sets the same release keys as an update by `install.py`.

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

## Known limits in v0.14

- The automatic drone update has been run from the Quest and desktop apps on one bench, not in the field. The phone app carries it and was not run against a drone. A power cut during the few seconds of the flash write can leave the drone without maburd; `install.py` from a PC recovers it. Two ground stations of different releases each install their own maburd, once per app start.
- Pairing uses the default link key on both ends.
- The changes of v0.13 and v0.14 have been run on one bench and not flown. The released Quest app has run there (video under interference and simulated loss); its menu was not opened, so the two new settings have not been used in the headset. The phone app was built but not run.
- Calibration: against a v0.13 or v0.14 drone the Calibrate button of the Quest and phone apps, and of a desktop started with the shipped `gs.toml` and no `--ch`, runs five steps: power and capacity at 20 MHz, the receive-gain floor, power and capacity at 40 MHz. The apps have no button for the 20 MHz part alone. A desktop pinned to a channel with `--ch` on a 20 MHz link runs the 20 MHz part only (run once on the bench against this maburd). The floor measurement has been run on one board. The 40 MHz power and capacity steps have host tests only.
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
- On one card the channel scan dwells on the link card itself: one dwell of about 12 ms every 250 ms, two passes, about 12.5 s for the 25-channel grid. On the bench that lost about 5 % of the drone's frames on average, with 16 truncated frames and 18 keyframe requests per scan. So with one card the menu does not scan when it opens; press "scan" on the ground, not in flight. With two cards the spare card scans when the menu opens and the video is not touched.
- Calibration with two cards measures the channels on the second card. If the second card does not receive, the link drops during the measure and the run fails with "calibration ack timeout". The OSD cards chip shows such a card as DEAF; replug it before calibrating.
- Calibrate in the field with the drone at a distance, never next to the GS. Next to the GS (RSSI around -30 dBm) the receivers overload and the higher MCS rungs fail, so the bitrate ladder also stays low on the bench.
- The two-card transmit switch, the channel menu and hops between any grid channels are tested on the bench, not yet in flight.

## Sources

- palianytsia: `main` @ @PAL_SHA@.
- mabur: fork branch `release/v0.14` @ @MABUR_SHA@ (`release/MABUR_RELEASE`): `palianytsia/base-v2` (upstream master with PR #120-#123 and our commits) without the bench-only `/sim` endpoint.
- `sensor_imx415_star6e.ko`: waybeam's six-mode IMX415 driver (sha256 `d9b09101...`; waybeam_venc 581804a, as shipped in openipc-builder's mabur image at 49087b5f): native 2944x1656 at 60.57 Hz and 2272x1278 at 100.52 Hz (the build before it, `c639856a...`, had 2304x1296 at 99.63 Hz).
- devourer: @DEVOURER_SHA@.
- FFmpeg 6.1.2 (LGPL), HEVC decoder, its nvdec/vaapi/vdpau hwaccels and swscale, linked into the desktop app (`tools/ffmpeg-static.sh`).
