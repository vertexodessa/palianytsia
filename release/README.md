# Palianytsia v0.12

A ground station for the mabur FPV link, plus the drone daemon it pairs with.

| File | What it is |
|---|---|
| `gs/palianytsia-quest-0.12.0.apk` | Meta Quest 3 app (OpenXR). |
| `gs/palianytsia-phone-0.12.0.apk` | Android phone app (arm64, Android 10+). |
| `gs/pal-desktop-linux-x86_64` | Linux desktop GS. |
| `gs/gs.toml` | The GS config the APKs carry. Pass it to the desktop app with `-c`. |
| `drone/maburd` | Drone daemon (OpenIPC SigmaStar, armv7 glibc). RC_VERSION 18; supports RTL8812EU/8822E radios. |
| `drone/install.py` | Installs maburd on a stock OpenIPC FPV image, updates or rolls it back, or removes it (see below). |
| `drone/S96mabur` | The init script that runs maburd; `install.py` puts it in place. |
| `drone/mabur.toml` | Drone config: 1920x1080@90 binned (`venc.binning = true`), 4 slices, channels 40/64/112/144. `radio.width = 40` lets the GS pick 20 or 40. |
| `drone/sensor_imx415_star6e.ko` | waybeam's six-mode IMX415 sensor driver (native 1:1 windows up to 100 fps). `install.py` installs it on an imx415 drone. |

## What changed since v0.11

Update both ends to get the drone-side changes. The radio protocol and the drone parameter table are unchanged, so a v0.12 ground station works with a v0.11 drone and the other way round.

- **The link stays on its top rung.** A piece of a frame that arrived as a requested re-send was counted as loss, and the link stepped down for it although nothing was lost. It no longer is.
- **Smaller keyframes by default.** `VENC_MIN_IQP` ships at 42 instead of 34.
- **No frame aggregation on the drone by default.** One lost aggregate used to take three or more video packets at once. The air-capacity tables in the drone config are re-measured for single frames.
- **40 MHz video no longer stalls on RTL8822E boards that need a receiver gain floor.** `maburd` now sets one (`radio.wide_tx_igi_floor` in `mabur.toml`, default 53). 40 MHz is still experimental, see the known limits.
- **Quest settings menu.** The menu is a panel of its own that opens in front of you and stays put in the room. Its tabs follow the task: Flight, Picture, Link, Recording, Screen, System. The right stick scrolls it.
- **The picture recovers from a decoder failure** on the phone and the Quest: a dead decoder is rebuilt and one keyframe is requested. A frozen picture is marked on the screen.
- **The ground station finds a drone on any 5 GHz channel**, not only on the channels of its set.
- **Faults a damaged or crafted video stream could trigger are fixed.** A forged picture size or slice layout could end the ground station, a forged or repeated frame number could freeze the picture, and a corrupted packet could put its data into the wrong stream.
- **RTL8812AU ground cards at 40 MHz on channels 153 and 161** transmit on their channel again.
- **Flight log.** `pal log` prints latency percentiles (capture on the drone to a complete frame on the ground) for the session and per rung, keyframe traffic, the decoder's loss episodes and the drone's parameters. A crash leaves `panic.txt`. `pal --version` and each session name the build.

An updated drone keeps its `mabur.toml`, so it keeps its old values. To get the new defaults on such a drone, set `[venc] min_iqp = 42`, `[ampdu] max_num = 0` and the two `[air_clock] efficiency_` lines from the shipped `mabur.toml`. `radio.wide_tx_igi_floor` needs no edit: a config without it gets the default.

On one bench (RTL8822E drone, 2944x1656 at 60 fps, 20 MHz, runs of 1 to 3 minutes per setting):
- Rung changes per hour: 254 and 507 in two runs of 150 s before, 234 in one run of 100 s after, with the top rung held 91 % of that run. With the re-sends switched off the same link changed rung 101 and 152 times per hour.
- Keyframe on the top rung: 214 kB median (230 ms of air) at `VENC_MIN_IQP` 34, 40 kB (43 ms) at 42.
- Latency, capture to complete frame: median 27 ms, 90 % under 31 ms, 99 % under 57 ms.
- Aggregation off costs air capacity at the fast rates: 11 % at MCS 4, 16 % at MCS 5. The shipped ladder tops out at MCS 4.

v0.11 notes: air clock bound, rung change hold, `VENC_MIN_IQP` in the menu.

## Ground station

Plug an RTL8812AU/8812EU card into the headset or phone with a USB-C OTG adapter.

1. Install:

   ```
   adb install palianytsia-quest-0.12.0.apk    # or the phone APK
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
./pal-desktop-linux-x86_64 -c gs.toml
```

It runs the same core and channel logic as the APKs, including the channel menu (`--core ref` runs mabur's C++ core instead, without the channel menu). The card needs USB access, either through a udev rule or by running as root.

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

## Known limits in v0.12

- Pairing uses the default link key on both ends.
- The changes of v0.12 have been run on one bench and not flown. The phone app was built but not run for this release; the Quest app and the desktop were.
- 40 MHz video is experimental. The gain floor default clears every board measured so far, but its cost for the control link at long range is not measured; a board's own lowest working value is lower (36 on the bench board), and nothing measures it for you yet. Calibrate with the ground station set to 40 MHz, or the 40 MHz power table stays the reference board's. Keep `encoder.bitrate_max_kbps` at what your drone can send: above that, frames arrive late and are cut.
- The link does not notice frames that arrive late, only frames that are lost. A bitrate cap set too high for the drone shows as cut frames on the top rung.
- The new FEC placement costs drone CPU: the hot thread goes from about 12 % to 19 % of a core at 120 fps and 13 Mbps.
- The changes of v0.11 have not been flown either.
- The per-frame FEC window of v0.10 has not been flown: host simulation and one bench run only. The worst inline repair batch at high rates is not measured.
- At 30-60 fps under steady random loss, a model of the link loses slightly more whole frames with repairs held to the frame end than without (0.6 % against 0.2 % of frames at 4 % loss, before NACK). On the air the difference was within noise.
- While disarmed, the drone runs low power: 30 fps at 1 Mbps. Full rate starts on arming.
- Above 90 fps and every native mode need waybeam's six-mode driver (`install.py` installs it); on any other sensor driver those modes are refused.
- DFS channels (52-144) are marked in the channel menu but neither end detects radar. Whether you may use them is up to your local rules.
- On one card the channel scan dwells on the link card in short slices; expect a little more loss while it runs. With two cards the spare card scans.
- Calibration with two cards measures the channels on the second card. If the second card does not receive, the link drops during the measure and the run fails with "calibration ack timeout". The OSD cards chip shows such a card as DEAF; replug it before calibrating.
- Calibrate in the field with the drone at a distance, never next to the GS. Next to the GS (RSSI around -30 dBm) the receivers overload and the higher MCS rungs fail, so the bitrate ladder also stays low on the bench.
- The two-card transmit switch, the channel menu and hops between any grid channels are tested on the bench, not yet in flight.

## Sources

- palianytsia: `main` @ @PAL_SHA@.
- mabur: fork branch `release/v0.12` @ @MABUR_SHA@ (`release/MABUR_RELEASE`): `release/v0.11` plus the receiver gain floor key, the new drone defaults and the ground-station fixes from `palianytsia/base-v2`.
- `sensor_imx415_star6e.ko`: waybeam's six-mode IMX415 driver (sha256 `d9b09101...`; waybeam_venc 581804a, as shipped in openipc-builder's mabur image at 49087b5f): native 2944x1656 at 60.57 Hz and 2272x1278 at 100.52 Hz (the build before it, `c639856a...`, had 2304x1296 at 99.63 Hz).
- devourer: @DEVOURER_SHA@.
- FFmpeg 6.1.2 (LGPL), HEVC decoder, its nvdec/vaapi/vdpau hwaccels and swscale, linked into the desktop app (`tools/ffmpeg-static.sh`).
