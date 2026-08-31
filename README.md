# Chuwi Hi10 X1 camera drivers (Linux)

Out-of-tree Linux kernel drivers that make **both cameras** on the Chuwi Hi10
X1 (N150) tablet work under Debian/Ubuntu-style distributions, packaged as
DKMS modules so they survive kernel upgrades.

This tablet ships two GalaxyCore camera sensors behind an Intel IPU6 ISP:

| Camera | Sensor    | ACPI HID   | Status |
|--------|-----------|------------|--------|
| Front  | GC5035 (5MP) | `GCTI5035` | Working |
| Rear   | GC8034 (8MP) | `GCTI8034` | Working |

Neither sensor had a public Linux driver anywhere before this. As far as I
could find, everyone else with this exact tablet (see
[intel/ipu6-drivers#341](https://github.com/intel/ipu6-drivers/issues/341))
hit the same dead end.

## What's in this repo

- **`gc5035/`** — front camera sensor driver. Based on Intel's own
  out-of-tree `gc5035.c` (written for an internal reference board, never
  merged upstream), fixed to work against a current mainline kernel's
  `INT3472` power/regulator handling.
- **`gc8034/`** — rear camera sensor driver. No driver or datasheet exists
  for this sensor publicly. This one was reverse-engineered from Intel's
  official signed Windows driver (`gc8034.sys`) using Ghidra, cross-validated
  against the known-good GC5035 register layout. See the comments at the top
  of `gc8034.c` for the one value (MIPI link frequency) that's a calculated
  estimate rather than an extracted one — capture works correctly with it,
  but flag it if you ever see corrupted frames.
- **`ipu-bridge/`** — a one-line patch to the in-kernel `ipu-bridge.c`
  (adds `GCTI5035`/`GCTI8034` to its sensor allowlist, `IPU_SENSOR_CONFIG`).
  Without this, the sensor drivers probe fine but never get wired into the
  CSI2 capture graph — this is the actual reason both cameras look
  "unsupported" out of the box even though the ISP driver itself
  (`intel-ipu6`) is already in mainline.

## Requirements

- A kernel with `intel-ipu6`/`intel-ipu6-isys` already built in (mainline
  since ~5.x; this was built/tested against Debian 13 "trixie", kernel
  6.12).
- `dkms`, `build-essential`, and matching kernel headers:
  ```
  sudo apt-get install dkms build-essential linux-headers-amd64
  ```
  (the `-amd64` metapackage, not just the version-pinned headers package,
  so a future kernel upgrade pulls matching headers automatically and DKMS
  can rebuild these modules without manual intervention)

## Install

```
git clone https://github.com/4WaYDp/chuwi-hi10x1-camera-drivers.git
cd chuwi-hi10x1-camera-drivers
sudo ./install.sh
sudo reboot
```

After rebooting:

```
media-ctl -p                 # gc5035 and gc8034 should show as [ENABLED]
cam -l                        # needs libcamera-tools; lists both cameras
```

GNOME's built-in Snapshot app, and anything else going through
PipeWire/libcamera, should now show a live preview from both cameras.

## Uninstall

```
sudo dkms remove -m gc5035 -v 1.0 --all
sudo dkms remove -m gc8034 -v 1.0 --all
sudo dkms remove -m ipu-bridge -v 1.0 --all
sudo depmod -a
```

## Known limitations

- No calibrated libcamera tuning file for either sensor (falls back to
  `uncalibrated.yaml`) — images can look slightly under-exposed or
  cool-toned compared to a fully tuned camera. Basic auto black-level/AWB/AGC
  still runs, this is a cosmetic gap, not a functional one.
- GC8034's analog-gain-to-register curve is a rough placeholder (affects
  exposure quality only).
- Camera index order from `cam -l` is **not** stable across boots — always
  check `cam -l` before picking a `-c N` index.
- Only tested on the Chuwi Hi10 X1 N150. It may well work unmodified (or
  with trivial changes) on other tablets/laptops using the same GC5035 or
  GC8034 sensor behind an Intel IPU6 ISP, but that's untested.

## Why isn't this upstream in the Linux kernel?

The `ipu-bridge.c` allowlist entries are small, safe, one-line additions
that would be reasonable to submit to `linux-media` — that just hasn't been
prepared/submitted yet. Contributions/PRs for that are welcome. The sensor
drivers themselves are more involved out-of-tree code that would need
proper upstream cleanup (Intel's original `gc5035.c` was never submitted
either) before they'd be a realistic upstream submission.

## Credits

- `gc5035.c` originally written by Intel/Bitland/Google for an internal
  ADL-M reference board (see file header), never merged upstream — sourced
  from the [`intel/ipu6-drivers`](https://github.com/intel/ipu6-drivers)
  repository and fixed here to work against a current mainline kernel.
- `ipu-bridge.c` is Linux kernel code, originally authored by Dan Scally
  (`djrscally`); this repo carries a small allowlist patch on top of it.
- `gc8034.c` is a new driver written for this project, reverse-engineered
  from Intel's official Windows driver for this exact tablet model.

## License

GPL-2.0, matching the license of the Linux kernel code this is built on
and derived from. See [LICENSE](LICENSE).
