# Vendored driver patches

The control center's calibration stack (Sensor Setup, `calibrate_room.py`,
static sensor poses, the OVR-style fusion) needs driver features that are
not upstream. They are vendored here as patches; **Setup & install applies
them automatically** after cloning the pinned upstream commits
(`riftcv1/config.py`: `OPENHMD_COMMIT`, `STEAMVR_OPENHMD_COMMIT`).

## openhmd-rift-room-config.patch

Applies to [thaytan/OpenHMD](https://github.com/thaytan/OpenHMD)
branch `rift-kalman-filter` at the pinned commit. Adds, on top of the
stock CV1 constellation driver:

- **Room config load/store** (`rift-tracker-config.{c,h}`):
  `~/.config/openhmd/rift-room-config.json` with per-sensor pose,
  `room-center-offset` and `room-yaw-offset`. Sensor poses are **static
  at runtime** — the per-session pose servo is disabled (it random-walked
  the calibration; see CALIBRATION.md).
- **Calibration capture hook** (`rift_cal_capture_*` in
  `rift-sensor-pose-search.c`, registration in `rift-sensor.c` /
  `rift-tracker.c`): with `OHMD_RIFT_CAL_CAPTURE=<file>` set, every
  LED-ID-verified observation is appended as a JSON line — the input for
  `calibrate_room.py` and the Sensor Setup wizard.
- **OVR-style fusion backend** (`rift-fusion-ovr.{c,h}`, selected by
  `OHMD_RIFT_FUSION=ovr`, the default; `ukf` = original backend): a port
  of the complementary-filter fusion Oculus shipped openly in SDK
  0.2.5–0.3.2, with the SDK's own gains (see FUSION.md).
- Velocity export, observation-confidence tiers, and related tracking
  robustness fixes.

## steamvr-openhmd-driver.patch

Applies to
[ChristophHaag/SteamVR-OpenHMD](https://github.com/ChristophHaag/SteamVR-OpenHMD)
master at the pinned commit: zero-initialises `DriverPose_t` and exports
the driver's linear/angular velocities to SteamVR instead of leaving
them stale.

## Licenses

These patches modify and derive from their upstream projects and carry
those projects' licenses (OpenHMD: Boost Software License 1.0;
SteamVR-OpenHMD: its repository license), not this repository's MIT
license. `rift-fusion-ovr.c` is a reimplementation/port of fusion code
published by Oculus in the openly-released SDK 0.2.5–0.3.2
(`OVR_SensorFusion.cpp`).

## Updating

The patches were generated from the working checkouts with:

    git diff <pinned-commit>            # in each repo

To move to a newer upstream, rebase the checkout, regenerate the patch,
and bump the pinned commit in `riftcv1/config.py`.
