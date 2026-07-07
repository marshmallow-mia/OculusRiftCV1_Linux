# How the official Oculus runtime tracks the CV1 — and our 1:1 port

Reverse-engineering synthesis (2026-07-06) from primary sources: the fusion
source code Oculus shipped openly in SDK 0.2.5–0.3.2 (`OVR_SensorFusion.cpp`),
the LaValle et al. ICRA 2014 paper and Oculus engineering blog posts, the
predictive-tracking patent US9063330, and the community reverse engineering of
the CV1 hardware pipeline (Jan Schmidt / thaytan, Oliver Kreylos / doc-ok).

## The Windows pipeline

**Hardware / vision front end** (identical in our stack — thaytan's driver
already replicates this):
- Sensors are dumb cameras (AR0134 global shutter, 1280×960 @ 52.08 Hz,
  19.2 ms interval); ALL processing happens on the host. Exposures (~350 µs)
  are radio-triggered by the headset so all sensors expose simultaneously.
- Headset LEDs blink 10-bit IDs (Hamming distance 3); pose from PnP against
  the factory LED model. (The late official runtime moved to
  correspondence-free matching; blink IDs still work.)
- HMD IMU 1000 Hz, Touch 500 Hz via the headset radio.

**Fusion** (the part we ported — SDK 0.3.2 was the last open version, direct
ancestor of the CV1 service):
- The displayed pose IS the IMU dead-reckoning state. Gyro integrated at
  1000 Hz: `Q = Q * quat(ω, |ω|·dt)`. No Kalman filter — Oculus explicitly
  rejected Kalman/particle filters for a complementary filter with hand-tuned
  scalar gains ("simplicity … adjustment based on perceptual experiments").
- **Vision never touches pitch/roll.** Gravity does: accel is low-passed in
  the body frame (gain 2.5, ~1 s window, confidence from stddev), and tilt is
  corrected by `gain 0.25/s`, snapping only when error > 0.1 rad with
  confidence > 0.75.
- **Vision corrects yaw only** (`extractYawRotation` of the error), gain
  0.25/s, snap above 0.1 rad.
- **Vision corrects position** through three parallel channels per axis:
  position (gain 10,10,8 /s), velocity (50,50,32 /s), and an accelerometer
  bias integral (25,25,16 /s); snap above 0.1 m or on reacquisition.
- Corrections are applied every IMU sample as `error_fraction = gain·dt`
  (≈0.00025 of the error per millisecond) — "large enough to correct all
  drift, small enough to be imperceptible". Applied corrections are also
  applied to the stored exposure records so the same error is never corrected
  twice.
- Camera latency is handled by snapshotting the state at each exposure
  (our delay slots = their `ExposureRecordHistory`) and computing the vision
  error against the snapshot, not against the present state.
- **Prediction is perceptually tuned** (patent US9063330 + shipped code):
  `dt_predict = min(requested, 0.2 × (|ω| + |v|))`, i.e. a stationary head
  gets (almost) no prediction — jitter is invisible when moving and latency is
  invisible when still. Constant-angular-velocity model, 0.1 s hard cap.
  Under SteamVR this stage belongs to vrserver; it predicts using the
  velocities the driver reports.

## What we ported and where

- `subprojects/openhmd/src/drv_oculus_rift/rift-fusion-ovr.{c,h}` — faithful
  port of the 0.3.2 algorithm with the SDK's own constants (above).
- `rift-tracker.c` dispatches per `OHMD_RIFT_FUSION`: default `ovr`
  (the port), `ukf` = thaytan's original UKF. Both stay built; switch at
  launch, no rebuild.
- Velocity export (both backends): linear velocity in WORLD space
  (undisputed). Angular velocity frame is genuinely contested: Valve's
  driver docs say "driver world space", but Valve's own lighthouse driver
  observably emits device-local (Monado converts local→world when consuming
  it and world→local when feeding SteamVR; ALVR conjugates controller ω).
  Default: device-local. `OHMD_RIFT_ANGVEL_FRAME=world` switches for A/B.
- `OHMD_RIFT_NO_BLEED=1` disables the UKF-mode output-correction smoothing
  (OVR mode never steps by construction, so it has nothing to bleed).

## Launch-option cheat sheet (SteamVR launch options)

    OHMD_RIFT_FUSION=ukf %command%          # back to the UKF backend
    OHMD_RIFT_ANGVEL_FRAME=world %command%  # world-frame angular velocity
    OHMD_RIFT_NO_BLEED=1 %command%          # UKF mode: raw optical steps

After every driver rebuild run `bash install_files_to_build.sh` in
~/git/SteamVR-OpenHMD — SteamVR loads `build/bin/linux64/driver_openhmd.so`,
a copy that plain ninja does NOT refresh (the control center's status row
warns when it is stale).

## Differences that remain (deliberate)

- Our camera extrinsics come from offline bundle adjustment
  (`calibrate_room.py`, CALIBRATION.md); the official runtime self-calibrates
  sensor pose from the IMU with a "very long-baseline low-pass filter".
  Ours is strictly tighter.
- Official prediction-to-photon-time lives in vrserver in a SteamVR world;
  matching its perceptual tuning would need SteamVR-side changes. The driver
  contributes correct velocities instead.
- Multi-sensor: the official merge policy was never documented. We feed every
  LED-ID-verified per-sensor PnP fix through the same correction path
  (interleaved ~2×54 Hz), weighted by the observation-confidence tiers.
