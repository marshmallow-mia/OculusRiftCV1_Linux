# Forward prediction: what was actually wrong, and what changed

## The diagnosis we started with was wrong

The standing belief (plan §6, `HANDOFF-LINUX.md`) was "our driver does no forward
prediction, Oculus does, that's a 10× gap." The first half is false.

SteamVR is not a passive consumer of our pose. `GetDeviceToAbsoluteTrackingPose`
extrapolates the driver's pose to photon time itself, using `vecVelocity`,
`vecAngularVelocity`, `vecAcceleration` and `poseTimeOffset` from the
`DriverPose_t` we publish. Prediction was already happening — we were just feeding
it bad inputs.

Two concrete defects:

1. **`poseTimeOffset` was hardcoded to 0.** That asserts the pose was sampled at
   the instant of the `TrackedDevicePoseUpdated()` call. It wasn't: the fused pose
   is a state estimate at the last IMU sample, which on this hardware is
   **1.00 ms old** (median; p95 1.05 ms, very stable). SteamVR therefore
   extrapolated from an epoch 1 ms later than the truth and under-predicted by
   exactly that much.

2. **`vecAcceleration` was never populated.** It stayed zero, so SteamVR's
   extrapolation was purely first-order — during angular/linear acceleration it
   systematically lags. OpenHMD has always exposed `OHMD_ACCELERATION_VECTOR`;
   `driver_openhmd.cpp` simply never read it.

**This also means in-driver photon prediction would have been the wrong fix.**
Predicting in the driver *and* letting SteamVR predict would double-count and
overshoot. That trap is why the change below defaults the in-driver predictor OFF.

## Changes

**OpenHMD** (`subprojects/openhmd`)

- `rift-tracker.c`: record `last_imu_local_ts` (host-clock arrival of each IMU
  sample) and expose `rift_tracked_device_get_pose_age_ns()`.
- `openhmd.h`: new `OHMD_POSE_AGE_SECONDS = 27` float property — age in seconds of
  the pose currently reported by `OHMD_ROTATION_QUAT` / `OHMD_POSITION_VECTOR`.
  Appended at the end of the enum, so ABI is preserved.
- `rift.c`: serve the new property for both HMD and Touch.
- `rift-tracker.c`: `rift_predict_pose()` — dead-reckons a pose forward by dt.
  Orientation integrates the **device-local** angular velocity, so the delta
  quaternion right-multiplies (the form Oculus used in SDK 0.3.2's
  `SensorFusion::GetPredictedOrientation`); position uses the world-frame velocity
  and acceleration: `p += v·dt + a·dt²/2`, `v += a·dt`.
  Gated by **`OHMD_RIFT_PREDICT_MS`, default 0 (off)**.

  > Do **not** enable this under SteamVR — it double-predicts on top of SteamVR's
  > own extrapolation. It exists for A/B measurement and for API consumers that do
  > no prediction of their own (`openhmd_pose_log`, `openhmd_simple_example`).

- `examples/poselog/poselog.c`: new `openhmd_pose_log` target — high-rate CSV of
  pose + velocity + angular velocity + acceleration + pose age. Kept separate from
  `openhmd_simple_example` so the existing stationary harness is untouched. Writes
  to a file, not stdout, because OpenHMD's own logging goes to stdout and would
  interleave into the rows.

**SteamVR driver** (`driver_openhmd.cpp`)

- `poseTimeOffset = -pose_age` instead of 0, for HMD and Touch.
- `vecAcceleration` populated from `OHMD_ACCELERATION_VECTOR`, for HMD and Touch.

## Validation

- **Predictor math**: unit-tested against a closed-form trajectory (constant
  body-frame angular rate + constant world acceleration). Error vs the analytic
  answer is 0.0 to floating point, and the zero-ω branch is a verified no-op (the
  `1/|ω|` guard holds).
- **Pose-age plumbing**: live on hardware, reports 1.00 ms median / 1.05 ms p95 —
  a real, stable value where SteamVR previously received 0.
- **At-rest regression**: over a 90 s stationary log, a 22 ms extrapolation adds
  only 0.021° / 0.16 mm of jitter (vs 0.003° / 0.08 mm unpredicted). Prediction
  always costs a little noise at rest — Oculus pays the same tax — and this is an
  order of magnitude below their *moving* predictor error, which says our
  velocities are clean.
- **Deploy**: `install_files_to_build.sh` run, and `build/driver_openhmd.so.0.0.1`
  vs `build/bin/linux64/driver_openhmd.so` checksums verified equal, so SteamVR
  loads the new driver (this is the trap that silently invalidated the 2026-07-06
  wear test).

## RESULT (2026-07-12, `captures/lin/2026-07-12/motion2.csv`)

100 s, 250 Hz, 98.4 % optical lock, 52 % of samples moving (median |ω| 0.71 rad/s,
p95 4.03, peaks to 17.3). Scored at a 22 ms horizon, tracked & moving:

| | our stack | Oculus runtime |
|---|---|---|
| **rot, predicted** | **0.127° med / 0.488 p95** | 0.190° med / 0.510 p95 |
| rot, zero-pred | 2.626° med / 6.295 p95 | 1.980° med / 3.860 p95 |
| **pos, predicted** | **3.81 mm med / 50.0 p95** | 2.20 mm med / 4.70 p95 |
| pos, zero-pred | 6.75 mm med / 48.0 p95 | 6.00 mm med / 15.30 p95 |

**Rotation prediction is done — we are at parity with the Oculus runtime**
(0.127° vs their 0.190° median; we were exercising the headset harder, hence the
larger zero-pred baseline). A ~20× reduction in orientation error, from the
poseTimeOffset + acceleration fixes alone.

**Position tells a different story, and it is not a prediction problem.** Our p95
is ~50 mm *whether or not we predict* (50.0 predicted vs 48.0 unpredicted), while
Oculus sits at 4.7/15.3 mm. An error that prediction neither creates nor removes
is not an extrapolation failure — the underlying position estimate is
discontinuous. Confirmed directly: subtracting the velocity-explained motion from
each 4 ms position step leaves

```
p50   -0.00 mm      p99    35.15 mm
p90    0.47 mm      p99.9 117.46 mm     max 376.54 mm
```

i.e. **541 unexplained jumps > 5 mm (5.4/s, against a ~52 Hz camera rate — roughly
one optical update in ten snaps the position)**, some by tens of centimetres. The
median is clean, so this is a tail of discrete teleports, not noise.

### Root cause (resolved 2026-07-12, later the same day — it went much deeper)

The "stale calibration" hypothesis was only the surface. The full chain, in
discovery order:

1. The room config's **world frame was ~154° off gravity** (inherited from a July
   solve). `WMTD3052400VZL` never passed the tracker's gravity gate under it —
   the rig had been effectively **single-camera for weeks**; the other sensor's
   flipped entry cancelled the flipped world and tracked alone, self-consistently.
2. The gravity gate itself was **roll-broken** (swing-twist about a wrong-frame
   axis; up to ~90° phantom error for rolled cameras — proven synthetically).
   Fixed in `correspondence_search.c`; `OHMD_RIFT_NO_GRAVITY_GATE=1` for A/B.
3. Recalibrated gravity-true via `calibrate_room.py setup`: extrinsics 2.26 mm /
   0.33° median, both sensors verifying at ~52/s, co-observed 46/s (was 0).
4. Which exposed the actual jump source: **the OVR fusion port is not
   multi-camera-safe**. With ~104 corrections/s from two sensors its velocity
   state rings (0.2–0.6 m/s while stationary; 5–60 mm displayed steps at every
   correction). `OHMD_RIFT_FUSION=ukf` (multi-sensor-native) is 2–4× cleaner and
   is the recommended backend until the OVR port's vision gains are made
   N-sensor-aware.

### Final validated numbers (motion-final.csv, 2026-07-12 evening)

After the multi-camera fix (position-error blending in `rift-fusion-ovr.c`
`vision_fix()` — position only; orientation keeps overwrite semantics, see the
comment there for why), full-motion validation (86 % moving, |ω| peaks
11.4 rad/s), 22 ms horizon, tracked & moving:

| | ours | Oculus runtime |
|---|---|---|
| rotation, predicted | **0.097° med / 0.280° p95** | 0.190° / 0.510° |
| position, predicted | **3.52 mm med / 7.54 p95** | 2.20 mm / 4.70 |
| orientation thrash | 0.03 /s | — |
| position jumps >5 mm | 0.17 /s (p99 1.45 mm) | — |

Rotation beats the Oculus runtime by ~2×; position is within 1.6× at higher
motion intensity (our zero-pred baseline 10.4 mm vs their 6.0). The morning's
starting point on the same rig was 5.4 position teleports/s of up to 377 mm.

## Appendix: the earlier stationary log

The validation log came back with the headset **stationary** (peak |ω| 0.06 rad/s;
zero samples above the 0.5 rad/s cut used for scoring). Prediction is trivial at
rest, so this says nothing about the case that matters.

To close it — the headset must actually be moved, facing the sensors:

```bash
~/git/SteamVR-OpenHMD/build/subprojects/openhmd/openhmd_pose_log \
    captures/lin/$(date +%F)/motion.csv 90 250
# ... pick the headset up: slow yaw sweeps, pitch/nod, brisk shakes, walk around.
# Keep it pointed at the sensors — no optical lock means the log is void.

python3 tools/score_prediction.py captures/lin/$(date +%F)/motion.csv
```

`score_prediction.py` scores our prediction offline from that single log (predict
each sample forward 22 ms using the velocities logged with it; compare against the
pose actually observed 22 ms later) and prints the Oculus runtime's numbers on the
same metric and horizon for direct comparison:

```
their predictor : pos median 2.20 mm  p95  4.70 mm | rot median 0.190 deg  p95 0.510 deg
zero prediction : pos median 6.00 mm  p95 15.30 mm | rot median 1.980 deg  p95 3.860 deg
```

If our moving numbers land near theirs, prediction is done. If the *predicted*
error stays high while the zero-prediction error looks normal, the problem is
velocity quality rather than prediction plumbing — and the contested angular
velocity frame (`OHMD_RIFT_ANGVEL_FRAME`, see the driver-mods notes) becomes the
next suspect.
