# Plan: comparing our CV1 tracking against the Oculus Windows runtime

Goal: find out *where* our tracking pipeline diverges from the Oculus PC runtime on
**this exact rig**, not just that it feels worse. The output of this plan is a set of
captured datasets plus a comparison script that answers, per stage: is our camera
calibration wrong, is our optical pose wrong, is our fusion wrong, or is our SteamVR
export wrong?

## 0. The rig (must not change between captures)

Two CV1 sensors, currently calibrated in `~/.config/openhmd/rift-room-config.json`:

| sensor serial     | our pos (m)                    | our orient (quat xyzw)                 |
| ----------------- | ------------------------------ | -------------------------------------- |
| `WMTD3052400VZL`  | (-1.072217, 1.642630, 1.109873)| (0.109617, 0.890406, -0.073757, 0.435570) |
| `WMTD306Q701DEK`  | ( 1.191647, 1.443214, 1.756374)| (see room-config)                      |

USB IDs: VID `2833`, HMD PID `0031` (HID, USB2, interrupt IN — IMU + config blobs),
sensor PID `0211` (UVC, USB3, isochronous — 1280×960 GRAY8 ≈ 74 MB/s each). The LED /
camera cadence is **~52 Hz**, not 60: the HMD's TRACKING_CONFIG period is 19200 µs
(52.08 Hz), and exposure events in the IMU stream measure 51.36 Hz (see
`findings-4a-hid.md` §1). Earlier drafts of this plan said 60 Hz — that was wrong.

**Two machines, one rig.** The Windows captures happen on a separate test PC, not a
dual-boot. That is fine — *better*, even (no reboot, so we can alternate Windows and
Linux captures within one afternoon under identical lighting) — but it forces one rule:

> **The Windows PC comes to the rig. The rig never goes to the Windows PC.**

Physically: wheel/carry the Windows box next to the existing setup, unplug the two
sensor USB cables and the HMD from the Linux machine, plug those *same cables* into the
Windows machine. The tripods are not touched, not re-aimed, not re-levelled. Everything
in Tier 0 (§2) is a comparison of *where the cameras physically are*; the moment a
tripod shifts, that comparison is measuring the shift and nothing else.

If the Windows PC genuinely cannot be brought into the room, Tier 0 is dead (there is no
way to reproduce sensor placement to sub-degree accuracy in another room) — and Tier 0 is
the most likely root cause. In that case, say so early and re-plan; don't quietly run the
rest and pretend the numbers mean something.

**Rig discipline — the whole plan is void without it:**

- Sensors stay bolted/taped down for the entire exercise. Mark the tripod feet on the
  floor with tape. Do not adjust tilt or aim when swapping the USB cables over.
- Mark one repeatable headset rest position on the desk (tape outline) for the static test.
- Build/borrow a rotation reference: a lazy-susan or tripod pan head with degree marks,
  so "slow 360° yaw" is the same motion on both machines.
- Mark 4 floor positions of a 1 m square with tape for the walk test.
- Lighting and IR-reflective clutter unchanged (no sun through the window on one run and
  not the other). With no reboot in the loop, run the Windows and Linux halves of each
  test back to back so this is nearly free.
- The CV1 sensors are famously fussy about USB3 host controllers. Check the Windows PC
  actually runs both sensors at full rate on USB3 (Oculus's own setup tool reports this)
  before trusting any capture. If it falls back to USB2, note it — the geometry
  comparison (Tier 0) still holds, but the dynamic comparison (Tier 1) is confounded and
  must be flagged.
- Bring the active USB extension cables with the sensors if they're in use; swapping to
  different-length or unpowered cables can change whether a sensor enumerates at USB3.

Take photos of the room before starting; they go in `captures/rig/`.

## 1. What Windows can and cannot give us

The Oculus runtime is closed; there is **no intermediate state** to read (no per-frame
blob list, no per-camera optical pose, no filter internals). Concretely:

- **Available:** `ovr_GetTrackingState` (head + hand pose, linear/angular velocity,
  linear/angular acceleration, status flags), `ovr_GetTrackerPose` (per-sensor pose,
  frustum, connected/pose-valid flags), `ovr_GetTrackerCount`,
  `ovr_GetTrackingOriginType`, `ovr_GetSessionStatus`, `ovr_GetPerfStats`.
- **Not available:** raw IMU samples (SDK 0.5's `ovrSensorData RawSensorData` was
  removed in 1.x, and CV1 requires 1.x), camera frames, optical poses, filter state.
- **Only via USB sniffing:** the raw HID/UVC bytes on the wire.

Two hard consequences:

1. Both stacks can never see the same input simultaneously — only one OS owns the USB
   devices at a time. So dynamic comparisons are **statistical over a repeated motion
   script**, not sample-by-sample.
2. Sniffing the *camera* stream on Windows is low value anyway: the sensor is a UVC
   camera and emits the same pixels regardless of OS. What differs is what each stack
   *does* with those pixels. So we capture frames on **Linux**, where we can capture
   them cheaply and correlate them with our own internals.

That leaves Windows responsible for three things: **static calibration ground truth**,
**output behaviour targets**, and **a sanity check of our IMU byte parsing**.

## 2. Tier 0 — static calibration ground truth (highest value per hour; do first)

`ovr_GetTrackerPose()` returns Oculus's own solved extrinsics for our exact physical
sensor placement. If our room calibration is off by even a degree per camera, the two
cameras disagree about where the headset is, the fusion fights itself, and no amount of
filter tuning will ever fix it. This is the single most likely root cause and the
cheapest to test.

**Gauge freedom — read this before comparing numbers.** Oculus's tracking origin (and
yaw) is set by their recenter/floor-level convention; ours is set by our room
calibration. The two extrinsic sets are therefore only comparable **up to a global rigid
transform**. Do **not** compare `pos`/`orient` element-wise. Compare the invariants:

- the **sensor-to-sensor relative transform** `T_A→B = T_A⁻¹ · T_B` (fully gauge-free —
  this is the sharp test);
- the **baseline distance** ‖pos_A − pos_B‖;
- the **relative optical-axis angle** between the two cameras;
- the sensor **heights above the floor**, if Oculus reports a floor-level origin.

Acceptance: relative rotation within ~0.5°, baseline within ~5 mm. Anything worse and
our room calibration is the bug — stop and fix that before touching fusion.

**Also capture (same session):** the HMD's firmware config blobs — LED model / position
map, IMU gyro+accel bias and scale matrices, and the sensor intrinsics. These come from
device firmware over HID, so *both stacks read identical bytes*; any disagreement is
purely our parsing (`packet.c`: `unpack_3x21bit`, `gyro_matrix`, `gyro_scale`). Dump
them from the Windows USB capture and diff against what our driver parses.

**Deliverables**
- `tools/win/ovr_static_dump.exe` — ~120 lines of C against LibOVR. Prints, as JSON:
  tracker count, each tracker's pose + frustum + flags, tracking-origin type, HMD serial,
  runtime version. One run, headset on, sensors idle.
- `tools/compare_extrinsics.py` — eats that JSON + `rift-room-config.json`, prints the
  gauge-free invariants side by side.

## 3. Tier 1 — output behaviour under a scripted motion protocol

Poll `ovr_GetTrackingState(session, ovr_GetTimeInSeconds(), ovrFalse)` — note
`latencyMarker=false` and prediction time = **now**, so we log the runtime's *current*
estimate, not a predicted one — as fast as the loop allows (~1 kHz), to CSV. Then repeat
the identical script on Linux with our stack and compare.

Log a second, parallel CSV with prediction enabled (`ovr_GetPredictedDisplayTime`) so we
can *measure how far ahead they predict* — we currently do not predict at all (§6), and
the delta between their now-pose and their predicted-pose tells us the size of that gap.

### The motion protocol (run identically on both OSes)

| # | test | duration | what it isolates |
|---|------|----------|------------------|
| 1 | Headset dead still on the taped desk spot, facing both sensors | 60 s | static noise floor, gravity sway, optical step size |
| 2 | Same, but with the HMD *off* the desk on the tripod at head height | 60 s | noise at realistic geometry |
| 3 | Slow 360° yaw on the turntable, ~20 s/rev, 2 revs | 40 s | yaw drift, LED-ID handoff between sensors |
| 4 | Pitch sweep ±45°, then roll sweep ±45°, slow | 30 s | gravity/tilt correction behaviour |
| 5 | Fast head shake (yaw ~2 Hz), then nod | 20 s | prediction, latency, filter overshoot |
| 6 | Walk the taped 1 m square, headset held facing forward | 30 s | position accuracy + scale |
| 7 | Occlude sensor A by hand for 5 s, uncover, repeat ×3 | 30 s | single-sensor degradation |
| 8 | Step fully out of view 5 s, return, ×3 | 30 s | re-acquisition time and snap magnitude |
| 9 | Both Touch controllers held still, then figure-8s | 40 s | controller path (our known-weak ω frame) |

### Metrics the comparison script computes for each test

Stationary: per-axis std of pitch/roll/yaw and position; peak-to-peak; PSD (does their
noise sit at the ~52 Hz camera rate like ours does?). Dynamic: yaw drift per revolution;
position RMS error against the taped square's known geometry; **step magnitude at optical
updates** (the discrete-jump artefact our correction-bleeding was written to hide — do
they show any?); re-acquisition latency and snap size; velocity/accel field magnitudes
(reveals their smoothing and prediction).

**Deliverables**
- `tools/win/ovr_pose_log.exe` — 1 kHz CSV logger, columns:
  `t_ovr, t_wall, px,py,pz, qx,qy,qz,qw, vx,vy,vz, wx,wy,wz, ax,ay,az, alx,aly,alz, status, hand0..., hand1...`
- `tools/lin_pose_log.py` — same schema out of our stack (extend `riftcv1/pose_test.py`,
  which already parses `openhmd_simple_example` output, or read via OpenVR so we also
  capture the SteamVR-facing pose).
- `tools/compare_dynamic.py` — the metric table above, both stacks side by side.

## 4. Tier 2 — raw stream capture

### 4a. HMD IMU over USB (Windows) — easy, do it

The HMD is a USB2 HID device on interrupt IN: 1000 Hz, ≤3 IMU samples per report, 64-byte
reports ≈ 64 KB/s. USBPcap handles this fine. Capture 60 s while the runtime tracks, in
parallel with a Tier-1 pose log.

What it buys us:
- **Byte-level validation of our IMU parse** (`packet.c` `decode_sample`,
  `unpack_3x21bit`, `gyro_scale`, `gyro_matrix`) against the same reports.
- **Timestamp semantics**: how the device's sample counter/timestamp advances, whether
  reports drop, how many samples per report in practice — the ground for our
  camera-exposure↔IMU alignment, which is a prime suspect for wobble.
- The command/feature reports the Oculus runtime *writes* to the HMD (keep-alive rate,
  LED pattern/brightness, exposure/sync configuration). **This is quietly the most
  interesting part**: if their LED and exposure sync configuration differs from ours,
  our blob detection is working on worse input than theirs and everything downstream
  inherits it.

### 4b. Camera stream (Windows) — probably skip

1280×960 GRAY8 at ~52 Hz over USB3 isochronous is ~64 MB/s per sensor, and USBPcap's USB3
+ isochronous support is unreliable. Since the pixels are OS-independent anyway (§1), the
cost/benefit is bad. **Attempt only if** the Tier-1 comparison points at the optical
front-end and we need their frames to prove ours are equivalent. If we do try it: one
sensor only, 20–30 s, straight to an SSD (~2 GB).

### 4c. Our own recorder (Linux) — build regardless

Widen the existing capture hook (`OHMD_RIFT_CAL_CAPTURE` in `rift-sensor-pose-search.c`,
which today only writes LED-ID-verified poses) into a full session recorder:
`OHMD_RIFT_RECORD=<dir>` writes

- `imu.jsonl` — raw HID report bytes + host timestamp + decoded accel/gyro,
- `frames/NNNNNN.pgm` (or a raw blob file) + `frames.jsonl` — per-frame sensor id,
  exposure timestamp, arrival timestamp,
- `optical.jsonl` — per-frame blobs, LED-ID assignment, PnP pose, reprojection error,
  `obs_scale`,
- `fusion.jsonl` — filter state in/out per update, and the pose actually exported to
  SteamVR (including the correction-bleed offset).

Plus a **replay** entry point that feeds a recorded session back through the pipeline
with no USB, so we can A/B fusion changes deterministically on identical input. This is
the workhorse: once it exists, every future tuning change is measurable in seconds
instead of requiring a wear test.

## 5. Independent ground truth (so we know who is *right*, not just who differs)

Both stacks can be wrong. Cheapest usable truth for this rig:

- Print a ChArUco board, tape it to the wall in view of a fixed external webcam; rigidly
  mount the board (or a second, small board) to the headset strap; solve the board pose
  per frame with OpenCV. Gives ~mm / ~0.2° truth at 30 Hz for tests 1–6, and requires no
  Windows at all.
- Poor man's fallback: the taped 1 m floor square and the degree-marked turntable already
  give us absolute distance and absolute angle checks with zero extra hardware.

## 6. The thing to fix regardless of any of this

Our driver does **no forward prediction**: `poseTimeOffset` is never set in
`driver_openhmd.cpp`, and there is a `FIXME` at `rift-kalman-6dof.c` ~line 922. Oculus
predicts to photon time. This produces motion-proportional lag/judder that will show up
in — and confound — every dynamic comparison above. Either close it or measure it
(Tier-1's now-pose vs predicted-pose logs size it exactly) **before** trusting any
dynamic result.

## 7. Order of work

1. Rig discipline + photos (§0). Nothing else is valid without it.
2. Tier 0 static extrinsics (§2). **Gate:** if the sensor-to-sensor invariant disagrees
   by >0.5° / >5 mm, stop — fix room calibration, then re-baseline.
3. Firmware blob / IMU parse diff (§2, §4a).
4. Linux recorder + replay (§4c) — unlocks fast iteration for everything after.
5. Tier 1 motion protocol on both OSes (§3), with prediction sized (§6).
6. Ground truth (§5) for whichever tests Tier 1 flags as divergent.
7. Tier 2b camera capture (§4b) only if the optical front-end is still suspect.

## 8. Prerequisites / open questions

- **Resolved:** Windows captures run on a separate test PC (not a dual-boot). It must be
  physically brought to the rig — see §0.
- **Can the test PC physically reach the rig** (space, power, cable length)? This is now
  the one true blocker. If not, Tier 0 dies and we fall back to §5 ground truth only:
  still catches a bad room calibration and bad scale, but cannot tell us what Oculus does
  differently.
- **Does the test PC's Oculus PC app actually track CV1?** Needs a supported GPU and the
  last CV1-capable client. We have the Meta Horizon v32.1.1 bundle at
  `/run/media/mia/New Volume/Meta Horizon/` if we need an installer of known-good vintage
  (see the `meta-horizon-bundle` note) — but note that is the *client app*, and a working
  install also needs the runtime/service and a completed Oculus setup on that machine.
- **Do the Oculus setup and our setup agree on floor level?** Their tracking origin is
  whatever their guardian/floor setup established on the test PC. Not a problem for the
  gauge-free invariants (§2), but it does mean their absolute `pos` numbers are in their
  frame, not ours — do not read anything into the raw values.
- Windows toolchain for the two loggers: LibOVR SDK (free download) + MSVC or MinGW.
- USBPcap installed on the Windows side for §4a.
- Disk: budget ~10 GB for Linux recordings, ~2 GB if we attempt §4b.

## 9. Repo layout for the artifacts

```
captures/
  rig/                     photos, tape-position notes, date
  win/<date>/              ovr_static.json, pose_*.csv, imu_*.pcap
  lin/<date>/              recorder sessions (imu/frames/optical/fusion)
tools/
  win/ovr_static_dump.c    LibOVR extrinsics + config dump
  win/ovr_pose_log.c       1 kHz tracking-state logger
  lin_pose_log.py          same-schema logger for our stack
  compare_extrinsics.py    gauge-free sensor-geometry diff
  compare_dynamic.py       motion-protocol metric table
```
