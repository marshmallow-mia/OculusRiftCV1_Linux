# Capture-day checklist (from cv1-capture-plan.md)

Print this. Work top to bottom. Every box in §A must be ticked before any capture
counts — plan §0: *the whole plan is void without rig discipline*.

## A. Rig discipline — BEFORE anything else

- [ ] Windows test PC physically moved next to the rig (PC comes to the rig,
      **never** the rig to the PC)
- [ ] Space/power/cable-length confirmed (this was the one open blocker, plan §8)
- [ ] Tripod feet marked on the floor with tape (both sensors)
- [ ] Sensors bolted/taped; tilt + aim untouched from here on
- [ ] Headset rest position tape outline on the desk
- [ ] Turntable / pan head with degree marks in place, position taped
- [ ] 1 m square marked on floor with tape (4 corners), sides measured with a
      real tape measure and noted below
- [ ] Lighting noted (blinds? lamps?) — must be identical for Win + Lin halves
- [ ] Photos of everything → `captures/rig/` (+ a `notes.md` with date,
      measured square sides, lighting, cable routing)
- [ ] Active USB extension cables travel with the sensors when replugging

Measured square sides (m): N: ____ E: ____ S: ____ W: ____ diag: ____ / ____

## B. Windows PC readiness (can be done before moving it)

- [ ] Oculus PC app installed and CV1-capable
      (fallback installer of known vintage: Meta Horizon v32.1.1 bundle at
      `/run/media/mia/New Volume/Meta Horizon/` on the Linux box — client only;
      runtime/service + Oculus setup still required)
- [ ] Oculus setup completed once (floor level etc.) — origin type is recorded
      by ovr_static_dump, absolute positions live in *their* frame (plan §8)
- [ ] LibOVR SDK downloaded, `OVR_SDK` env var set
- [ ] `tools\win\build.bat` run → `ovr_static_dump.exe` + `ovr_pose_log.exe` exist
- [ ] Smoke test: `ovr_static_dump.exe` prints JSON with `"tracker_count": 2`
- [ ] USBPcap installed (for §4a IMU sniff)
- [ ] ≥ 15 GB free disk (10 GB Lin recordings + margin; +2 GB if §4b attempted)
- [ ] **USB3 check**: Oculus setup tool reports BOTH sensors at USB3 / full
      60 Hz. If USB2 fallback → note it; Tier 0 still valid, Tier 1 confounded
      (plan §0). USB3 status: sensor A: ____ sensor B: ____

## C. Capture order (plan §7)

### 1. Tier 0 — static extrinsics (GATE)

- [ ] Plug sensors + HMD into Windows PC (same cables, tripods untouched)
- [ ] Headset on the taped desk spot
- [ ] `ovr_static_dump.exe captures\win\<date>\ovr_static.json`
- [ ] Copy `rift-room-config.json` from the Linux box
- [ ] `python tools\compare_extrinsics.py captures\win\<date>\ovr_static.json rift-room-config.json`
- [ ] **GATE: relative rotation ≤ 0.5° and baseline ≤ 5 mm.**
      FAIL → STOP. Fix room calibration, re-baseline, only then continue.

### 2. Firmware / IMU parse diff (§2 + §4a, same session)

- [ ] Start USBPcap on the HMD's HID interface (USB2 device, VID 2833 PID 0031)
- [ ] Capture 60 s while the Oculus runtime is tracking
      → `captures\win\<date>\imu_60s.pcap`
- [ ] In parallel: `ovr_pose_log.exe pose_imu_now.csv pose_imu_pred.csv 60`
- [ ] Note in `notes.md`: keep-alive rate, any feature reports written by the
      runtime (LED pattern / exposure sync config — "quietly the most
      interesting part", plan §4a)

### 3. Tier 1 — motion protocol, Windows half

For each test: `ovr_pose_log.exe pose_t<N>_now.csv pose_t<N>_pred.csv <dur>`

| # | test | dur | done |
|---|------|-----|------|
| 1 | dead still, taped desk spot, facing both sensors | 60 s | [ ] |
| 2 | still on tripod at head height | 60 s | [ ] |
| 3 | slow 360° yaw on turntable, ~20 s/rev, 2 revs | 40 s | [ ] |
| 4 | pitch sweep ±45°, then roll sweep ±45°, slow | 30 s | [ ] |
| 5 | fast yaw shake ~2 Hz, then nod | 20 s | [ ] |
| 6 | walk the taped 1 m square, headset facing forward | 30 s | [ ] |
| 7 | occlude sensor A by hand 5 s, uncover, ×3 | 30 s | [ ] |
| 8 | step fully out of view 5 s, return, ×3 | 30 s | [ ] |
| 9 | both Touch controllers still, then figure-8s | 40 s | [ ] |

### 4. Tier 1 — Linux half (same day, same lighting; back-to-back per test
if practical — no reboot needed, just cable swap)

- [ ] Replug same cables into Linux box, tripods untouched
- [ ] Repeat table above with `tools/lin_pose_log.py --out pose_t<N>.csv`
      (backend `openvr` for the SteamVR-facing pose)
- [ ] If the recorder (§4c, `OHMD_RIFT_RECORD`) is built by then: record full
      sessions for tests 1, 3, 5, 6 → `captures/lin/<date>/`

### 5. Compare

- [ ] `python tools\compare_dynamic.py --test still pose_t1_now.csv lin_t1.csv` … etc.
- [ ] Prediction gap: diff `pose_t5_now.csv` vs `pose_t5_pred.csv` (sizes §6)

## D. Explicitly deferred

- Camera-stream USB capture (§4b): only if Tier 1 points at the optical
  front-end. One sensor, 20–30 s, straight to SSD.
- ChArUco ground truth (§5): only for tests Tier 1 flags divergent.

## E. Known confounders to log, not forget

- Our driver does **no forward prediction** (`poseTimeOffset` unset in
  `driver_openhmd.cpp`; FIXME in `rift-kalman-6dof.c` ~l.922). Every dynamic
  delta is partly this. The now-vs-pred CSVs size it exactly (plan §6).
- Oculus absolute positions are in their recenter frame — only gauge-free
  invariants are comparable (plan §2).
