# Handoff: resume on the Linux PC

Written 2026-07-12 at the end of the Windows capture session. Everything on
this drive is portable — copy the whole tree (you can skip `third_party/`,
it's only needed to rebuild the Windows exes).

## What happened on the Windows test PC (this machine)

- Meta Horizon client v32.1.1 + runtime 1.77, OVRService working. CV1
  `WMHD316C1006SW` (fw 7.9) + 2 sensors connected and tracking after full
  Oculus setup (incl. floor level).
- Capture tools built (`tools/win/*.exe`, MSVC + vendored LibOVR 1.40 shim,
  see `tools/win/build.bat`) and used.

**IMPORTANT CONSTRAINT discovered during the session:** this Windows setup is
a *different physical environment* than the Linux rig. Plan §0's same-rig rule
is not satisfied, so **Tier 0 (extrinsics comparison) and quantitative Tier 1
are invalid between these captures and the Linux rig.** Do not diff
`ovr_static*.json` against `rift-room-config.json`. All banked captures below
are environment-independent and remain valid.

## Artifacts in `captures/win/2026-07-12/` (see notes.md there for detail)

| file | what it is |
|---|---|
| `setup_hid.pcap` (92 MB) | HMD USB traffic (devices: Rift hub+HID+audio, sensors excluded) during the FULL Oculus setup — contains the runtime's HID feature/command writes: LED patterns, exposure/sync config, radio pairing. Plan §4a's "quietly the most interesting part". |
| `imu_tracking.pcap` (27 MB) | Same devices during ~2 min of real tracked wear — raw 1 kHz IMU HID reports for byte-level parse validation. |
| `pose_imu_now.csv` | 500 Hz Oculus head+controller pose log during the pcap above (t_wall aligned within the file; ~45% worn). |
| `pose_pred22_now/pred.csv` | The §6 prediction dataset (22 ms horizon). |
| `ovr_static_floor.json` | This room's solved sensor extrinsics, FloorLevel origin — reference for "what a healthy solve looks like" only. |

## Measured result (plan §6) — drives the next code change

At 22 ms photon latency, during ordinary head motion (their own output as
truth; `tools/analyze_prediction.py`):

- their predictor:  0.19° / 2.2 mm median error
- zero prediction (our driver today): 1.98° / 6.0 mm median error

→ **Forward prediction is the single biggest known gap (10×).** Fix before
any dynamic comparison: `poseTimeOffset` never set in `driver_openhmd.cpp`,
FIXME at `rift-kalman-6dof.c` ~line 922.

## Next steps on the Linux box (plan §7 order, adjusted)

1. ~~**§4a analysis**~~ — **DONE 2026-07-12, see `findings-4a-hid.md`.**
   Tools: `tools/analyze_hid_pcap.py`, `tools/probe_sensor_config.py`.
   Outcome: three suspects eliminated, none of them the bug.
   - LED/exposure/camera-sync config: **byte-identical** to ours. Ruled out.
     (Also: the camera cadence is ~52 Hz, not 60 — plan corrected.)
   - IMU layout/scale/sign-extension: **validated** against the runtime's own raw
     stream (gravity lands at 9.47 m/s²). Ruled out.
   - On-HMD auto-calibration: the CV1 firmware **drops** the flag, so OpenHMD's
     `SETFLAG`s at `rift.c:1157-1158` are no-ops. No two-estimator conflict.
     Ruled out (tested on hardware, not just read from source).
   - Remaining unknowns: reports `0x0d` / `0x21`, which the runtime writes and we
     have never heard of. Low priority.
   → **Forward prediction (§6) is now the only confirmed large gap.**
2. **§4c recorder** — widen `OHMD_RIFT_CAL_CAPTURE` in
   `rift-sensor-pose-search.c` into `OHMD_RIFT_RECORD=<dir>` (imu.jsonl,
   frames/, optical.jsonl, fusion.jsonl) + a no-USB replay entry point.
3. **§6 prediction** — implement, then re-measure our stack with
   `tools/lin_pose_log.py` and score with `tools/analyze_prediction.py`
   (works on any now/pred CSV pair in this schema).
4. **§5 ground truth** (ChArUco board + external webcam) on the Linux rig —
   now the primary calibration check, since Tier 0 is parked.
5. **Tier 0/1 proper** — only if/when this Windows PC can be brought to the
   Linux rig (plan §0: rig never moves, PC does; same cables).

## Tools (all in `tools/`, Linux-usable except win/)

- `lin_pose_log.py` — our-stack logger, same CSV schema (openvr / openhmd
  backends; needs `pip install openvr numpy`).
- `compare_extrinsics.py` — gauge-free Tier-0 gate (parked until same-rig).
- `compare_dynamic.py` — Tier-1 metric tables (`--test still|yaw360|...`).
- `analyze_prediction.py` — predictor scoring (the §6 result above).
- `quick_invariants.py`, `quick_poselog_check.py` — sanity helpers.

Integrity: `captures/win/2026-07-12/SHA256SUMS` — verify after copying with
`sha256sum -c SHA256SUMS` in that directory.

## Open questions carried over

- USB3 vs USB2 status of the two sensors on this PC (not yet recorded).
- Sensor serials on this PC vs plan §0 (`WMTD3052400VZL`, `WMTD306Q701DEK`)
  — if these are DIFFERENT sensors, the firmware-blob diff still holds, but
  say so in notes.
- Whether the Windows PC can eventually reach the Linux rig (revives Tier 0).
