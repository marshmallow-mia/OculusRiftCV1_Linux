# Session notes — 2026-07-12 (prep, no valid captures yet)

## Machine state (this Windows test PC)

- Meta Horizon client **v32.1.1** installed (the known-good CV1 vintage from
  plan §8), oculus-drivers 1.77.0. `OVRService` running.
- Runtime path: `C:\Program Files\Meta Horizon\Support\oculus-runtime\`
  (`LibOVRRT64_1.dll` present — our tools load it via the CAPI shim).
- LibOVR reports `runtime_version 1.4.0` via `OVR_MAJOR/MINOR/PATCH` (that is
  the *header* version of the vendored SDK 1.40 mirror, not the runtime).

## Hardware seen by the runtime (probe run, today)

- HMD: **Oculus Rift CV1**, serial `WMHD316C1006SW`, firmware 7.9 — connected.
- **2 trackers connected.** `pose_valid=false` on both: runtime idle, HMD not
  worn. `ovr_static_PROBE_ONLY_invalid_poses.json` records this probe — the
  poses in it are placeholders `(0,0,-1)`, NOT extrinsics. Do not feed it to
  compare_extrinsics.py (it skips nothing — it checks `connected`, not
  `pose_valid`; the real capture must show `pose_valid: true`).
- Tracking origin currently **EyeLevel** → height comparison in Tier 0 will be
  meaningless unless origin is switched to FloorLevel (redo Oculus floor setup
  or ovr_SetTrackingOriginType) — gauge-free invariants unaffected either way.

## To make Tier 0 valid (the only missing step)

1. Put HMD on its taped desk spot, facing both sensors.
2. Cover the proximity sensor (tape/cloth between the lenses) so the runtime
   treats it as worn and starts tracking.
3. `D:\tools\win\ovr_static_dump.exe D:\captures\win\2026-07-12\ovr_static.json 30`
   — the wait loop prints `[present/mounted/visible]` flags; all trackers must
   reach pose_valid before the dump counts.
4. Copy `~/.config/openhmd/rift-room-config.json` from the Linux box to D:\
   then: `python D:\tools\compare_extrinsics.py D:\captures\win\2026-07-12\ovr_static.json D:\rift-room-config.json`

## Still outstanding on this PC

- USBPcap: installer staged at `D:\third_party\USBPcapSetup-1.5.4.0.exe` —
  needs admin install + **reboot** before §4a. Install before capture day.
- USB3 check for both sensors (Oculus app → Devices) — record result in
  CAPTURE-CHECKLIST.md §B.
- `rift-room-config.json` from the Linux box (needed for the Tier-0 compare).
- Sensor serials expected from plan §0: `WMTD3052400VZL`, `WMTD306Q701DEK` —
  verify these are the two the runtime sees (Oculus app → Devices).

## CRITICAL CONTEXT (added after capture)

**This Windows PC + sensors is a DIFFERENT physical environment from the Linux
rig.** The plan §0 rule (same tripods, same placement) is not satisfied.
Consequences:

- `ovr_static.json` / `ovr_static_floor.json` describe THIS room's sensor
  placement only (baseline 1.279 m, heights 0.886/1.114 m, rel rot 25.2°).
  They must NOT be compared against `rift-room-config.json` — the 1.28 vs
  2.36 m baseline difference is the difference between the two rooms, not a
  calibration error. Tier 0 as designed is DEAD in this configuration.
- Still fully valid from this machine (environment-independent):
  - `setup_hid.pcap` — firmware config blobs, LED/exposure/radio writes (§2/§4a)
  - IMU byte-level parse validation from any tracking-session pcap (§4a)
  - prediction-window sizing: now-pose vs pred-pose CSVs (§6)
  - qualitative runtime behaviour: PSD shape, optical step handling,
    re-acquisition strategy, velocity smoothing (§3 — but numbers are
    geometry-dependent; only character, not magnitudes, transfers)
- To resurrect Tier 0: the Windows PC must be brought to the Linux rig and
  the sensors plugged over without touching tripods (plan §0), OR accept the
  §8 fallback (ChArUco ground truth on the Linux rig only).

## §4a capture pair (done) + prediction-logging fix

- `imu_tracking.pcap` (27 MB): HMD HID traffic while the runtime tracked
  (headset worn ~45% of a 120 s window, real motion: |w| p95 2.6 rad/s).
  Companion pose log: `pose_imu_now.csv` (valid).
- `pose_imu_pred.csv` from this run is INVALID — ovr_GetPredictedDisplayTime
  needs a frame-submitting session. ovr_pose_log.exe was rebuilt to predict an
  explicit horizon instead (default +22 ms, 4th CLI arg in ms). Pred-file
  t_ovr = t_now + horizon; comparing pred pose vs the now pose logged at that
  later time measures predictor accuracy.

## Prediction sizing result (§6) — pose_pred22_*.csv, 22 ms horizon

Scored with tools/analyze_prediction.py (predicted pose vs pose actually
observed 22 ms later, from their own tracking output; 40212 rows, ~53% worn):

|                      | their predictor      | zero prediction (= our driver today) |
|----------------------|----------------------|--------------------------------------|
| tracked & moving pos | 2.2 mm med / 4.7 p95 | 6.0 mm med / 15.3 p95                |
| tracked & moving rot | 0.19° med / 0.51 p95 | 1.98° med / 3.86° p95                |

Conclusion: at 22 ms photon latency, NOT predicting costs ~2° median
orientation error during ordinary head motion — an order of magnitude larger
than what their predictor leaves behind. This is the single biggest known
gap and it confounds every dynamic comparison until fixed
(driver_openhmd.cpp poseTimeOffset / rift-kalman-6dof.c ~l.922 FIXME).

Note: ovr_pose_log prints status to stderr, which the session UI may render
as an error — benign. Reading its CSVs while it runs yields stale sizes /
truncated tails (NTFS metadata); wait for completion.
