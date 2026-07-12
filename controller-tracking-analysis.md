# Touch controller tracking: root-cause analysis (2026-07-12, evening)

## Symptom

Controllers "moving around weirdly" in SteamVR while the HMD (after the
same-exposure merge fix, see `rest-wander-analysis.md`) is clean.

First look (controllers at rest on desk, SteamVR rendered stream,
`ctrl-test2.csv`): catastrophic — valid only ~60% of the time, up to 11
teleports >50 mm per second, meter-scale position cloud. HMD in the same
capture: f2f p99 0.38 mm, zero jumps.

## Level-by-level findings (instrumented standalone captures, `ctrl-obs.jsonl` / scratchpad `ctrl-final.jsonl`)

1. **Optics are excellent when the LED ring faces the sensors**: 8 blobs,
   0.09 px reprojection, LED-ID-verified at full confidence, ~43 obs/s per
   controller, per-camera solutions stable to 0.5° / sub-mm over minutes
   (after ~5 s of acquisition churn).
2. **Cross-camera split, controller-sized**: the two cameras' world solutions
   for the same controller disagree by a constant ~86 mm / 4.2-4.7°
   (cf. HMD at head height: 0.4 mm / 0.22°; HMD on floor: 111 mm / 4.9°).
   Small 24-LED ring + steep desk view = large per-camera PnP bias. In-hand
   at chest height it will be smaller but still cm-level.
3. **Same-exposure merge extended to orientation and validated**: with the
   position+orientation merge (commit chain in openhmd checkout), the fused
   pose sits 4.8 mm from the two-camera midpoint (46 mm from either camera)
   and fast alternation ripple dropped 27-41 mm → 5 mm. Orientation merges
   only across same-exposure `orient_used` reports — nothing persists across
   exposures (that caused the historic thrash).
4. **Remaining disease A — constant ~8° fusion-vs-vision TILT conflict**:
   the fused orientation sits a steady ~8° from the optical solution, error
   axis mostly horizontal (tilt, not yaw). Accel-derived gravity and vision
   orientation disagree, so the tilt correction fights vision forever.
   Smoking-gun candidate: the driver prints the Touch (and HMD) IMU
   rectification matrices EMPTY (`gyro_calib = `, `accel_calib = ` in the
   startup log) — the flash calibration matrices are not decoded/applied.
   Touch accel_offset is large ([-0.06, -0.04, 0.39] m/s²), so unrectified
   axes plausibly tilt measured gravity by degrees.
5. **Remaining disease B — synchronized radio dropouts**: BOTH controllers
   lost all optical for the same 15.9 s window (and another 1.7 s), on both
   cameras, while at rest — the LED blink schedule (radio-synced) paused
   globally. During a dropout the fusion free-runs on IMU with disease A
   plus gyro bias and walks tens of degrees / hundreds of mm, then
   snap-reacquires: the teleports users see. This is not the controller
   sleeping (obs resumed without user interaction).

## Priorities

1. **Radio sync dropouts (B)** — find why the blink schedule pauses
   (rift-sensor radio keepalive / sync loop). Biggest user-visible impact.
2. **IMU rectification (A)** — decode and apply gyro_calib/accel_calib from
   flash for Touch (and check the HMD's, also printed empty); re-measure the
   8° conflict. Also verify Touch fusion_from_model rotation.
3. Joint multi-camera PnP (solve one pose from both cameras' blobs) — the
   principled cure for the 86 mm / 4° per-camera split, replay-recorder work.
4. All of this iterates 10× faster on the plan §4c replay recorder.

## Diagnostics used

- `OHMD_RIFT_CAL_CAPTURE=<file> openhmd_pose_log ...` — per-obs JSON with
  device id, per-camera pose, fused prior; controllers appear as d=1,2.
- Merge telemetry (LOGI, every 300 merges per device): count + mean shift.
- Wake the controllers (button press) right before capturing; they stop
  reporting IMU after ~20 s at rest (separate from disease B, which hits
  awake controllers).
