# Touch controller tracking: root-cause analysis (2026-07-12, evening)

> **UPDATE (same night):** both remaining diseases were root-caused and fixed
> — disease A is the IMU offset add-vs-subtract convention bug, disease B is
> the missing radio watchdog/wake-config. Full wire-level analysis and fix
> details: `windows-touch-protocol.md`.
>
> **UPDATE 2 (late night, in-game iteration — openhmd 7ad4c30):** three more
> controller root causes found from instrumented motion captures:
>
> 1. **Touch IMU timing (~25 ms)**: radio-relayed samples were stamped with
>    arrival time; every vision fix was computed against a state from AFTER
>    the exposure, dragging the fused pose backward along motion — 40-160 mm
>    of rubber-banding at hand speed, invisible at rest (which is why all
>    early validations passed). Fixed: transport compensation
>    (OHMD_RIFT_TOUCH_IMU_LATENCY_MS=25, tuned: 10 too little, 35 unstable) +
>    exposure-time mapping per device clock + constant-velocity snapshot
>    extrapolation. Fast-motion error p90 100-157 -> 16-23 mm.
> 2. **Accel-locked tilt under sustained motion**: the OVR port corrected
>    tilt from the accelerometer only (vision = yaw only); centripetal accel
>    masquerades as tilted gravity with low variance -> tilt confidently
>    stuck 9-16 deg wrong per run (verified: orientation fixes were 94-100%
>    ACCEPTED yet the error persisted — it is an equilibrium, not gating;
>    error is pure TILT: 10.6 of 10.7 deg). Fixed: vision tilt correction
>    (gain 0.5/s, snap 0.15 rad, OHMD_RIFT_NO_VISION_TILT=1 for A/B).
>    **Deployed ee5314b4499e but NOT yet validated — first thing next
>    session: motion capture with sustained smooth waving.**
> 3. Rejected hypotheses, so nobody re-chases them: fixed IMU-to-model trim
>    rotation (Wahba fit over 28k pairs across grips: best constant rotation
>    3.8 deg, residual unchanged), orientation prior-gating lock-in (fixes
>    were being accepted), optical failures during motion (optics stay
>    strong at 93-96 obs/s, zero gaps).
>
> Also added the same night: online extrinsic refinement (self-healing
> sensor poses from same-exposure co-observations, Windows-style — see
> windows-touch-protocol.md §5), after a physically-bumped sensor produced
> 150 mm cross-camera splits that desk captures could not see (steep-view
> PnP ambiguity masks extrinsic damage — never diagnose extrinsics from
> desk-resting captures).

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
