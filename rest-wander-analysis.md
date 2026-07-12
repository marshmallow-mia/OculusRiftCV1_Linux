# Rest-pose wander: root cause analysis (2026-07-12, late session)

## Symptom

In-headset (SteamVR, Half-Life: Alyx) the world visibly moves with the head
completely still. Present with both fusion backends (OVR port and
`OHMD_RIFT_FUSION=ukf`), unchanged by `OHMD_RIFT_VEL_SMOOTH_MS=60` and
`OHMD_RIFT_NO_BLEED=1`.

## The measurement that cracked it

A common rest-wander metric (band RMS + 10 s peak-to-peak + f2f steps on the
position stream) applied at every pipeline level, all from existing captures
in `captures/lin/2026-07-12/`:

| stream (at rest) | 7.5 Hz band RMS, Y | 10 s p2p Y | f2f p99 |
|---|---|---|---|
| per-camera optical world solution (cam0/cam1) | — (clean) | 0.8 / 1.0 mm | 3.7 / 5.8 mm (frame noise) |
| fused, **single camera** (`motion.csv`, pre-recal) | **0.021 mm** | 0.4 mm | 0.05 mm |
| fused, **two cameras** (`motion5.csv`) | **2.87 mm** | 6.9 mm | 1.31 mm |
| rendered SteamVR, OVR (`steamvr-live.csv`) | 1.77 mm | 5.1 mm | 1.7 mm |
| rendered SteamVR, UKF (`svr-pred-ukf.csv`) | 2.12 mm | 8.6 mm | 1.6 mm |
| rendered, OVR + `VEL_SMOOTH_MS=60` | 0.67 mm | — | — |

The wander is a **narrow spectral line at 7.51 Hz (harmonic at 15.0 Hz),
1500× above background**, vertical-dominant, present in every dual-camera
stream at nearly identical frequency (7.49–7.53 Hz), and **absent
(130× smaller) in the single-camera stream on the same rig**. Backend
identical ⇒ not fusion math. Per-camera solutions clean ⇒ not optics.
`VEL_SMOOTH_MS=60` attenuates ~3× but ±1 mm at 7.5 Hz is still visible —
matches the user's "same".

## Root cause: same-exposure fix application is order-dependent, and the order flips at ~7.5 Hz

From `final-verify.jsonl` (25 s at rest, headset on floor — the depth-ambiguity
pose where the two cameras' solutions disagree by a constant ~111 mm in Y,
which magnifies the mechanism):

1. Both sensors expose on the **same sync pulse** — 1156 of 1294 exposures are
   shared, identical `ts`, ~20 ms cadence.
2. The two sensors' fixes for the same exposure arrive in **racing order**:
   cam0-first 518×, cam1-first 673×, flipping ~20×/s, and the
   **last-arriver sequence has its dominant spectral peak at 7.49 Hz** —
   exactly the wander frequency.
3. The fused pose sits at a position along the cam0→cam1 axis that is
   **strongly determined by arrival order** (median fraction 0.48 when cam0
   corrected last vs 0.06 when cam1 did), and swings the **full range**
   between the two solutions (p5 −0.03 to p95 1.06) with spectrum peaks at
   7.53/15.01 Hz.

So: each sensor's fix is applied as an independent sequential correction with
an effective per-fix authority near 1; the fused state lands near whichever
sensor corrected last; the race winner flips quasi-periodically at ~7.5 Hz
(beat of two nearly-equal processing pipelines); the output oscillates at that
frequency with amplitude ≈ the instantaneous cross-camera disagreement.

Amplitude scaling confirms it:
- head height: cross-cam agreement sub-mm to ~3 mm (nogate-test: mean
  (−0.4, +0.2, +0.4) mm) → mm-level bounce (the user's shimmer)
- desk: a few mm, Y-dominant → motion5's 2.9 mm RMS line
- floor: 111 mm → fused f2f p99 of 126 mm in final-verify

The Y-dominance is PnP depth ambiguity: each camera's depth error projects
mostly vertically at these viewing angles, so the cameras disagree most in Y,
and the oscillation inherits that axis.

## Windows comparison

- The Oculus runtime's captures contain **no tracked-at-rest segment** (it
  drops position tracking the moment the headset is set down / prox uncovers),
  so a direct rest-vs-rest compare is impossible with what we have.
- But: a 36 s worn/moving 2-sensor Windows run shows **no narrow spectral line
  anywhere in 2–25 Hz**, while our rig at rest shows the 7.5 Hz line at 1500×
  background. Their multi-sensor merge is order-independent (SDK docs describe
  per-camera observations fused as measurements, not sequential overwrites).
- Their fused stream's f2f noise while tracked is ~0.01 mm (µm-level) vs our
  0.5 mm median at rest with two cameras — 50×. Our single-camera stream
  (0.017 mm) is at their level, which again indicts the dual-camera merge, not
  the estimator.

## THE FIX (implemented + validated same day)

**Same-exposure observation merge** in `rift-tracker.c`
`rift_tracked_device_model_pose_update()`: when a fix arrives for a delay
slot that already integrated used reports (= other sensors' fixes for the
same exposure), the fusion is corrected toward the **confidence-weighted mean
position** of ALL of this exposure's observations (weights 1/obs_scale²,
combined scale 1/√Σw) instead of the newest one alone. The cycle then ends at
the same midpoint regardless of arrival order — the race flip has nothing
left to modulate. Backend-agnostic (one change fixes OVR and UKF).

- Raw observations (not merged results) are stored in the slot's
  `pose_reports` (+ new `obs_scale` field) so later merges weight original
  measurements.
- Position only; orientation keeps its overwrite semantics (see the
  orientation-thrash lesson in `rift-fusion-ovr.c`).
- OVR port: a merged fix passes `replace_pending=true` → `vision_fix()`
  skips the 0.5-blend and **overwrites** the pending error (it already
  contains the earlier fix's information; blending would double-count).
- `dev->last_observed_pose` gets the merged position (better search prior).
- A/B: `OHMD_RIFT_NO_OBS_MERGE=1` restores old behaviour.

### Validation (same desk, same hour, 40 s rest each, `merge-test.csv` / `merge-off-test.csv`)

| | 7.5 Hz line Y RMS | f2f median | 10 s p2p Y |
|---|---|---|---|
| merge OFF (old behaviour) | 2.15 mm | 0.475 mm | 7.8 mm |
| **merge ON** | **0.36 mm (6×)** | **0.026 mm (18×)** | **2.1 mm (3.7×)** |
| single-camera reference | 0.02 mm | 0.017 mm | 0.4 mm |

Line power at 7.50 Hz: 36× reduced. f2f noise is now near the single-camera
reference. Residual line: solo exposures (frames only one camera solves) and
the few-ms excursion toward the first fix before the merged restatement —
candidates for the replay-recorder tuning loop, not blind tweaks.

## Remaining ideas

1. Defer/underweight the first fix when a second sensor's report is expected
   (kills the excursion window; needs care for solo exposures).
2. Per-fix authority is ~1 (a single fix pulls the fused pose nearly all the
   way) — retune so fixes average naturally over time.
3. Diagnostic for any future regression: the 7.5 Hz line — 10 s
   `openhmd_pose_log` at rest + FFT, no wear test needed.
4. Moving-case validation (score_prediction.py on a motion pass) still
   pending — desk-rest only so far.

Analysis scripts: scratchpad `rest_wander.py` / `run_wander2.py` (this
session); worth promoting the wander metric into `tools/` alongside the
replay recorder.
