# Why the image keeps moving after the head stops — analysis journal

Durable state for the offline investigation. Every entry: hypothesis, the
measurement that tested it, the number, and the verdict. Dead ends stay here so
they are not retried without new evidence.

Symptom: *"fast movement and the image still takes a bit to stop moving. When
not moving at all it feels like its correct."* A tail, not latency.

## Already eliminated in-headset (before this journal)

| hypothesis | test | number | verdict |
|---|---|---|---|
| velocity EMA lag | `OHMD_RIFT_VEL_SMOOTH_MS=0` | smoothing fully off | **DEAD** — no felt change |
| orientation settling | vision gain 0.25 → 3.0 | err 1.8°→0.09°, dwell 206–1158→4–25 ms | **DEAD** — no felt change |
| prediction horizon too long | measured pose age | 1.6 ms mean, 3.1 ms max; throw ≤5.1 mm | **DEAD** — horizon is honest |
| angular-velocity frame | `OHMD_RIFT_ANGVEL_FRAME=world` | — | **DEAD** — no felt change |

Surviving clue: `OHMD_STEAMVR_NO_PREDICT=1` felt **better** despite strictly
increasing latency. Something in the output path throws the pose past the
measurement.

---

## Iteration 1 — 2026-07-31

### Built: `tools/fusion_replay.c` (Track A)

Links the **real** `rift-fusion-ovr.c` and `exponential-filter.c` (same pattern
as `tools/calib_replay.c`) and drives them with a physically consistent
synthetic profile: analytic trapezoidal rate with cosine ramps, body-frame
specific-force accelerometer and body-frame gyro to the fusion's own
conventions, vision at 60 Hz through real delay slots with a 30 ms latency.
Vision is fed **exact truth**, so any residual is the estimator's own.

Profiles: `turn` (400 °/s peak — the top of what the in-headset horizon
telemetry actually recorded), `translate` (1.2 m/s), `still`.

### H1 — the One Euro output filter is the tail

**DEAD.** Bypassing it changes settling from 390 ms to 384 ms. It is not the
tail.

### H2 — the 3rd-order position observer's 1.79 s pole is the tail

**REAL BUT SMALL — not yet sufficient.** With clean vision input, after a
1.2 m/s translate-and-stop:

- overshoot past final truth **2.60 mm**, settling to <1 mm **390 ms**
- accel-bias state peaks **0.0389 m/s²**, decays 0.036 → 0.020 over 560 ms —
  the slow pole is visibly there, at the predicted timescale
- reported velocity decays 0.0188 → 0.002 m/s over ~500 ms, which at SteamVR's
  22 ms horizon contributes only **0.41 mm → 0.05 mm** of predicted throw

So the mechanism is confirmed to exist and to have the predicted time constant,
but at this amplitude it is likely sub-perceptual. **Not promoted to cause.**
The `turn` profile shows *no* positional tail at all.

### H3 — One Euro degrades orientation far more than its 5 ms suggests

**CONFIRMED, NEW DEFECT.** Worst orientation error during a 400 °/s turn:

| | with One Euro | bypassed |
|---|---|---|
| max orientation error | **2.121°** | **0.138°** |

A 15× degradation at *every* orientation tested (80°–380° final yaw). It is
exactly the filter's 5.3 ms lag × 400 °/s. Independent of the tail question,
this is 2° of avoidable orientation error under fast motion.

### H4 — One Euro is unsafe near 180° of rotation

**CONFIRMED, SERIOUS BUG.** At exactly 180° final yaw, 36 of 4301 samples
exceed 5° error, **max 149.50°**. Bypassed: max 0.138°, zero glitches. Confined
to 180° — the sweep shows 2.12° at every other orientation.

Cause is `adjust_exp_map_proximity()` in `src/exponential-filter.c`. The filter
smooths orientation as an exponential map, whose axis flips sign at π, and the
guard is broken two ways:

1. `delta_mag` is a vector **length**, so it is always ≥ 0 and the
   `delta_mag < -M_PI` branch is dead code.
2. The correction rescales `prev_map` **along its own direction**
   (`1 - n·2π/|prev_map|`), which is only meaningful when prev and current are
   near-parallel. At the π crossing they are near-**anti**-parallel, so the
   correction is wrong and the filter smooths across a discontinuity.

### Track B — `Rift.dll` extraction started

`rizin -A -q -c aflj` over
`~/.local/share/oculus-wine/.../server-plugins/disabled/Rift.dll` →
**5591 functions** in `rift_funcs.json`. No analysis yet.

Correction to an earlier assumption: the Windows runtime does **not** need the
dirty USB volume mounted. `Rift.dll` and `OVRServer_x64.exe` are on local disk
under `~/.local/share/oculus-wine/`.

### Verdict for iteration 1

Two genuine defects found (H3, H4), neither of which convincingly explains the
reported tail. H2 is real at the predicted timescale but too small at clean
input to promote.

**The clean-input result is the key limitation.** Feeding exact truth at 60 Hz
makes this a lower bound on the tail: real vision carries noise, viewpoint-
dependent per-camera bias of several mm, and dropouts, and a real head *turn*
also translates the HMD on a neck lever arm. The structural response is small;
the response to *realistic* input is untested.

### Next

1. **Drive the harness with real captured vision** — `captures/lin/2026-07-31/*.jsonl`
   holds `wp` world poses at 60 Hz from actual sessions. Replace synthetic truth
   with those and re-measure the tail. This is the highest-value open item.
2. Track B: find the fusion update in `Rift.dll` — gain structure, any output
   smoothing (do they have a One Euro equivalent at all?), and where their
   angular acceleration comes from.
3. Fix H4 regardless of the tail outcome; it is a 150° error in shipping code.

---

## Iteration 2 — 2026-07-31

### H2 revisited — the 1.79 s accel-bias pole

**DEAD, on numbers.** Iteration 1 left it unresolved because clean input barely
excites it. Tested properly by stepping the *vision measurement* — which is what
really happens when a head turn changes which camera dominates the solve and the
per-camera bias shifts by a few mm — and measuring the response:

| fraction of a 10 mm vision step absorbed | time |
|---|---|
| 50% | 92 ms |
| 90% | 134 ms |
| 99% | 146 ms |
| residual, exponential fit | **τ = 1.44 s** |

The slow pole is real and lands near its predicted 1.79 s, but it carries
**under 10% of the response**. A 10 mm step leaves ~1 mm crawling over ~1.4 s.
Sub-perceptual. The bulk of any vision correction is absorbed in ~134 ms by the
well-damped 1.1 Hz pair. **Not the cause.** (`--vision-bias` in the harness.)

### Track A limitation found: the recorded captures cannot drive a replay

`captures/lin/*.jsonl` `wp` poses are per-observation pose-search *outputs*, not
a clean trajectory: median inter-sample speed 0.66 m/s but p90 **285 m/s** and
max 32970 m/s on `horizon.jsonl` — many entries are failed matches. Only 57–93%
of samples fall under a 3 m/s sanity bar, and the longest contiguous sane run is
6.4 s. Reconstructing "truth" from them to test the fusion would be circular.
**Replay-from-capture is not viable with the data we have.**

### Phase 3 metric built, but it cannot separate estimator from physics

`tools/settling_metric.py` finds deceleration-to-rest events and measures how far
the pose keeps travelling. On the Oculus capture: translation coast **524 ms /
41.2 mm**, rotation **262 ms** — but only 1 event each in 61 s, and, more
importantly, *that number includes the head physically decelerating*. With only
the runtime's own output there is no independent truth to subtract. **Do not
treat 41 mm as a target.** The metric is only meaningful comparing two stacks on
the same motion, which needs a Linux motion capture we have agreed not to
request. Our one existing Linux poselog (`2026-07-12/steamvr-live.csv`) is
effectively stationary (|ω| mean 1.0 °/s) and predates this session.

### Track B — their bias is an EKF state, ours is a fixed integrator

Established from the string table without needing full decompilation:

```
%d: Update: AccBias %.2f, %.2f, %.2f (%.2f), GyroBias %.2f, %.2f, %.2f (%.2f), GravAlign ...
%d: Ekf Reset: Sigmas: pos %.1f, vel %.1f, orient %.2f
EKF: Update: P or Z is not positive definite
EKF: P not positive definite: %d = %.3g
velocity_max / linear_velocity / angular_velocity / angular_acceleration / linear_acceleration
Setting dynamic prediction failed!
```

Each bias is logged **with its own uncertainty** in parentheses, and a covariance
matrix `P` is maintained and checked for positive-definiteness. So their bias
gain is covariance-derived and adapts — high while converging, low once settled —
where ours is a fixed 25/s integrator with a permanent pole. A real structural
difference, though iteration 2 has now shown our pole is too small to be the
complaint, so this is a parity item rather than the fix.

`angular_acceleration` appears as a first-class named quantity alongside
`linear_velocity`/`angular_velocity`, confirming the CSV finding that they
populate it and we do not.

The prior function map in `decomp/rift-dll-functions.md` already locates the EKF
(`fcn.18013a920` vision update, `fcn.180138780` reset, `fcn.180141610` init
state, `fcn.18017d9e0` core), so exact Q/R extraction is possible later but was
not worth the cost this iteration.

### Verdict for iteration 2

The leading suspect is dead. Six candidates now eliminated with numbers.

### The strongest remaining candidate — `vision_recent`

Untested, and it fits the symptom better than anything eliminated so far.
`rift_fusion_ovr_imu_update()` integrates position **only while a vision fix is
newer than `VISION_RECENT_NS` (70 ms)**; otherwise it takes the `else` branch and
**zeroes `lin_vel` outright**, freezing position.

Fast head motion is exactly when vision drops out — motion blur costs blobs, and
the pose search needs 10 matched LEDs. So during a fast turn the position can
freeze, and when the head stops and vision recovers, the estimate has to travel
from where it froze to where the head actually is. **The image continues moving
after the wearer has stopped** — which is the complaint, in the wearer's own
words.

Test next: inject vision dropouts during the motion phase of the harness and
measure the post-motion catch-up distance and duration against dropout length.
The in-headset logs support the premise — `settle.log` recorded 26 "Matched
orientation after" gaps with a median of 0.19 s and a max of 0.87 s, i.e. vision
outages far longer than the 70 ms gate.

---

## Iteration 3 — 2026-07-31 — **CAUSE FOUND**

### H5 — freezing position during a vision outage, then reading the accumulated error as velocity

**CONFIRMED. This is the cause.** `--dropout-ms` in the harness, `translate`
profile (1.2 m/s), vision suppressed through the deceleration:

| vision dropout | post-stop travel | settling |
|---|---|---|
| 0 ms | 2.40 mm | 390 ms |
| 70 ms | 2.38 mm | 988 ms |
| 120 ms | 3.87 mm | 754 ms |
| **200 ms** | **59.32 mm** | **1608 ms** |
| **300 ms** | **178.43 mm** | **1222 ms** |
| 500 ms | 424.16 mm | 66 ms (snaps) |
| 900 ms | 600.00 mm | 68 ms (snaps) |

A 200–300 ms gap yields **6–18 cm of travel after the head has stopped**, over
1.2–1.6 s. Measured gaps in the wearer's own session: 0.19 s median, 0.87 s max.

### The mechanism, from the 200 ms trace

| t (s) | true (mm) | estimate (mm) | error (mm) | fusion \|v\| |
|---|---|---|---|---|
| 0.55 | 538.0 | 532.9 | −5.1 | 1.04 |
| 0.60 | 578.8 | **540.7** | −38.1 | **0.0000** |
| 0.70 | 600.0 | **540.7** | −59.3 | **0.0000** |
| 0.75 | 600.0 | 548.1 | −51.9 | 0.050 |
| 1.00 | 600.0 | **626.1** | **+26.1** | 0.216 |
| 1.30 | 600.0 | 610.9 | +10.9 | 0.000 |
| 2.30 | 600.0 | 601.0 | +1.0 | 0.004 |

Three compounding faults, all in `rift_fusion_ovr_imu_update()` /
`apply_position_correction()`:

1. **Position freezes.** With no vision fix newer than `VISION_RECENT_NS`
   (70 ms), the `else` branch sets `lin_vel` to zero and stops integrating. The
   head keeps moving; the estimate does not. Error grows to 59 mm.
2. **The accumulated error is then read as velocity.** On re-acquisition the
   position error is fed into `GAIN_VEL` (50/s) and `GAIN_ACCEL` (25/s) as well
   as `GAIN_POS`. The estimator interprets "59 mm behind" as "moving fast",
   injects phantom velocity, and **overshoots 26 mm past the truth**, then rings
   for ~2 s.
3. **The phantom velocity is exported.** 0.216 m/s is reported while the head is
   completely still, and SteamVR multiplies it by its prediction horizon.

### Why this explains everything the earlier hypotheses could not

- **only under fast motion** — dropouts are caused by motion blur, which needs
  speed; the pose search needs 10 matched LEDs
- **correct at rest** — no dropouts, no accumulated error, no phantom velocity
- **`OHMD_STEAMVR_NO_PREDICT=1` felt better** — it stops the phantom velocity
  being extrapolated, at the cost of latency, exactly as observed
- **velocity smoothing and vision gains changed nothing** — wrong subsystem
- **the 500 ms+ rows snap instead of drifting** (`VISION_REACQUIRE_NS`), so the
  damaging band is roughly **120–500 ms**, which is precisely where the measured
  gaps sit

### Next — fix, and prove it on this same sweep

Candidate fixes to test in the harness, choosing on measurement:

- **A**: keep dead-reckoning position from the IMU through short outages instead
  of freezing. This is what the accelerometer is for, and over 200–300 ms its
  drift is far below the 59 mm the freeze costs.
- **B**: on re-acquisition after a gap, apply the position correction without
  feeding it into the velocity and bias states — the error is stale, not
  evidence of motion. Optionally scale the velocity/bias gains by outage length.
- **C**: do not export a velocity the estimator has synthesised from a stale
  position error.

Windows precedent to check on Track B: `Ekf Freeze`, `Gyro saturation: %d
samples, orient sigma %.2f`, and `Ekf Reset after large pose update` suggest the
runtime has an explicit outage policy, and an EKF would grow covariance during
the gap and weight the returning fix accordingly rather than converting a stale
error into velocity.

Regression test: the dropout sweep above becomes the acceptance criterion —
post-stop travel must stay near the 2.4 mm no-dropout figure across
120–500 ms gaps.
