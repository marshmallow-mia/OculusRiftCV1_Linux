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

---

## Iteration 4 — 2026-07-31 — **FIXED**

### Fix A — coast on the IMU through a vision gap instead of freezing

Applied in `rift_fusion_ovr_imu_update()`: position now integrates while a fix
is newer than `VISION_REACQUIRE_NS` (500 ms) rather than `VISION_RECENT_NS`
(70 ms). Past that the fix snaps anyway, so coasting further buys nothing.
`OHMD_RIFT_DEADRECKON_MS` overrides; **70 reproduces the old behaviour**.

Post-stop **drift** at 1.2 m/s, old gate vs coasting:

| dropout | OLD drift | NEW drift |
|---|---|---|
| 0 ms | 3.14 | 3.14 |
| 70 ms | 5.17 | 4.67 |
| 120 ms | **9.97** | 5.81 |
| 200 ms | **114.81** | 6.48 |
| 300 ms | **88.78** | 3.91 |
| 500 ms | 3.54 | 3.40 |
| 700 ms | 3.53 | 3.32 |

The rationale for the old 70 ms gate was accelerometer drift under double
integration, but that is far cheaper than freezing: over a 300 ms gap a
0.05 m/s² residual bias contributes ~2 mm, against the 59 mm the freeze cost at
200 ms.

### Drift vs snap — an important distinction for the metric

Past the coast window the returning fix is applied as a **single-frame snap**
(`VISION_REACQUIRE_NS`), not a drift. Decomposed at 1.2 m/s with the fix in:

| dropout | total | largest single step | drift |
|---|---|---|---|
| 300 ms | 3.91 mm | 0.04 mm | 3.91 mm |
| 500 ms | 22.77 mm | 6.21 mm | 3.40 mm |
| 700 ms | 154.57 mm | 42.34 mm | 3.32 mm |

So drift — the reported symptom — is fixed across the whole range. What remains
at long outages is a discrete jump, which is pre-existing behaviour for genuine
tracking loss and reads as a pop, not as "takes a bit to stop moving". The
regression test therefore gates on **drift** and reports snap separately, so
neither can regress unnoticed and neither hides the other.

### Fix for H4 — the exponential-map singularity

`adjust_exp_map_proximity()` rewritten to choose whichever of the two equivalent
representations of the previous sample lies closer to the current one, instead
of rescaling along its own direction. Worst orientation error ending a 400 °/s
turn at exactly 180°: **149.503° → 2.121°**, i.e. now identical to every other
final orientation. The residual 2.121° is H3, the filter's 5.3 ms lag, which is
a separate open item.

### Regression test

`tools/run_dropout_check.sh` — builds `fusion_replay` against the real driver
sources and gates on post-stop drift (8 mm bar) plus the 180° orientation error
(5° bar). **Passes with the fix; fails on the old behaviour** at 120/200/300 ms,
which is the property that makes it a test rather than a demonstration.

Full unit suite passes. Driver rebuilt and deployed.

### Status

Cause found, fixed, and covered by a regression test, entirely offline. The
plan's stop condition is met. Remaining open items are separate from the
reported symptom:

- **H3**: the One Euro filter costs 2.121° of orientation error at 400 °/s
  against 0.138° bypassed. Its adaptation is also dead — `exp_filter3d_run()`
  computes `dy` as a raw sample difference and never divides by `dt`, so the
  `beta` term is ~0.001 Hz against a 30 Hz cutoff. Worth revisiting: the filter
  may not be earning its place at all.
- **Angular acceleration**: `DriverPose_t.vecAngularAcceleration` exists and we
  export nothing into it; the runtime populates it (786 °/s² mean, 21131 peak).
  Worth 0.05–0.27° at a 22 ms horizon.
- **Vision gain 0.25 → 3.0**: 10× better on its own metric (1.8° → 0.09° mean
  error), held back only to avoid confounding this hunt.
- **Track B parity**: their bias states carry covariance and adapt; ours are
  fixed-gain integrators.

---

## Iteration 5 — 2026-07-31 — the symptom is restated, and the search restarts

New information from the user that reshapes everything:

> *"Its the same on headset and controllers. I move and it overshoots, gets back
> too far, etc until it reached the real point."*

- **Identical on HMD and both Touch controllers** → the fault is in *shared*
  machinery, not anything device-specific. Rules out the HMD render pivot, IPD,
  and per-device offsets.
- **A damped oscillation**, not a lag or a one-way tail.

The iteration-4 fix (coast through vision dropouts) was deployed at 16:38 and the
user ran it at 16:56 — verified from `vrserver.txt`. It changed nothing. The
mechanism family was right; the trigger was not.

### Correction to iteration 2's method

`wp` in the capture is **not** a per-camera vision pose. It is
`exp_dev_info->capture_pose` — the **fusion's** pose at exposure time, written
identically into every sensor's record (verified in `rift-sensor-pose-search.c`
and by finding both sensors reporting bit-identical values). A first pass here
reported "cross-camera disagreement median 0.00 mm", which was a value being
compared against itself. The real per-camera vision estimate is
`campose ∘ cam`.

### H6 — the two cameras disagree, and the fusion chases the difference

**PARTLY TRUE, BUT NOT THE CAUSE.** With the composition done properly
(`angvelworld.jsonl`, LED-ID-verified, gross outliers dropped):

| device | cross-camera disagreement (median) | p90 | position dependence |
|---|---|---|---|
| HMD | 5.33 mm | 9.38 | changes 5.6–8.3 mm across the room |
| ctrl 1 | 9.48 mm | 13.92 | changes 3.5–9.3 mm |
| ctrl 2 | 6.06 mm | 11.48 | changes 3.0–5.3 mm |

So the calibration's self-reported **1.4 mm** is optimistic by 4–7× against live
data — it scores itself on the viewpoints it fitted, the trap already documented
in `cv1-auto-calibration`.

**But it is not a rigid calibration error.** Pooling all three devices (7318
co-observed exposures) and solving for the single best rigid correction:
median 7.79 → **6.48 mm**, needing only **3.30 mm / 0.084°**. The extrinsics are
essentially right; the residual is per-camera PnP bias that varies with viewing
geometry — which this project already established cannot be estimated away
(a full bundle adjustment scored no better than a robust mean).

### H7 — vision hands the fusion a wobbling target

**DEAD.** There is a same-exposure merge and a joint solve
(`rift-tracker.c:1753`), so the fusion receives one merged target, not two
conflicting ones. Measured wobble of that merged target against a 5-sample
running median:

| device | wobble median | p90 |
|---|---|---|
| HMD | 0.13 mm | 0.97 mm |
| ctrl 1 | 0.58 mm | 2.78 mm |
| ctrl 2 | 0.42 mm | 4.19 mm |

**The vision input is smooth to well under a millimetre.** The cross-camera
disagreement is a slowly-varying bias the merge averages away.

### Where that leaves it — a tight elimination

1. The merged vision target is smooth (0.13–0.69 mm) → the oscillation is **not**
   in the vision input.
2. The harness shows the fusion does **not** ring when fed clean input.
3. Therefore the remaining input is the **IMU**.

An IMU scale or timing error makes IMU-integrated motion disagree with vision
*in proportion to how much you move* — fighting during motion, correct at rest,
and identical on HMD and controllers if the fault is in shared decode/scaling
rather than per-device hardware. That matches the restated symptom exactly and
has never been tested.

### First IMU measurement

Decoding `captures/win/2026-07-12/imu_tracking.pcap` (65630 samples, the Oculus
runtime's own session) with our decoder's scaling:

- `|accel|` at rest (|gyro| < 0.05 rad/s, 27.4% of samples): **9.4582 m/s²**
  against 9.8067 expected → **−3.55%**.

This is the **raw** decode without the factory `AccCalibration` matrix, which
the driver does apply (`apply_imu_calibration`, `rift.c:413`), so it is not yet
evidence of a defect — it is the number the calibration is supposed to correct.

### Next

1. Extract the `IMU_CALIBRATION` (report 0x03) matrices from
   `captures/win/2026-07-12/setup_hid.pcap`, apply them to the raw stream, and
   check that `|accel|` lands on 9.8067. That validates our whole IMU
   calibration path end-to-end, offline.
2. The same report carries the **gyro** matrix. A gyro scale error is the single
   best fit to "overshoots, gets back too far": a 3% error turns a 90° turn into
   93°, which vision then drags back over a second or two.
3. Then Stage 3 proper — our fusion on their raw IMU, diffed against their poses.

Note a decoder discrepancy to resolve while doing this: `analyze_hid_pcap.py`
caps at `min(b[3], 2)` samples per report where `packet.c` caps at 3.

---

## Iteration 6 — 2026-07-31 — **a confirmed defect against an absolute reference**

### The first measurement that did not compare the system against itself

Live telemetry, headset stationary:

```
accelerometer scale check: |accel| at rest mean 10.0543 m/s^2 (+2.52% vs 9.8067),
range 9.816..10.272, 10013 quiet samples | hw calibration NOT requested
```

Steady at 10.0533–10.0547 over four independent 10 s windows. At rest an
accelerometer measures gravity and nothing else, so this **must** be 9.8067.

Correction to an earlier reading: `rift.c:1471` sets `RIFT_SCF_USE_CALIBRATION`,
from which I concluded `apply_imu_calibration()` was dead code. At runtime the
flag is **clear** — the device does not accept it — so the driver does apply its
own calibration, and that path produces the 10.0543.

### H8 — the factory matrix is applied wrongly

**CONFIRMED.** On the Oculus runtime's own raw capture, with this headset's
factory values:

| | \|accel\| at rest |
|---|---|
| raw | 9.4487 (−3.6%) |
| offset only | **9.7503 (−0.6%)** |
| offset + matrix (ours) | 10.0963 (**+3.0%**) |

Offline +3.0% against live +2.52%; the gap is only the resting orientation.
Every alternative convention was tried and **none** reaches 9.8067:

| convention | result |
|---|---|
| `M(raw−off)` (ours) | +3.01% |
| `M⁻¹(raw−off)` | −3.86% |
| `Mᵀ(raw−off)` | +3.04% |
| `M⁻ᵀ(raw−off)` | −3.83% |
| `M·raw − off` | +2.90% |
| `M·raw + off` | −3.22% |
| **offset only** | **−0.50%** |

### What it is not

- **Not the scale constants.** `1/((1<<20)-1)` and `1e-4` both appear in
  `Rift.dll` exactly as `packet.c` uses them.
- **Not the row layout.** Decoding accel and gyro rows *interleaved* yields two
  diagonally-dominant matrices; reading them sequentially does not.
- **Not a data-fit answer.** The ellipsoid fit over `imu_tracking.pcap` is
  under-determined: only **5 distinct gravity directions** among the stationary
  samples, and the fit is not positive definite. Recorded as a dead end.

### What it is

The HMD matrix carries a real **8% scale** — singular values 1.0381 / 1.0219 /
0.9614 — where its orthogonal part is near-identity with ~1° of cross-axis
alignment. The **Touch** matrix, from the radio JSON, is an axis permutation
already orthogonal to **0.41%**.

So one rule serves both: **keep the orthogonal polar factor, discard the scale.**

| | before | after |
|---|---|---|
| HMD `\|accel\|` | +3.01% | **−0.50%** |
| Touch matrix | permutation | unchanged (max element shift 0.005) |

Implemented as a Newton iteration `R ← (R + R⁻ᵀ)/2` at calibration load, which
matches the SVD polar factor to 3e-16 and is orthogonal to 1e-16.
`OHMD_RIFT_NO_CALIB_ORTHO=1` restores the raw factory matrix.

### Honest magnitude

Phantom acceleration drops from ~0.25 to ~0.05 m/s². In the harness at 1.2 m/s,
settling after a stop goes **868 ms → 414 ms** against an ideal of 390 ms, with
drift 5.50 → 4.54 mm. So this is a real defect corrected against an absolute
reference, but the predicted improvement is **moderate, not dramatic** — and the
residual −0.50% is unexplained, as is why the factory matrix carries a scale at
all. Both are open.

---

## Iteration: is the artifact in the runtime rather than the pose?

### Why this became the question

`tools/find_oscillation.py` on `captures/lin/2026-07-31/osc.csv` — 53456 samples
of what SteamVR is actually handed — reports residual **0.64 mm** while moving,
p90 1.74 mm, and **no dominant frequency**: every spectral component is under
0.2% of residual power. The Oculus Windows reference is 0.27 mm, also peakless.

There is no oscillation in the pose. Nine fixes to individual fusion stages were
each measured correct afterwards and none moved the reported symptom, which is
consistent with all nine having been in the wrong half of the system.

The user reports `hello_xr` under Monado feels better — same tracking code,
different runtime and compositor. That is suggestive but confounded: `hello_xr`
draws a few cubes, whereas a real application that misses frame deadlines gets
reprojected, and reprojection looks exactly like the world lagging and catching
up. So the test needs a *demanding* application under Monado.

### Layer audit: Bonelab (native OpenXR) under Proton + Monado

Bonelab reports "openxr loader failed to initialize". Every layer was checked;
all but one is correctly configured.

| Layer | State | Evidence |
|---|---|---|
| Monado service + display | good | swapchain 2160x1200, vblank thread running |
| DRM lease | good | `_lease_connector_withdrawn` is **benign** — it means the connector cannot be leased *again* because Monado holds it |
| Host OpenXR runtime pointer | good | `~/.config/openxr/1/active_runtime.json` -> Monado |
| pressure-vessel import | good | `PRESSURE_VESSEL_IMPORT_OPENXR_1_RUNTIMES=1` captures `libopenxr_monado.so`, generates a correct in-container `active_runtime.json`, sets `XDG_CONFIG_DIRS`. i386 capture fails; harmless, the game is 64-bit |
| Monado IPC socket in container | good | `/run/user/1000/monado_comp_ipc` visible inside |
| Runtime deps in container | good | `ldd` resolves everything |
| Native OpenXR loader in container | good | Proton's own `libopenxr_loader.so.1` (1.1.36), inside **steamrt4**, returns `XR_SUCCESS` with **57 extensions** incl. `XR_KHR_vulkan_enable`, `_enable2`, `_convert_timespec_time` |
| Wine registry + bridge files | good | `ActiveRuntime = C:\openxr\wineopenxr64.json`; json and dll present |
| Steam launch options | good | verbatim in `localconfig.vdf` |
| **Wine<->Linux OpenXR bridge** | **BROKEN** | `wineopenxr.dll`'s `xrNegotiateLoaderRuntimeInterface` returns **-6** (`XR_ERROR_INITIALIZATION_FAILED`) |

From `~/steam-1592190.log`:

```
RuntimeInterface::LoadRuntime skipping manifest file C:\openxr\wineopenxr64.json,
negotiation failed with error -6
```

DXVK's own OpenXR provider fails in the same run (`Unable to get required Vulkan
instance extensions size`), so the break is in the bridge, not in game code.

### Two corrections to earlier readings, both mine

- I called `_lease_connector_withdrawn` "KWin took back the lease" and blamed it
  for Alyx's black screen. Wrong: it is the normal protocol event for a connector
  that is now leased. Monado had the display the whole time.
- I verified the container stack in **sniper**. Proton Experimental 11.0 uses
  **steamrt4**; the log header (`depot: ... steamrt4`) says so. Re-run in
  steamrt4 gave the same result, so the conclusion survived, but the first pass
  was measuring the wrong container.

### Leading hypothesis, unconfirmed

Bonelab bundles OpenXR Loader **1.0.27**; Proton's bridge is built against
**1.1.36**. A version-negotiation rejection would produce exactly this error.
Not measured — confirming it means replacing a DLL inside the game install, which
was not authorised. Recorded as hypothesis, not finding.

### Verdict

Tracking is exonerated by measurement and is not the open question. The open
question is presentation, and answering it requires a real app rendering through
Monado. Next: a title that reaches Monado through xrizer (OpenVR), which bypasses
the broken bridge entirely and is already proven to connect.

---

## Iteration: SteamVR's extrapolation, not the pose

### Hypothesis

Bigscreen under Monado feels correct with the *same* OpenHMD tracking, so the
artifact is in the SteamVR driver layer. The two layers differ structurally in
exactly one way that can produce overshoot:

- **Monado never extrapolates.** `oh_device.c` never sets
  `XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT`, never writes `linear_velocity`,
  and ignores `at_timestamp_ns` (it appears only in the parameter list at `:383`).
  Latency is absorbed by the compositor reprojecting onto a freshly **measured**
  pose (`comp_renderer.c:1098`). Overshoot is structurally impossible.
- **SteamVR-OpenHMD hands SteamVR everything it needs to extrapolate**:
  `vecVelocity`, `vecAcceleration`, `vecAngularVelocity`, and
  `poseTimeOffset = -pose_age`, on both the HMD and the controllers. SteamVR
  dead-reckons all of it to photon time (~22-33 ms) and displays the result.

Monado's own SteamVR shim zeroes all five deliberately
(`steamvr_drv/ovrd_driver.cpp:1356-1379`, *"monado predicts pose 'now'"*).

### Correction: why the earlier evidence looked exculpatory

`tools/lin_pose_log.py` called
`getDeviceToAbsoluteTrackingPose(..., 0.0, poses)` — **prediction horizon zero**.
So `osc.csv`, the 0.64 mm / no-dominant-frequency result, describes the pose
SteamVR *received*, not the pose it *renders*. At horizon 0 there is no
extrapolation, so that capture could not contain the artifact under investigation.
Calling it "the entire tracking stack is exonerated" was too broad.

The earlier `OHMD_STEAMVR_NO_PREDICT` A/B was also confounded twice over: it was
wired only into the HMD path, so the controllers kept extrapolating; and it ran at
15:52, while `displayFrequency` was still `0` (not fixed until 18:18), so SteamVR's
photon clock was wrong at the time.

### What changed

- `tools/lin_pose_log.py` gained `--predict-ms`, accepting a comma-separated sweep.
  Every horizon is sampled **in the same loop iteration** as horizon 0, because a
  human cannot repeat a head turn twice; the motion is then identical by
  construction and the only difference between files is how far SteamVR
  extrapolated. Predicted rows are stamped with the time they were predicted *for*,
  matching `tools/win/ovr_pose_log.c`, so `analyze_prediction.py` recovers the
  horizon correctly and scores our predictor by the identical method used on the
  Oculus runtime's 0.19 deg / 2.2 mm.
- `driver_openhmd.cpp` gained `ApplyPredictionPolicy()`, applied to the HMD **and**
  both controllers. Default is to extrapolate with nothing;
  `OHMD_STEAMVR_PREDICT=1|all` restores the old behaviour, and individual terms
  (`offset,angvel,vel,accel`) can be earned back one at a time.
  `vecAngularAcceleration` is now explicitly zeroed - it was never populated while
  `vecAcceleration` was, making linear extrapolation second-order and angular
  first-order, an asymmetry that reads as some parts settling later than others.
- Two adjacent controller defects fixed: the untracked fallback position was
  written in `Activate()` and then erased by `pose = { 0 }` in `GetPose()` (dead
  since it was written); and `poseIsValid`/`result` were hardcoded true/`Running_OK`
  regardless of tracking flags, so an untracked controller was reported as
  confidently located at the origin.

### Measurement

PENDING - baseline sweep with `OHMD_STEAMVR_PREDICT=1` vs default, scored with
`analyze_prediction.py`. The falsifiable prediction: predictor error should exceed
zero-prediction error at 22-33 ms with prediction on, and the horizon sweep should
flatten with it off. If the error does *not* grow with horizon, this hypothesis is
wrong and the next target is the eye/projection geometry.

Regression at this point: `tools/run_dropout_check.sh` all within bars; orientation
error 2.121 deg unchanged.

### Measurement: VOID - the experiment tested one binary against itself

The prediction on/off A/B returned "both feel the same", and the pose data agreed
far too well:

| horizon | predold `\|pred-now\|` | prednew `\|pred-now\|` |
|---|---|---|
| 11 ms | 2.00 mm | 2.49 mm |
| 22 ms | 4.00 mm | 4.98 mm |
| 33 ms | 6.00 mm | 7.47 mm |

Both runs extrapolated, both perfectly linear in horizon, and `prednew` reported
100% non-zero velocities through OpenVR despite the driver being told to send
none. That is not a null result, it is a broken manipulation.

**Cause: SteamVR loads `<driver-dir>/bin/linux64/driver_openhmd.so`, and meson has
no rule that produces it.** Meson builds `driver_openhmd.so.0.0.1` in the build
root; the `bin/linux64` copy was placed by hand once and had been stale since
18:14. Confirmed directly:

```
build/driver_openhmd.so.0.0.1        HAS new prediction policy   (19:18)
build/bin/linux64/driver_openhmd.so  OLD - no prediction policy  (18:14)
```

So both runs loaded the same 18:14 binary. `OHMD_STEAMVR_PREDICT` was read by code
that was never loaded. The hypothesis is **untested, not falsified**.

OpenHMD is linked *into* the driver `.so` (`ldd` shows no separate `libopenhmd`),
so this staleness window silently covers OpenHMD changes too, not just the SteamVR
layer - any driver-side change made after 18:14 and tested before this fix reached
nothing.

### Fix to the harness

`tools/run_steamvr_test.sh` now refreshes `bin/linux64/driver_openhmd.so` from the
meson output when the latter is newer, and says which binary it is starting with a
timestamp. Same defensive posture the script already takes on the OpenHMD library
pin: fail or fix loudly rather than quietly measure the wrong code.

**Method note, third time this has bitten in this project:** every measurement must
first prove it can see the thing it claims to manipulate. The earlier
`OHMD_STEAMVR_NO_PREDICT` A/B at 15:52 shares this failure mode and its negative
result should not be trusted either.

### Re-run with the manipulation verified: prediction FALSIFIED, cleanly

Second attempt, after the harness fix. The driver log confirms both arms loaded
the intended code before anything was judged:

```
19:27:22  SteamVR prediction terms = 0xf (offset=1 angvel=1 vel=1 accel=1)
19:28:03  SteamVR prediction terms = 0x0 (offset=0 angvel=0 vel=0 accel=0)
```

| | driver vel reported | extrapolation @11/22/33 ms |
|---|---|---|
| `predold2` (0xf) | 100% nonzero, median 0.179 m/s | 1.98 / 3.97 / 5.94 mm |
| `prednew2` (0x0) | **0.0% nonzero** | **0.000 / 0.000 / 0.000 mm** |

The manipulation was total: SteamVR moved the pose by exactly nothing at every
horizon. **The artifact was unchanged.** Forward prediction is not the cause.

That is hypothesis ten, and the first one to die against a manipulation that was
independently verified to have taken effect rather than assumed to.

### And the prediction is good, so it stays on

`tools/analyze_prediction.py` on `predold2_*`, tracked & moving (|w|>0.5 rad/s),
n=10929:

| horizon | predicted | no prediction |
|---|---|---|
| 11 ms | 1.17 mm / 0.146 deg | 4.12 mm / 1.013 deg |
| 22 ms | 2.57 mm / 0.393 deg | 8.23 mm / 2.020 deg |
| 33 ms | 4.20 mm / 0.747 deg | 12.34 mm / 3.026 deg |

About 3x better than not predicting, and in the same league as the Oculus
runtime's 0.19 deg / 2.2 mm measured by the same method. Disabling it would be a
latency regression that fixes nothing, so the default is restored to predicting
with everything; `OHMD_STEAMVR_PREDICT=0|none|<subset>` remains for experiments.

Net gain from this iteration: prediction quality is now measured rather than
assumed, on both device types, with a repeatable harness - and one more suspect is
eliminated with numbers instead of a feeling.

### What the artifact must now be

It survives with SteamVR extrapolating by literally zero, and it is absent under
Monado on the same tracking. Both stacks therefore start from the same pose, so
the difference lies strictly downstream of it: eye/projection geometry, the
distortion mesh, or compositor frame pacing and reprojection under load.

Next, and cheapest: **run Bigscreen under SteamVR.** It is the one application
already known to feel correct under Monado, so running the same app on the other
runtime isolates runtime from application load - a confound present in every
comparison so far (hello_xr is trivial to render; Alyx never rendered).

---

## Iteration: the two stacks render at different fields of view

### What the eliminations force

Bigscreen is wrong under SteamVR and right under Monado - same app, same
tracking, same headset. Combined with the measured eliminations (pose clean at
0.64 mm; prediction provably 0.000 mm of extrapolation with the artifact intact;
2% dropped frames and 0 reprojected; direct mode confirmed), the cause has to lie
in the projection itself.

Also corrected: "only some parts lag behind" was withdrawn - the motion is
uniform. The partial-image signature I was reasoning from was not real.

### The number

Inverting this driver's own logged frustum (`projectionraw values lrtb, near far:
-0.824583 0.681474 -0.895002 0.778395 | 0.039620`) against `rift.c:1802-1826`:

```
h_screen 119.34 mm   v_screen 66.30 mm   lens_sep/2 27.00 mm  eye_to_screen 39.62 mm
per-eye 59.67 x 66.30 mm -> aspect 0.9000  (1080/1200 = 0.9000 exactly)
```

The aspect landing exactly on 0.9000 confirms the inversion.

| | H FOV | V FOV |
|---|---|---|
| SteamVR-OpenHMD | **73.78 deg** | 79.73 deg |
| Monado | **79.02 deg** | 85.07 deg |

Exactly a **uniform tangent scale of 1.09905**. At 30 deg of head turn the two
stacks disagree by 2.13 deg of world motion.

Cause of the divergence: we use `display_info.eye_to_screen_distance` (39.62 mm)
as the panel-metres-to-tangent scale. Monado reads `OHMD_RIGHT_EYE_FOV`, which
`rift.c:1822` computes as **twice the outer half-angle** - treating an asymmetric
frustum as symmetric - and back-solves 36.05 mm to make the totals agree.

### Why this fits, where the timing hypotheses did not

Rendered FOV is the gain between head motion and world motion. Too narrow means
the world is magnified and over-rotates while the head turns, agreeing with
reality only at rest. That is uniform, exactly zero at rest, proportional to
speed, identical on headset and controllers (shared projection), and untouchable
by anything on the tracking side - which is every surviving observation, and
explains why nine tracking fixes and two timing fixes changed nothing.

### Ground truth: Oculus treats these as different quantities

`server-plugins/Rift.dll` serialises, per lens:

```
LensConfigurations[%d].LensToScreen
LensConfigurations[%d].MetersPerTanAngleAtCenter
LensConfigurations[%d].EyeRelief
LensConfigurations[%d].PerMMEyeShiftSwim = [7 coefficients]
```

`MetersPerTanAngleAtCenter` is the panel-metres-per-unit-tangent scale and is a
**separate field from `LensToScreen`**. We are using a physical distance where a
tangent scale is required, which is very likely the defect itself. Oculus also
carries an explicit per-lens **swim** model as a function of eye shift in mm -
they consider this artifact class real enough to correct per unit.

The values are per-headset (there is an "Unable to read lens serials for device"
path), so extracting the CV1's actual numbers is a further dig.

### Change

`ApplyFovPolicy()` in `driver_openhmd.cpp`, applied in `GetProjectionRaw`:

```
OHMD_STEAMVR_FOV_SCALE=<f>   multiply all four tangents by f (0.5..2.0)
OHMD_STEAMVR_FOV=monado      Monado's derivation, solved at runtime
(unset)                      unchanged
```

Default changes nothing until the A/B says which direction is right. The solver
reproduces 1.09905 and H 79.02 deg from the real constants, verified standalone.
It logs `FOV <eye> mode=<...> scale=<...> -> H .. deg V .. deg` every run, so an
arm can be checked from its own log - the two void experiments earlier tonight
were both cases of a test that could not verify itself.

### Measurement

PENDING - A/B `OHMD_STEAMVR_FOV=monado` against default. Falsifiable: if the
swim is unchanged at a 7.1% FOV difference, rendered FOV is not the gain term and
this dies like the others.

---

## THE PREMISE WAS FALSE: the two stacks never ran the same tracking code

### What was assumed

Every inference of the last several hours rested on one sentence: *"Bigscreen
feels right under Monado and wrong under SteamVR, with the same OpenHMD tracking,
so tracking is exonerated and the fault is in the SteamVR runtime."* That sentence
is wrong, and it was never checked.

- SteamVR compiles OpenHMD **into** `driver_openhmd.so` from `subprojects/openhmd`,
  rebuilt with the driver. It was at `5e8fd10`, built 07-31 19:54.
- Monado loads a **separately installed** shared library from
  `~/.local/openhmd-windows-parity/lib`, last built **07-29 21:27**.

Verified by content, not timestamp:

| string | Monado | SteamVR |
|---|---|---|
| `calibration matrix: mean row norm` | no | YES |
| `OHMD_RIFT_DEADRECKON_MS` | no | YES |
| `OHMD_RIFT_VEL_ADAPTIVE` | no | YES |
| `OHMD_RIFT_BLEED` | no | YES |
| `accelerometer scale check` | no | YES |

**Fourteen tracking commits** separate them:

```
5e8fd10 07-31 18:03 Keep the IMU calibration matrix's alignment, drop its scale
c23e60e 07-31 17:43 Check whether the accelerometer is actually scaled correctly
6029f53 07-31 16:39 Coast on the IMU through a vision gap instead of freezing the pose
82f0858 07-31 15:57 Log which frame angular velocity is exported in
1019457 07-31 15:44 Measure the prediction horizon
c3234bf 07-31 00:06 Measure how long an orientation error takes to wash out
2e8915d 07-30 23:57 Smooth the exported velocity by how fast the device is moving
e4eff5c 07-30 23:24 Fix the cold-start deadlock
4101fc4 07-30 22:30 rift: optional HID feature-report dump
f5830f4 07-30 22:12 rift: stop bleeding optical corrections into the displayed pose
6437fbd 07-30 21:53 rift: render stereo at the IPD the slider is set to
10a58b2 07-30 20:27 rift-cam-calib: recover from a moved sensor
5391124 07-29 21:32 rift: advertise the CV1 HMD as positionally tracked
```

### What this invalidates

The Monado-versus-SteamVR comparison varied **two** things at once: the runtime
*and* the tracking code. So it never showed what it was taken to show. The
correct reading of the same datum is the opposite one: **the 07-29 tracking feels
right and the 07-31 tracking does not**, and several of those commits touch
exactly the machinery that would produce move-overshoot-settle - exported
velocity smoothing, correction bleeding, IMU dead-reckoning, and a 3.5% change in
accelerometer scale.

The individual eliminations still stand on their own evidence (prediction really
does extrapolate 0.000 mm; frames really are 98% delivered; FOV really was set to
Monado's value and changed nothing). What does not stand is the conclusion that
the pose is innocent.

### The accident is now the instrument

We have a known-good binary (07-29) and a known-bad one (07-31), and Monado can
load either with the runtime held constant. That is the controlled A/B that was
never run. Preserved as
`~/.local/openhmd-windows-parity/lib/libopenhmd.so.0.1.0.known-good-0729`; the
current build is installed alongside.

Next: Bigscreen under Monado on the CURRENT tracking. If it now feels wrong, the
artifact is in those fourteen commits and bisects in ~4 runs. If it still feels
right, the runtime difference is real and the search resumes there - but with the
tracking finally equalised.

**Method note, and the third instance tonight:** a comparison is only worth what
its controls are worth. The bin/linux64 staleness, the HMD-only prediction knob,
and now this, are all the same failure - the experiment did not verify that the
thing it claimed to vary was the only thing that varied.

### The controlled A/B finally ran, and it is positive

Bigscreen, under Monado, with the runtime and application held constant and only
the OpenHMD library swapped:

| tracking | result |
|---|---|
| 07-29 build (`libopenhmd.so.0.1.0.known-good-0729`) | feels right |
| 07-31 build (`5e8fd10`) | **same artifact** |

Control verified before judging: `client_connected ... application_name:
'Bigscreen'` present, and the session log carries `output correction bleeding
OFF`, a string that exists only in the new build.

**The artifact follows the tracking code, not the runtime.** It is in the fourteen
commits between those builds.

Two filters narrow it. Monado never reports linear velocity and never
extrapolates, so any commit that only changes *exported* velocity cannot be
responsible - that removes `2e8915d`, `82f0858`, `1019457`. Only pose-changing
commits qualify: `5391124`, `10a58b2`, `f5830f4`, `e4eff5c`, `6029f53`, `5e8fd10`.
Three of those have runtime switches, so they A/B without a rebuild:
`OHMD_RIFT_BLEED=1`, `OHMD_RIFT_DEADRECKON_MS=70`, `OHMD_RIFT_NO_CALIB_ORTHO=1`.

Live clue pointing at the last of those: the new build's own telemetry reads

```
accelerometer scale check: |accel| at rest mean 9.4340 m/s^2 (-3.80%)
                           9.5086 (-3.04%)   9.5421 (-2.70%)
```

where `5e8fd10` was measured to land it at **-0.50%**. A ~3% scale error is
phantom acceleration, and the replay harness put 3.5% at triple the overshoot and
quadruple the post-stop drift. The fix is not achieving what it was measured to
achieve on live hardware.

### Correction: the FOV A/B was impure

The user reports that under `OHMD_STEAMVR_FOV=monado`, "stuff on the edge curved
weirdly". That is my error: `ApplyFovPolicy` scales the four projection tangents,
but `ComputeDistortion` builds its mesh from the lens constants independently and
was left untouched. Widening the frustum 9.9% against a distortion mesh tuned for
the original one warps the periphery.

The central finding survives - the move-overshoot-settle artifact was unchanged,
which is what the test was for - but the experiment introduced a second artifact
and should not be cited as a clean test of field of view. It also establishes that
the distortion mesh is self-consistent with the present 73.78 deg frustum, so any
future FOV change must port the matching distortion or it will look worse whether
or not the new FOV is correct.

The override defaults to unset and changes nothing unless asked, so it does not
contaminate the tracking bisection now in progress.

## THE ANSWER: the "good" configuration was 3DOF

`OHMD_RIFT_NO_CALIB_ORTHO=1 OHMD_RIFT_BLEED=1 OHMD_RIFT_DEADRECKON_MS=70` -
verified applied (`correction bleeding ON`, accelerometer back to its pre-fix
+2.51%) - did not change the artifact. That eliminates correction bleeding, the
calibration ortho and the dead-reckoning window.

Reading the remaining candidates found it immediately. `5391124`, the first
commit after the known-good build, says so in its own message:

> Monado's OpenHMD driver defaults every device to 3dof and only makes the HMD
> 6dof when this flag is set, so the entire constellation solve was being
> discarded and replaced with a neck model.

Confirmed in `monado/src/xrt/drivers/ohmd/oh_device.c:1263`:

```c
// Default everything to 3dof (NONE), but 6dof when the HMD supports position tracking.
ohd->base.supported.position_tracking = (device_flags & OHMD_DEVICE_FLAGS_POSITIONAL_TRACKING) != 0;
```

**The 07-29 library did not set that flag, so every "Monado feels right" report was
a 3DOF headset on a neck model.** A 3DOF headset has no positional error, no
vision corrections and no settle, by construction.

So the artifact is not a regression, and not the runtime. It is **CV1 positional
tracking**, switched on for Monado at 07-29 21:32 and always on under SteamVR -
which is why the complaint dates from the first message of the session.

### The second measurement blindness

`tools/find_oscillation.py` removes smooth motion by fitting a local quadratic
over a **0.25 s** window. Anything settling more slowly is absorbed into "real
motion" and cannot appear in the residual. The position observer's poles are a
complex pair at 1.07 Hz plus a **real pole at -0.559, tau = 1.79 s**.

A 1.8 s settle is therefore structurally invisible to the tool used to declare
the pose clean at 0.64 mm - and "I move and it overshoots, gets back too far,
until it reached the real point" is a description of precisely that. The 1.79 s
pole was on the candidate list early and was dismissed on the strength of a
measurement that could not see it.

Both of tonight's blind spots have the same shape: a metric that excluded the
band the symptom lives in, then treated silence as absence.

### Where to go next

The question is now narrow, positional, and offline-measurable: **what does the
position estimate do in the 0.2-3 s after motion stops?** Not the residual after
smoothing - the trajectory itself, against a step input.

1. `tools/fusion_replay.c` already drives the real `rift-fusion-ovr.c` with
   synthetic profiles and reports overshoot and settling. Re-run the
   `--profile translate` stop test and read the tail out to 3 s, with no
   smoothing window applied.
2. Re-analyse the existing captures with a smoothing window of 3-5 s (or none),
   which is the change that makes the slow mode visible in data already on disk.
3. Then the gains: `GAIN_POS(10,10,8) / GAIN_VEL(50,50,32) / GAIN_ACCEL(25,25,16)`
   put a real pole at tau 1.79 s. Whether that is the artifact is now a
   measurement, not a guess.

---

## Step 1: quantify the settle. Instrument built; result not yet conclusive

### Harness: the accel scale error matters, but not by the mechanism I proposed

`tools/fusion_replay.c`, real fusion code, translate profile at 1.2 m/s:

| accel scale | overshoot | settle | bias peak | bias at end |
|---|---|---|---|---|
| 1.000 (perfect) | 2.60 mm | 390 ms | 0.0389 | **0.0003** |
| 0.975 (-2.5%) | 6.78 mm | 442 ms | 0.2428 | 0.2428 |
| 0.970 (-3.0%) | 7.62 mm | 444 ms | 0.2914 | 0.2914 |
| 0.962 (-3.8%) | 8.97 mm | 446 ms | 0.3691 | 0.3691 |

With a perfect accelerometer the slow mode barely excites and decays to zero, so
**the tau = 1.79 s pole alone is not a symptom** - it needs a forcing term. A 3%
scale error triples the overshoot.

But the `turn` profile shows **0.00 mm overshoot at every scale**, with
`bias_end` identical to the translate case. The bias state is held in the **body
frame**, where a scale error is constant, so rotation never forces it to
re-converge. **The mechanism written into the plan - rotation moves the bias
target, 1.8 s re-convergence - is wrong.**

What survives is simpler: a scale error scales *real* acceleration, injecting
error proportional to how hard you accelerate. Still motion-proportional and zero
at rest, but not a rotating bias.

### New metric, and three bugs in it worth recording

`tools/settle_profile.py` measures displacement from the stop point over a fixed
window with no speed threshold terminating it. Getting it right took three fixes,
each of which had produced confident nonsense:

1. **Quiet defined against FAST rather than SLOW.** 0.4 m/s sustained for 3 s is
   1.2 m of travel; `p_final` landed somewhere unrelated and the journey was
   reported as a settle - 107 mm at lag 0, non-monotonic.
2. **Speed differenced between adjacent samples.** At 810 Hz, 0.5 mm of noise
   across a 1.2 ms gap is 0.4 m/s, so every real stop looked like motion and zero
   events were found. Now computed over a fixed 50 ms base, which also makes
   captures at different rates comparable.
3. **Forward "fast then slow" scan.** During a settle the speed dithers across
   SLOW repeatedly and the first failing dip discarded the deceleration that
   caused it. Now finds quiet runs first, then asks which were preceded by
   movement.

### Preliminary numbers - NOT yet a result

Window 1.5 s, distance still to travel after the stop:

| capture | events | at stop | 0.5 s | 1.0 s | fitted amplitude | fitted tau |
|---|---|---|---|---|---|---|
| Oculus runtime | 3 | 11.2 mm | 10.8 | 7.4 | 14.2 mm | 0.85 s |
| ours `predold2` | 1 | 20.0 mm | 8.9 | 2.7 | 23.8 mm | 0.47 s |
| ours `osc` | 1 | 24.0 mm | 14.2 | 9.8 | 26.7 mm | 0.88 s |

Ours settles roughly **1.7-1.9x further** than the Oculus runtime, which is the
direction expected. But **n = 1 against n = 3**, so this is an indication, not a
measurement, and it must not be quoted as one.

Two further points against the plan's headline: the fitted tau is 0.5-0.9 s, not
1.79 s, so the slow bias pole is not visibly dominating; and the Oculus runtime
has a 14 mm settle of its own, so the target is not zero.

### What is actually blocking

Every capture on disk was made for a different experiment and contains almost no
clean stops - `osc.csv` has **one** in 60 s, `predold2` **one** in 26 s. The
motion was continuous by design. Nothing more can be concluded without a capture
made for this question: repeated fast movements each followed by a deliberate
2-3 second hold, which yields 15-20 events instead of one.

### Step 1 verdict: the positional settle is NOT the artifact

Purpose-made capture, headset moved and set down on a desk (true stillness, no
human sway, 14 clean stops in 60 s):

| | events | at stop | 0.5 s | 1.0 s | 1.5 s | amplitude | tau |
|---|---|---|---|---|---|---|---|
| Oculus runtime | 3 | 11.20 mm | 10.76 | 7.42 | 1.49 | 14.21 mm | 0.85 s |
| ours | 14 | 9.25 mm | 5.86 | 2.50 | 0.83 | **13.84 mm** | **0.53 s** |

**Ours is equal or better than the reference on every measure.** The fitted tau is
0.53 s, not 1.79 s, so the slow bias pole is not visible in real data either.

Caveats, both real: their capture is head-worn and ours is desk-mounted, so the
protocol that made our measurement clean also made it flattering; and their n=3.
Neither caveat rescues the hypothesis - a settle that is already at parity cannot
be what makes one configuration feel broken and the other fine.

**The plan's premise is falsified by its own step 1, as intended.** Retuning the
observer gains would be optimising something already at reference parity.

### What this leaves, and it fits better than anything so far

The desk test exercises **translation only**. The artifact is felt while wearing
the headset and turning the head, and it vanishes in 3DOF - a mode that has no
position at all, and therefore no rotation-to-translation term.

That points at the **lever arm**: the tracked point is not the eye. The driver's
own log reports `HMD device frame: IMU at [-0.0166 0.0345 0.0331] m`, and
`driver_openhmd.cpp` leaves `vecDriverFromHeadTranslation` at **zero** for the
HMD, so SteamVR places the eyes exactly on OpenHMD's tracked origin. A 3 cm error
in that offset produces `r x omega` = 0.03 * 1.57 = **47 mm/s** of spurious
translation during a modest 90 deg/s head turn.

That is zero at rest, proportional to turn rate, uniform across the image,
identical on headset and controllers, invisible to every pose-quality metric
(the pose is *correct* for the point it describes), and structurally absent in
3DOF. It matches every surviving observation.

This was raised earlier in the project and dropped on an argument rather than a
measurement: `windows-vs-linux-tracking.md:1043` reasons that the tracked origin
"sits 74 mm behind the visor face - about where a CV1 wearer's eye actually is,
so zero is approximately right". "Approximately right" is exactly the kind of
claim this artifact would hide behind.

**Next test, and it is a desk test too:** rotate the headset in place about a
known point and measure how far the reported position moves. If the pivot is
modelled correctly, spinning about the tracked origin should produce near-zero
translation; whatever it does produce is the lever-arm error, in mm, directly.

---

## BISECT RESULT: the overshoot enters at `b665454`

After twelve failed hypotheses, going back to pristine upstream and walking
forward found it in five runs. Harness: Monado (loads libopenhmd dynamically, so
a point is one file copy), Bigscreen, `tools/bisect_openhmd.sh`. Every point
carried two constants so neither became a variable: the OpenCV-5 build fix, and
`5391124` for the positional flag - without which Monado silently runs 3DOF,
the false "feels perfect" that cost the earlier part of the evening.

| point | commit | overshoot |
|---|---|---|
| A | `04f5276` pristine upstream | **no** (heavy vibration) |
| 6 | `bd1e9f9` all room-config work | **no** (light vibration) |
| 7 | `b665454` 2026-07-12 tracking session | **YES** |
| 8 | `b5ea958` obs merge -> orientation | yes |
| 10 | `7ad4c30` extrinsic refine, vision tilt | yes |
| 20 | `2266430` cam-calib auto-recovery | yes |
| 40 | `5e8fd10` HEAD | yes |

`b665454` is the commit. Every point was content-fingerprinted before its run and
every run was confirmed to have live positional tracking, so unlike the earlier
comparisons this one has controls.

### Two things established on the way

**Upstream's vibration is real and ours fixed it.** At point A both cameras were
placed with `gravity error 25.000000 degrees` - exactly the `MIN_ROT_ERROR` clamp,
i.e. the cold-start deadlock - so the extrinsics were badly conditioned and the
two sensors' fixes disagreed. That is the 7.5 Hz rest-wander, and it fades from
point 6 onward. The work in this repo demonstrably fixed something.

**A prediction I made was wrong and is worth recording as such.** Before the #8
run I named `f9b88f6` (IMU calibration offset convention) as the suspect on the
strength of the accelerometer telemetry. The bisect put the cause four commits
earlier, in code I had already reasoned about and not suspected.

### What is inside `b665454`, and what can still be excluded

Under Monado two of its changes cannot matter: the exported-velocity EMA and
`rift_predict_pose` only affect consumers that read velocity or request
prediction, and Monado does neither. That leaves:

- the **same-exposure observation merge** (position corrected toward the
  confidence-weighted mean of an exposure's observations, weights 1/obs_scale^2)
- the **rewritten gravity gate**
- `vision_fix` taking **`replace_pending`**

The first two have kill switches, so narrowing needs no rebuild. Neither logged
its state, so a log line was added first - the same defect that made a previous
A/B in this project unverifiable from its own output, noted in
`windows-vs-linux-tracking.md` and repeated twice tonight.

**Leading candidate, stated before the test:** the merge changed position from
"take the latest camera's fix" to "converge toward a weighted mean of the
exposure's observations". That is exactly the change that stops the cameras
fighting at rest - which it verifiably did - and it is also the kind of change
that would make the pose approach its target gradually during motion rather than
snapping to it.

## SOLVED: the OVR complementary fusion backend is the cause

`OHMD_RIFT_FUSION=ukf` at **current HEAD** - all 40 commits present - removes the
overshoot. Verified from the log before judging: `Device 0 using UKF fusion
backend`, `Now tracking`, 7 sensor placements.

`b665454` did not tune the fusion. It **added a second one**:
`rift-fusion-ovr.c | 470 +++++` and `rift-fusion-ovr.h | 88 +++`, both pure
additions - a port of the Oculus SDK 0.3.2 complementary filter - and made it the
default, displacing thaytan's UKF (`rift-kalman-6dof.c`).

That new file is exactly where `GAIN_POS(10,10,8) / GAIN_VEL(50,50,32) /
GAIN_ACCEL(25,25,16)` live, whose poles were computed earlier tonight:

| axis | poles |
|---|---|
| X, Y | -4.72 +/- 4.74j (1.06 Hz, zeta 0.71) and **-0.559, tau 1.789 s** |
| Z | -3.71 +/- 3.73j (0.84 Hz, zeta 0.71) and **-0.577, tau 1.732 s** |

The pole analysis was right about the mechanism and wrong about the scope: the
question was never "are these gains mistuned?" but "why is this filter running at
all?". The UKF has no such observer and does not overshoot.

Also falsified along the way, with the switch verified applied
(`same-exposure obs merge OFF (OHMD_RIFT_NO_OBS_MERGE=1)`): the obs merge is not
the cause. It was named as the leading candidate before the test, and was wrong -
as was `f9b88f6`, named one run earlier.

### Confirmed A/B, one variable, both ends verified

| config | fusion | overshoot |
|---|---|---|
| HEAD | OVR complementary (default) | yes |
| HEAD + `OHMD_RIFT_FUSION=ukf` | UKF | **no** |

Everything else from the 40 commits is unaffected and stays: cold-start fix,
same-exposure obs merge, gravity gate, auto-placement, room config, calibration.
Those live outside the fusion, and the vibration fix they provide is retained -
point A's `gravity error 25.000000 degrees` shows what happens without them.

### Open, and now well-posed

1. **Default.** The OVR backend should not be the default while it does this. A
   one-line change, but the choice deserves the measurements below rather than a
   reflex.
2. **Which is actually better?** The UKF removes the overshoot; whether it is
   worse on latency, rest jitter or dropout recovery is unmeasured. Both backends
   can now be scored offline with `tools/settle_profile.py` (14-event desk
   capture) against the Oculus runtime's 14.21 mm / tau 0.85 s.
3. **Or fix the complementary filter.** Its slow real pole comes from `Ka` being
   low relative to `Kp`/`Kv`: for `Kp=10`, placing all three poles together wants
   `Kv=33.3, Ka=36.9` against the shipped `50 / 25`. Worth testing, since the
   complementary filter was ported for a reason.
4. The **edge cropping** the user reports is independent of all of this - it
   appears at every bisect point and no lens/screen constant changes across them.
   Monado also forces the vertical lens centre to 0.5 with `//! @todo This are
   probably all wrong!`. Separate thread.
