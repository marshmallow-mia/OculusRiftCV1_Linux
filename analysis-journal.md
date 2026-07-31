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
