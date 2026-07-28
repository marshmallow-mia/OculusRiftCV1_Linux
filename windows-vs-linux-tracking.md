# Windows Oculus runtime vs. our Linux stack — tracking differential

**2026-07-28, second pass.** Supersedes the first pass (same filename) and the
"their tracking code is unavailable" assumption in `cv1-capture-plan.md`,
`HANDOFF-LINUX.md` and `findings-4a-hid.md`.

Companion artifacts: `decomp/rift-dll-classes.txt` (147 demangled `OVR::*`
classes), `decomp/rift-dll-tracking-strings.txt` (627 tracking log strings),
`decomp/rift-dll-functions.md` (function map + recovered constants).

---

## 0. Where their code is

```
C:\Program Files\Oculus\Support\oculus-runtime\server-plugins\Rift.dll   (5.6 MB)
```

Not `OVRServer_x64.exe` — that is the Link/compositor service and has no vision
pipeline at all (no `OVR::Vision` RTTI, no camera transport, no blob/PnP; only
IPC stubs carrying `ovrCameraIntrinsics_`/`ovrCameraExtrinsics_` for
passthrough). Searching it and finding nothing is what produced the belief that
Meta had stripped CV1 tracking. The runtime is plugin-structured; CV1 lives in
`server-plugins/Rift.dll`, Rift S in `RiftS.dll` + `RiftS/RiftSTracker.dll`,
Quest/Link in `RemoteHeadset.dll`.

It is a release build **with RTTI and diagnostic format strings intact**, so the
estimator's state vector, its reset taxonomy and its acceptance thresholds are
recoverable. Claims below are tagged **[rtti]**, **[diag]** (log format string),
**[decomp]** (constant or control flow read out of the binary), or **[ours]**
(our source, with file:line).

---

## 1. Their pipeline

```
TrackedObject state machine   ── "status c%d,e%d,a%d, was c%d,e%d,a%d, pose %s"
  └─ reconstruction driver    ── "Failed frame: object %d, count %d, Reprojection error: %.3f pix"
       ├─ JOINT MULTI-CAMERA RECONSTRUCTION
       │     "Pose reconstruction failed: count %d, error = %.3f, cameras %d, matches %d, distance %.1f"
       │     "Successful reconstruction after %d failures: object %d, cameras %d, used %d %d %d %d"
       ├─ RANSAC correspondence match
       │     "RansacMatch: Too many outliers: %d outliers out of %d, allowed %d"
       │     "Outlier %d: %.3f (imp %.0f%%) %d of %d >%.3f, worst %d %.3f"
       ├─ back-of-head group reconstructed SEPARATELY (same routine)
       │     "Back of head reconstruction failed. Reprojection error: %f"
       └─ single-camera fallback
             "Object %d using fallback camera %d, cameras %d"
  └─ IndirectEkf<18,9>        ── pose measurement in, 18 error states
  └─ Gravity aligner          ── per-camera "up", filtered, with confidence
  └─ Camera calibrator        ── online bundle adjustment, Settled/Unsettled
```

`used %d %d %d %d` is four per-camera counters, and `fcn.18010cac0` (the
reconstruction) is called from both the main-body and back-of-head paths
**[decomp]**. The EKF vision update `fcn.18013a920` is called only from the EKF
driver `fcn.180134cd0` **[decomp]** — so the vision side produces **one pose per
object per exposure**, and the EKF consumes it as a pose measurement. That is
architecturally the same shape as ours. The difference is entirely in *how that
one pose is produced* and *what the filter estimates around it*.

---

## 2. Delta 1 — the vision solve

### Theirs

One reconstruction pooling correspondences from every camera that saw the object
in that exposure, accepted at **2 px reprojection error** (`715` focal px and
`2/715` normalised, `fcn.18017f370` **[decomp]**), with RANSAC outlier rejection
and an iterative outlier-removal loop that reports the *improvement* each removal
buys (`imp %.0f%%`). Falls back to a single camera only when the joint solve
fails. Tracks the CV1's back-of-head LED group as a **separate rigid body**.

### Ours

Per camera, independently **[ours]**:

- `rift_pose_finder_process_blobs_fast` → prior scoring, `estimate_initial_pose`
  (`rift-sensor-opencv.cpp:37`) → `cv::solvePnPRansac`, threshold
  `3.0 / camera_matrix.m[0]` = 3 px (`rift-sensor-opencv.cpp:131`).
- `correspondence_search.c` brute-force P3P (lambdatwist, `:437`) for reacquire.
- Result converted to world with that sensor's extrinsics
  (`rift-sensor-pose-search.c:597`) and handed to the tracker
  (`rift-sensor.c:677`).
- The tracker merges *poses* per exposure — confidence-weighted mean, weights
  `1/obs_scale²` (`rift-tracker.c:1438-1487`).

So we do have RANSAC — but **per camera, over one camera's blobs**. There is no
stage anywhere that sees two cameras' blobs at once.

### Why this is the biggest gap

Our own measurements say the per-camera solutions are *biased*, not merely noisy:
Touch on a desk **86 mm / 4.2–4.7°** between cameras; HMD on the floor **111 mm**
in Y; HMD at head height 0.4 mm / 0.22°. Averaging two biased estimates yields
the average bias. The bias is PnP depth ambiguity on a small LED ring seen from a
steep angle — exactly the degeneracy a second viewpoint removes, but only if both
viewpoints enter the *same* solve.

### MEASURED, 2026-07-28 — `tools/replay_recon.py`, offline, no hardware

Replaying the recorded captures through today's logic (each sensor's own solve,
then the tracker's `1/obs_scale²` weighted merge) versus a joint solve over the
pooled blobs of both cameras with the extrinsics held fixed. Extrinsics from
`~/.config/openhmd/rift-room-config.json`, i.e. the current solved room.

| | `ctrl-obs.jsonl` — Touch L, 1199 exposures | `final-verify.jsonl` — HMD on floor, 1156 exposures |
|---|---|---|
| cross-camera disagreement | 12.76 mm / 0.97° | 5.50 mm / 1.08° |
| reproj, own solve in **own** camera | 0.090 px | 0.122 px |
| reproj, own solve in **other** camera | 5.65 px | 0.719 px |
| reproj, **merged** pose (today) | 2.83 px | 0.372 px |
| **reproj, joint pose, worst camera** | **0.662 px** | **0.257 px** |
| joint pose vs merged pose | 15.7 mm | 2.6 mm |
| **frame-to-frame step, merged (today)** | 0.490 mm | 1.377 mm |
| **frame-to-frame step, joint** | **0.045 mm** | **0.205 mm** |
| passes Oculus's 2 px acceptance | **100 %** | **100 %** |

Three conclusions:

1. **The joint solve reaches Oculus's own acceptance bar on every single
   exposure**, on real recorded data, on both the controller and the headset.
2. **Frame-to-frame jitter falls 11× (Touch) and 6.7× (HMD)** — 0.490 → 0.045 mm
   and 1.377 → 0.205 mm. This is on top of the same-exposure merge fix already
   deployed, and it is the rubber-banding/wander the user sees.
3. **The merged pose is systematically wrong**, not merely noisy: the joint pose
   sits 15.7 mm (Touch) and 2.6 mm (HMD) away from it. Averaging two poses that
   each fit only their own camera lands between two right answers to the wrong
   question.

**The joint reprojection error is also an extrinsic-quality detector.** Run
against each capture's *embedded* `campose` — the extrinsics the driver was
actually using when the capture was recorded, before the room was re-solved —
no single pose can satisfy both cameras at all:

| capture | extrinsics | disagreement | joint worst-camera reproj | passes 2 px |
|---|---|---|---|---|
| `ctrl-obs.jsonl` | capture-time `campose` | 86.3 mm / 4.67° | 18.3 px | 0 % |
| `final-verify.jsonl` | capture-time `campose` | 116.9 mm / 4.94° | 28.7 px | 0 % |
| `cal-capture.jsonl` | capture-time (pre-recal) | 568 mm / 155° | — | 0 % |

That is exactly the distinction Oculus draws — `"Bad calibration for camera %d,
object %d: reprojection err %.2f, tilt err %.2f"` **[diag]**. A joint residual
that cannot be driven below a few px means the *extrinsics* are wrong, not the
pose; a residual that collapses to sub-px means they are right. This project has
never had that test, and `controller-tracking-analysis.md` explicitly warned that
steep-view PnP ambiguity masks extrinsic damage. It no longer does.

(Bundle-adjusting `ctrl-obs.jsonl` moves sensor 1 by 20.7 mm / 0.98° and takes
the disagreement to 0.71 mm / 0.13°, so even the current config is slightly stale
for that capture — which is what §5's online calibration is for.)

**Not yet implemented in the prototype**: RANSAC and the iterative outlier
removal. The captures contain only LED-ID-verified correspondences, so gross
outliers are rare in them; both are required for the C port, where the
correspondence set is not pre-filtered.

### C implementation — `rift-joint-pose.{c,h}`

Gauss-Newton on the 6-DoF pose with a Huber loss and an analytic Jacobian, no
OpenCV and no external solver, residuals in undistorted normalised rays (the
space Oculus's `2/715` threshold implies). Unit tests in
`tests/unittests/joint_pose.c`, runnable standalone with
`tools/run_joint_tests.sh`.

**Cross-checked against the Python prototype on real recorded data**
(200 co-observed Touch exposures, `replay_recon.py export-c` → C solver):

| | |
|---|---|
| exposures solved | 200 / 200 |
| passing the 2 px bar | 100 % |
| C vs Python **position** | median 0.235 mm, max 0.237 mm |
| C vs Python **rotation** | median 0.097°, max 0.105° |

The 0.235 mm offset is systematic (median ≈ max), not numerical noise: Python
minimises in *distorted pixel* space, C in *undistorted normalised ray* space,
and the fisheye Jacobian weights points differently between the two. Blob
centroid noise is isotropic in pixels, so pixel space is the statistically
correct one — but normalised space is far cheaper in the driver (no fisheye
forward projection or its Jacobian per iteration), and 0.235 mm is two orders
below the 15.7 mm error it removes. Worth revisiting only if sub-mm accuracy
becomes the binding constraint.

### How much extrinsic error does the 2 px bar actually allow?

Measured by corrupting a synthetic rig (`rift-joint-pose.c` notes,
reproducible with the probe in the unit test):

| corruption of one camera | worst-view reprojection |
|---|---|
| 0.05° rotation | 0.21 px |
| 0.40° rotation | 1.71 px |
| 1.00° rotation | ~4.3 px |
| 16 mm translation | 0.99 px |
| 32 mm translation | 2.00 px |

So **~4.3 px per degree, ~1 px per 16 mm**, and Oculus's 2 px acceptance
corresponds to roughly **0.47° or 32 mm** of extrinsic error. A pure
translation is ~77 % absorbed by moving the object, which is why rotation is
much the stronger signal — and why the stale-extrinsics captures above land at
18–29 px: that is several degrees of rotational error, not a bumped position.

### What it would take

The rendezvous already exists structurally. Both sensors' frames for one exposure
receive an identical `rift_tracker_exposure_info` (copied by value,
`rift-tracker.c:798`), and the per-device delay slot
(`rift_tracker_pose_delay_slot`, `rift-tracker.c:104`, under `dev->device_lock`)
is already the cross-sensor meeting point — it currently carries
`posef` + `rift_pose_metrics` per sensor and nothing else.

The obstacle is blob lifetime and ownership: blobs live in each sensor's
`blobwatch` ring of 3 observations (`rift-sensor-blobwatch.c:92-105`), are
borrowed by the frame (`rift-sensor.h:48`), and are recycled in
`release_capture_frame` under `sensor_lock` (`rift-sensor.c:192-195`).
Correspondences are not stored as a list at all — they exist only as the
`led_id` field written into each blob (`rift-sensor-blobwatch.h:57`). Sensors are
opaque to one another (`rift_sensor_ctx` is an incomplete type outside
`rift-sensor.c`).

So a joint solve needs a new per-exposure, per-device correspondence buffer —
`{sensor_id, camera_pose, [(led_index, undistorted_ray)]}` — published into the
delay slot alongside the existing pose report, and a solver that minimises
reprojection across all of them with the extrinsics fixed. Note the existing
`FIXME` at `correspondence_search.c:297-298`: the scorer works in *distorted
pixel* space while the solvers work in *undistorted normalised* space, so the new
buffer should carry undistorted rays and the joint residual should be computed in
normalised units (as Oculus's `2/715` implies theirs is).

---

## 3. Delta 2 — the estimator

`OVR::Vision::IndirectEkf<18, 9>` **[rtti]**, an error-state EKF. The states are
enumerated verbatim **[diag]**:

```
%d: Initial State: AccBias %.2f, %.2f, %.2f (%.2f) G, GyroBias %.2f, %.2f, %.2f (%.2f) deg/s, Gravity %.3f
%d: Update: AccBias …, GyroBias …, GravAlign %.1f (axis %.3f, 0, %.3f), G %.3f
```

| block | dim |
|---|---|
| position, velocity, orientation | 9 |
| accelerometer bias | 3 |
| gyro bias | 3 |
| gravity alignment (rotation about a *horizontal* axis — the printed y-component is structurally 0) | 2 |
| gravity magnitude | 1 |

The gravity prior constant `9.80667` appears in the update function, and the
gravity-correction function `fcn.1801376b0` carries an acceptance band of
**9.71 … 9.91 m/s²** **[decomp]** — they estimate |g| and gate it.

### Ours

| | `rift-fusion-ovr.c` (**default**) | `rift-kalman-6dof.c` (`OHMD_RIFT_FUSION=ukf`) | theirs |
|---|---|---|---|
| type | complementary filter, SDK 0.3.2 | UKF, 19 state / 18 cov | indirect EKF, 18 error states |
| gyro bias | **none** | yes | yes |
| accel bias | **none**¹ | yes | yes |
| gravity direction | fixed world +Y | fixed world +Y | **2-dof state** |
| gravity magnitude | fixed `9.8f` | fixed `9.80665` | **state, gated 9.71–9.91** |
| measurement noise | scalar tiers 1.0/1.5/2.0/4.0/6.0 | same tiers → isotropic diagonal R | per-observation |
| saturation | none | none | sigma inflation + counters |
| resets | snap >0.1 rad/>0.1 m, reacquire >500 ms | none | 11 named causes, Hard/Soft |

¹ Confirmed: `rift_fusion_ovr::accel_offset` is fed **only** by the vision
position residual (`GAIN_ACCEL {25,25,16}`, `rift-fusion-ovr.c:306-328`),
expressed in the **world** frame, zeroed on every snap, never decayed. It is the
integral term of a position servo, not an accelerometer-bias observer. The header
comment calling it "world-frame accelerometer bias integral"
(`rift-fusion-ovr.h:36`) is misleading.

### The gravity assumption, enumerated **[ours]**

- `rift-kalman-6dof.c:482` — `MATRIX2D_Y(X, STATE_ACCEL+1) + GRAVITY_MAG`, the
  *only* gravity model: direction and magnitude both hardcoded.
- `rift-fusion-ovr.c:22, 222, 363` — `9.8f`, `up = {0,1,0}`, `accel_world.y -= GRAVITY_MAG`.
- `rift-sensor.c:705` and `rift-sensor-pose-search.c:562` — the gravity vector
  fed to the correspondence gate is a literal `{0,1,0}`.
- `rift-tracker-config.c:78-80, 200` — the room config's only rotational DOF is a
  scalar `room-yaw-offset` about world +Y. **A non-vertical room frame cannot be
  represented at all.**

Theirs, by contrast, keeps an `alignment_transform_pos` /
`alignment_transform_rot_ypr` with `num_alignment_cameras`, `is_gravity_aligned`,
`was_gravity_aligned` **[diag]** — a full pose, per camera, plus state.

Every gravity bug this project has fought — the 154° world flip, the roll-broken
gravity gate, the accel-locked Touch tilt — is a consequence of holding fixed what
they hold as state.

---

## 4. Delta 3 — the gravity aligner (a subsystem we simply do not have)

**[diag]**, all from `Rift.dll`:

```
DoGravityAlignment cam %d, aligned %d %d
Computed gravity alignment: camera %d:%s, tilt: calculated %.2f %.2f (%.2f), filtered %.2f %.2f (%.2f),
    delta %.2f %.2f (%.2f), tilt err %.3g deg, elapsed %.3f, wasAligned %d, numAlignmentCameras %d (%d)
Gravity filter update: change: %.2f/%.2f, weight %.2f/%d, conf %.2f/%.2f/%.2f,
    reliable %d/%d, large %d, wasAligned %d, numAlignmentCameras %d, forced %d
EstimatedUpInCamera camera %d, object %d, count %d: tilt %.2f %.2f (%.2f)
HMD: tilt %.1f, %.1f, %.1f (%.1f), confidence %.3f, count %d
Gravity aligner not aligned: alignment camera %d
Can't calibrate: don't have a fixed camera: gravity aligned %d: alignment camera = %d
Inclinometer %d: Correcting orientation: %.1f deg misalignment (uncertainty %.1f)
Warning: too many IMU samples between camera frames for gravity alignment
```

Recovered thresholds **[decomp]**: change/large/confidence tiers at **0.3°, 1.5°,
3°, 5°** (`fcn.180146860`); alignment tilt tolerance **0.5°** (`fcn.180141610`);
"filter stuck, too much movement" at **0.0076 rad ≈ 0.435°** std
(`fcn.18011ab20`); gravity correction clamped in steps of ~0.005°–0.098°
(`fcn.1801376b0`).

Read together: they estimate the world "up" **in each camera's frame**, from the
tracked object's own IMU gravity, filtered with an explicit confidence and a
reliability count, nominate one camera as the *alignment camera*, and refuse to
calibrate the rig until it is aligned. Our equivalent is a human running
`calibrate_room.py setup` and hoping the rig has not moved.

---

## 5. Delta 4 — online camera calibration (bundle adjustment)

**[rtti]** `OVR::Vision::CameraCalibrator::DoCalibration(int) → CalibrationResult`.
**[diag]**:

```
Multicam calibration computed: residual %.3f --> %.3f pix, sample counts: %d, %d, %d %d, num cameras: %d
Insufficient Multicam calibration improvement %.1f%%: %.3f/%.3f > %.3f
Multicam calibration error too large: %.3f/%.3f > %.3f
FastCalibrate: camera %d, fixed %d, poses %d, residual %.3f
FastCalibrate: Camera moved
FastCalibrateFromHistory time: %.2f msec
Good/Bad calibration for camera %d, object %d: reprojection err %.2f, tilt err %.2f
Fixed camera %d alignment changed during optimization: translation %.1f, rotation %.2f
Camera calibration data is from an old version of the bundle adjustment tool. It needs to be re-calibrated via bundle adjustment.
Camera Calibration Settled after alignment / after calibration. / Unsettled.
```

Improvement gate **0.85** — a new solve must cut the residual by ≥15 % or it is
rejected **[decomp]**.

We do have the offline analogue: `calibrate_room.py solve` is a real bundle
adjustment (`scipy.optimize.least_squares` over LED reprojection with the
reference camera fixed). What we lack is (a) it running *online*, (b) the
Settled/Unsettled state, and (c) telling the estimator when the extrinsics moved
— they emit `Ekf CameraPoseChange: dt … dr …` and then `Ekf Reset on
CameraPoseChange` **[diag]**, whereas
`rift_tracker_extrinsic_refine_apply` (`rift-tracker.c:1898`) rewrites the sensor
pose silently under a running filter whose state was conditioned on the old one.

---

## 6. Delta 5 — robustness the estimator has and ours does not

**[diag]** reset taxonomy: `Hard ResetEkf`, `Soft ResetEkf %d %d`, plus causes —
large pose update, camera pose change, freeze, too few matches, integrate-forward
failure, invalid reset sample time, filter empty, no velocity estimate, IMU first
sample, inclinometer aligned from gravity-aligned sample. Reset sigmas are logged
(`Ekf Reset: Sigmas: pos %.1f, vel %.1f, orient %.2f`; constants `2.5e-5` and
`9e-6`, i.e. 5 mm and 3 mm variances, `fcn.180132a20` **[decomp]**).

**[diag]** saturation is a first-class state:

```
%d: Begin Gyro saturation: %.4f, orient sigma %.2f      %d: Gyro saturation: %d samples, orient sigma %.2f
%d: Begin Acc saturation: %.4f, pos sigma %.1f          %d: Acc saturation: %d samples, pos sigma %.1f
```

**[diag]** numerical guards: `EKF: P not positive definite: %d = %.3g`,
`EKF: Update: Invalid P or K`, `EKF: Update: P or Z is not positive definite`.

Ours: `grep -rn "isnan\|isfinite"` over the driver, `ukf.c`, `unscented.c`,
`matrices.c` → **nothing**. A failed `ukf_base_predict` (Cholesky failure, the
usual symptom of a diverged P) logs and returns with the prior untouched, and
never calls `rift_kalman_6dof_reinit` (`rift-kalman-6dof.c:746-785`).
`priv->sensor_range` (`accel_scale`/`gyro_scale` from the range feature report) is
decoded, `LOGD`'d once and **never used** (`rift.c:1326-1328` are the only
references). Saturation rails are known and checkable: HMD wire format
±104.86 m/s² / ±104.86 rad/s (21-bit × 1e-4, `packet.c:189-204`, `:389-394`);
Touch ±16 g and ±2000 °/s (int16, `rift.c:464-480`).

---

## 7. Defects found in *our* code while doing this comparison

These are not Windows deltas — they are bugs, found by reading our fusion path
against theirs. Several would corrupt any parity measurement.

1. **`rift-kalman-6dof.c:711-714` — `m1.R = 1e-6` for the accelerometer**, i.e.
   a 1 mm/s² standard deviation, with a `FIXME: Set R matrix to something based
   on IMU noise` above it. The real CV1 accelerometer is ~3 orders of magnitude
   noisier. The UKF is being told to believe the accelerometer almost absolutely.
2. **Q is added once per `ukf_base_predict()` and is not scaled by `dt`**
   (`ukf.c:85-92`). At 1 kHz IMU that is 1000× the intended spectral density per
   second, and every delay-slot pseudo-measurement adds another full Q.
3. **`state->pose_slot` is never reset to −1** after an update
   (`rift-kalman-6dof.c`, set only at init `:734`), so any later update that does
   not set it reads the *previous* delay slot.
4. **`rift_kalman_6dof_position_update` does not zero `time`** the way
   `rift_kalman_6dof_pose_update` does (`:843-854` vs `:879-885`), so a delayed
   position-only update runs a predict with the wrong dt.
5. **UKF covariance output is floored at 25° / 10 cm** before the pose search sees
   it (`rift-tracker.c:76-77`, `:1263-1268`, `:1610-1623`), and
   `gravity_error_rad` is derived from that floored value
   (`rift-sensor-pose-search.c:171`). The gravity gate therefore *can never arm
   tighter than 25–30°*, no matter how confident the filter is.
6. **`replace_pending` is silently dropped on the UKF path**
   (`rift-tracker.c:271, 280`) — the same-exposure merge's "this supersedes the
   earlier fix" signal is lost, so with `OHMD_RIFT_FUSION=ukf` the merge
   double-counts.
7. **`rift-kalman-6dof.h:22-40` documents a 22-element state with an
   angular-velocity state that does not exist** — angular velocity is a control
   input. Anyone extending the filter from the header will get it wrong.
8. `rift_kalman_6dof_clear` leaks `m_position`; `rift_fusion_ovr_clear` is never
   called (`rift-tracker.c:925`).
9. `refine_pose()` (`rift-sensor-opencv.cpp:242`, `cv::solvePnPRefineLM`) is dead
   code — no call site.
10. `rift.c:455-462` comment says the Touch IMU latency default is "10, 0 = off";
    the actual default is 25.0 (`:351`).

---

## 8. The program to parity, ranked

| # | Work | Why | Size |
|---|---|---|---|
| **0** | **Replay harness.** Record `{exposure, per-sensor blobs + labels, IMU, extrinsics}` to disk; replay offline through the tracker. They ship exactly this in production (`OVR::Recording::{Manager,Replayer,Serializer}`, `Vision::RecordedCamera`, `RecordingCameraFactory`, `ReplayCameraEndpointAdapter` **[rtti]**). | Every item below is otherwise tuned blind on live hardware. This is the force multiplier and it is also the prerequisite for #1. | M |
| **1** | **Joint multi-camera reconstruction** (§2). Publish per-exposure correspondence buffers into the delay slot; solve one pose over pooled undistorted rays with extrinsics fixed; accept at ~2 px; keep the per-camera solve as fallback. | Removes the measured 86 mm / 111 mm per-camera bias that pose-averaging cannot touch. Largest remaining position error. | L |
| **2** | **Fix the UKF, then make it default** (§7 items 1–7). Realistic `m1.R`, dt-scaled Q, `pose_slot` reset, `position_update` time fix, honour `replace_pending`, remove the covariance floor or feed the search an unfloored value. | The UKF is the only backend with bias states, and it is currently mistuned in ways that would make any A/B against the complementary filter meaningless. | M |
| **3** | **Gravity magnitude + 2-dof gravity direction as UKF states** (§3). Both are euclidean-ish additions; `rift-kalman-6dof.c`'s `state_residual_func`/`state_sum_func` pick up appended euclidean states for free, but a 2-dof direction needs explicit residual/sum handling (like `calc_quat_residual`). Gate |g| to 9.71–9.91. | Retires the fixed-gravity assumption behind the whole class of tilt/world-frame bugs. | M |
| **4** | **Gravity aligner** (§4): estimate up-in-camera per sensor from tracked-object gravity, filter it with confidence, and extend the room config beyond a single yaw scalar to a full alignment transform. | Makes the room frame self-levelling instead of a manual calibration step. Requires a config schema change. | M |
| **5** | **Saturation + numerical guards** (§6): use `sensor_range`, detect per-axis rails pre-calibration (`rift.c:313-314`, `:465-480`), inflate R / skip the sample, add `isfinite` checks and a real reset path on Cholesky failure. | Cheap, and today a clipped sample enters the filter at face value during exactly the fast motion where it matters. | S |
| **6** | **Extrinsic-change → filter reset** (§5), plus a Settled/Unsettled state for the refinement. | Our new online refinement currently moves the world under a running filter. | S |
| **7** | **Back-of-head LED group as a separate body** (§1). The CV1 strap flexes; they reconstruct it separately. | Improves HMD tracking when facing away; explains part of the rear-facing degradation. | M |
| **8** | **Sync/latency validation telemetry**: predicted vs measured camera latency, repeated exposure time, late-pose age — they log all three **[diag]**. | The two costliest bugs in this project were both timing. Cheap regression detector. | S |
| **9** | Dynamic `SetLedOnTime`; IMU lost-sample accounting; IMU temperature compensation. | Second-order. | S |

---

## 8b. OpenCV 5 port (was a hard blocker — resolved 2026-07-28)

`pacman` upgraded **OpenCV 4.13 → 5.0.0 on 2026-07-26**, and OpenCV 5 split
calib3d into geometry/stereo/calib and dropped the `<module>/<module>.hpp`
layout. `libopenhmd` stopped building entirely, so *nothing* in the C driver
could be compiled or tested.

Fixed in `rift-sensor-opencv.cpp` + `meson.build`:

- includes moved to the flat `opencv2/calib3d.hpp` / `opencv2/imgproc.hpp`,
  which OpenCV 5 keeps as a compatibility umbrella and 3/4 also provide — so
  the tree builds against all three;
- the legacy `calib3d/calib3d_c.h` include dropped (nothing used it);
- meson probes `opencv5` before `opencv4`;
- `refine_pose()` had `if (!cv::solvePnPRefineLM(...))` on a **void** return.
  Its version guard omitted OpenCV 4 (`> 4 || == 3 && …`), so 4.x compiled the
  other branch and the bug stayed latent until OpenCV 5 made it live. The
  function has no callers, so only the bogus test was removed.

**Verified, because a major version bump must not move the geometry**:
`tests/unittests/opencv_geometry.c` requires the fisheye projection and
undistortion to round-trip a known ray to 1e-4, and `estimate_initial_pose` to
recover a known pose from exact fisheye projections to 1e-3, both with real CV1
intrinsics. Both pass on OpenCV 5.0.0, alongside the joint-pose tests — 22
tests, `meson test -C build`.

The vendored `patches/openhmd-rift-room-config.patch` has been regenerated and
verified to apply cleanly to the pinned upstream commit, so a fresh
`install.py` picks all of this up.

## 9. At parity — do not re-chase

- **Forward prediction.** Rotation 0.097° median vs their 0.190°; position
  3.52 mm vs 2.20 mm at higher motion intensity (`PREDICTION.md`).
- **LED / exposure / camera-sync HID config.** Byte-identical (`findings-4a-hid.md`).
- **IMU report layout, scaling, sign extension.** Validated against their raw stream.
- **On-HMD auto-calibration flags.** CV1 firmware drops them.
- **IMU calibration matrices and the subtract convention.** Matching since `f9b88f6`.
- **RANSAC in PnP.** We already use `cv::solvePnPRansac` at a 3 px threshold; the
  gap is that it runs per-camera, not that it is absent.
- **Offline bundle adjustment.** `calibrate_room.py solve` is a genuine BA with a
  fixed reference camera; the gap is online operation, not the maths.
- **Same-exposure fix ordering.** Our obs merge fixed the 7.5 Hz race (6–18×).
  They never have it because of #1 — one fix per exposure.

---

## 10. Reproducing

```bash
udisksctl mount -b /dev/sda3 -o ro          # no sudo needed
M="/run/media/mia/7A2C34DB2C3493DB/Program Files/Oculus/Support/oculus-runtime"
cp "$M/server-plugins/Rift.dll" /tmp/ovr/ && cd /tmp/ovr
rizin -q -A -c 'Ps rift.rzdb' -c 'q!' Rift.dll      # ~20 s, 5585 functions
```

Function addresses, recovered constants and the string→function method are in
`decomp/rift-dll-functions.md`.

Still unrecovered, worth a third pass: the exact residual form inside
`fcn.18010cac0` (whether the joint solve is a single Gauss-Newton over pooled
rays or a chained per-camera refinement), the `1/650` and `1/2043` constants
there, and the `CameraCalibrator` settle criteria in the 55 KB `fcn.18018d430`.
