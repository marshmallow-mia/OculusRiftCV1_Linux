# Windows Oculus runtime vs. our Linux stack — tracking differential

**2026-07-28, second pass.** Supersedes the first pass (same filename) and the
"their tracking code is unavailable" assumption in `cv1-capture-plan.md`,
`HANDOFF-LINUX.md` and `findings-4a-hid.md`.

Companion artifacts: `decomp/rift-dll-classes.txt` (147 demangled `OVR::*`
classes), `decomp/rift-dll-tracking-strings.txt` (627 tracking log strings),
`decomp/rift-dll-functions.md` (function map + recovered constants).

---

## Status — 2026-07-28

Branch `windows-parity`: 9 commits in the openhmd tree, 12 here, 33 unit tests
passing (`meson test -C build`). **No hardware was available**, so every number
below is from replaying recorded captures or from synthetic tests, and nothing
here has been seen tracking a real headset.

**Shipped, with the measurement that justifies it**

| | measured |
|---|---|
| **Joint multi-camera reconstruction** (§2) — one pose per exposure from every camera's blobs, replacing the pose average | worst-camera reprojection 2.83 → **0.662 px** (Touch), frame-to-frame jitter 0.490 → **0.045 mm** (11×) and 1.377 → **0.205 mm** (6.7×); **100 %** of 2355 real exposures inside Oculus's 2 px bar |
| **Extrinsic-change notification** (§5) — tell the fusion when a sensor pose moves | residual after a 50 mm correction: 45.22 → **0.00 mm** |
| **IMU saturation handling** (§7b) | a clipped reading claiming "down is sideways" tilts the filter 90.00° unflagged, **0.00°** flagged |
| **Accelerometer noise measured, not guessed** (§7) | `m1.R` was 1e-6 under a FIXME; measured **2.2e-3** from a real CV1's raw 1 kHz stream, ~2000× |
| **OpenCV 5 port** (§8b) | the driver did not build at all; now builds against OpenCV 3/4/5, with geometry round-trip tests so a future bump cannot silently move the tracking |
| **Numerical guards, timing telemetry, UKF defect fixes** (§7, §7b, §7c) | reset on Cholesky failure, non-finite catch, four continuous timing checks |

**Tried and deliberately reverted** — both recorded rather than quietly dropped

- **Gravity as EKF states** (§3b). Implemented, stable, and *inert*: the state
  moved 0.0004 m/s² in 20 s, and loosening its prior 4× changed nothing. With
  ~1000 accelerometer updates per second and physically-correct near-zero
  process noise, the covariance collapses in the first few samples and freezes.
  Which is why Oculus feeds its gravity states from a separate aligner.
- **dt-scaling the process noise** (§7). Correct in principle; makes the next
  Cholesky factorisation fail, because the unconditional addition had been
  quietly keeping P positive definite. A filter that will not factorise is
  worse than a mistuned one.

**Needs hardware**

1. **Live validation of everything above.** The joint solver's runtime plumbing
   is compile-verified only — that both sensors' correspondences land in the
   same delay slot under real threading is untested.
2. **The gravity aligner** (§4). The measurement works and found a repeatable
   **~6° tilt bias in the HMD's fused orientation** (differential between
   cameras 0.72–0.90°, consistent across two devices; common mode ~5.97° for the
   HMD vs 0.73° for Touch). The driver now records the accelerometer `grav`
   vector so a fresh capture can settle it without the circularity.
3. **The back-of-head LED group** (§7c). `rift_get_led_info()` *discards* every
   headband LED, so the headset is trackable only from the front — and no
   capture we hold contains one to test against.

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

### Wired into the driver (2026-07-28)

`rift-sensor-pose-search.c` builds a per-sensor correspondence set for each
exposure — the device's labelled blobs, undistorted to normalised rays via the
existing `undistort_points()` — and hands it to the tracker alongside the pose.
`rift_tracked_device_model_pose_update()` stores it in the exposure's delay slot
next to the existing pose report, and when a second sensor reports for the same
exposure it reconstructs **one** pose across all of them and uses that as the
fusion target instead of the confidence-weighted merge. The merge remains as
the fallback for solo exposures, failed solves, and
`OHMD_RIFT_NO_JOINT_SOLVE=1`.

A solve whose worst camera exceeds 2 px is **rejected** rather than used — at
that point no single pose explains every image, which indicts the extrinsics,
not the pose. The driver logs it as a calibration warning, mirroring Oculus's
`"Bad calibration for camera %d …"`.

**Verification chain, all offline:**

| link | how | result |
|---|---|---|
| driver's rays ≡ prototype's rays | OpenCV `undistort_points` vs the Python undistortion on 400 real blobs | **8.6e-8 normalised = 0.00006 px** |
| prototype's joint solve | replay over 1199 + 1156 real exposures | 100 % inside 2 px; jitter 11× / 6.7× better |
| C solver ≡ prototype | 200 real exposures via `export-c` | 0.235 mm / 0.097° |
| C solver correctness | 4 unit tests (exact recovery, two-beats-one, bad extrinsics, thin data) | pass |
| driver integration | builds; 22 unit tests pass | compile-verified only |

**Still unverified, and it needs hardware or a driver-level replay**: the
runtime plumbing itself — that views actually arrive from both sensors within
the same slot's lifetime under real threading and timing. Everything the solver
consumes and produces is validated; when the two sensors' reports race, and
whether the delay slot is still live for both, is not.

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

## 3b. Gravity as EKF states — tried, reverted, and what it taught us

**2026-07-28.** Item 3 of §8 said to add gravity magnitude and a 2-dof gravity
direction to our UKF, matching the states Oculus's `IndirectEkf<18,9>` carries.
That was implemented — as plain euclidean states so
`state_residual_func`/`state_sum_func` pick them up unchanged, with the
magnitude clamped to Oculus's own 9.71–9.91 band and the direction as a
rotation about a horizontal axis, exactly their `(axis %.3f, 0, %.3f)`
parameterisation. It built, it was numerically stable, and every existing test
passed.

**It was reverted, because the states are inert.** Driven with a synthetic
device whose true local gravity is 9.75 m/s² while the filter starts at
9.80665, over 20 s at 1 kHz with vision fixes every 20 ms:

| | after 20 s |
|---|---|
| stationary | 9.8061 (error +0.056) |
| rocking ±0.5 rad at 0.5 Hz | 9.8059 (error +0.056) |
| rocking, accel-bias prior tightened 100× | 9.7912 (error +0.041) |
| rocking, gravity prior loosened a further 4× | 9.7912 — **unchanged** |

The direction states never moved at all.

Two reasons, and the second is the interesting one:

1. **Gravity and accelerometer bias are confounded.** A constant offset in the
   body frame is a bias; only orientation diversity separates them, and the
   bias prior is far looser, so the filter attributes the mismatch there.
2. **The estimate saturates.** Loosening the gravity prior 4× changes the
   answer not at all. With ~1000 accelerometer updates per second and a process
   noise of 1e-16 — correct, since gravity genuinely does not change — the
   gravity covariance collapses within the first few samples and the state is
   frozen thereafter. Keeping it alive would need process noise that says
   gravity wanders, which is physically false.

**This is why Oculus does not estimate gravity from the EKF's accelerometer
residual either.** Their gravity states exist, but they are *fed* by a separate
`Gravity aligner` subsystem with its own filter, confidence tiers, reliability
counts, an explicit nominated alignment camera, and a "filter stuck, too much
movement" detector (§4). Adding states to the estimator is the easy half; the
subsystem that makes them observable is the real work. §8 item 3 was mis-ranked
as separable from item 4 — they are one job, and it is item 4.

Cost avoided: 3 covariance dimensions is 6 more sigma points, roughly 17 % more
unscented-transform work per IMU sample, for no measurable benefit.

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

### MEASURED, 2026-07-28 — `tools/replay_recon.py gravity`, offline

The aligner's core measurement is implementable from the captures today. A
tracked device's tilt relative to gravity is known from its own accelerometer,
independently of any camera extrinsics; the vision solve gives that same
device's orientation relative to a camera. Composing the two says which way is
up in the camera's frame:

```
up_in_camera = R(device→camera) · R(device→world)ᵀ · up_world
```

Comparing that against what the room config implies gives each camera's tilt
error — Oculus's `tilt err %.2f`, reported next to reprojection error in
`"Good/Bad calibration for camera %d, object %d"` **[diag]**.

| capture | device | per-camera tilt err | scatter | **common mode** | **differential** |
|---|---|---|---|---|---|
| `final-verify.jsonl` | HMD | 5.80° / 6.25° | 0.40° | **6.024°** | **0.717°** |
| `floor-anchor.jsonl` | HMD | 5.61° / 6.21° | 0.38° | **5.907°** | **0.904°** |
| `ctrl-obs.jsonl` | Touch | 1.11° / 0.35° | 0.88° | **0.727°** | **0.864°** |

Same two cameras, same room config, all three captures. The decomposition
matters:

- **The differential — how much the two cameras disagree about up once each is
  mapped through its own configured pose — is 0.72–0.90° in all three, across
  two different tracked devices.** A property of the cameras must be
  device-independent, and this is. That is genuine relative camera tilt error,
  and it is the quantity an aligner would correct.
- **The common mode is device-specific: ~5.97° for the HMD, 0.73° for Touch.**
  A tilt shared by both cameras cannot be the cameras — they are not bumped
  identically — so it is the tracked device's own fused tilt being off gravity.

The parsimonious reading is that **the HMD's fused orientation carries a ~6°
tilt bias**, since Touch's common mode is near zero on the same rig. That is
worth chasing on its own: a 6° tilt error is the world visibly leaning in the
headset. It is repeatable to 0.12° across two independent captures and precise
to 0.4° of scatter, so it is not noise. It would be consistent with an error in
the HMD's IMU-to-model mounting rotation (`fusion_from_model`).

**Caveat, and what would settle it.** This uses `wp`, the *fused* pose, and
since the vision-tilt correction was added the fused tilt is pulled partly
toward the optical solution — which depends on the extrinsics being measured.
Oculus avoids the circularity entirely by using the **raw accelerometer at the
exposure instant**: their warning `"too many IMU samples between camera frames
for gravity alignment"` **[diag]** shows they bin IMU samples per camera frame
rather than consulting the fused pose. Our capture format records no IMU at
all, so that separation cannot be done offline today.

**The capture format now records it.** `rift_cal_capture_obs` writes a `grav`
field per observation — the complementary filter's low-passed accelerometer
direction, rotated into the device model frame, which owes nothing to vision or
to the room config. `replay_recon.py gravity` prefers it when present and warns
when falling back to the fused pose. Existing captures predate the field, so
the numbers above are still the fused-pose estimate.

**The runtime aligner itself is not implemented, and is recorded as pending
rather than guessed at.** What remains: accumulate up-in-camera per (camera,
device) with Oculus's confidence and reliability counting, nominate an
alignment camera, and correct the room frame — which also needs the room config
extended beyond its single `room-yaw-offset` scalar, since a non-vertical room
frame cannot currently be represented at all
(`rift-tracker-config.c:78-80`). Scoring any of that needs a fresh capture
carrying `grav`, and therefore hardware.

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
reference camera fixed). What we lack is it running **online**.

**(b) and (c) are done, 2026-07-28.** `rift_tracker_extrinsic_refine_apply`
used to rewrite a sensor pose silently under a running filter whose state was
conditioned on the old one. It now logs the applied delta in the runtime's own
terms (`sensor %s CameraPoseChange: dt … dr …`) and notifies every tracked
device: the complementary filter drops its pending vision error and takes the
next fix in full, the UKF widens its pose covariance by the size of the change.
A per-sensor settle state was added alongside — UNSETTLED while refinement is
still moving a sensor, SETTLED after three consecutive quiet evaluations —
matching the runtime's `is_sensor_settled` / `are_sensors_settled`.

**Measured**, A/B over a 50 mm extrinsic correction, one fix after the change:

| | residual to the corrected pose |
|---|---|
| filter not told (old behaviour) | **45.22 mm** |
| filter told | **0.00 mm** |

Untold, the filter blends the new, *correct* fix 50/50 with the stale error it
is still holding, so a full observation buys under 5 mm of the 50 mm it should.
That is the mechanism behind the "physically-bumped sensor produced 150 mm
cross-camera splits" note in `controller-tracking-analysis.md`: the correction
was being applied to the extrinsics and then immediately half-ignored by the
fusion.

---

## 5b. Zero-interaction camera calibration (done 2026-07-29)

The live test on hardware failed, and the cause was not the parity work: the
stored `rift-room-config.json` was **9.00° / 214 mm wrong** because a sensor
had been moved. The driver trusted that file absolutely, so everything
downstream failed in a way that looks like a tracking bug — cross-camera
disagreement 596 mm, no pose able to satisfy both cameras (119 px, 0 % inside
the 2 px bar), the second sensor unable to match its prior and dropping to the
slow brute-force path (~100 ms late poses), and therefore **the joint
reconstruction never ran once**.

"Recalibrate" is the wrong response. On Windows tracking works with no
calibration step at all, so requiring a user to walk the room is a design
defect on our side, not a fact of the hardware.

### Why no movement is needed

Each camera solves the device's **full 6-DoF pose in its own frame**. So one
exposure seen by two cameras already determines the transform between them
outright:

> `cam_b → cam_ref  =  (obj → cam_ref) ∘ (obj → cam_b)⁻¹`

Motion only averages out per-camera PnP bias. It is a refinement, not a
precondition. This is what the runtime logs as `Single frame calibration,
camera %d, oneChanged %d` **[diag]**, ahead of `Estimated calibration`,
`FastCalibrate`, and `Camera Calibration Settled`.

### MEASURED — the driver's own C code, on `captures/lin/2026-07-29/live-joint.jsonl`

A **stationary** headset, no user interaction. `tools/run_calib_check.sh`
builds `tools/calib_replay.c` against the real `rift-cam-calib.c` and feeds it
the 1024 co-observed exposures, so these are the accumulator the driver runs,
not a Python model of it:

| | |
|---|---|
| exposures folded in | **1006 / 1024** (18 rejected at 3σ) |
| per-exposure scatter | **0.104° / 4.59 mm** |
| settles after | **30 exposures** (~0.6 s at 52 Hz) |
| recovered baseline | 1050 mm |
| agreement with the Python bootstrap | **0.0000° / 0.83 mm** |
| how wrong the stored config was | **9.00° / 214 mm** |

Re-scoring the joint reconstruction on the same data with those extrinsics:

| | stored config | recovered by the C code |
|---|---|---|
| cross-camera disagreement | 596.51 mm | **1.67 mm** |
| joint worst-camera reprojection | 119.23 px | **0.098 px** |
| inside Oculus's 2 px acceptance | 0.0 % | **100.0 %** |

0.098 px is better than anything else measured offline (0.662 px Touch,
0.257 px HMD) because the extrinsics are now consistent with the data rather
than with a three-week-old file.

### What shipped

`rift-cam-calib.{c,h}` holds per-sensor `UNCALIBRATED → ESTIMATED →
CALIBRATED` plus `settled`, accumulating an incremental quaternion/position
mean behind a 3σ gate. Deviations are bias-corrected EMAs: read raw they start
at zero, which made the gate ~1.4σ at n=30 and let the settle test fire before
the dispersion was known.

Two wiring points matter:

- The observation is handed over in `rift-sensor-pose-search.c` **before** the
  `have_camera_pose` gate and before the pose is composed into world
  coordinates. A sensor with no calibration still solves the device fine in its
  own frame; previously it returned early and delivered nothing, so the only
  way in was the gravity bootstrap — which needs the HMD's *fused* pose, and a
  sensor that has never contributed a fix never gets one. **This is why the old
  code had no way to recover.**
- Adoption restarts the online refiner's window for that sensor. Every
  measurement in it was taken against the pose just replaced, so applying it
  afterwards would drag the sensor straight back.

A stored calibration is now a claim, not truth: the pose in use is compared
against what co-observation measures, and a settled estimate overrules it when
they disagree beyond what refinement could explain (`Invalid calibration - high
error`, `Recalibrating camera %d: wasCalibrated %d, …` **[diag]**). The
corrected pose is written back, so the file heals.

A sensor knocked **mid-session** needs more than the σ gate, which cannot see
it: after a knock every incoming sample really is a 3σ outlier against the
pre-knock mean, so the gate rejects the correct data forever and defends the
stale geometry. Noise gives scattered rejections; a knock gives an unbroken
run. A run past `RIFT_CAM_CALIB_BUMP_RUN` (60, ~1.2 s) is taken as evidence the
history is what is wrong, and it is rebuilt from the sample in hand.

`OHMD_RIFT_NO_AUTO_CALIB=1` disables the path for A/B testing.

### LIVE VALIDATION, 2026-07-29 — zero interaction, on hardware

Cold start from a config **8.63° / 226 mm stale**, headset stationary on the
desk, no user input of any kind. `captures/lin/2026-07-29/autocalib3.jsonl`:

```
sensor WMTD3052400VZL: automatic calibration adopted after 39 exposures
                       (0 rejected, scatter 0.228 deg / 9.9 mm)
sensor WMTD3052400VZL: the stored calibration was 8.63 deg / 226.3 mm from
                       what is measured - moving the sensor 226.3 mm / 8.63 deg
sensor WMTD3052400VZL: automatic calibration adopted after 600 exposures
                       (0 rejected, scatter 0.085 deg / 3.8 mm)
Device 0: joint reconstruction 3601 solved, 0 rejected,
          (0 observations arrived too late to count)
```

That last line had **never appeared before** — the joint reconstruction had
run zero times on hardware.

| | stale config (2026-07-29, before) | after, zero interaction |
|---|---|---|
| cross-camera disagreement | 596.51 mm | **1.254 mm** |
| joint worst-camera reprojection | 119.23 px | **0.097 px** |
| inside Oculus's 2 px acceptance | 0.0 % | **100.0 %** |
| joint pose frame-to-frame step | — | **0.082 mm** (vs 0.502 mm merged) |

Time to usable tracking: **39 exposures, under a second.**

#### Two defects this found that offline work could not

Both were invisible until it ran on hardware.

1. **The observation gate re-created the chicken-and-egg.** It required
   `RIFT_POSE_MATCH_STRONG`, which is only granted to a pose that agrees with
   the *prior* — and the prior comes from the extrinsics being calibrated. A
   sensor whose stored pose is wrong is therefore denied STRONG **because** it
   is wrong, and is never allowed to supply the observations that would fix it.
   Measured: sensor 1 reported flags `0x321` on **all 934** of its observations
   (LED IDs verified, orientation matching, position rejected) and calibration
   never ran. The gate now uses LED-ID verification and per-LED reprojection
   error — properties of the camera's own image, immune to a bad camera pose.

2. **Settling at `MIN_SAMPLES` adopts a 30-sample mean.** Good enough to make
   tracking work immediately, but it left 4.05 mm of cross-camera disagreement
   where the same data supports 1.25 mm. The estimate is now adopted a second
   time at `RIFT_CAM_CALIB_REFINED_SAMPLES` and then left alone — the rest is
   the online refiner's job, and each adoption disturbs the fusion.

### CORRECTION — that validation was at ONE headset position (2026-07-29, later)

Everything above was measured with the headset in a single fixed spot. Moving
it to a **different static place** — not waving it about, just setting it down
28 cm away and 7° round — broke two assumptions.

**1. The estimate was overfit to its viewpoint.** Per-camera PnP bias is
viewpoint-dependent, so a fit built from one headset position bakes that
position's bias in. The same calibration, scored where it was fitted and 28 cm
away:

| | at the fitted spot | 28 cm away |
|---|---|---|
| cross-camera disagreement | 1.89 mm | **7.32 mm** |
| own solve reprojected into the other camera | 0.20 px | **2.02 px** |

So the headline "1.254 mm" above is the *fitted-viewpoint* number and flatters
itself. Pooling both positions into one fit halves the worst case:

| fitted on | at A | at B | worst |
|---|---|---|---|
| A alone | 1.89 mm / 0.20 px | 7.32 mm / 2.02 px | **7.32 mm / 2.02 px** |
| A and B pooled | 3.66 mm / 0.88 px | 3.99 mm / 1.02 px | **3.99 mm / 1.02 px** |

Worth recording as a negative result: **the estimator barely matters.** A full
`scipy` bundle adjustment over the same data scored no better than a robust
mean (5.16 mm vs 4.51 mm worst case). Conditioning — how many viewpoints the
history spans — is the whole game.

**2. A σ gate could not tell a moved headset from a moved camera.** Setting the
headset down elsewhere shifts the single-frame estimate by 13.2 mm. Against a
converged running mean the 3σ gate fires at ~15 mm. That is a factor of **1.14**
— no separation. Simulating the real `rift_cam_calib_add()` over the real
captures: any shift ≥15 mm triggered a false `history was rebuilt — it had been
moved` exactly 59 exposures (~1.1 s) after the headset was set down. This
particular pair of positions landed at 13.2 mm, just under, so it did not fire
in practice — latent, not benign.

The fix is the shape `Rift.dll` already uses: a bounded **history**, re-solved,
judged by a **reprojection residual**, with camera-moved an *output of the
solve* (`FastCalibrate: Camera moved`) rather than a threshold on scatter. The
same two cases then separate by ~600×:

| | residual |
|---|---|
| correct extrinsics, fitted viewpoint | 0.2 px |
| headset set down elsewhere | 2.0 px |
| camera genuinely knocked 226 mm | **119 px** |

The history is stratified by viewpoint, eviction taking from the fullest
bucket, so a headset sitting still for an hour cannot crowd out the minute it
spent somewhere else — Oculus log the same idea as four per-bucket counts,
`sample counts: %d, %d, %d %d`.

One subtlety cost a round of debugging: the **0.85 improvement gate belongs at
adoption, not at estimation**. Left inside the module it froze the estimate at
its first viewpoint's fit — re-fitting after the move was a real gain but only
11 %, so a 15 % bar rejected it and preserved the very overfitting the history
exists to remove. It now sits in `rift_tracker_cam_calib_apply()`, which is
what actually disturbs a running fusion.

`SETTLED` now also requires viewpoint spread, matching the runtime's own split
between `Estimated calibration: camera %d` and `Camera Calibration Settled
after calibration.` A single-viewpoint fit scores well against its own
viewpoint no matter how biased it is, so it stays `ESTIMATED` and says so.

**Live re-validation**, cold start from the stale config at the new position:
adopted from 30 poses at 0.30 px residual, **1.409 mm** disagreement, 0.112 px
joint worst-camera, **100 %** inside the 2 px bar, 2701 joint reconstructions
with 0 rejected, no false camera-moved. Realistic cross-viewpoint expectation
with two positions pooled is ~4 mm; more viewpoints, accrued from ordinary use,
tighten it further with nothing asked of the user.

### THREE positions — the conditioning claim, and a defect it exposed (2026-07-29)

A third static position, 50 cm from both others and **53° round in gaze** (vs
only 7° for B). Every fit cross-scored at every position, using the driver's own
C calibrator:

| fitted on | at A | at B | at C | **worst** |
|---|---|---|---|---|
| A alone | 0.54 px | 2.72 px | 3.80 px | **3.80** |
| B alone | 2.28 px | 0.63 px | 1.32 px | **2.28** |
| C alone | 3.62 px | 1.45 px | 0.20 px | **3.62** |
| A + B | 1.20 px | 1.54 px | 2.47 px | **2.47** |
| **A + B + C** | 1.93 px | 1.06 px | 1.67 px | **1.93** |

Confirmed: more viewpoints generalise better, and **the three-viewpoint fit is
the only one that keeps every position inside the 2 px bar.** Note also that
each single-viewpoint fit is superb *where it was fitted* (0.2–0.6 px) and poor
everywhere else — which is precisely what makes it dangerous.

#### The defect: a narrow fit outbids a good one

Because a one-viewpoint fit scores 0.2 px against its own history and a
well-conditioned fit scores 1–2 px against that same narrow history, **the
narrow fit wins every acceptance test.** Measured live: a fresh session with the
headset sitting still at C overwrote the accumulated calibration and moved the
sensor **20.1 mm**, replacing a better calibration with a worse one. The driver
converged to "overfit to wherever the headset happens to be right now", and
every restart threw away the conditioning earned before it.

The runtime persists more than a pose — `Settled after loading calibration
data: %d cameras`, `Found calibration by serial in cache`. So the room config
now carries a per-sensor `viewpoints` count, and a calibration fitted over fewer
viewpoints cannot replace one fitted over more (a genuinely moved camera still
overrides, since that is decided by the residual). Verified live: with the
A+B+C fit installed and `viewpoints: 3`, a single-viewpoint session at C logs

```
sensor WMTD3052400VZL: keeping the stored calibration - it was fitted over
3 viewpoints and this session has seen 1 (it leaves 1.87 px here)
```

and leaves the config **byte-identical**. The field is optional; configs without
it (including everything the offline solver writes) read as 0 and behave as
before.

At C with the pooled fit: 4.63 mm disagreement, 1.35 px cross-camera, 0.143 px
joint worst-camera, 100 % inside the 2 px bar. Worse *at C* than a C-only fit
would be, and that is the correct trade — worst case across the room goes from
3.80 px to 1.93 px.

### WORN, IN THE HEADSET — the whole thing end to end (2026-07-29)

The first session with the headset actually on a head and moving, through
SteamVR, on Wayland. Reported subjectively as "not laggy or shifting".

The viewpoint machinery did exactly what it was built to do, unprompted:

| event | viewpoints | residual | correction |
|---|---|---|---|
| startup | 1 (stored: 3) | 5.10 px | **refused** — narrower than stored |
| re-solve | 3 | 0.55 px | 16.2 mm |
| re-solve | 4 | 2.15 px | 6.9 mm |
| re-solve | 6 | 2.07 px | **1.4 mm** |

`bins_seen` climbed 1 → 3 → 4 → 6 from ordinary head movement, the guard
refused the narrow startup fit, and the corrections converged — 16.2 mm, then
6.9 mm, then 1.4 mm. The config persisted at `viewpoints: 6`. **No user
interaction of any kind, and no calibration step.**

Measured over 3608 co-observed exposures *during motion*:

| | |
|---|---|
| joint worst-camera reprojection | **0.184 px** mean, 0.257 p95, 0.915 max |
| inside Oculus's 2 px acceptance | **100.0 %** |
| joint reconstruction | 3601 solved, **1** rejected |
| observations strong + LED-ID verified | 300 / 300 |
| cross-camera disagreement | 4.71 mm mean (max 93 mm on fast motion) |

The single rejection in 3602 solves, and 100 % inside the 2 px bar while the
head is moving, is the number this whole program was aimed at.

Note the residual *rising* with viewpoint count (0.55 px over 3, 2.07 px over
6) is the fit being honest, not degrading: a narrow fit scores well against its
own narrow history and badly everywhere else, which is precisely the trap
§5b documents. What matters is the joint reprojection, which stayed at 0.18 px.

### MOVING A SENSOR (2026-07-29)

Asking what happens if the sensors are repositioned turned up two defects, the
second worse than the case being asked about.

**A sensor moved while the driver was stopped livelocked.** On the next start
the history is built entirely from post-move observations and compared against
the stored pre-move pose, so the move was detected and the history dropped —
but the history then rebuilt from post-move data, was tested against the *same
stale pose*, exceeded the threshold again, and reset again. Roughly one reset
per `MIN_SAMPLES` exposures, forever, with the sensor keeping its wrong pose.
Nothing else would have rescued it: `extrinsic_refine_measure()` requires
`RIFT_POSE_MATCH_STRONG`, which is withheld from exactly the sensor whose
extrinsics are wrong — the same chicken-and-egg as §5b, still present in the
refiner. A `recovering` state now short-circuits both the camera-moved test and
the viewpoint guard after a reset, so the rebuilt estimate is adopted rather
than re-litigated against a pose already known to be wrong.

**The stratified history never turned over.** Eviction took the first entry of
the fullest bucket *by index*, so a freshly written sample — which lands in
whichever low slot was just freed — was the first match found next time and was
immediately evicted again, while old high-index entries were never touched.
Measured: **191 of 192 entries still stale after 1100 newer samples.** The
window was frozen after its initial fill, so a nudged sensor could not be
followed at all (estimate stayed 63 mm out). Entries now carry an arrival
sequence and the *oldest* of the fullest bucket is evicted: 0 of 192 stale over
the same run, and the estimate converges to **0.27 mm**.

The worn-session results above stand — that history filled with diverse
viewpoints during the first seconds of movement, before the freeze mattered.

#### Verified on real capture data

A known 216 mm / 7° move injected into `autocalib3.jsonl` by transforming one
camera's `obj→cam` poses (`tools/calib_replay.c` reads those pairs directly):

| | |
|---|---|
| stored pre-move calibration, scored on post-move data | **149.7 px** → `MOVED` |
| camera-moved threshold | 16 px (9.4× margin) |
| recovered extrinsic vs the injected transform | **8.09 mm / 0.366°** |

The 8 mm residual is the single-viewpoint bias of §5b, not an error in the
recovery: the rebuilt fit necessarily starts from one viewpoint and tightens as
coverage returns.

#### Bands, because the threshold is ~40 mm at 2 m

| sensor moved by | behaviour |
|---|---|
| **> ~40 mm** | detected, history dropped, re-derived and adopted within ~30 exposures |
| **< ~40 mm** | no reset; the rolling history turns over and the 0.85 improvement gate walks the pose across |

**The anchor is the exception.** `idx <= 0` returns early — sensor 0 defines the
frame and is never adopted. Move it and every other sensor is still re-solved
correctly *relative* to it, so tracking stays self-consistent, but the whole
room frame is displaced by however far the anchor went. Detecting that needs an
absolute reference this stack does not have (gravity gives tilt only). SteamVR's
recentre covers the comfort side.

The decision itself now lives in `rift_cam_calib_decide()`, which is pure and
therefore testable — the livelock hid inside the side-effecting version of it.

#### Getting a picture at all: two bugs of ours, not the kernel's

Neither was visible from tracking work, because tracking never touches video.

**The HMD advertised itself as 3dof.** `rift.c` set only
`OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING` on the HMD descriptor while the Touch
descriptors beside it set `POSITIONAL`. Nothing in this driver reads that flag,
so it went unnoticed for the whole project — but a runtime that picks a device
model from it (Monado defaults everything to 3dof) was **discarding the entire
constellation solve and substituting a neck model.**

**The CV1 was described to SteamVR as a desktop window.**
`driver_openhmd.cpp` returned `IsDisplayOnDesktop() = true` while setting
`Prop_IsOnDesktop_Bool = false`, with window bounds from a hardcoded
`m_nWindowX = 1920; //TODO: real window offset`. The kernel marks this
connector `non-desktop=1`, so the compositor never lays it out and those
coordinates belong to whatever real monitor is there. Starting SteamVR that way
produced seven `dc_stream_state is NULL for crtc '2'` from `dm_set_vblank()`
and hard-locked the machine — no oops, journal simply stopping. Returning
`false` puts SteamVR in direct mode, and the same launch then ran clean with
zero amdgpu errors.

Worth recording what that lockup was *not*, since three plausible theories were
wrong: not the tracking driver (USB-only, never touches DRM/KMS); not a
disconnected cable (the CV1 panel sleeps until the HMD is opened over USB and
drops within one second of release, so an idle connector check says nothing);
and not Wayland (Monado drives the same headset on the same session through
`wp_drm_lease_device_v1` with zero errors — `tools/run_monado_test.sh`).

#### Jitter A/B — `OHMD_RIFT_NO_JOINT_SOLVE=1`

Fused **output** pose, stationary, 35 s each after settling, 250 Hz:

| | joint ON | joint OFF | |
|---|---|---|---|
| sample-to-sample step, mean | **0.005 mm** | 0.012 mm | 2.45× |
| sample-to-sample step, p95 | **0.009 mm** | 0.026 mm | 2.82× |
| shake vs 0.5 s mean, rms | **0.062 mm** | 0.256 mm | 4.12× |

(Re-measured after the history rewrite. The earlier run read 2.09× / 2.56× /
1.32× against the single-viewpoint extrinsics.)

The output figures are much smaller than the per-exposure ones above because
the fused pose is IMU-dominated at 250 Hz and correction bleeding smooths
vision steps. Peak shake is unchanged — it is not set by cross-camera
disagreement.

### Scope boundary

Relative extrinsics plus gravity give **tracking quality** with zero
interaction. The room **origin, floor height and forward direction** are
comfort/room-scale settings, not tracking inputs; they keep their one-time
anchor step (or SteamVR's recentre). No tracking-quality number above depends
on them.

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
against theirs. Status as of 2026-07-28; all fixes verified by synthetic unit
tests in `tests/unittests/kalman_6dof.c` (`meson test -C build`).

**Fixed**

1. **`m1.R = 1e-6` for the accelerometer** — a 1 mm/s² standard deviation, under
   a `FIXME: Set R matrix to something based on IMU noise`. It told the UKF to
   believe the accelerometer almost absolutely. Now **measured** rather than
   guessed: `tools/measure_imu_noise.py` decodes the raw 1 kHz IMU stream from
   `captures/win/2026-07-12/imu_tracking.pcap` and takes the per-axis standard
   deviation over the quietest windows — **0.035–0.047 m/s², a variance of
   2.2e-3, about 2000× the shipped value**. Set to (0.05 m/s²)². Those windows
   still hold a little real motion, so it is an upper bound.
2. **`state->pose_slot` never reset to −1** — any later update that did not set
   it silently read the previous update's delay slot. Now cleared after every
   update.
3. **`rift_kalman_6dof_position_update` did not zero `time`** the way the pose
   path does, so a delayed position-only fix ran a predict across the gap
   between the exposure and now, *on top of* the lag the delay slot exists to
   represent.
4. **Released delay slots collapsed their covariance to exactly zero**, leaving
   P singular. Found while attempting item 6 below. Released slots now get a
   positive, uncorrelated covariance.
5. **`m_position` was leaked** by `rift_kalman_6dof_clear`; the stale header
   comment documenting a 22-element state vector with an angular-velocity state
   that never existed is corrected (angular velocity is the control input).

**Attempted and reverted, with the reason**

6. **Q is added whole on every call rather than scaled by `dt`** (`ukf.c`), so
   the effective process noise tracks the call rate — 1000×/second at the IMU
   rate — and a zero-dt call injects a full step of noise for no elapsed time.
   Scaling by `dt` is correct in principle and was implemented, but it makes the
   next Cholesky factorisation fail: this unconditional addition has been
   quietly keeping P positive definite, and removing it exposes delay-slot
   covariance blocks that are never properly initialised. Fixing item 4 above
   was not sufficient. **Reverted rather than shipped**, with the reasoning left
   in the code — a filter that fails to factorise is worse than a mistuned one.
   Doing this properly means giving the delay slots a correct covariance of
   their own first.

   Note the accompanying test, `test_rift_kalman_two_sample_rates`, does **not**
   detect this defect: with pose observations every 20 ms the steady state is
   measurement-dominated and passes either way. Catching it needs a
   dead-reckoning-only test.

**Not a defect — earlier claim withdrawn**

7. `replace_pending` being dropped on the UKF path (`rift-tracker.c:271,280`)
   was listed here as a bug. It is not: the flag exists to tell the
   complementary filter to overwrite rather than blend its *pending vision
   error*, and the UKF has no such state — each fix is applied as a measurement.
   There is nothing to replace.

**Still open**

8. The **25° / 10 cm covariance floor** (`rift-tracker.c:76-77`) applied before
   the pose search sees the filter's uncertainty, from which `gravity_error_rad`
   is derived (`rift-sensor-pose-search.c:171`). The gravity gate can therefore
   never arm tighter than its own 30° tolerance floor, however confident the
   filter is. Bounded benefit (the tolerance would go 50° → 30°), so it is
   ranked below the items in §8.
9. No `isfinite` checks anywhere in the filter path, and a failed
   `ukf_base_predict` logs and returns with the prior untouched rather than
   triggering a reset — see §8 item 5.

## 8. The program to parity, ranked

| # | Work | Why | Size |
|---|---|---|---|
| **0** | **Replay harness.** Record `{exposure, per-sensor blobs + labels, IMU, extrinsics}` to disk; replay offline through the tracker. They ship exactly this in production (`OVR::Recording::{Manager,Replayer,Serializer}`, `Vision::RecordedCamera`, `RecordingCameraFactory`, `ReplayCameraEndpointAdapter` **[rtti]**). | Every item below is otherwise tuned blind on live hardware. This is the force multiplier and it is also the prerequisite for #1. | M |
| **1** | **Joint multi-camera reconstruction** (§2). Publish per-exposure correspondence buffers into the delay slot; solve one pose over pooled undistorted rays with extrinsics fixed; accept at ~2 px; keep the per-camera solve as fallback. | Removes the measured 86 mm / 111 mm per-camera bias that pose-averaging cannot touch. Largest remaining position error. | L |
| **2** | **Fix the UKF, then make it default** (§7 items 1–7). Realistic `m1.R`, dt-scaled Q, `pose_slot` reset, `position_update` time fix, honour `replace_pending`, remove the covariance floor or feed the search an unfloored value. | The UKF is the only backend with bias states, and it is currently mistuned in ways that would make any A/B against the complementary filter meaningless. | M |
| **3** | **Gravity magnitude + 2-dof gravity direction as UKF states** (§3). Both are euclidean-ish additions; `rift-kalman-6dof.c`'s `state_residual_func`/`state_sum_func` pick up appended euclidean states for free, but a 2-dof direction needs explicit residual/sum handling (like `calc_quat_residual`). Gate |g| to 9.71–9.91. | Retires the fixed-gravity assumption behind the whole class of tilt/world-frame bugs. | M |
| **4** | **Gravity aligner** (§4): estimate up-in-camera per sensor from tracked-object gravity, filter it with confidence, and extend the room config beyond a single yaw scalar to a full alignment transform. | Makes the room frame self-levelling instead of a manual calibration step. Requires a config schema change. | M |
| **5** | **Saturation + numerical guards** (§6): use `sensor_range`, detect per-axis rails pre-calibration (`rift.c:313-314`, `:465-480`), inflate R / skip the sample, add `isfinite` checks and a real reset path on Cholesky failure. | Cheap, and today a clipped sample enters the filter at face value during exactly the fast motion where it matters. | S |
| ~~6~~ | ~~Extrinsic-change → filter reset, plus a Settled/Unsettled state.~~ **DONE** — see §5. A/B: 45.22 mm → 0.00 mm residual after a 50 mm extrinsic correction. | | |
| **7** | **Back-of-head LED group as a separate body** (§1). The CV1 strap flexes; they reconstruct it separately. | Improves HMD tracking when facing away; explains part of the rear-facing degradation. | M |
| **8** | **Sync/latency validation telemetry**: predicted vs measured camera latency, repeated exposure time, late-pose age — they log all three **[diag]**. | The two costliest bugs in this project were both timing. Cheap regression detector. | S |
| **9** | Dynamic `SetLedOnTime`; IMU lost-sample accounting; IMU temperature compensation. | Second-order. | S |

---

## 7b. IMU saturation and numerical guards (done 2026-07-28)

Oculus treats a clipped IMU axis as a named condition and inflates the
corresponding sigma while it lasts — `"Begin Gyro saturation: %.4f, orient
sigma %.2f"`, `"Acc saturation: %d samples, pos sigma %.1f"` **[diag]**. We had
nothing: `grep -ci saturat` over the driver returned 0, and the sensor's own
full-scale range (`accel_scale`/`gyro_scale` from the RANGE feature report) was
decoded once, `LOGD`'d, and never referenced again.

Now detected per sample, against whichever rail binds first — the sensor's
configured range or the wire format's own limit (HMD: 21-bit at 1e-4 units, so
±104.86; Touch: int16, so ±16 g and ±2000 °/s) — and reported to the fusion:

- **Complementary filter**: skips the tilt correction entirely for that sample
  and refuses to let the reading into the low-passed gravity estimate.
- **UKF**: inflates the accelerometer measurement noise from (0.05 m/s²)² to
  (10 m/s²)², i.e. "this sample says nothing".

**Zeroing the confidence was not enough, and the test caught it.** The
complementary filter's `apply_tilt_correction` has a start-up branch that snaps
straight to the accelerometer *regardless of confidence*, so a clipped first
sample threw the orientation 90° over anyway. Measured with the same bogus
reading fed both ways:

| accelerometer claiming "down is sideways" | resulting tilt |
|---|---|
| not flagged saturated | **90.00°** |
| flagged saturated | **0.00°** |

The A/B is the point: if the unsaturated case had not moved, the test would
prove nothing.

**Numerical guards.** A failed `ukf_base_predict` (a Cholesky failure, i.e. P
has stopped being positive definite) or a failed update now re-seeds the
covariance instead of logging and carrying the corrupt state forward, and a
state that has gone non-finite is caught and reset rather than propagating into
every reported pose. Covered by a test that injects NaN accelerometer samples
and asserts the reported pose stays finite.

**Not measured**: how often saturation actually occurs in real use. The one raw
IMU capture available (`imu_tracking.pcap`) is ordinary seated wear, so this is
a guard rather than a fix for an observed event.

## 7c. The back-of-head LEDs are deliberately thrown away (2026-07-28)

Investigating §8 item 7 turned up something more consequential than expected.
`rift_get_led_info()` in `rift.c` ends with a filter that **discards every LED
the headset reports at `z < -100 mm`** — the entire headband group — under a
FIXME reading *"until the positional tracking copes with the device
articulation. At the moment, a camera that sees the back LEDs will extract the
wrong position."*

The reasoning is sound: the strap articulates relative to the visor, so visor
and headband are not one rigid body, and solving headband blobs against the
visor model gives a wrong pose. But the consequence is that **the headset is
only trackable from the front** — everything behind the head is thrown away
before the tracker ever sees it.

Our LED model after filtering is 34 points spanning `z ∈ [-0.009, +0.074]` —
a visor shell, ~16 cm wide and 8 cm deep, with nothing behind it.

**Oculus keeps them and solves the back group as its own rigid body.** The
runtime validates the split (`"Expected %d / %d front / back LEDs, but found
%d / %d"`), tracks its state (`"Back of head tracking status"`), and
reconstructs it through the *same* routine as the visor — `fcn.180103ff0` calls
the reconstruction `fcn.18010cac0` a second time and reports
`"Back of head reconstruction failed. Reprojection error: %f"` **[decomp]**.
The HMD definition carries `FrontLEDCount` / `BackLEDCount` / `LEDCalibrationPath`
so the split is configuration, not inference.

**Not implemented, and it cannot be validated here.** Two independent reasons:
every capture we hold was recorded *after* this filter, so there is not a single
headband-LED observation to test against; and a second rigid body needs a second
tracked model, its own correspondence-search entry, and a policy for relating
the two bodies when both are visible. Building that blind would be guessing.

What was done: the drop is no longer silent. It was a bare `printf` to stdout —
actively harmful, since OpenHMD's stdout is what `poselog.c` avoids writing CSV
to for exactly this reason — and is now a single `LOGI` summary naming the
capability gap, with the per-LED detail at `LOGV`.

**To pick this up**: keep the headband LEDs in a second `rift_leds`, register it
as its own tracked body, and let the existing joint reconstruction
(`rift-joint-pose.c`) solve it — the solver is already body-agnostic. Then
record a capture facing away from the sensors to score it.

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
