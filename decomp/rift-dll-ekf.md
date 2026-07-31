# Rift.dll's EKF — first extraction pass

Target: `server-plugins/Rift.dll`, sha256
`80d84739701f9fa184f6b7d44f396c084af442f54ad874de84379153f02a5d01`, 5621496
bytes. Available locally without mounting the Windows partition, at
`~/.local/share/oculus-wine/drive_c/Program Files/Oculus/Support/oculus-runtime/server-plugins/disabled/Rift.dll`.

Every value below is tagged **recovered** (read out of the binary) or
**inferred** (reasoned from strings or structure). That distinction is not
pedantry: `FUSION.md` presents `rift-fusion-ovr.c` as a "1:1 port of the official
Oculus runtime" when its sources are the *open SDK 0.2.5–0.3.2*, and it asserts
"No Kalman filter — Oculus explicitly rejected Kalman/particle filters". The CV1
runtime contains `OVR::Vision::IndirectEkf<18,9>`. That mislabelled inference was
made the default fusion backend and became the overshoot bug found on
2026-07-31.

## The headline result: their filter's internals can be logged

**Recovered.** `fcn.1801308c0` writes a CSV whose header is the runtime's own
state dump (`.rdata` `0x180453500`):

```
device,time, imu_acc_mag,acc_mag, x,y,z,vx,vy,vz,ax,ay,az,yaw,pitch,roll,
omega_x,omega_y,omega_z, alpha_x,alpha_y,alpha_z,
x_vis,y_vis,z_vis,vx_vis,vy_vis,vz_vis,yaw_vis,pitch_vis,roll_vis,
omega_x_vis,omega_y_vis,omega_z_vis, g_thresh,is_position_tracked
```

A controller-less variant is at `0x180452f30`, written by `fcn.1801265a0`.

It is gated by a **config key named `dump_csv`**:

```c
cVar3 = fcn.18021d340(cfg, 0x180452f08);   /* 0x180452f08 = "dump_csv" */
if (cVar3 != '\0') {
    ...build path...
    iVar5 = fcn.1803db194(path, 0x18044a818);      /* fopen(path, mode) */
    if (iVar5 != 0)
        fcn.180082170(&hdr, 0x180453500);          /* write the header */
}
```

`fcn.18021d340` looks up a named config entry and returns
`(int32_t)*(double *)(entry + 0x58) != 0`, i.e. the value is stored as a double
and tested for non-zero. The output path is built from `"\impact_"`
(`0x1804534f0`) + fields + `".csv"` (`0x180452f24`).

**What it is, precisely — correcting my own first reading of it.** I initially
described this as their filter's internals on one timeline, which oversold it.
The logger is **event-triggered, not continuous**. `dump_csv` sits among
telemetry keys in `.rdata` — `sensor_fusion`, `telemetry_tag`,
`oculus_imu_event`, `device_category`, `HmdDropThresholdInGs`,
`TouchImpactThresholdInGs` — and the two output paths are `\impact_` and
`\drop_hmd_`. The write loop fetches a count and iterates a stored buffer:

```c
var_3f8h = fcn.180118d10(arg2 + 0x48);      /* number of buffered samples */
do { ...write one row... } while (var_460h < var_3f8h);
```

So it dumps a **buffered window of samples around a detected drop or impact**
(an acceleration above `HmdDropThresholdInGs` / `TouchImpactThresholdInGs`), not
a continuous trace of a normal session.

**It is still worth having**, because each window contains what no capture in
`captures/win/` does: the fused state and the vision-only channel side by side,
with `g_thresh`. It is a sample of ground truth rather than a stream of it, and
high-G events are trivially producible on purpose. But it will not, by itself,
support fitting a filter against a normal tracking session.

**Not yet established:** where the config store reads `dump_csv` from (JSON under
`%LOCALAPPDATA%\Oculus`, registry, or a server config), whether it can be set in
the Wine install, and how many samples the buffer holds.

## Function map corrections

**Recovered.** The addresses recorded in `rift-dll-functions.md` are *inside*
larger functions; rizin's own boundaries are:

| recorded | actual start | size | role |
|---|---|---|---|
| `fcn.18017d9e0` | `fcn.18017d910` | 6543 | EKF core |
| `fcn.1801896e0` | `fcn.180189610` | 6942 | EKF core |
| `fcn.18017cae0` | `fcn.18017ca10` | 3743 | EKF core |
| `fcn.1801308c0` | — | 8207 | **CSV state logger (new)** |
| `fcn.1801265a0` | — | 11946 | CSV state logger, controller-less (new) |
| `fcn.18021d340` | — | 195 | **named config lookup (new)** |

Decompiled this pass and kept in the session scratchpad (not committed — ~8600
lines of generated C): `18017d9e0`, `1801896e0`, `18017cae0`, `180134cd0`,
`18013a920`, `180138780`, `180132a20`, `180141610`, `1801308c0`.

## Q and R are not compile-time constants

**Recovered, and it redirects the effort.** Harvesting every `.rdata` float these
functions reference yields almost nothing of estimator interest:

| function | constants read |
|---|---|
| `180134cd0` EKF IMU driver | 57.2957795 (180/pi), 1000, 1, 10, 0.25 |
| `180132a20` reset sigmas | 57.2957795, 1000, 0.5, 3 |
| `180138780` EKF reset | 57.2957795, 1000, 1, 0.25, 3 |
| `18017d9e0` EKF core | 57.2957795, 1000, **1e-06**, 1, -1, **1e-05**, pi |
| `1801896e0` EKF core | 1000, 1, -1, **0.1** |

`57.2957795` and `1000` are radians-to-degrees and a millisecond conversion, i.e.
logging. The `1e-06` and `1e-05` in the core are plausibly covariance
regularisation or a positive-definiteness epsilon (that function carries the "P
not positive definite" string) — **inferred, not confirmed**.

~~Notably absent: the `9.80667, 750, 0.01` previously attributed to
`180134cd0`, and the `2.5e-5, 9e-6` reset sigmas attributed to `180132a20`.~~
**RESOLVED in W8 — this paragraph was wrong.** Those constants are present; the
`data.*` scan used here is not a superset of a manual scan, because it misses
everything the decompiler folds into an expression. See "W8 result".

The structural conclusion stands regardless: **there is no inline Q or R.** The
process and measurement noise are object members, set at construction or from
configuration, so extracting them means finding the `IndirectEkf` constructor and
whatever populates it — not reading these functions harder. This is the single
biggest correction to the plan that assumed "decompile the four functions and the
matrices fall out".

## Tooling

`tools/rip_xref.py`. rizin's `axt` resolves nothing in this binary, which is why
only ~15 of 5585 functions have ever been named. The script scans `.text` for
RIP-relative operands (`modrm & 0xC7 == 0x05`) and maps them to targets; that is
how the `dump_csv` gate was found. It maps addresses **per section** via the
section table — the deltas are not uniform (`.text`/`.rdata` are `0x180000c00`,
`.data` is `0x180000e00`, `.pdata` onward diverge further), and the
TypeDescriptors live in `.data`. See the correction under W1.

Caveat learned this pass: a scanner limited to `movss`/`movsd` loads misses most
constants, because MSVC folds them into arithmetic (`mulsd xmm,[rip+d32]`).
Reading rizin's resolved `data.*` labels out of `pdf` output is both easier and
more complete.

## Queue

Durable state for a multi-session extraction. One item per iteration; each ends
in a commit. A fresh context should be able to resume from this table alone.

| # | item | status | notes |
|---|---|---|---|
| W1 | RTTI → COL → vftable → **constructor** for `IndirectEkf<18,9>` and `EkfFusion`; object size and layout | **DONE** | see "W1 result" below |
| W2 | **Q and initial P** from the constructor and its callers | **partly done** | ctor found and it only allocates/zeroes; P0 formula located in the reset path; **Q still open** — look for the propagation-time noise injection in W3 |
| W3 | **Propagation model** — predict step off the IMU path | **partly done** | P proven 18x18 row-major; per-state covariance FLOOR found; "EKF core" mislabelling corrected. Propagation math and Q still open — next look at `fcn.1801386a0` (8602 B) and the second 18 inline doubles at `+0xa0` |
| W4 | **Measurement model and per-observation R** | **started** | real function is `fcn.18013a840` (4408 B, 13 args, matrix-heavy); no inline constants, so R arrives as an argument — tracing which one needs the driver's argument setup |
| W5 | **Reset policy** — triggers and thresholds | open | `fcn.180138780`, `fcn.180132a20`; 11 named causes, only sigmas known |
| W6 | **Gravity aligner → EKF coupling** | open | `fcn.18011ab20`, `fcn.180146860`, `fcn.1801376b0`; a prior attempt at gravity states was inert for want of this |
| W7 | **`dump_csv` plumbing** — config source, settable under Wine?, buffer depth | **open, PROMOTED to next** | static passes are now yielding one fact each; a real capture would answer the R rule, reset thresholds and Q directly |
| W8 | **Reconcile the constant discrepancy** vs `rift-dll-functions.md` | **DONE** | resolved against me: the earlier note was right, my pass-1 scan was incomplete. See "W8 result" |

Stop when the queue is empty or every remaining item is blocked — and say which,
rather than looping on a blocked item.

## W1 result: the class hierarchy, the EKF's offset, and the constructor

**Recovered.** The RTTI walk, done with the new `--rva` / `--ptr` modes of
`tools/rip_xref.py`:

```
TypeDescriptor .?AU?$IndirectEkf@$0BC@$08@Vision@OVR@@   name 0x180526208, TD 0x1805261f8
TypeDescriptor .?AVEkfFusion@Vision@OVR@@                name 0x1805261d8, TD 0x1805261c8
TypeDescriptor .?AVSensorFusionFilter@RiftVision@OVR@@   TD 0x180525f40
CompleteObjectLocator (EkfFusion)                        0x18049de88
vftable (EkfFusion)                                      0x180453170
```

The COL validates: signature 1 (x64), offset 0, cdOffset 0, TD RVA `0x5261c8`
(EkfFusion), CHD `0x49deb0`, and `pSelf` equal to its own RVA `0x49de88`.

### `IndirectEkf<18,9>` is not polymorphic

**Recovered.** Both RVA references to its TypeDescriptor decode as
**BaseClassDescriptors**, not COLs — the COL reading gives signature 0 and an
implausible `pSelf`, while the BCD reading is coherent (`pdisp = 0xffffffff`, the
standard non-virtual-base marker, `attributes = 0x40`).

So it has no vftable and no constructor of its own to find. It exists only as a
subobject of classes that do carry RTTI. That kills the "find the IndirectEkf
constructor" phrasing of the plan: the initialisation happens in the *containing*
class's constructor.

### The hierarchy and the offset that matters

**Recovered**, by walking the BaseClassArray at `0x18049decc`:

```
OVR::RiftVision::SensorFusionFilter          mdisp 0
  <- OVR::Vision::EkfFusion                  mdisp 0
       <- OVR::Vision::IndirectEkf<18,9>     mdisp 512  (0x200)
```

**The EKF subobject begins at byte offset `0x200` within an `EkfFusion`.** That is
the anchor everything else hangs off: state, covariance, Q and R are all at fixed
displacements from `this + 0x200`, so field offsets seen in the decompiled
predict/update code can now be attributed.

The second BCD (`0x18049df60`, mdisp 0) is `IndirectEkf`'s own self-entry, which
every class lists first.

### Where construction happens

**Recovered.** Exactly two functions store the `EkfFusion` vftable:

| function | size | reading |
|---|---|---|
| `fcn.18012bbb0` | 1557 | constructor or destructor |
| `fcn.18012c1d0` | 1703 | the other |

**Inferred:** one is the constructor and one the destructor (MSVC writes the
vftable in both). W2 starts by decompiling both and taking whichever initialises
memory at `+0x200`.

### Correction to the method notes

**The section deltas are not uniform**, contrary to what the plan asserted.
`.text` and `.rdata` share `0x180000c00`, but `.data` is `0x180000e00` and
`.pdata` onward diverge further. The TypeDescriptors live in `.data`, so any
script using a single global delta misreads them. `tools/rip_xref.py` maps
per-section via the section table; the ad-hoc constant-dump scripts in pass 1 did
not, and were only correct because they happened to touch `.rdata` alone.

## W2 result: the `9` is settled, and the object's first fields

**Recovered.** `fcn.18012bbb0` is `EkfFusion`'s constructor, and its first act is
exactly what W1 predicted:

```c
fcn.18012b9b0(arg1 + 0x200);                       /* IndirectEkf subobject */
fcn.18017a7f0(arg1);
*(code **)arg1 = vtable.OVR::Vision::EkfFusion.0;
```

`fcn.18012b9b0` (468 bytes) is therefore the **`IndirectEkf<18,9>`
initialiser**, and its two heap allocations settle the dimensions by arithmetic:

| alloc | bytes | zeroing loop | shape | member |
|---|---|---|---|---|
| `fcn.18029718c(0xa20)` | 2592 | **18** iterations x 18 doubles | **18 x 18** | `+0x00` |
| `fcn.18029718c(0x510)` | 1296 | **9** iterations x 18 doubles | **9 x 18** | `+0x08` |

2592 = 18·18·8 and 1296 = 9·18·8 exactly.

**The `9` in `IndirectEkf<18,9>` is the measurement dimension.** The 18x18 is the
error covariance **P**; the 9x18 is the measurement Jacobian **H**. This was the
standing unknown that no note in the repo had ever stated, and it is now read off
the allocation sizes rather than inferred.

Layout so far, relative to the subobject base (`EkfFusion + 0x200`):

```
+0x00   double *P        heap, 18x18
+0x08   double *H        heap,  9x18
+0x10 .. +0x128          36 doubles inline, zeroed  (= two 18-vectors)
```

**Inferred:** the 36 inline doubles are two 18-vectors — plausibly the error
state and one working vector — but nothing yet distinguishes them.

### Q and P0 are not set at construction

**Recovered.** The constructor zeroes everything and sets no noise values at all.
So neither Q nor the initial covariance is a construction-time constant; both are
written later by the init/reset path. The reset path already shows the P0
arithmetic (`fcn.180132a20`):

```c
dVar32 = ((dVar33*dVar33 + dVar30*dVar30 + dVar32*dVar32) * 2.5e-05) / 3.0
       + *(double *)(arg1 + 0x440) * *(double *)(arg1 + 0x440) + dVar31 * 9e-06;
```

`2.5e-05` m^2 = (5 mm)^2 and `9e-06` m^2 = (3 mm)^2, matching the "reset sigmas"
reading in `rift-dll-functions.md`. So P0 is **built from a formula over current
state, not from a constant diagonal** — which is why no P0 matrix appears
anywhere as data.

**Q remains open.** It is not in the constructor and not in the reset path; the
remaining candidate is injection at propagation time, which is W3.

## W8 result: the earlier note was right and my pass-1 scan was wrong

**Recovered, and it is a correction against me.** Pass 1 reported that it could
not find `9.80667 / 750 / 0.01` in `fcn.180134cd0` or `2.5e-5 / 9e-6` in
`fcn.180132a20`, and flagged one of the two extractions as wrong. It was mine.

Harvesting numeric literals from the decompiled C finds them immediately:

| function | literals |
|---|---|
| `180132a20` | **2.5e-05**, **9e-06** |
| `180134cd0` | **750.0**, **0.01**, **0.25**, 10.0 |
| `180138780` | 0.25, 2.75 |
| `180141610` | 9.0 |
| `18017d9e0` | pi, 1e-06, 1e-05 |
| `1801896e0` | 0.1 |

**Method correction, and the same shape as the metric failures in the tracking
work:** pass 1 scanned only rizin's resolved `data.*` labels in `pdf` output.
That misses every constant the decompiler folds into an expression. Constants
must be harvested from **both** — `data.*` labels for the ones that stay as
memory operands, and numeric literals in `pdg` output for the rest. Neither alone
is complete, and reading one as exhaustive is what produced a false contradiction
against a correct note.

## W3 result: P is 18x18 row-major, and Oculus floors its diagonal

### Correction: the "EKF core" functions are the camera calibrator

**Recovered.** `rift-dll-functions.md` labels `fcn.18017d9e0`, `fcn.1801896e0`
and `fcn.18017cae0` "EKF core (generic)" on the strength of their
`P not positive definite` / `Update: Invalid P or K` strings. Those strings are
Kalman-shaped, but the call graph says what they serve. Scanning `.text` for
`E8 rel32` call sites (rizin's `axt` cannot do this):

```
0x18017d910 <- 0x18017d8b0 (85 B wrapper) <- 0x180199c40  ("OnWorldFromCameraChange")
0x18017ca10 <- 0x180199c40                <- 0x180197120  ("Estimate: obj=", "Estimate: cam=")
0x180189610 <- 0x18017ca10, 0x18017d910
```

They are reached only from the **world-from-camera estimator**, i.e. camera
calibration. They are a generic least-squares/Kalman library, not the tracking
fusion's predict step. Anyone hunting the tracking EKF in those functions is in
the wrong module.

The tracking EKF is driven from `fcn.180134bf0` (10709 B, contains the recorded
`0x180134cd0`), which calls its siblings with the EKF subobject directly:

```c
fcn.18013a840(ctx + 0x56be8, ctx + 0x200, ...)   /* vision update */
fcn.18013b980(ctx + 0x56be8, ctx + 0x200, ...)   /* covariance set + floor */
```

### P is 18x18, row-major — proven by a stride

**Recovered.** `fcn.18013b980` dereferences `*(double **)arg2` (which W2
established is **P**, at `IndirectEkf+0x00`) and touches indices

```
0, 0x13, 0x26, 0x39, 0x4c, 0x5f, 0x72, ...      = 19i
```

A stride of 19 is `P[i][i]` for an 18x18 row-major matrix (`i*18 + i`). That
settles both the storage order and, independently, the 18.

`fcn.18012b2d0`, called immediately before, copies an 18x18 matrix (18
iterations of 18 doubles), so `fcn.18013b980` is **"set P from source, then floor
its diagonal"**.

### The inline doubles are a per-state minimum-variance floor

**Recovered.** The clamp is

```c
P[i][i] = max(P[i][i], floor[i])
```

with `floor[i]` read from `IndirectEkf + 0x10 + 8i`, spanning `+0x10` to `+0x98`
— **exactly 18 doubles**, one per state. That names the first half of the 36
inline doubles W2 found.

**Why this matters beyond bookkeeping.** `windows-vs-linux-tracking.md:377-421`
records that this project already tried adding gravity states to its own filter
and reverted them because they were **inert**: "with ~1000 accelerometer updates
per second and a process noise of 1e-16 the gravity covariance collapses within
the first few samples and the state is frozen thereafter". Oculus prevents
exactly that failure with a per-state variance floor. The mechanism our attempt
lacked is now recovered, and it is not a tuning value but a structural feature.

The floor's extent is exact: the last clamp is `arg2 + 0x98` against
`P[0x143]`, and `0x143 = 323 = 19 x 17`, i.e. `P[17][17]`. So it covers states
0..17 and no further.

**The second 18 inline doubles (`+0xa0`..`+0x128`) are written by nothing.**
Searched across every function decompiled so far — the EkfFusion constructor, the
IndirectEkf initialiser, the IMU driver, the reset path, the floor path and the
vision update — the only code that touches that range is the constructor's
zeroing. Recorded as a **negative result**: it is not the process noise in any
path examined, and it may be reserved or dead. Assigning it a role would need
either a caller not yet reached or the `dump_csv` route.

### Q is still not located

Not in the constructor (W2), not in the reset path (W2), and not in the floor
path. Remaining candidate is injection during propagation, inside
`fcn.1801386a0` (8602 B) or the driver itself. That is where W3 resumes.

## W4 start: the vision update takes its noise as an argument

**Recovered.** The recorded `fcn.18013a920` is inside `fcn.18013a840` (4408 B),
which is the actual function. The driver calls it as

```c
fcn.18013a840(ctx + 0x56be8, ctx + 0x200, &stack, iVar20, ...9 more stack args)
```

so arg2 is the EKF subobject, arg1 is a per-device observation block, and the
remaining eleven arguments carry the measurement.

**Recovered:** the body is matrix work over 18-sized loops (`0x12` trip counts,
strides of `0x12`) and contains **no numeric literals at all** beyond `0.0`.
There is therefore no tabulated R anywhere in the update: the measurement noise
is passed in per call. That is direct confirmation of the "per-observation R"
reading in `windows-vs-linux-tracking.md:345`, which until now was qualitative.

### The update works in state space, not innovation space

**Recovered.** Allocation sizes name the matrices, since every one is a multiple
of 8 with an unambiguous factorisation:

| size | doubles | shape | where |
|---|---|---|---|
| `0xa20` | 324 | **18x18** | **six** allocations inside `fcn.18013a840`, one in the driver |
| `0x510` | 162 | 9x18 | the constructor's H |

**There is no `0x288` (9x9 = 648 byte) allocation anywhere in either function.**
A textbook EKF update forms the innovation covariance `S = H P H' + R` at 9x9 and
the gain `K = P H' S^-1` at 18x9. Six 18x18 temporaries and no 9x9 heap buffer
instead points at a **state-space formulation** — Joseph form
`(I-KH) P (I-KH)' + K R K'` is all 18x18 — though a 9x9 could equally live in the
864-byte stack buffer (`auStack_7e8`, 108 doubles) rather than the heap.

**Inferred, flagged as such:** Joseph-form or otherwise state-space update. The
allocation evidence is suggestive, not conclusive, and the stack buffer is an
unexcluded alternative.

Nine-element strides do appear (`pdVar15[9]`, offsets of `0x48` = 9 doubles), so
the 9-dimensional structure is present in the working set even without a 9x9
allocation.

**Still open:** which of the eleven trailing arguments is R, and the rule the
driver uses to build it (reprojection error, camera count, distance).

### Re-prioritisation

W7 (`dump_csv`) is promoted ahead of the remaining static items. The last few
iterations have each cost a full pass to recover one structural fact, and the
items still open — the R rule, reset thresholds, the gravity coupling, Q — are
exactly the things a real capture would show directly. If the key can be set
under Wine, one event window answers several of them empirically; if it cannot,
nothing is lost but one iteration.

### Scratch state worth preserving

`scratchpad/ekf/` holds `Rift.dll`, `rift.rzdb` (~20 s to rebuild, 5585
functions), `pdg_*.c` for the eight EKF functions plus the CSV logger, and
`dis_*.txt` disassembly. Regenerate with the recipe in
`rift-dll-functions.md:11-16` if lost.
