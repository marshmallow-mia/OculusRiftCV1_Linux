# `Rift.dll` tracking function map

Meta Horizon runtime **v79 / CAPI 1.111**, file
`Program Files/Oculus/Support/oculus-runtime/server-plugins/Rift.dll`
(5,619,960 bytes). Image base `0x180000000`. 5585 functions found by
`rizin -A`. No symbols — functions are identified by the log format strings
they reference.

Rebuild the analysis project:

```bash
udisksctl mount -b /dev/sda3 -o ro
cp "/run/media/mia/7A2C34DB2C3493DB/Program Files/Oculus/Support/oculus-runtime/server-plugins/Rift.dll" .
rizin -q -A -c 'Ps rift.rzdb' -c 'q!' Rift.dll      # ~20 s
rizin -q -p rift.rzdb -e scr.color=0 -c 's 0x180134cd0' -c pdg -c q!
```

## Pipeline

```
TrackedObject state machine        fcn.18019a5e0 / fcn.1801b3c70 / fcn.18019b770
  └─ reconstruction driver         fcn.18017f370   "Failed frame … Reprojection error: %.3f pix"
       └─ multi-cam gather         fcn.180100b20
            └─ POSE RECONSTRUCTION fcn.18010cac0   "… cameras %d, matches %d, distance %.1f"
            └─ RANSAC match        fcn.18010bdf0   "RansacMatch: Too many outliers"
            └─ outlier loop        fcn.18010dfe0   "Outlier %d: %.3f (imp %.0f%%) …"
       └─ back-of-head recon       fcn.180103ff0   (calls fcn.18010cac0 again)
       └─ brute-force bootstrap    fcn.180100f10   "%d, cam %d: Brute failed: %s"
  └─ EKF driver                    fcn.180134cd0   saturation, freeze, late pose
       └─ EKF vision update        fcn.18013a920   "Ekf Update", "Update Pose", "CameraPoseChange"
       └─ EKF reset                fcn.180138780   "Ekf Reset: …", "Hard/Soft ResetEkf"
       └─ EKF reset (sigmas)       fcn.180132a20   "Ekf Reset: Sigmas: pos %.1f, vel %.1f, orient %.2f"
       └─ EKF init state           fcn.180141610   "Initial State: AccBias … GyroBias … Gravity"
       └─ EKF core (generic)       fcn.1801896e0 / fcn.18017d9e0 / fcn.18017cae0
                                   "EKF: P not positive definite", "Update: Invalid P or K"
  └─ gravity aligner               fcn.18011ab20   "EKF Gravity alignment in %.1f ms"
       └─ gravity filter update    fcn.180146860   "Gravity filter update: change/weight/conf/reliable"
       └─ gravity correction       fcn.1801376b0   "gravity error: %.2f: correction … clamped …"
       └─ computed alignment       fcn.180141610   "Computed gravity alignment: camera %d …"
       └─ inclinometer             fcn.18011d9d0 / fcn.18011d1b0
  └─ camera calibration            fcn.18018d430 (55 KB!)  "Good/Bad calibration for camera %d"
       └─ multicam bundle adjust   fcn.18018f9b0   "Multicam calibration computed: residual %.3f --> %.3f pix"
       └─ fast calibrate           fcn.180187430 / fcn.180189320 / fcn.1801889d0
```

## Recovered constants

| function | constant | reading |
|---|---|---|
| `fcn.18017f370` | `715`, `2/715 = 0.002797…` | camera focal length in px; **2 px reprojection acceptance** for a reconstruction |
| `fcn.18010cac0` | `1/650`, `1/2043`, `0.1` | reconstruction internals (unresolved) |
| `fcn.18010bdf0` | `0.75`, `0.01` | RANSAC allowed-outlier fraction / tolerance |
| `fcn.18018f9b0` | `715`, `100/715`, `0.85`, `100` | bundle adjust; new/old residual must be `< 0.85` (≥15 % improvement) or it is rejected as "Insufficient Multicam calibration improvement" |
| `fcn.1801376b0` | **`9.71`, `9.91`** | **gravity magnitude acceptance band, m/s²** |
| `fcn.1801376b0` | `8.7266e-5` (0.005°), `1.7122e-3` (0.098°), `0.0037`, `0.0076` rad | gravity correction step / clamp |
| `fcn.180146860` | `5.236e-3` (0.3°), `2.618e-2` (1.5°), `5.236e-2` (3°), `8.727e-2` (5°) | gravity-filter change / "large" / confidence tiers |
| `fcn.18011ab20` | `0.0076` rad (0.435°) | "filter stuck, too much movement" std threshold |
| `fcn.180141610` | `8.727e-3` (0.5°) | alignment tilt tolerance |
| `fcn.180134cd0` | `9.80667`, `750`, `0.01`, `0.25` | gravity prior; EKF update |
| `fcn.180132a20` | `2.5e-5` (=5 mm²), `9e-6` (=3 mm²), `0.5`, `3` | reset sigmas |

## Notes

- `IndirectEkf<18,9>` is the only estimator template instantiated.
  `RiftVision::ComplementaryFusionFilter` (with an `ExposureRecord` ring) is a
  separate, secondary class.
- `fcn.18010cac0` is called from **both** `fcn.180100b20` (main body) and
  `fcn.180103ff0` (back-of-head), i.e. the same reconstruction routine serves
  both LED groups.
- `fcn.180134cd0` is the only caller of `fcn.18013a920`, so the vision update is
  strictly nested inside the IMU/EKF driver.
