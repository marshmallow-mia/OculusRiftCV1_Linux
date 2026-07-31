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

Notably absent: the `9.80667, 750, 0.01` previously attributed to `180134cd0`,
and the `2.5e-5, 9e-6` reset sigmas attributed to `180132a20`. Those readings
came from a different extraction method and one of the two is wrong; this pass
scanned rizin's resolved `data.*` operands over the whole function body, which
should be a superset of a manual scan. **Unresolved — flagging rather than
silently overwriting the earlier note.**

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
how the `dump_csv` gate was found. Sections in this image map with a uniform
delta of `0x180000C00` (vaddr = paddr + delta), which the script derives.

Caveat learned this pass: a scanner limited to `movss`/`movsd` loads misses most
constants, because MSVC folds them into arithmetic (`mulsd xmm,[rip+d32]`).
Reading rizin's resolved `data.*` labels out of `pdf` output is both easier and
more complete.

## Still open

1. **Where `dump_csv` is read from**, and whether it can be enabled under Wine.
   Highest value by a wide margin.
2. **The `IndirectEkf` constructor** — the only place Q, R and the initial
   covariance can come from.
3. **The propagation model.** Not attempted this pass.
4. **What the `9` in `<18,9>` is.** Still unstated anywhere.
5. **The nominal/error-state injection convention.**
6. **Per-observation R.** Known qualitatively only.
7. **Reconciling the constants** this pass found against the earlier note.
8. The gravity aligner's coupling into the EKF states, which is what made a
   previous attempt at gravity states inert.
