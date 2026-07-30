# Where the CV1 keeps its IPD

Short answer: **it does not report one.** It reports an *inter-axial distance*
(IAD) as a calibrated baseline plus accumulated adjustments, and the runtime
derives IPD from that. Our `display_info.lens_separation` is the baseline.

## What the runtime calls it

`LibOVRRTImpl64_1.dll` and `OVRServer_x64.exe` both carry:

```
UpdateGeneratedState: IPD = %g, IAD = %g, screen_sep = %g
```

Decompiled (`fcn.18018c030` in LibOVRRTImpl64_1.dll, found by brute-force
RIP-relative scan — `axt` resolves nothing here, same as `Rift.dll`):

```
IPD        = [rdx+0x88]  - [rdx+0x50]
IAD        = [rdx+0x804] - [rdx+0x7e4]
screen_sep = [r8+0x20]   + [r8+0x24]
```

So **IPD is generated, not sensed** — a difference of two eye-transform fields.
The hardware quantity is the IAD.

## What the hardware plugin tracks

`server-plugins/Rift.dll` has no `ipd` handling worth the name, but plenty of
IAD:

```
SupportsIADAdjust          a per-device capability flag
IADChanged                 IAD ChangeStart:      IAD ChangeEnd:
calibrated_iad   iad_adjust_changes   iad_adjust_distance
LensConfigurations[%d].EyeRelief / .LensToScreen / .MetersPerTanAngleAtCenter
Unable to read lens serials for device
```

`fcn.1800a12b0` serialises three fields out of one struct:

| source | type | key |
|---|---|---|
| `[r14+0x14]` | float | `calibrated_iad` |
| `[r14+0x18]` | int | `iad_adjust_changes` |
| `[r14+0x20]` | double | `iad_adjust_distance` |

A **calibrated baseline, a count of adjustments, and accumulated travel.** That
is a relative encoder integrated from a stored zero, not an absolute readout —
which explains everything measured on the hardware:

- `DISPLAY_INFO` (report 0x09) reports 63.5 mm and **does not move** with the
  slider. That figure is the `calibrated_iad` baseline.
- Sweeping feature reports `0x01..0x20` across a slider change showed **no**
  field tracking it: everything that differed was an enumeration cursor
  (`POSITION_INFO`, `PATTERN_INFO` advance per read) or a single-byte counter.
- OpenHMD has logged `unknown HMD message type` **zero** times, so no
  unrecognised input report is arriving either.

## What that means for us

`props->ipd` is now sourced from `display_info.lens_separation`, i.e. exactly
the `calibrated_iad` baseline. To do better one would have to catch the change
events the runtime counts in `iad_adjust_changes` and integrate them — which
means finding how they are delivered, not just where they are stored. Tracing
what writes `[r14+0x14]` is the next step if it is ever worth it.

Worth weighing first: moving the rendered IPD from 61 mm to 63.5 mm was
**imperceptible** in the headset, so the remaining error is a few millimetres
of slider travel at most.

## Hazard

Sweeping feature report ids the driver does not otherwise use **wedges the
headset** — `error sending keepalive` lands right after `0x0a`, and twice the
HMD went unresponsive, once needing a reboot. `OHMD_RIFT_DUMP_REPORTS=1` now
stops at `0x0a`; `=all` sweeps further and says so.

## Reproducing

```bash
udisksctl mount -b /dev/sda3 -o ro
cd "/run/media/mia/*/Program Files/Oculus/Support/oculus-runtime"
rizin -q -A -c 'Ps ovrrt.rzdb' -c 'q!' LibOVRRTImpl64_1.dll
```

Then find string xrefs by scanning `.text` for `48/4c 8d modrm(mod=00,rm=101)`
whose `rip+disp32` lands on the string — rizin's own `axt` misses them.
