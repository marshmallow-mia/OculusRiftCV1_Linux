# §4a findings — what the Oculus runtime writes to the CV1 that we don't

Source: `captures/win/2026-07-12/{setup_hid,imu_tracking}.pcap` (Windows test PC,
Meta Horizon v32.1.1 / runtime 1.77, HMD `WMHD316C1006SW` fw 7.9).
Reproduce with `tools/analyze_hid_pcap.py <pcap> [--imu]`.

These results are **environment-independent** — they are properties of the HMD
protocol, not of the room — so they survive the fact that the Windows rig is a
different physical environment than the Linux rig (see `HANDOFF-LINUX.md`).

## 1. Ruled out: LED / exposure / camera-sync config is already identical

The runtime's `TRACKING_CONFIG` (0x0c) write is **byte-for-byte what OpenHMD
already sends**:

```
0c 0000 ff 05 00 8f01 004b 0000 7f
     pattern=0xff  flags=0x05 [ENABLE|USE_CARRIER]
     exposure=399 us  period=19200 us (52.08 Hz)  vsync_offset=0  duty=0x7f
```

OpenHMD `rift.c:1027-1029` sets exactly `pattern=0xff`,
`flags=ENABLE|USE_CARRIER`, and `rift.h:79-83` defines
`EXPOSURE_US_CV1=399`, `PERIOD_US_CV1=19200`, `VSYNC_OFFSET=0`,
`DUTY_CYCLE=0x7f`. Identical.

**So the "are our LEDs configured worse than theirs?" hypothesis in plan §4a is
dead.** Our blob detector is working on the same illumination theirs is. Good
news, and it removes a whole branch of the search.

(Note for the plan: the LED/camera cadence is **~52 Hz**, not the 60 Hz the plan
text assumed. Measured exposure events in the IMU stream: **51.36 Hz** over 93 s,
matching the 19200 µs period. Corrected in `cv1-capture-plan.md`.)

## 2. Ruled out: our IMU decode is correct

Running our driver's exact struct layout (`packet.c` `decode_tracker_sensor_msg_dk2`,
`decode_sample`'s 3×21-bit unpack, `vec3f_from_rift_vec`'s 1e-4 scale) over the
runtime's *own* raw HID reports:

```
reports 32815   samples 65630   span 93.6 s
|accel| median 9.469 m/s^2   (expect 9.81)  -> gravity check OK
|gyro|  median 0.948 rad/s   p95 3.22       (headset was worn/moving ~45%)
temperature 58.0 C
```

Gravity lands where it should, so the 64-byte layout, the 21-bit sign extension
and the LSB scale are all right. The ~3.5 % shortfall from 9.81 is expected:
these are *raw uncalibrated* samples (see §3 — the runtime never asks the device
to apply factory calibration), and the factory `accel_matrix` supplies the scale
correction.

**Caveat — do not use this pcap for timing analysis.** Device timestamps advance
~2003 µs between consecutive reports (2 samples/report at 1 kHz = correct), but
only 350 reports/s of the expected 500/s are present in the capture. That is
USBPcap dropping under load, not the device. Layout/scale conclusions hold;
latency/jitter conclusions do not.

## 3. Investigated and RULED OUT: on-HMD auto-calibration

> **Verdict (tested on hardware, 2026-07-12): the flag does not stick. Dead end.**
> The reasoning below is kept because it was the leading hypothesis and the
> negative result is worth recording — but do not act on it.
> See §3.1 for the experiment that settled it.

The runtime's one and only `SENSOR_CONFIG` (0x02) write, for the whole session:

```
02 0000 20 01 e803
     flags = 0x20 [COMMAND_KEEP_ALIVE]
     packet_interval = 1
     keep_alive = 1000 ms
```

`flags = 0x20`. That is **COMMAND_KEEP_ALIVE and nothing else**. In particular:

| flag | Oculus runtime | OpenHMD (`rift.c:1157-1158`) |
| --- | --- | --- |
| `RIFT_SCF_USE_CALIBRATION` (0x04) | **not set** | **force-set to 1** |
| `RIFT_SCF_AUTO_CALIBRATION` (0x08) | **not set** | **force-set to 1** |
| `RIFT_SCF_COMMAND_KEEP_ALIVE` (0x20) | set | (whatever the device reported) |
| `RIFT_SCF_SENSOR_COORDINATES` (0x40) | not set | set iff coord frame is SENSOR |

```c
// rift.c:1155-1158 — enable calibration, but these don't seem to stick.
// We check later if we need to do it manually
SETFLAG(priv->sensor_config.flags, RIFT_SCF_USE_CALIBRATION, 1);
SETFLAG(priv->sensor_config.flags, RIFT_SCF_AUTO_CALIBRATION, 1);
```

`AUTO_CALIBRATION` asks the **HMD's own firmware** to continuously auto-calibrate
its gyro zero-rate offset. The Oculus runtime never turns it on — it estimates
bias inside its own fusion filter instead. And so do we: our OVR fusion port
(`rift-fusion-ovr.c`) runs its own accel-bias estimation, and the UKF backend
estimates gyro bias as a filter state.

**That means we plausibly have two bias estimators fighting each other** — the
HMD silently walking its own zero-rate offset underneath a filter that is
simultaneously trying to estimate that same offset. A slow, correlated,
unobservable disturbance under the filter is exactly the shape of the residual
drift/wobble we have been chasing, and it is *not* fixable by tuning the filter,
which is consistent with why calibration came out optimal (0.3 mm) yet the
wobble persisted.

### 3.1 The experiment that killed it

OpenHMD's own comment says these flags **"don't seem to stick"**. Rather than
trust either the comment or the hypothesis, `tools/probe_sensor_config.py --test`
writes the report and reads it straight back over hidraw (no root needed; udev
rules from `install.py` already grant access):

```
as found:      0200002000e803   flags=0x20 [COMMAND_KEEP_ALIVE]  packet_interval=0

[1] writing OpenHMD's flags: 0x2c [USE_CALIBRATION|AUTO_CALIBRATION|COMMAND_KEEP_ALIVE]
    readback:  0200002000e803   flags=0x20 [COMMAND_KEEP_ALIVE]
    USE_CALIBRATION  (0x04) stuck: False
    AUTO_CALIBRATION (0x08) stuck: False

[2] writing Oculus's flags:  0x20 [COMMAND_KEEP_ALIVE]  interval=1  keep_alive=1000
    readback:  0200002001e803   flags=0x20 [COMMAND_KEEP_ALIVE]  packet_interval=1
```

The CV1 firmware **accepts** `packet_interval` and `keep_alive` (0 → 1 stuck) but
**silently drops** both calibration flags. Its power-on default is already
`flags=0x20` — byte-identical to what the Oculus runtime writes.

So OpenHMD's `SETFLAG` calls at `rift.c:1157-1158` are **no-ops on CV1**. There is
no on-HMD auto-calibration running, no second bias estimator, and no conflict with
our filter. Both stacks receive raw uncalibrated IMU and apply the factory
calibration in software — exactly as `rift.c:262-269` already assumes.

The only surviving difference in `SENSOR_CONFIG` is `packet_interval`: Oculus
writes 1 (two samples bundled per HID report, ~500 reports/s), we leave the device
default of 0 (one sample per report, ~1000 reports/s). Both deliver the same 1 kHz
sample stream; this is USB transaction efficiency, not tracking quality. Not worth
changing.

## 4. Reports the runtime uses that OpenHMD has never heard of

Written by the runtime, unknown to `rift.h`:

| report | count | payload | guess |
| --- | --- | --- | --- |
| `0x0d` | 4 | `0d 0000 ff 00 04 0000 0801 0000 6400` | second tracking/pattern config; shape mirrors 0x0c |
| `0x21` | 19 | `21 0000 <idx> 80 00…` (69 B, idx ∈ {04,21,23,24,…}) | indexed upload — LED pattern or radio/controller firmware |
| `0x06` | 1 | `06 000000` | — |
| `0x00` | 5 | — | — |

Also *read* but never read by us: `0x13`, `0x15`, `0x1e`, `0x1f`, `0x22`.

Worth reverse-engineering `0x0d` eventually, but it is lower priority than §3:
the LED config we can already see (0x0c) is identical, so 0x0d is unlikely to be
changing illumination in a way that hurts us.

## 5. Optional: full wire-level diff of our own traffic

§3.1 answered the question that mattered without root. If we later want the
complete picture of what *we* put on the wire (e.g. to chase `0x0d`), the same
analyzer runs on a Linux usbmon capture:

```bash
sudo modprobe usbmon
lsusb -d 2833:0031                       # -> bus number
sudo tshark -i usbmon<BUS> -w captures/lin/$(date +%F)/openhmd_hid.pcap &
~/git/SteamVR-OpenHMD/build/subprojects/openhmd/openhmd_simple_example   # ~60 s
python3 tools/analyze_hid_pcap.py captures/lin/<date>/openhmd_hid.pcap --imu
```

Not urgent — the source already tells us what we write, and §1/§2/§3.1 confirmed
the three things that could have differed do not.

## Standing ranking of known gaps

1. **Prediction — implemented 2026-07-12, but the original diagnosis was wrong.**
   We were never at "zero prediction": SteamVR extrapolates the driver pose to
   photon time itself, from the velocities we hand it. The real defects were that
   `driver_openhmd.cpp` hardcoded `poseTimeOffset = 0` (claiming a pose that is
   actually **1.00 ms** stale is fresh, so SteamVR under-predicted by that much)
   and never populated `vecAcceleration` (so the extrapolation stayed
   first-order). Both fixed; see `PREDICTION.md`. **Quality during motion is still
   unmeasured** — the validation log came back stationary.
2. Unknown reports `0x0d` / `0x21` — unquantified, probably minor (the LED config
   we *can* read is identical, so they are unlikely to affect illumination).
3. ~~LED / exposure / camera-sync config~~ — **ruled out** (§1), byte-identical.
4. ~~IMU decode, layout, scale~~ — **ruled out** (§2), validated against their raw stream.
5. ~~On-HMD auto-calibration fighting our filter~~ — **ruled out** (§3.1), the CV1
   firmware drops the flag; OpenHMD's `SETFLAG`s are no-ops.

Net effect of this session: three of the four standing suspects are eliminated, and
the search narrows hard onto **forward prediction** — which is also the one we have
a measured number for.
