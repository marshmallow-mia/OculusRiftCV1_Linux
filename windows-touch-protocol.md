# How the Oculus Windows runtime does Touch calibration & tracking (wire-level)

Sources: `captures/win/2026-07-12/{setup_hid.pcap, imu_tracking.pcap}` (runtime
v1.77 / client 32.1.1, same physical HMD + controllers as the Linux rig),
runtime output CSVs from the same day, plus local experiments on our rig's
hardware. Companion analysis: `controller-tracking-analysis.md` (disease A/B).

## 1. Factory calibration: what is stored and how it is applied

### Touch flash blob (read over radio, cached by our driver in ~/.config/openhmd)

4-byte header (`01 00` + u16 JSON length) + JSON `{"TrackedObject": {...}}`,
JsonVersion 2. Both the runtime (pcap §3 below) and our driver read the SAME
bytes — the runtime's reassembled blobs from the pcap are byte-identical to our
cached `rift-touch-config-*.bin`. Keys: ImuPosition, GyroCalibration[12],
AccCalibration[12], ModelPoints (24 LEDs), IrLedConfig(=1850), Lensing,
joystick/trigger/grip ranges, CapSense tables.

The 12-float calibration blocks = 3x3 matrix (row-major) + 3 offsets:

- The matrices are near-orthonormal AXIS PERMUTATIONS (chip axes -> model
  axes), mirrored left vs right:
  - left  gyro: cal_x=raw_y, cal_y=raw_z, cal_z=raw_x (+corrections ~1%)
  - right gyro: cal_x=-raw_y, cal_y=raw_z, cal_z=-raw_x
- Offsets: left accel [-0.062, -0.039, +0.389] m/s², right [+0.131, -0.047,
  -0.551] m/s² — dominant 3rd component, classic MEMS Z-axis bias signature
  (suggests offsets live in the CHIP frame, i.e. subtract BEFORE the matrix).

### HMD flash calibration (HID feature report 0x03, 69 bytes)

Same 12-value scheme, packed 3x21-bit: accel_offset, gyro_offset (1e-4 units),
then interleaved accel/gyro matrix rows (scale 1/(2^20-1), +1.0 on diagonal).
Our HMD: accel_matrix diag [1.021, 1.037, 0.963], accel_offset
[0.108, -0.306, 0.025] m/s², gyro_offset [-0.0039, 0.0013, 0.0011] rad/s.

### THE CONVENTION — measured, not assumed

Live experiment on our rig (HMD at rest, raw hidraw/pyusb stream, scratchpad
`calib_sign_usb.py`): the flash **gyro_offset equals the measured raw rest
bias** (raw mean [-0.00395, 0.00085, 0.00199] vs stored [-0.0039, 0.0013,
0.0011]):

| convention | residual gyro at rest |
|---|---|
| `M@raw + b` (OpenHMD today) | **0.500 °/s** (doubles the bias) |
| `M@raw - b` / `M@(raw - b)` | **0.057 °/s** |

The offsets are biases TO SUBTRACT. OpenHMD's `rift.c` ADDS them (HMD path
`rift.c:261-268`, Touch path `rift.c:394-399`) — for Touch accel that is an
error of 2x|b| = 0.79 (L) / 1.13 (R) m/s² => **4.6° / 6.6° of phantom gravity
tilt**, matching the constant ~8° fusion-vs-vision tilt conflict (disease A in
`controller-tracking-analysis.md`).

Accel magnitude check at one orientation: raw |g|=9.397; subtract => 10.06,
add => 9.44 (neither exact — accel factory matrix is only good to ~2-3% at our
temperature; the sign question is settled by the gyro, which is unambiguous).

**Confirmed by the official LibOVR runtime source** (SDK 0.4.x,
`OVR_SensorCalibration.cpp::Apply()`):

```cpp
msg.RotationRate = GyroMatrix.Transform(msg.RotationRate - gyroOffset);
msg.Acceleration = AccelMatrix.Transform(msg.Acceleration - AccelOffset);
```

i.e. **subtract in the chip frame, then matrix** — `M @ (raw - b)`. Same in
OpenHMD master (PR #231: "Apply correction offsets first ... then the 3x3
rotation matrix") and in Jan Schmidt's own newer Monado rift_s driver, where
Meta's Rift S JSON literally names the fields `acc_m`/`acc_b` and
`RectificationMatrix`/`ConstantOffset`. Our vendored branch's add-after-matrix
dates to thaytan commit `e2f8be1` (2021-08-24, "seems more correct based on
monitoring raw IMU outputs") — judged against HMD reference code that had an
in-place aliasing bug at the time (fixed in the same day's `d43b551`), which
corrupted the comparison. ouvrt parses the offsets but never applies them
(neutral). The runtime additionally runs temperature-interpolated gyro-offset
auto-calibration seeded by the factory value; accel offset is used as-is.
Optional hardware confirmation for Touch: scratchpad `touch_raw_capture.py`
(15 s raw gyro at rest, awake controllers) — raw bias should equal `b`.

Oracle fits against the runtime's own CSV output validated time alignment
(corr 0.989) but the runtime's exported poses/velocities are
prediction-smoothed, so amplitude-sensitive fits are biased (gyro matrix fit
attenuated ~5%; accel preintegration rms 40 mm) — the runtime's *outputs*
cannot recover its *input* calibration precisely. Rest-bias physics above is
the reliable evidence.

## 2. Radio protocol: what the runtime actually sends (complete catalog)

Wire facts (setup_hid.pcap, 350 s incl. controller wake at t=167; imu pcap
93.7 s worn):

- Device types on the wire: 0x05 = HMD-local radio, **0x02 = LEFT Touch,
  0x03 = RIGHT Touch** (our driver already matches: rift.h 280-282).
- Command = SET 0x1a `[1a 00 00 a b c]`; parameter = preceding SET 0x1b with
  u16 at bytes [3:5]; busy-poll via GET 0x1a bit 0x80 of byte 3.

### At HMD init
- (0x05,0x03,0x05) read radio address; (0x05,0x82,0x05) radio fw version.
- (0x04,0x12,0x05) param 8 — unknown.
- **(0x04,0x02,0x05) param 19200** — programs the LED/exposure sync period
  (µs) into the HMD radio. Re-sent at every tracking start and after each
  controller's flash read. OpenHMD NEVER sends this.

### At controller wake (per controller, once)
1. (0x02,0x17,dev) param 2
2. (0x03,0x88,dev) read serial; (0x03,0x82/0x86,dev) fw version
3. flash calibration read loop: (0x03,0x0a,dev) x203, 20 B / 36 ms per read
4. (0x02,0x19,dev) param 10080; (0x03,0x1c,dev) read (resp 0xff...);
   (0x02,0x04,dev) param 35; (0x02,0x13,dev) param 0
OpenHMD sends only the flash read (3) — none of the config writes.

### Periodic maintenance while controllers are awake (THE disease-B lead)
- **(0x02,0x18,dev) param 60, every ~30 s per controller** — the ONLY
  recurring per-controller command. Shape = "stay active for 60 <units>,
  refreshed at half-life". OpenHMD never sends it.
- Nothing else: after the last command, both controllers streamed 47 s
  gap-free at ~494 msg/s. During Sensor-Setup optical tracking (183 s awake)
  there are ZERO optical/stream dropouts on Windows — vs our synchronized
  15.9 s LED-schedule pauses at rest.
- HMD DK2_KEEP_ALIVE (0x11, interval 10000 ms) re-sent every 3.0 s.
- GET 0x1f (radio/link telemetry) and GET 0x22 (uptime/status) each ~60 s.

### Haptics
(0x02,0x16,dev) bursts (~44 ms apart) writing amplitude ramps into the 256 B
ring buffer at incrementing u16 positions — a richer scheme than our
single-amplitude (0x02,0x03,dev) writes. (0x02,0x03,dev) param 0 appears
event-driven (likely "haptics stop"/buffer reset), always both controllers.

### Sleep behavior
Asleep controllers emit one beacon per 750.0 ms (report 0x0c, our "radio
message" report); awake they stream at 2 ms spacing. Report 0x0d (all-zero
payload) is an empty-slot heartbeat at ~250 ms when nothing is paired/awake.

## 3. Camera / LED config (unchanged from findings-4a-hid.md)
TRACKING_CONFIG 0x0c: pattern 0xff, ENABLE|USE_CARRIER, exposure 399 µs,
period 19200 µs, duty 0x7f — byte-identical to OpenHMD's constants; disabled
to carrier-only + SENSOR_CONFIG flag tweak (0x20->0x13) at tracking stop.
The runtime reads POSITION_INFO (0x0f) x85 for the HMD LED sweep but NEVER
reads PATTERN_INFO (0x10) — LED blink IDs come from the radio-synced schedule,
not per-LED pattern queries.

## 4. Other findings from the open-source RE corpus

- **Touch LEDs carry NO blink-ID patterns** (OpenHMD issue #189; parser sets
  `pattern = 0xff`): controller LED disambiguation is correspondence-search
  only. The "LED schedule" the radio maintains is the exposure-synchronized
  illumination itself, not ID modulation.
- thaytan (noraisin.net p=986): "the controller LED models are supplied in a
  different orientation to the HMD" — relevant to any residual tilt after the
  offset fix.
- Touch gyro scale: MPU-6500 at ±2000 °/s → 16.384 LSB/°/s (our driver has
  this right; ouvrt/OpenHMD-master's 2.0/2048 is ~10% off).
- The Windows runtime caches controller calibration as `LTOUCH`/`RTOUCH`
  files in `%LOCALAPPDATA%\Oculus` (same JSON) — inspectable on the Windows
  disk (sda3) if we ever mount it.
- No open driver sends ANY periodic radio maintenance — the ~30 s watchdog
  discovery in §2 is not documented anywhere public.

## 5. Fixes implemented in our driver (2026-07-12 night)

1. **Disease A — offset convention** (`rift.c`): both HMD and Touch IMU paths
   now use `apply_imu_calibration()` with `M @ (raw - b)` by default.
   `OHMD_RIFT_CALIB_OFFSET=add` restores legacy, `=model` gives `M@raw - b`
   for A/B. Expected: constant tilt conflict drops from ~8° to ~1-2°
   (Touch), HMD phantom tilt of ~3.7° also removed. NOTE: the room config
   was gravity-calibrated under the legacy convention — after validating,
   re-run `calibrate_room.py setup` (world is ~3.7° off gravity-true under
   the new convention).
2. **Disease B — radio maintenance** (`rift.c`, `rift-hmd-radio.c`): new
   `rift_radio_send_cmd_u16()`; per-controller wake-config sequence
   (0x17=2, 0x19=10080, 0x04=35, 0x13=0, then HMD sync period
   (0x04,0x02,0x05)=19200) sent after calibration read and re-sent when the
   IMU stream resumes after a >5 s gap; (0x02,0x18,dev)=60 watchdog every
   30 s while awake. `OHMD_RIFT_NO_RADIO_MAINT=1` disables for A/B.
   Validation: at-rest controller capture, look for the synchronized ~16 s
   optical gaps.
3. Haptics could adopt the ring-buffer write scheme later (fidelity, not
   tracking).

Deployed as driver hash `b8cf54f9ca41` (build + bin/linux64 verified equal).

## 6. Validation (2026-07-12/13 night, on-rig)

- **Raw Touch accel** (awake controllers at rest, `touch_raw_capture.py`):
  |g| error with subtract = +0.02 (L) / −0.05 (R) m/s²; with legacy add =
  −0.46 / +0.85. Convention proven on Touch hardware directly. Chip vs model
  frame indistinguishable by magnitude (both < 0.06); chip kept per LibOVR.
- **Disease B**: 120 s at-rest instrumented capture, radio maintenance on:
  ZERO optical gaps > 1 s on HMD + both controllers (previously synchronized
  15.9 s dropouts within a minute), and the controllers kept streaming the
  full 120 s at rest (previously slept/stopped after ~20 s). Wake-config +
  watchdog encodings verified byte-identical to the Windows wire (incl.
  0x18=60 every ~30 s; Windows' 0x1b trailing bytes are stale heap garbage,
  zeros are fine).
- **Disease A in-hand A/B** (controllers held up facing sensors, 50 s each):
  fused-vs-optical tilt right 7.1°→3.7°, left 11.8°→6.0° (legacy→subtract);
  new-convention residuals equal the cross-camera optical split (5-6°), i.e.
  no measurable systematic fusion tilt remains. Deltas match the predicted
  2|b|/g error.
- **Metric caveat**: fused-vs-optical tilt at arbitrary DESK resting poses is
  dominated by correlated PnP tilt ambiguity of the sparse ring (measured up
  to ~27° with both cameras agreeing) — do not use desk captures to judge
  orientation health; use in-hand captures or |g| tests.

Still pending: in-headset SteamVR feel test; room re-calibration under the
new convention (`calibrate_room.py setup` — config world is ~3.7° off
gravity-true now); moving-case regression (score_prediction.py).
