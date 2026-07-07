# Room calibration (offline bundle adjustment)

Camera poses are **static at runtime** — the driver no longer refines them
while tracking (the old servo was an un-anchored feedback loop that made the
calibration random-walk). Instead, calibration is a capture → solve → write
cycle done offline with `calibrate_room.py`.

## Sensor Setup (the normal path)

The control center's **Sensor Setup** button runs a 1:1 recreation of the
Windows Oculus app's CV1 sensor setup — same step order, titles, copy and
look (verified against the client bundle, whose step machine runs
bandwidth_check → intro → height → prepare space → place sensors →
tracking → confirm). Under the hood each capture step runs a short
standalone OpenHMD session (`openhmd_simple_example`) with
`OHMD_RIFT_CAL_CAPTURE` pointing at a phase file in
`~/.cache/rift-cv1-center/`, then:

    calibrate_room.py setup hold.jsonl standing.jsonl --height 1.75

1. **Tracking capture** ("move the headset from side to side, down towards
   the floor, then in front of your head"): stage-1 closed-form relative
   pose + a quick bundle adjustment over ~400 exposures solves the sensor
   extrinsics. Gravity alignment is recovered from the driver's own fused
   poses (`world_T_cam = world_T_dev · inv(cam_T_dev)`, robustly averaged),
   so it works even without an existing room config.
2. **Confirm capture** (stand at the play-area centre facing the sensors):
   a rigid world re-anchor puts the origin on the floor under the headset,
   the floor at `y = 0` via your entered height (eye height = height −
   0.103 m, the Oculus SDK's own offset), and forward (−Z) where the
   headset faces. Sensor placement is then validated like the Oculus
   client (distance from origin, equidistance, separation, aim angle) —
   failures show the original "too close / too far / center yourself"
   messages.

Without `--height` the anchor capture is instead interpreted as the
headset **resting on the floor** (origin under it, floor at its LED origin
minus 4.5 cm) — useful headless.

SteamVR must be closed; restart it afterwards to pick up the new config.
The walk-around cycle below remains the highest-accuracy option — Sensor
Setup gets you a correct, well-anchored calibration in under a minute,
the bundle-adjustment walk refines it with full-volume coverage.

## How it works

With `OHMD_RIFT_CAL_CAPTURE=<file>` set, the driver appends one JSON line for
every LED-ID-verified observation from each sensor (blob centroids + LED ids +
exposure timestamp + per-sensor PnP pose), plus one-time records of each
sensor's EEPROM intrinsics and each device's LED model.

The solver groups observations from both sensors by exposure timestamp — both
sensors saw the same LEDs at the same instant, which ties them rigidly:

1. **Stage 1 (closed form):** every co-observed exposure implies the same
   sensor-2→sensor-1 relative pose; a robust average over hundreds of
   exposures gives a near-exact estimate.
2. **Stage 2 (bundle adjustment):** jointly refines sensor 2's pose and every
   HMD pose by minimizing LED reprojection error over the whole capture
   (Huber loss, sparse Jacobian). The reference sensor keeps its pose from
   the existing config, so the world origin / yaw / floor do not move.

Validated on synthetic data: a 20 cm / 3° perturbation is recovered to
0.3 mm / 0.004°.

## GUI

The control center's **Room calibration (advanced)** window wraps this
whole cycle: it detects whether the running SteamVR has capture enabled
(with a copyable launch-option line if not), shows live per-device
observation rates from both sensors, progress bars against the four
coverage targets with wake/visibility hints, and runs solve / verify
with streamed output. A live top-down map shows the sensor view cones,
every visited 25 cm floor cell (green = co-observed and counted,
brighter with more height levels; grey = seen by one sensor only) and
your current position/heading, plus a 12-sector compass marking which
facing directions are still missing. The CLI below remains for
headless use.

## Procedure

1. Quit SteamVR. Delete any old capture file.
2. Launch SteamVR with capture enabled — either set it in SteamVR's Steam
   *Launch Options*:

       OHMD_RIFT_CAL_CAPTURE=/home/mia/cal-capture.jsonl %command%

   or export the variable and start Steam/SteamVR from that shell.
3. Put the headset on (or carry it) and move it slowly through the **whole
   play volume** for 3–5 minutes: different heights, depths, and yaw angles.
   Keep it visible to **both** sensors — only co-observed exposures constrain
   the calibration. Slow movement = less blur = better centroids.
   To check whether you've collected enough (works mid-capture, from a second
   terminal):

       venv/bin/python calibrate_room.py coverage /home/mia/cal-capture.jsonl

   It prints ok/not-ok against four targets (co-observed exposures, duration,
   positions visited, yaw directions) and says DONE when all pass.
4. Quit SteamVR, then solve:

       venv/bin/python calibrate_room.py solve /home/mia/cal-capture.jsonl

   Sanity-check the report (expect thousands of co-observed exposures,
   "after" position disagreement well under 10 mm). The old config is backed
   up next to it; restart SteamVR to use the new one.
5. Optional root-cause diagnostic for the residual 1.5–3 px reprojection band:

       venv/bin/python calibrate_room.py solve capture.jsonl --intrinsics

   Large fisheye-coefficient deltas would mean the EEPROM intrinsics
   interpretation is off (compare with Monado's rift driver). This mode never
   writes the config.

## Checking a calibration later

Any capture file doubles as a test set — no fitting involved:

    venv/bin/python calibrate_room.py verify capture.jsonl

reports how far apart the two sensors place the HMD (median/p95, mm) under
the current config. If it creeps up over weeks (a knocked sensor), just
re-run the capture + solve cycle.

## Notes

- `calibrate_room.py solve --devices 0,1,2` also uses Touch controller
  observations; default is HMD-only.
- `riftcv1/calibrate.py`, `calwizard.py`, `roomsetup.py` are earlier
  approaches (two-spot manual fit, online joint-cal) — superseded by this.
- The capture hook lives in `rift-sensor-pose-search.c`
  (`rift_cal_capture_*`); registration calls are in `rift-sensor.c` and
  `rift-tracker.c`.
