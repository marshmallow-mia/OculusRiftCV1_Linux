#!/usr/bin/env python3
"""SteamVR pose quality test — run while SteamVR is running.

Captures HMD + controller poses, prints live progress, then a per-device
stability analysis with verdicts. Used by rift_cv1_center.py (Advanced).
"""
import math
import statistics as st
import sys
import time

import openvr

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 35.0
HZ = 50
JUMP_M = 0.05

vr = openvr.init(openvr.VRApplication_Background)
cls_names = {openvr.TrackedDeviceClass_HMD: "HMD",
             openvr.TrackedDeviceClass_Controller: "Controller"}

devices = {}
for i in range(openvr.k_unMaxTrackedDeviceCount):
    c = vr.getTrackedDeviceClass(i)
    if c in cls_names:
        role = ""
        if c == openvr.TrackedDeviceClass_Controller:
            r = vr.getControllerRoleForTrackedDeviceIndex(i)
            role = {1: " L", 2: " R"}.get(r, "")
        devices[i] = cls_names[c] + role
print("devices found:", ", ".join(devices.values()), flush=True)

samples = {i: [] for i in devices}
t0 = time.time()
while True:
    el = time.time() - t0
    if el >= DURATION:
        break
    poses = vr.getDeviceToAbsoluteTrackingPose(
        openvr.TrackingUniverseStanding, 0.0,
        openvr.k_unMaxTrackedDeviceCount)
    for i in devices:
        p = poses[i]
        if p.bDeviceIsConnected:
            m = p.mDeviceToAbsoluteTracking
            samples[i].append((el, p.bPoseIsValid,
                               m[0][3], m[1][3], m[2][3]))
    if int(el) and int(el) % 5 == 0 and abs(el - int(el)) < 1.0 / HZ:
        print(f"  …{int(el)}/{int(DURATION)} s", flush=True)
    time.sleep(1.0 / HZ)

print("\n" + "=" * 52, flush=True)
for i, name in devices.items():
    ss = samples[i]
    if not ss:
        print(f"\n{name}: never connected", flush=True)
        continue
    valid = [s for s in ss if s[1]]
    print(f"\n{name}: {len(ss)} samples, "
          f"{100 * len(valid) // len(ss)}% valid", flush=True)
    if len(valid) < 20:
        continue
    frozen = True
    jumps = 0
    biggest = 0.0
    for a, b in zip(valid, valid[1:]):
        d = math.dist(a[2:5], b[2:5])
        biggest = max(biggest, d)
        if d > JUMP_M:
            jumps += 1
        if d > 1e-9:
            frozen = False
    for axi, ax in enumerate("xyz"):
        vals = [s[2 + axi] for s in valid]
        print(f"  {ax}: mean {st.mean(vals):+.3f} m   "
              f"std {st.pstdev(vals) * 1e3:7.1f} mm   "
              f"range {(max(vals) - min(vals)) * 1e3:7.1f} mm", flush=True)
    print(f"  jumps >5 cm: {jumps}   biggest: {biggest * 1e3:.0f} mm",
          flush=True)
    if frozen:
        print("  VERDICT: pose FROZEN — device is in standby "
              "(cover the proximity sensor / press a button)", flush=True)
    elif jumps == 0:
        print("  VERDICT: excellent — no tracking discontinuities",
              flush=True)
    elif jumps <= 5:
        print("  VERDICT: good — occasional re-acquire snaps "
              "(brief LED occlusion)", flush=True)
    else:
        print("  VERDICT: poor — frequent optical loss; device is often "
              "hidden from the sensor", flush=True)

print("\nNotes: 'mean y' well below 0 ⇒ run SteamVR Room Setup. "
      "Frequent jumps on one controller ⇒ it is occluded from the "
      "sensor; consider a second sensor on the opposite side.",
      flush=True)
openvr.shutdown()
