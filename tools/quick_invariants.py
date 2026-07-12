#!/usr/bin/env python3
"""Print the gauge-free invariants of an ovr_static_dump JSON, plus the
known values from our room calibration (plan section 0) for eyeballing.
Full comparison needs rift-room-config.json -> compare_extrinsics.py."""
import json
import math
import sys

d = json.load(open(sys.argv[1]))
print("origin:", d["tracking_origin"], "  runtime:", d.get("runtime_version"))
for t in d["trackers"]:
    p = t["pose"]["position"]
    q = t["pose"]["orientation"]
    print(f"tracker {t['index']}: valid={t['pose_valid']} "
          f"pos=({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f}) "
          f"quat=({q[0]:+.4f}, {q[1]:+.4f}, {q[2]:+.4f}, {q[3]:+.4f})")

t0, t1 = d["trackers"][:2]
p0, p1 = t0["pose"]["position"], t1["pose"]["position"]
q0, q1 = t0["pose"]["orientation"], t1["pose"]["orientation"]

def conj(q):
    return [-q[0], -q[1], -q[2], q[3]]

def mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz]

def rot(q, v):
    p = mul(mul(q, [v[0], v[1], v[2], 0.0]), conj(q))
    return p[:3]

rel = mul(conj(q0), q1)
ang = math.degrees(2 * math.acos(min(1.0, abs(rel[3]))))
a0, a1 = rot(q0, [0, 0, -1]), rot(q1, [0, 0, -1])
axis = math.degrees(math.acos(max(-1.0, min(1.0, sum(x*y for x, y in zip(a0, a1))))))

print(f"baseline         : {math.dist(p0, p1):.4f} m     (ours, plan s0: 2.3628 m)")
print(f"relative rotation: {ang:.3f} deg")
print(f"optical-axis angle: {axis:.3f} deg")
print(f"heights          : {p0[1]:+.4f} / {p1[1]:+.4f} m "
      f"(ours, above floor: 1.6426 / 1.4432)")
