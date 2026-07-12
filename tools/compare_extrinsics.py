#!/usr/bin/env python3
"""
tools/compare_extrinsics.py — Tier-0 gauge-free sensor-geometry comparison.

Eats:
  - the JSON from tools/win/ovr_static_dump.exe   (Oculus's solved extrinsics)
  - our rift-room-config.json                     (OpenHMD room calibration)

Because the two stacks use different tracking origins (Oculus recenter/floor
convention vs our room calibration), poses are only comparable UP TO A GLOBAL
RIGID TRANSFORM. This script therefore compares only the invariants:

  * sensor-to-sensor relative transform  T_A->B = inv(T_A) . T_B   (the sharp test)
  * baseline distance                    ||pos_A - pos_B||
  * relative optical-axis angle between the two cameras
  * sensor heights above floor (only meaningful if Oculus origin is FloorLevel)

Acceptance gate (plan §2): relative rotation within ~0.5 deg, baseline within
~5 mm. Worse than that => our room calibration is the bug; stop and fix it
before touching fusion.

Usage:
  python compare_extrinsics.py captures/win/<date>/ovr_static.json \
                               path/to/rift-room-config.json
"""

import argparse
import json
import math
import sys

# ---------------------------------------------------------------- quaternion/pose math
# Quaternions are (x, y, z, w) throughout — matches both LibOVR and our room config.

def q_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    if n == 0:
        raise ValueError("zero quaternion")
    return tuple(c / n for c in q)

def q_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)

def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )

def q_rotate(q, v):
    """Rotate vector v by quaternion q."""
    p = (v[0], v[1], v[2], 0.0)
    x, y, z, _ = q_mul(q_mul(q, p), q_conj(q))
    return (x, y, z)

def q_angle_deg(q):
    """Rotation angle of quaternion, in degrees, in [0, 180]."""
    q = q_normalize(q)
    w = min(1.0, max(-1.0, abs(q[3])))
    return math.degrees(2.0 * math.acos(w))

def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

def v_norm(v):
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)

def v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

def angle_between_deg(a, b):
    c = v_dot(a, b) / (v_norm(a) * v_norm(b))
    return math.degrees(math.acos(min(1.0, max(-1.0, c))))


class Pose:
    """Rigid transform: world-from-sensor. p world position, q (x,y,z,w)."""

    def __init__(self, p, q):
        self.p = tuple(p)
        self.q = q_normalize(tuple(q))

    def inv(self):
        qi = q_conj(self.q)
        return Pose(tuple(-c for c in q_rotate(qi, self.p)), qi)

    def __mul__(self, other):
        return Pose(
            tuple(a + b for a, b in zip(q_rotate(self.q, other.p), self.p)),
            q_mul(self.q, other.q),
        )

    def optical_axis(self):
        # Camera looks down its local -Z (both LibOVR and OpenHMD convention).
        return q_rotate(self.q, (0.0, 0.0, -1.0))


def rel_transform(a: Pose, b: Pose) -> Pose:
    """T_A->B = inv(T_A) . T_B — fully gauge-free."""
    return a.inv() * b


# ---------------------------------------------------------------- loaders

def load_ovr(path):
    """Load ovr_static_dump.exe output -> list of (label, Pose), origin type."""
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    poses = []
    for t in d.get("trackers", []):
        if not t.get("connected", False):
            print(f"WARNING: OVR tracker {t.get('index')} not connected — skipped",
                  file=sys.stderr)
            continue
        p = t["pose"]["position"]
        q = t["pose"]["orientation"]
        poses.append((f"ovr[{t['index']}]", Pose(p, q)))
    return poses, d.get("tracking_origin", "Unknown"), d


def load_room_config(path):
    """Load OpenHMD rift-room-config.json -> list of (serial, Pose).

    Tolerates a couple of plausible schema layouts; fails loudly otherwise.
    """
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)

    entries = None
    if isinstance(d, dict):
        for key in ("sensors", "cameras", "trackers"):
            if key in d and isinstance(d[key], (list, dict)):
                entries = d[key]
                break
        if entries is None:
            # maybe the top level maps serial -> {pos, orient}
            if all(isinstance(v, dict) for v in d.values()):
                entries = d
    if entries is None:
        raise SystemExit(f"Unrecognised room-config schema in {path}; "
                         "inspect the file and adjust load_room_config().")

    def read_pose(e):
        pos = e.get("pos") or e.get("position")
        rot = e.get("orient") or e.get("orientation") or e.get("rot")
        if pos is None or rot is None:
            raise SystemExit(f"Sensor entry missing pos/orient: {e}")
        if isinstance(pos, dict):
            pos = (pos["x"], pos["y"], pos["z"])
        if isinstance(rot, dict):
            rot = (rot["x"], rot["y"], rot["z"], rot["w"])
        return Pose(pos, rot)

    poses = []
    if isinstance(entries, dict):
        for serial, e in entries.items():
            poses.append((serial, read_pose(e)))
    else:
        for e in entries:
            serial = e.get("serial") or e.get("name") or f"sensor{len(poses)}"
            poses.append((serial, read_pose(e)))
    return poses


# ---------------------------------------------------------------- comparison

def invariants(label, poses):
    """Compute + print the gauge-free invariants of a 2-sensor set."""
    (la, a), (lb, b) = poses
    rel = rel_transform(a, b)
    baseline = v_norm(v_sub(a.p, b.p))
    axis_angle = angle_between_deg(a.optical_axis(), b.optical_axis())

    print(f"\n--- {label} ---")
    print(f"  sensors: {la}  |  {lb}")
    print(f"  baseline distance      : {baseline * 1000:9.2f} mm")
    print(f"  relative rotation angle: {q_angle_deg(rel.q):9.3f} deg   "
          f"(quat xyzw: {', '.join(f'{c:+.6f}' for c in rel.q)})")
    print(f"  relative translation   : "
          f"({rel.p[0]*1000:+8.2f}, {rel.p[1]*1000:+8.2f}, {rel.p[2]*1000:+8.2f}) mm")
    print(f"  optical-axis angle     : {axis_angle:9.3f} deg")
    print(f"  heights (y)            : {a.p[1]:.4f} m , {b.p[1]:.4f} m")
    return rel, baseline, axis_angle


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("ovr_json", help="output of ovr_static_dump.exe")
    ap.add_argument("room_config", help="OpenHMD rift-room-config.json")
    ap.add_argument("--rot-gate-deg", type=float, default=0.5,
                    help="acceptance gate on relative-rotation delta (default 0.5)")
    ap.add_argument("--baseline-gate-mm", type=float, default=5.0,
                    help="acceptance gate on baseline delta (default 5.0)")
    args = ap.parse_args()

    ovr_poses, origin, ovr_raw = load_ovr(args.ovr_json)
    our_poses = load_room_config(args.room_config)

    if len(ovr_poses) != 2 or len(our_poses) != 2:
        raise SystemExit(f"Expected exactly 2 sensors on both sides, got "
                         f"{len(ovr_poses)} (ovr) / {len(our_poses)} (ours).")

    print(f"OVR runtime {ovr_raw.get('runtime_version', '?')}, "
          f"origin: {origin}, HMD: {ovr_raw.get('hmd_serial', '?')}")
    if origin != "FloorLevel":
        print("NOTE: OVR origin is not FloorLevel — height comparison is meaningless.")

    rel_ovr, base_ovr, ax_ovr = invariants("Oculus runtime", ovr_poses)
    rel_our, base_our, ax_our = invariants("Our room calibration", our_poses)

    # OVR tracker indices may map to our serials in either order; test both and
    # take the better match — the wrong pairing shows up as a huge rotation delta.
    rel_our_swapped, base_our_s, _ = (
        rel_transform(our_poses[1][1], our_poses[0][1]), base_our, None)

    def delta(rel_a, rel_b, base_a, base_b):
        dq = q_mul(q_conj(rel_a.q), rel_b.q)
        return q_angle_deg(dq), abs(base_a - base_b) * 1000.0

    d_rot, d_base = delta(rel_ovr, rel_our, base_ovr, base_our)
    d_rot_sw, d_base_sw = delta(rel_ovr, rel_our_swapped, base_ovr, base_our_s)
    swapped = d_rot_sw < d_rot
    if swapped:
        d_rot, d_base = d_rot_sw, d_base_sw
        print("\nNOTE: sensor order swapped for best match "
              "(OVR index order != room-config order).")

    print("\n=== DELTAS (gauge-free) ===")
    print(f"  relative-rotation delta: {d_rot:8.3f} deg   (gate: {args.rot_gate_deg})")
    print(f"  baseline delta         : {d_base:8.2f} mm    (gate: {args.baseline_gate_mm})")
    print(f"  optical-axis-angle diff: {abs(ax_ovr - ax_our):8.3f} deg")

    ok = d_rot <= args.rot_gate_deg and d_base <= args.baseline_gate_mm
    print()
    if ok:
        print("PASS — room calibration agrees with the Oculus runtime within gates.")
        print("       Proceed to Tier 1 (plan §3).")
    else:
        print("FAIL — sensor geometry disagrees beyond the gates.")
        print("       Per plan §7: STOP. Fix room calibration and re-baseline")
        print("       before comparing anything dynamic.")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
