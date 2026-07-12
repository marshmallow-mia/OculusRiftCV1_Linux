#!/usr/bin/env python3
"""
tools/lin_pose_log.py — Tier-1 pose logger for OUR stack (Linux side).

Writes the exact same CSV schema as tools/win/ovr_pose_log.exe so that
compare_dynamic.py can eat both without special-casing:

  t_ovr,t_wall,
  hpx,hpy,hpz,hqx,hqy,hqz,hqw,
  hvx,hvy,hvz,hwx,hwy,hwz,
  hax,hay,haz,halx,haly,halz,
  h_status,
  c0px,c0py,c0pz,c0qx,c0qy,c0qz,c0qw,c0status,
  c1px,c1py,c1pz,c1qx,c1qy,c1qz,c1qw,c1status

(t_ovr is the stack's own clock; on Linux we use time.monotonic().)

Two backends:

  --backend openvr   (default) Read via pyopenvr, so we log the pose our
                     SteamVR driver actually exports — including the
                     correction-bleed offset. This is the apples-to-apples
                     comparison point against the Oculus runtime output.

  --backend openhmd  Parse the stdout of openhmd_simple_example, like
                     riftcv1/pose_test.py does. Catches the pose BEFORE the
                     SteamVR driver layer. Velocities are finite-differenced
                     (marked in the h_status column, see below).

h_status bitmask (mirrors the spirit of ovrStatusFlags):
  bit0 = orientation tracked
  bit1 = position tracked
  bit15 = velocities are finite-differenced by this script, not device-native

Usage:
  python3 lin_pose_log.py --out pose_now.csv --duration 60 [--backend openvr]
  python3 lin_pose_log.py --backend openhmd --cmd path/to/openhmd_simple_example ...
"""

import argparse
import csv
import math
import subprocess
import sys
import time

HEADER = (
    "t_ovr,t_wall,"
    "hpx,hpy,hpz,hqx,hqy,hqz,hqw,"
    "hvx,hvy,hvz,hwx,hwy,hwz,"
    "hax,hay,haz,halx,haly,halz,"
    "h_status,"
    "c0px,c0py,c0pz,c0qx,c0qy,c0qz,c0qw,c0status,"
    "c1px,c1py,c1pz,c1qx,c1qy,c1qz,c1qw,c1status"
).split(",")

FLAG_ORIENT = 1 << 0
FLAG_POS    = 1 << 1
FLAG_FDIFF  = 1 << 15  # velocities finite-differenced by this script


def mat34_to_pq(m):
    """OpenVR HmdMatrix34 (row-major 3x4) -> (pos, quat xyzw)."""
    px, py, pz = m[0][3], m[1][3], m[2][3]
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        qw = 0.25 * s
        qx = (m[2][1] - m[1][2]) / s
        qy = (m[0][2] - m[2][0]) / s
        qz = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        qw = (m[2][1] - m[1][2]) / s
        qx = 0.25 * s
        qy = (m[0][1] + m[1][0]) / s
        qz = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        qw = (m[0][2] - m[2][0]) / s
        qx = (m[0][1] + m[1][0]) / s
        qy = 0.25 * s
        qz = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        qw = (m[1][0] - m[0][1]) / s
        qx = (m[0][2] + m[2][0]) / s
        qy = (m[1][2] + m[2][1]) / s
        qz = 0.25 * s
    return (px, py, pz), (qx, qy, qz, qw)


def run_openvr(out_path, duration):
    try:
        import openvr
    except ImportError:
        sys.exit("pyopenvr not installed: pip install openvr")

    vr = openvr.init(openvr.VRApplication_Background)
    poses_t = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount

    # find HMD + up to two controllers
    def classify():
        hmd, ctrls = None, []
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            c = vr.getTrackedDeviceClass(i)
            if c == openvr.TrackedDeviceClass_HMD:
                hmd = i
            elif c == openvr.TrackedDeviceClass_Controller:
                ctrls.append(i)
        return hmd, (ctrls + [None, None])[:2]

    hmd_i, ctrl_i = classify()
    if hmd_i is None:
        sys.exit("No HMD visible to OpenVR — is SteamVR running with our driver?")

    prev = {}  # device -> (t, v) for accel finite-diff

    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(HEADER)
        t0 = time.monotonic()
        n = 0
        while True:
            t_wall = time.time()
            t_mono = time.monotonic()
            if t_mono - t0 >= duration:
                break

            poses = poses_t()
            vr.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0.0, poses)

            def dev_fields(i):
                if i is None or not poses[i].bDeviceIsConnected:
                    return [0.0] * 7 + [0]
                p = poses[i]
                pos, q = mat34_to_pq(
                    [list(p.mDeviceToAbsoluteTracking[r]) for r in range(3)])
                status = 0
                if p.bPoseIsValid:
                    status |= FLAG_ORIENT | FLAG_POS
                return list(pos) + list(q) + [status]

            def hmd_fields():
                p = poses[hmd_i]
                pos, q = mat34_to_pq(
                    [list(p.mDeviceToAbsoluteTracking[r]) for r in range(3)])
                v = list(p.vVelocity.v)
                omega = list(p.vAngularVelocity.v)
                # accel by finite difference on velocity
                a = [0.0, 0.0, 0.0]
                al = [0.0, 0.0, 0.0]
                key = ("hmd", hmd_i)
                if key in prev:
                    pt, pv, pw = prev[key]
                    dt = t_mono - pt
                    if dt > 0:
                        a = [(v[k] - pv[k]) / dt for k in range(3)]
                        al = [(omega[k] - pw[k]) / dt for k in range(3)]
                prev[key] = (t_mono, v, omega)
                status = FLAG_FDIFF
                if p.bPoseIsValid:
                    status |= FLAG_ORIENT | FLAG_POS
                return list(pos) + list(q) + v + omega + a + al + [status]

            row = [f"{t_mono:.6f}", f"{t_wall:.6f}"]
            row += [f"{x:.9f}" if isinstance(x, float) else x
                    for x in hmd_fields()]
            for ci in ctrl_i:
                row += [f"{x:.9f}" if isinstance(x, float) else x
                        for x in dev_fields(ci)]
            w.writerow(row)
            n += 1
            time.sleep(0.001)  # ~1 kHz ceiling; actual rate limited by runtime

    openvr.shutdown()
    print(f"wrote {n} rows to {out_path}", file=sys.stderr)


def run_openhmd(out_path, duration, cmd):
    """Parse `openhmd_simple_example`-style stdout lines.

    Expected line shape (same as riftcv1/pose_test.py parses):
      position/quaternion floats somewhere on the line; we take the first 7
      floats as px py pz qx qy qz qw. Adjust FLOAT_COUNT/slicing if the tool's
      output differs on this checkout.
    """
    import re
    float_re = re.compile(r"[-+]?\d*\.\d+(?:[eE][-+]?\d+)?")

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True, bufsize=1)
    prev = None  # (t, pos, q, v, w) for finite differences

    def qdiff_omega(q0, q1, dt):
        # small-angle approx: omega = 2 * (q1 * conj(q0)).xyz / dt
        x0, y0, z0, w0 = q0
        x1, y1, z1, w1 = q1
        # q1 * conj(q0)
        dx = w1 * -x0 + x1 * w0 + y1 * -z0 - z1 * -y0
        dy = w1 * -y0 - x1 * -z0 + y1 * w0 + z1 * -x0
        dz = w1 * -z0 + x1 * -y0 - y1 * -x0 + z1 * w0
        return (2 * dx / dt, 2 * dy / dt, 2 * dz / dt)

    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(HEADER)
        t0 = time.monotonic()
        n = 0
        try:
            for line in proc.stdout:
                t_mono = time.monotonic()
                if t_mono - t0 >= duration:
                    break
                floats = [float(x) for x in float_re.findall(line)]
                if len(floats) < 7:
                    continue
                pos, q = floats[0:3], floats[3:7]

                v = om = a = al = [0.0, 0.0, 0.0]
                if prev is not None:
                    pt, ppos, pq, pv, pom = prev
                    dt = t_mono - pt
                    if dt > 0:
                        v = [(pos[k] - ppos[k]) / dt for k in range(3)]
                        om = list(qdiff_omega(pq, q, dt))
                        a = [(v[k] - pv[k]) / dt for k in range(3)]
                        al = [(om[k] - pom[k]) / dt for k in range(3)]
                prev = (t_mono, pos, q, v, om)

                status = FLAG_ORIENT | FLAG_POS | FLAG_FDIFF
                row = ([f"{t_mono:.6f}", f"{time.time():.6f}"]
                       + [f"{x:.9f}" for x in pos + q + v + om + a + al]
                       + [status]
                       + ["0.0"] * 7 + [0]     # c0: not available on this backend
                       + ["0.0"] * 7 + [0])    # c1
                w.writerow(row)
                n += 1
        finally:
            proc.terminate()
    print(f"wrote {n} rows to {out_path}", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(description="Tier-1 pose logger, our stack")
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--backend", choices=["openvr", "openhmd"], default="openvr")
    ap.add_argument("--cmd", nargs=argparse.REMAINDER,
                    help="(openhmd backend) command to run, e.g. "
                         "--cmd ./openhmd_simple_example")
    args = ap.parse_args()

    if args.backend == "openvr":
        run_openvr(args.out, args.duration)
    else:
        if not args.cmd:
            sys.exit("--backend openhmd requires --cmd <example binary> ...")
        run_openhmd(args.out, args.duration, args.cmd)


if __name__ == "__main__":
    main()
