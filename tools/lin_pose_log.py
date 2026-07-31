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

                     READ --predict-ms BEFORE TRUSTING A CAPTURE. The horizon
                     passed to getDeviceToAbsoluteTrackingPose decides WHICH
                     pose you get, and the two are not interchangeable:

                       --predict-ms 0   the pose SteamVR RECEIVED from us
                       --predict-ms 30  the pose SteamVR RENDERS with, after
                                        extrapolating over the velocity and
                                        acceleration our driver hands it

                     A capture at 0 says nothing about SteamVR's extrapolation,
                     because at horizon 0 there is none. That distinction cost a
                     whole evening: osc.csv was logged at 0, read 0.64 mm
                     residual with no dominant frequency, and was taken as
                     exonerating the entire stack — while the artifact the user
                     was reporting lives in a term that capture cannot contain.
                     Default stays 0 so older captures remain comparable.

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


def run_openvr(out_path, duration, predict_ms=(0.0,)):
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

    prev = {}  # (stream, device) -> (t, v, omega) for accel finite-diff

    def dev_fields(poses, i):
        if i is None or not poses[i].bDeviceIsConnected:
            return [0.0] * 7 + [0]
        p = poses[i]
        pos, q = mat34_to_pq(
            [list(p.mDeviceToAbsoluteTracking[r]) for r in range(3)])
        status = 0
        if p.bPoseIsValid:
            status |= FLAG_ORIENT | FLAG_POS
        return list(pos) + list(q) + [status]

    def hmd_fields(poses, stream, t_mono):
        p = poses[hmd_i]
        pos, q = mat34_to_pq(
            [list(p.mDeviceToAbsoluteTracking[r]) for r in range(3)])
        v = list(p.vVelocity.v)
        omega = list(p.vAngularVelocity.v)
        # accel by finite difference on velocity
        a = [0.0, 0.0, 0.0]
        al = [0.0, 0.0, 0.0]
        key = (stream, hmd_i)
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

    def sample(horizon_s, stream, t_mono, t_wall):
        poses = poses_t()
        vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, horizon_s, poses)
        # A predicted row is stamped with the time it was predicted FOR, not the
        # time it was taken — analyze_prediction.py recovers the horizon as
        # median(pred.t_ovr - now.t_ovr), so stamping both with the sample time
        # would collapse the horizon to zero and score the predictor against
        # itself. Same convention as tools/win/ovr_pose_log.c.
        t_stamp = t_mono + horizon_s
        row = [f"{t_stamp:.6f}", f"{t_wall + horizon_s:.6f}"]
        row += [f"{x:.9f}" if isinstance(x, float) else x
                for x in hmd_fields(poses, stream, t_stamp)]
        for ci in ctrl_i:
            row += [f"{x:.9f}" if isinstance(x, float) else x
                    for x in dev_fields(poses, ci)]
        return row

    # Every horizon is sampled in the SAME loop iteration. The human cannot
    # repeat a head turn twice, so separate runs would not be comparable;
    # sampling back to back makes the motion identical by construction and the
    # only difference between the files is how far SteamVR extrapolated. A
    # sweep rather than a single horizon turns the result from a point into a
    # curve: prediction error should fall then rise, and where it starts rising
    # is the horizon past which SteamVR is overshooting on our data.
    stem = out_path[:-4] if out_path.endswith(".csv") else out_path
    horizons = [h for h in predict_ms if h > 0.0]
    now_path = stem + "_now.csv" if horizons else out_path
    pred_paths = [f"{stem}_pred{h:g}.csv" for h in horizons]

    files = [open(now_path, "w", newline="")]
    files += [open(p, "w", newline="") for p in pred_paths]
    try:
        writers = [csv.writer(f) for f in files]
        for w in writers:
            w.writerow(HEADER)

        t0 = time.monotonic()
        n = 0
        while True:
            t_wall = time.time()
            t_mono = time.monotonic()
            if t_mono - t0 >= duration:
                break

            writers[0].writerow(sample(0.0, "now", t_mono, t_wall))
            for w, h in zip(writers[1:], horizons):
                w.writerow(sample(h / 1000.0, f"pred{h:g}", t_mono, t_wall))
            n += 1
            time.sleep(0.001)  # ~1 kHz ceiling; actual rate limited by runtime
    finally:
        for f in files:
            f.close()

    openvr.shutdown()
    print(f"wrote {n} rows to {now_path}", file=sys.stderr)
    for p, h in zip(pred_paths, horizons):
        print(f"  + {p}  (horizon {h:g} ms)", file=sys.stderr)
    for p in pred_paths:
        print(f"  score: venv/bin/python tools/analyze_prediction.py "
              f"{now_path} {p}", file=sys.stderr)


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
    ap.add_argument("--predict-ms", default="0",
                    help="(openvr backend) comma-separated prediction horizons "
                         "handed to getDeviceToAbsoluteTrackingPose, e.g. "
                         "'11,22,33'. Horizon 0 is always logged as <stem>_now."
                         " 0 alone logs the pose as SteamVR received it; a real "
                         "horizon logs the pose it renders with. See the note at "
                         "the top of this file.")
    args = ap.parse_args()

    if args.backend == "openvr":
        try:
            horizons = [float(x) for x in args.predict_ms.split(",") if x.strip()]
        except ValueError:
            sys.exit(f"--predict-ms: not a number list: {args.predict_ms!r}")
        run_openvr(args.out, args.duration, horizons)
    else:
        if not args.cmd:
            sys.exit("--backend openhmd requires --cmd <example binary> ...")
        run_openhmd(args.out, args.duration, args.cmd)


if __name__ == "__main__":
    main()
