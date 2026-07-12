#!/usr/bin/env python3
"""Score OUR forward prediction, using the same methodology as
tools/analyze_prediction.py used on the Oculus runtime — so the numbers are
directly comparable to their 0.19 deg / 2.2 mm.

Input is one CSV from openhmd_pose_log (pose + velocities + accel + pose age at
high rate). Because the velocities are logged alongside each pose, we can
predict offline instead of needing a second live run:

  truth      : the pose actually observed at t+h, interpolated from the same log
  predicted  : the pose at t, dead-reckoned forward by h using the velocity and
               acceleration logged at t (the exact math rift_predict_pose does)
  zero pred  : the pose at t, used as-is

The gap between the last two is what forward prediction buys us.

  python3 score_prediction.py captures/lin/<date>/poselog.csv [--horizon-ms 22]
"""
import argparse

import numpy as np


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def quat_mul(a, b):
    ax, ay, az, aw = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    bx, by, bz, bw = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    return np.stack([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ], 1)


def normalize(q):
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def slerp_shortest(q0, q1, u):
    d = np.sum(q0 * q1, axis=1, keepdims=True)
    q1 = np.where(d < 0, -q1, q1)
    d = np.abs(d).clip(-1, 1)
    theta = np.arccos(d)
    s = np.sin(theta)
    small = s.squeeze() < 1e-6
    out = np.where(s < 1e-6, q0 * (1 - u) + q1 * u,
                   (np.sin((1 - u) * theta) * q0 + np.sin(u * theta) * q1) /
                   np.where(s < 1e-6, 1, s))
    out[small] = (q0 * (1 - u) + q1 * u)[small]
    return normalize(out)


def ang_deg(a, b):
    d = np.abs(np.sum(normalize(a) * normalize(b), axis=1)).clip(-1, 1)
    return np.degrees(2 * np.arccos(d))


def predict(P, Q, V, W, A, h):
    """rift_predict_pose, vectorised. Body-frame omega -> right-multiply."""
    wn = np.linalg.norm(W, axis=1, keepdims=True)
    half = 0.5 * wn * h
    axis = np.divide(W, np.where(wn < 1e-6, 1, wn))
    dq = np.concatenate([axis * np.sin(half), np.cos(half)], 1)
    dq[wn.squeeze() < 1e-6] = [0, 0, 0, 1]
    Qp = normalize(quat_mul(Q, dq))
    Pp = P + V * h + 0.5 * A * h * h
    return Pp, Qp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--horizon-ms", type=float, default=22.0,
                    help="photon latency to predict over (default 22, matching "
                         "the Oculus measurement)")
    a = ap.parse_args()
    h = a.horizon_ms / 1000.0

    d = load(a.csv)
    t = d["t"]
    n = len(t)
    P = np.stack([d["px"], d["py"], d["pz"]], 1)
    Q = np.stack([d["qx"], d["qy"], d["qz"], d["qw"]], 1)
    V = np.stack([d["vx"], d["vy"], d["vz"]], 1)
    W = np.stack([d["wx"], d["wy"], d["wz"]], 1)
    A = np.stack([d["ax"], d["ay"], d["az"]], 1)

    rate = (n - 1) / (t[-1] - t[0])
    print(f"rows {n}   span {t[-1] - t[0]:.1f} s   rate {rate:.0f} Hz   "
          f"horizon {a.horizon_ms:.0f} ms")
    print(f"pose age: median {np.median(d['age']) * 1000:.2f} ms   "
          f"p95 {np.percentile(d['age'], 95) * 1000:.2f} ms")

    # truth at t+h, interpolated from this same log
    tt = t + h
    idx = np.searchsorted(t, tt).clip(1, n - 1)
    t0, t1 = t[idx - 1], t[idx]
    u = ((tt - t0) / np.where(t1 == t0, 1, t1 - t0)).clip(0, 1)[:, None]
    Ptruth = P[idx - 1] * (1 - u) + P[idx] * u
    Qtruth = slerp_shortest(Q[idx - 1], Q[idx], u)

    Pp, Qp = predict(P, Q, V, W, A, h)

    tracked = np.linalg.norm(P, axis=1) > 1e-9   # all-zero pos = no optical lock
    valid = (tt <= t[-1]) & tracked
    moving = np.linalg.norm(W, axis=1) > 0.5     # rad/s, same cut as the Oculus scoring

    if not tracked.any():
        print("\n*** no optical lock in this log (position is all zeros) — "
              "the headset must FACE the sensors. Test invalid. ***")
        return

    for label, mask in [("all tracked", valid),
                        ("tracked & moving (|w|>0.5 rad/s)", valid & moving)]:
        if not mask.any():
            print(f"\n[{label}] no samples")
            continue
        ep_pred = np.linalg.norm(Pp[mask] - Ptruth[mask], axis=1) * 1000
        er_pred = ang_deg(Qp[mask], Qtruth[mask])
        ep_zero = np.linalg.norm(P[mask] - Ptruth[mask], axis=1) * 1000
        er_zero = ang_deg(Q[mask], Qtruth[mask])
        print(f"\n[{label}]  n={mask.sum()}")
        print(f"  our predictor   : pos median {np.median(ep_pred):6.2f} mm  "
              f"p95 {np.percentile(ep_pred, 95):6.2f} mm | "
              f"rot median {np.median(er_pred):6.3f} deg  "
              f"p95 {np.percentile(er_pred, 95):6.3f} deg")
        print(f"  zero prediction : pos median {np.median(ep_zero):6.2f} mm  "
              f"p95 {np.percentile(ep_zero, 95):6.2f} mm | "
              f"rot median {np.median(er_zero):6.3f} deg  "
              f"p95 {np.percentile(er_zero, 95):6.3f} deg")

    print("\nOculus runtime, same metric & horizon (captures/win/2026-07-12):")
    print("  their predictor : pos median   2.20 mm  p95   4.70 mm | "
          "rot median  0.190 deg  p95  0.510 deg")
    print("  zero prediction : pos median   6.00 mm  p95  15.30 mm | "
          "rot median  1.980 deg  p95  3.860 deg")


if __name__ == "__main__":
    main()
