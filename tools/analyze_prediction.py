#!/usr/bin/env python3
"""
tools/analyze_prediction.py — score the Oculus predictor (plan §6).

Input: a now/pred CSV pair from ovr_pose_log.exe run with an explicit
prediction horizon (pred file row i = pose predicted for now-row-i's
t_ovr + horizon).

For every sample time t we interpolate the *actually observed* pose at
t + horizon from the now log, then compare two candidates against it:

  predictor error : their pose predicted for t+h, vs truth at t+h
  zero-pred error : their pose AT t (no prediction — what our driver ships,
                    since poseTimeOffset is never set), vs truth at t+h

The gap between those two errors is what forward prediction buys, measured
on their own tracking output.

Usage:
  python analyze_prediction.py pose_pred22_now.csv pose_pred22_pred.csv
"""

import sys
import numpy as np

def load(p):
    d = np.genfromtxt(p, delimiter=",", names=True, invalid_raise=False)
    return d[~np.isnan(d["h_status"])]

def slerp_shortest(q0, q1, u):
    """Row-wise quaternion slerp, u in [0,1], shape (n,4) xyzw."""
    dot = np.sum(q0 * q1, axis=1, keepdims=True)
    q1 = np.where(dot < 0, -q1, q1)
    dot = np.abs(dot).clip(-1, 1)
    th = np.arccos(dot)
    s = np.sin(th)
    small = s[:, 0] < 1e-6
    w0 = np.where(small[:, None], 1 - u, np.sin((1 - u) * th) / np.where(s == 0, 1, s))
    w1 = np.where(small[:, None], u,     np.sin(u * th)       / np.where(s == 0, 1, s))
    out = w0 * q0 + w1 * q1
    return out / np.linalg.norm(out, axis=1, keepdims=True)

def ang_deg(qa, qb):
    dot = np.abs(np.sum(qa * qb, axis=1)).clip(-1, 1)
    return np.degrees(2 * np.arccos(dot))

def main():
    now, pred = load(sys.argv[1]), load(sys.argv[2])
    n = min(len(now), len(pred))
    now, pred = now[:n], pred[:n]

    t = now["t_ovr"]
    horizon = np.median(pred["t_ovr"] - now["t_ovr"])
    print(f"rows: {n}   horizon: {horizon*1000:.1f} ms")

    P = np.stack([now["hpx"], now["hpy"], now["hpz"]], 1)
    Q = np.stack([now["hqx"], now["hqy"], now["hqz"], now["hqw"]], 1)
    Pp = np.stack([pred["hpx"], pred["hpy"], pred["hpz"]], 1)
    Qp = np.stack([pred["hqx"], pred["hqy"], pred["hqz"], pred["hqw"]], 1)

    # truth at t+h: interpolate the now log
    tt = t + horizon
    idx = np.searchsorted(t, tt).clip(1, n - 1)
    t0, t1 = t[idx - 1], t[idx]
    u = ((tt - t0) / np.where(t1 == t0, 1, t1 - t0)).clip(0, 1)[:, None]
    Ptruth = P[idx - 1] * (1 - u) + P[idx] * u
    Qtruth = slerp_shortest(Q[idx - 1], Q[idx], u)

    valid = (tt <= t[-1]) & ((now["h_status"].astype(int) & 2) != 0)
    # only score while actually moving — prediction is trivial at rest
    w = np.linalg.norm(np.stack([now["hwx"], now["hwy"], now["hwz"]], 1), axis=1)
    moving = w > 0.5  # rad/s

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
        print(f"  their predictor : pos median {np.median(ep_pred):6.2f} mm  "
              f"p95 {np.percentile(ep_pred,95):6.2f} mm | "
              f"rot median {np.median(er_pred):6.3f} deg  "
              f"p95 {np.percentile(er_pred,95):6.3f} deg")
        print(f"  zero prediction : pos median {np.median(ep_zero):6.2f} mm  "
              f"p95 {np.percentile(ep_zero,95):6.2f} mm | "
              f"rot median {np.median(er_zero):6.3f} deg  "
              f"p95 {np.percentile(er_zero,95):6.3f} deg")

if __name__ == "__main__":
    main()
