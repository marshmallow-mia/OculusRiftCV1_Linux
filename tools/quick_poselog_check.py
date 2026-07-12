#!/usr/bin/env python3
"""Sanity-check a now/pred CSV pair from ovr_pose_log: sample rate, tracked
fraction, and the runtime's prediction horizon + its effect on the pose."""
import sys
import numpy as np

def load(p):
    d = np.genfromtxt(p, delimiter=",", names=True, invalid_raise=False)
    return d[~np.isnan(d["h_status"])]

now, pred = load(sys.argv[1]), load(sys.argv[2])
n = min(len(now), len(pred))
now, pred = now[:n], pred[:n]

dt = np.diff(now["t_wall"])
print(f"rows: {n}   median rate: {1/np.median(dt[dt>0]):.0f} Hz   "
      f"span: {now['t_wall'][-1]-now['t_wall'][0]:.1f} s")

st = now["h_status"].astype(int)
otrk = (st & 1) != 0
ptrk = (st & 2) != 0
print(f"orientation tracked: {otrk.mean()*100:.1f} %   "
      f"position tracked: {ptrk.mean()*100:.1f} %")

horizon = pred["t_ovr"] - now["t_ovr"]
print(f"prediction horizon: median {np.median(horizon)*1000:.2f} ms   "
      f"p5 {np.percentile(horizon,5)*1000:.2f}   p95 {np.percentile(horizon,95)*1000:.2f}")

dp = np.linalg.norm(np.stack([pred["hpx"]-now["hpx"], pred["hpy"]-now["hpy"],
                              pred["hpz"]-now["hpz"]], 1), axis=1)
dot = np.abs(pred["hqx"]*now["hqx"] + pred["hqy"]*now["hqy"]
             + pred["hqz"]*now["hqz"] + pred["hqw"]*now["hqw"])
dang = np.degrees(2*np.arccos(np.clip(dot, -1, 1)))
m = ptrk
if m.any():
    print(f"pred-now pose delta while tracked: pos median {np.median(dp[m])*1000:.2f} mm "
          f"p95 {np.percentile(dp[m],95)*1000:.2f} mm | "
          f"rot median {np.median(dang[m]):.3f} deg p95 {np.percentile(dang[m],95):.3f} deg")
w = np.linalg.norm(np.stack([now["hwx"], now["hwy"], now["hwz"]], 1), axis=1)
v = np.linalg.norm(np.stack([now["hvx"], now["hvy"], now["hvz"]], 1), axis=1)
print(f"motion seen: |w| p95 {np.percentile(w,95):.2f} rad/s   "
      f"|v| p95 {np.percentile(v,95):.2f} m/s")
