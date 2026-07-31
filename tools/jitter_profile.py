#!/usr/bin/env python3
"""How much does the pose move while the headset is physically stationary?

A headset sitting on a desk has exactly zero true motion, so every millimetre
in the reported pose is artifact. That makes this the cleanest measurement in
the toolkit: no stop detection, no smoothing window to choose, no reference
trajectory to estimate, nothing that can quietly exclude the band the symptom
lives in — which is how two earlier metrics here missed a real effect
(find_oscillation.py filters anything slower than its 0.25 s window;
settling_metric.py stops at a 0.03 m/s threshold).

Run against the paired output of

  lin_pose_log.py --out jitter.csv --predict-ms 11,22,33

with the headset left alone on the desk. The _now stream is what SteamVR was
handed; each _predN is what it extrapolates to at that horizon. Because the true
motion is zero, the growth from _now to _predN is precisely what the
extrapolation adds, isolated from tracking quality.

The spectrum is the diagnostic that matters, because each source has its own
line and they need different fixes:

  ~52 Hz      camera exposure rate — vision reaching the output raw
  broadband   IMU noise amplified by extrapolation (tune velocity smoothing)
  ~1 Hz       the position observer's own pole pair
  ~90 Hz      display/compositor beat

  tools/jitter_profile.py captures/lin/<date>/jitter
"""
import argparse
import glob
import os

import numpy as np


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True, invalid_raise=False)
    d = d[~np.isnan(d["h_status"])]
    t = d["t_wall"] if "t_wall" in d.dtype.names else d["t_ovr"]
    P = np.c_[d["hpx"], d["hpy"], d["hpz"]]
    Q = np.c_[d["hqx"], d["hqy"], d["hqz"], d["hqw"]]
    t = t - t[0]
    k = np.r_[True, np.diff(t) > 0]
    return t[k], P[k], Q[k]


def report(label, t, P, Q):
    dev = np.linalg.norm(P - P.mean(axis=0), axis=1) * 1000.0
    qm = Q.mean(axis=0)
    qm /= np.linalg.norm(qm)
    ang = np.degrees(2 * np.arccos(np.abs(Q @ qm).clip(-1, 1)))
    print("  %-8s pos rms %6.3f mm  p95 %6.3f  p2p %7.3f   |   "
          "rot rms %6.4f deg  p95 %6.4f"
          % (label, dev.std(), np.percentile(dev, 95), dev.max() - dev.min(),
             ang.std(), np.percentile(ang, 95)))
    return dev


def spectrum(t, dev, rate):
    g = np.arange(t[0], t[-1], 1.0 / rate)
    r = np.interp(g, t, dev)
    r = r - r.mean()
    if len(r) < 64:
        return []
    F = np.abs(np.fft.rfft(r * np.hanning(len(r))))
    f = np.fft.rfftfreq(len(r), 1.0 / rate)
    keep = (f > 0.3) & (f < min(120.0, rate / 2))
    f, F = f[keep], F[keep]
    out, seen = [], []
    for i in np.argsort(F)[::-1]:
        if any(abs(f[i] - s) < max(0.5, 0.08 * s) for s in seen):
            continue
        seen.append(f[i])
        out.append((f[i], 100.0 * F[i] / F.sum()))
        if len(out) == 5:
            break
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stem", help="capture stem, e.g. captures/.../jitter")
    a = ap.parse_args()

    stem = a.stem[:-4] if a.stem.endswith(".csv") else a.stem
    files = [("now", stem + "_now.csv")]
    for p in sorted(glob.glob(stem + "_pred*.csv")):
        files.append((os.path.basename(p).split("_pred")[1][:-4] + "ms", p))
    files = [(k, v) for k, v in files if os.path.exists(v)]
    if not files:
        raise SystemExit("no capture found at %s_now.csv" % stem)

    print("motion of a stationary headset (all of it is artifact)\n")
    devs = {}
    for label, path in files:
        t, P, Q = load(path)
        rate = 1.0 / np.median(np.diff(t))
        devs[label] = (t, report(label, t, P, Q), rate)

    base = devs["now"][1].std()
    print("\n  extrapolation's own contribution, over the raw pose:")
    for label, (t, d, _) in devs.items():
        if label == "now":
            continue
        print("    %-8s %+.3f mm rms  (x%.2f)" % (label, d.std() - base,
                                                  d.std() / max(base, 1e-9)))

    # The headline number. Deviation WITHIN a short window separates fast
    # jitter from slow wander: a pose that drifts a centimetre over a minute is
    # a different defect from one that jumps 6 mm in 50 ms, and only the second
    # is felt as vibration. Measured 2026-07-31 on a desk-stationary headset:
    # ours 6.501 mm median, the Oculus runtime 0.042 mm on its quiet segments.
    t, P, _ = load(files[0][1])
    W = 0.05
    ex = []
    for s in np.arange(0, t[-1] - W, W):
        m = (t >= s) & (t < s + W)
        if m.sum() < 10:
            continue
        Q = P[m]
        ex.append(np.linalg.norm(Q - Q.mean(axis=0), axis=1).max() * 1000)
    ex = np.array(ex)
    if len(ex):
        print("\n  within-%.0f ms excursion (the vibration metric): "
              "median %.3f mm  p95 %.3f  max %.2f"
              % (W * 1000, np.median(ex), np.percentile(ex, 95), ex.max()))
        print("    Oculus runtime reference on quiet segments: 0.042 mm median")

    print("\n  strongest components (of the longest-horizon stream):")
    label = files[-1][0]
    t, d, rate = devs[label]
    for fr, share in spectrum(t, d, rate):
        note = ""
        for lo, hi, what in ((0.5, 1.6, "the position observer's pole pair"),
                             (45, 60, "camera exposure rate - vision entering raw"),
                             (85, 95, "display/compositor rate")):
            if lo <= fr <= hi:
                note = "  <- " + what
        print("    %7.2f Hz  %5.1f%%%s" % (fr, share, note))
    print("\n  (no dominant line and power spread broadly = IMU noise amplified"
          " by\n   extrapolation, which is what velocity smoothing addresses)")


if __name__ == "__main__":
    main()
