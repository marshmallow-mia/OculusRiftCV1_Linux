#!/usr/bin/env python3
"""Find the overshoot in a pose log, and let its frequency name the source.

Eight fixes have now been applied to individual stages of the pipeline, every
one of them measured correct afterwards, and the reported symptom - "I move and
it overshoots, gets back too far, etc until it reached the real point" - has not
moved. That pattern says the guessing is the problem: stages were chosen by
plausibility and then verified in isolation, while the actual OUTPUT trajectory
was never examined.

An oscillation has a frequency, and in this system every candidate source has a
different one. So measure it and read the answer off:

    ~1.1 Hz   the position observer's complex pole pair (zeta 0.71, computed
              from GAIN_POS/GAIN_VEL/GAIN_ACCEL = 10/50/25)
    ~0.6 Hz   the accel-bias mode (tau 1.44 s, measured)
    ~52 Hz    camera exposure rate (19.2 ms nominal)
    ~90 Hz    display frame rate / compositor
    ~500 Hz   IMU sample rate
    broadband, no peak
              not an oscillation at all - a nonlinearity, a scale error, or
              something outside the pose entirely

Method: fit and remove the smooth motion (Savitzky-Golay-style local polynomial
via a moving least-squares window), then look at the residual - its spectrum,
and its behaviour after motion stops, where a ringing mode shows as a decaying
sinusoid rather than noise.

  tools/find_oscillation.py captures/lin/<date>/<tag>.csv
"""
import argparse

import numpy as np


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True, invalid_raise=False)
    if "h_status" in (d.dtype.names or ()):
        d = d[~np.isnan(d["h_status"])]
        st = d["h_status"].astype(int)
        d = d[(st & 2) != 0]                     # position tracked
    return d


def smooth(y, t, win_s):
    """Local quadratic fit — follows real motion, leaves oscillation behind."""
    out = np.empty_like(y)
    n = len(y)
    for i in range(n):
        lo = np.searchsorted(t, t[i] - win_s / 2)
        hi = np.searchsorted(t, t[i] + win_s / 2)
        lo, hi = max(0, lo), min(n, max(hi, lo + 3))
        tt = t[lo:hi] - t[i]
        try:
            c = np.polyfit(tt, y[lo:hi], 2)
        except Exception:
            out[i] = y[i]
            continue
        out[i] = c[-1]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--window", type=float, default=0.25,
                    help="smoothing window in seconds; the oscillation must be "
                         "faster than this to survive it")
    a = ap.parse_args()

    d = load(a.csv)
    t = d["t_wall"] if "t_wall" in d.dtype.names else d["t_ovr"]
    t = t - t[0]
    P = np.c_[d["hpx"], d["hpy"], d["hpz"]]
    rate = 1.0 / np.median(np.diff(t))
    print("%s\n  %d samples, %.1f s, %.0f Hz" % (a.csv, len(d), t[-1], rate))

    speed = np.r_[0, np.linalg.norm(np.diff(P, axis=0), axis=1) / np.maximum(np.diff(t), 1e-9)]
    print("  speed: median %.3f  p90 %.3f  max %.3f m/s" %
          (np.median(speed), np.percentile(speed, 90), speed.max()))

    resid = np.stack([P[:, i] - smooth(P[:, i], t, a.window) for i in range(3)], 1)
    rmag = np.linalg.norm(resid, axis=1) * 1000
    moving = speed > 0.15
    print("\n=== residual after removing smooth motion ===")
    print("  overall  median %6.2f mm  p90 %7.2f  max %8.1f" %
          (np.median(rmag), np.percentile(rmag, 90), rmag.max()))
    if moving.sum() > 50:
        print("  moving   median %6.2f mm  p90 %7.2f" %
              (np.median(rmag[moving]), np.percentile(rmag[moving], 90)))
        print("  still    median %6.2f mm  p90 %7.2f" %
              (np.median(rmag[~moving]), np.percentile(rmag[~moving], 90)))

    # Spectrum of the residual on a uniform grid.
    g = np.arange(t[0], t[-1], 1.0 / rate)
    R = np.stack([np.interp(g, t, resid[:, i]) for i in range(3)], 1)
    R = R - R.mean(0)
    win = np.hanning(len(R))[:, None]
    F = np.abs(np.fft.rfft(R * win, axis=0)).sum(1)
    f = np.fft.rfftfreq(len(R), 1.0 / rate)
    keep = (f > 0.2) & (f < min(120.0, rate / 2))
    f, F = f[keep], F[keep]
    if len(f) < 8:
        print("\n  too little data for a spectrum")
        return

    print("\n=== residual spectrum: strongest components ===")
    order = np.argsort(F)[::-1]
    seen = []
    for i in order:
        if any(abs(f[i] - s) < max(0.3, 0.1 * s) for s in seen):
            continue
        seen.append(f[i])
        share = F[i] / F.sum() * 100
        print("   %7.2f Hz   %5.1f%% of residual power" % (f[i], share))
        if len(seen) == 6:
            break

    top = seen[0]
    print("\n=== reading ===")
    for lo, hi, what in ((0.4, 0.9, "the accelerometer-bias mode (tau 1.44 s)"),
                         (0.9, 1.6, "the position observer's pole pair (1.07 Hz, zeta 0.71)"),
                         (45, 60, "the camera exposure rate — vision entering the output raw"),
                         (80, 100, "the display/compositor frame rate"),
                         (400, 600, "the IMU sample rate")):
        if lo <= top <= hi:
            print("   dominant %.2f Hz matches %s" % (top, what))
            break
    else:
        print("   dominant %.2f Hz matches no known stage — if the power is also"
              % top)
        print("   spread broadly, this is not a ringing mode at all.")


if __name__ == "__main__":
    main()
