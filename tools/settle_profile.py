#!/usr/bin/env python3
"""Where does the pose go in the seconds AFTER the head stops?

This exists because the two metrics already here cannot see the thing the whole
investigation is chasing, and both were used to declare the tracking clean:

  find_oscillation.py   subtracts a locally-fitted quadratic over a 0.25 s
                        window, so anything settling more slowly is absorbed
                        into "real motion" and never reaches the residual. The
                        position observer's slowest pole is tau = 1.79 s.
  settling_metric.py    stops measuring once speed falls under STILL_LIN =
                        0.030 m/s. A 10 mm settle spread over 1.8 s averages
                        6 mm/s, so it reads as "already still" and the coast
                        comes back near zero.

Both answer "is there a fast wiggle?". Neither answers "does it creep to its
final value over a second or two?", which is what "I move and it overshoots,
gets back too far, until it reached the real point" describes.

Method, deliberately free of speed thresholds after the stop is found:

  1. find decelerations - speed above FAST, then falling below SLOW
  2. take t_stop at that crossing
  3. require the next `--window` seconds to stay slow, so a second movement
     cannot be mistaken for a settle
  4. take the FINAL position as the mean over the last 20% of that window
  5. report |p(t) - p_final| at a series of lags, and fit an exponential
     for amplitude and tau

The final position is measured, not assumed, so this reports convergence rather
than absolute accuracy - drift toward a wrong place and drift toward the right
one are both settles.

Run it against the Oculus runtime capture to get a target rather than a guess:

  tools/settle_profile.py captures/win/2026-07-12/pose_imu_now.csv
  tools/settle_profile.py captures/lin/2026-07-31/predold2_now.csv
"""
import argparse

import numpy as np

FAST = 0.50      # m/s - a real movement, not a twitch
SLOW = 0.10      # m/s - the stop threshold
LAGS = (0.0, 0.1, 0.25, 0.5, 1.0, 1.5, 2.0)


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True, invalid_raise=False)
    if "h_status" in (d.dtype.names or ()):
        d = d[~np.isnan(d["h_status"])]
        st = d["h_status"].astype(int)
        d = d[(st & 2) != 0]                  # position tracked
    t = d["t_wall"] if "t_wall" in d.dtype.names else d["t_ovr"]
    P = np.c_[d["hpx"], d["hpy"], d["hpz"]]
    t = t - t[0]
    keep = np.r_[True, np.diff(t) > 0]        # strictly increasing
    return t[keep], P[keep]


def find_stops(t, P, window):
    # Speed over a FIXED time base, not between adjacent samples. Differencing
    # neighbours makes the estimate depend on the log rate: at 810 Hz, half a
    # millimetre of position noise across a 1.2 ms gap reads as 0.4 m/s, so
    # every genuine stop looks like continued motion and no settle is ever
    # found. A 50 ms base is short against real head motion and long enough to
    # bury the noise, and it makes captures at different rates comparable.
    BASE_S = 0.05
    j = np.searchsorted(t, t + BASE_S).clip(0, len(t) - 1)
    dt = np.maximum(t[j] - t, 1e-9)
    vs = np.linalg.norm(P[j] - P, axis=1) / dt
    vs[j == len(t) - 1] = 0.0          # no forward base left at the tail

    # Find the quiet runs first, then ask which were preceded by a movement.
    # Scanning forward for "fast, then slow" instead loses almost every event:
    # during a settle the speed dithers across SLOW many times, and the first
    # dip that fails the window test discards the deceleration that produced
    # it. Working from the runs is order-independent and needs no hysteresis.
    quiet = vs < SLOW
    stops = []
    i = 0
    n = len(t)
    while i < n:
        if not quiet[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and quiet[j + 1]:
            j += 1
        if t[j] - t[i] >= window:
            # genuine deceleration into it, not a pause inside stillness
            pre = (t >= t[i] - 1.0) & (t < t[i])
            if pre.any() and vs[pre].max() > FAST:
                stops.append(i)
        i = j + 1
    return stops, vs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--window", type=float, default=3.0,
                    help="seconds of quiet required after the stop")
    a = ap.parse_args()

    t, P = load(a.csv)
    rate = 1.0 / np.median(np.diff(t))
    stops, vs = find_stops(t, P, a.window)
    print("%s\n  %d samples, %.0f s, %.0f Hz  |  %d clean stops with %.1f s quiet"
          % (a.csv, len(t), t[-1], rate, len(stops), a.window))
    if not stops:
        print("  no qualifying stop events - try a shorter --window")
        return

    disp, taus, amps = [], [], []
    for i in stops:
        t0 = t[i]
        seg = (t >= t0) & (t <= t0 + a.window)
        ts, Ps = t[seg] - t0, P[seg]
        tail = ts >= 0.8 * a.window
        if tail.sum() < 5:
            continue
        p_final = Ps[tail].mean(axis=0)
        r = np.linalg.norm(Ps - p_final, axis=1)
        disp.append([np.interp(l, ts, r) for l in LAGS])

        # exponential fit on the decaying part, r(t) = A*exp(-t/tau)
        m = (r > 1e-5) & (ts < 0.8 * a.window)
        if m.sum() > 20:
            c = np.polyfit(ts[m], np.log(r[m]), 1)
            if c[0] < 0:
                taus.append(-1.0 / c[0])
                amps.append(np.exp(c[1]))

    D = np.array(disp) * 1000.0
    print("\n  distance still to travel, measured from the stop (mm):")
    print("    %-8s %8s %8s %8s" % ("lag", "median", "p90", "max"))
    for j, l in enumerate(LAGS):
        print("    %-8s %8.2f %8.2f %8.2f"
              % ("%.2f s" % l, np.median(D[:, j]), np.percentile(D[:, j], 90),
                 D[:, j].max()))

    if taus:
        print("\n  exponential fit over %d events:" % len(taus))
        print("    amplitude  median %6.2f mm" % (np.median(amps) * 1000))
        print("    tau        median %6.2f s   p90 %.2f s"
              % (np.median(taus), np.percentile(taus, 90)))
        print("\n  (the position observer's slowest pole is tau = 1.79 s;"
              " a fit near that")
        print("   value is the bias mode, one near 0.2 s is the fast pair)")


if __name__ == "__main__":
    main()
