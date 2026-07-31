#!/usr/bin/env python3
"""How far does the pose keep travelling after the head stops?

That is the complaint this whole investigation is chasing — "fast movement and
the image still takes a bit to stop moving" — so it needs to be a number rather
than a feeling, and it needs a target. This measures it on any log in the
ovr_pose_log / lin_pose_log schema, so the Oculus runtime's own capture supplies
the target and our stack is judged against it.

Method: find deceleration-to-rest events (speed above FAST falling below SLOW),
then from the moment speed crosses SLOW, measure how much further the pose
travels before it is genuinely still. That "coast" is exactly what a wearer
perceives as the image not stopping when they do.

Reported separately for translation and rotation, because they are predicted by
different terms and only rotation lacks a second-order one.

  tools/settling_metric.py captures/win/2026-07-12/pose_imu_now.csv
"""
import argparse
import numpy as np

# Thresholds are set from the measured distribution of a real session rather
# than guessed: on the Oculus capture, linear speed runs a 0.22 m/s median with
# a 0.68 p90, and angular 60 deg/s median with 152 p90. "Still" has to be a bar
# the signal actually reaches (24% of samples sit under 0.05 m/s) or the search
# runs straight past the stop it is looking for.
FAST_LIN, SLOW_LIN, STILL_LIN = 0.50, 0.10, 0.030     # m/s
FAST_ANG, SLOW_ANG, STILL_ANG = 2.00, 0.50, 0.150     # rad/s

# The quiet has to hold, or a single noise dip reads as a stop.
QUIET_HOLD_S = 0.050


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True, invalid_raise=False)
    if "h_status" in (d.dtype.names or ()):
        d = d[~np.isnan(d["h_status"])]
        st = d["h_status"].astype(int)
        d = d[(st & 2) != 0]          # position tracked
    return d


def coast_events(t, speed, pos, fast, slow, still, max_coast=1.0, lookback=1.5):
    """Each event: (time from crossing `slow` until the pose is genuinely still,
    distance travelled during it).

    Found by locating SUSTAINED quiet first and then looking backwards for the
    deceleration that produced it, which is far more robust than scanning
    forwards for a stop that a noise dip can fake.
    """
    n = len(t)
    quiet = speed < still

    # starts of quiet runs lasting at least QUIET_HOLD_S
    starts = []
    i = 0
    while i < n:
        if not quiet[i]:
            i += 1
            continue
        j = i
        while j < n and quiet[j]:
            j += 1
        if t[j - 1] - t[i] >= QUIET_HOLD_S:
            starts.append(i)
        i = j

    out = []
    for k in starts:
        # back up to the moment speed last fell below `slow`
        j = k
        while j > 0 and speed[j - 1] < slow:
            j -= 1
        if j == 0 or t[k] - t[j] > max_coast:
            continue
        # and require a genuine fast motion shortly before that
        m = j
        while m > 0 and t[j] - t[m] < lookback and speed[m] < fast:
            m -= 1
        if m <= 0 or speed[m] < fast:
            continue
        dist = float(np.sum(np.linalg.norm(np.diff(pos[j:k + 1], axis=0), axis=1))) \
            if k > j else 0.0
        out.append((t[k] - t[j], dist))
    return out


def report(label, ev, unit, scale):
    if not ev:
        print(f"  {label:<12s} no deceleration-to-rest events found")
        return
    dur = np.array([e[0] for e in ev]) * 1000.0
    dist = np.array([e[1] for e in ev]) * scale
    print(f"  {label:<12s} {len(ev):3d} events | coast time  median {np.median(dur):6.0f} ms  "
          f"p90 {np.percentile(dur, 90):6.0f}  max {dur.max():6.0f}")
    print(f"  {'':<12s}     | coast {'dist':<5s}  median {np.median(dist):6.2f} {unit}  "
          f"p90 {np.percentile(dist, 90):6.2f}  max {dist.max():6.2f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    a = ap.parse_args()

    d = load(a.csv)
    t = d["t_ovr"] if "t_ovr" in d.dtype.names else d["t_wall"]
    pos = np.c_[d["hpx"], d["hpy"], d["hpz"]]
    vel = np.c_[d["hvx"], d["hvy"], d["hvz"]]
    ang = np.c_[d["hwx"], d["hwy"], d["hwz"]]

    # rotation "distance" = integrated angle, so treat the quat as a path
    q = np.c_[d["hqx"], d["hqy"], d["hqz"], d["hqw"]]

    print(f"{a.csv}")
    print(f"  {len(d)} rows, {t[-1]-t[0]:.1f} s, {1/np.median(np.diff(t)):.0f} Hz")
    print(f"  |v| mean {np.linalg.norm(vel,axis=1).mean():.3f} max "
          f"{np.linalg.norm(vel,axis=1).max():.3f} m/s | "
          f"|w| mean {np.degrees(np.linalg.norm(ang,axis=1)).mean():.0f} max "
          f"{np.degrees(np.linalg.norm(ang,axis=1)).max():.0f} deg/s")

    report("translation", coast_events(t, np.linalg.norm(vel, axis=1), pos,
                                       FAST_LIN, SLOW_LIN, STILL_LIN), "mm", 1000.0)
    report("rotation", coast_events(t, np.linalg.norm(ang, axis=1), q,
                                    FAST_ANG, SLOW_ANG, STILL_ANG), "quat", 1.0)


if __name__ == "__main__":
    main()
