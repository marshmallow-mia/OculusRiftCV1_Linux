#!/usr/bin/env python3
"""Does the oscillation exist in the vision solve, or is it added after it?

The reported symptom is "I move and it overshoots, gets back too far, etc until
it reached the real point" — a damped oscillation, on the HMD and both Touch
controllers alike. Vision is an independent per-frame PnP solve with no temporal
state, so it *cannot* ring on its own. Either it is already jumping around (and
the fusion is faithfully chasing a bad input), or it is smooth and the
oscillation is manufactured downstream. That splits the problem in half.

Two measurements, both from an existing OHMD_RIFT_CAL_CAPTURE log:

  smoothness   per device, the vision position against a short median-filtered
               reference. Real head motion is smooth at 60 Hz; anything the
               fusion has to chase shows up here as scatter.

  camera pair  where both sensors solved the SAME exposure, the disagreement
               between their two independent world-pose estimates. This is the
               one that matters for a shared HMD+controller artifact: if the
               extrinsics are off, the estimate SHIFTS as the device moves
               between the cameras' fields of view, and the fusion chases a
               target that moves whenever the wearer does.

  tools/vision_quality.py captures/lin/<date>/<tag>.jsonl
"""
import argparse
import json
from collections import defaultdict

import numpy as np

LED_IDS = 0x200          # rift-sensor-pose-helper.h
STRONG = 0x2


def load(path, max_re=None):
    """Observations that the tracker itself would have accepted."""
    obs = []
    for line in open(path, errors="replace"):
        line = line.strip()
        if not line:
            continue
        try:
            d = json.loads(line)
        except ValueError:
            continue
        if d.get("t") != "obs" or "wp" not in d:
            continue
        if not (d.get("flags", 0) & LED_IDS):
            continue
        obs.append(d)
    if max_re is not None and obs:
        res = np.array([o.get("re", 0.0) for o in obs])
        keep = res <= np.percentile(res, max_re)
        obs = [o for o, k in zip(obs, keep) if k]
    return obs


def smoothness(obs, label):
    """Scatter of the vision position about a short running median."""
    if len(obs) < 30:
        print(f"  {label:<14s} only {len(obs)} observations - skipped")
        return
    obs = sorted(obs, key=lambda d: d["ts"])
    t = np.array([d["ts"] for d in obs]) * 1e-9
    p = np.array([d["wp"][:3] for d in obs])

    # running median over 5 samples ~= 80 ms: follows real motion, rejects
    # single-frame excursions
    k = 5
    med = np.stack([
        np.median(p[max(0, i - k // 2):i + k // 2 + 1], axis=0)
        for i in range(len(p))
    ])
    resid = np.linalg.norm(p - med, axis=1) * 1000.0

    dt = np.diff(t)
    speed = np.linalg.norm(np.diff(p, axis=0), axis=1) / np.maximum(dt, 1e-6)
    moving = np.concatenate([[False], speed > 0.15])

    print(f"  {label:<14s} n={len(obs):5d}  residual from running median: "
          f"median {np.median(resid):6.2f} mm  p90 {np.percentile(resid, 90):7.2f}  "
          f"max {resid.max():8.1f}")
    if moving.sum() > 20:
        print(f"  {'':<14s}        while moving (>0.15 m/s): "
              f"median {np.median(resid[moving]):6.2f} mm  "
              f"p90 {np.percentile(resid[moving], 90):7.2f}")


def camera_pair(obs, label, tol_ns=8_000_000):
    """Disagreement between the two sensors solving the same exposure."""
    by_sensor = defaultdict(list)
    for d in obs:
        by_sensor[d["s"]].append(d)
    if len(by_sensor) < 2:
        print(f"  {label:<14s} only one sensor present - skipped")
        return
    (sa, la), (sb, lb) = sorted(by_sensor.items())[:2]
    la = sorted(la, key=lambda d: d["ts"])
    lb = sorted(lb, key=lambda d: d["ts"])
    tb = np.array([d["ts"] for d in lb])

    diffs, positions = [], []
    for d in la:
        j = int(np.searchsorted(tb, d["ts"]))
        for cand in (j - 1, j):
            if 0 <= cand < len(lb) and abs(lb[cand]["ts"] - d["ts"]) <= tol_ns:
                pa = np.array(d["wp"][:3])
                pb = np.array(lb[cand]["wp"][:3])
                diffs.append(np.linalg.norm(pa - pb))
                positions.append((pa + pb) / 2.0)
                break
    if len(diffs) < 10:
        print(f"  {label:<14s} only {len(diffs)} co-observed exposures - skipped")
        return

    diffs = np.array(diffs) * 1000.0
    positions = np.array(positions)
    print(f"  {label:<14s} co-observed {len(diffs):5d}  cross-camera disagreement: "
          f"median {np.median(diffs):6.2f} mm  p90 {np.percentile(diffs, 90):7.2f}  "
          f"max {diffs.max():8.1f}")

    # Does it vary with WHERE the device is? A constant offset is a calibration
    # bias the fusion absorbs once; one that varies with position is a moving
    # target it has to chase every time the wearer moves.
    span = positions.max(axis=0) - positions.min(axis=0)
    if span.max() > 0.10:
        axis = int(np.argmax(span))
        order = np.argsort(positions[:, axis])
        q = len(order) // 4
        lo = diffs[order[:q]].mean()
        hi = diffs[order[-q:]].mean()
        print(f"  {'':<14s}        across {span[axis]*100:.0f} cm of travel on "
              f"axis {'xyz'[axis]}: {lo:.2f} mm at one end vs {hi:.2f} mm at the "
              f"other (delta {abs(hi-lo):.2f} mm)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--re-percentile", type=float, default=90.0,
                    help="drop observations above this reprojection-error percentile")
    a = ap.parse_args()

    obs = load(a.capture, a.re_percentile)
    print(f"{a.capture}")
    print(f"  {len(obs)} LED-ID-verified observations kept "
          f"(reprojection error under the {a.re_percentile:.0f}th percentile)")

    names = {0: "HMD", 1: "controller 1", 2: "controller 2", 3: "controller 3"}
    by_dev = defaultdict(list)
    for d in obs:
        by_dev[d.get("d", 0)].append(d)

    print("\n=== is the vision position itself smooth? ===")
    for dev in sorted(by_dev):
        smoothness(by_dev[dev], names.get(dev, f"device {dev}"))

    print("\n=== do the two cameras agree with each other? ===")
    for dev in sorted(by_dev):
        camera_pair(by_dev[dev], names.get(dev, f"device {dev}"))


if __name__ == "__main__":
    main()
