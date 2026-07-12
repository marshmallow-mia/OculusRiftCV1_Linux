#!/usr/bin/env python3
"""
tools/compare_dynamic.py — Tier-1 motion-protocol metric table.

Eats two CSVs in the ovr_pose_log/lin_pose_log schema (one per stack, same
motion-protocol test) and prints the plan §3 metrics side by side:

Stationary tests (1, 2):
  - per-axis std + peak-to-peak of position (mm) and pitch/roll/yaw (deg)
  - PSD peak frequency of position (does noise sit at 60 Hz camera rate?)
  - optical step size: histogram of |Δpos| between consecutive samples,
    top percentile = discrete-jump magnitude

Rotation tests (3, 4):
  - yaw drift per revolution (test 3, needs --revs)
  - pitch/roll excursion symmetry (test 4)

Dynamic tests (5, 6):
  - velocity / angular-velocity field magnitudes (their smoothing+prediction)
  - position RMS error vs the taped 1 m square (test 6, --square 1.0)

Occlusion tests (7, 8):
  - dropout intervals (status flag transitions)
  - re-acquisition latency + snap magnitude (pose jump at reacquisition)

Usage:
  python compare_dynamic.py --test still  win.csv lin.csv
  python compare_dynamic.py --test yaw360 --revs 2 win.csv lin.csv
  python compare_dynamic.py --test square --square 1.0 win.csv lin.csv
  python compare_dynamic.py --test shake|sweep|occlude|reacquire|controllers win.csv lin.csv

Only stdlib + numpy required.
"""

import argparse
import csv
import math
import sys

try:
    import numpy as np
except ImportError:
    sys.exit("numpy required: pip install numpy")


# ---------------------------------------------------------------- load

def load(path):
    """Return dict of numpy arrays keyed by column name."""
    with open(path, newline="") as f:
        r = csv.reader(f)
        header = next(r)
        rows = [row for row in r if len(row) == len(header)]
    if not rows:
        sys.exit(f"{path}: no data rows")
    data = np.array(rows, dtype=float)
    return {name: data[:, i] for i, name in enumerate(header)}


def euler_deg(d):
    """quat (hqx..hqw) -> pitch, roll, yaw in degrees (y-up, -z forward)."""
    x, y, z, w = d["hqx"], d["hqy"], d["hqz"], d["hqw"]
    # yaw (y), pitch (x), roll (z) — Tait-Bryan YXZ, matches HMD conventions
    sinp = np.clip(2 * (w * x - y * z), -1, 1)
    pitch = np.degrees(np.arcsin(sinp))
    yaw = np.degrees(np.arctan2(2 * (w * y + x * z),
                                1 - 2 * (x * x + y * y)))
    roll = np.degrees(np.arctan2(2 * (w * z + x * y),
                                 1 - 2 * (x * x + z * z)))
    return pitch, roll, yaw


def unwrap_deg(a):
    return np.degrees(np.unwrap(np.radians(a)))


def rate_hz(d):
    dt = np.diff(d["t_ovr"])
    dt = dt[dt > 0]
    return 1.0 / np.median(dt) if len(dt) else float("nan")


# ---------------------------------------------------------------- metrics

def m_still(d):
    """Stationary noise floor."""
    out = {}
    out["sample rate (Hz)"] = rate_hz(d)
    pitch, roll, yaw = euler_deg(d)
    for name, arr, scale, unit in [
        ("pos x", d["hpx"], 1000, "mm"), ("pos y", d["hpy"], 1000, "mm"),
        ("pos z", d["hpz"], 1000, "mm"),
        ("pitch", pitch, 1, "deg"), ("roll", roll, 1, "deg"),
        ("yaw", unwrap_deg(yaw), 1, "deg"),
    ]:
        a = arr * scale
        out[f"{name} std ({unit})"] = np.std(a)
        out[f"{name} p2p ({unit})"] = np.ptp(a)

    # step magnitude between consecutive samples — the discrete optical jump
    pos = np.stack([d["hpx"], d["hpy"], d["hpz"]], axis=1)
    steps = np.linalg.norm(np.diff(pos, axis=0), axis=1) * 1000
    nz = steps[steps > 1e-6]  # runtime may return identical pose between updates
    if len(nz):
        out["step p50 (mm)"] = np.percentile(nz, 50)
        out["step p99 (mm)"] = np.percentile(nz, 99)
        out["step max (mm)"] = nz.max()

    # PSD peak of horizontal position — camera-rate noise shows at 60 Hz
    t = d["t_ovr"]
    fs = rate_hz(d)
    if len(t) > 256 and math.isfinite(fs):
        x = d["hpx"] - d["hpx"].mean()
        # resample to uniform grid for FFT
        tu = np.linspace(t[0], t[-1], len(t))
        xu = np.interp(tu, t, x)
        psd = np.abs(np.fft.rfft(xu * np.hanning(len(xu)))) ** 2
        freqs = np.fft.rfftfreq(len(xu), (tu[-1] - tu[0]) / len(tu))
        band = (freqs > 1.0)  # ignore DC/drift
        if band.any():
            out["PSD peak freq (Hz)"] = freqs[band][np.argmax(psd[band])]
    return out


def m_yaw360(d, revs):
    """Yaw drift per revolution on the turntable."""
    _, _, yaw = euler_deg(d)
    yawu = unwrap_deg(yaw)
    total = yawu[-1] - yawu[0]
    expected = 360.0 * revs * (1 if total >= 0 else -1)
    return {
        "sample rate (Hz)": rate_hz(d),
        "total yaw (deg)": total,
        "expected (deg)": expected,
        "drift per rev (deg)": (total - expected) / max(revs, 1),
    }


def m_sweep(d):
    pitch, roll, _ = euler_deg(d)
    return {
        "sample rate (Hz)": rate_hz(d),
        "pitch min/max (deg)": (pitch.min(), pitch.max()),
        "roll  min/max (deg)": (roll.min(), roll.max()),
        "pitch symmetry (deg)": pitch.max() + pitch.min(),  # 0 = symmetric
        "roll  symmetry (deg)": roll.max() + roll.min(),
    }


def m_shake(d):
    v = np.linalg.norm(np.stack([d["hvx"], d["hvy"], d["hvz"]], 1), axis=1)
    w = np.linalg.norm(np.stack([d["hwx"], d["hwy"], d["hwz"]], 1), axis=1)
    a = np.linalg.norm(np.stack([d["hax"], d["hay"], d["haz"]], 1), axis=1)
    return {
        "sample rate (Hz)": rate_hz(d),
        "|v| p95 (m/s)": np.percentile(v, 95),
        "|w| p95 (rad/s)": np.percentile(w, 95),
        "|a| p95 (m/s2)": np.percentile(a, 95),
        "|v| max": v.max(), "|w| max": w.max(),
    }


def m_square(d, side):
    """Walked square: how close are the corner-to-corner distances to `side`?
    Crude but assumption-free: fit via extremes of the horizontal bounding box."""
    x, z = d["hpx"], d["hpz"]
    return {
        "sample rate (Hz)": rate_hz(d),
        "x extent (m)": np.ptp(x),
        "z extent (m)": np.ptp(z),
        "x scale err (%)": (np.ptp(x) / side - 1) * 100,
        "z scale err (%)": (np.ptp(z) / side - 1) * 100,
        "height drift p2p (mm)": np.ptp(d["hpy"]) * 1000,
    }


def m_reacquire(d, pos_bit=2):
    """Dropout + reacquisition from the h_status position-tracked bit."""
    status = d["h_status"].astype(int)
    tracked = (status & pos_bit) != 0
    t = d["t_ovr"]
    pos = np.stack([d["hpx"], d["hpy"], d["hpz"]], axis=1)

    out = {"sample rate (Hz)": rate_hz(d)}
    drops = []
    snaps = []
    i = 0
    n = len(tracked)
    while i < n:
        if not tracked[i]:
            j = i
            while j < n and not tracked[j]:
                j += 1
            if j < n:
                drops.append(t[j] - t[i])
                # snap = pose jump across the reacquisition edge
                if i > 0:
                    snaps.append(np.linalg.norm(pos[j] - pos[i - 1]) * 1000)
            i = j
        else:
            i += 1
    out["dropouts (count)"] = len(drops)
    if drops:
        out["dropout mean (s)"] = float(np.mean(drops))
        out["reacq snap p50 (mm)"] = float(np.median(snaps)) if snaps else 0.0
        out["reacq snap max (mm)"] = float(np.max(snaps)) if snaps else 0.0
    else:
        out["NOTE"] = ("no status dropouts — stack may coast on IMU without "
                       "flagging; check step metrics instead")
        s = m_still(d)
        for k in ("step p99 (mm)", "step max (mm)"):
            if k in s:
                out[k] = s[k]
    return out


def m_controllers(d):
    out = {"sample rate (Hz)": rate_hz(d)}
    for c in ("c0", "c1"):
        pos = np.stack([d[f"{c}px"], d[f"{c}py"], d[f"{c}pz"]], axis=1)
        if not np.any(pos):
            out[f"{c}"] = "no data"
            continue
        steps = np.linalg.norm(np.diff(pos, axis=0), axis=1) * 1000
        nz = steps[steps > 1e-6]
        out[f"{c} pos p2p (mm)"] = float(np.ptp(np.linalg.norm(pos, axis=1)) * 1000)
        if len(nz):
            out[f"{c} step p99 (mm)"] = float(np.percentile(nz, 99))
    return out


TESTS = {
    "still":       lambda d, a: m_still(d),
    "yaw360":      lambda d, a: m_yaw360(d, a.revs),
    "sweep":       lambda d, a: m_sweep(d),
    "shake":       lambda d, a: m_shake(d),
    "square":      lambda d, a: m_square(d, a.square),
    "occlude":     lambda d, a: m_reacquire(d),
    "reacquire":   lambda d, a: m_reacquire(d),
    "controllers": lambda d, a: m_controllers(d),
}


def fmt(v):
    if isinstance(v, float):
        return f"{v:.4f}"
    if isinstance(v, tuple):
        return "(" + ", ".join(f"{x:.3f}" for x in v) + ")"
    return str(v)


def main():
    ap = argparse.ArgumentParser(description="Tier-1 metric comparison")
    ap.add_argument("--test", required=True, choices=sorted(TESTS))
    ap.add_argument("--revs", type=float, default=2.0,
                    help="turntable revolutions for yaw360 (default 2)")
    ap.add_argument("--square", type=float, default=1.0,
                    help="taped square side in metres (default 1.0)")
    ap.add_argument("csv_a", help="Windows/Oculus CSV")
    ap.add_argument("csv_b", help="Linux/ours CSV")
    args = ap.parse_args()

    da, db = load(args.csv_a), load(args.csv_b)
    ma = TESTS[args.test](da, args)
    mb = TESTS[args.test](db, args)

    keys = list(dict.fromkeys(list(ma) + list(mb)))
    wk = max(len(k) for k in keys)
    print(f"\n=== test: {args.test} ===")
    print(f"{'metric':<{wk}}  {'oculus (A)':>16}  {'ours (B)':>16}")
    print("-" * (wk + 36))
    for k in keys:
        print(f"{k:<{wk}}  {fmt(ma.get(k, '—')):>16}  {fmt(mb.get(k, '—')):>16}")
    print()


if __name__ == "__main__":
    main()
