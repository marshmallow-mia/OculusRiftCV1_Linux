#!/usr/bin/env python3
"""Recover the accelerometer correction from the data, and check the factory one.

At rest an accelerometer measures gravity and nothing else, so every stationary
reading must land on a sphere of radius 9.8067 once corrected. Uncorrected they
land on an ELLIPSOID instead — its centre is the bias and its axes are the scale
and cross-axis errors. Fitting that ellipsoid therefore recovers the true
correction from nothing but the raw stream, with no reference hardware.

This exists because our driver's correction is measurably wrong: applied to the
Oculus runtime's own raw capture, the factory offset alone lands |accel| at
9.7503 (-0.6%) but offset+matrix overshoots to 10.0963 (+3.0%), and the live
headset reads 10.0543 (+2.5%). Something about how we decode or apply the matrix
is off, and the fit says what the matrix SHOULD be, independent of any
convention argument.

The ellipsoid fit is linear least squares (a quadric through the samples), so it
needs no optimiser and no scipy.

  tools/fit_accel_calib.py captures/win/2026-07-12/imu_tracking.pcap
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from measure_imu_noise import load_samples  # noqa: E402

G = 9.80665

# The factory values this headset reports, as our packet.c decodes them
# (logged by dump_packet_imu_calibration).
FACTORY_M = np.array([[1.021, 0.001, -0.018],
                      [0.001, 1.037, 0.022],
                      [0.004, -0.005, 0.963]])
FACTORY_OFF = np.array([0.108, -0.3056, 0.0249])


def fit_ellipsoid(P):
    """Least-squares quadric through P: x^T A x + 2 v^T x + w = 0."""
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    D = np.column_stack([x * x, y * y, z * z,
                         2 * x * y, 2 * x * z, 2 * y * z,
                         2 * x, 2 * y, 2 * z,
                         np.ones_like(x)])
    # smallest singular vector = best fit up to scale
    _, _, Vt = np.linalg.svd(D, full_matrices=False)
    a, b, c, d, e, f, g, h, i, w = Vt[-1]
    A = np.array([[a, d, e], [d, b, f], [e, f, c]])
    v = np.array([g, h, i])
    return A, v, w


def correction_from_ellipsoid(A, v, w):
    """Centre (bias) and the matrix M with |M(x - centre)| = G on the fit."""
    centre = -np.linalg.solve(A, v)
    k = float(centre @ A @ centre - w)
    if k <= 0:                       # sign convention of the null vector
        A, v, w, k = -A, -v, -w, -k
    # M^T M = A * G^2 / k  -> symmetric square root
    ev, R = np.linalg.eigh(A * G * G / k)
    if np.any(ev <= 0):
        return None, None
    M = R @ np.diag(np.sqrt(ev)) @ R.T
    return M, centre


def score(name, X):
    m = np.linalg.norm(X, axis=1)
    print("  %-38s %.4f m/s^2  (%+.2f%%)   spread %.4f" %
          (name, np.median(m), (np.median(m) / G - 1) * 100,
           np.percentile(m, 75) - np.percentile(m, 25)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcap")
    ap.add_argument("--quiet-gyro", type=float, default=0.05)
    a = ap.parse_args()

    accel, gyro = load_samples(a.pcap)
    A_all = np.array(accel)
    q = np.linalg.norm(np.array(gyro), axis=1) < a.quiet_gyro
    P = A_all[q]
    print("%s\n  %d samples, %d stationary" % (a.pcap, len(A_all), len(P)))

    # Orientation coverage decides whether the fit is even determined.
    u = P / np.linalg.norm(P, axis=1, keepdims=True)
    cover = len(np.unique(np.round(u, 1), axis=0))
    print("  distinct gravity directions (0.1 rounding): %d" % cover)
    if cover < 12:
        print("  WARNING: too few orientations to constrain a full 3x3 fit;"
              " treat the matrix as under-determined.")

    Aq, v, w = fit_ellipsoid(P)
    M, centre = correction_from_ellipsoid(Aq, v, w)
    if M is None:
        print("  ellipsoid fit is not positive definite - cannot recover a correction")
        return

    print("\n=== |accel| at rest under each correction ===")
    score("raw (no correction)", P)
    score("factory offset only", P - FACTORY_OFF)
    score("factory offset + matrix (ours)", (FACTORY_M @ (P - FACTORY_OFF).T).T)
    score("fitted from the data", (M @ (P - centre).T).T)

    print("\n=== fitted correction vs the factory one ===")
    print("  fitted bias   [%+8.4f %+8.4f %+8.4f]" % tuple(centre))
    print("  factory offset[%+8.4f %+8.4f %+8.4f]" % tuple(FACTORY_OFF))
    print("\n  fitted matrix                    factory matrix (as we decode it)")
    for r in range(3):
        print("  [%+7.4f %+7.4f %+7.4f]      [%+7.4f %+7.4f %+7.4f]" %
              (*M[r], *FACTORY_M[r]))

    # If our decode merely over-scales the stored deltas, the fitted deviation
    # from identity is the factory deviation divided by some constant factor.
    dev_fit = M - np.eye(3)
    dev_fac = FACTORY_M - np.eye(3)
    mask = np.abs(dev_fac) > 5e-3
    if mask.sum():
        ratios = dev_fac[mask] / dev_fit[mask]
        print("\n  deviation-from-identity ratio factory/fitted, on the %d "
              "significant terms:" % mask.sum())
        print("    median %.2f   mean %.2f   spread %.2f..%.2f" %
              (np.median(ratios), ratios.mean(), ratios.min(), ratios.max()))
        print("    (a consistent integer-ish ratio means our decode scales the"
              " stored deltas wrongly)")


if __name__ == "__main__":
    main()
