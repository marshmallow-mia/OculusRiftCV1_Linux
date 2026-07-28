#!/usr/bin/env python
"""Measure CV1 IMU noise from a raw HID capture, to set filter R matrices.

`rift-kalman-6dof.c` shipped with `m1.R = 1e-6` for the accelerometer — a
1 mm/s^2 standard deviation — under a `FIXME: Set R matrix to something based
on IMU noise`. That tells the UKF to believe the accelerometer essentially
absolutely. This measures what the noise actually is, from the raw 1 kHz IMU
reports in a USB capture, so the value can be derived rather than invented.

Method: decode every 0x0b report (same layout as packet.c
decode_tracker_sensor_msg_dk2), find the quietest windows by gyro magnitude —
where the device is closest to stationary and true acceleration is nearly
constant — and take the per-axis standard deviation of the accelerometer there.
That is an upper bound on the sensor noise, since any residual real motion adds
to it.

Usage:
  measure_imu_noise.py captures/win/2026-07-12/imu_tracking.pcap [--window 200]
"""

import argparse
import statistics as st
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from analyze_hid_pcap import decode_irq, tshark, unhex  # noqa: E402


def load_samples(path):
    rows = tshark(path, "usb.transfer_type==0x01 && usb.endpoint_address.direction==1",
                  ["frame.time_relative", "usbhid.data", "usb.capdata"])
    accel, gyro = [], []
    for r in rows:
        raw = unhex(r[1]) if len(r) > 1 and r[1].strip() else (
            unhex(r[2]) if len(r) > 2 else b"")
        m = decode_irq(raw)
        if not m:
            continue
        for a, g in m["samples"]:
            accel.append(a)
            gyro.append(g)
    return accel, gyro


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcap", type=Path)
    ap.add_argument("--window", type=int, default=200,
                    help="samples per window (200 = 200 ms at 1 kHz)")
    ap.add_argument("--quietest", type=int, default=20,
                    help="how many of the quietest windows to report over")
    args = ap.parse_args()

    accel, gyro = load_samples(args.pcap)
    if not accel:
        raise SystemExit("no IMU reports decoded")
    print(f"samples {len(accel)}")

    w = args.window
    windows = []
    for s in range(0, len(accel) - w, w):
        g = gyro[s:s + w]
        a = accel[s:s + w]
        gmag = sum((gx * gx + gy * gy + gz * gz) ** 0.5 for gx, gy, gz in g) / w
        sd = [st.pstdev([v[i] for v in a]) for i in range(3)]
        windows.append((gmag, sd, a))

    windows.sort(key=lambda t: t[0])
    quiet = windows[:args.quietest]
    print(f"windows {len(windows)} of {w} samples; "
          f"quietest {len(quiet)} have mean |gyro| "
          f"{min(q[0] for q in quiet):.5f}..{max(q[0] for q in quiet):.5f} rad/s")

    per_axis = [[q[1][i] for q in quiet] for i in range(3)]
    print("\naccelerometer standard deviation in the quietest windows (m/s^2):")
    for i, name in enumerate("xyz"):
        v = per_axis[i]
        print(f"  {name}: median {st.median(v):.5f}   min {min(v):.5f}   max {max(v):.5f}")

    worst = max(st.median(v) for v in per_axis)
    print(f"\n  representative sigma  {worst:.5f} m/s^2")
    print(f"  variance (R diagonal)  {worst ** 2:.3e} (m/s^2)^2")
    print(f"  vs the shipped 1e-6:   {worst ** 2 / 1e-6:.0f}x larger")


if __name__ == "__main__":
    main()
