#!/usr/bin/env python3
"""Plan §4a: mine a CV1 HMD USB capture for what the host writes/reads, and
validate our IMU parse byte-for-byte against the driver's own struct layouts.

Works on both USBPcap captures (Windows / Oculus runtime) and Linux usbmon
captures (our OpenHMD driver), so the two can be diffed directly.

    ./analyze_hid_pcap.py setup_hid.pcap              # feature-report census
    ./analyze_hid_pcap.py imu_tracking.pcap --imu     # + IMU stream validation

Layouts mirror openhmd/src/drv_oculus_rift/{packet.c,rift.h}. If those change,
this must change with them.
"""
import argparse
import struct
import subprocess
import sys
from collections import Counter, defaultdict

# --- rift.h: feature report IDs ------------------------------------------
REPORTS = {
    0x02: "SENSOR_CONFIG",
    0x03: "IMU_CALIBRATION",
    0x04: "RANGE",
    0x08: "DK1_KEEP_ALIVE",
    0x09: "DISPLAY_INFO",
    0x0C: "TRACKING_CONFIG",
    0x0F: "POSITION_INFO",
    0x10: "PATTERN_INFO",
    0x11: "DK2_KEEP_ALIVE",
    0x1A: "RADIO_CONTROL",
    0x1B: "RADIO_READ_DATA",
    0x1D: "ENABLE_COMPONENTS",
}
SCF = [
    (0x01, "RAW_MODE"), (0x02, "CALIBRATION_TEST"), (0x04, "USE_CALIBRATION"),
    (0x08, "AUTO_CALIBRATION"), (0x10, "MOTION_KEEP_ALIVE"),
    (0x20, "COMMAND_KEEP_ALIVE"), (0x40, "SENSOR_COORDINATES"),
]
TRK = [
    (0x01, "ENABLE"), (0x02, "AUTO_INCREMENT"), (0x04, "USE_CARRIER"),
    (0x08, "SYNC_INPUT"), (0x10, "VSYNC_LOCK"), (0x20, "CUSTOM_PATTERN"),
]
COMPONENTS = [(1, "DISPLAY"), (2, "AUDIO"), (4, "LEDS")]

RIFT_IRQ_SENSORS_DK2 = 0x0B
SAMPLE_SCALE = 0.0001  # packet.c vec3f_from_rift_vec


def bits(val, table):
    on = [n for b, n in table if val & b]
    return f"0x{val:02x} [{'|'.join(on) if on else 'none'}]"


def tshark(path, dfilter, fields):
    cmd = ["tshark", "-r", path, "-Y", dfilter, "-T", "fields"]
    for f in fields:
        cmd += ["-e", f]
    out = subprocess.run(cmd, capture_output=True, text=True).stdout
    return [ln.split("\t") for ln in out.splitlines() if ln.strip()]


def unhex(s):
    return bytes.fromhex(s.replace(":", "").strip()) if s.strip() else b""


# --- decoders (packet.c) --------------------------------------------------
def decode_sensor_config(b):
    if len(b) < 7:
        return None
    _, cmd, flags, interval, keepalive = struct.unpack("<BHBBH", b[:7])
    return (f"flags={bits(flags, SCF)}  packet_interval={interval}  "
            f"keep_alive={keepalive} ms  (cmd_id={cmd})")


def decode_tracking_config(b):
    if len(b) < 13:
        return None
    _, cmd, pat, flags, _rsv, exp, per, vs, duty = struct.unpack("<BHBBBHHHB", b[:13])
    hz = f"{1e6 / per:.2f} Hz" if per else "-"
    return (f"pattern=0x{pat:02x}  flags={bits(flags, TRK)}  exposure={exp} us  "
            f"period={per} us ({hz})  vsync_offset={vs}  duty=0x{duty:02x}")


def decode_enable_components(b):
    if len(b) < 4:
        return None
    return f"components={bits(b[3], COMPONENTS)}"


DECODERS = {0x02: decode_sensor_config, 0x0C: decode_tracking_config,
            0x1D: decode_enable_components}


def sample(buf):
    """packet.c decode_sample: 3x21-bit signed, tightly packed in 8 bytes."""
    x = (buf[0] << 24) | (buf[1] << 16) | ((buf[2] & 0xF8) << 8)
    y = ((buf[2] & 0x07) << 29) | (buf[3] << 21) | (buf[4] << 13) | ((buf[5] & 0xC0) << 5)
    z = ((buf[5] & 0x3F) << 26) | (buf[6] << 18) | (buf[7] << 10)
    to_s32 = lambda v: v - (1 << 32) if v & 0x80000000 else v
    return tuple((to_s32(v) >> 11) * SAMPLE_SCALE for v in (x, y, z))


def decode_irq(b):
    """packet.c decode_tracker_sensor_msg_dk2 — the 64-byte CV1 IMU report."""
    if len(b) != 64 or b[0] != RIFT_IRQ_SENSORS_DK2:
        return None
    num = min(b[3], 2)
    total, temp, ts = struct.unpack("<HHI", b[4:12])
    samples = [(sample(b[12 + i * 16: 20 + i * 16]),
                sample(b[20 + i * 16: 28 + i * 16])) for i in range(num)]
    frame_count, frame_ts = struct.unpack("<HI", b[50:56])
    frame_id, led_phase = b[56], b[57]
    exp_count, exp_ts = struct.unpack("<HI", b[58:64])
    return dict(num=num, total=total, temp=temp / 100.0, ts=ts, samples=samples,
                frame_count=frame_count, frame_ts=frame_ts, frame_id=frame_id,
                led_phase=led_phase, exp_count=exp_count, exp_ts=exp_ts)


def features(path):
    print(f"\n{'=' * 72}\nFEATURE REPORTS  ({path})\n{'=' * 72}")
    for req, label in ((0x21, "WRITE (host -> HMD, SET_REPORT)"),
                       (0xA1, "READ  (HMD -> host, GET_REPORT)")):
        rows = tshark(path, f"usb.bmRequestType=={hex(req)}",
                      ["usbhid.setup.ReportID", "usb.data_fragment"])
        by_id = defaultdict(list)
        for r in rows:
            rid = r[0].strip()
            if rid.isdigit():
                by_id[int(rid)].append(unhex(r[1] if len(r) > 1 else ""))
        print(f"\n--- {label} ---")
        if not by_id:
            print("  (none)")
        for rid in sorted(by_id):
            payloads = by_id[rid]
            name = REPORTS.get(rid, "** UNKNOWN TO OPENHMD **")
            print(f"\n  0x{rid:02x} {name}  x{len(payloads)}")
            for raw, n in Counter(p for p in payloads if p).most_common(4):
                print(f"      x{n:<4} {raw.hex()[:80]}")
                dec = DECODERS.get(rid, lambda _: None)(raw)
                if dec:
                    print(f"            -> {dec}")


def imu(path):
    print(f"\n{'=' * 72}\nIMU STREAM  ({path})\n{'=' * 72}")
    # USBPcap puts interrupt-IN payloads in usbhid.data; usbmon uses usb.capdata.
    rows = tshark(path, "usb.transfer_type==0x01 && usb.endpoint_address.direction==1",
                  ["frame.time_relative", "usbhid.data", "usb.capdata"])
    msgs = []
    for r in rows:
        raw = unhex(r[1]) if len(r) > 1 and r[1].strip() else (
            unhex(r[2]) if len(r) > 2 else b"")
        m = decode_irq(raw)
        if m:
            m["t"] = float(r[0])
            msgs.append(m)
    if not msgs:
        print("  no 0x0b IMU reports found")
        return

    n_samp = sum(m["num"] for m in msgs)
    span = msgs[-1]["t"] - msgs[0]["t"]
    print(f"  reports {len(msgs)}   samples {n_samp}   span {span:.1f} s")
    print(f"  report rate {len(msgs) / span:.1f} Hz   sample rate {n_samp / span:.1f} Hz"
          "   (expect ~1000 Hz samples)")

    # Device timestamp continuity: ts is in us, should advance 1000 us/sample.
    gaps = Counter()
    lost = 0
    for a, b in zip(msgs, msgs[1:]):
        dt = (b["ts"] - a["ts"]) & 0xFFFFFFFF
        gaps[dt] += 1
        expected = a["num"] * 1000
        if dt != expected:
            lost += abs(dt - expected) // 1000
    print(f"  device dt histogram (us): {dict(gaps.most_common(5))}")
    print(f"  implied lost samples: {lost}")

    # total_sample_count is the device's own counter -> ground truth for drops.
    d_total = (msgs[-1]["total"] - msgs[0]["total"]) & 0xFFFF
    print(f"  total_sample_count delta {d_total} vs {n_samp} decoded"
          f"  (wraps at 65536; only meaningful for short captures)")

    # Physics check: if our scale/layout is right, |accel| ~ 9.81 at rest.
    import statistics as st
    accels = [(ax * ax + ay * ay + az * az) ** 0.5
              for m in msgs for (ax, ay, az), _ in m["samples"]]
    gyros = [(gx * gx + gy * gy + gz * gz) ** 0.5
             for m in msgs for _, (gx, gy, gz) in m["samples"]]
    accels.sort()
    gyros.sort()
    med_a = st.median(accels)
    print(f"\n  |accel| median {med_a:.3f} m/s^2   p05 {accels[len(accels) // 20]:.3f}"
          f"   p95 {accels[len(accels) * 19 // 20]:.3f}")
    print(f"  |gyro|  median {st.median(gyros):.4f} rad/s"
          f"   p95 {gyros[len(gyros) * 19 // 20]:.3f}")
    verdict = "OK" if 9.0 < med_a < 10.6 else "*** WRONG — scale/layout mismatch ***"
    print(f"  -> gravity check: {verdict}  (expect 9.81 m/s^2)")
    print(f"  temperature {msgs[0]['temp']:.1f} -> {msgs[-1]['temp']:.1f} C")

    exposures = sum(1 for a, b in zip(msgs, msgs[1:]) if b["exp_count"] != a["exp_count"])
    print(f"\n  camera exposure events {exposures} over {span:.1f} s"
          f" = {exposures / span:.2f} Hz  (expect ~52 Hz: 19200 us LED period)")
    phases = Counter(m["led_phase"] for m in msgs)
    print(f"  led_pattern_phase values seen: {sorted(phases)[:12]}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("pcap")
    ap.add_argument("--imu", action="store_true", help="also validate the IMU stream")
    a = ap.parse_args()
    features(a.pcap)
    if a.imu:
        imu(a.pcap)
