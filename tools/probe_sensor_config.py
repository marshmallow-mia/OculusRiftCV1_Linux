#!/usr/bin/env python3
"""Plan §4a / findings-4a-hid.md §3: does RIFT_SCF_AUTO_CALIBRATION stick?

OpenHMD (rift.c:1157-1158) force-sets USE_CALIBRATION|AUTO_CALIBRATION in the
CV1's SENSOR_CONFIG feature report, with the comment "these don't seem to stick".
The Oculus runtime writes flags=0x20 (COMMAND_KEEP_ALIVE only) and never enables
on-HMD auto-calibration at all.

If the flag sticks, the HMD firmware is walking its own gyro zero-rate offset
underneath our fusion filter, which is also estimating that offset — two
estimators fighting. This decides it, by writing the report and reading it back.

Read-only by default. --test performs the write/readback experiment and restores
the original config afterwards. Nothing here is persistent: SENSOR_CONFIG resets
on HMD power cycle, and every OpenHMD start rewrites it.

Requires no root (udev rules from install.py give hidraw access). SteamVR and
openhmd_simple_example must NOT be running.
"""
import argparse
import fcntl
import glob
import os
import struct
import sys

HIDIOCGFEATURE = lambda n: (3 << 30) | (n << 16) | (0x48 << 8) | 0x07
HIDIOCSFEATURE = lambda n: (3 << 30) | (n << 16) | (0x48 << 8) | 0x06

RIFT_CMD_SENSOR_CONFIG = 0x02
SCF = [
    (0x01, "RAW_MODE"), (0x02, "CALIBRATION_TEST"), (0x04, "USE_CALIBRATION"),
    (0x08, "AUTO_CALIBRATION"), (0x10, "MOTION_KEEP_ALIVE"),
    (0x20, "COMMAND_KEEP_ALIVE"), (0x40, "SENSOR_COORDINATES"),
]


def fmt(flags):
    on = [n for b, n in SCF if flags & b]
    return f"0x{flags:02x} [{'|'.join(on) if on else 'none'}]"


def hmd_nodes():
    out = []
    for path in glob.glob("/sys/class/hidraw/hidraw*"):
        try:
            with open(os.path.join(path, "device/uevent")) as f:
                if "00002833:00000031" in f.read():
                    out.append("/dev/" + os.path.basename(path))
        except OSError:
            pass
    return sorted(out)


def get_config(fd):
    buf = bytearray(7)
    buf[0] = RIFT_CMD_SENSOR_CONFIG
    fcntl.ioctl(fd, HIDIOCGFEATURE(7), buf)
    _, cmd, flags, interval, keepalive = struct.unpack("<BHBBH", bytes(buf))
    return dict(cmd=cmd, flags=flags, interval=interval, keepalive=keepalive,
                raw=bytes(buf))


def set_config(fd, flags, interval, keepalive, cmd=0):
    buf = bytearray(struct.pack("<BHBBH", RIFT_CMD_SENSOR_CONFIG, cmd, flags,
                                interval, keepalive))
    fcntl.ioctl(fd, HIDIOCSFEATURE(7), buf)


def show(tag, c):
    print(f"  {tag:<26} {c['raw'].hex()}  flags={fmt(c['flags'])}"
          f"  packet_interval={c['interval']}  keep_alive={c['keepalive']} ms")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--test", action="store_true",
                    help="write OpenHMD's flags, read back, then restore")
    a = ap.parse_args()

    nodes = hmd_nodes()
    if not nodes:
        sys.exit("no CV1 HMD hidraw node found (2833:0031) — is it plugged in?")

    fd = None
    for n in nodes:
        try:
            f = os.open(n, os.O_RDWR)
            get_config(f)  # the config interface is the one that answers
            fd, node = f, n
            break
        except OSError:
            try:
                os.close(f)
            except Exception:
                pass
    if fd is None:
        sys.exit(f"none of {nodes} answered a SENSOR_CONFIG GET_REPORT")

    print(f"HMD hidraw: {node}\n")
    orig = get_config(fd)
    show("as found:", orig)

    if not a.test:
        print("\n(run with --test to perform the write/readback experiment)")
        return

    # 1. Exactly what OpenHMD rift.c:1157-1158 does: OR in USE|AUTO calibration.
    want = orig["flags"] | 0x04 | 0x08
    print(f"\n[1] writing OpenHMD's flags: {fmt(want)}")
    set_config(fd, want, orig["interval"], orig["keepalive"])
    got = get_config(fd)
    show("readback:", got)

    stuck_use = bool(got["flags"] & 0x04)
    stuck_auto = bool(got["flags"] & 0x08)
    print(f"\n    USE_CALIBRATION  (0x04) stuck: {stuck_use}")
    print(f"    AUTO_CALIBRATION (0x08) stuck: {stuck_auto}")

    # 2. Exactly what the Oculus runtime writes.
    print(f"\n[2] writing Oculus's flags: {fmt(0x20)}  (interval=1, keep_alive=1000)")
    set_config(fd, 0x20, 1, 1000)
    got2 = get_config(fd)
    show("readback:", got2)

    print("\n" + "=" * 66)
    if stuck_auto:
        print("VERDICT: AUTO_CALIBRATION STICKS.")
        print("  The HMD firmware is running its own gyro-bias auto-calibration")
        print("  underneath our filter, which also estimates that bias. Oculus")
        print("  never enables it. -> drop the SETFLAG at rift.c:1158 and A/B.")
    else:
        print("VERDICT: AUTO_CALIBRATION DOES NOT STICK.")
        print("  OpenHMD's source comment is right; the firmware drops the flag.")
        print("  findings-4a-hid.md §3 is a dead end — no two-estimator conflict.")
    print("=" * 66)

    print(f"\nrestoring config as found: {orig['raw'].hex()}")
    set_config(fd, orig["flags"], orig["interval"], orig["keepalive"], orig["cmd"])
    show("final:", get_config(fd))
    os.close(fd)


if __name__ == "__main__":
    main()
