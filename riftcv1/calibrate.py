"""Guided end-to-end tracking calibration. Two halves, in order:

  1. optical — OpenHMD's rift-kalman-filter re-estimates the sensor
     (camera) pose at the START of every session, seeded from the
     headset's IMU gravity vector; nothing is persisted to disk. The
     optical step verifies this convergence happens fast and locks
     solidly — slow/failed convergence is what makes the world shift
     and swim when the head moves.
  2. SteamVR — Room Setup (Standing Only) anchors the standing centre
     and floor height. Because the tracking origin is wherever the
     headset rests when a session starts, that anchor only stays valid
     if every session starts from the same headset spot.

UI-agnostic: the CLI command and the GTK wizard both drive these helpers.
The capture functions use the same callback style as diagnostics
(set_text/append/cancel/set_proc)."""
import os
import subprocess
import threading
import time

from . import config, hw, runtime

PLACEMENT = """\
SENSOR PLACEMENT (do this before calibrating)
 - plug the sensor into a motherboard USB 3 port, directly — no hub, no
   add-in USB card (Renesas cards especially drop the camera mid-session)
 - stable mount 1-2 m from where you play, at head height or a little
   above, tilted to look at the middle of the play space
 - clear line of sight; no direct sunlight or halogen light into the
   lens; no mirrors or glossy surfaces in view

Move the sensor later = redo this calibration."""

STILL_HINT = """\
Put the headset 1-1.5 m in front of the sensor, front LEDs facing the
lens, and leave it COMPLETELY STILL (on a desk or tripod - do not hold
it). Do not touch it until the step reports a result."""

SESSION_NOTE = """\
This OpenHMD build does not store a room calibration on disk: the
sensor pose is re-estimated at the START of every session, seeded from
the headset's IMU gravity vector. Two practical consequences:
 - the headset's resting spot when a session starts becomes the
   tracking origin - pick a fixed start spot (mark it on the desk),
   facing the sensor, and use it every session
 - this step verifies that convergence: the tracker should get an
   optical lock within seconds and keep it"""

QUICK_ROOM_NOTE = """\
Sets the SteamVR standing centre and floor directly through the
Chaperone API - the official Room Setup app is skipped because it
crashes (segfaults) on many Linux systems.

The centre/floor stay valid across sessions as long as each session
starts with the headset at the same marked spot. Leave the headset
there now, facing the sensor, enter its height above the floor
(0 if it sits on the floor), and apply."""

ROOM_SETUP_STEPS = """\
Fallback - the official Room Setup app (known to crash on Linux):
  1. choose "Standing Only"
  2. leave the headset at your standing spot facing the sensor; once it
     shows tracking, click "Calibrate Center"
  3. floor: enter the headset's current height above the floor in cm
     (put it on the floor and enter 0), then click "Calibrate Floor"
"""

VERIFY_HINTS = """\
Quick sanity check with the headset on:
 - the horizon stays level and the world does NOT slide when you
   translate your head side to side
 - crouching changes your height correctly
If it still shifts, rerun the optical check with a better sensor view.
If only the height/centre is off after a restart, the headset probably
started from a different spot - restart SteamVR from the marked spot,
or rerun Room Setup."""


def preflight():
    """Pre-calibration checks as (ok, label, detail) rows.

    ok is True/None/False like StatusRow; False rows block the flow.
    """
    rows = []
    built = os.path.exists(config.OPENHMD_EXAMPLE)
    rows.append((built, "OpenHMD driver",
                 "built" if built else "not built — run Setup & install"))

    state = hw.hmd_hid_state()
    if state is None:
        rows.append((False, "Headset", "not found on USB"))
    elif state == "unbound":
        rows.append((False, "Headset", "HID not bound — run Fix USB first"))
    elif state == "in-use":
        rows.append((None, "Headset", "owned by a running VR session"))
    else:
        rows.append((True, "Headset", "connected, HID bound"))

    sensors = hw.tracking_sensors()
    if not sensors:
        rows.append((False, "Tracking sensor", "none found on USB"))
    for i, s in enumerate(sensors, 1):
        spd = s["speed"] or 0
        rows.append((True if spd >= 5000 else None, f"Sensor {i}",
                     f"USB 3.x, {spd} Mbps" if spd >= 5000 else
                     f"USB 2 ({spd} Mbps) — use a USB 3 port"))
        if s["power"] and s["power"] != "on":
            rows.append((None, f"Sensor {i} power",
                         "autosuspend enabled — reinstall udev rules "
                         "(Setup & install)"))

    svr = hw.proc_running("vrserver")
    rows.append((not svr, "SteamVR",
                 "running — close it, the optical step needs the headset"
                 if svr else "closed"))
    return rows


def preflight_ok(rows):
    return not any(ok is False for ok, _, _ in rows)


def optical_check(set_text, append, cancel, set_proc, duration=30):
    """Verify the per-session sensor-pose convergence.

    Runs a tracking session and measures how quickly the tracker
    acquires the camera pose (first non-zero fused position — the
    tracker refuses to fuse optical data before the pose is solved) and
    how solid the optical lock stays. Returns (ok, report_text).
    """
    if not os.path.exists(config.OPENHMD_EXAMPLE):
        return False, (f"Missing {config.OPENHMD_EXAMPLE} — build "
                       "SteamVR-OpenHMD first (Setup & install).")
    if hw.proc_running("vrserver"):
        return False, "Close SteamVR first — it owns the headset."
    if not hw.tracking_sensors():
        return False, "No tracking sensor on USB."
    if hw.hmd_hid_state() == "unbound":
        return False, "Headset HID not bound — run Fix USB first."

    sensors_before = {s["serial"]: s["node"] for s in hw.tracking_sensors()}
    set_text("OPTICAL TRACKING CHECK — capturing %d s\n\n%s\n"
             % (duration, STILL_HINT))
    t0 = time.time()
    proc = subprocess.Popen([config.OPENHMD_EXAMPLE, "0"],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    set_proc(proc)
    stats = {"pos": 0, "nz": 0, "first": None, "acq": None}

    def reader():
        for line in proc.stdout:
            # the driver logs this when it solves the camera pose
            if "Set sensor" in line and "pose from device" in line:
                stats["acq"] = stats["acq"] or time.time() - t0
            elif line.startswith("position vec:"):
                stats["pos"] += 1
                try:
                    if any(float(v) for v in line.split(":", 1)[1].split()):
                        stats["nz"] += 1
                        stats["first"] = stats["first"] or time.time() - t0
                except ValueError:
                    pass
    rd = threading.Thread(target=reader, daemon=True)
    rd.start()

    try:
        while time.time() - t0 < duration:
            if cancel is not None and cancel.is_set():
                append("cancelled — analyzing what we got")
                break
            time.sleep(1)
            elapsed = int(time.time() - t0)
            if elapsed and elapsed % 5 == 0:
                lock = 100 * stats["nz"] // stats["pos"] if stats["pos"] \
                    else 0
                append(f"  …{elapsed}/{duration} s — optical lock {lock}%")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        set_proc(None)
    rd.join(timeout=5)

    lock = 100 * stats["nz"] // stats["pos"] if stats["pos"] else 0
    acquired = stats["acq"] or stats["first"]
    out = []
    sensors_after = {s["serial"]: s["node"] for s in hw.tracking_sensors()}
    if sensors_after != sensors_before:
        out.append("WARNING: the sensor re-enumerated on USB during the "
                   "capture — the\ncamera link is unstable (this alone "
                   "causes shifting in VR). Move it\nto a motherboard "
                   "USB 3 port without a hub, then rerun this check.")
    if not stats["pos"]:
        out.insert(0, "FAILED — no tracking data from the headset. "
                   "Try 'Fix USB', then rerun.")
        return False, "\n\n".join(out)
    if acquired is None:
        out.insert(0, "FAILED — the tracker never acquired an optical "
                   "pose: the camera\nnever saw the headset LEDs well "
                   "enough. Check distance (1-1.5 m),\naim, line of "
                   "sight, and strong IR sources (sunlight, halogen).")
        return False, "\n\n".join(out)
    ok = acquired <= 20 and lock >= 70
    if ok:
        out.insert(0, f"TRACKING CONVERGED ✓   pose acquired after "
                   f"{acquired:.0f} s, optical lock {lock}%\n\n"
                   "Sessions will start clean as long as the headset "
                   "faces the sensor\nfrom your marked start spot.")
    else:
        out.insert(0, f"POOR CONVERGENCE — pose acquired after "
                   f"{acquired:.0f} s, optical lock only\n{lock}%. "
                   "Keep the headset still and fully visible to the "
                   "sensor, remove\nIR reflections (mirrors, glossy "
                   "furniture), then rerun.")
    return ok, "\n\n".join(out)


def launch_steamvr():
    subprocess.Popen(["steam", "steam://rungameid/250820"],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def wait_for_steamvr(append, cancel=None, timeout=90):
    """Wait until vrserver is up (plus a grace period for the compositor)."""
    for i in range(timeout):
        if cancel is not None and cancel.is_set():
            return False
        if hw.proc_running("vrserver"):
            append("SteamVR is up — giving the compositor a few seconds…")
            time.sleep(8)
            return True
        if i and i % 15 == 0:
            append(f"  …still waiting for SteamVR ({i}/{timeout} s)")
        time.sleep(1)
    return False


def quick_room_setup(height_m, log, dry_run=False):
    """Commit a standing universe from the resting headset's pose.

    Replaces the crash-prone Room Setup app. Returns (ok, text).
    """
    if not hw.proc_running("vrserver"):
        return False, "SteamVR is not running — launch it first."
    venv_py = runtime.ensure_openvr_venv(log)
    if not venv_py:
        return False, "Could not install the python-openvr bindings."
    cmd = [venv_py, config.ROOMSETUP, "%.3f" % height_m]
    if dry_run:
        cmd.append("--dry-run")
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        return False, "room setup script timed out"
    msg = (r.stdout + r.stderr).strip()
    if r.returncode == 0:
        return True, msg
    return False, msg or {
        2: "openvr bindings missing in the venv",
        3: "could not connect to SteamVR",
    }.get(r.returncode, f"failed (code {r.returncode})")


def room_setup_script():
    p = os.path.join(config.STEAMVR_PATH,
                     "tools/steamvr_room_setup/linux64/"
                     "steamvr_room_setup.sh")
    return p if os.path.exists(p) else None


def launch_room_setup():
    """Start SteamVR Room Setup; False if the tool is missing."""
    sh = room_setup_script()
    if not sh:
        return False
    subprocess.Popen(["sh", sh], cwd=os.path.dirname(sh),
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return True


def verify():
    """Post-calibration summary as (ok, text)."""
    lines = []
    devs = runtime.vr_device_status()
    if devs is None:
        ok = True
        lines.append("(SteamVR device query unavailable — skipping the "
                     "device check)")
    else:
        hmd = any(d.get("class") == "HMD" and d.get("connected")
                  for d in devs)
        ctl = sum(1 for d in devs if d.get("class") == "Controller"
                  and d.get("connected"))
        lines.append("headset in SteamVR: " +
                     ("connected ✓" if hmd else "NOT CONNECTED"))
        lines.append(f"controllers connected: {ctl}")
        ok = hmd
    lines.append("")
    lines.append(VERIFY_HINTS)
    return ok, "\n".join(lines)
