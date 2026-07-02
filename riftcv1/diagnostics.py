"""One-shot diagnostic tools. Each returns human-readable text (or logs
through a callback), usable from both the GUI and the CLI."""
import glob
import json
import os
import re
import shutil
import statistics
import subprocess
import time

from . import config, hw, install, runtime


def wake_test(log):
    """Wake the headset via a short tracking session; verify the HDMI link.

    Returns True if the GPU saw the Rift's EDID.
    """
    if not os.path.exists(config.OPENHMD_EXAMPLE):
        log(f"Missing {config.OPENHMD_EXAMPLE} — build SteamVR-OpenHMD first")
        return False
    if hw.proc_running("vrserver"):
        log("SteamVR is running — it already owns the headset; close it "
            "before running the wake test.")
        return False
    if hw.usb_sysfs_device(hw.OCULUS_VID, hw.HMD_PID) and \
            not hw.hmd_hid_bound():
        log("Headset HID not bound — run 'Fix USB' first.")
        return False
    log("Waking headset (15 s)…")
    p = subprocess.Popen([config.OPENHMD_EXAMPLE, "0"],
                         stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    ok = False
    try:
        for _ in range(14):
            time.sleep(1)
            _, ovr = hw.rift_edid_connector()
            if ovr:
                ok = True
                break
    finally:
        p.terminate()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
    if ok:
        log("SUCCESS: headset display detected by the GPU (EDID: Oculus). "
            "Tracking + video link OK.")
    elif hw.hmd_hid_bound():
        log("Tracking works but no display detected — check the HDMI cable "
            "at GPU and headset ends.")
    else:
        log("Headset HID unavailable — try 'Fix USB'.")
    return ok


def tracking_quality_test(set_text, append, cancel, set_proc):
    """30 s stationary optical-tracking capture with jitter analysis.

    set_text/append: output callbacks; cancel: threading.Event;
    set_proc: receives the capture Popen so the caller can terminate it.
    """
    if not os.path.exists(config.OPENHMD_EXAMPLE):
        return (f"Missing {config.OPENHMD_EXAMPLE} — build SteamVR-OpenHMD "
                "first (Setup & install).")
    if hw.proc_running("vrserver"):
        return "Close SteamVR first — it owns the headset."
    set_text(
        "TRACKING QUALITY TEST — capturing 30 s…\n\n"
        "Keep the headset COMPLETELY STILL, 1–1.5 m from the sensor,\n"
        "front LEDs facing the camera lens. Do not touch it.\n")
    proc = subprocess.Popen([config.OPENHMD_EXAMPLE, "0"],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    set_proc(proc)
    secs = 30
    for t in range(secs):
        if cancel.is_set():
            append(f"  cancelled at {t} s — analyzing partial data")
            break
        time.sleep(1)
        if (t + 1) % 5 == 0:
            append(f"  …{t + 1}/{secs} s")
    proc.terminate()
    try:
        out, _ = proc.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        out, _ = proc.communicate()
    set_proc(None)

    pos = [tuple(map(float, m)) for m in re.findall(
        r"position vec:\s+([-\d.]+) ([-\d.]+) ([-\d.]+)", out)]
    nz = [p for p in pos if p != (0.0, 0.0, 0.0)]
    expo = len(re.findall("Exposure timestamp", out))
    errs = len(re.findall(r"\[EE\]|error", out))
    rep = ["TRACKING QUALITY REPORT\n"]
    if not pos:
        return ("TRACKING TEST FAILED — no data from the headset.\n"
                "Is it connected and HID bound? Try 'Fix USB'.")
    lockpct = 100 * len(nz) / len(pos)
    rep.append(f"samples: {len(pos)}   optical position lock: "
               f"{lockpct:.0f}%")
    if lockpct < 50:
        rep.append("\nVERDICT: POOR — the camera rarely sees the "
                   "headset LEDs.\nCheck: sensor aimed at the headset, "
                   "1–2.5 m distance, clear line of\nsight, no strong "
                   "IR sources (sunlight, incandescent lamps).")
        return "\n".join(rep)
    worst = 0.0
    tail = nz[len(nz) // 4:]
    for i, ax in enumerate("xyz"):
        vals = [p[i] for p in tail]
        std = statistics.pstdev(vals) * 1000
        rng = (max(vals) - min(vals)) * 1000
        worst = max(worst, std)
        rep.append(f"  {ax}: mean {statistics.mean(vals):+.4f} m   "
                   f"jitter {std:.1f} mm (std)   range {rng:.1f} mm")
    rep.append(f"exposure-sync warnings: {expo}   driver errors: {errs}")
    if worst < 2:
        rep.append("\nVERDICT: EXCELLENT — stable optical lock, "
                   "sub-2 mm jitter.")
    elif worst < 6:
        rep.append("\nVERDICT: OK — minor jitter. Check for IR "
                   "reflections (mirrors,\nglossy surfaces) near the "
                   "play area.")
    else:
        rep.append("\nVERDICT: UNSTABLE — try 'Reset room "
                   "calibration', check that the\nsensor cannot "
                   "wobble, and remove reflective surfaces. If it\n"
                   "persists, a second CV1 sensor helps a lot.")
    rep.append("\nIf height is wrong in VR: run SteamVR Room Setup "
               "(standing mode).")
    return "\n".join(rep)


def room_reset(log):
    path = config.ROOM_CONFIG
    if not os.path.exists(path):
        return ("No room calibration file present — a fresh one is "
                "calibrated\nautomatically during the next tracking "
                "session.")
    bak = path + ".bak-" + time.strftime("%Y%m%d-%H%M%S")
    with open(path) as f:
        content = f.read()
    os.rename(path, bak)
    log("Room calibration reset (backup: " + bak + ")")
    return ("ROOM CALIBRATION RESET\n\n"
            "The stored sensor-camera pose was removed; the tracker "
            "will\nre-calibrate it automatically next session.\n\n"
            "Do this whenever you MOVE the sensor camera — a stale "
            "pose\ncauses wrong height and position shifting. "
            "Afterwards, re-run\nSteamVR Room Setup.\n\n"
            f"Backup: {bak}\n\nOld content:\n{content}")


def room_calibration_info():
    """Summarize rift-room-config.json (tolerant of schema changes)."""
    path = config.ROOM_CONFIG
    try:
        with open(path) as f:
            raw = f.read()
        mtime = time.strftime("%Y-%m-%d %H:%M",
                              time.localtime(os.path.getmtime(path)))
    except OSError:
        return ("No room calibration file present — one is calibrated\n"
                "automatically during the next tracking session.\n\n"
                f"(would be at {path})")
    out = [f"ROOM CALIBRATION\n\nfile: {path}\nlast updated: {mtime}\n"]
    try:
        data = json.loads(raw)
    except ValueError:
        return "\n".join(out) + "\n(unparseable JSON)\n\n" + raw

    vecs = []

    def walk(obj, key):
        if isinstance(obj, dict):
            for k, v in obj.items():
                walk(v, f"{key}.{k}" if key else k)
        elif isinstance(obj, list):
            if len(obj) in (3, 4) and obj and all(
                    isinstance(x, (int, float)) and not isinstance(x, bool)
                    for x in obj):
                vecs.append((key, obj))
            else:
                for i, v in enumerate(obj):
                    walk(v, f"{key}[{i}]")

    walk(data, "")
    pos3 = [(k, v) for k, v in vecs if len(v) == 3]
    if pos3:
        out.append("stored vectors (x y z — y is height above the "
                   "tracking origin):")
        for k, v in pos3:
            out.append("  %-40s %+9.3f %+9.3f %+9.3f" % (k, *v))
        out.append("")
    out.append("If height or position is wrong in VR: leave the sensor "
               "where it is,\nrun 'Reset room calibration', start SteamVR "
               "once with the headset\nfacing the sensor, then run SteamVR "
               "Room Setup (Standing).\n")
    out.append("raw config:\n" + json.dumps(data, indent=2))
    return "\n".join(out)


def device_list():
    if not os.path.exists(config.OPENHMD_EXAMPLE):
        return (f"Missing {config.OPENHMD_EXAMPLE} — build SteamVR-OpenHMD "
                "first (Setup & install).")
    try:
        r = subprocess.run([config.OPENHMD_EXAMPLE, "999"],
                           capture_output=True, text=True, timeout=10)
        out = r.stdout
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or b"").decode(errors="replace")
    except OSError as e:
        return f"failed to run openhmd_simple_example: {e}"
    keep = []
    for line in out.splitlines():
        if line.startswith(("num devices", "device ")) or \
                line.startswith(("  vendor", "  product", "  path",
                                 "  class")):
            keep.append(line)
    return "OPENHMD DEVICE LIST\n\n" + "\n".join(keep)


def boot_mode_text():
    mode, node = hw.hmd_boot_mode()
    if node is None:
        return "Headset HID not found (asleep, unplugged, or wedged)."
    if mode is None:
        return f"Found {node} but failed to read boot report."
    name = hw.BOOT_MODES.get(mode, f"unknown ({mode})")
    extra = ""
    if mode == 2:
        extra = ("\n\nWARNING: headset is stuck in radio-pairing boot "
                 "mode.\nUse 'Reboot headset (normal mode)' to fix.")
    return f"HEADSET BOOT MODE\n\nnode: {node}\nmode: {name}{extra}"


def _sensor_link_text(s):
    spd = s["speed"]
    if spd is None:
        return "unknown link speed"
    if spd >= 5000:
        return f"USB 3.x ({spd} Mbps)"
    return f"USB 2 ({spd} Mbps) — use a USB 3 port!"


def headset_info():
    info = hw.hmd_usb_info()
    out = ["HEADSET INFO\n"]
    if not info:
        out.append("Headset not found on USB.")
    else:
        out.append("product        : %s %s" % (info.get("manufacturer") or
                                               "-", info.get("product") or
                                               "-"))
        out.append("serial         : %s" % (info.get("serial") or "-"))
        out.append("firmware (USB) : bcdDevice %s" %
                   (info.get("bcdDevice") or "-"))
        out.append("USB link       : usb %s, %s Mbps" %
                   (info.get("version") or "-", info.get("speed") or "-"))
        out.append("device node    : %s" % info["node"])
        mode, hidnode = hw.hmd_boot_mode()
        out.append("HID node       : %s" % (hidnode or "not bound"))
        if mode is not None:
            out.append("boot mode      : %s" %
                       hw.BOOT_MODES.get(mode, str(mode)))
    sensors = hw.tracking_sensors()
    out.append(f"\nTRACKING SENSORS: {len(sensors)}")
    for i, s in enumerate(sensors, 1):
        out.append(f"  sensor {i}: serial {s['serial'] or '-'} — "
                   + _sensor_link_text(s))
    return "\n".join(out)


def edid_info():
    out = ["DISPLAY / EDID INFO\n"]
    for c in sorted(glob.glob("/sys/class/drm/card*-HDMI-*") +
                    glob.glob("/sys/class/drm/card*-DP-*")):
        name = os.path.basename(c)
        try:
            with open(c + "/status") as f:
                status = f.read().strip()
            with open(c + "/enabled") as f:
                enabled = f.read().strip()
            with open(c + "/edid", "rb") as f:
                edid = f.read()
            with open(c + "/modes") as f:
                modes = f.read().split()
        except OSError:
            continue
        out.append(f"{name}: {status}, {enabled}, EDID {len(edid)} "
                   f"bytes, modes: {', '.join(modes[:4]) or '-'}")
        if len(edid) >= 10 and edid[8] == 0x3E and edid[9] == 0xD2:
            out.append("  ^ manufacturer OVR (Oculus) — this is the Rift\n")
            out.append(hw.hexdump(edid[:128]))
    out.append("\nNote: the Rift asserts hotplug only while a VR "
               "driver holds it awake.")
    return "\n".join(out)


def usb_topology():
    try:
        r = subprocess.run(["lsusb"], capture_output=True, text=True)
        lines = [ln for ln in r.stdout.splitlines() if "2833" in ln]
    except OSError:
        lines = []
    out = ["OCULUS USB DEVICES\n"] + (lines or ["none found"])
    for i, s in enumerate(hw.tracking_sensors(), 1):
        out.append(f"\nSensor {i} (serial {s['serial'] or '-'}): "
                   + _sensor_link_text(s))
    mode, _ = hw.hmd_boot_mode()
    if mode is not None:
        out.append("Headset boot mode: " + hw.BOOT_MODES.get(mode, str(mode)))
    return "\n".join(out)


def force_probe():
    script = ("for c in /sys/class/drm/card*-HDMI-*/status; do "
              "echo detect > $c; done")
    if not shutil.which("pkexec"):
        return "pkexec not found — run as root:\n" + script
    r = subprocess.run(["pkexec", "sh", "-c", script],
                       capture_output=True, text=True)
    if r.returncode != 0:
        return "Re-probe failed:\n" + (r.stderr or r.stdout)
    time.sleep(2)
    return "Connector re-probe done.\n\n" + edid_info()


def kernel_log():
    r = subprocess.run(["journalctl", "-k", "-b", "-n", "600",
                        "--no-pager"], capture_output=True, text=True)
    if r.returncode != 0:
        return "journalctl failed:\n" + r.stderr
    pat = re.compile(r"usb|2833|hdmi|edid|hidraw|xhci|drm", re.IGNORECASE)
    lines = [ln for ln in r.stdout.splitlines() if pat.search(ln)]
    return ("KERNEL USB/DRM EVENTS (latest last)\n\n" +
            "\n".join(lines[-40:]))


def steamvr_logs():
    out = ["STEAMVR LOG ERRORS/WARNINGS (latest last)\n"]
    for name in ("vrserver.txt", "vrcompositor.txt"):
        path = os.path.join(config.STEAM_LOGS, name)
        try:
            with open(path, errors="replace") as f:
                lines = f.readlines()
        except OSError:
            out.append(f"--- {name}: not found")
            continue
        hits = [ln.rstrip() for ln in lines
                if "[Error]" in ln or "[Warning]" in ln]
        out.append(f"--- {name} ---")
        out.extend(hits[-12:] or ["(no errors)"])
        out.append("    last line: " +
                   (lines[-1].strip() if lines else "-"))
    return "\n".join(out)


def export_report():
    """Write the full diagnostics report; returns (path, summary_text)."""
    path = os.path.join(
        config.HOME, "rift-cv1-diagnostics-%s.txt" %
        time.strftime("%Y%m%d-%H%M%S"))
    sections = [
        ("SETUP STATE", "\n".join(
            f"{k}: {v[1]}" for k, v in install.setup_state().items())),
        ("OPENVR RUNTIME", runtime.active_runtime()),
        ("HEADSET", headset_info()),
        ("USB", usb_topology()),
        ("DEVICES", device_list()),
        ("DISPLAY", edid_info()),
        ("ROOM CALIBRATION", room_calibration_info()),
        ("KERNEL EVENTS", kernel_log()),
        ("STEAMVR LOGS", steamvr_logs()),
    ]
    with open(path, "w") as f:
        f.write("Rift CV1 diagnostics — " + time.ctime() + "\n\n")
        for title, body in sections:
            f.write(f"{'=' * 60}\n{title}\n{'=' * 60}\n{body}\n\n")
    return path, f"Diagnostics report written to:\n{path}"
