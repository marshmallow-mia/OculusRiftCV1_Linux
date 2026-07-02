"""OpenVR runtime / driver state and SteamVR session helpers."""
import glob
import json
import os
import subprocess

from . import config, hw


def read_openvr_paths():
    try:
        with open(config.OPENVR_PATHS) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def active_runtime():
    d = read_openvr_paths()
    if not d or not d.get("runtime"):
        return "none"
    rt = d["runtime"][0]
    if "SteamVR" in rt:
        return "SteamVR"
    if "wivrn" in rt.lower() or "xrizer" in rt.lower():
        return "WiVRn"
    return rt


def wivrn_xrizer_path():
    for pat in config.WIVRN_XRIZER_GLOB:
        hits = glob.glob(pat)
        if hits:
            return hits[0]
    return None


def driver_registered():
    d = read_openvr_paths()
    return bool(d) and any(
        "SteamVR-OpenHMD" in p for p in d.get("external_drivers") or [])


def switch_runtime(target=None):
    """Set the OpenVR runtime; toggles SteamVR <-> WiVRn if no target."""
    if hw.proc_running("vrserver"):
        return False, ("Close SteamVR first — it rewrites openvrpaths.vrpath "
                       "on exit and would undo the switch.")
    d = read_openvr_paths()
    if not d:
        return False, "Cannot read openvrpaths.vrpath"
    cur = active_runtime()
    if target is None:
        target = "wivrn" if cur == "SteamVR" else "steamvr"
    if target.lower() == "steamvr":
        path, name = config.STEAMVR_PATH, "SteamVR"
    else:
        path, name = wivrn_xrizer_path(), "WiVRn"
        if not path:
            return False, "WiVRn/xrizer not found (flatpak missing?)"
    d["runtime"] = [path]
    with open(config.OPENVR_PATHS, "w") as f:
        json.dump(d, f, indent=1)
    return True, f"OpenVR runtime switched to {name}"


def room_config_mtime():
    try:
        return os.path.getmtime(config.ROOM_CONFIG)
    except OSError:
        return None


def vr_device_status(timeout=10):
    """Tracked-device list (battery, connected, …) from a running SteamVR.

    Uses the openvr bindings inside the pose-test venv via vrinfo.py.
    Returns a list of dicts, or None if unavailable.
    """
    venv_py = os.path.join(config.POSE_VENV, "bin/python")
    if not (os.path.exists(venv_py) and os.path.exists(config.VRINFO)):
        return None
    if not hw.proc_running("vrserver"):
        return None
    try:
        r = subprocess.run([venv_py, config.VRINFO],
                           capture_output=True, text=True, timeout=timeout)
        if r.returncode != 0:
            return None
        return json.loads(r.stdout)
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return None
