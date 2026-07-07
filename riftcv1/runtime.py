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


def driver_deploy_state():
    """Whether the driver copy SteamVR loads matches the built one.

    ninja only refreshes build/driver_openhmd.so; SteamVR loads
    build/bin/linux64/driver_openhmd.so, which install_files_to_build.sh
    copies. A rebuild without that step leaves SteamVR on a stale driver.
    Returns True (fresh), False (stale) or None (not built/deployed).
    """
    built = os.path.join(config.STEAMVR_OPENHMD, "build",
                         "driver_openhmd.so")
    deployed = os.path.join(config.STEAMVR_OPENHMD, "build",
                            "bin", "linux64", "driver_openhmd.so")
    if not os.path.exists(built) or not os.path.exists(deployed):
        return None
    import hashlib

    def digest(path):
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        return h.digest()

    try:
        return digest(built) == digest(deployed)
    except OSError:
        return None


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


def ensure_openvr_venv(log=None):
    """Path to the venv python with `openvr` installed, creating the venv
    on first use. None on failure."""
    venv_py = os.path.join(config.POSE_VENV, "bin/python")
    if os.path.exists(venv_py) and subprocess.run(
            [venv_py, "-c", "import openvr"],
            capture_output=True).returncode == 0:
        return venv_py
    if log:
        log("One-time setup: installing the python-openvr bindings…")
    r = subprocess.run(["python3", "-m", "venv", config.POSE_VENV],
                       capture_output=True, text=True)
    if r.returncode != 0:
        if log:
            log("venv creation failed: " + r.stderr.strip())
        return None
    r = subprocess.run([venv_py, "-m", "pip", "install", "--quiet",
                        "openvr"], capture_output=True, text=True)
    if r.returncode != 0:
        if log:
            log("pip install openvr failed: " + r.stderr.strip())
        return None
    return venv_py


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
