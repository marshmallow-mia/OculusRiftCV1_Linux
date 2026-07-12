"""Paths, URLs and constants.

Every path below can be overridden, highest precedence first:
  1. environment:  RIFT_CV1_<KEY>   (e.g. RIFT_CV1_STEAMVR_OPENHMD=~/src/…)
  2. config file:  ~/.config/rift-cv1-center/config.json
  3. built-in defaults

Config file example:
  { "steamvr_openhmd": "~/src/SteamVR-OpenHMD", "ouvrt_dir": "~/src/ouvrt" }
"""
import json
import os

VERSION = "2.0"

HOME = os.path.expanduser("~")
PKG_DIR = os.path.dirname(os.path.abspath(__file__))
APP_DIR = os.path.dirname(PKG_DIR)
CONFIG_FILE = os.path.join(HOME, ".config/rift-cv1-center/config.json")

_DEFAULTS = {
    "steamvr_openhmd": os.path.join(HOME, "git/SteamVR-OpenHMD"),
    "ouvrt_dir": os.path.join(HOME, "git/ouvrt"),
    "steamvr_path": os.path.join(
        HOME, ".local/share/Steam/steamapps/common/SteamVR"),
    "steam_logs": os.path.join(HOME, ".local/share/Steam/logs"),
    "openhmd_config": os.path.join(HOME, ".config/openhmd"),
}


def _load():
    cfg = dict(_DEFAULTS)
    try:
        with open(CONFIG_FILE) as f:
            data = json.load(f)
        if isinstance(data, dict):
            for k in cfg:
                if isinstance(data.get(k), str):
                    cfg[k] = os.path.expanduser(data[k])
    except (OSError, ValueError):
        pass
    for k in cfg:
        v = os.environ.get("RIFT_CV1_" + k.upper())
        if v:
            cfg[k] = os.path.expanduser(v)
    return cfg


_cfg = _load()

STEAMVR_OPENHMD = _cfg["steamvr_openhmd"]
OUVRT_DIR = _cfg["ouvrt_dir"]
STEAMVR_PATH = _cfg["steamvr_path"]
STEAM_LOGS = _cfg["steam_logs"]
OPENHMD_CONFIG = _cfg["openhmd_config"]

OPENHMD_EXAMPLE = os.path.join(
    STEAMVR_OPENHMD, "build/subprojects/openhmd/openhmd_simple_example")
OUVRTD = os.path.join(OUVRT_DIR, "build/src/ouvrtd")
ROOM_CONFIG = os.path.join(OPENHMD_CONFIG, "rift-room-config.json")

STEAMVR_OPENHMD_REPO = "https://github.com/ChristophHaag/SteamVR-OpenHMD.git"
OPENHMD_REPO = "https://github.com/thaytan/OpenHMD.git"
OPENHMD_BRANCH = "rift-kalman-filter"
OUVRT_REPO = "https://github.com/pH5/ouvrt.git"

# upstream commits the vendored driver patches (patches/) apply to;
# Setup & install checks these out on fresh clones before patching
OPENHMD_COMMIT = "04f5276bfc679968ceea62e4d1df6cbe6376941c"
STEAMVR_OPENHMD_COMMIT = "55e266814b2da82bc33774dc781b6b59709766a3"
PATCH_DIR = os.path.join(APP_DIR, "patches")

# small persistent app state (remembered wizard inputs etc.), separate
# from the read-only path config above
STATE_FILE = os.path.join(HOME, ".config/rift-cv1-center/state.json")


def state_get(key, default=None):
    try:
        with open(STATE_FILE) as f:
            data = json.load(f)
        return data.get(key, default) if isinstance(data, dict) else default
    except (OSError, ValueError):
        return default


def state_set(key, value):
    try:
        with open(STATE_FILE) as f:
            data = json.load(f)
        if not isinstance(data, dict):
            data = {}
    except (OSError, ValueError):
        data = {}
    data[key] = value
    os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
    tmp = STATE_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=1)
    os.replace(tmp, STATE_FILE)


OPENVR_PATHS = os.path.join(HOME, ".config/openvr/openvrpaths.vrpath")
WIVRN_XRIZER_GLOB = (
    "/var/lib/flatpak/app/io.github.wivrn.wivrn/current/active/files/xrizer",
    "/var/lib/flatpak/app/io.github.wivrn.wivrn/x86_64/*/*/files/xrizer",
)

DESKTOP_FILE = os.path.join(
    HOME, ".local/share/applications/rift-cv1-center.desktop")
ENTRY_POINT = os.path.join(APP_DIR, "rift_cv1_center.py")
POSE_TEST = os.path.join(PKG_DIR, "pose_test.py")
VRINFO = os.path.join(PKG_DIR, "vrinfo.py")
ROOMSETUP = os.path.join(PKG_DIR, "roomsetup.py")
CAL_TOOL = os.path.join(APP_DIR, "calibrate_room.py")
CAL_CAPTURE_DEFAULT = os.path.join(HOME, "cal-capture.jsonl")
CAL_CAPTURE_ENV = "OHMD_RIFT_CAL_CAPTURE"

# short per-phase captures written by the Sensor Setup wizard
SETUP_CACHE_DIR = os.path.join(HOME, ".cache/rift-cv1-center")
SETUP_HOLD_CAPTURE = os.path.join(SETUP_CACHE_DIR, "sensor-setup-hold.jsonl")
SETUP_FLOOR_CAPTURE = os.path.join(SETUP_CACHE_DIR, "sensor-setup-floor.jsonl")

# venv holding the python-openvr bindings: prefer a pre-existing one next to
# the checkout, else a stable per-user location
_venv_local = os.path.join(APP_DIR, "venv")
POSE_VENV = (_venv_local if os.path.isdir(_venv_local) else
             os.path.join(HOME, ".local/share/rift-cv1-center/venv"))
