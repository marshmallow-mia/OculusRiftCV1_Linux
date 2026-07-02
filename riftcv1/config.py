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

# venv holding the python-openvr bindings: prefer a pre-existing one next to
# the checkout, else a stable per-user location
_venv_local = os.path.join(APP_DIR, "venv")
POSE_VENV = (_venv_local if os.path.isdir(_venv_local) else
             os.path.join(HOME, ".local/share/rift-cv1-center/venv"))
