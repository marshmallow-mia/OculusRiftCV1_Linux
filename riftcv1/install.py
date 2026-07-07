"""Build / install steps for the OpenHMD stack.

Functions take `run(cmd, cwd=None) -> bool` (a command runner that logs
output somewhere) and/or `log(msg)`, so the GUI and CLI can both drive them.
"""
import glob
import os
import shutil
import subprocess
import time

from . import config, hw, runtime

UDEV_RULE_FILE = "/etc/udev/rules.d/70-oculus-rift.rules"
UDEV_RULES = """\
# Oculus Rift CV1 — headset, sensors, remote (OpenHMD / ouvrt)
# power/control=on disables USB autosuspend: a suspended tracking camera
# drops off the bus mid-session (position jumps / drift in VR).
SUBSYSTEM=="usb", ATTR{idVendor}=="2833", MODE="0666", TAG+="uaccess", ATTR{power/control}="on"
KERNEL=="hidraw*", ATTRS{idVendor}=="2833", MODE="0666", TAG+="uaccess"
"""
PACMAN_HINT = ("sudo pacman -S --needed git meson ninja gcc pkgconf "
               "hidapi libusb glib2")
APT_HINT = ("sudo apt install git meson ninja-build gcc pkg-config "
            "libhidapi-dev libusb-1.0-0-dev libglib2.0-dev")
DNF_HINT = ("sudo dnf install git meson ninja-build gcc pkgconf "
            "hidapi-devel libusb1-devel glib2-devel")


def deps_hint():
    """Install command for the build dependencies on this distro."""
    if shutil.which("pacman"):
        return PACMAN_HINT
    if shutil.which("apt"):
        return APT_HINT
    if shutil.which("dnf"):
        return DNF_HINT
    return ("install (dev packages): git meson ninja gcc pkg-config "
            "hidapi libusb-1.0 glib2")


def _scan_udev_rules():
    for d in ("/etc/udev/rules.d", "/usr/lib/udev/rules.d"):
        for f in glob.glob(d + "/*.rules"):
            try:
                with open(f) as fh:
                    yield fh.read()
            except OSError:
                pass


def udev_rules_present():
    return any("2833" in txt for txt in _scan_udev_rules())


def udev_power_rule_present():
    """True if some rule also disables autosuspend for Oculus devices."""
    return any("2833" in txt and "power/control" in txt
               for txt in _scan_udev_rules())


def check_deps():
    missing = [exe for exe in ("git", "meson", "ninja", "pkg-config")
               if not shutil.which(exe)]
    if not (shutil.which("cc") or shutil.which("gcc")):
        missing.append("gcc")
    if shutil.which("pkg-config"):
        for pkg, label in (("libusb-1.0", "libusb"),
                           ("gio-unix-2.0", "glib2")):
            if subprocess.run(["pkg-config", "--exists", pkg],
                              capture_output=True).returncode:
                missing.append(label)
        if (subprocess.run(["pkg-config", "--exists", "hidapi-libusb"],
                           capture_output=True).returncode and
                subprocess.run(["pkg-config", "--exists", "hidapi"],
                               capture_output=True).returncode):
            missing.append("hidapi")
    return missing


def _apply_patch(run, log, repo, patch):
    """git-apply a vendored patch; True if applied or already present.

    An existing checkout that already contains the changes (e.g. the
    development machine, or a rerun of Setup) reverse-applies cleanly and
    is left untouched."""
    name = os.path.basename(patch)
    if subprocess.run(["git", "apply", "--reverse", "--check", patch],
                      cwd=repo, capture_output=True).returncode == 0:
        log(f"{name}: already applied")
        return True
    if subprocess.run(["git", "apply", "--check", patch],
                      cwd=repo, capture_output=True).returncode == 0:
        log(f"applying {name}")
        return run(["git", "apply", patch], cwd=repo)
    log(f"{name} does NOT apply — {repo} has diverged from the pinned "
        "upstream commit. Move that checkout aside and rerun Setup.")
    return False


def install_steamvr_openhmd(run, log=print):
    os.makedirs(os.path.dirname(config.STEAMVR_OPENHMD), exist_ok=True)
    if not os.path.isdir(config.STEAMVR_OPENHMD):
        if not run(["git", "clone", config.STEAMVR_OPENHMD_REPO,
                    config.STEAMVR_OPENHMD]):
            return False
        if not run(["git", "checkout", "--detach",
                    config.STEAMVR_OPENHMD_COMMIT],
                   cwd=config.STEAMVR_OPENHMD):
            return False
    sub = os.path.join(config.STEAMVR_OPENHMD, "subprojects/openhmd")
    if not os.path.exists(os.path.join(sub, "meson.build")):
        if not run(["git", "clone", "-b", config.OPENHMD_BRANCH,
                    config.OPENHMD_REPO, sub]):
            return False
        if not run(["git", "checkout", "--detach", config.OPENHMD_COMMIT],
                   cwd=sub):
            return False
    # the vendored driver patches (see patches/README.md) carry the room
    # config, calibration capture hook and OVR fusion the app depends on
    if not _apply_patch(run, log, config.STEAMVR_OPENHMD, os.path.join(
            config.PATCH_DIR, "steamvr-openhmd-driver.patch")):
        return False
    if not _apply_patch(run, log, sub, os.path.join(
            config.PATCH_DIR, "openhmd-rift-room-config.patch")):
        return False
    if not os.path.isdir(os.path.join(config.STEAMVR_OPENHMD, "build")):
        if not run(["meson", "setup", "build"], cwd=config.STEAMVR_OPENHMD):
            return False
    if not run(["ninja", "-C", "build"], cwd=config.STEAMVR_OPENHMD):
        return False
    return run(["./install_files_to_build.sh"], cwd=config.STEAMVR_OPENHMD)


def install_ouvrt(run):
    os.makedirs(os.path.dirname(config.OUVRT_DIR), exist_ok=True)
    if not os.path.isdir(config.OUVRT_DIR):
        if not run(["git", "clone", config.OUVRT_REPO, config.OUVRT_DIR]):
            return False
    if not os.path.isdir(os.path.join(config.OUVRT_DIR, "build")):
        if not run(["meson", "setup", "build",
                    "-Dgstreamer=false", "-Dopencv=false",
                    "-Dpipewire=false"], cwd=config.OUVRT_DIR):
            return False
    return run(["ninja", "-C", "build"], cwd=config.OUVRT_DIR)


def install_udev(run, log):
    if udev_power_rule_present():
        log("udev rules already present — skipping")
        return True
    if udev_rules_present():
        log("Existing udev rules lack the no-autosuspend fix — adding "
            + UDEV_RULE_FILE)
    script = (f"cat > {UDEV_RULE_FILE} << 'EOF'\n{UDEV_RULES}EOF\n"
              "udevadm control --reload && udevadm trigger")
    if shutil.which("pkexec"):
        log("Installing udev rules (authentication dialog will appear)…")
        return run(["pkexec", "sh", "-c", script])
    log("pkexec not found — run this as root:\n" + script)
    return False


def register_driver(run, log):
    if hw.proc_running("vrserver"):
        log("Close SteamVR before registering the driver.")
        return False
    reg = os.path.join(config.STEAMVR_OPENHMD, "register.sh")
    if not os.path.exists(reg):
        log("register.sh not found — build SteamVR-OpenHMD first")
        return False
    ok = run([reg], cwd=config.STEAMVR_OPENHMD)
    # vrpathreg spawns a vrmonitor prompt that lingers headless
    time.sleep(2)
    subprocess.run(["pkill", "-f", "vrmonitor://"], capture_output=True)
    return ok


def install_desktop(log):
    content = (
        "[Desktop Entry]\nType=Application\n"
        "Name=Rift CV1 Control Center\n"
        "Comment=Status, display test, USB fix, controller pairing "
        "and SteamVR for the Oculus Rift CV1\n"
        f"Exec=python3 {config.ENTRY_POINT}\n"
        "Icon=preferences-desktop-display\nTerminal=false\n"
        "Categories=Settings;HardwareSettings;Game;\n"
        "Keywords=VR;Oculus;Rift;SteamVR;OpenHMD;\n")
    os.makedirs(os.path.dirname(config.DESKTOP_FILE), exist_ok=True)
    with open(config.DESKTOP_FILE, "w") as f:
        f.write(content)
    subprocess.run(["update-desktop-database",
                    os.path.dirname(config.DESKTOP_FILE)],
                   capture_output=True)
    log("Desktop menu entry installed")
    return True


def setup_state():
    missing = check_deps()
    built = os.path.exists(config.OPENHMD_EXAMPLE)
    ouvrt = os.path.exists(config.OUVRTD)
    udev = udev_rules_present()
    power = udev_power_rule_present()
    reg = runtime.driver_registered()
    desk = os.path.exists(config.DESKTOP_FILE)
    return {
        "deps": (not missing, "all present" if not missing
                 else "missing: " + ", ".join(missing)),
        "svrohmd": (built, "built" if built else "not built"),
        "ouvrt": (ouvrt, "built" if ouvrt else "not built"),
        "udev": (True if power else (None if udev else False),
                 "present" if power else
                 ("no autosuspend fix — reinstall" if udev else "missing")),
        "reg": (reg, "registered" if reg else "not registered"),
        "desk": (desk, "installed" if desk else "not installed"),
    }
