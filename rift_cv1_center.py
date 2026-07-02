#!/usr/bin/env python3
"""Rift CV1 Control Center — manage an Oculus Rift CV1 on Linux/SteamVR.

Wraps the OpenHMD (rift-kalman-filter) + SteamVR-OpenHMD stack:
  * status of headset USB/HID, tracking camera, HDMI link, driver, runtime
  * wake test (verifies the display link end-to-end)
  * USB reset for the "HID not bound after headset reboot" wedge
  * Touch controller pairing via ouvrt (reboots headset radio into
    pairing mode, bonds controllers, reboots back)
  * OpenVR runtime switching (SteamVR <-> WiVRn) and SteamVR launch
"""
import fcntl
import glob
import json
import os
import re
import shutil
import struct
import subprocess
import threading
import time

import gi
gi.require_version("Gtk", "4.0")
gi.require_version("Adw", "1")
from gi.repository import Adw, Gdk, Gio, GLib, Gtk

HOME = os.path.expanduser("~")
STEAMVR_OPENHMD = os.path.join(HOME, "git/SteamVR-OpenHMD")
OPENHMD_EXAMPLE = os.path.join(
    STEAMVR_OPENHMD, "build/subprojects/openhmd/openhmd_simple_example")
OUVRT_DIR = os.path.join(HOME, "git/ouvrt")
OUVRTD = os.path.join(OUVRT_DIR, "build/src/ouvrtd")
STEAMVR_OPENHMD_REPO = "https://github.com/ChristophHaag/SteamVR-OpenHMD.git"
OPENHMD_REPO = "https://github.com/thaytan/OpenHMD.git"
OPENHMD_BRANCH = "rift-kalman-filter"
OUVRT_REPO = "https://github.com/pH5/ouvrt.git"
UDEV_RULE_FILE = "/etc/udev/rules.d/70-oculus-rift.rules"
UDEV_RULES = """\
# Oculus Rift CV1 — headset, sensors, remote (OpenHMD / ouvrt)
SUBSYSTEM=="usb", ATTR{idVendor}=="2833", MODE="0666", TAG+="uaccess"
KERNEL=="hidraw*", ATTRS{idVendor}=="2833", MODE="0666", TAG+="uaccess"
"""
DESKTOP_FILE = os.path.join(
    HOME, ".local/share/applications/rift-cv1-center.desktop")
APP_DIR = os.path.dirname(os.path.abspath(__file__))
POSE_VENV = os.path.join(APP_DIR, "venv")
POSE_TEST = os.path.join(APP_DIR, "pose_test.py")
PACMAN_HINT = ("sudo pacman -S --needed git meson ninja gcc pkgconf "
               "hidapi libusb glib2")
OPENVR_PATHS = os.path.join(HOME, ".config/openvr/openvrpaths.vrpath")
STEAMVR_PATH = os.path.join(
    HOME, ".local/share/Steam/steamapps/common/SteamVR")
WIVRN_XRIZER_GLOB = (
    "/var/lib/flatpak/app/io.github.wivrn.wivrn/current/active/files/xrizer",
    "/var/lib/flatpak/app/io.github.wivrn.wivrn/x86_64/*/*/files/xrizer",
)
USBDEVFS_RESET = 21780
OCULUS_VID, HMD_PID, CAMERA_PID = "2833", "0031", "0211"
OUVRT_BUS = "de.phfuenf.ouvrt.Ouvrtd"
OUVRT_ROOT = "/de/phfuenf/ouvrt"
RADIO_IFACE = "de.phfuenf.ouvrt.Radio1"


# ---------------------------------------------------------------- helpers

def usb_sysfs_device(vid, pid):
    """Return (sysfs_path, /dev/bus/usb node) for a USB device, or None."""
    for p in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(p + "/idVendor") as f:
                if f.read().strip() != vid:
                    continue
            with open(p + "/idProduct") as f:
                if f.read().strip() != pid:
                    continue
            with open(p + "/busnum") as f:
                bus = int(f.read())
            with open(p + "/devnum") as f:
                dev = int(f.read())
            return p, "/dev/bus/usb/%03d/%03d" % (bus, dev)
        except (OSError, ValueError):
            continue
    return None


def hmd_hid_bound():
    """True if hidraw nodes exist for the headset HID interfaces."""
    for h in glob.glob("/sys/class/hidraw/hidraw*/device/uevent"):
        try:
            with open(h) as f:
                if "00002833:00000031" in f.read():
                    return True
        except OSError:
            pass
    return False


def rift_edid_connector():
    """Return (connector_name, has_ovr_edid). Detects the Rift's display."""
    best = None
    for c in glob.glob("/sys/class/drm/card*-HDMI-*") + \
            glob.glob("/sys/class/drm/card*-DP-*"):
        try:
            with open(c + "/edid", "rb") as f:
                edid = f.read()
        except OSError:
            continue
        # EDID manufacturer id "OVR" encodes as 0x3E 0xD2
        if len(edid) >= 10 and edid[8] == 0x3E and edid[9] == 0xD2:
            return os.path.basename(c), True
        if "HDMI" in c and best is None:
            best = os.path.basename(c)
    return best, False


def read_openvr_paths():
    try:
        with open(OPENVR_PATHS) as f:
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
    for pat in WIVRN_XRIZER_GLOB:
        hits = glob.glob(pat)
        if hits:
            return hits[0]
    return None


def driver_registered():
    d = read_openvr_paths()
    return bool(d) and any(
        "SteamVR-OpenHMD" in p for p in d.get("external_drivers") or [])


def udev_rules_present():
    for d in ("/etc/udev/rules.d", "/usr/lib/udev/rules.d"):
        for f in glob.glob(d + "/*.rules"):
            try:
                with open(f) as fh:
                    if "2833" in fh.read():
                        return True
            except OSError:
                pass
    return False


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


def find_hmd_hidraw():
    """Find the headset's IMU HID node (interface 0)."""
    for h in sorted(glob.glob("/sys/class/hidraw/hidraw*")):
        try:
            with open(h + "/device/uevent") as f:
                data = f.read()
            if "00002833:00000031" in data and "input0" in data:
                return "/dev/" + os.path.basename(h)
        except OSError:
            pass
    return None


def hid_get_feature(path, report_id, length):
    import array
    buf = array.array("B", [0] * length)
    buf[0] = report_id
    ioc = (3 << 30) | (length << 16) | (ord("H") << 8) | 0x07
    fd = os.open(path, os.O_RDWR)
    try:
        fcntl.ioctl(fd, ioc, buf, True)
        return bytes(buf)
    finally:
        os.close(fd)


def hid_set_feature(path, payload):
    ioc = (3 << 30) | (len(payload) << 16) | (ord("H") << 8) | 0x06
    fd = os.open(path, os.O_RDWR)
    try:
        fcntl.ioctl(fd, ioc, bytes(payload))
    finally:
        os.close(fd)


BOOT_MODES = {0: "normal", 1: "bootloader", 2: "radio pairing"}


def hmd_boot_mode():
    """Read the CV1 bootload feature report (id 0x06). None if no HMD."""
    node = find_hmd_hidraw()
    if not node:
        return None, None
    try:
        rep = hid_get_feature(node, 0x06, 4)
        return rep[3], node
    except OSError:
        return None, node


def hexdump(b):
    lines = []
    for i in range(0, len(b), 16):
        chunk = b[i:i + 16]
        lines.append("%04x  %s" % (i, " ".join(f"{x:02x}" for x in chunk)))
    return "\n".join(lines)


def pgrep(name):
    return subprocess.run(["pgrep", "-x", name],
                          capture_output=True).returncode == 0


def gdbus_call(objpath, method):
    return subprocess.run(
        ["gdbus", "call", "--session", "-d", OUVRT_BUS,
         "-o", objpath, "-m", RADIO_IFACE + "." + method],
        capture_output=True, text=True, timeout=15)


def find_radio_object():
    """Find the ouvrt DBus object that exposes the Radio1 interface."""
    out = subprocess.run(
        ["busctl", "--user", "tree", OUVRT_BUS],
        capture_output=True, text=True).stdout
    for dev in re.findall(r"dev_\d+", out):
        objpath = f"{OUVRT_ROOT}/{dev}"
        intro = subprocess.run(
            ["busctl", "--user", "introspect", OUVRT_BUS, objpath],
            capture_output=True, text=True).stdout
        if "Radio1" in intro:
            return objpath
    return None


USBDEVFS_IOCTL = (3 << 30) | (16 << 16) | (0x55 << 8) | 18
USBDEVFS_CONNECT = (0x55 << 8) | 23


def usb_reattach_hid():
    """Ask the kernel to reattach drivers to the headset HID interfaces.

    Gentler than a reset: fixes the 'interfaces left with NO-DRIVER after
    the headset reboots itself' wedge without restarting the device.
    """
    found = usb_sysfs_device(OCULUS_VID, HMD_PID)
    if not found:
        return False, "Headset not found on USB"
    _, node = found
    try:
        fd = os.open(node, os.O_WRONLY)
        try:
            for ifno in (0, 1):
                fcntl.ioctl(fd, USBDEVFS_IOCTL,
                            struct.pack("iiP", ifno, USBDEVFS_CONNECT, 0))
        finally:
            os.close(fd)
        return True, f"driver reattach requested ({node})"
    except OSError as e:
        return False, f"driver reattach failed: {e}"


def usb_reset_hmd():
    """Reset the headset USB device (fixes 'HID not bound' wedge)."""
    found = usb_sysfs_device(OCULUS_VID, HMD_PID)
    if not found:
        return False, "Headset not found on USB"
    _, node = found
    try:
        with open(node, "wb") as f:
            fcntl.ioctl(f, USBDEVFS_RESET, 0)
        return True, f"USB reset OK ({node})"
    except OSError as e:
        return False, f"USB reset failed: {e}"


def ensure_hid_bound(log, tries=3):
    """Wait for headset HID; reattach drivers / reset if not bound."""
    for i in range(tries):
        for _ in range(6):
            if hmd_hid_bound():
                return True
            time.sleep(1)
        ok, msg = usb_reattach_hid()
        log(f"HID not bound, reattaching drivers (attempt {i + 1}): {msg}")
        time.sleep(2)
        if hmd_hid_bound():
            return True
        ok, msg = usb_reset_hmd()
        log(f"still not bound, resetting USB: {msg}")
        time.sleep(3)
    return hmd_hid_bound()


# ---------------------------------------------------------------- GUI

class StatusRow(Gtk.Box):
    """Compact one-line status row: state icon, name, value."""
    ICONS = {True: ("emblem-ok-symbolic", "success"),
             False: ("dialog-error-symbolic", "error"),
             None: ("dialog-warning-symbolic", "warning")}

    def __init__(self, label):
        super().__init__(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.set_margin_top(3)
        self.set_margin_bottom(3)
        self.set_margin_start(10)
        self.set_margin_end(10)
        self.icon = Gtk.Image.new_from_icon_name("content-loading-symbolic")
        self.name = Gtk.Label(label=label, xalign=0, hexpand=True)
        self.value = Gtk.Label(label="…", xalign=1)
        self.value.add_css_class("dim-label")
        self.value.set_wrap(True)
        self.append(self.icon)
        self.append(self.name)
        self.append(self.value)

    def set(self, ok, text):
        name, cls = self.ICONS[ok]
        self.icon.set_from_icon_name(name)
        for c in ("success", "error", "warning"):
            self.icon.remove_css_class(c)
        self.icon.add_css_class(cls)
        self.value.set_text(text)


class App(Adw.Application):
    def __init__(self):
        super().__init__(application_id="de.local.RiftCV1Center")
        self.ouvrtd_proc = None
        self.busy = False
        self._last_autofix = 0.0
        self.adv_cancel = threading.Event()
        self.adv_proc = None
        self.adv_live_proc = None

    # ---------------- window
    def do_activate(self):
        win = Adw.ApplicationWindow(application=self,
                                    title="Rift CV1 Control Center")
        win.set_default_size(640, 800)
        self.win = win

        tb = Adw.ToolbarView()
        header = Adw.HeaderBar()
        header.set_title_widget(Adw.WindowTitle(
            title="Rift CV1", subtitle="Control Center"))
        menu = Gio.Menu()
        menu.append("Auto-fix USB wedge", "app.autofix")
        menu.append("Re-register driver", "app.register")
        menu.append("Open OpenHMD config folder", "app.open-config")
        menu.append("Open SteamVR logs folder", "app.open-logs")
        menu.append("Export diagnostics report", "app.export")
        menu.append("About", "app.about")
        mb = Gtk.MenuButton(icon_name="open-menu-symbolic",
                            menu_model=menu)
        header.pack_end(mb)
        tb.add_top_bar(header)

        self.autofix = Gio.SimpleAction.new_stateful(
            "autofix", None, GLib.Variant.new_boolean(True))
        self.autofix.connect("change-state",
                             lambda a, v: a.set_state(v))
        self.add_action(self.autofix)
        for name, cb in [
                ("register", lambda *_: self.on_register(None)),
                ("open-config", lambda *_: self.open_folder(
                    os.path.join(HOME, ".config/openhmd"))),
                ("open-logs", lambda *_: self.open_folder(
                    os.path.join(HOME, ".local/share/Steam/logs"))),
                ("export", lambda *_: self.run_async(
                    lambda: (self.adv_export(),
                             self.toast("Diagnostics report saved to your "
                                        "home folder")))),
                ("about", lambda *_: self.on_about())]:
            a = Gio.SimpleAction.new(name, None)
            a.connect("activate", cb)
            self.add_action(a)

        self.toaster = Adw.ToastOverlay()
        tb.set_content(self.toaster)
        win.set_content(tb)

        scroll = Gtk.ScrolledWindow()
        scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        self.toaster.set_child(scroll)
        clamp = Adw.Clamp(maximum_size=640)
        scroll.set_child(clamp)
        outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        for m in (outer.set_margin_top, outer.set_margin_bottom,
                  outer.set_margin_start, outer.set_margin_end):
            m(12)
        clamp.set_child(outer)

        # smart suggestion banner
        self.banner = Adw.Banner()
        self.banner_action = None
        self.banner.connect(
            "button-clicked",
            lambda *_: self.banner_action and self.banner_action(None))
        outer.append(self.banner)

        # status card — compact, everything visible at a glance
        card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        card.add_css_class("card")
        pad = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
        for m in (pad.set_margin_top, pad.set_margin_bottom):
            m(8)
        card.append(pad)
        outer.append(card)
        self.rows = {}
        for key, label in [
                ("usb", "Headset USB"),
                ("hid", "Headset HID (driver access)"),
                ("cam", "Tracking camera"),
                ("disp", "Display link (HDMI)"),
                ("room", "Room calibration"),
                ("drv", "SteamVR driver (OpenHMD)"),
                ("rt", "OpenVR runtime"),
                ("svr", "SteamVR")]:
            row = StatusRow(label)
            self.rows[key] = row
            pad.append(row)

        # actions
        grid = Gtk.Grid(column_spacing=8, row_spacing=8)
        grid.set_column_homogeneous(True)
        outer.append(grid)

        def button(label, icon, cb, cls=None):
            b = Gtk.Button()
            b.set_child(Adw.ButtonContent(label=label, icon_name=icon,
                                          halign=Gtk.Align.CENTER))
            b.connect("clicked", cb)
            if cls:
                b.add_css_class(cls)
            return b

        self.btn_steamvr = button("Launch SteamVR",
                                  "media-playback-start-symbolic",
                                  self.on_launch_steamvr,
                                  "suggested-action")
        self.btn_test = button("Test headset", "video-display-symbolic",
                               self.on_test)
        self.btn_pair = button("Pair controllers", "input-gaming-symbolic",
                               self.on_pair)
        self.btn_reset = button("Fix USB", "drive-harddisk-usb-symbolic",
                                self.on_reset)
        self.btn_rt = button("Switch runtime",
                             "emblem-synchronizing-symbolic",
                             self.on_switch_runtime)
        self.btn_setup = button("Setup & install", "emblem-system-symbolic",
                                self.on_setup)
        self.btn_adv = button("Advanced", "utilities-terminal-symbolic",
                              self.on_advanced)
        grid.attach(self.btn_steamvr, 0, 0, 2, 1)
        grid.attach(self.btn_test, 0, 1, 1, 1)
        grid.attach(self.btn_pair, 1, 1, 1, 1)
        grid.attach(self.btn_reset, 0, 2, 1, 1)
        grid.attach(self.btn_rt, 1, 2, 1, 1)
        grid.attach(self.btn_setup, 0, 3, 1, 1)
        grid.attach(self.btn_adv, 1, 3, 1, 1)
        self.btn_cam = button("Camera & placement guide",
                              "camera-video-symbolic", self.on_camera)
        grid.attach(self.btn_cam, 0, 4, 2, 1)

        # activity log
        exp = Gtk.Expander(label="Activity log")
        exp.set_expanded(True)
        self.logbuf = Gtk.TextBuffer()
        self.log_tv = Gtk.TextView(buffer=self.logbuf, editable=False,
                                   monospace=True,
                                   wrap_mode=Gtk.WrapMode.WORD_CHAR)
        for m in (self.log_tv.set_top_margin, self.log_tv.set_bottom_margin,
                  self.log_tv.set_left_margin, self.log_tv.set_right_margin):
            m(8)
        lsw = Gtk.ScrolledWindow(min_content_height=180)
        lsw.set_child(self.log_tv)
        lsw.add_css_class("card")
        exp.set_child(lsw)
        outer.append(exp)
        self.log_expander = exp

        win.connect("close-request", self.on_close)
        win.present()
        self.refresh_status()
        GLib.timeout_add_seconds(3, self.refresh_status)

    # ---------------- helpers
    def open_folder(self, path):
        os.makedirs(path, exist_ok=True)
        subprocess.Popen(["xdg-open", path],
                         stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)

    def toast(self, msg):
        def _t():
            self.toaster.add_toast(Adw.Toast.new(msg))
            return False
        GLib.idle_add(_t)

    def on_about(self):
        d = Adw.AboutDialog(
            application_name="Rift CV1 Control Center",
            application_icon="preferences-desktop-display",
            version="1.1",
            developer_name="mia",
            comments="Status, display test, USB fixes, Touch controller "
                      "pairing, tracking diagnostics and SteamVR "
                      "integration for the Oculus Rift CV1 on Linux.\n\n"
                      "Stack: SteamVR-OpenHMD + OpenHMD "
                      "(rift-kalman-filter) + ouvrt.")
        d.present(self.win)

    # ---------------- logging helper (thread-safe)
    def log(self, msg):
        def _append():
            end = self.logbuf.get_end_iter()
            self.logbuf.insert(end, time.strftime("%H:%M:%S ") + msg + "\n")
            end = self.logbuf.get_end_iter()
            self.log_tv.scroll_to_iter(end, 0.0, False, 0.0, 1.0)
            # surface activity while something is actually happening
            if self.busy:
                self.log_expander.set_expanded(True)
            return False
        GLib.idle_add(_append)

    # ---------------- status
    def refresh_status(self):
        usb = usb_sysfs_device(OCULUS_VID, HMD_PID) is not None
        self.rows["usb"].set(usb, "connected" if usb else "not found")

        hid = hmd_hid_bound()
        self.rows["hid"].set(
            hid if usb else False,
            "bound" if hid else ("NOT BOUND — use Fix USB" if usb
                                 else "—"))

        cam = usb_sysfs_device(OCULUS_VID, CAMERA_PID) is not None
        self.rows["cam"].set(cam, "connected" if cam else "not found")

        conn, ovr = rift_edid_connector()
        self.rows["disp"].set(
            True if ovr else None,
            f"Rift active on {conn}" if ovr
            else "asleep / unknown (run test)")

        rc = os.path.join(HOME, ".config/openhmd/rift-room-config.json")
        if os.path.exists(rc):
            self.rows["room"].set(True, time.strftime(
                "calibrated %d %b %H:%M",
                time.localtime(os.path.getmtime(rc))))
        else:
            self.rows["room"].set(None, "will auto-calibrate next session")

        drv = driver_registered()
        self.rows["drv"].set(drv, "registered" if drv else "not registered")

        rt = active_runtime()
        self.rows["rt"].set(rt == "SteamVR", rt)

        svr = pgrep("vrserver")
        self.rows["svr"].set(True if svr else None,
                             "running" if svr else "not running")

        # auto-fix the USB wedge (gentle driver reattach, no reset)
        if (usb and not hid and not self.busy
                and self.autofix.get_state().get_boolean()
                and time.time() - self._last_autofix > 20):
            self._last_autofix = time.time()
            self.log("Auto-fix: headset HID unbound — reattaching driver…")
            threading.Thread(target=usb_reattach_hid, daemon=True).start()

        # smart suggestion banner
        def banner(msg, btn=None, action=None):
            self.banner.set_title(msg)
            self.banner.set_button_label(btn)
            self.banner_action = action
            self.banner.set_revealed(True)

        if not usb:
            banner("Headset not detected — check its USB connection "
                   "and power")
        elif not hid:
            banner("Headset USB is wedged (no HID)", "Fix now",
                   self.on_reset)
        elif not drv:
            banner("OpenHMD driver is not registered with SteamVR",
                   "Register", self.on_register)
        elif rt != "SteamVR":
            banner(f"OpenVR runtime is set to {rt}, not SteamVR",
                   "Switch", self.on_switch_runtime)
        elif svr:
            self.banner.set_revealed(False)
        elif not cam:
            banner("No tracking camera — you would get rotation-only "
                   "tracking")
        else:
            banner("All systems go", "Launch SteamVR",
                   self.on_launch_steamvr)
        return True

    def refresh_once(self):
        """idle_add-safe wrapper: refresh exactly once (returns False)."""
        self.refresh_status()
        return False

    def run_async(self, fn):
        if self.busy:
            self.log("Busy with another operation, please wait…")
            return
        self.busy = True

        def wrapper():
            try:
                fn()
            except Exception as e:
                self.log(f"Error: {e}")
            finally:
                self.busy = False
        threading.Thread(target=wrapper, daemon=True).start()

    # ---------------- actions
    def on_test(self, _b):
        def task():
            if not os.path.exists(OPENHMD_EXAMPLE):
                self.log(f"Missing {OPENHMD_EXAMPLE} — build "
                         "SteamVR-OpenHMD first")
                return
            if usb_sysfs_device(OCULUS_VID, HMD_PID) and not hmd_hid_bound():
                self.log("Headset HID not bound — run 'Fix USB' first.")
                return
            self.log("Waking headset (15 s)…")
            p = subprocess.Popen([OPENHMD_EXAMPLE, "0"],
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
            ok = False
            try:
                for _ in range(14):
                    time.sleep(1)
                    _, ovr = rift_edid_connector()
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
                self.log("SUCCESS: headset display detected by the GPU "
                         "(EDID: Oculus). Tracking + video link OK.")
            elif hmd_hid_bound():
                self.log("Tracking works but no display detected — check "
                         "the HDMI cable at GPU and headset ends.")
            else:
                self.log("Headset HID unavailable — try 'Fix USB'.")
            GLib.idle_add(self.refresh_once)
        self.run_async(task)

    def on_reset(self, _b):
        def task():
            ok, msg = usb_reattach_hid()
            self.log(msg)
            time.sleep(2)
            if not hmd_hid_bound():
                ok, msg = usb_reset_hmd()
                self.log(msg)
                time.sleep(3)
            self.log("HID bound: %s" % hmd_hid_bound())
            GLib.idle_add(self.refresh_once)
        self.run_async(task)

    def on_register(self, _b):
        def task():
            self.register_driver()
            GLib.idle_add(self.refresh_once)
        self.run_async(task)

    # ---------------- setup / install
    def run_cmd(self, cmd, cwd=None):
        """Run a command, mirror its tail output into the log pane."""
        self.log("$ " + " ".join(cmd))
        try:
            r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
        except OSError as e:
            self.log(f"  failed to run: {e}")
            return False
        out = (r.stdout + r.stderr).strip()
        for line in out.splitlines()[-6:]:
            self.log("  " + line)
        if r.returncode != 0:
            self.log(f"  exited with code {r.returncode}")
        return r.returncode == 0

    def install_steamvr_openhmd(self):
        os.makedirs(os.path.dirname(STEAMVR_OPENHMD), exist_ok=True)
        if not os.path.isdir(STEAMVR_OPENHMD):
            if not self.run_cmd(["git", "clone", STEAMVR_OPENHMD_REPO,
                                 STEAMVR_OPENHMD]):
                return False
        sub = os.path.join(STEAMVR_OPENHMD, "subprojects/openhmd")
        if not os.path.exists(os.path.join(sub, "meson.build")):
            if not self.run_cmd(["git", "clone", "--depth=1", "-b",
                                 OPENHMD_BRANCH, OPENHMD_REPO, sub]):
                return False
        if not os.path.isdir(os.path.join(STEAMVR_OPENHMD, "build")):
            if not self.run_cmd(["meson", "setup", "build"],
                                cwd=STEAMVR_OPENHMD):
                return False
        if not self.run_cmd(["ninja", "-C", "build"], cwd=STEAMVR_OPENHMD):
            return False
        return self.run_cmd(["./install_files_to_build.sh"],
                            cwd=STEAMVR_OPENHMD)

    def install_ouvrt(self):
        os.makedirs(os.path.dirname(OUVRT_DIR), exist_ok=True)
        if not os.path.isdir(OUVRT_DIR):
            if not self.run_cmd(["git", "clone", OUVRT_REPO, OUVRT_DIR]):
                return False
        if not os.path.isdir(os.path.join(OUVRT_DIR, "build")):
            if not self.run_cmd(["meson", "setup", "build",
                                 "-Dgstreamer=false", "-Dopencv=false",
                                 "-Dpipewire=false"], cwd=OUVRT_DIR):
                return False
        return self.run_cmd(["ninja", "-C", "build"], cwd=OUVRT_DIR)

    def install_udev(self):
        if udev_rules_present():
            self.log("udev rules already present — skipping")
            return True
        script = (f"cat > {UDEV_RULE_FILE} << 'EOF'\n{UDEV_RULES}EOF\n"
                  "udevadm control --reload && udevadm trigger")
        if shutil.which("pkexec"):
            self.log("Installing udev rules (authentication dialog "
                     "will appear)…")
            return self.run_cmd(["pkexec", "sh", "-c", script])
        self.log("pkexec not found — run this as root:\n" + script)
        return False

    def register_driver(self):
        if pgrep("vrserver"):
            self.log("Close SteamVR before registering the driver.")
            return False
        reg = os.path.join(STEAMVR_OPENHMD, "register.sh")
        if not os.path.exists(reg):
            self.log("register.sh not found — build SteamVR-OpenHMD first")
            return False
        ok = self.run_cmd([reg], cwd=STEAMVR_OPENHMD)
        # vrpathreg spawns a vrmonitor prompt that lingers headless
        time.sleep(2)
        subprocess.run(["pkill", "-f", "vrmonitor://"], capture_output=True)
        return ok

    def install_desktop(self):
        content = (
            "[Desktop Entry]\nType=Application\n"
            "Name=Rift CV1 Control Center\n"
            "Comment=Status, display test, USB fix, controller pairing "
            "and SteamVR for the Oculus Rift CV1\n"
            f"Exec=python3 {os.path.abspath(__file__)}\n"
            "Icon=preferences-desktop-display\nTerminal=false\n"
            "Categories=Settings;HardwareSettings;Game;\n"
            "Keywords=VR;Oculus;Rift;SteamVR;OpenHMD;\n")
        os.makedirs(os.path.dirname(DESKTOP_FILE), exist_ok=True)
        with open(DESKTOP_FILE, "w") as f:
            f.write(content)
        subprocess.run(["update-desktop-database",
                        os.path.dirname(DESKTOP_FILE)],
                       capture_output=True)
        self.log("Desktop menu entry installed")
        return True

    def setup_state(self):
        missing = check_deps()
        built = os.path.exists(OPENHMD_EXAMPLE)
        ouvrt = os.path.exists(OUVRTD)
        udev = udev_rules_present()
        reg = driver_registered()
        desk = os.path.exists(DESKTOP_FILE)
        return {
            "deps": (not missing, "all present" if not missing
                     else "missing: " + ", ".join(missing)),
            "svrohmd": (built, "built" if built else "not built"),
            "ouvrt": (ouvrt, "built" if ouvrt else "not built"),
            "udev": (udev, "present" if udev else "missing"),
            "reg": (reg, "registered" if reg else "not registered"),
            "desk": (desk, "installed" if desk else "not installed"),
        }

    def on_setup(self, _b):
        dlg = Gtk.Window(title="Setup & Install", modal=True)
        dlg.set_transient_for(self.win)
        dlg.set_default_size(520, 380)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        for m in (box.set_margin_top, box.set_margin_bottom,
                  box.set_margin_start, box.set_margin_end):
            m(14)
        dlg.set_child(box)

        lb = Gtk.ListBox()
        lb.add_css_class("boxed-list")
        lb.set_selection_mode(Gtk.SelectionMode.NONE)
        box.append(lb)
        rows = {}
        for key, label in [
                ("deps", "Build dependencies"),
                ("svrohmd", "SteamVR-OpenHMD driver"),
                ("ouvrt", "ouvrt pairing daemon"),
                ("udev", "udev rules (device permissions)"),
                ("reg", "Driver registered with SteamVR"),
                ("desk", "Desktop menu entry")]:
            r = StatusRow(label)
            rows[key] = r
            lb.append(r)

        hint = Gtk.Label(xalign=0, wrap=True)
        hint.set_markup("<small>If dependencies are missing, install "
                        f"them first:\n<tt>{PACMAN_HINT}</tt></small>")
        box.append(hint)
        btn = Gtk.Button()
        btn.set_child(Adw.ButtonContent(
            label="Install / update everything",
            icon_name="software-update-available-symbolic",
            halign=Gtk.Align.CENTER))
        btn.add_css_class("pill")
        btn.add_css_class("suggested-action")
        box.append(btn)

        def refresh():
            for k, (ok, txt) in self.setup_state().items():
                rows[k].set(ok, txt)
            return False

        def install_all():
            missing = check_deps()
            if missing:
                self.log("Missing build dependencies: " +
                         ", ".join(missing))
                self.log("Install them first: " + PACMAN_HINT)
                GLib.idle_add(refresh)
                return
            steps = [
                ("SteamVR-OpenHMD", self.install_steamvr_openhmd),
                ("ouvrt", self.install_ouvrt),
                ("udev rules", self.install_udev),
                ("driver registration", self.register_driver),
                ("desktop entry", self.install_desktop),
            ]
            for name, fn in steps:
                self.log(f"--- {name} ---")
                if not fn():
                    self.log(f"{name}: FAILED (see output above); "
                             "continuing with remaining steps")
                GLib.idle_add(refresh)
            GLib.idle_add(self.refresh_once)
            self.log("Setup finished.")

        btn.connect("clicked", lambda _btn: self.run_async(install_all))
        refresh()
        dlg.present()

    def on_switch_runtime(self, _b):
        def task():
            d = read_openvr_paths()
            if not d:
                self.log("Cannot read openvrpaths.vrpath")
                return
            cur = active_runtime()
            if cur == "SteamVR":
                target = wivrn_xrizer_path()
                if not target:
                    self.log("WiVRn/xrizer not found (flatpak missing?)")
                    return
                name = "WiVRn"
            else:
                target = STEAMVR_PATH
                name = "SteamVR"
            d["runtime"] = [target]
            with open(OPENVR_PATHS, "w") as f:
                json.dump(d, f, indent=1)
            self.log(f"OpenVR runtime switched to {name}")
            GLib.idle_add(self.refresh_once)
        self.run_async(task)

    def on_launch_steamvr(self, _b):
        subprocess.Popen(["steam", "steam://rungameid/250820"],
                         stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
        self.log("Asked Steam to launch SteamVR…")

    # ---------------- advanced tools
    def adv_set(self, text):
        def _set():
            if getattr(self, "adv_win", None):
                self.adv_buf.set_text(text)
            return False
        GLib.idle_add(_set)

    def adv_append(self, text):
        def _a():
            if not getattr(self, "adv_win", None):
                return False
            end = self.adv_buf.get_end_iter()
            self.adv_buf.insert(
                end, text if text.endswith("\n") else text + "\n")
            end = self.adv_buf.get_end_iter()
            self.adv_tv.scroll_to_iter(end, 0.0, False, 0.0, 1.0)
            return False
        GLib.idle_add(_a)

    def _adv_state(self, running, msg):
        if not getattr(self, "adv_win", None):
            return False
        if running:
            self.adv_spinner.start()
        else:
            self.adv_spinner.stop()
        self.adv_status.set_text(msg)
        self.adv_cancel_btn.set_sensitive(running)
        return False

    def adv_run(self, label, fn):
        """Run an advanced tool with spinner, live status and cancel."""
        if self.busy:
            self.adv_append("Busy with another operation — wait for it "
                            "or press Cancel.")
            return
        self.busy = True
        self.adv_cancel.clear()
        self.adv_proc = None
        GLib.idle_add(self._adv_state, True, f"Running: {label}…")

        def worker():
            try:
                res = fn()
                if res:
                    self.adv_set(res)
            except Exception as e:
                self.adv_append(f"Error: {e}")
            finally:
                self.busy = False
                cancelled = self.adv_cancel.is_set()
                GLib.idle_add(self._adv_state, False,
                              "Cancelled" if cancelled else "Done")
        threading.Thread(target=worker, daemon=True).start()

    def on_adv_cancel(self, _b):
        self.adv_cancel.set()
        proc = self.adv_proc
        if proc:
            try:
                proc.terminate()
            except OSError:
                pass
        if self.adv_live_proc:
            self.adv_live_stop()
        self.log("Advanced tool cancelled")

    def on_advanced(self, _b):
        if getattr(self, "adv_win", None):
            self.adv_win.present()
            return
        win = Gtk.Window(title="Advanced — Rift CV1")
        win.set_transient_for(self.win)
        win.set_default_size(820, 560)
        self.adv_win = win
        self.adv_live_proc = None

        vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        for m in (vbox.set_margin_top, vbox.set_margin_bottom,
                  vbox.set_margin_start, vbox.set_margin_end):
            m(12)
        win.set_child(vbox)

        outer = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        outer.set_vexpand(True)
        vbox.append(outer)

        # status bar: spinner + current task + cancel
        sbar = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.adv_spinner = Gtk.Spinner()
        self.adv_status = Gtk.Label(label="Idle", xalign=0, hexpand=True)
        self.adv_status.add_css_class("dim-label")
        self.adv_cancel_btn = Gtk.Button(label="Cancel")
        self.adv_cancel_btn.add_css_class("destructive-action")
        self.adv_cancel_btn.set_sensitive(False)
        self.adv_cancel_btn.connect("clicked", self.on_adv_cancel)
        sbar.append(self.adv_spinner)
        sbar.append(self.adv_status)
        sbar.append(self.adv_cancel_btn)
        vbox.append(sbar)

        tools = [
            ("Tracking quality test (30 s, SteamVR off)",
             self.adv_tracking_test),
            ("SteamVR pose test (35 s, SteamVR on)", self.adv_pose_test),
            ("Reset room calibration", self.adv_room_reset),
            ("OpenHMD device list", self.adv_devices),
            ("Headset boot mode", self.adv_boot_mode),
            ("Reboot headset (normal mode)", self.adv_reboot_normal),
            ("Display / EDID info", self.adv_edid),
            ("USB topology", self.adv_usb),
            ("Force display re-probe (root)", self.adv_force_probe),
            ("Kernel USB/DRM events", self.adv_kernel_log),
            ("SteamVR log errors", self.adv_steamvr_logs),
            ("Export diagnostics report", self.adv_export)]

        side_sw = Gtk.ScrolledWindow(
            hscrollbar_policy=Gtk.PolicyType.NEVER)
        side_sw.set_size_request(240, -1)
        lb = Gtk.ListBox()
        lb.add_css_class("navigation-sidebar")
        lb.set_selection_mode(Gtk.SelectionMode.NONE)
        side_sw.set_child(lb)
        outer.append(side_sw)

        self.adv_live_btn = Gtk.Label(label="Live tracking data", xalign=0)
        for widget in [self.adv_live_btn] + [
                Gtk.Label(label=lbl, xalign=0) for lbl, _ in tools]:
            row = Gtk.ListBoxRow()
            for m in (widget.set_margin_top, widget.set_margin_bottom):
                m(8)
            row.set_child(widget)
            lb.append(row)

        def on_row(_lb, row):
            idx = row.get_index()
            if idx == 0:
                self.adv_live_toggle()
                return
            label, fn = tools[idx - 1]
            self.adv_run(label, fn)
        lb.connect("row-activated", on_row)
        lb.set_activate_on_single_click(True)
        lb.set_selection_mode(Gtk.SelectionMode.SINGLE)

        self.adv_buf = Gtk.TextBuffer()
        self.adv_tv = Gtk.TextView(buffer=self.adv_buf, editable=False,
                                   monospace=True)
        for m in (self.adv_tv.set_top_margin, self.adv_tv.set_bottom_margin,
                  self.adv_tv.set_left_margin,
                  self.adv_tv.set_right_margin):
            m(10)
        sw = Gtk.ScrolledWindow()
        sw.set_child(self.adv_tv)
        sw.set_hexpand(True)
        sw.set_vexpand(True)
        sw.add_css_class("card")
        outer.append(sw)

        def on_adv_close(_w):
            self.adv_cancel.set()
            if self.adv_proc:
                try:
                    self.adv_proc.terminate()
                except OSError:
                    pass
            self.adv_live_stop()
            self.adv_win = None
            return False
        win.connect("close-request", on_adv_close)
        win.present()

    # ---- live tracking view
    def adv_live_toggle(self):
        if self.adv_live_proc:
            self.adv_live_stop()
            return
        if pgrep("vrserver"):
            self.adv_set("Close SteamVR first — it owns the headset.")
            return
        if self.busy:
            self.adv_set("Busy with another operation.")
            return
        self.busy = True
        self.adv_live_btn.set_label("Stop live view")
        GLib.idle_add(self._adv_state, True,
                      "Live tracking — click the row again or Cancel "
                      "to stop")
        self.adv_live_proc = subprocess.Popen(
            ["stdbuf", "-oL", OPENHMD_EXAMPLE, "0"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

        def reader():
            data = {"rot": "-", "pos": "-", "ctl": "-", "msgs": []}
            last_draw = 0.0
            proc = self.adv_live_proc
            for line in proc.stdout:
                if not self.adv_live_proc:
                    break
                line = line.strip()
                if line.startswith("rotation quat:"):
                    data["rot"] = line.split(":", 1)[1].strip()
                elif line.startswith("position vec:"):
                    data["pos"] = line.split(":", 1)[1].strip()
                elif line.startswith("controls state:"):
                    data["ctl"] = line.split(":", 1)[1].strip()
                elif line and not line.startswith(("device ", "  ", "vendor",
                                                   "product", "path",
                                                   "class", "flags", "num ",
                                                   "OpenHMD", "opening")):
                    data["msgs"] = (data["msgs"] + [line])[-12:]
                now = time.time()
                if now - last_draw > 0.15:
                    last_draw = now
                    self.adv_set(
                        "LIVE TRACKING (raw)\n\n"
                        f"rotation quat : {data['rot']}\n"
                        f"position  vec : {data['pos']}\n"
                        f"controls      : {data['ctl']}\n\n"
                        "driver messages:\n  " +
                        "\n  ".join(data["msgs"]))
            try:
                proc.stdout.close()
            except OSError:
                pass

        threading.Thread(target=reader, daemon=True).start()

    def adv_live_stop(self):
        proc = self.adv_live_proc
        self.adv_live_proc = None
        if proc:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
            self.busy = False
            if getattr(self, "adv_win", None):
                GLib.idle_add(self.adv_live_btn.set_label,
                              "Live tracking data")
                GLib.idle_add(self._adv_state, False, "Idle")

    # ---- one-shot tools (each returns the text to display)
    def adv_tracking_test(self):
        import statistics
        if pgrep("vrserver"):
            return "Close SteamVR first — it owns the headset."
        self.adv_set(
            "TRACKING QUALITY TEST — capturing 30 s…\n\n"
            "Keep the headset COMPLETELY STILL, 1–1.5 m from the sensor,\n"
            "front LEDs facing the camera lens. Do not touch it.\n")
        proc = subprocess.Popen([OPENHMD_EXAMPLE, "0"],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True)
        self.adv_proc = proc
        secs = 30
        for t in range(secs):
            if self.adv_cancel.is_set():
                self.adv_append(f"  cancelled at {t} s — analyzing "
                                "partial data")
                break
            time.sleep(1)
            if (t + 1) % 5 == 0:
                self.adv_append(f"  …{t + 1}/{secs} s")
        proc.terminate()
        try:
            out, _ = proc.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, _ = proc.communicate()
        self.adv_proc = None

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
        rep.append(f"exposure-sync warnings: {expo}   driver errors: "
                   f"{errs}")
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

    def adv_pose_test(self):
        """Measure pose stability from inside a running SteamVR session."""
        if not pgrep("vrserver"):
            return ("SteamVR is not running — this test measures poses "
                    "inside a live\nSteamVR session. Launch SteamVR first "
                    "(or use the other tracking test).")
        venv_py = os.path.join(POSE_VENV, "bin/python")
        have = (os.path.exists(venv_py) and subprocess.run(
            [venv_py, "-c", "import openvr"],
            capture_output=True).returncode == 0)
        if not have:
            self.adv_set("STEAMVR POSE TEST\n\nOne-time setup: installing "
                         "Python OpenVR bindings…")
            r = subprocess.run(["python3", "-m", "venv", POSE_VENV],
                               capture_output=True, text=True)
            if r.returncode != 0:
                return "venv creation failed:\n" + r.stderr
            r = subprocess.run([venv_py, "-m", "pip", "install",
                                "--quiet", "openvr"],
                               capture_output=True, text=True)
            if r.returncode != 0:
                return "pip install openvr failed:\n" + r.stderr
            self.adv_append("bindings installed ✓")
        self.adv_set(
            "STEAMVR POSE TEST — capturing 35 s\n\n"
            "Put the headset ON and move naturally: look around, move "
            "the\ncontrollers, include one slow full turn. (For a "
            "stationary\nbaseline instead: leave everything on the desk, "
            "facing the sensor,\nand cover the proximity sensor.)\n")
        proc = subprocess.Popen([venv_py, "-u", POSE_TEST, "35"],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True)
        self.adv_proc = proc
        lines = []
        for line in proc.stdout:
            if self.adv_cancel.is_set():
                break
            line = line.rstrip()
            lines.append(line)
            self.adv_append(line)
        proc.wait()
        self.adv_proc = None
        if lines and not self.adv_cancel.is_set():
            path = os.path.join(HOME, "rift-cv1-pose-test-%s.txt" %
                                time.strftime("%Y%m%d-%H%M%S"))
            with open(path, "w") as f:
                f.write("SteamVR pose test — " + time.ctime() + "\n\n")
                f.write("\n".join(lines) + "\n")
            self.adv_append(f"\nreport saved: {path}")
        return None

    def adv_room_reset(self):
        path = os.path.join(HOME, ".config/openhmd/rift-room-config.json")
        if not os.path.exists(path):
            return ("No room calibration file present — a fresh one is "
                    "calibrated\nautomatically during the next tracking "
                    "session.")
        bak = path + ".bak-" + time.strftime("%Y%m%d-%H%M%S")
        with open(path) as f:
            content = f.read()
        os.rename(path, bak)
        self.log("Room calibration reset (backup: " + bak + ")")
        return ("ROOM CALIBRATION RESET\n\n"
                "The stored sensor-camera pose was removed; the tracker "
                "will\nre-calibrate it automatically next session.\n\n"
                "Do this whenever you MOVE the sensor camera — a stale "
                "pose\ncauses wrong height and position shifting. "
                "Afterwards, re-run\nSteamVR Room Setup.\n\n"
                f"Backup: {bak}\n\nOld content:\n{content}")

    def adv_devices(self):
        try:
            r = subprocess.run([OPENHMD_EXAMPLE, "999"],
                               capture_output=True, text=True, timeout=10)
            out = r.stdout
        except subprocess.TimeoutExpired as e:
            out = (e.stdout or b"").decode(errors="replace")
        keep = []
        for line in out.splitlines():
            if line.startswith(("num devices", "device ")) or \
                    line.startswith(("  vendor", "  product", "  path",
                                     "  class")):
                keep.append(line)
        return "OPENHMD DEVICE LIST\n\n" + "\n".join(keep)

    def adv_boot_mode(self):
        mode, node = hmd_boot_mode()
        if node is None:
            return "Headset HID not found (asleep, unplugged, or wedged)."
        if mode is None:
            return f"Found {node} but failed to read boot report."
        name = BOOT_MODES.get(mode, f"unknown ({mode})")
        extra = ""
        if mode == 2:
            extra = ("\n\nWARNING: headset is stuck in radio-pairing boot "
                     "mode.\nUse 'Reboot headset (normal mode)' to fix.")
        return f"HEADSET BOOT MODE\n\nnode: {node}\nmode: {name}{extra}"

    def adv_reboot_normal(self):
        node = find_hmd_hidraw()
        if not node:
            return "Headset HID not found."
        self.adv_set("REBOOT HEADSET (normal mode)\n")
        try:
            hid_set_feature(node, [0x06, 0x00, 0x00, 0x00])
        except OSError as e:
            return f"Failed to send bootload report: {e}"
        self.adv_append("bootload report sent — headset is rebooting…")
        self.log("Sent reboot-to-normal to headset")
        for _ in range(6):
            if self.adv_cancel.is_set():
                return None
            time.sleep(1)
        self.adv_append("waiting for HID to come back…")
        ensure_hid_bound(self.adv_append)
        mode, _ = hmd_boot_mode()
        self.adv_append("boot mode now: " +
                        BOOT_MODES.get(mode, f"unknown ({mode})"))
        return None

    def adv_edid(self):
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
                out.append("  ^ manufacturer OVR (Oculus) — this is "
                           "the Rift\n")
                out.append(hexdump(edid[:128]))
        out.append("\nNote: the Rift asserts hotplug only while a VR "
                   "driver holds it awake.")
        return "\n".join(out)

    def adv_usb(self):
        r = subprocess.run(["lsusb"], capture_output=True, text=True)
        lines = [ln for ln in r.stdout.splitlines() if "2833" in ln]
        out = ["OCULUS USB DEVICES\n"] + (lines or ["none found"])
        found = usb_sysfs_device(OCULUS_VID, CAMERA_PID)
        if found:
            try:
                with open(found[0] + "/speed") as f:
                    speed = f.read().strip()
                usb3 = "USB 3.x" if int(speed) >= 5000 else \
                    f"USB 2 ({speed} Mbps) — use a USB 3 port!"
                out.append(f"\nTracking camera link: {usb3}")
            except (OSError, ValueError):
                pass
        mode, _ = hmd_boot_mode()
        if mode is not None:
            out.append("Headset boot mode: " +
                       BOOT_MODES.get(mode, str(mode)))
        return "\n".join(out)

    def adv_force_probe(self):
        script = ("for c in /sys/class/drm/card*-HDMI-*/status; do "
                  "echo detect > $c; done")
        if not shutil.which("pkexec"):
            return "pkexec not found — run as root:\n" + script
        r = subprocess.run(["pkexec", "sh", "-c", script],
                           capture_output=True, text=True)
        if r.returncode != 0:
            return "Re-probe failed:\n" + (r.stderr or r.stdout)
        time.sleep(2)
        return "Connector re-probe done.\n\n" + self.adv_edid()

    def adv_kernel_log(self):
        r = subprocess.run(["journalctl", "-k", "-b", "-n", "600",
                            "--no-pager"], capture_output=True, text=True)
        if r.returncode != 0:
            return "journalctl failed:\n" + r.stderr
        pat = re.compile(r"usb|2833|hdmi|edid|hidraw|xhci|drm",
                         re.IGNORECASE)
        lines = [ln for ln in r.stdout.splitlines() if pat.search(ln)]
        return ("KERNEL USB/DRM EVENTS (latest last)\n\n" +
                "\n".join(lines[-40:]))

    def adv_steamvr_logs(self):
        out = ["STEAMVR LOG ERRORS/WARNINGS (latest last)\n"]
        logdir = os.path.join(HOME, ".local/share/Steam/logs")
        for name in ("vrserver.txt", "vrcompositor.txt"):
            path = os.path.join(logdir, name)
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

    def adv_export(self):
        path = os.path.join(
            HOME, "rift-cv1-diagnostics-%s.txt" %
            time.strftime("%Y%m%d-%H%M%S"))
        sections = [
            ("SETUP STATE", "\n".join(
                f"{k}: {v[1]}" for k, v in self.setup_state().items())),
            ("OPENVR RUNTIME", active_runtime()),
            ("USB", self.adv_usb()),
            ("DEVICES", self.adv_devices()),
            ("DISPLAY", self.adv_edid()),
            ("KERNEL EVENTS", self.adv_kernel_log()),
            ("STEAMVR LOGS", self.adv_steamvr_logs()),
        ]
        with open(path, "w") as f:
            f.write("Rift CV1 diagnostics — " + time.ctime() + "\n\n")
            for title, body in sections:
                f.write(f"{'=' * 60}\n{title}\n{'=' * 60}\n{body}\n\n")
        self.log("Diagnostics written to " + path)
        return f"Diagnostics report written to:\n{path}"

    # ---------------- camera view & placement guide
    GUIDES = [
        ("Normal play",
         "• Stand 1.5–2 m from the sensor, centred in its view\n"
         "• Face the sensor for aim-critical games\n"
         "• Keep hands in front of your body — behind your back the\n"
         "  sensor cannot see the controller LEDs\n"
         "• No sunlight, halogen lamps or mirrors in the play area"),
        ("Room calibration (sensor moved)",
         "• Place the HEADSET on the desk, 1–1.5 m in front of the "
         "sensor\n• Front visor LEDs facing the camera lens directly\n"
         "• Advanced → Reset room calibration first\n"
         "• Start SteamVR, leave the headset untouched ~30 s\n"
         "• Then run SteamVR Room Setup (Standing) for floor height"),
        ("Controller pairing",
         "• Close SteamVR, open the pairing wizard\n"
         "• Hold each controller within ~30 cm of the headset\n"
         "• RIGHT: hold Oculus + B until its LED blinks\n"
         "• LEFT: hold Menu + Y until its LED blinks\n"
         "• Watch the wizard rows turn green"),
        ("Tracking test / baseline",
         "• Headset on the desk 1–1.5 m from the sensor, LEDs facing "
         "it\n• Controllers next to it, tracking rings facing the "
         "sensor\n• Do not touch anything during the capture\n"
         "• In the camera view you should see the LED dots light up"),
    ]

    def on_camera(self, _b):
        if getattr(self, "cam_win", None):
            self.cam_win.present()
            return
        try:
            gi.require_version("Gst", "1.0")
            from gi.repository import Gst
        except (ValueError, ImportError) as e:
            self.log(f"GStreamer not available: {e} — install gstreamer "
                     "and gst-plugin-pipewire")
            return
        if not getattr(self, "_gst_ready", False):
            Gst.init(None)
            self._gst_ready = True
        self._Gst = Gst

        win = Gtk.Window(title="Camera & Placement — Rift CV1")
        win.set_transient_for(self.win)
        win.set_default_size(1040, 600)
        self.cam_win = win
        self.cam_pipe = None
        self.cam_proc = None
        self.cam_stop = threading.Event()

        outer = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=12)
        for m in (outer.set_margin_top, outer.set_margin_bottom,
                  outer.set_margin_start, outer.set_margin_end):
            m(12)
        win.set_child(outer)

        left = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        left.set_hexpand(True)
        outer.append(left)
        self.cam_picture = Gtk.Picture()
        self.cam_picture.set_vexpand(True)
        self.cam_picture.add_css_class("card")
        left.append(self.cam_picture)
        self.cam_status = Gtk.Label(
            label="Starting camera stream…", xalign=0)
        self.cam_status.add_css_class("dim-label")
        left.append(self.cam_status)

        right = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        right.set_size_request(380, -1)
        outer.append(right)
        dd = Gtk.DropDown.new_from_strings(
            [name for name, _ in self.GUIDES])
        right.append(dd)
        self.guide_sel = 0
        self.guide_area = Gtk.DrawingArea()
        self.guide_area.set_content_height(280)
        self.guide_area.add_css_class("card")
        self.guide_area.set_draw_func(self.draw_guide)
        right.append(self.guide_area)
        self.guide_text = Gtk.Label(xalign=0, wrap=True)
        self.guide_text.set_text(self.GUIDES[0][1])
        right.append(self.guide_text)

        def on_mode(dd_, _p):
            self.guide_sel = dd_.get_selected()
            self.guide_text.set_text(self.GUIDES[self.guide_sel][1])
            self.guide_area.queue_draw()
        dd.connect("notify::selected", on_mode)

        def on_cam_close(_w):
            self.cam_stop.set()
            if self.cam_pipe:
                self.cam_pipe.set_state(Gst.State.NULL)
                self.cam_pipe = None
            if self.cam_proc:
                self.cam_proc.terminate()
                try:
                    self.cam_proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.cam_proc.kill()
                    self.cam_proc.wait()
                self.cam_proc = None
            self.cam_win = None
            return False
        win.connect("close-request", on_cam_close)
        win.present()
        self.cam_stream_start()

    def cam_stream_start(self):
        Gst = self._Gst

        def set_status(txt):
            def _s():
                if getattr(self, "cam_win", None):
                    self.cam_status.set_text(txt)
                return False
            GLib.idle_add(_s)

        def worker():
            # a tracking session must be running to publish the stream
            if not pgrep("vrserver") and not self.cam_proc:
                set_status("Starting a tracking session "
                           "(headset wakes up)…")
                self.cam_proc = subprocess.Popen(
                    [OPENHMD_EXAMPLE, "0"], stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL)
            node = None
            deadline = time.time() + 25
            while time.time() < deadline and not self.cam_stop.is_set():
                try:
                    out = subprocess.run(["pw-dump"], capture_output=True,
                                         text=True, timeout=5).stdout
                    objs = json.loads(out)
                except (OSError, ValueError,
                        subprocess.TimeoutExpired):
                    objs = []
                cands = []
                for o in objs:
                    p = (o.get("info") or {}).get("props") or {}
                    if str(p.get("media.name", "")).startswith(
                            "openhmd-rift-sensor"):
                        cands.append((o["id"], json.dumps(o)))
                for oid, blob in cands:
                    if '"RGB"' in blob:   # annotated tracking view
                        node = oid
                        break
                if node is None and cands:
                    node = cands[0][0]
                if node is not None:
                    break
                time.sleep(1)
            if self.cam_stop.is_set():
                return
            if node is None:
                set_status("No camera stream found. Restart SteamVR (the "
                           "driver was rebuilt with camera support), or "
                           "close this window and retry.")
                return
            set_status(f"Live — sensor debug view (PipeWire node {node}). "
                       "LED dots + tracking overlay appear when the "
                       "headset is awake and visible.")
            pipe = Gst.parse_launch(
                f"pipewiresrc path={node} ! video/x-raw,format=RGB ! "
                "appsink name=sink emit-signals=true max-buffers=2 "
                "drop=true")
            sink = pipe.get_by_name("sink")
            last = [0.0]

            def on_sample(s):
                sample = s.emit("pull-sample")
                if sample is None:
                    return Gst.FlowReturn.OK
                now = time.time()
                if now - last[0] < 0.05:      # ~20 fps is plenty
                    return Gst.FlowReturn.OK
                last[0] = now
                buf = sample.get_buffer()
                st = sample.get_caps().get_structure(0)
                w = st.get_value("width")
                h = st.get_value("height")
                ok, mi = buf.map(Gst.MapFlags.READ)
                if not ok:
                    return Gst.FlowReturn.OK
                data = bytes(mi.data)
                buf.unmap(mi)

                def upd():
                    if getattr(self, "cam_win", None):
                        tex = Gdk.MemoryTexture.new(
                            w, h, Gdk.MemoryFormat.R8G8B8,
                            GLib.Bytes.new(data), w * 3)
                        self.cam_picture.set_paintable(tex)
                    return False
                GLib.idle_add(upd)
                return Gst.FlowReturn.OK

            sink.connect("new-sample", on_sample)
            pipe.set_state(Gst.State.PLAYING)
            self.cam_pipe = pipe
        threading.Thread(target=worker, daemon=True).start()

    def draw_guide(self, _area, cr, w, h):
        mode = self.guide_sel
        # background
        cr.set_source_rgb(0.12, 0.12, 0.14)
        cr.paint()
        m = 18

        def txt(x, y, s, size=11, col=(0.85, 0.85, 0.85)):
            cr.set_source_rgb(*col)
            cr.set_font_size(size)
            cr.move_to(x, y)
            cr.show_text(s)

        # room outline
        cr.set_source_rgb(0.4, 0.4, 0.45)
        cr.set_line_width(1.5)
        cr.rectangle(m, m, w - 2 * m, h - 2 * m)
        cr.stroke()
        # desk along the top edge
        cr.set_source_rgb(0.30, 0.25, 0.20)
        cr.rectangle(m, m, w - 2 * m, 34)
        cr.fill()
        txt(w / 2 - 14, m + 21, "desk", 10, (0.7, 0.65, 0.6))
        # sensor: upper-left on the desk, FOV cone into the room
        sx, sy = m + 30, m + 17
        cr.set_source_rgba(0.25, 0.55, 0.95, 0.18)
        cr.move_to(sx, sy)
        cr.line_to(w * 0.95, h * 0.62)
        cr.line_to(w * 0.25, h * 0.98)
        cr.close_path()
        cr.fill()
        cr.set_source_rgb(0.3, 0.65, 1.0)
        cr.arc(sx, sy, 6, 0, 6.2832)
        cr.fill()
        txt(sx - 12, sy - 12, "sensor", 10, (0.5, 0.75, 1.0))

        def headset(x, y):
            cr.set_source_rgb(0.9, 0.9, 0.9)
            cr.rectangle(x - 13, y - 8, 26, 16)
            cr.fill()
            txt(x - 13, y + 26, "headset", 10)

        def ctrl(x, y, label, col):
            cr.set_source_rgb(*col)
            cr.arc(x, y, 7, 0, 6.2832)
            cr.fill()
            txt(x - 4, y + 20, label, 10, col)

        if mode == 0:      # normal play
            cr.set_source_rgb(0.75, 0.75, 0.78)
            cr.arc(w * 0.52, h * 0.62, 13, 0, 6.2832)
            cr.fill()
            txt(w * 0.52 - 12, h * 0.62 + 30, "you", 10)
            ctrl(w * 0.42, h * 0.55, "L", (0.4, 0.85, 0.5))
            ctrl(w * 0.62, h * 0.55, "R", (0.95, 0.55, 0.4))
            cr.set_source_rgba(0.9, 0.9, 0.9, 0.6)
            cr.set_dash([4, 4])
            cr.move_to(w * 0.52, h * 0.58)
            cr.line_to(sx + 8, sy + 8)
            cr.stroke()
            cr.set_dash([])
            txt(w * 0.30, h * 0.40, "1.5–2 m, face the sensor", 10,
                (0.6, 0.8, 1.0))
        elif mode in (1, 3):   # room calibration / tracking test
            hx, hy = w * 0.42, h * 0.42
            headset(hx, hy)
            cr.set_source_rgba(0.9, 0.9, 0.9, 0.6)
            cr.set_dash([4, 4])
            cr.move_to(hx, hy - 8)
            cr.line_to(sx + 6, sy + 6)
            cr.stroke()
            cr.set_dash([])
            txt(w * 0.20, h * 0.30, "1–1.5 m, LEDs facing sensor", 10,
                (0.6, 0.8, 1.0))
            txt(w * 0.30, h * 0.80,
                "keep completely still" if mode == 3
                else "untouched for ~30 s after SteamVR start", 10,
                (0.95, 0.8, 0.4))
            if mode == 3:
                ctrl(w * 0.58, h * 0.44, "L", (0.4, 0.85, 0.5))
                ctrl(w * 0.68, h * 0.48, "R", (0.95, 0.55, 0.4))
        elif mode == 2:    # pairing
            hx, hy = w * 0.45, h * 0.50
            headset(hx, hy)
            ctrl(hx - 55, hy + 8, "L", (0.4, 0.85, 0.5))
            ctrl(hx + 55, hy + 8, "R", (0.95, 0.55, 0.4))
            txt(w * 0.28, h * 0.78, "controllers within ~30 cm of the "
                "headset", 10, (0.6, 0.8, 1.0))

    # ---------------- pairing wizard
    def on_pair(self, _b):
        if self.busy:
            self.log("Busy with another operation, please wait…")
            return
        if pgrep("vrserver"):
            self.log("Close SteamVR before pairing (it owns the headset).")
            return
        if not os.path.exists(OUVRTD):
            self.log(f"Missing {OUVRTD} — build ouvrt first")
            return
        self.busy = True
        dlg = Gtk.Window(title="Pair Touch Controllers", modal=True)
        dlg.set_transient_for(self.win)
        dlg.set_default_size(480, 320)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        for m in (box.set_margin_top, box.set_margin_bottom,
                  box.set_margin_start, box.set_margin_end):
            m(14)
        dlg.set_child(box)
        self.pair_status = Gtk.Label(label="Starting pairing daemon…",
                                     wrap=True)
        self.pair_status.add_css_class("title-4")
        box.append(self.pair_status)

        plist = Gtk.ListBox()
        plist.add_css_class("boxed-list")
        plist.set_selection_mode(Gtk.SelectionMode.NONE)
        box.append(plist)
        lr_rows = {"R": StatusRow("Right controller"),
                   "L": StatusRow("Left controller")}
        lr_rows["R"].set(None, "hold Oculus + B until the LED blinks")
        lr_rows["L"].set(None, "hold Menu + Y until the LED blinks")
        plist.append(lr_rows["R"])
        plist.append(lr_rows["L"])

        info = Gtk.Label(xalign=0, wrap=True)
        info.set_markup(
            "<small>Hold the controllers near the headset. Each row turns "
            "green when its\ncontroller is bonded. Then press Done — the "
            "headset reboots back to\nnormal mode automatically.</small>")
        info.add_css_class("dim-label")
        box.append(info)

        done = Gtk.Button()
        done.set_child(Adw.ButtonContent(
            label="Done — back to normal mode",
            icon_name="emblem-ok-symbolic", halign=Gtk.Align.CENTER))
        done.add_css_class("pill")
        done.add_css_class("suggested-action")
        done.set_sensitive(False)
        box.append(done)
        dlg.present()

        state = {"paired": set(), "stop": False, "finishing": False,
                 "done": False}

        def set_status(txt):
            GLib.idle_add(self.pair_status.set_text, txt)

        def log_since(logf, pos):
            try:
                with open(logf) as f:
                    f.seek(pos)
                    chunk = f.read()
                    return chunk, f.tell()
            except OSError:
                return "", pos

        def wait_for(logf, pos, patterns, timeout_s):
            """Tail logf until any pattern appears."""
            deadline = time.time() + timeout_s
            buf = ""
            while time.time() < deadline:
                chunk, pos = log_since(logf, pos)
                buf += chunk
                for p in patterns:
                    if p in buf:
                        return p, pos
                time.sleep(0.5)
            return None, pos

        def find_radio_retry(tries=6):
            for _ in range(tries):
                obj = find_radio_object()
                if obj:
                    return obj
                ensure_hid_bound(self.log, tries=1)
                time.sleep(3)
            return None

        PAIRING_CONFIRM = ("Rebooting in radio pairing mode",
                           "Already in radio pairing mode")

        def pair_task():
            logf = "/tmp/rift-pairing.log"
            pos = 0
            try:
                # a stale daemon would own the DBus name and shadow ours
                subprocess.run(["pkill", "-x", "ouvrtd"],
                               capture_output=True)
                time.sleep(1)
                with open(logf, "w") as logfh:
                    # stdbuf: ouvrtd block-buffers stdout when piped, which
                    # would delay pairing events by minutes
                    self.ouvrtd_proc = subprocess.Popen(
                        ["stdbuf", "-oL", "-eL", OUVRTD],
                        stdout=logfh, stderr=subprocess.STDOUT)
                self.log("ouvrtd started for pairing")
                time.sleep(5)
                obj = find_radio_retry()
                if not obj:
                    set_status("ERROR: headset radio not found — "
                               "press Done to clean up")
                    GLib.idle_add(done.set_sensitive, True)
                    return
                gdbus_call(obj, "StartDiscovery")
                hit, pos = wait_for(logf, pos, PAIRING_CONFIRM, 15)
                if hit and "Already" in hit:
                    # Stale pairing boot from an aborted session: the boot
                    # mode still reads "pairing" but the radio's listening
                    # window has expired. Cycle through normal mode to open
                    # a fresh window.
                    self.log("Stale pairing mode detected — cycling for a "
                             "fresh pairing window…")
                    gdbus_call(obj, "StopDiscovery")
                    time.sleep(6)
                    obj = find_radio_retry()
                    if obj:
                        gdbus_call(obj, "StartDiscovery")
                        hit, pos = wait_for(logf, pos, PAIRING_CONFIRM, 15)
                if not hit:
                    self.log("WARNING: headset did not confirm pairing "
                             "mode — pairing may not work")
                time.sleep(5)
                ensure_hid_bound(self.log)
                set_status("Headset in PAIRING MODE — pair controllers now")
                GLib.idle_add(done.set_sensitive, True)
            except Exception as e:
                self.log(f"Pairing error: {e}")
                set_status("ERROR — press Done to clean up")
                GLib.idle_add(done.set_sensitive, True)
                return
            pos = 0
            while not state["stop"]:
                time.sleep(1)
                try:
                    with open(logf) as f:
                        f.seek(pos)
                        chunk = f.read()
                        pos = f.tell()
                except OSError:
                    continue
                for line in chunk.splitlines():
                    if ("Pairing" in line or "Detected" in line
                            or "pairing mode" in line
                            or "Rebooting" in line):
                        self.log(line.strip())
                    m = re.search(r"Pairing Touch Controller (\w) .*finished",
                                  line)
                    if m:
                        side = m.group(1)
                        state["paired"].add(side)
                        if side in lr_rows:
                            GLib.idle_add(lr_rows[side].set, True,
                                          "paired ✓")
                        set_status("Paired: " +
                                   ", ".join(sorted(state["paired"])) +
                                   (" — press Done"
                                    if len(state["paired"]) >= 2 else
                                    " — pair the other one"))

        def finish_task():
            set_status("Rebooting headset to normal mode…")
            try:
                for _attempt in range(4):
                    obj = find_radio_object()
                    if obj and gdbus_call(obj,
                                          "StopDiscovery").returncode == 0:
                        break
                    ensure_hid_bound(self.log)
                    time.sleep(4)
                time.sleep(5)
                ensure_hid_bound(self.log)
            except Exception as e:
                self.log(f"Cleanup error: {e}")
            finally:
                if self.ouvrtd_proc:
                    self.ouvrtd_proc.terminate()
                    try:
                        self.ouvrtd_proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        self.ouvrtd_proc.kill()
                        self.ouvrtd_proc.wait()
                    self.ouvrtd_proc = None
                self.busy = False
                state["done"] = True
                self.log("Pairing session closed; headset back in normal "
                         "mode. Paired: " +
                         (", ".join(sorted(state["paired"])) or "none"))
                GLib.idle_add(dlg.close)
                GLib.idle_add(self.refresh_once)

        def begin_finish():
            if state["finishing"]:
                return
            state["finishing"] = True
            state["stop"] = True
            GLib.idle_add(done.set_sensitive, False)
            threading.Thread(target=finish_task, daemon=True).start()

        def on_dlg_close(_w):
            if state["done"]:
                return False  # cleanup finished — allow the close
            begin_finish()
            return True  # block close until headset is back in normal mode

        done.connect("clicked", lambda _b: begin_finish())
        dlg.connect("close-request", on_dlg_close)
        threading.Thread(target=pair_task, daemon=True).start()

    def on_close(self, _w):
        if self.ouvrtd_proc:
            # best effort: never leave the headset in pairing-boot mode
            try:
                obj = find_radio_object()
                if obj:
                    gdbus_call(obj, "StopDiscovery")
                    time.sleep(2)
            except Exception:
                pass
            self.ouvrtd_proc.terminate()
        return False


if __name__ == "__main__":
    App().run(None)
