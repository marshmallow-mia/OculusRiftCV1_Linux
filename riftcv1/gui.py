"""GTK4/Adwaita GUI for the Rift CV1 Control Center."""
import os
import subprocess
import threading
import time

import gi
gi.require_version("Gtk", "4.0")
gi.require_version("Adw", "1")
from gi.repository import Adw, Gio, GLib, Gtk

from . import config, diagnostics, hw, install, runtime
from .calwizard import open_calibration_wizard
from .camview import open_camera_window
from .pairing import find_radio_object, gdbus_call, open_pairing_wizard
from .roomcal import open_roomcal_window
from .sensorsetup import open_sensor_setup
from .widgets import StatusRow


class App(Adw.Application):
    def __init__(self):
        super().__init__(application_id="de.local.RiftCV1Center")
        self.ouvrtd_proc = None
        self.busy = False
        self._last_autofix = 0.0
        self._refreshing = False
        self._ctl_checked = 0.0
        self._ctl_cache = None
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
        menu.append("Tracking convergence check", "app.convergence")
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
                ("convergence",
                 lambda *_: open_calibration_wizard(self)),
                ("register", lambda *_: self.on_register(None)),
                ("open-config", lambda *_: self.open_folder(
                    config.OPENHMD_CONFIG)),
                ("open-logs", lambda *_: self.open_folder(
                    config.STEAM_LOGS)),
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
                ("cam", "Tracking sensors"),
                ("ctl", "Controllers"),
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
        self.btn_cal = button("Sensor Setup",
                              "find-location-symbolic", self.on_calibrate,
                              "suggested-action")
        grid.attach(self.btn_cam, 0, 4, 1, 1)
        grid.attach(self.btn_cal, 1, 4, 1, 1)
        self.btn_roomcal = button("Room calibration (advanced)",
                                  "view-grid-symbolic", self.on_roomcal)
        grid.attach(self.btn_roomcal, 0, 5, 2, 1)

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
            version=config.VERSION,
            developer_name="mia",
            comments="Status, display test, USB fixes, Touch controller "
                      "pairing, tracking diagnostics and SteamVR "
                      "integration for the Oculus Rift CV1 on Linux.\n\n"
                      "Stack: SteamVR-OpenHMD + OpenHMD "
                      "(rift-kalman-filter) + ouvrt.\n\n"
                      "Also scriptable from the terminal:\n"
                      "rift_cv1_center.py status | fix-usb | test | "
                      "export | switch-runtime | …")
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
        # gather off the main thread — sysfs walks, JSON parsing and the
        # occasional openvr query must not stall the GTK main loop
        if not self._refreshing:
            self._refreshing = True
            threading.Thread(target=self._gather_status,
                             daemon=True).start()
        return True

    def _gather_status(self):
        try:
            st = {
                "usb": hw.usb_sysfs_device(hw.OCULUS_VID,
                                           hw.HMD_PID) is not None,
                "hid": hw.hmd_hid_state(),
                "sensors": hw.tracking_sensors(),
                "drv": runtime.driver_registered(),
                "drv_fresh": runtime.driver_deploy_state(),
                "rt": runtime.active_runtime(),
                "svr": hw.proc_running("vrserver"),
            }
            st["conn"], st["ovr"] = hw.rift_edid_connector()
            if st["svr"]:
                now = time.time()
                if now - self._ctl_checked > 15:   # battery poll is slow
                    self._ctl_checked = now
                    self._ctl_cache = runtime.vr_device_status()
            else:
                self._ctl_cache = None
            st["ctl"] = self._ctl_cache
            GLib.idle_add(self._apply_status, st)
        finally:
            self._refreshing = False

    def _apply_status(self, st):
        usb, hid, svr = st["usb"], st["hid"], st["svr"]
        self.rows["usb"].set(usb, "connected" if usb else "not found")
        self.rows["hid"].set(
            {"bound": True, "in-use": True, "unbound": False}.get(hid),
            {"bound": "bound",
             "in-use": "in use by VR driver",
             "unbound": "NOT BOUND — use Fix USB"}.get(hid, "—"))

        sensors = st["sensors"]
        cam = bool(sensors)
        if not sensors:
            self.rows["cam"].set(False, "not found")
        else:
            n = len(sensors)
            label = "connected" if n == 1 else f"{n} connected"
            slow = [s for s in sensors if s["speed"] and s["speed"] < 5000]
            known = [s for s in sensors if s["speed"]]
            nosuspend = [s for s in sensors
                         if s["power"] and s["power"] != "on"]
            if slow:
                self.rows["cam"].set(None, label +
                                     " — on USB 2, use a USB 3 port!")
            elif nosuspend:
                self.rows["cam"].set(None, label +
                                     " — autosuspend on, reinstall "
                                     "udev rules (Setup)")
            else:
                self.rows["cam"].set(True, label +
                                     (" (USB 3)" if known else ""))

        ctl = st["ctl"]
        if not svr:
            self.rows["ctl"].set(None, "shown while SteamVR runs")
        elif ctl is None:
            self.rows["ctl"].set(None, "no data — run the SteamVR pose "
                                       "test once to install bindings")
        else:
            parts = []
            for d in ctl:
                if d.get("class") != "Controller":
                    continue
                s = d.get("role") or "?"
                if not d.get("connected"):
                    parts.append(s + " off")
                elif d.get("battery") is not None:
                    parts.append("%s %d%%%s" % (
                        s, d["battery"], "⚡" if d.get("charging") else ""))
                else:
                    parts.append(s + " ✓")
            self.rows["ctl"].set(True if parts else None,
                                 " · ".join(sorted(parts))
                                 if parts else "none detected")

        conn, ovr = st["conn"], st["ovr"]
        self.rows["disp"].set(
            True if ovr else None,
            f"Rift active on {conn}" if ovr
            else "asleep / unknown (run test)")

        mtime = runtime.room_config_mtime()
        if mtime:
            self.rows["room"].set(
                True, "set up " + time.strftime("%b %d", time.localtime(
                    mtime)) + " — rerun Sensor Setup if a sensor moved")
        else:
            self.rows["room"].set(None, "not set up — run Sensor Setup")

        drv = st["drv"]
        if drv and st.get("drv_fresh") is False:
            self.rows["drv"].set(False, "deployed copy STALE — rerun "
                                        "install_files_to_build.sh")
        else:
            self.rows["drv"].set(drv,
                                 "registered" if drv else "not registered")

        rt = st["rt"]
        self.rows["rt"].set(rt == "SteamVR", rt)

        self.rows["svr"].set(True if svr else None,
                             "running" if svr else "not running")

        # auto-fix the USB wedge (gentle driver reattach, no reset).
        # 'in-use' is NOT a wedge — a VR driver detached the kernel HID on
        # purpose; reattaching would yank the headset out of the session.
        if (hid == "unbound" and not svr and not self.busy
                and self.autofix.get_state().get_boolean()
                and time.time() - self._last_autofix > 20):
            self._last_autofix = time.time()
            self.log("Auto-fix: headset HID unbound — reattaching driver…")
            threading.Thread(target=hw.usb_reattach_hid,
                             daemon=True).start()

        # smart suggestion banner
        def banner(msg, btn=None, action=None):
            self.banner.set_title(msg)
            self.banner.set_button_label(btn)
            self.banner_action = action
            self.banner.set_revealed(True)

        if not usb:
            banner("Headset not detected — check its USB connection "
                   "and power")
        elif hid == "unbound" and not svr:
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
        return False

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
            diagnostics.wake_test(self.log)
            GLib.idle_add(self.refresh_once)
        self.run_async(task)

    def on_reset(self, _b):
        def task():
            hw.fix_usb(self.log)
            GLib.idle_add(self.refresh_once)
        self.run_async(task)

    def on_register(self, _b):
        def task():
            install.register_driver(self.run_cmd, self.log)
            GLib.idle_add(self.refresh_once)
        self.run_async(task)

    def on_switch_runtime(self, _b):
        def task():
            _, msg = runtime.switch_runtime()
            self.log(msg)
            GLib.idle_add(self.refresh_once)
        self.run_async(task)

    def on_launch_steamvr(self, _b):
        subprocess.Popen(["steam", "steam://rungameid/250820"],
                         stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
        self.log("Asked Steam to launch SteamVR…")

    def on_pair(self, _b):
        open_pairing_wizard(self)

    def on_calibrate(self, _b):
        open_sensor_setup(self)

    def on_roomcal(self, _b):
        open_roomcal_window(self)

    def on_camera(self, _b):
        open_camera_window(self)

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
                        f"them first:\n<tt>{install.deps_hint()}</tt>"
                        "</small>")
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
            for k, (ok, txt) in install.setup_state().items():
                rows[k].set(ok, txt)
            return False

        def install_all():
            missing = install.check_deps()
            if missing:
                self.log("Missing build dependencies: " +
                         ", ".join(missing))
                self.log("Install them first: " + install.deps_hint())
                GLib.idle_add(refresh)
                return
            steps = [
                ("SteamVR-OpenHMD",
                 lambda: install.install_steamvr_openhmd(self.run_cmd,
                                                         self.log)),
                ("ouvrt", lambda: install.install_ouvrt(self.run_cmd)),
                ("udev rules",
                 lambda: install.install_udev(self.run_cmd, self.log)),
                ("driver registration",
                 lambda: install.register_driver(self.run_cmd, self.log)),
                ("desktop entry",
                 lambda: install.install_desktop(self.log)),
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

    def _set_adv_proc(self, proc):
        self.adv_proc = proc

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
            ("Room calibration info", diagnostics.room_calibration_info),
            ("Headset info (serial / firmware)", diagnostics.headset_info),
            ("OpenHMD device list", diagnostics.device_list),
            ("Headset boot mode", diagnostics.boot_mode_text),
            ("Reboot headset (normal mode)", self.adv_reboot_normal),
            ("Display / EDID info", diagnostics.edid_info),
            ("USB topology", diagnostics.usb_topology),
            ("Force display re-probe (root)", diagnostics.force_probe),
            ("Kernel USB/DRM events", diagnostics.kernel_log),
            ("SteamVR log errors", diagnostics.steamvr_logs),
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
        if hw.proc_running("vrserver"):
            self.adv_set("Close SteamVR first — it owns the headset.")
            return
        if not os.path.exists(config.OPENHMD_EXAMPLE):
            self.adv_set(f"Missing {config.OPENHMD_EXAMPLE} — build "
                         "SteamVR-OpenHMD first (Setup & install).")
            return
        if self.busy:
            self.adv_set("Busy with another operation.")
            return
        self.busy = True
        try:
            self.adv_live_proc = subprocess.Popen(
                ["stdbuf", "-oL", config.OPENHMD_EXAMPLE, "0"],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True)
        except OSError as e:
            self.busy = False
            self.adv_set(f"Failed to start tracking session: {e}")
            return
        self.adv_live_btn.set_label("Stop live view")
        GLib.idle_add(self._adv_state, True,
                      "Live tracking — click the row again or Cancel "
                      "to stop")

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
        return diagnostics.tracking_quality_test(
            self.adv_set, self.adv_append, self.adv_cancel,
            self._set_adv_proc)

    def adv_pose_test(self):
        """Measure pose stability from inside a running SteamVR session."""
        if not hw.proc_running("vrserver"):
            return ("SteamVR is not running — this test measures poses "
                    "inside a live\nSteamVR session. Launch SteamVR first "
                    "(or use the other tracking test).")
        venv_py = runtime.ensure_openvr_venv(self.adv_append)
        if not venv_py:
            return "Failed to install the Python OpenVR bindings."
        self.adv_set(
            "STEAMVR POSE TEST — capturing 35 s\n\n"
            "Put the headset ON and move naturally: look around, move "
            "the\ncontrollers, include one slow full turn. (For a "
            "stationary\nbaseline instead: leave everything on the desk, "
            "facing the sensor,\nand cover the proximity sensor.)\n")
        proc = subprocess.Popen([venv_py, "-u", config.POSE_TEST, "35"],
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
            path = os.path.join(config.HOME,
                                "rift-cv1-pose-test-%s.txt" %
                                time.strftime("%Y%m%d-%H%M%S"))
            with open(path, "w") as f:
                f.write("SteamVR pose test — " + time.ctime() + "\n\n")
                f.write("\n".join(lines) + "\n")
            self.adv_append(f"\nreport saved: {path}")
        return None

    def adv_room_reset(self):
        return diagnostics.room_reset(self.log)

    def adv_reboot_normal(self):
        node = hw.find_hmd_hidraw()
        if not node:
            return "Headset HID not found."
        self.adv_set("REBOOT HEADSET (normal mode)\n")
        try:
            hw.hid_set_feature(node, [0x06, 0x00, 0x00, 0x00])
        except OSError as e:
            return f"Failed to send bootload report: {e}"
        self.adv_append("bootload report sent — headset is rebooting…")
        self.log("Sent reboot-to-normal to headset")
        for _ in range(6):
            if self.adv_cancel.is_set():
                return None
            time.sleep(1)
        self.adv_append("waiting for HID to come back…")
        hw.ensure_hid_bound(self.adv_append)
        mode, _ = hw.hmd_boot_mode()
        self.adv_append("boot mode now: " +
                        hw.BOOT_MODES.get(mode, f"unknown ({mode})"))
        return None

    def adv_export(self):
        path, text = diagnostics.export_report()
        self.log("Diagnostics written to " + path)
        return text

    # ---------------- shutdown
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


def run_gui():
    return App().run(None)
