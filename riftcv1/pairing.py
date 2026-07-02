"""Touch controller pairing: ouvrt DBus helpers and the pairing wizard.

Pairing reboots the headset radio into pairing mode via ouvrtd, bonds the
controllers, then reboots back to normal mode.
"""
import os
import re
import subprocess
import threading
import time

import gi
gi.require_version("Gtk", "4.0")
gi.require_version("Adw", "1")
from gi.repository import Adw, GLib, Gtk

from . import config, hw
from .widgets import StatusRow

OUVRT_BUS = "de.phfuenf.ouvrt.Ouvrtd"
OUVRT_ROOT = "/de/phfuenf/ouvrt"
RADIO_IFACE = "de.phfuenf.ouvrt.Radio1"

PAIRING_CONFIRM = ("Rebooting in radio pairing mode",
                   "Already in radio pairing mode")


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


def open_pairing_wizard(app):
    if app.busy:
        app.log("Busy with another operation, please wait…")
        return
    if hw.proc_running("vrserver"):
        app.log("Close SteamVR before pairing (it owns the headset).")
        return
    if not os.path.exists(config.OUVRTD):
        app.log(f"Missing {config.OUVRTD} — build ouvrt first")
        return
    app.busy = True
    dlg = Gtk.Window(title="Pair Touch Controllers", modal=True)
    dlg.set_transient_for(app.win)
    dlg.set_default_size(480, 320)
    box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
    for m in (box.set_margin_top, box.set_margin_bottom,
              box.set_margin_start, box.set_margin_end):
        m(14)
    dlg.set_child(box)
    status_lbl = Gtk.Label(label="Starting pairing daemon…", wrap=True)
    status_lbl.add_css_class("title-4")
    box.append(status_lbl)

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
        GLib.idle_add(status_lbl.set_text, txt)

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
            hw.ensure_hid_bound(app.log, tries=1)
            time.sleep(3)
        return None

    def pair_task():
        logf = "/tmp/rift-pairing.log"
        pos = 0
        try:
            # a stale daemon would own the DBus name and shadow ours
            subprocess.run(["pkill", "-x", "ouvrtd"], capture_output=True)
            time.sleep(1)
            with open(logf, "w") as logfh:
                # stdbuf: ouvrtd block-buffers stdout when piped, which
                # would delay pairing events by minutes
                app.ouvrtd_proc = subprocess.Popen(
                    ["stdbuf", "-oL", "-eL", config.OUVRTD],
                    stdout=logfh, stderr=subprocess.STDOUT)
            app.log("ouvrtd started for pairing")
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
                app.log("Stale pairing mode detected — cycling for a "
                        "fresh pairing window…")
                gdbus_call(obj, "StopDiscovery")
                time.sleep(6)
                obj = find_radio_retry()
                if obj:
                    gdbus_call(obj, "StartDiscovery")
                    hit, pos = wait_for(logf, pos, PAIRING_CONFIRM, 15)
            if not hit:
                app.log("WARNING: headset did not confirm pairing "
                        "mode — pairing may not work")
            time.sleep(5)
            hw.ensure_hid_bound(app.log)
            set_status("Headset in PAIRING MODE — pair controllers now")
            GLib.idle_add(done.set_sensitive, True)
        except Exception as e:
            app.log(f"Pairing error: {e}")
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
                    app.log(line.strip())
                m = re.search(r"Pairing Touch Controller (\w) .*finished",
                              line)
                if m:
                    side = m.group(1)
                    state["paired"].add(side)
                    if side in lr_rows:
                        GLib.idle_add(lr_rows[side].set, True, "paired ✓")
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
                hw.ensure_hid_bound(app.log)
                time.sleep(4)
            time.sleep(5)
            hw.ensure_hid_bound(app.log)
        except Exception as e:
            app.log(f"Cleanup error: {e}")
        finally:
            if app.ouvrtd_proc:
                app.ouvrtd_proc.terminate()
                try:
                    app.ouvrtd_proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    app.ouvrtd_proc.kill()
                    app.ouvrtd_proc.wait()
                app.ouvrtd_proc = None
            app.busy = False
            state["done"] = True
            app.log("Pairing session closed; headset back in normal "
                    "mode. Paired: " +
                    (", ".join(sorted(state["paired"])) or "none"))
            GLib.idle_add(dlg.close)
            GLib.idle_add(app.refresh_once)

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
