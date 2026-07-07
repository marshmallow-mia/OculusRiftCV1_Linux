"""GTK window for two-sensor room calibration.

Live view of an OHMD_RIFT_CAL_CAPTURE capture (per-device observation
rates, co-observation coverage against the solve targets, wake/visibility
hints) plus the offline solve/verify, wrapping calibrate_room.py.

The coverage math mirrors calibrate_room.py `coverage`: HMD-only,
observations with >= MIN_BLOBS LED-verified blobs, grouped by exposure
timestamp, counted when both sensors saw the same exposure."""
import json
import math
import os
import subprocess
import threading
import time
from collections import deque

import gi
gi.require_version("Gtk", "4.0")
gi.require_version("Gdk", "4.0")
gi.require_version("Adw", "1")
from gi.repository import Gdk, GLib, Gtk

from . import config, hw, runtime
from .widgets import StatusRow

MIN_BLOBS = 6
TARGETS = (("co-observed exposures", 1500),
           ("capture duration (min)", 3.0),
           ("25 cm positions visited", 30),
           ("yaw directions (of 12)", 8))
DEV_NAMES = {0: "HMD", 1: "ctrl L", 2: "ctrl R"}
RECENT_S = 3.0          # "live" means an observation within this window


def capture_path():
    """The capture file of the running vrserver (from its environment),
    else the default path. Second value: is capture enabled in vrserver?"""
    pid = hw.proc_pid("vrserver")
    if pid:
        p = hw.proc_env(pid, config.CAL_CAPTURE_ENV)
        if p:
            return os.path.expanduser(p), True
    return config.CAL_CAPTURE_DEFAULT, False


UI_STATE_FILE = os.path.join(os.path.dirname(config.CONFIG_FILE), "ui.json")


def _load_ui_state():
    try:
        with open(UI_STATE_FILE) as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def _save_ui_state(**kw):
    d = _load_ui_state()
    d.update(kw)
    os.makedirs(os.path.dirname(UI_STATE_FILE), exist_ok=True)
    with open(UI_STATE_FILE, "w") as f:
        json.dump(d, f)


class CaptureMonitor:
    """Incremental parser of a capture file; cheap to poll every second."""

    def __init__(self, path):
        self.path = path
        self.reset()

    def reset(self):
        self.pos = 0
        self.tail = b""
        self.n_sensors = 0
        self.counts = {}        # (device, sensor) -> total obs
        self.groups = {}        # exposure_ts -> set of sensors (HMD only)
        self.group_wp = {}      # exposure_ts -> world pose (first obs)
        self.co = set()         # co-observed exposure_ts
        self.voxels = set()
        self.yaws = set()
        self.recent = deque()   # (walltime, device, sensor)
        self.sensor_serials = {}  # sensor id -> serial
        # spatial data for the map (sanity-filtered: fusion glitches can
        # report the HMD tens of metres away)
        self.cells = {}         # (ix,iz) 25cm floor cell -> set of iy (co-obs)
        self.seen_xz = set()    # (ix,iz) any HMD obs, even single-sensor
        self.last_hmd = None    # (walltime, wp) newest HMD observation
        self.last_dist = {}     # sensor -> (walltime, HMD distance in m,
                                #            from the per-sensor PnP pose)

    def poll(self):
        try:
            size = os.path.getsize(self.path)
        except OSError:
            self.reset()
            return
        if size < self.pos:     # truncated / replaced
            self.reset()
        if size == self.pos:
            self._trim()
            return
        with open(self.path, "rb") as f:
            f.seek(self.pos)
            data = self.tail + f.read()
            self.pos = f.tell()
        lines = data.split(b"\n")
        self.tail = lines.pop()          # possibly-partial last line
        now = time.time()
        for line in lines:
            if not line.strip():
                continue
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            t = rec.get("t")
            if t == "sensor":
                self.n_sensors = max(self.n_sensors, rec["s"] + 1)
                self.sensor_serials[rec["s"]] = rec.get("serial", "?")
            elif t == "obs":
                key = (rec["d"], rec["s"])
                self.counts[key] = self.counts.get(key, 0) + 1
                self.recent.append((now, rec["d"], rec["s"]))
                if rec["d"] != 0 or len(rec["blobs"]) < MIN_BLOBS:
                    continue
                cam = rec.get("cam")
                if cam:
                    self.last_dist[rec["s"]] = (now, math.sqrt(
                        cam[0] ** 2 + cam[1] ** 2 + cam[2] ** 2))
                ts = rec["ts"]
                wp = rec["wp"]
                sane = (abs(wp[0]) < 5 and abs(wp[2]) < 5
                        and -1 < wp[1] < 3)
                if sane:
                    self.last_hmd = (now, wp)
                    self.seen_xz.add((int(wp[0] / 0.25),
                                      int(wp[2] / 0.25)))
                seen = self.groups.setdefault(ts, set())
                seen.add(rec["s"])
                self.group_wp.setdefault(ts, wp)
                if len(seen) >= 2 and ts not in self.co:
                    self.co.add(ts)
                    wp = self.group_wp[ts]
                    px, py, pz = wp[0:3]
                    qx, qy, qz, qw = wp[3:7]
                    yaw = math.degrees(math.atan2(
                        2 * (qw * qy + qx * qz),
                        1 - 2 * (qy * qy + qx * qx)))
                    self.voxels.add((int(px / 0.25), int(py / 0.25),
                                     int(pz / 0.25)))
                    self.yaws.add(int((yaw + 180) / 30) % 12)
                    if abs(px) < 5 and abs(pz) < 5:
                        self.cells.setdefault(
                            (int(px / 0.25), int(pz / 0.25)),
                            set()).add(int(py / 0.25))
        self._trim()

    def _trim(self):
        cut = time.time() - RECENT_S
        while self.recent and self.recent[0][0] < cut:
            self.recent.popleft()

    def snapshot(self):
        minutes = 0.0
        if len(self.co) >= 2:
            minutes = (max(self.co) - min(self.co)) / 60e9
        rates = {}
        for _, d, s in self.recent:
            rates[(d, s)] = rates.get((d, s), 0) + 1 / RECENT_S
        return {
            "total": sum(self.counts.values()),
            "counts": dict(self.counts),
            "rates": rates,
            "values": (len(self.co), minutes, len(self.voxels),
                       len(self.yaws)),
            "cells": {k: len(v) for k, v in self.cells.items()},
            "seen_xz": set(self.seen_xz),
            "yaws": set(self.yaws),
            "last_hmd": self.last_hmd,
            "dist": {s: d for s, (t, d) in self.last_dist.items()
                     if time.time() - t < RECENT_S},
            "serials": dict(self.sensor_serials),
        }


def _hint(snap, svr, enabled, n_usb_sensors):
    rates = snap["rates"]
    hmd_sensors = {s for (d, s) in rates if d == 0}
    if n_usb_sensors < 2:
        return ("Only one tracking sensor on USB — this calibration needs "
                "two.")
    if not svr:
        return ("Start SteamVR with capture enabled, then put the headset "
                "on and move\nslowly through the play space.")
    if not enabled:
        return ("SteamVR is running WITHOUT capture — add the launch "
                "option below in\nSteam (SteamVR ▸ Properties ▸ Launch "
                "Options), restart SteamVR.")
    done = all(v >= need for v, (_, need) in
               zip(snap["values"], TARGETS))
    if done:
        return ("DONE — coverage targets met. Quit SteamVR, then press "
                "'Solve & write config'.")
    if len(hmd_sensors) >= 2:
        return ("Both sensors see the headset — keep moving SLOWLY, vary "
                "position,\nheight and facing direction.")
    if len(hmd_sensors) == 1:
        return (f"Only sensor {next(iter(hmd_sensors))} sees the headset — "
                "face BETWEEN the two sensors\n(the front LEDs are "
                "directional).")
    if any(d in (1, 2) for (d, _s) in rates):
        return ("Controllers are tracked but the headset is not — put the "
                "headset ON\n(its LEDs stop flashing when it sleeps).")
    return ("No observations arriving — wear the headset (keeps it awake) "
            "and stay\nvisible to both sensors.")


# ---------------------------------------------------------------- drawing

CELL = 0.25                     # floor cell size (m), matches the tool
SENSOR_RANGE = 3.0              # drawn view-cone length (m)
SENSOR_HALF_FOV = math.radians(35)

GREEN = (0.20, 0.65, 0.30)
GREY = (0.50, 0.50, 0.50)
BLUE = (0.25, 0.50, 0.90)
ORANGE = (0.95, 0.55, 0.10)


def _yaw_of(wp):
    qx, qy, qz, qw = wp[3:7]
    return math.degrees(math.atan2(2 * (qw * qy + qx * qz),
                                   1 - 2 * (qy * qy + qx * qx)))


def _heading(yaw_deg):
    """Floor-plane facing vector (x, z) for a yaw angle."""
    y = math.radians(yaw_deg)
    return -math.sin(y), -math.cos(y)


def _sensor_layout(serials):
    """[(label, x, z, fx, fz)] — sensor floor positions and facing vectors
    from the room config, labelled with their capture sensor id."""
    try:
        with open(config.ROOM_CONFIG) as f:
            cfg = json.load(f)
    except (OSError, ValueError):
        return []
    id_by_serial = {v: k for k, v in serials.items()}
    out = []
    for i, s in enumerate(cfg.get("sensors", [])):
        try:
            x, _y, z = s["pos"]
            qx, qy, qz, qw = s["orient"]
        except (KeyError, TypeError, ValueError):
            continue
        # OpenCV convention: the camera looks along +Z of its local frame,
        # so the floor-plane facing vector is +R[:,2]
        fx = 2 * (qx * qz + qw * qy)
        fz = 1 - 2 * (qx * qx + qy * qy)
        n = math.hypot(fx, fz) or 1.0
        label = id_by_serial.get(s["serial"], i)
        out.append((f"S{label}", x, z, fx / n, fz / n))
    return out


def _draw_map(cr, w, h, snap, sensors, rot_deg=0.0):
    """Top-down play-space map: sensor cones, visited cells, headset.

    rot_deg rotates the VIEW only (the tracker's world yaw is arbitrary —
    it comes from the headset's pose at session start), so the user can
    align the diagram with their room.
    """
    th = math.radians(rot_deg)
    rc, rs = math.cos(th), math.sin(th)

    def R(x, z):
        return rc * x - rs * z, rs * x + rc * z

    # bounds over everything of interest (in rotated space),
    # min 3.5 m span, square, margin
    pts = [(0.0, 0.0)] + [(x, z) for _l, x, z, _fx, _fz in sensors]
    for ix, iz in set(snap["cells"]) | snap["seen_xz"]:
        pts.append((ix * CELL, iz * CELL))
    rxs, rzs = zip(*(R(x, z) for x, z in pts))
    x0, x1 = min(rxs) - 0.6, max(rxs) + 0.6
    z0, z1 = min(rzs) - 0.6, max(rzs) + 0.6
    span = max(x1 - x0, z1 - z0, 3.5)
    ctrx, ctrz = (x0 + x1) / 2, (z0 + z1) / 2
    x0, z0 = ctrx - span / 2, ctrz - span / 2
    scale = min(w, h) / span
    ox = (w - span * scale) / 2
    oz = (h - span * scale) / 2

    def P(x, z):
        xr, zr = R(x, z)
        return ox + (xr - x0) * scale, oz + (zr - z0) * scale

    cr.rectangle(0, 0, w, h)
    cr.clip()

    # world-aligned metre grid (rotates with the view)
    gx0, gx1 = min(x for x, _ in pts) - span, max(x for x, _ in pts) + span
    gz0, gz1 = min(z for _, z in pts) - span, max(z for _, z in pts) + span
    cr.set_line_width(1)
    cr.set_source_rgba(*GREY, 0.18)
    for m in range(int(gx0), int(gx1) + 2):
        cr.move_to(*P(m, gz0))
        cr.line_to(*P(m, gz1))
    for m in range(int(gz0), int(gz1) + 2):
        cr.move_to(*P(gx0, m))
        cr.line_to(*P(gx1, m))
    cr.stroke()

    # sensor view cones (overlap shows darker = the zone that counts)
    for _label, x, z, fx, fz in sensors:
        sx, sy = P(x, z)
        fxr, fzr = R(fx, fz)
        a = math.atan2(fzr, fxr)
        cr.set_source_rgba(*BLUE, 0.10)
        cr.move_to(sx, sy)
        cr.arc(sx, sy, SENSOR_RANGE * scale,
               a - SENSOR_HALF_FOV, a + SENSOR_HALF_FOV)
        cr.close_path()
        cr.fill()

    # visited cells: grey = one sensor only (does NOT count),
    # green = co-observed, brighter with more height levels covered
    def cell(ix, iz):
        x, z = ix * CELL, iz * CELL
        cr.move_to(*P(x, z))
        for dx, dz in ((CELL, 0), (CELL, CELL), (0, CELL)):
            cr.line_to(*P(x + dx, z + dz))
        cr.close_path()
        cr.fill()

    cr.set_source_rgba(*GREY, 0.30)
    for ix, iz in snap["seen_xz"] - set(snap["cells"]):
        cell(ix, iz)
    for (ix, iz), n_heights in snap["cells"].items():
        cr.set_source_rgba(*GREEN, (0.35, 0.60, 0.90)[min(n_heights, 3) - 1])
        cell(ix, iz)

    # origin cross (standing centre)
    sx, sy = P(0, 0)
    cr.set_source_rgba(*GREY, 0.8)
    cr.move_to(sx - 6, sy)
    cr.line_to(sx + 6, sy)
    cr.move_to(sx, sy - 6)
    cr.line_to(sx, sy + 6)
    cr.stroke()

    # sensors on top
    cr.set_font_size(11)
    for label, x, z, _fx, _fz in sensors:
        sx, sy = P(x, z)
        cr.set_source_rgba(*BLUE, 0.95)
        cr.arc(sx, sy, 5, 0, 2 * math.pi)
        cr.fill()
        cr.move_to(sx + 7, sy + 4)
        cr.show_text(label)

    # headset marker with heading arrow; grey when stale
    last = snap["last_hmd"]
    if last:
        age = time.time() - last[0]
        wp = last[1]
        col = ORANGE if age < RECENT_S else GREY
        sx, sy = P(wp[0], wp[2])
        fx, fz = R(*_heading(_yaw_of(wp)))
        cr.set_source_rgba(*col, 0.95)
        cr.arc(sx, sy, 8, 0, 2 * math.pi)
        cr.fill()
        cr.set_line_width(3.5)
        cr.move_to(sx, sy)
        cr.line_to(sx + fx * 24, sy + fz * 24)
        cr.stroke()


def _draw_compass(cr, w, h, snap, rot_deg=0.0):
    """12 yaw sectors: green = covered, outline = still missing.
    Rotated together with the map so both always agree."""
    th = math.radians(rot_deg)
    rc, rs = math.cos(th), math.sin(th)

    def R(x, z):
        return rc * x - rs * z, rs * x + rc * z

    cx, cy = w / 2, h / 2
    r = min(w, h) / 2 - 14
    covered = snap["yaws"]
    for k in range(12):
        yaw = -180 + 30 * k + 15          # bin centre
        fx, fz = R(*_heading(yaw))
        a = math.atan2(fz, fx)
        cr.move_to(cx, cy)
        cr.arc(cx, cy, r, a - math.radians(14), a + math.radians(14))
        cr.close_path()
        if k in covered:
            cr.set_source_rgba(*GREEN, 0.75)
            cr.fill()
        else:
            cr.set_source_rgba(*GREY, 0.45)
            cr.set_line_width(1)
            cr.stroke()
    # current heading needle
    last = snap["last_hmd"]
    if last and time.time() - last[0] < RECENT_S:
        fx, fz = R(*_heading(_yaw_of(last[1])))
        cr.set_source_rgba(*ORANGE, 0.95)
        cr.set_line_width(3)
        cr.move_to(cx, cy)
        cr.line_to(cx + fx * r, cy + fz * r)
        cr.stroke()
    cr.set_source_rgba(*GREY, 0.9)
    cr.set_font_size(12)
    txt = f"{len(covered)}/12"
    ext = cr.text_extents(txt)
    cr.move_to(cx - ext.width / 2, cy + h / 2 - 2)
    cr.show_text(txt)


def _ensure_solver_deps(log):
    venv_py = runtime.ensure_openvr_venv(log)
    if not venv_py:
        return None
    if subprocess.run([venv_py, "-c", "import numpy, scipy"],
                      capture_output=True).returncode != 0:
        log("installing numpy/scipy into the venv (one-time)…")
        r = subprocess.run([venv_py, "-m", "pip", "install", "--quiet",
                            "numpy", "scipy"], capture_output=True,
                           text=True)
        if r.returncode != 0:
            log("pip install failed: " + r.stderr.strip())
            return None
    return venv_py


def open_roomcal_window(app):
    if getattr(app, "roomcal_win", None):
        app.roomcal_win.present()
        return
    win = Gtk.Window(title="Room Calibration (two sensors)")
    win.set_transient_for(app.win)
    win.set_default_size(700, 940)
    app.roomcal_win = win

    box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
    for m in (box.set_margin_top, box.set_margin_bottom,
              box.set_margin_start, box.set_margin_end):
        m(14)
    win.set_child(box)

    # --- state rows
    lb = Gtk.ListBox()
    lb.add_css_class("boxed-list")
    lb.set_selection_mode(Gtk.SelectionMode.NONE)
    box.append(lb)
    row_sensors = StatusRow("Tracking sensors")
    row_svr = StatusRow("SteamVR")
    row_cap = StatusRow("Capture")
    for r in (row_sensors, row_svr, row_cap):
        lb.append(r)

    # --- launch-option helper (shown when capture is not enabled)
    launch_line = f"{config.CAL_CAPTURE_ENV}={config.CAL_CAPTURE_DEFAULT} %command%"
    lo_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
    lo_entry = Gtk.Entry(text=launch_line, editable=False, hexpand=True)
    lo_copy = Gtk.Button(label="Copy")
    lo_copy.connect("clicked", lambda _b: (
        Gdk.Display.get_default().get_clipboard().set(launch_line),
        app.toast("Launch option copied — paste it in Steam ▸ SteamVR ▸ "
                  "Properties ▸ Launch Options")))
    lo_box.append(lo_entry)
    lo_box.append(lo_copy)
    box.append(lo_box)

    # --- live map + yaw compass
    st = {"stop": threading.Event(), "mon": None, "running": False,
          "svr": False, "live": False, "enabled": False,
          "snap": None, "sensors": [],
          "rot": float(_load_ui_state().get("roomcal_rotation", 0.0))}

    map_area = Gtk.DrawingArea(hexpand=True, vexpand=True)
    map_area.set_content_height(340)
    map_frame = Gtk.Frame()
    map_frame.set_child(map_area)
    compass = Gtk.DrawingArea()
    compass.set_content_width(170)
    compass.set_content_height(170)
    comp_lbl = Gtk.Label(label="facing directions", xalign=0.5)
    comp_lbl.add_css_class("dim-label")
    right = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4,
                    valign=Gtk.Align.START)
    right.append(compass)
    right.append(comp_lbl)

    # view rotation — the tracker's yaw is arbitrary, so let the user
    # turn the diagram until it matches their room
    def set_rot(deg):
        st["rot"] = deg % 360.0
        _save_ui_state(roomcal_rotation=st["rot"])
        map_area.queue_draw()
        compass.queue_draw()

    rot_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=4,
                      halign=Gtk.Align.CENTER)
    b_ccw = Gtk.Button(label="⟲ 15°")
    b_ccw.set_tooltip_text("rotate the view counter-clockwise")
    b_ccw.connect("clicked", lambda _b: set_rot(st["rot"] - 15))
    b_cw = Gtk.Button(label="⟳ 15°")
    b_cw.set_tooltip_text("rotate the view clockwise")
    b_cw.connect("clicked", lambda _b: set_rot(st["rot"] + 15))
    rot_row.append(b_ccw)
    rot_row.append(b_cw)
    right.append(rot_row)
    b_face = Gtk.Button(label="My facing = up")
    b_face.set_tooltip_text("rotate the view so the direction the headset "
                            "currently faces points up on the map")
    right.append(b_face)
    rot_lbl = Gtk.Label(label="rotate view", xalign=0.5)
    rot_lbl.add_css_class("dim-label")
    right.append(rot_lbl)

    def align_face(_b):
        last = st["snap"] and st["snap"]["last_hmd"]
        if not last:
            app.toast("No headset position seen yet — wake the headset "
                      "in view of a sensor first")
            return
        fx, fz = _heading(_yaw_of(last[1]))
        set_rot(-90.0 - math.degrees(math.atan2(fz, fx)))
    b_face.connect("clicked", align_face)

    viz = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
    viz.append(map_frame)
    viz.append(right)
    box.append(viz)
    legend = Gtk.Label(
        xalign=0,
        label="■ green: counted (both sensors) — brighter = more heights "
              "(crouch/stand) · ■ grey: one sensor only, does not count · "
              "▲ blue: sensors & view cones · ● orange: you")
    legend.add_css_class("dim-label")
    legend.set_wrap(True)
    box.append(legend)

    def on_draw_map(_a, cr, w, h):
        if st["snap"]:
            _draw_map(cr, w, h, st["snap"], st["sensors"], st["rot"])

    def on_draw_compass(_a, cr, w, h):
        if st["snap"]:
            _draw_compass(cr, w, h, st["snap"], st["rot"])
    map_area.set_draw_func(on_draw_map)
    compass.set_draw_func(on_draw_compass)

    # --- coverage progress
    grid = Gtk.Grid(column_spacing=10, row_spacing=6)
    bars = []
    for i, (label, need) in enumerate(TARGETS):
        lbl = Gtk.Label(label=label, xalign=0)
        lbl.set_size_request(230, -1)
        bar = Gtk.ProgressBar(show_text=True, hexpand=True, valign=Gtk.Align.CENTER)
        grid.attach(lbl, 0, i, 1, 1)
        grid.attach(bar, 1, i, 1, 1)
        bars.append(bar)
    box.append(grid)

    # --- live activity
    act = Gtk.Label(xalign=0)
    act.add_css_class("monospace")
    box.append(act)
    hint = Gtk.Label(xalign=0, wrap=True)
    hint.add_css_class("dim-label")
    box.append(hint)

    # --- actions
    btns = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
    b_solve = Gtk.Button(label="Solve & write config")
    b_solve.add_css_class("suggested-action")
    b_verify = Gtk.Button(label="Verify current config")
    b_clear = Gtk.Button(label="Delete capture, start fresh")
    b_clear.add_css_class("destructive-action")
    for b in (b_solve, b_verify, b_clear):
        btns.append(b)
    box.append(btns)

    out_buf = Gtk.TextBuffer()
    out_tv = Gtk.TextView(buffer=out_buf, editable=False, monospace=True)
    for m in (out_tv.set_top_margin, out_tv.set_bottom_margin,
              out_tv.set_left_margin, out_tv.set_right_margin):
        m(8)
    sw = Gtk.ScrolledWindow(vexpand=True)
    sw.set_child(out_tv)
    sw.add_css_class("card")
    box.append(sw)

    def out_append(text):
        def _a():
            end = out_buf.get_end_iter()
            out_buf.insert(end, text if text.endswith("\n") else text + "\n")
            out_tv.scroll_to_iter(out_buf.get_end_iter(), 0.0, False,
                                  0.0, 1.0)
            return False
        GLib.idle_add(_a)

    # --- background: poll capture + system state every second
    def update_ui(svr, enabled, path, n_usb, snap):
        st["svr"], st["enabled"] = svr, enabled
        st["live"] = bool(snap["rates"])
        st["snap"] = snap
        map_area.queue_draw()
        compass.queue_draw()
        row_sensors.set(n_usb >= 2, f"{n_usb} connected"
                        + ("" if n_usb >= 2 else " — two needed"))
        row_svr.set(True if svr else None,
                    "running" if svr else "not running")
        if enabled:
            row_cap.set(True, "enabled → " + os.path.basename(path)
                        + ("  (receiving)" if snap["rates"] else "  (idle)"))
        elif svr:
            row_cap.set(False, "NOT enabled in this SteamVR session")
        else:
            row_cap.set(None, f"will read {os.path.basename(path)}")
        lo_box.set_visible(not enabled)

        for bar, (label, need), val in zip(bars, TARGETS, snap["values"]):
            frac = min(1.0, val / need)
            bar.set_fraction(frac)
            v = f"{val:.1f}" if isinstance(val, float) else str(val)
            bar.set_text(f"{v} / {need}")

        parts = []
        for d in (0, 1, 2):
            r = [f"s{s} {snap['rates'].get((d, s), 0):4.1f}/s"
                 for s in range(max(2, st['mon'].n_sensors))]
            parts.append(f"{DEV_NAMES[d]:6s} " + "  ".join(r))
        act.set_text("live observations:  " + "   |   ".join(parts)
                     + f"   (total {snap['total']})")
        hint.set_text(_hint(snap, svr, enabled, n_usb))

        can_tool = not st["running"] and os.path.exists(st["mon"].path)
        co = snap["values"][0]
        b_solve.set_sensitive(can_tool and not svr and co >= 50)
        if svr:
            b_solve.set_label("Solve & write config — quit SteamVR first")
        elif co < 50:
            b_solve.set_label(f"Solve & write config — need ≥50 "
                              f"co-observed, have {co}")
        else:
            b_solve.set_label("Solve & write config")
        b_verify.set_sensitive(can_tool)
        b_clear.set_sensitive(not st["running"] and not enabled
                              and os.path.exists(st["mon"].path))
        return False

    def monitor_loop():
        while not st["stop"].is_set():
            path, enabled = capture_path()
            if st["mon"] is None or st["mon"].path != path:
                st["mon"] = CaptureMonitor(path)
            st["mon"].poll()
            st["sensors"] = _sensor_layout(st["mon"].sensor_serials)
            GLib.idle_add(update_ui, hw.proc_running("vrserver"), enabled,
                          path, len(hw.tracking_sensors()),
                          st["mon"].snapshot())
            st["stop"].wait(0.5)

    # --- solve / verify subprocesses (streamed)
    def run_tool(mode):
        if st["running"]:
            return
        st["running"] = True
        for b in (b_solve, b_verify, b_clear):
            b.set_sensitive(False)
        out_buf.set_text("")

        def worker():
            try:
                venv_py = _ensure_solver_deps(out_append)
                if not venv_py:
                    out_append("cannot run the solver — venv setup failed")
                    return
                cmd = [venv_py, config.CAL_TOOL, mode, st["mon"].path]
                out_append("$ " + " ".join(cmd))
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True)
                for line in proc.stdout:
                    out_append(line.rstrip())
                proc.wait()
                if mode == "solve" and proc.returncode == 0:
                    out_append("\nDone. Restart SteamVR to pick up the new "
                               "calibration.")
                elif proc.returncode != 0:
                    out_append(f"\n{mode} exited with code {proc.returncode}")
            except Exception as e:
                out_append(f"error: {e}")
            finally:
                st["running"] = False
        threading.Thread(target=worker, daemon=True).start()

    def clear_capture(_b):
        path = st["mon"].path
        try:
            os.rename(path, path + ".old-" + time.strftime("%Y%m%d-%H%M%S"))
            out_append(f"capture moved aside; a fresh {path} starts with "
                       "the next SteamVR launch")
        except OSError as e:
            out_append(f"could not move capture: {e}")
        st["mon"].reset()

    b_solve.connect("clicked", lambda _b: run_tool("solve"))
    b_verify.connect("clicked", lambda _b: run_tool("verify"))
    b_clear.connect("clicked", clear_capture)

    def on_close(_w):
        st["stop"].set()
        app.roomcal_win = None
        return False
    win.connect("close-request", on_close)

    threading.Thread(target=monitor_loop, daemon=True).start()
    win.present()
