"""Sensor Setup wizard — the Windows Oculus app's CV1 sensor setup, 1:1.

Page order, titles and copy replicate the real client's Multi-Sensor
Setup module (extracted from the Oculus/Meta Horizon PC app bundle,
`app.asar`, which still ships the CV1 flow). Its step machine runs:

  bandwidth_check -> multi_sensor_intro -> sensor_height ->
  multi_sensor_prepare_space -> place_two_sensors ->
  sensor_tracking_intro -> multi_sensor_tracking -> (guardian, in VR)

mapped here onto eight pages:

  1. Sensor Connection            USB / hardware check
  2. Set Up Your Oculus Sensors   intro / remove the lens film
  3. Height                       sets the floor position in VR
  4. Clear Your Play Area         safety
  5. Place Your Sensors           placement guidance
  6. Set Up Sensor Tracking       move the headset through the play
                                  area; locates the sensors
  7. Confirm Sensor Tracking      stand at the centre facing the
                                  sensors (with the Oculus distance
                                  checks); sets floor/centre/forward
  8. Sensor Tracking Confirmed    top-down view of the result

Guardian is not part of sensor setup (the real client draws it in VR
with a Touch controller).

Mechanics: steps 6 and 7 each run a short standalone OpenHMD session
(openhmd_simple_example) with OHMD_RIFT_CAL_CAPTURE pointing at a phase
file; `calibrate_room.py setup` then solves the sensor extrinsics from
the tracking capture and re-anchors the world at the standing centre
(height-based floor, exactly like the Oculus app), writing
rift-room-config.json. SteamVR must be closed throughout — like the
Oculus app, setup owns the hardware while it runs."""
import math
import os
import subprocess
import threading
import time

import cairo
import gi
gi.require_version("Gtk", "4.0")
gi.require_version("Gdk", "4.0")
from gi.repository import Gdk, GLib, Gtk

from . import config, hw
from .roomcal import CaptureMonitor, _ensure_solver_deps, _sensor_layout

TRACK_TARGET = 300      # co-observed exposures (2+ sensors) / HMD obs (1)
STAND_TARGET = 150      # HMD observations standing at the centre
PAGES = 8

# Oculus distance validation for the confirm step
DIST_TOO_CLOSE = 0.7    # m
DIST_TOO_FAR = 2.9      # m
DIST_RATIO_MAX = 1.8    # farthest / nearest sensor

INK = (0.91, 0.92, 0.95)
DIM = (0.46, 0.49, 0.58)
BLUE = (0.10, 0.47, 0.95)
GREEN = (0.23, 0.83, 0.50)
ORANGE = (0.95, 0.62, 0.35)
TRACK = (0.20, 0.22, 0.30)

# palette and metrics from the client's own nux CSS:
#   .nux-background  #1c1e20, gradient from #323436
#   headings #ffffff (2.67rem), body rgba(255,255,255,0.6) (1.25rem)
#   .button  border-radius 0.333rem, weight 600; blue #0880fa,
#            hover/active #2792ff, disabled #005bb7
_CSS = b"""
window.oculus-setup { background-color: #1c1e20;
    background-image: linear-gradient(180deg, #323436, #1c1e20 70%);
    color: #ffffff; }
.oc-step   { color: rgba(255,255,255,0.6); font-size: 12px;
             font-weight: 700; letter-spacing: 3px; }
.oc-title  { color: #ffffff; font-size: 32px; font-weight: 400; }
.oc-body   { color: rgba(255,255,255,0.6); font-size: 15px; }
.oc-status { color: #ffffff; font-size: 15px; }
.oc-warn   { color: #f0a35e; }
.oc-ok     { color: #3bd47f; font-weight: 700; }
.oc-bad    { color: #f2695c; font-weight: 700; }
.oc-dim    { color: rgba(255,255,255,0.6); }
.oc-row-name { color: #ffffff; }
.oc-row { background-color: #323436; border-radius: 4px;
          padding: 12px 16px; }
button.oc-continue { background-image: none; background-color: #0880fa;
                     color: #ffffff; border-radius: 4px;
                     padding: 12px 40px; font-weight: 600;
                     font-size: 15px; border: none; box-shadow: none; }
button.oc-continue:hover { background-color: #2792ff; }
button.oc-continue:disabled { background-color: #005bb7;
                              color: rgba(255,255,255,0.55); }
button.oc-back { background-image: none; background-color: transparent;
                 border: none; box-shadow: none;
                 color: rgba(255,255,255,0.6); }
button.oc-back:hover { color: #ffffff;
                       background-color: rgba(255,255,255,0.08); }
.oculus-setup spinbutton { background-color: #323436; color: #ffffff;
                           border: none; border-radius: 4px; }
.oculus-setup spinbutton entry,
.oculus-setup spinbutton text { background-color: #323436;
                                color: #ffffff; }
.oculus-setup spinbutton button { background-color: #46484a;
                                  color: #ffffff; border: none; }
.oculus-setup textview.oc-log, .oculus-setup textview.oc-log text {
    background-color: #141618; color: rgba(255,255,255,0.6); }
.oculus-setup expander-widget title label {
    color: rgba(255,255,255,0.6); }
"""


def _install_css():
    disp = Gdk.Display.get_default()
    if getattr(_install_css, "done", False) or disp is None:
        return
    prov = Gtk.CssProvider()
    prov.load_from_data(_CSS)
    Gtk.StyleContext.add_provider_for_display(
        disp, prov, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
    _install_css.done = True


# ---------------------------------------------------------------- drawing

def _rounded(cr, x, y, w, h, r):
    cr.new_sub_path()
    cr.arc(x + w - r, y + r, r, -math.pi / 2, 0)
    cr.arc(x + w - r, y + h - r, r, 0, math.pi / 2)
    cr.arc(x + r, y + h - r, r, math.pi / 2, math.pi)
    cr.arc(x + r, y + r, r, math.pi, 3 * math.pi / 2)
    cr.close_path()


def _sensor(cr, x, y, h, col, alpha=1.0):
    """Line-art CV1 sensor: base ellipse, stem, capsule head, lens dot.
    (x, y) is the centre of the base; h the overall height."""
    cr.set_source_rgba(*col, alpha)
    cr.set_line_width(2)
    cr.save()
    cr.translate(x, y)
    cr.scale(1.0, 0.32)
    cr.arc(0, 0, h * 0.17, 0, 2 * math.pi)
    cr.restore()
    cr.stroke()
    cr.move_to(x, y - h * 0.03)
    cr.line_to(x, y - h * 0.46)
    cr.stroke()
    cw, ch = h * 0.135, h * 0.50
    _rounded(cr, x - cw, y - h * 0.46 - ch, 2 * cw, ch, cw)
    cr.stroke()
    cr.arc(x, y - h * 0.46 - ch * 0.70, h * 0.05, 0, 2 * math.pi)
    cr.fill()


def _hmd_front(cr, cx, cy, w, col, alpha=1.0):
    """Line-art headset, front view: rounded body, strap stubs, logo."""
    h = w * 0.60
    cr.set_source_rgba(*col, alpha)
    cr.set_line_width(2)
    _rounded(cr, cx - w / 2, cy - h / 2, w, h, h * 0.30)
    cr.stroke()
    for sgn in (-1, 1):
        cr.move_to(cx + sgn * w / 2, cy - h * 0.16)
        cr.line_to(cx + sgn * (w / 2 + w * 0.14), cy - h * 0.24)
        cr.move_to(cx + sgn * w / 2, cy + h * 0.16)
        cr.line_to(cx + sgn * (w / 2 + w * 0.14), cy + h * 0.24)
        cr.stroke()
    cr.save()
    cr.translate(cx, cy)
    cr.scale(1.0, 0.62)
    cr.arc(0, 0, w * 0.085, 0, 2 * math.pi)
    cr.restore()
    cr.stroke()


def _person(cr, x, floor_y, h, col, hmd=True):
    """Line-art person standing on the floor line, wearing the headset."""
    cr.set_source_rgba(*col)
    cr.set_line_width(2)
    head_r = h * 0.075
    head_y = floor_y - h + head_r
    cr.arc(x, head_y, head_r, 0, 2 * math.pi)
    cr.stroke()
    if hmd:
        cr.set_line_width(3)
        cr.move_to(x - head_r * 1.15, head_y - head_r * 0.15)
        cr.line_to(x + head_r * 1.15, head_y - head_r * 0.15)
        cr.stroke()
        cr.set_line_width(2)
    neck_y = head_y + head_r
    hip_y = floor_y - h * 0.42
    cr.move_to(x, neck_y)
    cr.line_to(x, hip_y)
    cr.stroke()
    cr.move_to(x - h * 0.16, neck_y + h * 0.16)
    cr.line_to(x, neck_y + h * 0.05)
    cr.line_to(x + h * 0.16, neck_y + h * 0.16)
    cr.stroke()
    cr.move_to(x - h * 0.10, floor_y)
    cr.line_to(x, hip_y)
    cr.line_to(x + h * 0.10, floor_y)
    cr.stroke()


def _dashed(cr, on=5.0, off=5.0):
    cr.set_dash([on, off])


def _text_centered(cr, x, y, txt, size=12, col=DIM):
    cr.set_source_rgba(*col)
    cr.set_font_size(size)
    ext = cr.text_extents(txt)
    cr.move_to(x - ext.width / 2, y)
    cr.show_text(txt)


def draw_check_scene(cr, w, h, n_sensors):
    """Sensor Connection: the detected sensors, softly lit."""
    floor_y = h * 0.86
    sh = h * 0.55
    n = max(n_sensors, 1)
    xs = [w * (0.5 + (i - (n - 1) / 2) * 0.28) for i in range(n)]
    for i, x in enumerate(xs):
        _sensor(cr, x, floor_y, sh, INK if i < n_sensors else DIM,
                1.0 if i < n_sensors else 0.5)


def draw_intro(cr, w, h):
    """Intro: one sensor up close, its lens (glossy side) called out."""
    floor_y = h * 0.88
    sh = h * 0.72
    x = w * 0.42
    _sensor(cr, x, floor_y, sh, INK)
    lens_y = floor_y - sh * 0.46 - sh * 0.50 * 0.70
    cr.set_source_rgba(*BLUE, 0.9)
    cr.set_line_width(1.5)
    _dashed(cr, 3, 3)
    cr.arc(x, lens_y, sh * 0.13, 0, 2 * math.pi)
    cr.stroke()
    cr.set_dash([])
    cr.move_to(x + sh * 0.13, lens_y)
    cr.line_to(w * 0.62, lens_y)
    cr.stroke()
    cr.set_source_rgba(*DIM)
    cr.set_font_size(12)
    cr.move_to(w * 0.63, lens_y + 4)
    cr.show_text("Sensor Lens (Glossy Side)")


def draw_height(cr, w, h):
    """Height: person with a dashed height measure beside them."""
    floor_y = h * 0.86
    cr.set_source_rgba(*DIM, 0.7)
    cr.set_line_width(1.5)
    cr.move_to(w * 0.22, floor_y)
    cr.line_to(w * 0.78, floor_y)
    cr.stroke()
    ph = h * 0.66
    _person(cr, w * 0.46, floor_y, ph, INK, hmd=False)
    hx = w * 0.58
    cr.set_source_rgba(*BLUE, 0.9)
    cr.set_line_width(1.5)
    _dashed(cr, 3, 3)
    cr.move_to(hx, floor_y)
    cr.line_to(hx, floor_y - ph)
    cr.stroke()
    cr.set_dash([])
    for y in (floor_y, floor_y - ph):
        cr.move_to(hx - 6, y)
        cr.line_to(hx + 6, y)
        cr.stroke()


def draw_prepare(cr, w, h):
    """Clear Your Play Area: dashed safe zone with you at the centre."""
    # perspective floor quad
    x0, x1 = w * 0.18, w * 0.82
    y0, y1 = h * 0.42, h * 0.88
    inset = w * 0.12
    cr.set_source_rgba(*BLUE, 0.75)
    cr.set_line_width(1.8)
    _dashed(cr, 6, 5)
    cr.move_to(x0 + inset, y0)
    cr.line_to(x1 - inset, y0)
    cr.line_to(x1, y1)
    cr.line_to(x0, y1)
    cr.close_path()
    cr.stroke()
    cr.set_dash([])
    _person(cr, w * 0.5, h * 0.72, h * 0.38, INK)


def draw_placement(cr, w, h):
    """Place Your Sensors: two sensors, view cones, distance arrow."""
    floor_y = h * 0.80
    cr.set_source_rgba(*DIM, 0.7)
    cr.set_line_width(1.5)
    cr.move_to(w * 0.08, floor_y)
    cr.line_to(w * 0.92, floor_y)
    cr.stroke()

    sh = h * 0.42
    x1, x2 = w * 0.32, w * 0.68
    for x in (x1, x2):
        _sensor(cr, x, floor_y, sh, INK)

    # translucent view cones fanning from each lens into the play area
    py = h * 1.05
    for x in (x1, x2):
        lens_y = floor_y - sh * 0.80
        cx = x + (w * 0.5 - x) * 0.55       # lean toward the centre
        cr.set_source_rgba(*BLUE, 0.10)
        cr.move_to(x, lens_y)
        cr.line_to(cx - w * 0.14, py)
        cr.line_to(cx + w * 0.14, py)
        cr.close_path()
        cr.fill()
        cr.set_source_rgba(*BLUE, 0.45)
        cr.set_line_width(1.5)
        _dashed(cr)
        for dx in (-w * 0.14, w * 0.14):
            cr.move_to(x, lens_y)
            cr.line_to(cx + dx, py)
            cr.stroke()
        cr.set_dash([])

    ay = floor_y - sh * 1.12
    cr.set_source_rgba(*DIM)
    cr.set_line_width(1.5)
    cr.move_to(x1 + 8, ay)
    cr.line_to(x2 - 8, ay)
    cr.stroke()
    for x, sgn in ((x1 + 8, 1), (x2 - 8, -1)):
        cr.move_to(x, ay)
        cr.line_to(x + sgn * 7, ay - 4)
        cr.move_to(x, ay)
        cr.line_to(x + sgn * 7, ay + 4)
        cr.stroke()
    _text_centered(cr, w / 2, ay - 8, "3 – 6 ft  (1 – 2 m)")


def draw_tracking(cr, w, h, frac, state, live, n_usb):
    """Set Up Sensor Tracking: progress ring around the headset, motion
    arrows, one dot per sensor lit while that sensor sees the headset."""
    cx, cy = w / 2, h * 0.46
    r = min(h * 0.36, w * 0.22)
    cr.set_line_width(7)
    cr.set_line_cap(cairo.LINE_CAP_ROUND)
    cr.set_source_rgba(*TRACK)
    cr.arc(cx, cy, r, 0, 2 * math.pi)
    cr.stroke()
    if frac > 0:
        col = GREEN if state == "done" else BLUE
        cr.set_source_rgba(*col)
        cr.arc(cx, cy, r, -math.pi / 2, -math.pi / 2 + 2 * math.pi *
               min(frac, 1.0))
        cr.stroke()
    _hmd_front(cr, cx, cy, r * 0.95,
               GREEN if state == "done" else INK)
    if state != "done":
        cr.set_source_rgba(*DIM)
        cr.set_line_width(2)
        for sgn in (-1, 1):
            ax = cx + sgn * r * 0.72
            cr.move_to(cx + sgn * r * 0.58, cy)
            cr.line_to(ax, cy)
            cr.stroke()
            cr.move_to(ax - sgn * 6, cy - 5)
            cr.line_to(ax, cy)
            cr.line_to(ax - sgn * 6, cy + 5)
            cr.stroke()
    if state == "done":
        _text_centered(cr, cx, cy + r + 26, "✓", 22, GREEN)
    else:
        _text_centered(cr, cx, cy + r + 26, f"{int(min(frac, 1) * 100)} %",
                       14, DIM)

    for i in range(n_usb):
        x = cx + (i - (n_usb - 1) / 2) * 26
        y = cy + r + 52
        cr.new_path()                # no connector from previous drawing
        if i in live:
            cr.set_source_rgba(*GREEN)
            cr.arc(x, y, 4.5, 0, 2 * math.pi)
            cr.fill()
        else:
            cr.set_source_rgba(*DIM)
            cr.set_line_width(1.5)
            cr.arc(x, y, 4.5, 0, 2 * math.pi)
            cr.stroke()


def draw_confirm(cr, w, h, frac, state):
    """Confirm Sensor Tracking: person at the centre between the sensors,
    progress bar while the anchor capture / solve runs."""
    floor_y = h * 0.72
    cr.set_source_rgba(*DIM, 0.7)
    cr.set_line_width(1.5)
    cr.move_to(w * 0.10, floor_y)
    cr.line_to(w * 0.90, floor_y)
    cr.stroke()
    sh = h * 0.34
    for x in (w * 0.24, w * 0.76):
        _sensor(cr, x, floor_y, sh, INK, 0.85)
    _person(cr, w * 0.5, floor_y, h * 0.52,
            GREEN if state == "solved" else INK)
    # equal-distance hints from each sensor to the person
    cr.set_line_width(1.2)
    _dashed(cr, 3, 4)
    cr.set_source_rgba(*BLUE, 0.5)
    for x in (w * 0.24, w * 0.76):
        cr.move_to(x, floor_y - sh * 0.9)
        cr.line_to(w * 0.5, floor_y - h * 0.30)
        cr.stroke()
    cr.set_dash([])

    bw, bh = w * 0.46, 6.0
    bx, by = (w - bw) / 2, h * 0.88
    if state in ("capturing", "solving", "solved"):
        cr.set_source_rgba(*TRACK)
        _rounded(cr, bx, by, bw, bh, bh / 2)
        cr.fill()
        f = 1.0 if state in ("solving", "solved") else min(frac, 1.0)
        if f > 0.02:
            cr.set_source_rgba(*(GREEN if state == "solved" else BLUE))
            _rounded(cr, bx, by, bw * f, bh, bh / 2)
            cr.fill()


def draw_complete(cr, w, h, layout):
    """Sensor Tracking Confirmed: green check + top-down setup view."""
    cx = w / 2
    r = 26
    cy = h * 0.16
    cr.set_source_rgba(*GREEN)
    cr.set_line_width(3)
    cr.arc(cx, cy, r, 0, 2 * math.pi)
    cr.stroke()
    cr.move_to(cx - r * 0.42, cy + r * 0.05)
    cr.line_to(cx - r * 0.08, cy + r * 0.40)
    cr.line_to(cx + r * 0.48, cy - r * 0.32)
    cr.stroke()

    if not layout:
        return
    # top-down map: +X right, +Z down, so forward (-Z) points up
    pts = [(0.0, 0.0)] + [(x, z) for _l, x, z, _fx, _fz in layout]
    xs, zs = zip(*pts)
    span = max(max(xs) - min(xs), max(zs) - min(zs), 2.5) + 1.2
    scale = min(w * 0.7, h * 0.62) / span
    ox = cx - (min(xs) + max(xs)) / 2 * scale
    oy = h * 0.62 - (min(zs) + max(zs)) / 2 * scale

    def P(x, z):
        return ox + x * scale, oy + z * scale

    for label, x, z, fx, fz in layout:
        sx, sy = P(x, z)
        a = math.atan2(fz, fx)
        cr.set_source_rgba(*BLUE, 0.12)
        cr.move_to(sx, sy)
        cr.arc(sx, sy, 2.2 * scale, a - math.radians(32),
               a + math.radians(32))
        cr.close_path()
        cr.fill()
    cr.set_font_size(11)
    for label, x, z, _fx, _fz in layout:
        sx, sy = P(x, z)
        cr.set_source_rgba(*BLUE)
        cr.arc(sx, sy, 5, 0, 2 * math.pi)
        cr.fill()
        cr.set_source_rgba(*DIM)
        cr.move_to(sx + 8, sy + 4)
        cr.show_text(str(label))
    sx, sy = P(0, 0)
    cr.set_source_rgba(*ORANGE)
    cr.arc(sx, sy, 6, 0, 2 * math.pi)
    cr.fill()
    cr.set_line_width(2.5)
    cr.move_to(sx, sy)
    cr.line_to(sx, sy - 18)
    cr.stroke()
    cr.move_to(sx - 5, sy - 12)
    cr.line_to(sx, sy - 18)
    cr.line_to(sx + 5, sy - 12)
    cr.stroke()


# ---------------------------------------------------------------- widgets

class CheckRow(Gtk.Box):
    """Oculus-style check row: glyph, name, right-aligned detail."""

    def __init__(self, name):
        super().__init__(orientation=Gtk.Orientation.HORIZONTAL, spacing=12)
        self.add_css_class("oc-row")
        self.glyph = Gtk.Label(label="•")
        self.glyph.set_width_chars(2)
        self.glyph.add_css_class("oc-dim")
        self.name = Gtk.Label(label=name, xalign=0, hexpand=True)
        self.name.add_css_class("oc-row-name")
        self.detail = Gtk.Label(label="", xalign=1)
        self.detail.add_css_class("oc-dim")
        for widget in (self.glyph, self.name, self.detail):
            self.append(widget)

    def set(self, ok, detail, name=None):
        for c in ("oc-ok", "oc-bad", "oc-dim"):
            self.glyph.remove_css_class(c)
        glyph, cls = {True: ("✓", "oc-ok"), False: ("✕", "oc-bad"),
                      None: ("•", "oc-dim")}[ok]
        self.glyph.set_text(glyph)
        self.glyph.add_css_class(cls)
        if name is not None:
            self.name.set_text(name)
        self.detail.set_text(detail)


class PhaseCapture:
    """One short standalone tracking session writing a capture file."""

    def __init__(self, path):
        self.path = path
        self.proc = None
        self.mon = CaptureMonitor(path)

    def start(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        try:
            os.remove(self.path)
        except OSError:
            pass
        self.mon.reset()
        env = dict(os.environ)
        env[config.CAL_CAPTURE_ENV] = self.path
        self.proc = subprocess.Popen(
            [config.OPENHMD_EXAMPLE, "0"], env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def poll(self):
        self.mon.poll()
        return self.mon.snapshot()

    def stop(self):
        proc, self.proc = self.proc, None
        if proc is None:
            return
        try:
            proc.terminate()
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        except OSError:
            pass


# ---------------------------------------------------------------- wizard

STEP_LABEL = "SENSOR SETUP"

# titles and body copy verbatim from the Oculus PC app's CV1 sensor setup
COPY = (
    ("Sensor Connection",
     "Let's make sure that your sensors are ready to support tracking. "
     "Be sure no more than two sensors are connected to USB 3 ports."),
    ("Set Up Your Oculus Sensors",
     "Your sensors work with your headset and Touch controllers to track "
     "your movement in VR. To get started, remove the protective film on "
     "the sensor lenses."),
    ("Height",
     "Enter your height. This lets us set the floor position in VR, to "
     "make experiences feel even more real. Don't worry about how tall "
     "other people who use your Rift might be. We only need the height "
     "of the person wearing the headset during setup."),
    ("Clear Your Play Area",
     "Clear your play area and be aware of your surroundings. For "
     "safety, be sure to route cables outside of your planned play "
     "area."),
    ("Place Your Sensors",
     "Place your sensors where you'll be using your Rift, at least 3 "
     "feet (1 meter) away from you. Rotate and tilt your sensors so the "
     "glossy side is pointing into your play area."),
    ("Set Up Sensor Tracking",
     "Get your headset and go to where you'll be using VR. Move the "
     "headset from side to side, down towards the floor, then in front "
     "of your head."),
    ("Confirm Sensor Tracking",
     "Move to the center of your play area and face the sensors. Hold "
     "your headset at eye level (or wear it), then select Continue and "
     "hold still."),
    ("Sensor Tracking Confirmed",
     "Great! Your sensors are now tracking correctly. Your sensors and "
     "headset are ready to use — restart SteamVR to pick up the new "
     "calibration."),
)

P_CHECK, P_INTRO, P_HEIGHT, P_PREPARE, P_PLACE, P_TRACK, P_CONFIRM, \
    P_DONE = range(8)


def open_sensor_setup(app):
    if getattr(app, "sensorsetup_win", None):
        app.sensorsetup_win.present()
        return
    _install_css()

    win = Gtk.Window(title="Sensor Setup")
    win.set_transient_for(app.win)
    win.set_default_size(880, 660)
    win.add_css_class("oculus-setup")
    app.sensorsetup_win = win

    st = {"page": 0, "closed": False, "timer": None,
          "phase": None, "stop_phase": threading.Event(),
          "trk": {"frac": 0.0, "state": "idle", "live": set(), "n_usb": 0},
          "stand": {"frac": 0.0, "state": "idle", "dists": {}},
          "layout": [], "check_ok": False, "warns": []}

    root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
    win.set_child(root)

    # top bar, like the client's nux title: back arrow, then the setup
    # name between two thin lines that fill left-to-right with progress
    top = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
    for m in (top.set_margin_top, top.set_margin_start,
              top.set_margin_end):
        m(10)
    back = Gtk.Button(icon_name="go-previous-symbolic")
    back.add_css_class("oc-back")
    top.append(back)

    def line_draw(side):
        def draw(_a, cr, w, h):
            frac = st["page"] / (PAGES - 1)
            fill = min(1.0, frac * 2) if side == 0 else \
                max(0.0, frac * 2 - 1)
            y = h / 2
            cr.set_line_width(2)
            cr.set_source_rgba(1, 1, 1, 0.25)
            cr.move_to(0, y)
            cr.line_to(w, y)
            cr.stroke()
            if fill > 0:
                cr.set_source_rgba(1, 1, 1, 0.95)
                cr.move_to(0, y)
                cr.line_to(w * fill, y)
                cr.stroke()
        return draw

    line_l = Gtk.DrawingArea(hexpand=True, valign=Gtk.Align.CENTER)
    line_l.set_content_height(4)
    line_l.set_draw_func(line_draw(0))
    top_title = Gtk.Label(label=STEP_LABEL)
    top_title.add_css_class("oc-step")
    line_r = Gtk.DrawingArea(hexpand=True, valign=Gtk.Align.CENTER)
    line_r.set_content_height(4)
    line_r.set_draw_func(line_draw(1))
    top.append(line_l)
    top.append(top_title)
    top.append(line_r)
    # symmetry spacer matching the back button's width
    spacer = Gtk.Box()
    spacer.set_size_request(34, -1)
    top.append(spacer)
    root.append(top)

    stack = Gtk.Stack(vexpand=True, hexpand=True)
    stack.set_transition_type(Gtk.StackTransitionType.CROSSFADE)
    stack.set_transition_duration(220)
    root.append(stack)

    bottom = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
    bottom.set_margin_bottom(26)
    cont = Gtk.Button(label="Continue", halign=Gtk.Align.CENTER)
    cont.add_css_class("oc-continue")
    bottom.append(cont)
    root.append(bottom)

    def page_box(idx):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=14,
                      halign=Gtk.Align.CENTER)
        box.set_margin_top(14)
        title = Gtk.Label(label=COPY[idx][0])
        title.add_css_class("oc-title")
        body = Gtk.Label(label=COPY[idx][1], wrap=True,
                         justify=Gtk.Justification.CENTER)
        body.add_css_class("oc-body")
        body.set_max_width_chars(62)
        for widget in (title, body):
            box.append(widget)
        stack.add_named(box, f"p{idx}")
        return box

    # ---- page 1: sensor connection (bandwidth check)
    p_check = page_box(P_CHECK)
    art_check = Gtk.DrawingArea(halign=Gtk.Align.CENTER)
    art_check.set_content_width(360)
    art_check.set_content_height(120)
    art_check.set_draw_func(lambda _a, cr, w, h:
                            draw_check_scene(cr, w, h,
                                             st["trk"]["n_usb"]))
    p_check.append(art_check)
    rows_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
    rows_box.set_size_request(460, -1)
    row_hmd = CheckRow("Headset")
    row_s = [CheckRow("Sensor 1"), CheckRow("Sensor 2")]
    row_svr = CheckRow("SteamVR")
    row_drv = CheckRow("OpenHMD driver")
    for r in (row_hmd, *row_s, row_svr, row_drv):
        rows_box.append(r)
    p_check.append(rows_box)

    def poll_check():
        if st["closed"] or st["page"] != P_CHECK:
            st["timer"] = None
            return False
        sensors = hw.tracking_sensors()
        st["trk"]["n_usb"] = len(sensors)
        hid = hw.hmd_hid_state()
        svr = hw.proc_running("vrserver")
        built = os.path.exists(config.OPENHMD_EXAMPLE)

        row_hmd.set({"bound": True, "in-use": False,
                     "unbound": False}.get(hid, False),
                    {"bound": "Connected",
                     "in-use": "In use by another VR session — close it",
                     "unbound": "USB issue — use Fix USB in the control "
                                "center"}.get(hid, "Not found"))
        for i, row in enumerate(row_s):
            if i < len(sensors):
                s = sensors[i]
                spd = s["speed"] or 0
                serial = s["serial"] or "?"
                if spd >= 5000:
                    row.set(True, "Connected · USB 3.0",
                            name=f"Sensor {i + 1}  ({serial})")
                else:
                    row.set(None, "USB 2 — move it to a USB 3 port",
                            name=f"Sensor {i + 1}  ({serial})")
            elif i == 0:
                row.set(False, "No sensor connected", name="Sensor 1")
            else:
                row.set(None, "Optional — a second sensor improves "
                              "tracking", name="Sensor 2")
        row_svr.set_visible(svr)
        if svr:
            row_svr.set(False, "Running — close SteamVR to continue")
        row_drv.set_visible(not built)
        if not built:
            row_drv.set(False, "Not built — run Setup & install first")

        st["check_ok"] = (hid == "bound" and len(sensors) >= 1
                          and not svr and built)
        cont.set_sensitive(st["check_ok"])
        art_check.queue_draw()
        return True

    # ---- page 2: set up your oculus sensors (intro)
    p_intro = page_box(P_INTRO)
    art_intro = Gtk.DrawingArea(halign=Gtk.Align.CENTER)
    art_intro.set_content_width(560)
    art_intro.set_content_height(230)
    art_intro.set_draw_func(lambda _a, cr, w, h: draw_intro(cr, w, h))
    p_intro.append(art_intro)

    # ---- page 3: height
    p_height = page_box(P_HEIGHT)
    art_height = Gtk.DrawingArea(halign=Gtk.Align.CENTER)
    art_height.set_content_width(420)
    art_height.set_content_height(200)
    art_height.set_draw_func(lambda _a, cr, w, h: draw_height(cr, w, h))
    p_height.append(art_height)
    hrow = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10,
                   halign=Gtk.Align.CENTER)
    hlabel = Gtk.Label(label="Height")
    hlabel.add_css_class("oc-status")
    hspin = Gtk.SpinButton.new_with_range(120, 220, 1)
    hspin.set_value(175)
    himp = Gtk.Label(label="")
    himp.add_css_class("oc-dim")

    def update_imperial(*_a):
        cm = hspin.get_value()
        inches = cm / 2.54
        himp.set_text("cm   (%d′ %d″)" % (int(inches // 12),
                                          round(inches % 12)))
    hspin.connect("value-changed", update_imperial)
    update_imperial()
    hrow.append(hlabel)
    hrow.append(hspin)
    hrow.append(himp)
    p_height.append(hrow)

    # ---- page 4: clear your play area
    p_prep = page_box(P_PREPARE)
    art_prep = Gtk.DrawingArea(halign=Gtk.Align.CENTER)
    art_prep.set_content_width(560)
    art_prep.set_content_height(230)
    art_prep.set_draw_func(lambda _a, cr, w, h: draw_prepare(cr, w, h))
    p_prep.append(art_prep)

    # ---- page 5: place your sensors
    p_place = page_box(P_PLACE)
    art_place = Gtk.DrawingArea(halign=Gtk.Align.CENTER)
    art_place.set_content_width(560)
    art_place.set_content_height(230)
    art_place.set_draw_func(lambda _a, cr, w, h: draw_placement(cr, w, h))
    p_place.append(art_place)

    # ---- page 6: set up sensor tracking
    p_track = page_box(P_TRACK)
    art_track = Gtk.DrawingArea(halign=Gtk.Align.CENTER)
    art_track.set_content_width(360)
    art_track.set_content_height(300)
    art_track.set_draw_func(lambda _a, cr, w, h: draw_tracking(
        cr, w, h, st["trk"]["frac"], st["trk"]["state"],
        st["trk"]["live"], max(st["trk"]["n_usb"], 1)))
    p_track.append(art_track)
    trk_status = Gtk.Label(label="")
    trk_status.add_css_class("oc-status")
    p_track.append(trk_status)

    def update_tracking(snap, alive, elapsed, n_usb):
        if st["closed"] or st["page"] != P_TRACK:
            return False
        t = st["trk"]
        t["n_usb"] = n_usb
        t["live"] = {s for (d, s) in snap["rates"] if d == 0}
        total = sum(v for (d, _s), v in snap["counts"].items() if d == 0)
        value = snap["values"][0] if n_usb >= 2 else total
        t["frac"] = value / TRACK_TARGET
        if t["state"] not in ("done", "error"):
            if not alive and total == 0 and elapsed > 4:
                t["state"] = "error"
                trk_status.set_text("The tracking session ended "
                                    "unexpectedly. Select Try Again.")
                cont.set_label("Try Again")
                cont.set_sensitive(True)
            elif value >= TRACK_TARGET:
                t["state"] = "done"
                stop_phase()
                trk_status.set_text("Great! Your sensors are now tracking "
                                    "correctly.")
                cont.set_sensitive(True)
                GLib.timeout_add(900, advance_from_tracking)
            elif total == 0:
                t["state"] = "searching"
                trk_status.set_text(
                    "Checking sensors…" if elapsed < 8 else
                    "Sensors can't track headset — make sure the front of "
                    "your headset faces the sensors.")
            elif n_usb >= 2 and len(t["live"]) < 2:
                t["state"] = "tracking"
                seen = next(iter(t["live"])) + 1 if t["live"] else None
                trk_status.set_text(
                    f"Only sensor {seen} can see your headset — move it "
                    "where both sensors can see its front."
                    if seen else "Move the headset from side to side, "
                                 "facing the sensors.")
            else:
                t["state"] = "tracking"
                trk_status.set_text("Move the headset from side to side, "
                                    "down towards the floor, then in front "
                                    "of your head.")
        art_track.queue_draw()
        return False

    def advance_from_tracking():
        if not st["closed"] and st["page"] == P_TRACK \
                and st["trk"]["state"] == "done":
            show(P_CONFIRM)
        return False

    def tracking_loop():
        ph = st["phase"]
        t0 = time.time()
        while not st["stop_phase"].is_set() and ph is st["phase"]:
            snap = ph.poll()
            n_usb = len(hw.tracking_sensors())
            GLib.idle_add(update_tracking, snap, ph.alive(),
                          time.time() - t0, n_usb)
            st["stop_phase"].wait(0.3)

    def start_tracking():
        st["trk"].update(frac=0.0, state="searching", live=set())
        trk_status.set_text("Checking sensors…")
        cont.set_label("Continue")
        cont.set_sensitive(False)
        st["stop_phase"] = threading.Event()
        st["phase"] = PhaseCapture(config.SETUP_HOLD_CAPTURE)
        try:
            st["phase"].start()
        except OSError as e:
            st["trk"]["state"] = "error"
            trk_status.set_text(f"Could not start tracking: {e}")
            cont.set_label("Try Again")
            cont.set_sensitive(True)
            return
        threading.Thread(target=tracking_loop, daemon=True).start()

    # ---- page 7: confirm sensor tracking (standing capture + solve)
    p_conf = page_box(P_CONFIRM)
    art_conf = Gtk.DrawingArea(halign=Gtk.Align.CENTER)
    art_conf.set_content_width(560)
    art_conf.set_content_height(210)
    art_conf.set_draw_func(lambda _a, cr, w, h: draw_confirm(
        cr, w, h, st["stand"]["frac"], st["stand"]["state"]))
    p_conf.append(art_conf)
    conf_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8,
                       halign=Gtk.Align.CENTER)
    conf_spinner = Gtk.Spinner()
    conf_status = Gtk.Label(label="")
    conf_status.add_css_class("oc-status")
    conf_row.append(conf_spinner)
    conf_row.append(conf_status)
    p_conf.append(conf_row)
    log_exp = Gtk.Expander(label="Details")
    log_buf = Gtk.TextBuffer()
    log_tv = Gtk.TextView(buffer=log_buf, editable=False, monospace=True)
    log_tv.add_css_class("oc-log")
    for m in (log_tv.set_top_margin, log_tv.set_bottom_margin,
              log_tv.set_left_margin, log_tv.set_right_margin):
        m(6)
    log_sw = Gtk.ScrolledWindow(min_content_height=110)
    log_sw.set_size_request(560, -1)
    log_sw.set_child(log_tv)
    log_exp.set_child(log_sw)
    log_exp.set_halign(Gtk.Align.CENTER)
    p_conf.append(log_exp)

    def log(text):
        def _a():
            end = log_buf.get_end_iter()
            log_buf.insert(end, text if text.endswith("\n") else text + "\n")
            log_tv.scroll_to_iter(log_buf.get_end_iter(), 0.0, False,
                                  0.0, 1.0)
            return False
        GLib.idle_add(_a)

    def check_distances(dists):
        """The Oculus distance validation, from the sensors' own PnP
        ranges. Returns an error message or None if the position is OK."""
        med = {s: sorted(v)[len(v) // 2] for s, v in dists.items() if v}
        if not med:
            return None
        if min(med.values()) < DIST_TOO_CLOSE:
            return "You are too close to your sensors."
        if max(med.values()) > DIST_TOO_FAR:
            return "You are too far away from your sensors."
        if len(med) >= 2 and max(med.values()) / min(med.values()) \
                > DIST_RATIO_MAX:
            return ("Move closer to the center, so that each sensor is "
                    "about equally far away from you, and try again.")
        return None

    def update_confirm(snap, alive, elapsed):
        if st["closed"] or st["page"] != P_CONFIRM \
                or st["stand"]["state"] != "capturing":
            return False
        total = sum(v for (d, _s), v in snap["counts"].items() if d == 0)
        st["stand"]["frac"] = total / STAND_TARGET
        for s, dist in snap.get("dist", {}).items():
            st["stand"]["dists"].setdefault(s, []).append(dist)
        if not alive and total == 0 and elapsed > 4:
            confirm_fail("The tracking session ended unexpectedly.")
        elif total == 0 and elapsed > 10:
            conf_status.set_text("Sensors can't track headset — face the "
                                 "sensors and hold still.")
        elif total >= STAND_TARGET:
            bad = check_distances(st["stand"]["dists"])
            if bad:
                stop_phase()
                confirm_fail(bad)
            else:
                st["stand"]["state"] = "solving"
                stop_phase()
                conf_status.set_text("Checking sensors…")
                conf_spinner.start()
                threading.Thread(target=solve_worker, daemon=True).start()
        else:
            live = check_distances(st["stand"]["dists"])
            if live and elapsed > 3:
                conf_status.set_text(live)
            else:
                conf_status.set_text("Hold still…")
        art_conf.queue_draw()
        return False

    def confirm_loop():
        ph = st["phase"]
        t0 = time.time()
        while not st["stop_phase"].is_set() and ph is st["phase"]:
            snap = ph.poll()
            GLib.idle_add(update_confirm, snap, ph.alive(),
                          time.time() - t0)
            st["stop_phase"].wait(0.3)

    def confirm_fail(msg):
        def _f():
            st["stand"].update(state="idle", frac=0.0, dists={})
            conf_spinner.stop()
            conf_status.set_text(msg + "  Then select Try Again.")
            cont.set_label("Try Again")
            cont.set_sensitive(True)
            art_conf.queue_draw()
            return False
        GLib.idle_add(_f)

    def solve_worker():
        try:
            venv_py = _ensure_solver_deps(log)
            if not venv_py:
                confirm_fail("Could not set up the solver environment.")
                GLib.idle_add(log_exp.set_expanded, True)
                return
            height = "%.3f" % (hspin.get_value() / 100.0)
            cmd = [venv_py, config.CAL_TOOL, "setup",
                   config.SETUP_HOLD_CAPTURE, config.SETUP_FLOOR_CAPTURE,
                   "--height", height]
            log("$ " + " ".join(cmd))
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True)
            last = ""
            st["warns"] = []
            for line in proc.stdout:
                line = line.rstrip()
                if line:
                    last = line
                if line.startswith("placement: "):
                    st["warns"].append(line[len("placement: "):])
                log(line)
            proc.wait()
            if proc.returncode == 0:
                GLib.idle_add(solve_done)
            else:
                confirm_fail(last or "Calibration failed.")
                GLib.idle_add(log_exp.set_expanded, True)
        except Exception as e:
            confirm_fail(f"Error: {e}")

    def solve_done():
        if st["closed"]:
            return False
        st["stand"]["state"] = "solved"
        conf_spinner.stop()
        conf_status.set_text("Sensor check complete.")
        st["layout"] = _sensor_layout({})
        art_conf.queue_draw()

        def advance():
            if not st["closed"] and st["page"] == P_CONFIRM:
                show(P_DONE)
            return False
        GLib.timeout_add(1100, advance)
        app.refresh_once()
        return False

    def start_confirm():
        st["stand"].update(frac=0.0, state="capturing", dists={})
        conf_status.set_text("Hold still…")
        cont.set_label("Continue")
        cont.set_sensitive(False)
        st["stop_phase"] = threading.Event()
        st["phase"] = PhaseCapture(config.SETUP_FLOOR_CAPTURE)
        try:
            st["phase"].start()
        except OSError as e:
            confirm_fail(f"Could not start tracking: {e}")
            return
        threading.Thread(target=confirm_loop, daemon=True).start()

    # ---- page 8: sensor tracking confirmed
    p_done = page_box(P_DONE)
    art_done = Gtk.DrawingArea(halign=Gtk.Align.CENTER)
    art_done.set_content_width(520)
    art_done.set_content_height(300)
    art_done.set_draw_func(lambda _a, cr, w, h:
                           draw_complete(cr, w, h, st["layout"]))
    p_done.append(art_done)
    done_warn = Gtk.Label(label="", wrap=True,
                          justify=Gtk.Justification.CENTER)
    done_warn.add_css_class("oc-warn")
    done_warn.set_max_width_chars(62)
    p_done.append(done_warn)

    # ---- navigation
    def stop_phase():
        st["stop_phase"].set()
        ph, st["phase"] = st["phase"], None
        if ph:
            threading.Thread(target=ph.stop, daemon=True).start()

    def show(i):
        stop_phase()
        st["page"] = i
        stack.set_visible_child_name(f"p{i}")
        back.set_visible(P_CHECK < i < P_DONE)
        cont.set_label("Finish" if i == P_DONE else "Continue")
        cont.set_sensitive(i in (P_INTRO, P_HEIGHT, P_PREPARE, P_PLACE,
                                 P_DONE))
        line_l.queue_draw()
        line_r.queue_draw()
        if i == P_CHECK:
            poll_check()
            if st["timer"] is None:
                st["timer"] = GLib.timeout_add_seconds(1, poll_check)
        if i == P_TRACK:
            start_tracking()
        if i == P_CONFIRM:
            st["stand"].update(frac=0.0, state="idle", dists={})
            conf_status.set_text("")
            cont.set_sensitive(True)
            art_conf.queue_draw()
        if i == P_DONE:
            st["layout"] = _sensor_layout({})
            if st["warns"]:
                done_warn.set_text(
                    "It looks like your sensors aren't set up in the best "
                    "possible configuration. This may affect your "
                    "tracking:\n• " + "\n• ".join(st["warns"]))
            else:
                done_warn.set_text("")
            art_done.queue_draw()

    def on_continue(_b):
        i = st["page"]
        if i == P_CHECK:
            if st["check_ok"]:
                show(P_INTRO)
        elif i in (P_INTRO, P_HEIGHT, P_PREPARE, P_PLACE):
            show(i + 1)
        elif i == P_TRACK:
            if st["trk"]["state"] == "error":
                start_tracking()
            elif st["trk"]["state"] == "done":
                show(P_CONFIRM)
        elif i == P_CONFIRM:
            if st["stand"]["state"] == "idle":
                start_confirm()
        elif i == P_DONE:
            win.close()
    cont.connect("clicked", on_continue)
    back.connect("clicked", lambda _b: show(max(0, st["page"] - 1)))

    def on_close(_w):
        st["closed"] = True
        stop_phase()
        if st["timer"]:
            GLib.source_remove(st["timer"])
            st["timer"] = None
        app.sensorsetup_win = None
        app.refresh_once()
        return False
    win.connect("close-request", on_close)

    show(P_CHECK)
    win.present()
