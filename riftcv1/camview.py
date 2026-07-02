"""Sensor camera view (PipeWire debug stream) + placement guide window."""
import json
import os
import subprocess
import threading
import time

import gi
gi.require_version("Gtk", "4.0")
from gi.repository import Gdk, GLib, Gtk

from . import config, hw

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


def open_camera_window(app):
    if getattr(app, "cam_win", None):
        app.cam_win.present()
        return
    try:
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
    except (ValueError, ImportError) as e:
        app.log(f"GStreamer not available: {e} — install gstreamer "
                "and gst-plugin-pipewire")
        return
    if not getattr(app, "_gst_ready", False):
        Gst.init(None)
        app._gst_ready = True
    app._Gst = Gst

    win = Gtk.Window(title="Camera & Placement — Rift CV1")
    win.set_transient_for(app.win)
    win.set_default_size(1040, 600)
    app.cam_win = win
    app.cam_pipe = None
    app.cam_proc = None
    app.cam_stop = threading.Event()

    outer = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=12)
    for m in (outer.set_margin_top, outer.set_margin_bottom,
              outer.set_margin_start, outer.set_margin_end):
        m(12)
    win.set_child(outer)

    left = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
    left.set_hexpand(True)
    outer.append(left)
    app.cam_picture = Gtk.Picture()
    app.cam_picture.set_vexpand(True)
    app.cam_picture.add_css_class("card")
    left.append(app.cam_picture)
    app.cam_status = Gtk.Label(label="Starting camera stream…", xalign=0)
    app.cam_status.add_css_class("dim-label")
    left.append(app.cam_status)

    right = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
    right.set_size_request(380, -1)
    outer.append(right)
    dd = Gtk.DropDown.new_from_strings([name for name, _ in GUIDES])
    right.append(dd)
    guide = {"sel": 0}
    area = Gtk.DrawingArea()
    area.set_content_height(280)
    area.add_css_class("card")
    area.set_draw_func(lambda _a, cr, w, h: _draw_guide(guide["sel"],
                                                        cr, w, h))
    right.append(area)
    guide_text = Gtk.Label(xalign=0, wrap=True)
    guide_text.set_text(GUIDES[0][1])
    right.append(guide_text)

    def on_mode(dd_, _p):
        guide["sel"] = dd_.get_selected()
        guide_text.set_text(GUIDES[guide["sel"]][1])
        area.queue_draw()
    dd.connect("notify::selected", on_mode)

    def on_cam_close(_w):
        app.cam_stop.set()
        if app.cam_pipe:
            app.cam_pipe.set_state(Gst.State.NULL)
            app.cam_pipe = None
        if app.cam_proc:
            app.cam_proc.terminate()
            try:
                app.cam_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                app.cam_proc.kill()
                app.cam_proc.wait()
            app.cam_proc = None
        app.cam_win = None
        return False
    win.connect("close-request", on_cam_close)
    win.present()
    _stream_start(app)


def _stream_start(app):
    Gst = app._Gst

    def set_status(txt):
        def _s():
            if getattr(app, "cam_win", None):
                app.cam_status.set_text(txt)
            return False
        GLib.idle_add(_s)

    def worker():
        # a tracking session must be running to publish the stream
        if not hw.proc_running("vrserver") and not app.cam_proc:
            if not os.path.exists(config.OPENHMD_EXAMPLE):
                set_status("openhmd_simple_example not built — run "
                           "Setup & install first.")
                return
            set_status("Starting a tracking session (headset wakes up)…")
            try:
                app.cam_proc = subprocess.Popen(
                    [config.OPENHMD_EXAMPLE, "0"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except OSError as e:
                set_status(f"failed to start tracking session: {e}")
                return
        node = None
        deadline = time.time() + 25
        while time.time() < deadline and not app.cam_stop.is_set():
            try:
                out = subprocess.run(["pw-dump"], capture_output=True,
                                     text=True, timeout=5).stdout
                objs = json.loads(out)
            except (OSError, ValueError, subprocess.TimeoutExpired):
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
        if app.cam_stop.is_set():
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
                if getattr(app, "cam_win", None):
                    tex = Gdk.MemoryTexture.new(
                        w, h, Gdk.MemoryFormat.R8G8B8,
                        GLib.Bytes.new(data), w * 3)
                    app.cam_picture.set_paintable(tex)
                return False
            GLib.idle_add(upd)
            return Gst.FlowReturn.OK

        sink.connect("new-sample", on_sample)
        pipe.set_state(Gst.State.PLAYING)
        app.cam_pipe = pipe
    threading.Thread(target=worker, daemon=True).start()


def _draw_guide(mode, cr, w, h):
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
