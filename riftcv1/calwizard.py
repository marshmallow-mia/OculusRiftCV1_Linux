"""GTK wizard for the full tracking calibration (logic in calibrate.py)."""
import threading

import gi
gi.require_version("Gtk", "4.0")
gi.require_version("Adw", "1")
from gi.repository import Adw, GLib, Gtk

from . import calibrate, hw
from .widgets import StatusRow

STEPS = ("Checks", "Sensor placement", "Optical tracking check",
         "Standing centre & floor", "Verify")


def _text_view():
    buf = Gtk.TextBuffer()
    tv = Gtk.TextView(buffer=buf, editable=False, monospace=True,
                      wrap_mode=Gtk.WrapMode.WORD_CHAR)
    for m in (tv.set_top_margin, tv.set_bottom_margin,
              tv.set_left_margin, tv.set_right_margin):
        m(8)
    sw = Gtk.ScrolledWindow(vexpand=True)
    sw.set_child(tv)
    sw.add_css_class("card")
    return sw, buf, tv


def _label(text):
    lbl = Gtk.Label(label=text, xalign=0, wrap=True)
    lbl.add_css_class("dim-label")
    return lbl


def open_calibration_wizard(app):
    if app.busy:
        app.log("Busy with another operation, please wait…")
        return
    st = {"step": 0, "cancel": threading.Event(), "proc": None,
          "optical_ok": False, "timer": None}

    win = Gtk.Window(title="Calibrate Tracking", modal=True)
    win.set_transient_for(app.win)
    win.set_default_size(600, 640)
    box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
    for m in (box.set_margin_top, box.set_margin_bottom,
              box.set_margin_start, box.set_margin_end):
        m(14)
    win.set_child(box)

    title = Gtk.Label(xalign=0)
    title.add_css_class("title-2")
    box.append(title)
    stack = Gtk.Stack(vexpand=True)
    box.append(stack)

    nav = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
    back = Gtk.Button(label="Back")
    nav_status = Gtk.Label(hexpand=True, xalign=0)
    nav_status.add_css_class("dim-label")
    nxt = Gtk.Button(label="Next")
    nxt.add_css_class("suggested-action")
    nav.append(back)
    nav.append(nav_status)
    nav.append(nxt)
    box.append(nav)

    # ---- page 1: preflight checks
    p1 = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
    lb = Gtk.ListBox()
    lb.add_css_class("boxed-list")
    lb.set_selection_mode(Gtk.SelectionMode.NONE)
    p1.append(lb)
    recheck = Gtk.Button(label="Re-check")
    recheck.set_halign(Gtk.Align.START)
    p1.append(recheck)
    p1.append(_label("Fix anything red before continuing. Yellow rows "
                     "are worth fixing but don't block calibration."))
    def run_preflight(*_a):
        rows = calibrate.preflight()
        # append() wraps children in ListBoxRows — remove those, not the
        # StatusRow widgets, or the old rows silently stay
        while (row := lb.get_first_child()) is not None:
            lb.remove(row)
        for ok, label, detail in rows:
            r = StatusRow(label)
            r.set(ok, detail)
            lb.append(r)
        good = calibrate.preflight_ok(rows)
        if st["step"] == 0:
            nxt.set_sensitive(good)
            nav_status.set_text("" if good else
                                "resolve the red rows, then Re-check")
    recheck.connect("clicked", run_preflight)

    # ---- page 2: placement
    p2 = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
    p2.append(_label(calibrate.PLACEMENT))
    cam_btn = Gtk.Button(label="Open camera view (aim the sensor)")
    cam_btn.set_halign(Gtk.Align.START)
    cam_btn.connect("clicked", lambda _b: app.on_camera(None))
    p2.append(cam_btn)

    # ---- page 3: optical tracking check
    p3 = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
    p3.append(_label(calibrate.SESSION_NOTE))
    p3.append(_label(calibrate.STILL_HINT))
    start = Gtk.Button()
    start.set_child(Adw.ButtonContent(label="Start check",
                                      icon_name="media-record-symbolic",
                                      halign=Gtk.Align.CENTER))
    start.add_css_class("suggested-action")
    start.set_halign(Gtk.Align.START)
    p3.append(start)
    o_sw, o_buf, o_tv = _text_view()
    p3.append(o_sw)

    def o_set(txt):
        GLib.idle_add(o_buf.set_text, txt)

    def o_append(txt):
        def _a():
            end = o_buf.get_end_iter()
            o_buf.insert(end, txt if txt.endswith("\n") else txt + "\n")
            o_tv.scroll_to_iter(o_buf.get_end_iter(), 0.0, False, 0.0, 1.0)
            return False
        GLib.idle_add(_a)

    def optical_done(ok, report):
        o_append("\n" + report)
        st["optical_ok"] = st["optical_ok"] or ok
        start.set_sensitive(True)
        start.get_child().set_label("Run again" if ok else "Retry check")
        if st["step"] == 2:
            nxt.set_sensitive(st["optical_ok"])
            nav_status.set_text("converged ✓ — continue to SteamVR"
                                if ok else "check failed — see above")
        app.refresh_once()
        return False

    def start_optical(_b):
        if app.busy:
            o_append("Busy with another operation — wait for it first.")
            return
        app.busy = True
        st["cancel"].clear()
        start.set_sensitive(False)
        nav_status.set_text("capturing — keep the headset still…")

        def worker():
            try:
                ok, report = calibrate.optical_check(
                    o_set, o_append, st["cancel"],
                    lambda p: st.update(proc=p))
            except Exception as e:      # never leave the app wedged busy
                ok, report = False, f"Error: {e}"
            finally:
                app.busy = False
            GLib.idle_add(optical_done, ok, report)
        threading.Thread(target=worker, daemon=True).start()
    start.connect("clicked", start_optical)

    # ---- page 4: standing centre & floor (no Room Setup app)
    p4 = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
    svr_row = StatusRow("SteamVR")
    svr_lb = Gtk.ListBox()
    svr_lb.add_css_class("boxed-list")
    svr_lb.set_selection_mode(Gtk.SelectionMode.NONE)
    svr_lb.append(svr_row)
    p4.append(svr_lb)
    b_svr = Gtk.Button(label="Launch SteamVR")
    b_svr.set_halign(Gtk.Align.START)
    b_svr.connect("clicked", lambda _b: (calibrate.launch_steamvr(),
                                         app.log("Asked Steam to launch "
                                                 "SteamVR…")))
    p4.append(b_svr)
    p4.append(_label(calibrate.QUICK_ROOM_NOTE))
    hrow = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
    hrow.append(Gtk.Label(label="headset height above floor (cm):"))
    height_spin = Gtk.SpinButton.new_with_range(0, 250, 1)
    hrow.append(height_spin)
    b_set = Gtk.Button(label="Set centre & floor")
    b_set.add_css_class("suggested-action")
    hrow.append(b_set)
    p4.append(hrow)
    room_status = _label("")
    p4.append(room_status)
    fb_exp = Gtk.Expander(label="Official Room Setup app (fallback)")
    fb = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
    fb.set_margin_top(8)
    fb.append(_label(calibrate.ROOM_SETUP_STEPS))
    b_room = Gtk.Button(label="Launch Room Setup app")
    b_room.set_halign(Gtk.Align.START)
    fb.append(b_room)
    fb_exp.set_child(fb)
    p4.append(fb_exp)

    def on_quick(_b):
        st["setting"] = True
        b_set.set_sensitive(False)
        room_status.set_text("committing standing universe…")
        height_m = height_spin.get_value() / 100.0

        def worker():
            ok, msg = calibrate.quick_room_setup(
                height_m, lambda t: GLib.idle_add(room_status.set_text, t))

            def done():
                st["setting"] = False
                room_status.set_text(("✓ " if ok else "✗ ") + msg)
                b_set.set_sensitive(True)
                if ok and st["step"] == 3:
                    nav_status.set_text("standing centre & floor set ✓")
                return False
            GLib.idle_add(done)
        threading.Thread(target=worker, daemon=True).start()
    b_set.connect("clicked", on_quick)

    def open_room_setup(_b):
        if not hw.proc_running("vrserver"):
            app.toast("Launch SteamVR first")
            return
        if calibrate.launch_room_setup():
            app.log("Room Setup launched")
        else:
            app.toast("Room Setup tool not found — use the SteamVR menu "
                      "▸ Room Setup")
    b_room.connect("clicked", open_room_setup)

    def poll_svr():
        if st["step"] != 3:
            st["timer"] = None
            return False
        run = hw.proc_running("vrserver")
        svr_row.set(True if run else None,
                    "running" if run else "not running — launch it")
        b_set.set_sensitive(run and not st.get("setting"))
        b_room.set_sensitive(run)
        return True

    # ---- page 5: verify
    p5 = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
    v_sw, v_buf, _v_tv = _text_view()
    p5.append(v_sw)

    def run_verify():
        def worker():
            _ok, text = calibrate.verify()
            GLib.idle_add(v_buf.set_text, text)
        v_buf.set_text("checking…")
        threading.Thread(target=worker, daemon=True).start()

    # ---- navigation
    for i, page in enumerate((p1, p2, p3, p4, p5)):
        stack.add_named(page, str(i))

    def show(i):
        st["step"] = i
        stack.set_visible_child_name(str(i))
        title.set_text(f"Step {i + 1}/5 — {STEPS[i]}")
        back.set_sensitive(i > 0)
        nxt.set_label("Close" if i == 4 else "Next")
        nxt.set_sensitive(True)
        nav_status.set_text("")
        if i == 0:
            run_preflight()
        elif i == 2:
            nxt.set_sensitive(st["optical_ok"])
            nav_status.set_text("" if st["optical_ok"] else
                                "run the check to continue")
        elif i == 3 and st["timer"] is None:
            poll_svr()
            st["timer"] = GLib.timeout_add_seconds(2, poll_svr)
        elif i == 4:
            run_verify()

    def on_nav(delta):
        if st["step"] == 4 and delta > 0:
            win.close()
            return
        show(st["step"] + delta)
    back.connect("clicked", lambda _b: on_nav(-1))
    nxt.connect("clicked", lambda _b: on_nav(+1))

    def on_close(_w):
        st["cancel"].set()
        proc = st["proc"]
        if proc:
            try:
                proc.terminate()
            except OSError:
                pass
        if st["timer"]:
            GLib.source_remove(st["timer"])
            st["timer"] = None
        app.refresh_once()
        return False
    win.connect("close-request", on_close)

    show(0)
    win.present()
