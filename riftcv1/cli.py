"""Command-line interface — scriptable status and fixes, no GUI needed.

Examples:
  rift_cv1_center.py status          # exit code 1 if something is wrong
  rift_cv1_center.py fix-usb
  rift_cv1_center.py test
  rift_cv1_center.py switch-runtime steamvr
"""
import argparse

from . import calibrate, config, diagnostics, hw, runtime


def _row(label, value):
    print(f"  {label:<18} {value}")


def cmd_status(_args):
    problems = []
    usb = hw.usb_sysfs_device(hw.OCULUS_VID, hw.HMD_PID) is not None
    _row("headset USB", "connected" if usb else "not found")
    if not usb:
        problems.append("headset not on USB")

    hid = hw.hmd_hid_state()
    _row("headset HID",
         {"bound": "bound",
          "in-use": "in use by VR driver (normal during a session)",
          "unbound": "NOT BOUND — run: fix-usb"}.get(hid, "—"))
    if hid == "unbound":
        problems.append("HID not bound")

    sensors = hw.tracking_sensors()
    if not sensors:
        _row("tracking sensors", "none found (rotation-only tracking)")
        problems.append("no tracking sensor")
    else:
        for i, s in enumerate(sensors, 1):
            spd = s["speed"]
            link = ("?" if spd is None else
                    f"USB 3.x, {spd} Mbps" if spd >= 5000 else
                    f"USB 2, {spd} Mbps — use a USB 3 port!")
            _row(f"sensor {i}", f"serial {s['serial'] or '-'} ({link})")
            if s["power"] and s["power"] != "on":
                _row("", "autosuspend enabled — reinstall udev rules")
                problems.append(f"sensor {i} autosuspend enabled")

    conn, ovr = hw.rift_edid_connector()
    _row("display link", f"Rift active on {conn}" if ovr
         else "asleep / unknown (run: test)")

    _row("room calibration",
         "auto at session start — verify with: calibrate")

    drv = runtime.driver_registered()
    fresh = runtime.driver_deploy_state()
    if drv and fresh is False:
        _row("SteamVR driver", "registered, but deployed copy is STALE — "
             "run ./install_files_to_build.sh in SteamVR-OpenHMD")
        problems.append("deployed driver stale after rebuild")
    else:
        _row("SteamVR driver", "registered" if drv else "NOT REGISTERED")
    if not drv:
        problems.append("driver not registered")

    _row("OpenVR runtime", runtime.active_runtime())

    svr = hw.proc_running("vrserver")
    _row("SteamVR", "running" if svr else "not running")
    if svr:
        devs = runtime.vr_device_status()
        for d in devs or []:
            if d.get("class") != "Controller":
                continue
            desc = "connected" if d.get("connected") else "off"
            if d.get("battery") is not None:
                desc += ", battery %d%%%s" % (
                    d["battery"], " (charging)" if d.get("charging") else "")
            _row(f"controller {d.get('role') or '?'}", desc)

    if problems:
        print("\nproblems: " + "; ".join(problems))
        return 1
    return 0


def cmd_fix_usb(_args):
    return 0 if hw.fix_usb(print) else 1


def cmd_test(_args):
    return 0 if diagnostics.wake_test(print) else 1


def cmd_export(_args):
    path, _ = diagnostics.export_report()
    print(path)
    return 0


def cmd_switch_runtime(args):
    ok, msg = runtime.switch_runtime(args.target)
    print(msg)
    return 0 if ok else 1


def cmd_devices(_args):
    print(diagnostics.device_list())
    return 0


def cmd_info(_args):
    print(diagnostics.headset_info())
    return 0


def cmd_room(args):
    if args.reset:
        print(diagnostics.room_reset(print))
    else:
        print(diagnostics.room_calibration_info())
    return 0


def _pause(prompt):
    try:
        input(prompt)
        return True
    except (EOFError, KeyboardInterrupt):
        print("\naborted")
        return False


def cmd_calibrate(_args):
    marks = {True: " ok ", None: "WARN", False: "FAIL"}
    print("FULL TRACKING CALIBRATION — 5 steps\n\n[1/5] preflight checks")
    rows = calibrate.preflight()
    for ok, label, detail in rows:
        print(f"  [{marks[ok]}] {label:<18} {detail}")
    if not calibrate.preflight_ok(rows):
        print("\nfix the FAIL rows above, then rerun calibrate")
        return 1

    print("\n[2/5] sensor placement\n" + calibrate.PLACEMENT)
    if not _pause("\nsensor placed and plugged in? press Enter… "):
        return 1

    print("\n[3/5] optical tracking check\n" + calibrate.SESSION_NOTE
          + "\n\n" + calibrate.STILL_HINT)
    if not _pause("\nheadset in place? press Enter to start… "):
        return 1
    ok, report = calibrate.optical_check(
        print, print, None, lambda _p: None)
    print("\n" + report)
    if not ok:
        return 1

    print("\n[4/5] SteamVR standing centre & floor")
    if not hw.proc_running("vrserver"):
        print("launching SteamVR…")
        calibrate.launch_steamvr()
        if not calibrate.wait_for_steamvr(print):
            print("SteamVR did not come up — start it manually and rerun "
                  "calibrate")
            return 1
    print("\n" + calibrate.QUICK_ROOM_NOTE)
    try:
        raw = input("\nheadset height above the floor in cm [0]: ").strip()
        height_cm = float(raw or 0)
    except (EOFError, KeyboardInterrupt):
        print("\naborted")
        return 1
    except ValueError:
        print("not a number — aborted")
        return 1
    ok, msg = calibrate.quick_room_setup(height_cm / 100, print)
    print(msg)
    if not ok:
        print("\n" + calibrate.ROOM_SETUP_STEPS)
        return 1

    print("\n[5/5] verify")
    ok, text = calibrate.verify()
    print(text)
    return 0 if ok else 1


def main(argv):
    p = argparse.ArgumentParser(
        prog="rift_cv1_center.py",
        description="Rift CV1 control center — run without arguments "
                    "for the GUI.")
    p.add_argument("--version", action="version",
                   version="%(prog)s " + config.VERSION)
    sub = p.add_subparsers(dest="cmd", required=True, metavar="command")

    for name, fn, help_ in [
            ("status", cmd_status,
             "print hardware/driver status (exit 1 on problems)"),
            ("fix-usb", cmd_fix_usb,
             "reattach HID drivers / reset the headset USB device"),
            ("test", cmd_test,
             "wake the headset and verify the display link (15 s)"),
            ("export", cmd_export,
             "write a diagnostics report, print its path"),
            ("devices", cmd_devices, "list OpenHMD devices"),
            ("info", cmd_info, "headset serial / firmware / sensor info"),
            ("calibrate", cmd_calibrate,
             "guided full tracking calibration (optical + SteamVR "
             "Room Setup)")]:
        sp = sub.add_parser(name, help=help_)
        sp.set_defaults(fn=fn)

    sp = sub.add_parser("switch-runtime",
                        help="set the OpenVR runtime (toggles if omitted)")
    sp.add_argument("target", nargs="?", choices=("steamvr", "wivrn"))
    sp.set_defaults(fn=cmd_switch_runtime)

    sp = sub.add_parser("room", help="show (or --reset) room calibration")
    sp.add_argument("--reset", action="store_true")
    sp.set_defaults(fn=cmd_room)

    args = p.parse_args(argv)
    return args.fn(args)
