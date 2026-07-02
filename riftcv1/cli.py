"""Command-line interface — scriptable status and fixes, no GUI needed.

Examples:
  rift_cv1_center.py status          # exit code 1 if something is wrong
  rift_cv1_center.py fix-usb
  rift_cv1_center.py test
  rift_cv1_center.py switch-runtime steamvr
"""
import argparse
import time

from . import config, diagnostics, hw, runtime


def _row(label, value):
    print(f"  {label:<18} {value}")


def cmd_status(_args):
    problems = []
    usb = hw.usb_sysfs_device(hw.OCULUS_VID, hw.HMD_PID) is not None
    _row("headset USB", "connected" if usb else "not found")
    if not usb:
        problems.append("headset not on USB")

    hid = hw.hmd_hid_bound()
    _row("headset HID", "bound" if hid else
         ("NOT BOUND — run: fix-usb" if usb else "—"))
    if usb and not hid:
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

    conn, ovr = hw.rift_edid_connector()
    _row("display link", f"Rift active on {conn}" if ovr
         else "asleep / unknown (run: test)")

    mt = runtime.room_config_mtime()
    _row("room calibration",
         time.strftime("calibrated %d %b %H:%M", time.localtime(mt))
         if mt else "will auto-calibrate next session")

    drv = runtime.driver_registered()
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
            ("info", cmd_info, "headset serial / firmware / sensor info")]:
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
