"""Low-level hardware access: USB sysfs, hidraw feature reports, EDID,
USB resets and process checks. No GTK dependencies."""
import array
import fcntl
import glob
import os
import struct
import time

OCULUS_VID, HMD_PID, CAMERA_PID = "2833", "0031", "0211"
USBDEVFS_RESET = 21780
USBDEVFS_IOCTL = (3 << 30) | (16 << 16) | (0x55 << 8) | 18
USBDEVFS_CONNECT = (0x55 << 8) | 23
BOOT_MODES = {0: "normal", 1: "bootloader", 2: "radio pairing"}


def _read_attr(syspath, name):
    try:
        with open(os.path.join(syspath, name)) as f:
            return f.read().strip()
    except OSError:
        return None


def usb_sysfs_devices(vid, pid):
    """All (sysfs_path, /dev/bus/usb node) pairs for a USB vid:pid."""
    found = []
    for p in sorted(glob.glob("/sys/bus/usb/devices/*")):
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
            found.append((p, "/dev/bus/usb/%03d/%03d" % (bus, dev)))
        except (OSError, ValueError):
            continue
    return found


def usb_sysfs_device(vid, pid):
    """First (sysfs_path, /dev/bus/usb node) for a USB device, or None."""
    devs = usb_sysfs_devices(vid, pid)
    return devs[0] if devs else None


def usb_speed(syspath):
    """Negotiated link speed in Mbps, or None if unreadable."""
    v = _read_attr(syspath, "speed")
    try:
        return int(float(v))
    except (TypeError, ValueError):
        return None


def tracking_sensors():
    """All connected CV1 tracking cameras, with link speed and serial.

    "power" is the sysfs autosuspend policy — anything but "on" means the
    kernel may suspend the camera, which drops it off the bus mid-session.
    """
    return [{"path": p, "node": n, "speed": usb_speed(p),
             "serial": _read_attr(p, "serial"),
             "power": _read_attr(p, "power/control")}
            for p, n in usb_sysfs_devices(OCULUS_VID, CAMERA_PID)]


def hmd_usb_info():
    """USB descriptor info for the headset (serial, firmware rev, link)."""
    found = usb_sysfs_device(OCULUS_VID, HMD_PID)
    if not found:
        return None
    path, node = found
    info = {"node": node}
    for attr in ("manufacturer", "product", "serial", "bcdDevice",
                 "version", "speed"):
        info[attr] = _read_attr(path, attr)
    return info


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


def hmd_hid_state():
    """How the headset's HID interfaces are driven right now:

    'bound'    kernel HID driver attached — idle and ready
    'in-use'   claimed by a VR driver through libusb (normal during a
               session; the kernel driver is detached so hidraw is gone)
    'unbound'  no driver at all — the post-self-reboot wedge (fix_usb)
    None       headset not on USB
    """
    found = usb_sysfs_device(OCULUS_VID, HMD_PID)
    if not found:
        return None
    drivers = set()
    for ifpath in glob.glob(found[0] + ":*"):
        try:
            drivers.add(os.path.basename(
                os.readlink(os.path.join(ifpath, "driver"))))
        except OSError:
            pass
    if "usbfs" in drivers:
        return "in-use"
    return "bound" if "usbhid" in drivers else "unbound"


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


def hid_get_feature(path, report_id, length):
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


def proc_pid(name):
    """PID of the first process with this comm name, or None."""
    short = name[:15]      # /proc/*/comm is truncated to 15 chars
    for p in glob.glob("/proc/[0-9]*/comm"):
        try:
            with open(p) as f:
                if f.read().strip() == short:
                    return int(p.split("/")[2])
        except (OSError, ValueError):
            pass
    return None


def proc_running(name):
    """True if a process with this comm name runs (pgrep -x without a fork)."""
    return proc_pid(name) is not None


def proc_env(pid, key):
    """One variable from a process's environment (same-user only)."""
    try:
        with open(f"/proc/{pid}/environ", "rb") as f:
            data = f.read()
    except OSError:
        return None
    prefix = key.encode() + b"="
    for item in data.split(b"\0"):
        if item.startswith(prefix):
            return item[len(prefix):].decode(errors="replace")
    return None


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


def fix_usb(log):
    """Reattach drivers, then USB-reset if still unbound. Returns bound."""
    if hmd_hid_state() == "in-use":
        log("A VR driver holds the headset (HID detached on purpose) — "
            "nothing to fix. Close the VR session first if something is "
            "actually wrong.")
        return False
    _, msg = usb_reattach_hid()
    log(msg)
    time.sleep(2)
    if not hmd_hid_bound():
        _, msg = usb_reset_hmd()
        log(msg)
        time.sleep(3)
    bound = hmd_hid_bound()
    log("HID bound: %s" % bound)
    return bound


def ensure_hid_bound(log, tries=3):
    """Wait for headset HID; reattach drivers / reset if not bound."""
    for i in range(tries):
        for _ in range(6):
            if hmd_hid_bound():
                return True
            time.sleep(1)
        _, msg = usb_reattach_hid()
        log(f"HID not bound, reattaching drivers (attempt {i + 1}): {msg}")
        time.sleep(2)
        if hmd_hid_bound():
            return True
        _, msg = usb_reset_hmd()
        log(f"still not bound, resetting USB: {msg}")
        time.sleep(3)
    return hmd_hid_bound()
