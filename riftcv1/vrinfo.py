#!/usr/bin/env python3
"""Print SteamVR tracked-device status as JSON on stdout.

Run with the venv python that has the `openvr` bindings (the same venv
pose_test.py uses). Used for the controller/battery status row and the
CLI `status` command. Exit codes: 2 = no bindings, 3 = no VR session.
"""
import json
import sys

try:
    import openvr
except ImportError:
    sys.exit(2)


def main():
    try:
        vr = openvr.init(openvr.VRApplication_Background)
    except Exception:
        sys.exit(3)
    names = {openvr.TrackedDeviceClass_HMD: "HMD",
             openvr.TrackedDeviceClass_Controller: "Controller",
             openvr.TrackedDeviceClass_TrackingReference: "Sensor"}
    out = []
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        cls = vr.getTrackedDeviceClass(i)
        if cls not in names:
            continue
        dev = {"class": names[cls], "role": "",
               "connected": bool(vr.isTrackedDeviceConnected(i))}
        if cls == openvr.TrackedDeviceClass_Controller:
            role = vr.getControllerRoleForTrackedDeviceIndex(i)
            dev["role"] = {1: "L", 2: "R"}.get(role, "")
        # not every driver reports battery — omit the keys rather than guess
        try:
            if vr.getBoolTrackedDeviceProperty(
                    i, openvr.Prop_DeviceProvidesBatteryStatus_Bool):
                dev["battery"] = round(100 * vr.getFloatTrackedDeviceProperty(
                    i, openvr.Prop_DeviceBatteryPercentage_Float))
                dev["charging"] = bool(vr.getBoolTrackedDeviceProperty(
                    i, openvr.Prop_DeviceIsCharging_Bool))
        except Exception:
            pass
        out.append(dev)
    print(json.dumps(out))
    openvr.shutdown()


if __name__ == "__main__":
    main()
