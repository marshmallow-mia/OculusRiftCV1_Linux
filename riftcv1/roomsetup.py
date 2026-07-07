#!/usr/bin/env python3
"""Standing-only room calibration via the Chaperone API — replaces the
official Room Setup app, which segfaults on many Linux systems.

Reads the headset's current raw pose and commits a standing universe:
origin on the floor under the headset, +Y up, facing where the headset
points. Run with the pose-test venv python (needs `openvr`) while
SteamVR is running and the headset rests at the desired centre spot.

Usage: roomsetup.py <headset_height_above_floor_m> [--dry-run]
Exit codes: 2 = no bindings, 3 = no VR session, 4 = headset pose invalid.
"""
import math
import sys

try:
    import openvr
except ImportError:
    sys.exit(2)

BOUNDS_SIZE = 1.0       # 1 x 1 m standing spot
BOUNDS_HEIGHT = 2.43    # what Room Setup uses


def main():
    args = [a for a in sys.argv[1:] if a != "--dry-run"]
    dry = "--dry-run" in sys.argv
    height = float(args[0]) if args else 0.0

    try:
        vr = openvr.init(openvr.VRApplication_Background)
    except Exception:
        sys.exit(3)
    try:
        poses = vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseRawAndUncalibrated, 0.0,
            openvr.k_unMaxTrackedDeviceCount)
        hmd = poses[openvr.k_unTrackedDeviceIndex_Hmd]
        if not hmd.bPoseIsValid:
            print("headset pose is not valid — wake the headset (briefly "
                  "cover its\nproximity sensor) and keep it visible to the "
                  "sensor, then retry")
            sys.exit(4)
        m = hmd.mDeviceToAbsoluteTracking
        hx, hy, hz = m[0][3], m[1][3], m[2][3]
        # headset forward is -Z in device space; project onto the floor
        fx, fz = -m[0][2], -m[2][2]
        norm = math.hypot(fx, fz)
        if norm < 1e-3:     # headset pointing straight up/down
            fx, fz = 0.0, -1.0
        else:
            fx, fz = fx / norm, fz / norm

        # standing origin in raw coords: on the floor under the headset,
        # -Z (forward) towards where the headset faces. Columns:
        # Z = -(fx, 0, fz), Y = up, X = Y cross Z
        mat = openvr.HmdMatrix34_t()
        cols = ((-fz, 0.0, fx), (0.0, 1.0, 0.0), (-fx, 0.0, -fz))
        trans = (hx, hy - height, hz)
        for row in range(3):
            for col in range(3):
                mat[row][col] = cols[col][row]
            mat[row][3] = trans[row]

        chap = openvr.VRChaperoneSetup()
        chap.roomSetupStarting()
        chap.setWorkingStandingZeroPoseToRawTrackingPose(mat)
        chap.setWorkingSeatedZeroPoseToRawTrackingPose(mat)
        chap.setWorkingPlayAreaSize(BOUNDS_SIZE, BOUNDS_SIZE)

        c = BOUNDS_SIZE / 2
        corners = ((-c, -c), (c, -c), (c, c), (-c, c))
        quads = (openvr.HmdQuad_t * 4)()
        for i in range(4):
            (x0, z0), (x1, z1) = corners[i], corners[(i + 1) % 4]
            pts = ((x0, 0.0, z0), (x0, BOUNDS_HEIGHT, z0),
                   (x1, BOUNDS_HEIGHT, z1), (x1, 0.0, z1))
            for j, (px, py, pz) in enumerate(pts):
                quads[i].vCorners[j].v[0] = px
                quads[i].vCorners[j].v[1] = py
                quads[i].vCorners[j].v[2] = pz
        chap.setWorkingCollisionBoundsInfo(quads)

        if dry:
            chap.revertWorkingCopy()
            verb = "dry run OK — would commit"
        else:
            if not chap.commitWorkingCopy(openvr.EChaperoneConfigFile_Live):
                print("chaperone commit failed")
                sys.exit(5)
            verb = "committed"
        print("standing universe %s: centre under headset at "
              "(%.2f, %.2f), floor %.0f cm\nbelow the headset, facing "
              "yaw %.0f°" % (verb, hx, hz, height * 100,
                             math.degrees(math.atan2(fx, fz))))
    finally:
        openvr.shutdown()


if __name__ == "__main__":
    main()
