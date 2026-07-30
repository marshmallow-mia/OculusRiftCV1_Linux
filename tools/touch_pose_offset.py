#!/usr/bin/env python3
"""Re-derive the Touch driverFromHead transform used in driver_openhmd.cpp.

SteamVR draws oculus_cv1_controller_left/right - and derives the OpenXR grip
and aim poses - in Valve's render-model frame, but OpenHMD reports the pose of
the LED-model frame (rift.c registers the controller with an identity
model_pose). driverFromHead is the transform between them.

Both frames are pinned by data we already have, so the constant is measured
rather than tuned:

  * Oculus's own 24-LED model, read out of the controller's flash and printed
    by rift.c's "LED model: Controller N" dump, sits physically on the ring.
  * Valve's mesh models that same ring.

So registering the LED cloud onto the mesh recovers the transform. Both
controllers are solved independently and must come out mirror-image; the
residual must land near the depth the LEDs sit below the shell (~2 mm).

Usage:
    tools/touch_pose_offset.py captures/lin/<date>/<tag>.log

The log needs the one-time "LED model: Controller N" dump, which rift.c emits
at controller calibration read.
"""
import sys, os, re, json
import numpy as np

RM = os.path.expanduser(
    "~/.local/share/Steam/steamapps/common/SteamVR/resources/rendermodels")


def leds_from_log(path):
    """{'Controller 2': Nx3 array} - LED positions in the device frame."""
    lines = open(path, errors="replace").read().splitlines()
    out, cur = {}, None
    for l in lines:
        if l.startswith("LED model:"):
            cur, out[cur] = l.split(":", 1)[1].strip(), []
            continue
        m = re.search(r"\.pos = \{([-\d.]+),([-\d.]+),([-\d.]+)\}", l)
        if m and cur:
            out[cur].append([float(x) for x in m.groups()])
        elif cur and out[cur]:
            cur = None
    return {k: np.array(v) for k, v in out.items() if len(v)}


def obj_verts(path):
    return np.array([l.split()[1:4] for l in open(path)
                     if l.startswith("v ")], dtype=float)


def kabsch(P, Q):
    cp, cq = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - cp).T @ (Q - cq))
    R = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
    return R, cq - R @ cp


def nearest(P, M):
    return M[np.argmin(((P[:, None, :] - M[None, :, :]) ** 2).sum(-1), axis=1)]


def register(L, M):
    """Rigid ICP of LED cloud L onto mesh M, best over several initial dz."""
    best = None
    for dz0 in np.arange(0.03, 0.09, 0.005):
        R, t = np.eye(3), np.array([0.0, 0.0, dz0])
        for _ in range(150):
            R, t = kabsch(L, nearest((R @ L.T).T + t, M))
        P = (R @ L.T).T + t
        res = np.sqrt(((P - nearest(P, M)) ** 2).sum(-1)).mean()
        if best is None or res < best[0]:
            best = (res, R, t)
    return best


def main(log):
    leds = leds_from_log(log)
    got = {}
    for ctrl, side in (("Controller 2", "left"), ("Controller 3", "right")):
        if ctrl not in leds:
            sys.exit("no '%s' LED dump in %s" % (ctrl, log))
        mesh = "%s/oculus_cv1_controller_%s/oculus_cv1_controller_%s.obj" % (
            RM, side, side)
        if not os.path.exists(mesh):
            sys.exit("missing SteamVR render model: %s" % mesh)
        res, R, t = register(leds[ctrl], obj_verts(mesh))
        # driverFromHead is device <- rendermodel, the inverse of the fit
        Rh, th = R.T, -R.T @ t
        ang = np.degrees(np.arctan2(Rh[2, 1], Rh[2, 2]))
        got[side] = (Rh, th, ang, res)
        print("%-5s  residual %.2f mm  |  t [%7.2f %7.2f %7.2f] mm  "
              "X-rot %+6.2f deg" % (side, res * 1000, *(th * 1000), ang))

    (_, tl, al, rl), (_, tr, ar, rr) = got["left"], got["right"]
    if max(rl, rr) > 0.004:
        print("\nWARNING: residual above 4 mm - the fit did not find the ring.")
    if abs(al - ar) > 4.0 or abs(abs(tl[0]) - abs(tr[0])) > 0.005:
        print("\nWARNING: left and right disagree beyond fit noise; the "
              "controllers are mirror-symmetric, so they should not.")

    ang = (al + ar) / 2
    tx = (abs(tl[0]) + abs(tr[0])) / 2
    ty, tz = (tl[1] + tr[1]) / 2, (tl[2] + tr[2]) / 2
    a = np.radians(ang)
    print("\n=== constants for driver_openhmd.cpp (symmetrised) ===")
    print("  qDriverFromHeadRotation      { %.6f, %.6f, 0, 0 }   // %+.2f deg about X"
          % (np.cos(a / 2), np.sin(a / 2), ang))
    print("  vecDriverFromHeadTranslation left  [ %+.5f, %+.5f, %+.5f ]"
          % (-tx, ty, tz))
    print("                               right [ %+.5f, %+.5f, %+.5f ]"
          % (tx, ty, tz))

    # Landmarks must land on the physically correct side of the ring.
    j = json.load(open("%s/oculus_cv1_controller_left/"
                       "oculus_cv1_controller_left.json" % RM))
    Rh, th = got["left"][0], got["left"][1]
    print("\n=== sanity: render-model landmarks in the device frame ===")
    for name in ("openxr_aim", "tip", "openxr_grip", "body", "base"):
        o = np.array(j["components"][name]["component_local"]["origin"])
        print("  %-12s -> [%6.1f %6.1f %6.1f] mm" % (name, *((Rh @ o + th) * 1000)))
    print("  aim/tip must sit at negative Z (the ring's outward side) and "
          "grip/base at positive Z (the hand side).")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
