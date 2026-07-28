#!/usr/bin/env python
"""Offline replay of constellation pose reconstruction from a capture.

Reads an `OHMD_RIFT_CAL_CAPTURE` JSONL capture and replays, per exposure, what
the driver does today: each sensor solves the device pose from its own blobs
(the `cam` field is that solve, as recorded), the poses are converted to world
with that sensor's extrinsics, and the tracker merges them with weights
1/obs_scale^2 (rift-tracker.c `rift_tracked_device_model_pose_update`).

This is the baseline harness for the Windows-parity program: every change to the
reconstruction is scored against these numbers, with no hardware involved. See
windows-vs-linux-tracking.md §2 for why the per-camera solve is the thing under
suspicion — the two cameras disagree by a constant offset, which pose-averaging
cannot remove.

  baseline   replay today's per-camera solve + weighted merge, and report
             cross-camera disagreement, per-camera reprojection error, and the
             frame-to-frame step of the merged pose.

Usage:
  replay_recon.py baseline CAPTURE.jsonl [--device N] [--config PATH]
                  [--min-blobs N] [--limit N] [--json OUT.json]

By default the camera extrinsics come from the capture's own `campose` field
(what the driver was actually using). Captures recorded with no room config have
`campose` all-zero; pass --config to supply extrinsics for those.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from calibrate_room import (  # noqa: E402
    group_by_exposure,
    load_capture,
    load_config,
    pose_to_rt,
    project,
)

# rift-sensor-pose-helper.h
RIFT_POSE_MATCH_GOOD = 0x1
RIFT_POSE_MATCH_STRONG = 0x2
RIFT_POSE_MATCH_POSITION = 0x10
RIFT_POSE_MATCH_ORIENT = 0x20
RIFT_POSE_HAD_PRIOR = 0x100
RIFT_POSE_MATCH_LED_IDS = 0x200


def obs_scale(flags):
    """The confidence tier rift-tracker.c:1339-1351 would assign.

    Captures only contain LED-ID-verified observations, so in practice this is
    1.0 (also STRONG) or 1.5 (LED IDs only).
    """
    if flags & RIFT_POSE_MATCH_STRONG and flags & RIFT_POSE_MATCH_LED_IDS:
        return 1.0
    if flags & RIFT_POSE_MATCH_LED_IDS:
        return 1.5
    if flags & RIFT_POSE_MATCH_STRONG:
        return 2.0
    if (flags & RIFT_POSE_MATCH_POSITION) and (flags & RIFT_POSE_MATCH_ORIENT):
        return 4.0
    return 6.0


# ------------------------------------------------------------------ geometry

def world_pose(obs, cam_pose):
    """Object pose in world, from this sensor's obj->cam solve + extrinsics."""
    Rc, tc = cam_pose
    Ro, to = pose_to_rt(obs["cam"])
    return Rc @ Ro, Rc @ to + tc


def blob_arrays(obs, led_pos):
    """(led xyz in object frame, measured uv) for this observation's blobs."""
    b = np.asarray(obs["blobs"], dtype=float)
    ids = b[:, 0].astype(int)
    keep = (ids >= 0) & (ids < len(led_pos))
    return led_pos[ids[keep]], b[keep, 1:3]


def reproj_px(R_obj, t_obj, cam_pose, sensor, pts, uv):
    """Per-blob reprojection error (px) of an object WORLD pose in one camera."""
    Rc, tc = cam_pose
    Xw = pts @ R_obj.T + t_obj
    Xc = (Xw - tc) @ Rc
    return np.linalg.norm(project(Xc, sensor["K"], sensor["dist"]) - uv, axis=1)


def solve_joint(entries, cam_poses, sensors, R0, t0, huber_px=2.0):
    """One object world pose minimising reprojection across ALL cameras.

    The extrinsics are held fixed and the 6-DoF object pose is the only
    unknown — the shape of Oculus's reconstruction (fcn.18010cac0 called with
    camera index -1 = all cameras). Residuals are in pixels; their acceptance
    threshold is 2 px (their `2/715` normalised, fcn.18017f370).
    """
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation

    packed = []
    for e in entries:
        Rc, tc = cam_poses[e["sid"]]
        packed.append((Rc, tc, sensors[e["sid"]], e["pts"], e["uv"]))

    rv0 = Rotation.from_matrix(R0).as_rotvec()

    def residuals(x):
        R = Rotation.from_rotvec(x[0:3]).as_matrix()
        t = x[3:6]
        out = []
        for Rc, tc, sen, pts, uv in packed:
            Xw = pts @ R.T + t
            Xc = (Xw - tc) @ Rc
            out.append((project(Xc, sen["K"], sen["dist"]) - uv).ravel())
        return np.concatenate(out)

    res = least_squares(residuals, np.concatenate([rv0, t0]),
                        loss="huber", f_scale=huber_px, x_scale="jac",
                        max_nfev=200)
    R = Rotation.from_rotvec(res.x[0:3]).as_matrix()
    return R, res.x[3:6], res.success


def quat_angle_deg(Ra, Rb):
    cos = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


def weighted_merge(entries):
    """The tracker's same-exposure merge: 1/obs_scale^2 weighted mean position,
    incremental slerp for orientation (rift-tracker.c:1438-1487)."""
    from scipy.spatial.transform import Rotation

    w = np.array([1.0 / (e["scale"] ** 2) for e in entries])
    pos = np.array([e["t"] for e in entries])
    merged_t = (pos * w[:, None]).sum(axis=0) / w.sum()

    q = Rotation.from_matrix([e["R"] for e in entries]).as_quat()
    q = np.where((q @ q[0])[:, None] < 0, -q, q)
    qm = (q * w[:, None]).sum(axis=0)
    merged_R = Rotation.from_quat(qm / np.linalg.norm(qm)).as_matrix()
    return merged_R, merged_t, 1.0 / np.sqrt(w.sum())


# -------------------------------------------------------------------- report

def stats(v, scale=1.0):
    if len(v) == 0:
        return None
    a = np.asarray(v, dtype=float) * scale
    return {
        "n": int(a.size),
        "mean": float(a.mean()),
        "median": float(np.median(a)),
        "p95": float(np.percentile(a, 95)),
        "max": float(a.max()),
    }


def fmt(label, s, unit, width=34):
    if s is None:
        print(f"  {label:<{width}} (none)")
        return
    print(f"  {label:<{width}} mean {s['mean']:8.3f}  median {s['median']:8.3f}"
          f"  p95 {s['p95']:8.3f}  max {s['max']:9.3f}  {unit}  (n={s['n']})")


# ---------------------------------------------------------------- the replay

def cam_poses_for(capture_sensors, groups, config_path, use_config):
    """Extrinsics per sensor id: from --config, else the capture's campose."""
    if use_config:
        _, poses = load_config(config_path)
        out = {}
        for sid, s in capture_sensors.items():
            if s["serial"] not in poses:
                raise SystemExit(
                    f"sensor {sid} serial {s['serial']} not in {config_path}")
            pos, quat = poses[s["serial"]]
            out[sid] = pose_to_rt(np.concatenate([pos, quat]))
        return out, f"config {config_path}"

    out = {}
    for sid in capture_sensors:
        p7 = None
        for obs_by_sensor in groups.values():
            if sid in obs_by_sensor:
                p7 = obs_by_sensor[sid]["campose"]
                break
        if p7 is None:
            continue
        if not np.any(np.asarray(p7[:3])) and abs(p7[6]) < 1e-9:
            raise SystemExit(
                f"sensor {sid} has an all-zero campose in this capture "
                "(recorded with no room config) — pass --config")
        out[sid] = pose_to_rt(p7)
    return out, "capture campose"


def run_baseline(args):
    sensors, leds, observations = load_capture(
        args.capture, args.min_blobs, {args.device})
    if args.device not in leds:
        raise SystemExit(f"no LED model for device {args.device} in capture")
    led_pos = leds[args.device][0]

    groups = group_by_exposure(observations)
    cam_poses, cam_src = cam_poses_for(sensors, groups, args.config, args.config is not None)

    keys = sorted(groups)
    if args.limit:
        keys = keys[: args.limit]

    disagree_pos, disagree_ang = [], []
    own_reproj, cross_reproj, merged_reproj = [], [], []
    joint_reproj, joint_worst, joint_shift = [], [], []
    merged_track, joint_track = [], []
    n_solo = 0

    for key in keys:
        obs_by_sensor = groups[key]
        entries = []
        for sid, obs in sorted(obs_by_sensor.items()):
            if sid not in cam_poses:
                continue
            pts, uv = blob_arrays(obs, led_pos)
            if len(pts) < args.min_blobs:
                continue
            R, t = world_pose(obs, cam_poses[sid])
            entries.append({"sid": sid, "R": R, "t": t, "pts": pts, "uv": uv,
                            "scale": obs_scale(obs["flags"])})
        if not entries:
            continue
        if len(entries) < 2:
            n_solo += 1
            continue

        # each sensor's own solve, scored in its own camera and in the other's
        for e in entries:
            own_reproj.append(reproj_px(e["R"], e["t"], cam_poses[e["sid"]],
                                        sensors[e["sid"]], e["pts"], e["uv"]).mean())
        for e in entries:
            for o in entries:
                if o["sid"] == e["sid"]:
                    continue
                cross_reproj.append(
                    reproj_px(e["R"], e["t"], cam_poses[o["sid"]],
                              sensors[o["sid"]], o["pts"], o["uv"]).mean())

        a, b = entries[0], entries[1]
        disagree_pos.append(np.linalg.norm(a["t"] - b["t"]))
        disagree_ang.append(quat_angle_deg(a["R"], b["R"]))

        mR, mt, _ = weighted_merge(entries)
        for e in entries:
            merged_reproj.append(reproj_px(mR, mt, cam_poses[e["sid"]],
                                           sensors[e["sid"]], e["pts"], e["uv"]).mean())
        merged_track.append((key[0], mt))

        if args.joint:
            jR, jt, ok = solve_joint(entries, cam_poses, sensors, mR, mt)
            if ok:
                worst = 0.0
                for e in entries:
                    r = reproj_px(jR, jt, cam_poses[e["sid"]], sensors[e["sid"]],
                                  e["pts"], e["uv"]).mean()
                    joint_reproj.append(r)
                    worst = max(worst, r)
                joint_worst.append(worst)
                joint_shift.append(np.linalg.norm(jt - mt))
                joint_track.append((key[0], jt))

    def f2f(track):
        out = []
        track.sort(key=lambda kv: kv[0])
        for (t0, p0), (t1, p1) in zip(track, track[1:]):
            if 0 < (t1 - t0) < 60_000_000:   # consecutive exposures, < 60 ms
                out.append(np.linalg.norm(p1 - p0))
        return out

    steps = f2f(merged_track)
    joint_steps = f2f(joint_track)

    print(f"\ncapture      {args.capture}")
    print(f"device       {args.device}")
    print(f"extrinsics   {cam_src}")
    print(f"sensors      " + ", ".join(
        f"{sid}:{s['serial']}" for sid, s in sorted(sensors.items())))
    print(f"exposures    {len(keys)} total, {len(disagree_pos)} co-observed, "
          f"{n_solo} single-sensor")

    print("\nBASELINE — per-camera solve, then weighted merge (today's driver)")
    fmt("cross-camera disagreement", stats(disagree_pos, 1000.0), "mm")
    fmt("cross-camera disagreement", stats(disagree_ang), "deg")
    fmt("reproj: own solve in own camera", stats(own_reproj), "px")
    fmt("reproj: own solve in OTHER camera", stats(cross_reproj), "px")
    fmt("reproj: merged pose, both cameras", stats(merged_reproj), "px")
    fmt("merged pose frame-to-frame step", stats(steps, 1000.0), "mm")

    if args.joint:
        print("\nJOINT — one pose per exposure over all cameras' blobs (Oculus shape)")
        fmt("reproj: joint pose, each camera", stats(joint_reproj), "px")
        fmt("reproj: joint pose, WORST camera", stats(joint_worst), "px")
        fmt("joint vs merged position shift", stats(joint_shift, 1000.0), "mm")
        fmt("joint pose frame-to-frame step", stats(joint_steps, 1000.0), "mm")
        w = np.asarray(joint_worst)
        if w.size:
            print(f"\n  exposures whose worst-camera reprojection is <= 2 px "
                  f"(Oculus's acceptance): {100.0 * (w <= 2.0).mean():.1f}%")
            print(f"  ... <= 5 px: {100.0 * (w <= 5.0).mean():.1f}%")

    if args.json:
        out = {
            "capture": str(args.capture), "device": args.device,
            "extrinsics": cam_src,
            "exposures": len(keys), "co_observed": len(disagree_pos),
            "baseline": {
                "disagree_mm": stats(disagree_pos, 1000.0),
                "disagree_deg": stats(disagree_ang),
                "reproj_own_px": stats(own_reproj),
                "reproj_cross_px": stats(cross_reproj),
                "reproj_merged_px": stats(merged_reproj),
                "f2f_step_mm": stats(steps, 1000.0),
            },
        }
        Path(args.json).write_text(json.dumps(out, indent=2) + "\n")
        print(f"\nwrote {args.json}")


def undistort_fisheye(uv, K, D):
    """Pixels -> normalised camera rays, inverse of `project` (OpenCV fisheye)."""
    x = (uv[:, 0] - K[0, 2]) / K[0, 0]
    y = (uv[:, 1] - K[1, 2]) / K[1, 1]
    rd = np.hypot(x, y)
    th = rd.copy()                      # theta_d ~= theta for small angles
    for _ in range(12):                 # Newton on theta_d(theta) - rd
        th2 = th * th
        f = th * (1 + D[0] * th2 + D[1] * th2**2 + D[2] * th2**3 + D[3] * th2**4) - rd
        df = (1 + 3 * D[0] * th2 + 5 * D[1] * th2**2
              + 7 * D[2] * th2**3 + 9 * D[3] * th2**4)
        th = th - f / np.maximum(df, 1e-9)
    scale = np.where(rd > 1e-9, np.tan(th) / np.maximum(rd, 1e-9), 1.0)
    return np.stack([x * scale, y * scale], axis=1)


def run_export(args):
    """Dump real correspondences + the Python joint solution, so the C solver
    can be checked against this prototype on recorded data (no hardware)."""
    from scipy.spatial.transform import Rotation

    sensors, leds, observations = load_capture(
        args.capture, args.min_blobs, {args.device})
    led_pos = leds[args.device][0]
    groups = group_by_exposure(observations)
    cam_poses, _ = cam_poses_for(sensors, groups, args.config, args.config is not None)

    keys = sorted(groups)
    out = [f"LEDS {len(led_pos)}"]
    for p in led_pos:
        out.append(f"{p[0]:.9g} {p[1]:.9g} {p[2]:.9g}")

    n_written = 0
    for key in keys:
        if n_written >= args.limit:
            break
        entries = []
        for sid, obs in sorted(groups[key].items()):
            if sid not in cam_poses:
                continue
            pts, uv = blob_arrays(obs, led_pos)
            if len(pts) < args.min_blobs:
                continue
            b = np.asarray(obs["blobs"], dtype=float)
            ids = b[:, 0].astype(int)
            keep = (ids >= 0) & (ids < len(led_pos))
            R, t = world_pose(obs, cam_poses[sid])
            entries.append({"sid": sid, "R": R, "t": t, "pts": pts, "uv": uv,
                            "ids": ids[keep], "scale": obs_scale(obs["flags"])})
        if len(entries) < 2:
            continue

        mR, mt, _ = weighted_merge(entries)
        jR, jt, ok = solve_joint(entries, cam_poses, sensors, mR, mt)
        if not ok:
            continue

        out.append(f"EXPOSURE {len(entries)}")
        for e in entries:
            sen = sensors[e["sid"]]
            Rc, tc = cam_poses[e["sid"]]
            q = Rotation.from_matrix(Rc).as_quat()
            rays = undistort_fisheye(e["uv"], sen["K"], sen["dist"])
            out.append(f"VIEW {sen['K'][0, 0]:.9g} "
                       + " ".join(f"{v:.9g}" for v in list(tc) + list(q))
                       + f" {len(rays)}")
            for i, r in zip(e["ids"], rays):
                out.append(f"{i} {r[0]:.9g} {r[1]:.9g}")
        for name, (R, t) in (("INIT", (mR, mt)), ("PY", (jR, jt))):
            q = Rotation.from_matrix(R).as_quat()
            out.append(f"{name} " + " ".join(f"{v:.9g}" for v in list(t) + list(q)))
        n_written += 1

    Path(args.out).write_text("\n".join(out) + "\n")
    print(f"wrote {n_written} exposures to {args.out}")


def run_gravity(args):
    """Estimate world-up in each camera's frame, and hence each camera's tilt
    error, from the captures alone.

    This is the mechanism behind Oculus's gravity aligner
    (`EstimatedUpInCamera camera %d, object %d, count %d: tilt %.2f %.2f`).
    A tracked device's tilt in the world is known from its own accelerometer,
    independently of any camera extrinsics; the vision solve gives that same
    device's orientation relative to a camera. Composing the two says which way
    is up in the camera's frame:

        up_in_camera = R(device->camera) * R(device->world)^T * up_world

    Comparing that against what the room config claims (R(camera->world)^T *
    up_world) gives the camera's tilt error — the `tilt err %.2f` Oculus reports
    next to reprojection error in "Good/Bad calibration for camera %d".

    Caveat worth stating: `wp` is the fused pose, and since the vision-tilt
    correction was added the fused tilt is pulled partly toward the optical
    solution, which does depend on the extrinsics. The accelerometer still
    dominates, but this is not perfectly independent of the thing it measures.
    """
    sensors, leds, observations = load_capture(
        args.capture, args.min_blobs, {args.device})
    groups = group_by_exposure(observations)
    cam_poses, cam_src = cam_poses_for(sensors, groups, args.config,
                                       args.config is not None)

    up_world = np.array([0.0, 1.0, 0.0])
    per_sensor = {}

    for obs_by_sensor in groups.values():
        for sid, obs in obs_by_sensor.items():
            if sid not in cam_poses:
                continue
            R_wp, _ = pose_to_rt(obs["wp"])        # device -> world
            R_cam, _ = pose_to_rt(obs["cam"])      # device -> camera
            # up expressed in the device frame, then in the camera frame
            up_dev = R_wp.T @ up_world
            per_sensor.setdefault(sid, []).append(R_cam @ up_dev)

    print(f"\ncapture      {args.capture}")
    print(f"device       {args.device}")
    print(f"extrinsics   {cam_src}\n")
    print(f"  {'sensor':<22} {'n':>6}  {'tilt err':>9}  {'scatter':>9}   measured up in camera")

    for sid in sorted(per_sensor):
        ups = np.array(per_sensor[sid])
        mean_up = ups.mean(axis=0)
        mean_up /= np.linalg.norm(mean_up)

        Rc, _ = cam_poses[sid]
        expected = Rc.T @ up_world                 # what the config implies

        cos = np.clip(float(mean_up @ expected), -1.0, 1.0)
        tilt_err = np.degrees(np.arccos(cos))

        # dispersion of the individual estimates about their mean = confidence
        cosines = np.clip(ups @ mean_up / np.linalg.norm(ups, axis=1), -1.0, 1.0)
        scatter = float(np.degrees(np.arccos(cosines)).std())

        print(f"  {sensors[sid]['serial']:<22} {len(ups):>6}  "
              f"{tilt_err:>8.3f}°  {scatter:>8.3f}°   "
              f"({mean_up[0]:+.4f}, {mean_up[1]:+.4f}, {mean_up[2]:+.4f})")

    # Decompose: a tilt shared by every camera cannot be the cameras (they are
    # not all bumped the same way) - it is the tracked device's own fused tilt
    # being off. What differs BETWEEN cameras is genuine relative tilt error.
    sids = sorted(per_sensor)
    if len(sids) >= 2:
        errs, meas = [], []
        for sid in sids:
            ups = np.array(per_sensor[sid])
            m = ups.mean(axis=0)
            m /= np.linalg.norm(m)
            meas.append(m)
            Rc, _ = cam_poses[sid]
            expected = Rc.T @ up_world
            errs.append(np.degrees(np.arccos(np.clip(float(m @ expected), -1, 1))))

        # relative: how much the cameras disagree about up once each is mapped
        # back into the world through its own configured pose
        world_ups = []
        for sid, m in zip(sids, meas):
            Rc, _ = cam_poses[sid]
            world_ups.append(Rc @ m)
        cos = np.clip(float(world_ups[0] @ world_ups[1]), -1.0, 1.0)
        differential = np.degrees(np.arccos(cos))

        print(f"\n  common mode   {np.mean(errs):7.3f}°   shared by both cameras — this is the\n"
              f"                            tracked device's fused tilt, not the cameras\n"
              f"  differential  {differential:7.3f}°   the cameras disagree about up by this much\n"
              f"                            once each is mapped through its configured pose;\n"
              f"                            THIS is genuine relative camera tilt error")

    print("\n  tilt err = angle between the measured up-in-camera and the one the room\n"
          "             config implies. Oculus reports this per camera alongside\n"
          "             reprojection error and refuses to calibrate against a camera\n"
          "             that is not gravity aligned.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    b = sub.add_parser("baseline", help="replay today's per-camera solve + merge")
    b.add_argument("capture", type=Path)
    b.add_argument("--device", type=int, default=0)
    b.add_argument("--config", type=Path, default=None,
                   help="room config for extrinsics (default: capture campose)")
    b.add_argument("--min-blobs", type=int, default=6)
    b.add_argument("--limit", type=int, default=0)
    b.add_argument("--json", type=Path, default=None)
    b.add_argument("--joint", action="store_true",
                   help="also solve one pose per exposure over all cameras")
    b.set_defaults(func=run_baseline)

    e = sub.add_parser("export-c",
                       help="dump correspondences + Python joint pose for the C cross-check")
    e.add_argument("capture", type=Path)
    e.add_argument("--out", type=Path, required=True)
    e.add_argument("--device", type=int, default=0)
    e.add_argument("--config", type=Path, default=None)
    e.add_argument("--min-blobs", type=int, default=6)
    e.add_argument("--limit", type=int, default=200)
    e.set_defaults(func=run_export)

    g = sub.add_parser("gravity",
                       help="estimate up-in-camera and each camera's tilt error")
    g.add_argument("capture", type=Path)
    g.add_argument("--device", type=int, default=0)
    g.add_argument("--config", type=Path, default=None)
    g.add_argument("--min-blobs", type=int, default=6)
    g.set_defaults(func=run_gravity)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
