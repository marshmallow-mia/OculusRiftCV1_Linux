#!/usr/bin/env python
"""Offline room calibration for CV1 constellation tracking.

Consumes a raw-observation capture produced by the SteamVR-OpenHMD driver
with OHMD_RIFT_CAL_CAPTURE=<file> set: JSON lines of LED-ID-verified blob
observations from every sensor, grouped here by exposure timestamp so both
sensors constrain the same device pose instant.

  solve   Jointly fit the non-reference camera pose(s) (and, diagnostically,
          the fisheye intrinsics) by minimizing LED reprojection error over
          the whole capture, then write rift-room-config.json.
  verify  No fitting: measure how much the sensors disagree about the
          device's world position under a given room config.
  setup   Oculus-style sensor setup from two short captures: a tracking
          capture (move the headset through the play area; solves the
          sensor extrinsics) and an anchor capture that sets floor
          height, centre and forward direction — either the user
          standing at the play-area centre wearing the headset
          (--height, like the Oculus app) or the headset resting on the
          floor. Writes rift-room-config.json.

For `solve` the reference camera's pose is taken from the existing room
config and kept fixed, so the world origin, yaw and floor height do not
move. `setup` re-anchors the world at the floor mark instead — origin on
the floor under the headset, -Z where it faces — like the Windows
Oculus app's sensor setup.

Usage:
  calibrate_room.py solve  capture.jsonl [--config PATH] [--intrinsics]
                    [--max-poses N] [--min-blobs N] [--devices 0,1,2] [--dry-run]
  calibrate_room.py verify capture.jsonl [--config PATH]
  calibrate_room.py setup  hold.jsonl floor.jsonl [--config PATH] [--dry-run]
"""

import argparse
import datetime
import json
import shutil
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

DEFAULT_CONFIG = Path.home() / ".config/openhmd/rift-room-config.json"

RIFT_POSE_MATCH_LED_IDS = None  # capture is already gated on LED_IDS


# ---------------------------------------------------------------- loading

def load_capture(path, min_blobs, devices):
    sensors = {}      # sensor_id -> dict
    leds = {}         # device_id -> (pos (N,3), dir (N,3))
    observations = []
    with open(path) as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                print(f"warning: skipping malformed line {line_no}")
                continue
            t = rec.get("t")
            if t == "sensor":
                sensors[rec["s"]] = {
                    "serial": rec["serial"],
                    "K": np.array(rec["K"], dtype=float).reshape(3, 3),
                    "dist": np.array(rec["dist"][:4], dtype=float),
                    "fisheye": bool(rec["fisheye"]),
                    "width": rec["w"], "height": rec["h"],
                }
            elif t == "leds":
                pts = np.array(rec["points"], dtype=float)
                pos, dirs = pts[:, :3], pts[:, 3:6]
                m = np.abs(pos).max()
                # stored unit is unclear from the struct comment; infer it
                scale = 1.0 if m < 1.0 else (1e-3 if m < 1000.0 else 1e-6)
                leds[rec["d"]] = (pos * scale, dirs)
            elif t == "obs":
                if rec["d"] not in devices or len(rec["blobs"]) < min_blobs:
                    continue
                observations.append(rec)
    if not sensors:
        sys.exit("no sensor records in capture - was OHMD_RIFT_CAL_CAPTURE set "
                 "before SteamVR started?")
    return sensors, leds, observations


def group_by_exposure(observations):
    """(exposure_ts, device) -> {sensor_id: obs} keeping the best obs per sensor."""
    groups = defaultdict(dict)
    for o in observations:
        key = (o["ts"], o["d"])
        prev = groups[key].get(o["s"])
        if prev is None or len(o["blobs"]) > len(prev["blobs"]):
            groups[key][o["s"]] = o
    return groups


def coobserved(groups, ref_id):
    return {k: v for k, v in groups.items() if len(v) >= 2 and ref_id in v}


def subsample(groups, max_poses):
    """Spatially diverse subset: bucket by fused world position and yaw,
    round-robin across buckets so no desk-corner dominates the fit."""
    if len(groups) <= max_poses:
        return dict(groups)
    buckets = defaultdict(list)
    for key, obs_by_sensor in groups.items():
        o = next(iter(obs_by_sensor.values()))
        px, py, pz = o["wp"][0:3]
        qx, qy, qz, qw = o["wp"][3:7]
        yaw = np.degrees(np.arctan2(2 * (qw * qy + qx * qz),
                                    1 - 2 * (qy * qy + qx * qx)))
        b = (int(px / 0.15), int(py / 0.15), int(pz / 0.15), int(yaw / 30))
        buckets[b].append(key)
    picked = []
    lists = list(buckets.values())
    i = 0
    while len(picked) < max_poses and any(lists):
        for lst in lists:
            if i < len(lst):
                picked.append(lst[i])
                if len(picked) >= max_poses:
                    break
        lists = [l for l in lists if len(l) > i]
        i += 1
    return {k: groups[k] for k in picked}


# ---------------------------------------------------------------- geometry

def pose_to_rt(p7):
    """[px,py,pz,qx,qy,qz,qw] -> (R, t) mapping local -> parent frame."""
    t = np.asarray(p7[0:3], dtype=float)
    R = Rotation.from_quat(p7[3:7]).as_matrix()
    return R, t


def project(Xc, K, D):
    """OpenCV fisheye (equidistant + k1..k4) projection of camera-frame pts."""
    x = Xc[:, 0] / Xc[:, 2]
    y = Xc[:, 1] / Xc[:, 2]
    r = np.sqrt(x * x + y * y)
    th = np.arctan(r)
    th2 = th * th
    th_d = th * (1 + D[0] * th2 + D[1] * th2**2 + D[2] * th2**3 + D[3] * th2**4)
    s = np.where(r > 1e-9, th_d / np.maximum(r, 1e-9), 1.0)
    u = K[0, 0] * x * s + K[0, 2]
    v = K[1, 1] * y * s + K[1, 2]
    return np.stack([u, v], axis=1)


def load_config(path):
    cfg = json.loads(Path(path).read_text())
    poses = {}
    for s in cfg["sensors"]:
        poses[s["serial"]] = (np.array(s["pos"], dtype=float),
                              np.array(s["orient"], dtype=float))  # x,y,z,w
    return cfg, poses


# ---------------------------------------------------------------- verify

def sensor_world_positions(obs_by_sensor, cam_poses):
    """Per-sensor device world pose implied by that sensor's PnP alone."""
    out = {}
    for sid, o in obs_by_sensor.items():
        if sid not in cam_poses:
            continue
        Rc, tc = cam_poses[sid]
        Ro_c, to_c = pose_to_rt(o["cam"])
        out[sid] = (Rc @ Ro_c, Rc @ to_c + tc)
    return out


def print_disagreement(groups, cam_poses, label):
    dpos, dang = [], []
    for obs_by_sensor in groups.values():
        w = sensor_world_positions(obs_by_sensor, cam_poses)
        if len(w) < 2:
            continue
        sids = sorted(w)
        (Ra, ta), (Rb, tb) = w[sids[0]], w[sids[1]]
        dpos.append(np.linalg.norm(ta - tb))
        cosang = (np.trace(Ra.T @ Rb) - 1) / 2
        dang.append(np.degrees(np.arccos(np.clip(cosang, -1, 1))))
    dpos, dang = np.array(dpos) * 1000, np.array(dang)
    if len(dpos) == 0:
        print(f"{label}: no co-observed exposures")
        return
    print(f"{label}: {len(dpos)} co-observed exposures")
    print(f"  position disagreement  mean {dpos.mean():7.2f}  median "
          f"{np.median(dpos):7.2f}  p95 {np.percentile(dpos, 95):7.2f}  "
          f"max {dpos.max():7.2f}  (mm)")
    print(f"  rotation disagreement  mean {dang.mean():7.3f}  median "
          f"{np.median(dang):7.3f}  p95 {np.percentile(dang, 95):7.3f}  (deg)")


# ---------------------------------------------------------------- solve

def average_poses(Rs, ts):
    """Chordal-mean rotation (sign-aligned quaternion mean) + mean position."""
    q = Rotation.from_matrix(Rs).as_quat()
    q = np.where((q @ q[0])[:, None] < 0, -q, q)
    qm = q.mean(axis=0)
    Rm = Rotation.from_quat(qm / np.linalg.norm(qm)).as_matrix()
    return Rm, np.asarray(ts).mean(axis=0)


def robust_average_poses(Rs, ts):
    """average_poses with one round of 3-sigma outlier rejection.
    Returns (R, t, keep_mask)."""
    Rs, ts = np.asarray(Rs), np.asarray(ts)
    Rm, tm = average_poses(Rs, ts)
    ang = Rotation.from_matrix(Rs @ Rm.T).magnitude()
    dt = np.linalg.norm(ts - tm, axis=1)
    keep = (ang < np.median(ang) + 3 * ang.std()) & (dt < np.median(dt) + 3 * dt.std())
    if keep.sum() >= 3:
        Rm, tm = average_poses(Rs[keep], ts[keep])
    return Rm, tm, keep


def relative_pose_estimate(groups, ref_id, sid):
    """Closed-form estimate of camera sid's pose relative to the reference
    camera: every co-observed exposure's pair of PnP poses implies the same
    (constant) relative transform, so robustly average them."""
    Rs, ts = [], []
    for obs_by_sensor in groups.values():
        if ref_id not in obs_by_sensor or sid not in obs_by_sensor:
            continue
        Ra, ta = pose_to_rt(obs_by_sensor[ref_id]["cam"])  # obj -> ref cam
        Rb, tb = pose_to_rt(obs_by_sensor[sid]["cam"])     # obj -> cam sid
        R = Ra @ Rb.T                                      # cam sid -> ref cam
        Rs.append(R)
        ts.append(ta - R @ tb)
    Rm, tm, keep = robust_average_poses(Rs, ts)
    spread = np.linalg.norm(np.asarray(ts)[keep] - tm, axis=1)
    print(f"  stage 1: relative pose from {keep.sum()}/{len(ts)} exposure pairs, "
          f"per-exposure scatter median {np.median(spread)*1000:.1f} mm")
    return Rm, tm


class Problem:
    """Flattened arrays over every matched blob for vectorized residuals."""

    def __init__(self, groups, sensors, leds, ref_id, cam_poses, solve_intrinsics):
        self.sensors = sensors
        self.ref_id = ref_id
        self.solve_intrinsics = solve_intrinsics
        self.sensor_ids = sorted(sensors)
        self.solved_cams = [s for s in self.sensor_ids if s != ref_id]
        self.group_keys = sorted(groups)

        # parameter layout
        self.n_cam_par = 6 * len(self.solved_cams)
        self.n_intr_par = 8 * len(self.sensor_ids) if solve_intrinsics else 0
        self.n_par = self.n_cam_par + self.n_intr_par + 6 * len(self.group_keys)

        pts, uv, sensor_idx, group_idx = [], [], [], []
        for gi, key in enumerate(self.group_keys):
            dev = key[1]
            led_pos = leds[dev][0]
            for sid, o in sorted(groups[key].items()):
                ids = np.array([b[0] for b in o["blobs"]], dtype=int)
                keep = ids < len(led_pos)
                ids = ids[keep]
                if len(ids) == 0:
                    continue
                bl = np.array([b[1:3] for b in o["blobs"]], dtype=float)[keep]
                pts.append(led_pos[ids])
                uv.append(bl)
                sensor_idx.append(np.full(len(ids), sid))
                group_idx.append(np.full(len(ids), gi))
        self.pts = np.concatenate(pts)
        self.uv = np.concatenate(uv)
        self.sensor_idx = np.concatenate(sensor_idx)
        self.group_idx = np.concatenate(group_idx)
        self.n_blob = len(self.pts)

        # initial parameters: cameras from the closed-form relative-pose
        # average (NOT from the possibly-bad config), objects from ref PnP
        Rr, tr = cam_poses[ref_id]
        self.ref_pose = (Rr, tr)
        x0 = []
        for sid in self.solved_cams:
            R_rel, t_rel = relative_pose_estimate(groups, ref_id, sid)
            R = Rr @ R_rel
            t = Rr @ t_rel + tr
            x0.extend(Rotation.from_matrix(R).as_rotvec())
            x0.extend(t)
        self.intr0 = {}
        if solve_intrinsics:
            for sid in self.sensor_ids:
                K, D = sensors[sid]["K"], sensors[sid]["dist"]
                p = [K[0, 0], K[1, 1], K[0, 2], K[1, 2], *D]
                self.intr0[sid] = np.array(p)
                x0.extend(p)
        for key in self.group_keys:
            # init object pose from the reference camera's PnP
            o = groups[key][ref_id]
            Ro_c, to_c = pose_to_rt(o["cam"])
            Ro = Rr @ Ro_c
            to = Rr @ to_c + tr
            x0.extend(Rotation.from_matrix(Ro).as_rotvec())
            x0.extend(to)
        self.x0 = np.array(x0)

    # -- parameter unpacking helpers
    def cam_pose(self, x, sid):
        if sid == self.ref_id:
            return self.ref_pose
        i = 6 * self.solved_cams.index(sid)
        R = Rotation.from_rotvec(x[i:i + 3]).as_matrix()
        return R, x[i + 3:i + 6]

    def intrinsics(self, x, sid):
        if not self.solve_intrinsics:
            K = self.sensors[sid]["K"]
            return K, self.sensors[sid]["dist"]
        i = self.n_cam_par + 8 * self.sensor_ids.index(sid)
        fx, fy, cx, cy, k1, k2, k3, k4 = x[i:i + 8]
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
        return K, np.array([k1, k2, k3, k4])

    def group_poses(self, x):
        base = self.n_cam_par + self.n_intr_par
        g = x[base:].reshape(-1, 6)
        Rg = Rotation.from_rotvec(g[:, :3]).as_matrix()
        return Rg, g[:, 3:6]

    def residuals(self, x):
        Rg, tg = self.group_poses(x)
        # world coords of every blob's LED
        Xw = np.einsum("nij,nj->ni", Rg[self.group_idx], self.pts) + tg[self.group_idx]
        res = np.empty((self.n_blob, 2))
        for sid in self.sensor_ids:
            m = self.sensor_idx == sid
            if not m.any():
                continue
            Rc, tc = self.cam_pose(x, sid)
            K, D = self.intrinsics(x, sid)
            Xc = (Xw[m] - tc) @ Rc  # == Rc.T @ (Xw - tc)
            res[m] = project(Xc, K, D) - self.uv[m]
        out = [res.ravel()]
        if self.solve_intrinsics:
            # keep intrinsics near the EEPROM values
            sig = []
            pri = []
            for sid in self.sensor_ids:
                p0 = self.intr0[sid]
                i = self.n_cam_par + 8 * self.sensor_ids.index(sid)
                pri.append(x[i:i + 8] - p0)
                sig.append([0.01 * p0[0], 0.01 * p0[1], 4.0, 4.0,
                            0.02, 0.02, 0.02, 0.02])
            out.append((np.concatenate(pri) / np.concatenate(sig)))
        return np.concatenate(out)

    def sparsity(self):
        n_res = 2 * self.n_blob + (self.n_intr_par if self.solve_intrinsics else 0)
        S = lil_matrix((n_res, self.n_par), dtype=int)
        base = self.n_cam_par + self.n_intr_par
        rows = np.arange(self.n_blob)
        for sid in self.solved_cams:
            m = self.sensor_idx == sid
            i = 6 * self.solved_cams.index(sid)
            for r in rows[m]:
                S[2 * r:2 * r + 2, i:i + 6] = 1
        if self.solve_intrinsics:
            for sid in self.sensor_ids:
                m = self.sensor_idx == sid
                i = self.n_cam_par + 8 * self.sensor_ids.index(sid)
                for r in rows[m]:
                    S[2 * r:2 * r + 2, i:i + 8] = 1
                # prior rows
                pr = 2 * self.n_blob + 8 * self.sensor_ids.index(sid)
                S[pr:pr + 8, i:i + 8] = np.eye(8, dtype=int)
        for r in rows:
            g = base + 6 * self.group_idx[r]
            S[2 * r:2 * r + 2, g:g + 6] = 1
        return S

    def report_residuals(self, x, label):
        r = self.residuals(x)[:2 * self.n_blob].reshape(-1, 2)
        err = np.linalg.norm(r, axis=1)
        print(f"{label}: reprojection error over {self.n_blob} blobs")
        for sid in self.sensor_ids:
            m = self.sensor_idx == sid
            e = err[m]
            print(f"  sensor {sid} ({self.sensors[sid]['serial']}): "
                  f"rms {np.sqrt((e**2).mean()):6.3f}  median {np.median(e):6.3f}  "
                  f"p95 {np.percentile(e, 95):6.3f} px  ({m.sum()} blobs)")
        # radial profile: systematic growth toward the image edge implies
        # the fisheye model, not the extrinsics, is what limits accuracy
        for sid in self.sensor_ids:
            m = self.sensor_idx == sid
            K = self.sensors[sid]["K"]
            rad = np.linalg.norm(self.uv[m] - [K[0, 2], K[1, 2]], axis=1)
            bins = np.linspace(0, rad.max() + 1e-6, 7)
            prof = []
            for b0, b1 in zip(bins[:-1], bins[1:]):
                sel = (rad >= b0) & (rad < b1)
                prof.append(f"{np.median(err[m][sel]):5.2f}" if sel.any() else "    -")
            print(f"  sensor {sid} median err by image radius: [{' '.join(prof)}] px")


def cmd_coverage(args):
    """Is the capture good enough to solve yet? Safe to run mid-capture."""
    sensors, leds, obs = load_capture(args.capture, args.min_blobs, {0})
    groups = {k: v for k, v in group_by_exposure(obs).items() if len(v) >= 2}
    if not groups:
        sys.exit("no co-observed exposures yet - is the HMD visible to both sensors?")

    ts = np.array([k[0] for k in groups.keys()], dtype=float)
    minutes = (ts.max() - ts.min()) / 60e9

    voxels, yaws = set(), set()
    for obs_by_sensor in groups.values():
        o = next(iter(obs_by_sensor.values()))
        px, py, pz = o["wp"][0:3]
        qx, qy, qz, qw = o["wp"][3:7]
        yaw = np.degrees(np.arctan2(2 * (qw * qy + qx * qz),
                                    1 - 2 * (qy * qy + qx * qx)))
        voxels.add((int(px / 0.25), int(py / 0.25), int(pz / 0.25)))
        yaws.add(int((yaw + 180) / 30) % 12)

    checks = [
        ("co-observed exposures", len(groups), 1500),
        ("capture duration (min)", minutes, 3.0),
        ("25cm-cube positions visited", len(voxels), 30),
        ("yaw directions covered (of 12)", len(yaws), 8),
    ]
    ok = True
    for label, value, need in checks:
        good = value >= need
        ok &= good
        val = f"{value:.1f}" if isinstance(value, float) else str(value)
        print(f"  [{'ok' if good else '..'}] {label:32s} {val:>8}  (want >= {need})")
    print("\nDONE - quit SteamVR and run solve" if ok else
          "\nkeep moving - vary position AND facing direction, stay visible to both sensors")


def cmd_solve(args):
    devices = set(int(d) for d in args.devices.split(","))
    sensors, leds, obs = load_capture(args.capture, args.min_blobs, devices)
    cfg, cfg_poses = load_config(args.config)

    serial_to_id = {v["serial"]: k for k, v in sensors.items()}
    cam_poses = {}
    for serial, (pos, quat) in cfg_poses.items():
        if serial in serial_to_id:
            cam_poses[serial_to_id[serial]] = (Rotation.from_quat(quat).as_matrix(), pos)
    missing = set(sensors) - set(cam_poses)
    if missing:
        sys.exit(f"sensors {missing} not in room config {args.config}")

    ref_id = (serial_to_id[args.ref_serial] if args.ref_serial
              else min(sensors))
    print(f"reference sensor: {ref_id} ({sensors[ref_id]['serial']}) - pose kept fixed")

    groups = coobserved(group_by_exposure(obs), ref_id)
    print(f"{len(obs)} usable observations, {len(groups)} co-observed exposures")
    if len(groups) < 50:
        sys.exit("not enough co-observed exposures - capture more data with the "
                 "HMD visible to BOTH sensors")
    kept = subsample(groups, args.max_poses)
    if len(kept) < len(groups):
        print(f"subsampled to {len(kept)} spatially-diverse exposures "
              f"(--max-poses {args.max_poses})")

    print_disagreement(kept, cam_poses, "\nbefore (current config)")

    prob = Problem(kept, sensors, leds, ref_id, cam_poses, args.intrinsics)
    print(f"\n{prob.n_par} parameters, {2 * prob.n_blob} reprojection residuals")
    prob.report_residuals(prob.x0, "initial")

    result = least_squares(
        prob.residuals, prob.x0, jac_sparsity=prob.sparsity(),
        loss="huber", f_scale=2.0, x_scale="jac", tr_solver="lsmr",
        max_nfev=2000, verbose=1)
    prob.report_residuals(result.x, "\nfinal")

    new_cam_poses = dict(cam_poses)
    for sid in prob.solved_cams:
        R, t = prob.cam_pose(result.x, sid)
        R = np.array(R)
        old_R, old_t = cam_poses[sid]
        dpos = np.linalg.norm(t - old_t) * 1000
        dang = np.degrees(np.arccos(np.clip((np.trace(old_R.T @ R) - 1) / 2, -1, 1)))
        print(f"\nsensor {sid} ({sensors[sid]['serial']}) moved "
              f"{dpos:.1f} mm / {dang:.3f} deg from config value")
        print(f"  pos    {t[0]:.6f} {t[1]:.6f} {t[2]:.6f}")
        q = Rotation.from_matrix(R).as_quat()
        print(f"  orient {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}")
        new_cam_poses[sid] = (R, t)

    print_disagreement(kept, new_cam_poses, "\nafter (solved extrinsics)")

    if args.intrinsics:
        print("\nsolved intrinsics deltas (DIAGNOSTIC ONLY - the driver reads "
              "intrinsics from sensor EEPROM and cannot load these):")
        for sid in prob.sensor_ids:
            K, D = prob.intrinsics(result.x, sid)
            p0 = prob.intr0[sid]
            print(f"  sensor {sid}: dfx {K[0,0]-p0[0]:+.2f} dfy {K[1,1]-p0[1]:+.2f} "
                  f"dcx {K[0,2]-p0[2]:+.2f} dcy {K[1,2]-p0[3]:+.2f} "
                  f"dk {D[0]-p0[4]:+.4f} {D[1]-p0[5]:+.4f} {D[2]-p0[6]:+.4f} {D[3]-p0[7]:+.4f}")
        print("intrinsics were solved jointly, so the extrinsics above pair with "
              "them - NOT writing the config. Re-run without --intrinsics to "
              "produce a config that matches the EEPROM intrinsics.")
        return

    if args.dry_run:
        print("\n--dry-run: not writing config")
        return

    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup = f"{args.config}.bak-{stamp}"
    shutil.copy(args.config, backup)
    for s in cfg["sensors"]:
        sid = serial_to_id.get(s["serial"])
        if sid is not None and sid in new_cam_poses:
            R, t = new_cam_poses[sid]
            s["pos"] = [round(float(v), 6) for v in t]
            s["orient"] = [round(float(v), 6) for v in Rotation.from_matrix(R).as_quat()]
    Path(args.config).write_text(json.dumps(cfg, indent=2) + "\n")
    print(f"\nwrote {args.config} (backup: {backup})")
    print("restart SteamVR to pick up the new calibration")


def cmd_verify(args):
    sensors, leds, obs = load_capture(args.capture, args.min_blobs, {0, 1, 2})
    cfg, cfg_poses = load_config(args.config)
    serial_to_id = {v["serial"]: k for k, v in sensors.items()}
    cam_poses = {}
    for serial, (pos, quat) in cfg_poses.items():
        if serial in serial_to_id:
            cam_poses[serial_to_id[serial]] = (Rotation.from_quat(quat).as_matrix(), pos)
    groups = {k: v for k, v in group_by_exposure(obs).items() if len(v) >= 2}
    print_disagreement(groups, cam_poses, f"config {args.config}")


# ---------------------------------------------------------------- setup

# LED-model origin height above the floor when the CV1 rests on it
# (it sits on the facial-interface foam and the strap)
HMD_FLOOR_HEIGHT = 0.045

# eye height below body height: the Oculus SDK's defaults are
# PlayerHeight 1.778 m vs EyeHeight 1.675 m
EYE_HEIGHT_OFFSET = 0.103


def ref_pose_from_capture(observations, ref_id):
    """World pose of the reference camera implied by the driver's own fused
    world poses: world_T_cam = world_T_dev @ inv(cam_T_dev).

    This inherits the driver's gravity alignment — from the room config if
    one was loaded, else from the IMU-seeded session bootstrap — so a
    fresh setup still ends up with a level floor. None if the capture has
    no usable fused poses."""
    Rs, ts = [], []
    for o in observations:
        if o["s"] != ref_id:
            continue
        wp = o["wp"]
        # skip pre-lock zeros and fusion glitches
        if np.linalg.norm(wp[0:3]) < 1e-6 or max(abs(v) for v in wp[0:3]) > 10:
            continue
        R_cd, t_cd = pose_to_rt(o["cam"])
        R_wd, t_wd = pose_to_rt(wp)
        R = R_wd @ R_cd.T
        Rs.append(R)
        ts.append(t_wd - R @ t_cd)
    if len(Rs) < 20:
        return None
    Rm, tm, keep = robust_average_poses(Rs, ts)
    spread = np.linalg.norm(np.asarray(ts)[keep] - tm, axis=1)
    print(f"  gravity reference from {keep.sum()}/{len(ts)} fused poses, "
          f"scatter median {np.median(spread)*1000:.1f} mm")
    return Rm, tm


def device_world_poses(observations, poses_by_sid):
    """(positions (N,3), forward floor-vectors (N,2)) of the device under
    the given per-sensor world poses. The optical pose is in the LED-model
    frame, where the faceplate LEDs emit along +Z — the wearer faces +Z
    (NOT -Z; that OpenGL-view assumption put the sensors behind the user)."""
    pos, fwd = [], []
    for o in observations:
        if o["s"] not in poses_by_sid:
            continue
        Rc, tc = poses_by_sid[o["s"]]
        R_cd, t_cd = pose_to_rt(o["cam"])
        Rw = Rc @ R_cd
        pos.append(Rc @ t_cd + tc)
        fwd.append(Rw[:, 2])
    return np.array(pos), np.array(fwd)


def mean_floor_forward(fwd, min_norm=0.3):
    """Mean floor-plane forward direction (fx, fz), or None if the device
    points too close to straight up/down for the projection to mean much."""
    if len(fwd) == 0:
        return None
    f2 = fwd[:, [0, 2]]
    n = np.linalg.norm(f2, axis=1)
    good = n > min_norm
    if good.sum() < min(10, len(fwd)):
        return None
    fx, fz = (f2[good] / n[good, None]).mean(axis=0)
    n = np.hypot(fx, fz)
    if n < 1e-6:
        return None
    return fx / n, fz / n


def placement_check(poses):
    """Advisory placement messages for sensor poses in the anchored world
    (origin = where the user stood). Mirrors the Oculus client's checks:
    origin distance, equidistance, sensor separation, aim angle."""
    msgs = []
    dists = {}
    for serial, (R, t) in poses.items():
        dists[serial] = np.hypot(t[0], t[2])
        # the camera looks along its +Z axis; it should aim at the origin
        f = np.asarray(R)[:, 2]
        fn = np.hypot(f[0], f[2])
        on = np.hypot(t[0], t[2])
        if fn > 1e-6 and on > 1e-6:
            cosang = -(f[0] * t[0] + f[2] * t[2]) / (fn * on)
            if cosang < np.cos(np.radians(40)):
                msgs.append(f"sensor {serial} is not aimed at your play "
                            "area — rotate and tilt it toward you")
    if dists:
        dmin, dmax = min(dists.values()), max(dists.values())
        if dmin < 0.9:
            msgs.append("you were too close to your sensors")
        elif dmax > 2.7:
            msgs.append("you were too far away from your sensors")
        elif len(dists) >= 2 and dmax / max(dmin, 1e-6) > 1.6:
            msgs.append("you were not centered between your sensors — "
                        "each sensor should be about equally far from you")
    if len(poses) >= 2:
        pts = [t for _R, t in poses.values()]
        width = max(np.hypot(a[0] - b[0], a[2] - b[2])
                    for i, a in enumerate(pts) for b in pts[i + 1:])
        if width < 0.9:
            msgs.append("your sensors are close together — moving them "
                        "farther apart improves tracking")
    return msgs


def cmd_setup(args):
    print("stage: reading captures")
    sensors, leds, obs = load_capture(args.hold, args.min_blobs, {0})
    if 0 not in leds:
        sys.exit("hold capture has no HMD LED model record — capture is "
                 "incomplete, redo the hold step")

    try:
        cfg, cfg_poses = load_config(args.config)
    except (OSError, ValueError):
        cfg, cfg_poses = {}, {}          # first-ever setup: no config yet
    cfg.setdefault("room-center-offset", [0.0, 0.0, 0.0])
    cfg.setdefault("room-yaw-offset", 0.0)
    cfg.setdefault("sensors", [])

    ref_id = min(sensors)
    ref_serial = sensors[ref_id]["serial"]
    print(f"{len(sensors)} sensor(s) in the hold capture, "
          f"reference: {ref_serial}")

    ref = ref_pose_from_capture(obs, ref_id)
    if ref is None and ref_serial in cfg_poses:
        pos, quat = cfg_poses[ref_serial]
        ref = (Rotation.from_quat(quat).as_matrix(), pos)
        print("warning: no usable fused poses in the hold capture — using "
              "the existing config pose for gravity alignment")
    if ref is None:
        ref = (np.eye(3), np.array([0.0, 1.0, 0.0]))
        print("warning: no gravity reference available — assuming the "
              "reference sensor is level")

    # sensor extrinsics in the (pre-anchor) reference frame, by serial
    solved = {ref_serial: ref}
    if len(sensors) >= 2:
        print("stage: locating sensors")
        groups = coobserved(group_by_exposure(obs), ref_id)
        print(f"{len(groups)} co-observed exposures")
        if len(groups) < 100:
            sys.exit("not enough co-observed exposures — hold the headset "
                     "where its front LEDs are visible to ALL sensors")

        # a sensor that (almost) never saw the headset together with the
        # reference cannot be solved — keep its config pose instead of
        # feeding unconstrained parameters to the fit
        pairs = {}
        for obs_by_sensor in groups.values():
            for sid in obs_by_sensor:
                if sid != ref_id:
                    pairs[sid] = pairs.get(sid, 0) + 1
        weak = {sid for sid in sensors
                if sid != ref_id and pairs.get(sid, 0) < 50}
        for sid in weak:
            print(f"warning: sensor {sensors[sid]['serial']} shares only "
                  f"{pairs.get(sid, 0)} exposures with the reference — "
                  "keeping its existing config pose")
        solve_sensors = {sid: s for sid, s in sensors.items()
                         if sid not in weak}
        if weak:
            obs = [o for o in obs if o["s"] not in weak]
            groups = coobserved(group_by_exposure(obs), ref_id)

        kept = subsample(groups, args.max_poses)
        prob = Problem(kept, solve_sensors, leds, ref_id, {ref_id: ref},
                       False)
        result = least_squares(
            prob.residuals, prob.x0, jac_sparsity=prob.sparsity(),
            loss="huber", f_scale=2.0, x_scale="jac", tr_solver="lsmr",
            max_nfev=200)
        prob.report_residuals(result.x, "hold-capture fit")
        for sid in prob.solved_cams:
            R, t = prob.cam_pose(result.x, sid)
            solved[sensors[sid]["serial"]] = (np.array(R), np.array(t))
        print_disagreement(
            kept, {sid: solved[s["serial"]]
                   for sid, s in solve_sensors.items()},
            "solved extrinsics")

    print("stage: setting floor and centre")
    fsensors, _fleds, fobs = load_capture(args.floor, args.min_blobs, {0})
    fpose = {sid: solved[s["serial"]] for sid, s in fsensors.items()
             if s["serial"] in solved}
    pos, fwd = device_world_poses(fobs, fpose)
    if len(pos) < 30:
        sys.exit("could not see the headset during the anchor capture — "
                 "keep it in view of a sensor and redo that step")
    p = np.median(pos, axis=0)
    scatter = np.linalg.norm(pos - p, axis=1)
    print(f"  anchor point from {len(pos)} observations, scatter p95 "
          f"{np.percentile(scatter, 95) * 1000:.1f} mm")

    forward = mean_floor_forward(fwd)
    if forward is None:
        # headset pointing up/down on the floor — fall back to where it
        # faced while held up (the user was facing the sensors then too)
        hpose = {sid: solved[s["serial"]] for sid, s in sensors.items()
                 if s["serial"] in solved}
        _hp, hfwd = device_world_poses(obs, hpose)
        forward = mean_floor_forward(hfwd)
        print("  floor forward direction ambiguous — using the hold-phase "
              "facing direction")
    if forward is None:
        forward = (0.0, -1.0)
        print("warning: no usable forward direction — keeping -Z")
    fx, fz = forward

    # rigid world re-anchor: origin on the floor under the headset, y=0 at
    # the floor, -Z where the headset faces (rows = new axes in old coords)
    if args.height:
        # headset worn at eye level by a user of known height
        floor_y = p[1] - max(args.height - EYE_HEIGHT_OFFSET, 0.5)
    else:
        # headset resting on the floor
        floor_y = p[1] - HMD_FLOOR_HEIGHT
    Rw = np.array([[-fz, 0.0, fx], [0.0, 1.0, 0.0], [-fx, 0.0, -fz]])
    t0 = np.array([p[0], floor_y, p[2]])
    print(f"  floor height {floor_y:+.3f} m, forward yaw "
          f"{np.degrees(np.arctan2(fx, fz)):.0f}° (old frame)")

    # every sensor shares the world: transform config sensors that were not
    # part of this setup too, and overwrite the ones we just solved
    final = {serial: (Rotation.from_quat(quat).as_matrix(), np.asarray(t))
             for serial, (t, quat) in cfg_poses.items()}
    final.update(solved)
    for serial, (R, t) in final.items():
        final[serial] = (Rw @ R, Rw @ (t - t0))

    for serial, (R, t) in sorted(final.items()):
        note = "" if serial in solved else "  (from config, not re-solved)"
        print(f"  sensor {serial}: pos {t[0]:+.3f} {t[1]:+.3f} {t[2]:+.3f}"
              f"{note}")
        if t[1] < 0.2:
            print(f"    warning: sensor {serial} ends up {t[1]:.2f} m above "
                  "the floor — that is unusually low, check the result")

    # placement advisory in the new world (origin = the user's standing
    # spot) — the same checks the Oculus client renders after tracking:
    # per-sensor distance from origin, equidistance, inter-sensor width,
    # and whether each sensor is aimed at the play area
    for msg in placement_check({s: final[s] for s in solved}):
        print("placement: " + msg)

    cfg["room-center-offset"] = [0.0, 0.0, 0.0]
    cfg["room-yaw-offset"] = 0.0
    by_serial = {s.get("serial"): s for s in cfg["sensors"]}
    for serial, (R, t) in final.items():
        entry = by_serial.get(serial)
        if entry is None:
            entry = {"serial": serial}
            cfg["sensors"].append(entry)
        entry["pos"] = [round(float(v), 6) for v in t]
        entry["orient"] = [round(float(v), 6)
                           for v in Rotation.from_matrix(R).as_quat()]

    if args.dry_run:
        print("\n--dry-run: not writing config")
        return
    cfg_path = Path(args.config)
    if cfg_path.exists():
        stamp = datetime.datetime.now(
            datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        backup = f"{args.config}.bak-{stamp}"
        shutil.copy(args.config, backup)
        print(f"backed up old config to {backup}")
    cfg_path.parent.mkdir(parents=True, exist_ok=True)
    cfg_path.write_text(json.dumps(cfg, indent=2) + "\n")
    print(f"wrote {args.config}")
    print("SETUP-COMPLETE")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("solve", help="fit camera extrinsics and write room config")
    s.add_argument("capture")
    s.add_argument("--config", default=str(DEFAULT_CONFIG))
    s.add_argument("--ref-serial", help="serial of the fixed reference sensor")
    s.add_argument("--intrinsics", action="store_true",
                   help="also solve fisheye intrinsics (diagnostic, no config write)")
    s.add_argument("--max-poses", type=int, default=1500)
    s.add_argument("--min-blobs", type=int, default=6)
    s.add_argument("--devices", default="0", help="device ids to use (default HMD only)")
    s.add_argument("--dry-run", action="store_true")
    s.set_defaults(func=cmd_solve)

    v = sub.add_parser("verify", help="measure inter-sensor disagreement")
    v.add_argument("capture")
    v.add_argument("--config", default=str(DEFAULT_CONFIG))
    v.add_argument("--min-blobs", type=int, default=6)
    v.set_defaults(func=cmd_verify)

    c = sub.add_parser("coverage", help="check if a capture has enough data yet")
    c.add_argument("capture")
    c.add_argument("--min-blobs", type=int, default=6)
    c.set_defaults(func=cmd_coverage)

    u = sub.add_parser("setup", help="Oculus-style sensor setup: tracking + "
                                     "anchor captures -> full room config")
    u.add_argument("hold", help="capture from the sensor-tracking step "
                                "(headset moved through the play area)")
    u.add_argument("floor", help="anchor capture (standing at the centre "
                                 "with --height, else headset on the floor)")
    u.add_argument("--config", default=str(DEFAULT_CONFIG))
    u.add_argument("--height", type=float, default=None,
                   help="user height in metres; the anchor capture is then "
                        "the user standing at the play-area centre wearing "
                        "the headset (Oculus-style height-based floor)")
    u.add_argument("--max-poses", type=int, default=400)
    u.add_argument("--min-blobs", type=int, default=6)
    u.add_argument("--dry-run", action="store_true")
    u.set_defaults(func=cmd_setup)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
