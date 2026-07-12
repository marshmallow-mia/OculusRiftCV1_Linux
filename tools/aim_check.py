#!/usr/bin/env python3
"""Aim check: how much of the play area does each sensor actually see?

Room calibration needs exposures where BOTH sensors see the headset at the same
instant — only those tie the two cameras together. A sensor that is powered,
streaming and USB3-happy can still contribute nothing if it is pointed at a wall.
This measures that directly.

Run it, and while it runs move the headset around the play area the way you
actually play. It reports each sensor's observation rate and, crucially, the
co-observed rate.

    python3 tools/aim_check.py [seconds]

Re-run after nudging a sensor until the co-observed rate is high. SteamVR must be
closed.
"""
import collections
import json
import os
import subprocess
import sys
import tempfile

POSE_LOG = os.path.expanduser(
    "~/git/SteamVR-OpenHMD/build/subprojects/openhmd/openhmd_pose_log")


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
    tmp = tempfile.mkdtemp(prefix="aimcheck-")
    cap = os.path.join(tmp, "cap.jsonl")

    print(f"Capturing {secs:.0f}s — move the headset around the play area now.\n")
    env = dict(os.environ, OHMD_RIFT_CAL_CAPTURE=cap)
    subprocess.run([POSE_LOG, os.path.join(tmp, "poses.csv"), str(secs), "100"],
                   env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    if not os.path.exists(cap):
        sys.exit("no capture file — is the HMD connected and SteamVR closed?")

    serials, per, exp = {}, collections.Counter(), collections.defaultdict(set)
    for line in open(cap):
        try:
            r = json.loads(line)
        except ValueError:
            continue
        if r.get("t") == "sensor":
            serials[r["s"]] = r["serial"]
        elif r.get("t") == "obs":
            per[r["s"]] += 1
            exp[r["ts"]].add(r["s"])

    if not exp:
        sys.exit("no LED observations at all — headset not visible to any sensor.")

    both = sum(1 for v in exp.values() if len(v) > 1)
    print(f"{'sensor':<18} {'obs':>7} {'rate':>10}   share of exposures")
    for sid in sorted(serials):
        n = per[sid]
        share = 100.0 * n / len(exp)
        bar = "#" * int(share / 4)
        print(f"{serials[sid]:<18} {n:>7} {n / secs:>7.1f}/s   {share:5.1f}%  {bar}")

    print(f"\nexposures            {len(exp)}")
    print(f"co-observed (BOTH)   {both}  ({100.0 * both / len(exp):.1f}%)"
          f"  = {both / secs:.1f}/s")

    print()
    if both / secs >= 5:
        print("GOOD — both sensors see the headset together. Calibration capture "
              "will converge; a 3-5 min walk gives the ~1500 co-observed exposures "
              "the solver wants.")
    elif both / secs >= 1:
        print("WEAK — some overlap, but a full capture would take a long time. "
              "Aim the sensors so their cones overlap over the space you actually "
              "play in.")
    else:
        print("BAD — the sensors almost never see the headset at the same time. "
              "Room calibration CANNOT be solved from this. Re-aim the weaker "
              "sensor above at the play area (and check nothing occludes it).")
        weak = min(serials, key=lambda s: per[s])
        print(f"     weakest: {serials[weak]} ({per[weak]} obs)")


if __name__ == "__main__":
    main()
