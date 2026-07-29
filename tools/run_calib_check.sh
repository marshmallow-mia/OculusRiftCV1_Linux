#!/bin/sh
# End-to-end check that the DRIVER'S OWN accumulator recovers the camera
# extrinsics from a capture, with no user calibration step and a stationary
# headset. Builds tools/calib_replay.c against the real rift-cam-calib.c.
#
#   tools/run_calib_check.sh [CAPTURE.jsonl]
#
# Targets (windows-vs-linux-tracking.md): the C estimate must agree with the
# Python bootstrap to well under a degree, and re-scoring the joint
# reconstruction with it must give < 2 mm cross-camera disagreement and 100%
# of exposures inside Oculus's 2 px acceptance.
set -e

here=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cap=${1:-$here/captures/lin/2026-07-29/live-joint.jsonl}
src=$here/../SteamVR-OpenHMD/subprojects/openhmd/src
out=${TMPDIR:-/tmp}/rift-calib-check
mkdir -p "$out"

cc -O2 -o "$out/calib_replay" "$here/tools/calib_replay.c" \
   "$src/drv_oculus_rift/rift-cam-calib.c" "$src/omath.c" \
   -I"$src/drv_oculus_rift" -I"$src" -lm

"$here/venv/bin/python" "$here/tools/replay_recon.py" bootstrap "$cap" \
    --dump "$out/pairs.txt" --limit 400 | tee "$out/py.txt"

truth=$(sed -n 's/^REL //p' "$out/py.txt")
echo
echo "=== the C accumulator (rift_cam_calib_add) on the same exposures ==="
"$out/calib_replay" "$out/pairs.txt" $truth

rel=$("$out/calib_replay" "$out/pairs.txt" | sed -n \
    's/^cam1->cam0 *pos \(.*\) quat \(.*\)$/\1 \2/p')
echo
echo "=== joint reconstruction re-scored with the C estimate ==="
"$here/venv/bin/python" "$here/tools/replay_recon.py" bootstrap "$cap" \
    --limit 400 --rel "$rel" | sed -n '/BOOTSTRAPPED/,$p'
