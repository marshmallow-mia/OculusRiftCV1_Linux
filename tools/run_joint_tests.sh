#!/usr/bin/env bash
# Build and run the joint-pose unit tests WITHOUT libopenhmd.
#
# The normal path is `meson test -C build` in the openhmd tree, but that needs
# libopenhmd to link, which currently fails: the system moved to OpenCV 5
# (2026-07-26) and rift-sensor-opencv.cpp still uses the OpenCV 4 header
# layout. The joint solver itself has no OpenCV dependency, so it can be built
# and tested standalone — which is how it was verified.
#
# Usage: tools/run_joint_tests.sh [openhmd_dir]
set -euo pipefail

OHMD="${1:-$HOME/git/SteamVR-OpenHMD/subprojects/openhmd}"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cat > "$OUT/main.c" <<'EOF'
#include <string.h>
#include <stdlib.h>
#include "tests.h"

bool float_eq(float a, float b, float t) { return fabsf(a - b) < t; }
bool vec3f_eq(vec3f v1, vec3f v2, float t)
{ return float_eq(v1.x,v2.x,t) && float_eq(v1.y,v2.y,t) && float_eq(v1.z,v2.z,t); }
bool quatf_eq(quatf q1, quatf q2, float t)
{
    if (float_eq(q1.x,q2.x,t) && float_eq(q1.y,q2.y,t) && float_eq(q1.z,q2.z,t) && float_eq(q1.w,q2.w,t))
        return true;   /* q and -q are the same rotation */
    return float_eq(q1.x,-q2.x,t) && float_eq(q1.y,-q2.y,t) && float_eq(q1.z,-q2.z,t) && float_eq(q1.w,-q2.w,t);
}

#define Test(_t) printf("   "#_t); _t(); printf("%*sok\n", 52 - (int)strlen(#_t), "");
int main(void)
{
    printf("joint pose tests\n");
    Test(test_rift_joint_pose_exact);
    Test(test_rift_joint_pose_two_cameras_beat_one);
    Test(test_rift_joint_pose_detects_bad_extrinsics);
    Test(test_rift_joint_pose_rejects_thin_data);
    printf("\nall a-ok\n");
    return 0;
}
EOF

gcc -O1 -g -Wall -o "$OUT/jp_test" \
    -I"$OHMD/include" -I"$OHMD/src" -I"$OHMD/tests/unittests" \
    "$OUT/main.c" \
    "$OHMD/tests/unittests/joint_pose.c" \
    "$OHMD/src/drv_oculus_rift/rift-joint-pose.c" \
    "$OHMD/src/omath.c" -lm

"$OUT/jp_test"
