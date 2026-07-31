#!/bin/sh
# Regression test for the vision-dropout tail.
#
# The reported symptom was "fast movement and the image still takes a bit to
# stop moving". Cause: vision drops out under fast motion (motion blur costs
# blobs, the pose search needs ten matched LEDs), the fusion stopped integrating
# position and zeroed its velocity, and the error that accumulated while it was
# frozen was then fed to GAIN_VEL and GAIN_ACCEL as if it were evidence of
# motion - so the estimate overshot the truth and rang for ~2 s, exporting a
# phantom velocity that SteamVR multiplied by its prediction horizon.
#
# This builds tools/fusion_replay.c against the REAL rift-fusion-ovr.c and
# exponential-filter.c and measures how far the pose travels AFTER the head has
# stopped, across the dropout band that actually occurs on this hardware
# (measured in-headset: 0.19 s median, 0.87 s max).
#
#   tools/run_dropout_check.sh
#
# Fails if post-stop travel exceeds the bar anywhere in 0-500 ms.
set -e

here=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
oh=$here/../SteamVR-OpenHMD/subprojects/openhmd
src=$oh/src
out=${TMPDIR:-/tmp}/rift-dropout-check
mkdir -p "$out"

# Bar: 8 mm of post-stop DRIFT. The no-dropout floor is ~3 mm (the estimator
# legitimately finishing its correction) and the defect produced 59-424 mm, so
# this sits well clear of both.
#
# Drift, specifically, not total travel. Past the coast window a returning fix
# is applied as a single-frame snap (VISION_REACQUIRE_NS), which is pre-existing
# behaviour for a genuine tracking loss and reads as a pop rather than as the
# reported "takes a bit to stop moving". Decomposed at 1.2 m/s: a 700 ms outage
# is 154.57 mm total but 42.34 mm of that is one step and only 3.32 mm is drift.
# Gating on total would therefore fail on an artifact this fix is not about, and
# would hide a real drift regression behind a large snap. The snap size is
# reported alongside so it cannot regress unnoticed.
BAR_MM=8.0

cc -O2 -o "$out/fusion_replay" "$here/tools/fusion_replay.c" \
   "$src/drv_oculus_rift/rift-fusion-ovr.c" \
   "$src/exponential-filter.c" "$src/omath.c" \
   -I"$src/drv_oculus_rift" -I"$src" -I"$oh/include" -lm

fail=0
echo "post-stop motion, translate profile at 1.2 m/s (drift bar ${BAR_MM} mm)"
printf "  %-10s %10s %10s %10s\n" "dropout" "drift" "snap" "total"
for d in 0 70 120 200 300 500 700; do
	"$out/fusion_replay" --profile translate --dropout-ms "$d" \
		--csv "$out/d.csv" >/dev/null
	set -- $(python3 - "$out/d.csv" <<-'EOF'
	import csv, sys, numpy as np
	r = list(csv.DictReader(open(sys.argv[1])))
	t = np.array([float(x["t"]) for x in r])
	x = np.array([float(x["est_x"]) for x in r])
	d = np.diff(x[np.searchsorted(t, 0.70):])
	a = abs(d)
	# a step over 1 mm at 500 Hz is a discontinuity, not motion
	print("%.2f %.2f %.2f" % ((a[a < 0.001].sum()) * 1000,
	                          (a.max() if len(a) else 0) * 1000,
	                          a.sum() * 1000))
	EOF
	)
	drift=$1; snap=$2; total=$3
	verdict=$(python3 -c "print('ok' if $drift <= $BAR_MM else 'FAIL')")
	printf "  %-10s %10s %10s %10s  %s\n" "${d} ms" "$drift" "$snap" "$total" "$verdict"
	[ "$verdict" = FAIL ] && fail=1
done

# The One Euro output filter smooths orientation as an exponential map, whose
# representation flips at half a turn. Ending a fast turn at exactly 180 deg
# used to produce errors up to 149.50 deg.
echo
echo "orientation error ending a 400 deg/s turn at 180 deg"
deg=$("$out/fusion_replay" --profile turn --noise --hold 0.30 |
	sed -n 's/.*worst orientation error EVER: *\([0-9.]*\).*/\1/p')
verdict=$(python3 -c "print('ok' if $deg <= 5.0 else 'FAIL')")
printf "  worst error      : %8s deg  %s\n" "$deg" "$verdict"
[ "$verdict" = FAIL ] && fail=1

echo
if [ "$fail" = 0 ]; then
	echo "all within bars"
else
	echo "REGRESSION" >&2
fi
exit "$fail"
