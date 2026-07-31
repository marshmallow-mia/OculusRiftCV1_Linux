#!/bin/sh
# Build a point of the OpenHMD history and hand it to Monado, verifiably.
#
#   tools/bisect_openhmd.sh build <label> <commit>   build and stash a point
#   tools/bisect_openhmd.sh use   <label>            make that point live
#   tools/bisect_openhmd.sh list                     what is built, what is live
#
# Why this exists rather than "just rebuild":
#
#   - Monado loads libopenhmd.so from a prefix, so a point is swapped by
#     copying one file. SteamVR statically links OpenHMD into
#     driver_openhmd.so and would need a full driver rebuild per point, so the
#     bisect runs under Monado.
#   - Upstream sets only ROTATIONAL_TRACKING. Monado defaults every device to
#     3dof without the POSITIONAL flag (oh_device.c:1263), so an unpatched
#     upstream build would silently be a 3dof headset - which is exactly the
#     false "feels perfect" that cost an evening on 2026-07-31. Commit 5391124
#     is therefore applied to EVERY point, making it a constant rather than a
#     variable.
#   - OpenCV 5.0.0 is installed and upstream does not build against it, so the
#     build-only half of 90390c6 is applied to every point too, for the same
#     reason.
#   - Every operation prints a content fingerprint. Two experiments that night
#     were void because the binary under test was assumed from a timestamp
#     rather than checked. Never trust mtime here.
set -e

here=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
ohmd=$here/../SteamVR-OpenHMD/subprojects/openhmd
prefix=$HOME/.local/openhmd-windows-parity/lib
store=$prefix/bisect
scratch=${TMPDIR:-/tmp}/ohmd-bisect-work
wt=$scratch/tree

fingerprint() {
	# Markers chosen to separate the eras cheaply. Absent/present is enough;
	# this is an identity check, not a feature list.
	for m in "output correction bleeding" "OHMD_RIFT_VEL_ADAPTIVE" \
	         "rift-tracker-config" "OHMD_RIFT_NO_JOINT_SOLVE" \
	         "OHMD_RIFT_TOUCH_IMU_LATENCY_MS"; do
		if strings "$1" 2>/dev/null | grep -qF "$m"; then
			printf '  %-34s present\n' "$m"
		else
			printf '  %-34s absent\n' "$m"
		fi
	done
	printf '  %-34s %s\n' "sha256" "$(sha256sum "$1" | cut -c1-16)"
}

case ${1:-} in
build)
	label=${2:?need a label}; commit=${3:?need a commit}
	mkdir -p "$store"
	if [ ! -d "$wt" ]; then
		git -C "$ohmd" worktree add --detach "$wt" "$commit"
	else
		git -C "$wt" checkout --detach -f "$commit"
		git -C "$wt" clean -fdq
	fi

	# OpenCV 5: probe it before opencv4, and use the flat headers that all of
	# 3/4/5 provide. Build-only, no behaviour.
	python3 - "$wt" <<-'PY'
	import sys, pathlib
	wt = pathlib.Path(sys.argv[1])
	p = wt / "meson.build"; s = p.read_text()
	old = "dep_opencv = dependency('opencv4', required: false)"
	new = ("dep_opencv = dependency('opencv5', required: false)\n"
	       "if not dep_opencv.found()\n"
	       "\tdep_opencv = dependency('opencv4', required: false)\n"
	       "endif")
	if old in s and "opencv5" not in s:
	    p.write_text(s.replace(old, new, 1))
	p = wt / "src/drv_oculus_rift/rift-sensor-opencv.cpp"; s = p.read_text()
	oldinc = ("#include <opencv2/calib3d/calib3d.hpp>\n"
	          "#include <opencv2/imgproc/imgproc.hpp>\n"
	          "#if CV_MAJOR_VERSION >= 4\n"
	          "#include <opencv2/calib3d/calib3d_c.h>\n"
	          "#endif\n")
	if oldinc in s:
	    s = s.replace(oldinc, "#include <opencv2/calib3d.hpp>\n"
	                          "#include <opencv2/imgproc.hpp>\n", 1)
	oldlm = ("\tif (!cv::solvePnPRefineLM (list_points3d, list_points2d_undistorted,"
	         " dummyK, dummyD, rvec, tvec))\n\t\treturn false;\n")
	if oldlm in s:
	    s = s.replace(oldlm, "\tcv::solvePnPRefineLM (list_points3d,"
	                         " list_points2d_undistorted, dummyK, dummyD,"
	                         " rvec, tvec);\n", 1)
	p.write_text(s)
	PY

	# The 3dof trap: force the positional capability flag at every point.
	if ! grep -q 'OHMD_DEVICE_FLAGS_POSITIONAL_TRACKING' \
	     "$wt/src/drv_oculus_rift/rift.c" ||
	   ! git -C "$wt" diff --quiet 2>/dev/null; then
		git -C "$ohmd" show 5391124 -- src/drv_oculus_rift/rift.c \
			> "$scratch/flag.patch" 2>/dev/null || true
		git -C "$wt" apply "$scratch/flag.patch" 2>/dev/null || true
	fi
	grep -q 'OHMD_DEVICE_FLAGS_POSITIONAL_TRACKING' \
		"$wt/src/drv_oculus_rift/rift.c" || {
		echo "REFUSING: positional flag missing - Monado would run 3dof" >&2
		exit 1
	}

	rm -rf "$scratch/build"
	meson setup "$scratch/build" "$wt" --default-library=shared \
		-Dtests=false >"$scratch/setup.log" 2>&1 || {
		tail -20 "$scratch/setup.log" >&2; exit 1; }
	ninja -C "$scratch/build" libopenhmd.so.0.1.0 >"$scratch/build.log" 2>&1 || {
		tail -25 "$scratch/build.log" >&2; exit 1; }

	cp -f "$scratch/build/libopenhmd.so.0.1.0" "$store/libopenhmd-$label.so"
	echo "built $label ($(git -C "$wt" log --oneline -1))"
	fingerprint "$store/libopenhmd-$label.so"
	;;

use)
	label=${2:?need a label}
	src=$store/libopenhmd-$label.so
	[ -f "$src" ] || { echo "no such build: $label" >&2; exit 1; }
	if pgrep -x monado-service >/dev/null 2>&1; then
		echo "monado-service is running - stop it first, it holds the old .so" >&2
		exit 1
	fi
	cp -f "$src" "$prefix/libopenhmd.so.0.1.0"
	echo "$label" > "$store/CURRENT"
	echo "live: $label"
	fingerprint "$prefix/libopenhmd.so.0.1.0"
	;;

list)
	echo "built points:"
	ls "$store"/libopenhmd-*.so 2>/dev/null |
		sed 's|.*/libopenhmd-||; s|\.so$||' | sed 's/^/  /' || echo "  (none)"
	echo "live now: $(cat "$store/CURRENT" 2>/dev/null || echo unknown)"
	fingerprint "$prefix/libopenhmd.so.0.1.0"
	;;
*)
	sed -n '2,12p' "$0"
	exit 2
	;;
esac
