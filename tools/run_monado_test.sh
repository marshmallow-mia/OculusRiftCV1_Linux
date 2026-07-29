#!/bin/sh
# Run the CV1 through Monado, on Wayland, using THIS project's OpenHMD.
#
# This is the Wayland-native path. Monado leases the headset display through
# wp_drm_lease_device_v1, which KWin advertises, so no X11 session is needed --
# unlike SteamVR, whose display acquisition hard-locked this machine on
# 2026-07-29 (see run_steamvr_test.sh for that write-up).
#
#   tools/run_monado_test.sh [tag]
#
# Writes, under captures/lin/<today>/:
#   <tag>.log     Monado's output, including the OpenHMD driver's own logging -
#                 calibration events, joint reconstruction, sync warnings
#   <tag>.jsonl   an OHMD_RIFT_CAL_CAPTURE observation log, for replay_recon.py
#
# THE LIBRARY PIN MATTERS. Arch ships openhmd 0.3.0 at /usr/lib/libopenhmd.so.0
# and it is ~92 KB of stock upstream with none of this project's tracking work.
# Meson strips RPATH on install, so an installed binary silently resolves to
# that one - it is how a "positional tracking: no" reading was got from a build
# that does report positional tracking. LD_LIBRARY_PATH below pins ours, and
# the check after it fails loudly rather than quietly measuring the wrong code.
set -e

here=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
tag=${1:-monado-session}
out="$here/captures/lin/$(date +%Y-%m-%d)"
mkdir -p "$out"

ohmd_prefix=$HOME/.local/openhmd-windows-parity
monado_build=$HOME/git/monado/build
service=$monado_build/src/xrt/targets/service/monado-service

[ -x "$service" ] || {
	echo "monado-service not built at $service" >&2
	echo "  cmake -GNinja -B build -DXRT_BUILD_DRIVER_OHMD=ON \\" >&2
	echo "        -DXRT_HAVE_SYSTEM_CJSON=OFF ... with" >&2
	echo "        PKG_CONFIG_PATH=$ohmd_prefix/lib/pkgconfig" >&2
	exit 1
}
[ -e "$ohmd_prefix/lib/libopenhmd.so.0" ] || {
	echo "this project's OpenHMD is not installed at $ohmd_prefix" >&2
	echo "  meson setup <build> subprojects/openhmd --prefix=$ohmd_prefix \\" >&2
	echo "        --default-library=shared && ninja -C <build> install" >&2
	exit 1
}

resolved=$(LD_LIBRARY_PATH="$ohmd_prefix/lib" ldd "$service" 2>/dev/null |
	awk '/libopenhmd/ {print $3}')
case $resolved in
"$ohmd_prefix"/*) ;;
*)
	echo "REFUSING TO RUN: monado-service resolves libopenhmd to" >&2
	echo "  ${resolved:-<not found>}" >&2
	echo "rather than this project's build under $ohmd_prefix." >&2
	echo "Anything measured that way is the stock upstream driver." >&2
	exit 1
	;;
esac

echo "openhmd: $resolved"
echo "driver:  $(cd "$here/../SteamVR-OpenHMD/subprojects/openhmd" && git log --oneline -1)"
echo "monado:  $(cd "$HOME/git/monado" && git log --oneline -1)"
echo "log:     $out/$tag.log"
echo "capture: $out/$tag.jsonl"
echo
echo "Point OpenXR apps at this runtime with:"
echo "  XR_RUNTIME_JSON=$monado_build/openxr_monado-dev.json"
echo

# monado-service adds stdin to its epoll set and refuses to start if that
# fails, so it needs a tty or a pipe - not a closed fd or a redirected file.
# Interactively it inherits the terminal (and Enter still quits it); run
# non-interactively it gets a pipe that never reaches EOF, which would
# otherwise read as "quit".
if [ -t 0 ]; then
	LD_LIBRARY_PATH="$ohmd_prefix/lib" \
	OHMD_RIFT_CAL_CAPTURE="$out/$tag.jsonl" \
		exec "$service" > "$out/$tag.log" 2>&1
else
	sleep 2147483647 | {
		LD_LIBRARY_PATH="$ohmd_prefix/lib" \
		OHMD_RIFT_CAL_CAPTURE="$out/$tag.jsonl" \
			exec "$service" > "$out/$tag.log" 2>&1
	}
fi
