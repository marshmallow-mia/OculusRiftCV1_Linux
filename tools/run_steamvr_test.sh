#!/bin/sh
# Launch SteamVR with the OpenHMD driver's own logging captured.
#
# !! This hard-locked the machine once, on 2026-07-29. The preflight below !!
# !! exists so it cannot happen the same way again. Read this first.       !!
#
# What happened: starting SteamVR from a KDE *Wayland* session produced
#
#   amdgpu 0000:0b:00.0: [drm] *ERROR* dc_stream_state is NULL for crtc '2'!
#
# seven times - dm_set_vblank() in amdgpu_dm_crtc.c, i.e. vblank enabled on a
# CRTC with no stream attached - then seven seconds of display reconfiguration
# and a hard lockup with no oops, the journal simply stopping.
#
# The cause is the session, not the hardware and not the tracking driver:
#
#   * The tracking driver is USB-only (libusb/hidapi) and never touches
#     DRM/KMS. It cannot produce this.
#   * The headset's video is healthy. HDMI-A-1 presents EDID "Rift", serial
#     WMHD316C1006S, one mode 2160x1200@90, and the kernel correctly marks it
#     non-desktop=1 so the compositor leaves it alone.
#   * SteamVR needs X11. It acquires a non-desktop display through DRM
#     leasing, which it does not support under Wayland, and the failed
#     acquisition is what walks over the display state.
#
# So: log into "Plasma (X11)" at SDDM before running this.
#
# One more thing worth knowing, because it makes a healthy headset look
# unplugged: the CV1 panel sleeps until something opens the HMD over USB.
# Until then HDMI-A-1 reads "disconnected" with no EDID, and it goes back to
# "disconnected" a few seconds after the driver exits. Checking the connector
# with nothing running tells you nothing about the cable. The preflight below
# therefore wakes the headset itself before deciding.
#
# For anything that only needs TRACKING - calibration behaviour, viewpoint
# accumulation, joint reconstruction, pose quality - use
# openhmd_simple_example instead. It exercises the same driver with no display
# involvement at all, and every measurement in windows-vs-linux-tracking.md was
# taken that way without incident. Only reach for this script when you
# specifically need the in-headset view.
#
# SteamVR does not capture a driver's stderr into vrserver.txt, so the
# calibration and joint-reconstruction lines this project cares about are
# invisible unless SteamVR is started from a shell with stderr redirected.
# That is the other reason this script exists.
#
#   tools/run_steamvr_test.sh [tag]
#
# Writes, under captures/lin/<today>/:
#   <tag>.log     the driver's own stderr - calibration events, joint solves
#   <tag>.jsonl   an OHMD_RIFT_CAL_CAPTURE observation log, for replay_recon.py
#
# Stop SteamVR normally when done; the capture closes with it.
set -e

here=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
tag=${1:-worn-session}
out="$here/captures/lin/$(date +%Y-%m-%d)"
mkdir -p "$out"

steamvr=$HOME/.local/share/Steam/steamapps/common/SteamVR/bin/vrmonitor.sh
[ -x "$steamvr" ] || { echo "SteamVR not found at $steamvr" >&2; exit 1; }

if pgrep -x vrserver >/dev/null 2>&1; then
	echo "vrserver is already running - stop SteamVR first" >&2
	exit 1
fi

# ---- preflight 1: session must be X11 -------------------------------------
session=${XDG_SESSION_TYPE:-}
[ -n "$session" ] || session=$(loginctl show-session "$(loginctl show-user "$USER" -p Display --value 2>/dev/null)" -p Type --value 2>/dev/null || true)

if [ "$session" = wayland ]; then
	echo >&2
	echo "REFUSING TO START: this is a Wayland session." >&2
	echo >&2
	echo "  SteamVR acquires the headset display through DRM leasing, which it" >&2
	echo "  does not support under Wayland. Starting it here is what hard-locked" >&2
	echo "  this machine on 2026-07-29 (dc_stream_state NULL for crtc, then the" >&2
	echo "  journal stops)." >&2
	echo >&2
	echo "  Log out and pick \"Plasma (X11)\" at the SDDM session menu, then" >&2
	echo "  re-run this. Everything else about the setup is fine." >&2
	exit 1
fi

# ---- preflight 2: the headset's video path actually comes up --------------
# The panel sleeps until the HMD is opened, so wake it and watch for the
# connector rather than trusting its idle state.
sample=$here/../SteamVR-OpenHMD/build/subprojects/openhmd/openhmd_simple_example
rift_display=

if [ -x "$sample" ]; then
	echo "waking the headset to check its video path..."
	"$sample" >/dev/null 2>&1 &
	waker=$!
	i=0
	while [ $i -lt 12 ]; do
		for c in /sys/class/drm/card*-*/; do
			[ "$(cat "$c/status" 2>/dev/null)" = connected ] || continue
			if cat "$c/edid" 2>/dev/null | strings 2>/dev/null |
					grep -qiE 'rift|oculus'; then
				rift_display=$(basename "$c")
				break
			fi
		done
		[ -n "$rift_display" ] && break
		i=$((i + 1))
		sleep 1
	done
	kill "$waker" 2>/dev/null || true
	wait "$waker" 2>/dev/null || true
else
	echo "note: $sample not built - skipping the video-path check" >&2
fi

if [ -x "$sample" ] && [ -z "$rift_display" ]; then
	echo >&2
	echo "REFUSING TO START: the headset never presented a display." >&2
	echo >&2
	echo "  It was woken over USB and no connector reported Rift EDID within" >&2
	echo "  12 s, so this is the video path - check the HDMI end of the" >&2
	echo "  headset cable. Connectors seen:" >&2
	echo >&2
	for c in /sys/class/drm/card*-*/; do
		printf "    %-22s %s\n" "$(basename "$c")" \
			"$(cat "$c/status" 2>/dev/null)" >&2
	done
	exit 1
fi

[ -n "$rift_display" ] && echo "display: $rift_display presents Rift EDID - OK"
echo "session: $session - OK"
echo "driver:  $(cd "$here/../SteamVR-OpenHMD/subprojects/openhmd" && git log --oneline -1)"
echo "log:     $out/$tag.log"
echo "capture: $out/$tag.jsonl"

OHMD_RIFT_CAL_CAPTURE="$out/$tag.jsonl" \
	exec "$steamvr" > "$out/$tag.log" 2>&1
