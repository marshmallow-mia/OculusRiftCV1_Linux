#!/bin/sh
# Launch SteamVR with the OpenHMD driver's own logging captured.
#
# !! This hard-locked the machine once, on 2026-07-29. Read this first. !!
#
# What happened: starting SteamVR produced
#
#   amdgpu 0000:0b:00.0: [drm] *ERROR* dc_stream_state is NULL for crtc '2'!
#
# seven times - dm_set_vblank() in amdgpu_dm_crtc.c, i.e. vblank enabled on a
# CRTC with no stream attached - then seven seconds of display reconfiguration
# and a hard lockup with no oops, the journal simply stopping.
#
# What that was NOT:
#
#   * Not the tracking driver. It is USB-only (libusb/hidapi) and never
#     touches DRM/KMS.
#   * Not a missing cable. The headset's video is healthy: HDMI-A-1 presents
#     EDID "Rift", serial WMHD316C1006S, one mode 2160x1200@90, non-desktop=1.
#     The CV1 panel just sleeps until something opens the HMD over USB, and
#     drops again within a second of release - so checking the connector with
#     nothing running tells you nothing, and there is no window in which to
#     "pre-wake" it for another process.
#   * Not Wayland as such. Monado drives this same headset on this same
#     Wayland session through wp_drm_lease_device_v1 with zero amdgpu errors
#     (tools/run_monado_test.sh).
#
# The likely cause, and what changed since: driver_openhmd.cpp claimed
# IsDisplayOnDesktop() == true while setting Prop_IsOnDesktop_Bool false, and
# reported window bounds from a hardcoded "m_nWindowX = 1920; //TODO". The CV1
# is non-desktop, so that pointed SteamVR at a desktop rectangle belonging to a
# real monitor while the headset's own connector sat outside it. It now returns
# false, which puts SteamVR in direct mode - the same acquisition Monado does
# successfully here. OHMD_STEAMVR_EXTENDED=1 restores the old behaviour.
#
# That is a reasoned fix, not a verified one. It may still lock up. Save your
# work before running this, and prefer tools/run_monado_test.sh when you only
# need tracking or do not specifically need SteamVR.
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

# ---- preflight 1: report the session, do not block on it ------------------
session=${XDG_SESSION_TYPE:-}
[ -n "$session" ] || session=$(loginctl show-session "$(loginctl show-user "$USER" -p Display --value 2>/dev/null)" -p Type --value 2>/dev/null || true)

if [ "$session" = wayland ]; then
	echo "session: wayland - SteamVR's display acquisition is least tested here."
	echo "         Monado's Wayland path works if this misbehaves:"
	echo "         tools/run_monado_test.sh"
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
