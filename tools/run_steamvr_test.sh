#!/bin/sh
# Launch SteamVR with the OpenHMD driver's own logging captured.
#
# !! WARNING - this hard-locked this machine on 2026-07-29. !!
#
# Starting SteamVR made the kernel's AMD display stack fault immediately:
#
#   amdgpu 0000:0b:00.0: [drm] *ERROR* dc_stream_state is NULL for crtc '2'!
#
# repeating, then a DRM hotplug on card1-DP-3 (an ordinary monitor - the Rift
# display was not even attached), then the journal stops dead with no oops:
# a hard lockup. Nothing to do with the tracking driver, which is USB-only
# (libusb/hidapi) and never touches DRM/KMS - it is SteamVR reconfiguring
# displays that amdgpu cannot survive here.
#
# For anything that only needs TRACKING - calibration behaviour, viewpoint
# accumulation, joint reconstruction, pose quality - use
# examples/simple/openhmd_simple_example instead. It exercises the same driver
# with no display involvement at all, and every measurement in
# windows-vs-linux-tracking.md was taken that way without incident. Only reach
# for this script when you specifically need the in-headset view.
#
# SteamVR does not capture a driver's stderr into vrserver.txt, so the
# calibration and joint-reconstruction lines this project cares about are
# invisible unless SteamVR is started from a shell with stderr redirected.
# That is the whole reason this script exists.
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

echo "driver:  $(cd "$here/../SteamVR-OpenHMD/subprojects/openhmd" && git log --oneline -1)"
echo "log:     $out/$tag.log"
echo "capture: $out/$tag.jsonl"

OHMD_RIFT_CAL_CAPTURE="$out/$tag.jsonl" \
	exec "$steamvr" > "$out/$tag.log" 2>&1
