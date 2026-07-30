#!/bin/sh
# What IPD is the headset actually rendering at?
#
# The CV1 has no software readout for its hardware IPD slider, and OpenHMD only
# reads the slider position when it OPENS the device - so moving the slider
# during a session changes nothing until SteamVR is restarted. This reports
# what the running (or last) session picked up.
#
#   tools/show_ipd.sh
set -e

log=$HOME/.local/share/Steam/logs/vrserver.txt
line=$(grep -ah "driver_openhmd: IPD" "$log" 2>/dev/null | tail -1)

if [ -z "$line" ]; then
	echo "No IPD line in $log - has SteamVR run with this driver yet?" >&2
	exit 1
fi

ipd=$(printf '%s\n' "$line" | sed 's/.*IPD: *//')
when=$(printf '%s\n' "$line" | sed 's/ \[Info\].*//')

printf 'IPD in use: %.1f mm   (read %s)\n' \
	"$(printf '%s\n' "$ipd" | awk '{print $1*1000}')" "$when"

if pgrep -x vrserver >/dev/null 2>&1; then
	echo
	echo "SteamVR is running. If you moved the slider since $when, this is the"
	echo "OLD value - the slider is only read when the device is opened."
	echo "Restart SteamVR and run this again to see the new position."
fi
