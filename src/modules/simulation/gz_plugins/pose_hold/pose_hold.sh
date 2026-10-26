#!/usr/bin/env bash
# Send a command to the PoseHoldSystem gz plugin, see README.md.
set -eu

if [ $# -eq 0 ] || [[ "$*" =~ [^a-zA-Z0-9_.\ -] ]]; then
	echo "usage: $(basename "$0") [model] level|nose_down|tail_down|left_down|right_down|inverted|rotate|release" >&2
	exit 1
fi

exec gz topic -t /pose_hold -m gz.msgs.StringMsg -p "data: \"$*\""
