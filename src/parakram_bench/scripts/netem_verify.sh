#!/usr/bin/env bash
# CLAUDE_CODE/05 netem verification on this host (the secondary loss mechanism). Run it with sudo
# from your own account, with no simulation running:
#
#   sudo bash src/parakram_bench/scripts/netem_verify.sh [--cases 0:0,10:0,30:0,60:0,30:25]
#
# For ~11 min (every installed fabric: Fast DDS, CycloneDDS, rmw_zenoh) it applies tc netem to the
# loopback interface (ALL local traffic on this machine sees the loss while a case runs); the
# qdisc is removed after every case and on any exit. The ROS probe processes run as you
# (SUDO_USER), not as root. See parakram_bench/netem_check.py.
HERE=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
WS=$(cd "$HERE/../../.." && pwd)
if [ "$(id -u)" -ne 0 ] || [ -z "${SUDO_USER:-}" ]; then
  echo "run with sudo from your user account: sudo bash $0" >&2
  exit 1
fi
trap 'tc qdisc del dev lo root 2>/dev/null || true' EXIT INT TERM
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
cd "$WS"
python3 -m parakram_bench.netem_check --ws "$WS" --results-dir "$WS/src/parakram_bench/results" "$@"
