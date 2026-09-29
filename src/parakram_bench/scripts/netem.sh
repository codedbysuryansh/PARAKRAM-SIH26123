#!/usr/bin/env bash
# Network-level packet loss (CLAUDE_CODE/05, the secondary loss mechanism): tc netem.
#
#   LOSS=30 [CORR=25] [IFACE=wlan0] netem.sh add    # before the trial
#   [IFACE=wlan0] netem.sh del                       # after it
#   [IFACE=wlan0] netem.sh show
#
# Hardware: apply on every robot's wlan0 (only inter-robot traffic crosses it). Single-host sim:
# IFACE=lo hits ALL local traffic, and Fast DDS shared memory bypasses it altogether, so the sim
# sweep uses the app-level seeded drop (loss:=) and netem_verify.sh checks netem on its own probe.
# CORR is netem's loss correlation (the work order's default 25 %; 0 = Bernoulli).
# Every add / del is appended to <log_root>/netem_events.csv.
IFACE=${IFACE:-wlan0}
HERE=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
LOG_ROOT=${PARAKRAM_LOG_ROOT:-$(cd "$HERE/../../.." && pwd)/bench/logs}
log_event() {
  mkdir -p "$LOG_ROOT"
  [ -f "$LOG_ROOT/netem_events.csv" ] || echo "utc,host,action,iface,loss_pct,corr_pct" > "$LOG_ROOT/netem_events.csv"
  echo "$(date -u +%Y-%m-%dT%H:%M:%S.%3NZ),$(hostname),$1,$IFACE,${2:-},${3:-}" >> "$LOG_ROOT/netem_events.csv"
}
case "${1:-}" in
  add)
    : "${LOSS:?set LOSS=<percent>}"
    sudo tc qdisc add dev "$IFACE" root netem loss "${LOSS}%" "${CORR:-25}%"
    log_event add "$LOSS" "${CORR:-25}"
    ;;
  del)
    sudo tc qdisc del dev "$IFACE" root
    log_event del
    ;;
  show)
    tc qdisc show dev "$IFACE"
    ;;
  *)
    echo "usage: LOSS=<pct> [CORR=25] [IFACE=wlan0] $0 add|del|show" >&2
    exit 2
    ;;
esac
