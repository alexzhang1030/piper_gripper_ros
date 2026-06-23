#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  can_activate.sh [--iface can0] [--bitrate 1000000] [--restart-ms 100]
                  [--fd] [--dbitrate 5000000]

Examples:
  ./scripts/can_activate.sh --iface can0 --bitrate 1000000
  ./scripts/can_activate.sh --iface can0 --fd --bitrate 1000000 --dbitrate 5000000
EOF
}

iface="can0"
bitrate="1000000"
dbitrate="5000000"
restart_ms="100"
fd_mode="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --iface) iface="$2"; shift 2 ;;
    --bitrate) bitrate="$2"; shift 2 ;;
    --dbitrate) dbitrate="$2"; shift 2 ;;
    --restart-ms) restart_ms="$2"; shift 2 ;;
    --fd) fd_mode="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1"; usage; exit 1 ;;
  esac
done

if ! command -v ip >/dev/null 2>&1; then
  echo "ip command not found. Please install iproute2." >&2
  exit 1
fi

if [[ $EUID -eq 0 ]]; then
  SUDO=""
elif command -v sudo >/dev/null 2>&1; then
  SUDO="sudo"
else
  echo "Need root privileges (run as root or install sudo)." >&2
  exit 1
fi

set -x
$SUDO ip link set "$iface" down || true
if [[ "$fd_mode" == "true" ]]; then
  $SUDO ip link set "$iface" type can bitrate "$bitrate" dbitrate "$dbitrate" fd on restart-ms "$restart_ms"
else
  $SUDO ip link set "$iface" type can bitrate "$bitrate" restart-ms "$restart_ms"
fi
$SUDO ip link set "$iface" up
set +x

echo
echo "Interface status:"
ip -details link show "$iface"
