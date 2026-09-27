#!/usr/bin/env bash
# Native Ubuntu desktop entrypoint. No separate G10 UDP socket.
set -euo pipefail
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$(cd -- "$REPO/.." && pwd)"

if [[ "${EUID}" -eq 0 ]]; then
  echo "Run the dashboard as the normal Ubuntu user, never root." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
export G10_LOG_DIR="${G10_LOG_DIR:-$WORKSPACE/measurements}"
cd "$REPO"
exec python3 -m bidirectional_motor_test.g10_gui
