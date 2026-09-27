#!/usr/bin/env bash
# The Tk viewer starts without ROS; only ROS commands use this helper.
set -eo pipefail
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${G10_WORKSPACE:-$(cd -- "$REPO/.." && pwd)}"
if [[ "$#" -eq 0 ]]; then
  echo "Usage: $0 ros2 <arguments>" >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS Humble not found: /opt/ros/humble/setup.bash" >&2
  exit 2
fi
if [[ ! -f "$WORKSPACE/install/setup.bash" ]]; then
  echo "ROS workspace has not been built: $WORKSPACE/install/setup.bash" >&2
  exit 2
fi
# Do not use set -u: vendor/colcon environment hooks can reference
# optional shell variables.
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
exec "$@"
