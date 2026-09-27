#!/usr/bin/env bash
# Registers a desktop application; user runs this ONCE, not on every launch.
set -euo pipefail
if [[ "${EUID}" -eq 0 ]]; then
  echo "Do not run the desktop installer with sudo." >&2
  exit 1
fi
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
LAUNCH="$REPO/scripts/launch_g10_desktop.sh"
WORKSPACE="$(cd -- "$REPO/.." && pwd)"
if [[ ! -f "$WORKSPACE/install/setup.bash" ]]; then
  echo "Build the ROS workspace first: colcon build --symlink-install --packages-up-to bidirectional_motor_test" >&2
  exit 1
fi
if ! python3 -c "import tkinter" >/dev/null 2>&1; then
  echo "Tkinter is missing. Install it: sudo apt install python3-tk" >&2
  exit 1
fi
chmod u+x "$LAUNCH"
mkdir -p "$HOME/.local/share/applications"
DEST="$HOME/.local/share/applications/bidirectional-g10.desktop"
cat > "$DEST" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=G10 推力测量
Comment=G10 UDP & ROS 2 live measurements
Exec=$LAUNCH
Path=$REPO
Icon=utilities-system-monitor
Terminal=false
Categories=Science;Education;
StartupNotify=true
EOF
chmod u+x "$DEST"
if [[ -d "$HOME/Desktop" ]]; then
  cp "$DEST" "$HOME/Desktop/G10推力测量.desktop"
  chmod u+x "$HOME/Desktop/G10推力测量.desktop"
  gio set "$HOME/Desktop/G10推力测量.desktop" metadata::trusted true \
    >/dev/null 2>&1 || true
fi
echo "Installed: $DEST"
echo "Open Applications and search for G10 推力测量."
echo "On some Ubuntu desktops, right-click the desktop icon and Allow Launching."
