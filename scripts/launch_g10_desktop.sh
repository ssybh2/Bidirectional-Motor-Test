#!/usr/bin/env bash
# Ubuntu desktop entrypoint. Opening the viewer does NOT require sourcing ROS.
# ROS is sourced only when the user starts acquisition/calls a ROS service.
set -eo pipefail
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$(cd -- "$REPO/.." && pwd)"
STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/bidirectional-g10"
mkdir -p "$STATE_DIR"
LOG="$STATE_DIR/dashboard-launch.log"

echo "G10 desktop launcher: $LOG"
{
  echo
  echo "=== $(date -Is) desktop launch ==="
  echo "repo=$REPO workspace=$WORKSPACE"
  echo "DISPLAY=${DISPLAY:-unset} WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-unset}"
} >> "$LOG"

show_error() {
  local detail="$1"
  echo "G10 dashboard error: $detail; see $LOG" >&2
  printf '%s\n' "ERROR: $detail" >> "$LOG"
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="G10 推力测量启动失败" \
      --text="无法打开 G10 图形界面。\n$detail\n\n日志：$LOG" \
      --width=540 >/dev/null 2>&1 || true
  elif command -v notify-send >/dev/null 2>&1; then
    notify-send "G10 推力测量启动失败" \
      "$detail。日志：$LOG" >/dev/null 2>&1 || true
  fi
}
if [[ "${EUID}" -eq 0 ]]; then
  show_error "请使用普通 Ubuntu 用户启动，不能使用 sudo/root"
  exit 1
fi
if [[ ! -d "$REPO/bidirectional_motor_test" ]]; then
  show_error "仓库位置不存在：$REPO"
  exit 1
fi
export G10_REPO="$REPO"
export G10_WORKSPACE="$WORKSPACE"
export G10_LOG_DIR="${G10_LOG_DIR:-$WORKSPACE/measurements}"
export G10_ROS_HELPER="$REPO/scripts/ros_env_exec.sh"
cd "$REPO"
if ! /usr/bin/python3 -c 'import tkinter; import bidirectional_motor_test.g10_gui' \
     >> "$LOG" 2>&1; then
  show_error "Python/Tk 组件无法导入；请检查 python3-tk 与最新 Git 代码"
  exit 1
fi
if /usr/bin/python3 -m bidirectional_motor_test.g10_gui >> "$LOG" 2>&1; then
  echo "GUI exited normally" >> "$LOG"
else
  result=$?
  show_error "窗口进程退出（状态码 $result），请查看日志中的 Python 错误"
  exit "$result"
fi
