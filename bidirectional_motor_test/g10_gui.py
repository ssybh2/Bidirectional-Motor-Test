"""Native Ubuntu Tk dashboard for the existing ROS G10 acquisition process.

The GUI reads CSV snapshots. It NEVER opens the G10 UDP port, publishes
DSHOT, or replaces physical emergency-stop controls.
"""
from __future__ import annotations

from collections import deque
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import threading
import time

try:
    import tkinter as tk
    from tkinter import messagebox
    from tkinter import font as tkfont
except ImportError as exc:
    raise SystemExit(
        "Ubuntu desktop support is missing. Install: sudo apt install python3-tk"
    ) from exc

from .gui_data import (
    last_row, latest_session, parse_channels, parse_force, plot_limits,
    recent_rows, stream_fresh,
)


ROOT = "#101827"
SURFACE = "#192436"
PANEL = "#1c2a3c"
EDGE = "#314056"
INK = "#eff5fb"
MUTED = "#9cb0c8"
CYAN = "#53d6dd"
GREEN = "#6cdda4"
AMBER = "#efca76"
RED = "#fa8887"
FONT = "Noto Sans CJK SC"
ROS_NODE = "/bidirectional_motor_test"
LOG_DIR = Path(os.environ.get(
    "G10_LOG_DIR", "~/bidirectional/measurements")).expanduser()
REPO_DIR = Path(os.environ.get(
    "G10_REPO", Path(__file__).resolve().parents[1])).expanduser()
ROS_HELPER = Path(os.environ.get(
    "G10_ROS_HELPER", REPO_DIR / "scripts/ros_env_exec.sh")).expanduser()


def ros_command(*args):
    """Only ROS-dependent actions load ROS, never the GUI startup itself."""
    if ROS_HELPER.is_file():
        return [str(ROS_HELPER), *args]
    # This fallback supports 'ros2 run g10_dashboard' from a sourced
    # terminal when the source-repository scripts are unavailable.
    return list(args)


class Dashboard:
    def __init__(self, root):
        self.root = root
        root.title("G10 · Ubuntu 推力测量")
        root.configure(bg=ROOT)
        root.geometry("1300x820")
        root.minsize(1010, 660)
        self.prefix = None
        self.last_mono = -1
        self.last_unit = None
        self.history = deque(maxlen=580)
        self.own_proc = None
        self.own_log = None
        self.ros_job_busy = False
        self.closing = False
        self.stop_requested = False
        self.status_note = tk.StringVar(value="准备就绪 · 尚未连接采集")
        self.network_note = tk.StringVar(value="等待采集")
        self.mass_var = tk.StringVar(value="")
        self.main_value = tk.StringVar(value="—")
        self.raw_value = tk.StringVar(value="—")
        self.dshot_value = tk.StringVar(value="0")
        self.stream_value = tk.StringVar(value="等待数据")
        self.unit_value = tk.StringVar(value="raw_count")
        self.mode_value = tk.StringVar(value="—")
        self.age_value = tk.StringVar(value="—")
        self.channel_vars = [tk.StringVar(value="—") for _ in range(8)]
        self._build()
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        root.after(250, self._refresh)

    def _txt(self, parent, value, size=11, fg=INK, bold=False, **kwargs):
        return tk.Label(
            parent, text=value, fg=fg, bg=parent.cget("bg"),
            font=(FONT, size, "bold" if bold else "normal"),
            **kwargs)

    def _build(self):
        header = tk.Frame(self.root, bg=ROOT, padx=24, pady=18)
        header.pack(fill="x")
        left = tk.Frame(header, bg=ROOT)
        left.pack(side="left")
        self._txt(left, "G10   /   实时推力测量", 23, bold=True).pack(
            side="top", anchor="w")
        self._txt(left, "Ubuntu · ROS 2 · 独立 UDP 网口 enp5s0",
                  10, MUTED).pack(anchor="w", pady=(4, 0))
        self.connection = self._txt(
            header, "● 等待接收", 12, AMBER, bold=True)
        self.connection.pack(side="right", padx=4)

        metrics = tk.Frame(self.root, bg=ROOT, padx=20)
        metrics.pack(fill="x")
        for column in range(4):
            metrics.grid_columnconfigure(
                column, weight=1, uniform="metrics")
        cards = [
            ("推力 · 已去皮", self.main_value, self.unit_value),
            ("G10 原始 ADC", self.raw_value, "ADC count"),
            ("当前 DSHOT", self.dshot_value, "ROS 发布指令"),
            ("采集状态", self.stream_value, self.age_value),
        ]
        for col, (label, var, detail) in enumerate(cards):
            frame = tk.Frame(metrics, bg=SURFACE, padx=20, pady=15,
                             highlightbackground=EDGE, highlightthickness=1)
            frame.grid(row=0, column=col, sticky="ew", padx=5)
            self._txt(frame, label, 10, MUTED).pack(anchor="w")
            self._txt(frame, "", 24, CYAN if col == 0 else INK,
                      bold=True, textvariable=var).pack(
                anchor="w", pady=(8, 3))
            if isinstance(detail, tk.StringVar):
                self._txt(frame, "", 10, MUTED, textvariable=detail).pack(
                    anchor="w")
            else:
                self._txt(frame, detail, 10, MUTED).pack(anchor="w")

        work = tk.Frame(self.root, bg=ROOT, padx=24, pady=18)
        work.pack(fill="both", expand=True)
        work.grid_columnconfigure(0, weight=5)
        work.grid_columnconfigure(1, weight=2)
        work.grid_rowconfigure(0, weight=1)
        chart_panel = tk.Frame(
            work, bg=SURFACE, padx=16, pady=14,
            highlightbackground=EDGE, highlightthickness=1)
        chart_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        top = tk.Frame(chart_panel, bg=SURFACE)
        top.pack(fill="x", pady=(0, 12))
        self._txt(top, "推力时序曲线", 15, bold=True).pack(side="left")
        self._txt(top, "实时快照 · 最近约 30 秒", 10, MUTED).pack(
            side="right")
        self.graph = tk.Canvas(
            chart_panel, bg=SURFACE, highlightthickness=0, bd=0)
        self.graph.pack(fill="both", expand=True)
        bottom = tk.Frame(chart_panel, bg=SURFACE)
        bottom.pack(fill="x", pady=(9, 0))
        self._txt(bottom, "● 正向 +  /  反向 −  · 零点在中央", 10, CYAN).pack(side="left")
        self._txt(bottom, "仅按 ADC 测量符号显示；未标定时不是 kgf",
                  10, MUTED).pack(side="right")

        channels = tk.Frame(
            work, bg=SURFACE, padx=18, pady=14,
            highlightbackground=EDGE, highlightthickness=1)
        channels.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        self._txt(channels, "G10 · 8 路原始 ADC", 15, bold=True).pack(
            anchor="w")
        self._txt(channels, "其余通道尚未映射到物理单位",
                  10, MUTED).pack(anchor="w", pady=(4, 18))
        table = tk.Frame(channels, bg=SURFACE)
        table.pack(fill="x")
        for column in range(2):
            table.grid_columnconfigure(column, weight=1, uniform="adc")
        for i, var in enumerate(self.channel_vars):
            cell = tk.Frame(
                table, bg=PANEL, padx=11, pady=10,
                highlightbackground=CYAN if i == 6 else EDGE,
                highlightthickness=1)
            cell.grid(row=i // 2, column=i % 2,
                      padx=4, pady=5, sticky="ew")
            name = "ADC 6 · 推力候选" if i == 6 else "ADC %d" % i
            self._txt(cell, name, 10, CYAN if i == 6 else MUTED).pack(
                anchor="w")
            self._txt(cell, "", 14, INK, bold=True, textvariable=var).pack(
                anchor="w", pady=(4, 0))
        tk.Frame(channels, bg=SURFACE, height=13).pack()
        self._txt(channels,
                  "原始 ADC 可跨越 ±32768；推力曲线按相对零点计算。"
                  "反向编码和实际受力仍需双向静态验证。",
                  10, AMBER, wraplength=275, justify="left").pack(
                anchor="w")

        controls = tk.Frame(
            self.root, bg=SURFACE, padx=23, pady=13,
            highlightbackground=EDGE, highlightthickness=1)
        controls.pack(fill="x", padx=24, pady=(0, 12))
        first = tk.Frame(controls, bg=SURFACE)
        first.pack(fill="x")
        self._txt(first, "测量与标定", 13, bold=True).pack(
            side="left", padx=(0, 16))
        self.btn_start = self._button(
            first, "▶ 启动采集", self._start_owned, GREEN)
        self.btn_stop = self._button(
            first, "■ 停止采集", self._stop_owned, AMBER)
        self.btn_tare = self._button(
            first, "↺ 空载去皮", self._tare, CYAN)
        self.btn_status = self._button(
            first, "查看标定", self._status, INK)
        self._txt(first, "已知质量 kg", 10, MUTED).pack(
            side="left", padx=(18, 7))
        self.mass = tk.Entry(
            first, textvariable=self.mass_var, width=8,
            bg=ROOT, fg=INK, insertbackground=INK, bd=0,
            font=(FONT, 11), highlightbackground=EDGE,
            highlightthickness=1)
        self.mass.pack(side="left", ipady=7)
        self.btn_calibrate = self._button(
            first, "砝码标定", self._calibrate, CYAN)
        self._txt(
            controls,
            "GUI 仅显示/标定测量数据，不代替遥控器停机和独立硬件急停。"
            " 去皮与标定要求 DSHOT=0、遥控器 DISARM、静态载荷。",
            10, MUTED, wraplength=1180, justify="left"
        ).pack(anchor="w", pady=(9, 0))

        footer = tk.Frame(self.root, bg=ROOT, padx=24, pady=7)
        footer.pack(fill="x")
        self._txt(footer, "", 10, MUTED,
                  textvariable=self.status_note,
                  anchor="w", justify="left",
                  wraplength=1190).pack(side="left")
        self.root.after(350, self._draw)

    def _button(self, parent, caption, command, color):
        b = tk.Button(
            parent, text=caption, command=command,
            bg=PANEL, fg=color, activebackground=EDGE,
            activeforeground=INK, relief="flat", bd=0,
            cursor="hand2", font=(FONT, 11, "bold"),
            padx=12, pady=8)
        b.pack(side="left", padx=4)
        return b

    def _set_note(self, text):
        self.status_note.set(text)

    def _refresh(self):
        if self.closing:
            return
        try:
            prefix = latest_session(LOG_DIR)
            if prefix != self.prefix:
                self.prefix = prefix
                self.last_mono = -1
                self.history.clear()
                self.last_unit = None
            if prefix is None:
                self.connection.config(text="● 无采集会话", fg=AMBER)
                self.stream_value.set("尚无数据")
            else:
                healthy = stream_fresh(prefix)
                self.connection.config(
                    text="● UDP / ROS 数据在线" if healthy else
                         "● 数据已停止",
                    fg=GREEN if healthy else AMBER)
                rows = recent_rows(prefix + "_force.csv")
                for row in rows:
                    sample = parse_force(row)
                    if sample is None:
                        continue
                    ns, force, unit, raw, dshot = sample
                    if ns <= self.last_mono:
                        continue
                    if unit != self.last_unit:
                        self.history.clear()
                        self.last_unit = unit
                    # Downsample UI graph; measurement CSV remains untouched.
                    if (not self.history or
                            ns - self.history[-1][0] >= 50_000_000):
                        self.history.append((ns, force))
                    self.last_mono = ns
                if rows:
                    sample = parse_force(rows[-1])
                    if sample is not None:
                        _, force, unit, raw, dshot = sample
                        self.main_value.set("%+.3f" % force)
                        self.unit_value.set(unit)
                        self.raw_value.set("%.0f" % raw)
                        self.dshot_value.set(str(dshot))
                channel_row = last_row(prefix, "g10_channels")
                if channel_row is not None:
                    numbers = parse_channels(channel_row)
                    if numbers is not None:
                        for i, val in enumerate(numbers):
                            self.channel_vars[i].set("%d" % val)
                quality = last_row(prefix, "g10_quality")
                if quality is not None:
                    ready = quality.get("stream_ready") == "1"
                    reason = quality.get("reason", "—")
                    if not healthy:
                        self.stream_value.set("已断开")
                    elif ready:
                        self.stream_value.set("正常")
                    else:
                        self.stream_value.set("未就绪")
                    self.age_value.set(
                        "%s · 丢包 %s" %
                        (reason, quality.get("queue_dropped", "?")))
                mode = last_row(prefix, "command")
                if mode is not None:
                    self.mode_value.set(mode.get("mode", "—"))
                if not healthy and self.own_proc is None:
                    self._set_note(
                        "最近会话已停止；点击“启动采集”，或运行原 ROS 节点。")
            if self.own_proc is not None and self.own_proc.poll() is not None:
                code = self.own_proc.returncode
                self.own_proc = None
                self.btn_stop.config(state="disabled")
                if not self.closing:
                    self._set_note(
                        "采集进程已退出（code=%s）。日志：%s" %
                        (code, self.own_log))
            elif self.own_proc is None:
                self.btn_stop.config(state="disabled")
            self._draw()
        except Exception as exc:
            self._set_note("界面刷新异常（采集不受影响）：" + str(exc))
        self.root.after(250, self._refresh)

    def _draw(self):
        c = self.graph
        c.delete("all")
        w, h = max(330, c.winfo_width()), max(230, c.winfo_height())
        x0, x1 = 70, w - 22
        y0, y1 = 19, h - 43
        values = [v for _, v in self.history]
        low, high = plot_limits(values)

        def yy(v):
            return y1 - (v - low) * (y1 - y0) / (high - low)

        for i in range(5):
            value = low + (high - low) * i / 4
            y = yy(value)
            c.create_line(x0, y, x1, y, fill=EDGE, dash=(2, 6))
            c.create_text(x0 - 9, y, text="%+.2g" % value,
                          fill=MUTED, anchor="e", font=(FONT, 9))
        if low <= 0 <= high:
            c.create_line(x0, yy(0), x1, yy(0),
                          fill="#68859a", dash=(5, 5), width=1)
        for i in range(5):
            xx = x0 + (x1 - x0) * i / 4
            c.create_line(xx, y0, xx, y1, fill="#263347")
            c.create_text(xx, y1 + 20,
                          text="-%ds" % int(30 * (1 - i / 4)),
                          fill=MUTED, font=(FONT, 9))
        points = list(self.history)
        if len(points) >= 2:
            right = points[-1][0]
            start = right - 30_000_000_000
            coords = []
            for t, val in points:
                if t >= start:
                    coords.extend((
                        x0 + (t - start) / 30_000_000_000 * (x1 - x0),
                        yy(val)))
            if len(coords) >= 4:
                c.create_line(*coords, fill=CYAN, width=2,
                              smooth=False)
                c.create_oval(coords[-2]-4, coords[-1]-4,
                              coords[-2]+4, coords[-1]+4,
                              fill=CYAN, outline=SURFACE)
        elif not points:
            c.create_text((x0+x1)/2, (y0+y1)/2,
                          text="等待 G10 数据 · 无需打开终端",
                          fill=MUTED, font=(FONT, 13))
        c.create_text(x0, 8, text=self.last_unit or "raw_count",
                      fill=MUTED, anchor="w", font=(FONT, 9))

    def _start_owned(self):
        if self.own_proc is not None and self.own_proc.poll() is None:
            self._set_note("该 GUI 已启动采集，不需要再次启动。")
            return
        if self.prefix and stream_fresh(self.prefix):
            self._set_note(
                "已有正在运行的 ROS 采集节点，GUI 已自动附着；"
                "不会再次绑定 UDP 4800。")
            return
        # A stale CSV is not proof the UDP port is free. Protect other apps.
        try:
            test = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                test.bind(("0.0.0.0", 4800))
            finally:
                test.close()
        except OSError as exc:
            messagebox.showerror(
                "UDP 端口已占用",
                "UDP 4800 已被其他进程占用，请先退出 g10_probe 或检查"
                "正在运行的 ROS 节点。\n" + str(exc))
            return
        try:
            test = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                test.bind(("192.168.127.55", 0))
            finally:
                test.close()
        except OSError:
            messagebox.showwarning(
                "G10 网卡未配置",
                "请先给 enp5s0 配置 192.168.127.55/24。"
                "请勿修改 EtherCAT 专用网卡。")
            return
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.own_log = str(LOG_DIR / "g10_dashboard_ros.log")
        try:
            handle = open(self.own_log, "a", encoding="utf-8")
            try:
                self.own_proc = subprocess.Popen(
                    ros_command("ros2", "launch", "bidirectional_motor_test",
                                "motor_test.launch.py"),
                    stdin=subprocess.DEVNULL, stdout=handle,
                    stderr=subprocess.STDOUT, start_new_session=True)
            finally:
                handle.close()
        except (OSError, FileNotFoundError) as exc:
            messagebox.showerror(
                "启动失败",
                "请先完成 colcon build 并从桌面启动器进入。\n%s" % exc)
            return
        self.btn_stop.config(state="normal")
        self._set_note(
            "已启动 ROS 采集。自动归零前保持台架空载；"
            "G10 原始 UDP 与 EtherCAT 使用两块独立网卡。")

    def _stop_owned(self):
        if self.own_proc is None or self.own_proc.poll() is not None:
            self._set_note(
                "当前节点不是由该 GUI 启动，不能从这里停止。")
            return
        if not messagebox.askyesno(
                "停止采集（非硬件急停）",
                "先将遥控器置于 DISARM，切断 ESC 动力。\n\n"
                "现在请求退出 GUI 启动的 ROS 节点吗？"
                "此操作不是物理急停。"):
            return
        try:
            os.killpg(self.own_proc.pid, signal.SIGINT)
            self._set_note(
                "已请求 ROS 退出；正在等待节点发送零指令并关闭数据文件。")
        except ProcessLookupError:
            pass
        except OSError as exc:
            self._set_note("停止请求失败：" + str(exc))
        self.btn_stop.config(state="disabled")

    def _run_cli(self, args, callback, timeout=12):
        """Run ROS service/parameter CLI off the Tk event loop."""
        if self.ros_job_busy:
            self._set_note("请等待前一项 ROS 操作完成。")
            return
        self.ros_job_busy = True

        def worker():
            try:
                result = subprocess.run(
                    ros_command(*args), capture_output=True, text=True,
                    timeout=timeout, check=False)
                output = (result.stdout + "\n" + result.stderr).strip()
                self.root.after(
                    0, lambda: callback(
                        result.returncode == 0, output[-1800:]))
            except Exception as exc:
                message = str(exc)
                self.root.after(
                    0, lambda message=message: callback(False, message))

        threading.Thread(target=worker, daemon=True).start()

    def _done_cli(self, ok, output):
        self.ros_job_busy = False
        if not ok or re.search(r"success\s*[:=]\s*(?:False|false)",
                               output):
            self._set_note("操作未成功：" + output)
            messagebox.showwarning("ROS 操作未完成", output[-1100:])
        else:
            self._set_note("ROS 返回：" + output)

    def _request_service(self, service):
        self._run_cli(
            ["ros2", "service", "call", ROS_NODE + "/" + service,
             "std_srvs/srv/Trigger", "{}"],
            self._done_cli)

    def _tare(self):
        if not messagebox.askyesno(
                "空载去皮",
                "确认台架已卸载、静止至少 2 秒；"
                "DSHOT=0，遥控器 DISARM，ESC 动力已切断？\n\n"
                "去皮后旧零点将被新零点替换。"):
            return
        self._request_service("g10_tare")

    def _status(self):
        self._request_service("g10_calibration_status")

    def _calibrate(self):
        try:
            mass = float(self.mass_var.get().strip())
            if not math.isfinite(mass) or not 0 < mass <= 10:
                raise ValueError
        except ValueError:
            messagebox.showwarning(
                "质量输入错误",
                "请输入经确认的已知质量，单位 kg，范围 0～10。"
                "未知手按压力不能用于标定。")
            return
        if not messagebox.askyesno(
                "已知质量标定",
                "确认已先空载去皮，然后沿推力测量轴施加"
                " %.5g kg 的已知稳定静载。\n\n"
                "DSHOT=0、遥控器 DISARM、ESC 动力断开？\n"
                "标定会保存 kgf/count 系数，但不复用旧零点。" % mass):
            return

        def after_parameter(ok, output):
            if not ok or re.search(r"(?i)failed|error:", output):
                self._done_cli(False, "无法设置标定质量：" + output)
                return
            # Keep the busy flag across the second ROS CLI operation.
            self.ros_job_busy = False
            self._request_service("g10_calibrate")

        self._run_cli(
            ["ros2", "param", "set", ROS_NODE,
             "g10_calibration_mass_kg", str(mass)],
            after_parameter)

    def _on_close(self):
        if self.own_proc is None or self.own_proc.poll() is not None:
            self.closing = True
            self.root.destroy()
            return
        if not messagebox.askyesno(
                "关闭 G10 界面",
                "GUI 启动的 ROS 节点还在运行。\n"
                "请先 DISARM 并切断 ESC 动力。\n\n"
                "退出界面前请求停止该节点吗？"):
            return
        self.closing = True
        try:
            os.killpg(self.own_proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        except OSError as exc:
            self.closing = False
            messagebox.showerror("退出失败", str(exc))
            return
        self._finish_close(time.monotonic() + 8.0)

    def _finish_close(self, deadline):
        if self.own_proc is None or self.own_proc.poll() is not None:
            self.root.destroy()
        elif time.monotonic() > deadline:
            self.closing = False
            messagebox.showerror(
                "ROS 节点尚未退出",
                "请先切断 ESC 动力并查看进程。"
                "不强制杀死进程，因为这不能保证 DSHOT 硬件停止。")
            self.root.after(250, self._refresh)
        else:
            self.root.after(150, lambda: self._finish_close(deadline))


def main():
    root = tk.Tk()
    try:
        tkfont.nametofont("TkDefaultFont").configure(family=FONT, size=10)
    except tk.TclError:
        pass
    Dashboard(root)
    # Used solely by CI to exercise the installed .desktop startup path
    # under a virtual X display, without ROS or an actual G10.
    if os.environ.get("G10_GUI_SMOKE_TEST") == "1":
        root.after(350, root.destroy)
    root.mainloop()


if __name__ == "__main__":
    main()
