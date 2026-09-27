"""Native Ubuntu Tk dashboard for the existing ROS G10 acquisition process.

The GUI reads CSV snapshots. It NEVER opens the G10 UDP port, publishes
DSHOT, or replaces physical emergency-stop controls.
"""
from __future__ import annotations

from collections import deque
import math
import os
import queue
import tempfile
from pathlib import Path
import re
import signal
import socket
import subprocess
import threading
import time

try:
    import tkinter as tk
    from tkinter import filedialog, messagebox
    from tkinter import font as tkfont
except ImportError as exc:
    raise SystemExit(
        "Ubuntu desktop support is missing. Install: sudo apt install python3-tk"
    ) from exc

from .gui_data import (
    last_row, latest_session, parse_channels, parse_force, plot_limits,
    recent_rows, stream_fresh,
)
from .session_export import export_recording, ExportError


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
        root.geometry("1300x840")
        root.minsize(1010, 740)
        self.prefix = None
        self.last_mono = -1
        self.last_unit = None
        self.history = deque(maxlen=580)
        self.own_proc = None
        self.own_log = None
        self.ros_job_busy = False
        self.closing = False
        self.stop_requested = False
        self.recording = None
        self.record_saving = False
        self.worker_results = queue.Queue()
        self.record_dir = Path(os.environ.get(
            "G10_EXPORT_DIR", "~/bidirectional/recordings")).expanduser()
        self.record_status = tk.StringVar(value="● 未录制")
        self.record_folder_text = tk.StringVar(
            value=str(self.record_dir))
        self.record_latency = tk.StringVar(
            value="延迟：等待录制")
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
        root.after(40, self._drain_worker_results)

    def _txt(self, parent, value, size=11, fg=INK, bold=False, **kwargs):
        return tk.Label(
            parent, text=value, fg=fg, bg=parent.cget("bg"),
            font=(FONT, size, "bold" if bold else "normal"),
            **kwargs)

    def _build(self):
        # Reserve header/metrics/controls/footer rows in the root grid.
        # The chart is the only flexible row; recording controls must never
        # disappear below a fixed-size 820 px window.
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(2, weight=1)
        header = tk.Frame(self.root, bg=ROOT, padx=24, pady=9)
        header.grid(row=0, column=0, sticky="ew")
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
        metrics.grid(row=1, column=0, sticky="ew")
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
            frame = tk.Frame(metrics, bg=SURFACE, padx=20, pady=10,
                             highlightbackground=EDGE, highlightthickness=1)
            frame.grid(row=0, column=col, sticky="ew", padx=5)
            self._txt(frame, label, 10, MUTED).pack(anchor="w")
            if col == 0:
                # Put the *actual* force unit next to the value, not only
                # in a separate small line the user may overlook.
                number_row = tk.Frame(frame, bg=SURFACE)
                number_row.pack(anchor="w", pady=(8, 3))
                self._txt(
                    number_row, "", 24, CYAN, bold=True,
                    textvariable=var).pack(side="left")
                self._txt(
                    number_row, "", 12, MUTED,
                    textvariable=self.unit_value).pack(
                    side="left", padx=(9, 0), pady=(7, 0))
                self._txt(
                    frame, "raw_count 未标定；kgf 为砝码标定后单位",
                    9, MUTED).pack(anchor="w")
            else:
                self._txt(frame, "", 24, INK,
                          bold=True, textvariable=var).pack(
                    anchor="w", pady=(8, 3))
                if isinstance(detail, tk.StringVar):
                    self._txt(frame, "", 10, MUTED,
                              textvariable=detail).pack(anchor="w")
                else:
                    self._txt(frame, detail, 10, MUTED).pack(anchor="w")

        work = tk.Frame(self.root, bg=ROOT, padx=24, pady=7)
        work.grid(row=2, column=0, sticky="nsew")
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
                  10, MUTED).pack(anchor="w", pady=(3, 8))
        table = tk.Frame(channels, bg=SURFACE)
        table.pack(fill="x")
        for column in range(2):
            table.grid_columnconfigure(column, weight=1, uniform="adc")
        for i, var in enumerate(self.channel_vars):
            cell = tk.Frame(
                table, bg=PANEL, padx=11, pady=6,
                highlightbackground=CYAN if i == 6 else EDGE,
                highlightthickness=1)
            cell.grid(row=i // 2, column=i % 2,
                      padx=4, pady=3, sticky="ew")
            name = "ADC 6 · 推力候选" if i == 6 else "ADC %d" % i
            self._txt(cell, name, 10, CYAN if i == 6 else MUTED).pack(
                anchor="w")
            self._txt(cell, "", 14, INK, bold=True, textvariable=var).pack(
                anchor="w", pady=(4, 0))
        tk.Frame(channels, bg=SURFACE, height=7).pack()
        self._txt(channels,
                  "原始 ADC 可跨越 ±32768；推力曲线按相对零点计算。"
                  "反向编码和实际受力仍需双向静态验证。",
                  10, AMBER, wraplength=275, justify="left").pack(
                anchor="w")

        controls = tk.Frame(
            self.root, bg=SURFACE, padx=19, pady=8,
            highlightbackground=EDGE, highlightthickness=1)
        controls.grid(row=3, column=0, sticky="ew",
                      padx=24, pady=(0, 6))
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
        record_bar = tk.Frame(controls, bg=SURFACE)
        record_bar.pack(fill="x", pady=(5, 0))
        self._txt(record_bar, "实验录制", 13, bold=True).pack(
            side="left", padx=(0, 12))
        self.btn_record_start = self._button(
            record_bar, "● 开始录制", self._record_start, GREEN)
        self.btn_record_stop = self._button(
            record_bar, "■ 结束并保存", self._record_stop, AMBER)
        self.btn_record_stop.config(state="disabled")
        self.btn_record_folder = self._button(
            record_bar, "选择保存目录…", self._choose_record_dir, CYAN)
        self._txt(
            record_bar, "", 10, MUTED,
            textvariable=self.record_status).pack(
                side="left", padx=(12, 7))
        self._txt(
            record_bar, "", 10, CYAN,
            textvariable=self.record_latency).pack(side="right")

        folder_bar = tk.Frame(controls, bg=SURFACE)
        folder_bar.pack(fill="x", pady=(1, 0))
        self._txt(folder_bar, "保存到：", 10, MUTED).pack(side="left")
        self._txt(folder_bar, "", 10, MUTED,
                  textvariable=self.record_folder_text,
                  anchor="w").pack(side="left", fill="x", expand=True)
        self._txt(
            controls,
            "录制文件为一个 ZIP，内含原始 DSHOT/推力 CSV、"
            "时间轴、每次换向的延迟及测量说明；界面刷新不参与计时。",
            10, MUTED, justify="left").pack(anchor="w", pady=(2, 0))
        self._txt(
            controls,
            "GUI 仅显示/标定测量数据，不代替遥控器停机和独立硬件急停."
            " 去皮与标定要求 DSHOT=0、遥控器 DISARM、静态载荷。",
            10, MUTED, wraplength=1180, justify="left"
        ).pack(anchor="w", pady=(4, 0))

        footer = tk.Frame(self.root, bg=ROOT, padx=24, pady=3)
        footer.grid(row=4, column=0, sticky="ew")
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
            padx=12, pady=5)
        b.pack(side="left", padx=4)
        return b

    def _set_note(self, text):
        self.status_note.set(text)

    def _drain_worker_results(self):
        """Only Tk's main thread is allowed to touch widgets or root.after.

        Worker threads publish plain objects into Queue. This also works
        under root.update() tests, where Tk mainloop() is not active.
        """
        try:
            while True:
                item = self.worker_results.get_nowait()
                kind = item[0]
                if kind == "record_success":
                    self._record_exported(item[1], item[2])
                elif kind == "record_failure":
                    self._record_export_failed(item[1])
                elif kind == "cli":
                    item[1](item[2], item[3])
        except queue.Empty:
            pass
        try:
            if self.root.winfo_exists():
                self.root.after(40, self._drain_worker_results)
        except tk.TclError:
            pass

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
            self._refresh_recording()
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

    def _choose_record_dir(self):
        if self.recording is not None or self.record_saving:
            self._set_note("请先结束当前录制，再更改保存目录。")
            return
        initial = self.record_dir if self.record_dir.is_dir() else Path.home()
        selected = filedialog.askdirectory(
            title="选择 G10 实验记录保存目录",
            initialdir=str(initial), mustexist=True)
        if selected:
            self.record_dir = Path(selected).expanduser()
            self.record_folder_text.set(str(self.record_dir))
            self._set_note("录制完成后将保存一个 ZIP 至所选目录。")

    def _record_start(self):
        """Mark recording on the SAME host monotonic clock as ROS/G10."""
        if self.record_saving or self.recording is not None:
            self._set_note("已有录制正在进行或正在保存。")
            return
        prefix = latest_session(LOG_DIR)
        if prefix is None or not stream_fresh(prefix, max_age_sec=2.0):
            messagebox.showwarning(
                "无法开始录制",
                "请先点击「启动采集」，并等待 G10 实时数据在线。")
            return
        quality = last_row(prefix, "g10_quality")
        if quality is None or quality.get("stream_ready") != "1":
            messagebox.showwarning(
                "G10 数据尚未就绪",
                "等待自动归零完成，并确认采集状态为「正常」。"
                "无数据或数据过期时不能开始有效的推力实验录制。")
            return
        try:
            self.record_dir.mkdir(parents=True, exist_ok=True)
            # Verify that the destination can actually be written now.
            with tempfile.TemporaryFile(dir=self.record_dir):
                pass
        except OSError as exc:
            messagebox.showerror(
                "保存目录不可写",
                "请重新选择一个有写入权限的目录。\n%s" % exc)
            return
        started_ns = time.monotonic_ns()
        self.recording = {
            "prefix": prefix,
            "folder": str(self.record_dir),
            "start_ns": started_ns,
            "start_wall_ns": time.time_ns(),
            "stop_ns": None,
            "stop_wall_ns": None,
        }
        self.record_status.set("● 正在录制 00:00")
        self.record_latency.set("延迟：等待第一次有效指令变化")
        self.btn_record_start.config(state="disabled")
        self.btn_record_stop.config(state="normal")
        self.btn_record_folder.config(state="disabled")
        self._set_note(
            "录制开始：DSHOT 指令与 G10 推力沿用同一 Ubuntu 单调时间基准。"
            "请通过遥控器按原安全流程进行测试。")

    def _refresh_recording(self):
        session = self.recording
        if session is None or session["stop_ns"] is not None:
            return
        elapsed = max(
            0, (time.monotonic_ns() - session["start_ns"]) // 1_000_000_000)
        self.record_status.set(
            "● 正在录制 %02d:%02d" % (elapsed // 60, elapsed % 60))
        # Show only authoritative, detector-produced results for this
        # recording. Never estimate onset from the 4 Hz screen refresh.
        latency_path = session["prefix"] + "_latency.csv"
        for row in reversed(recent_rows(latency_path, 16384)):
            try:
                within = (int(row["command_mono_ns"]) >=
                          session["start_ns"])
            except (ValueError, TypeError, KeyError):
                continue
            if (within and row.get("metric") == "force_onset" and
                    row.get("status") == "detected" and
                    row.get("latency_ms")):
                self.record_latency.set(
                    "最近推力起效：%s ms（观测值）" %
                    row["latency_ms"])
                break

    def _record_stop(self):
        if self.recording is None:
            self._set_note("当前没有正在录制的实验。")
            return
        self._finish_recording()

    def _finish_recording(self, after_save=None):
        """Export a bounded archive off the GUI thread, without halting ROS."""
        if self.recording is None:
            if after_save:
                after_save()
            return
        if self.record_saving:
            self._set_note("实验数据正在保存，请等待完成。")
            return
        session = self.recording
        if session["stop_ns"] is None:
            session["stop_ns"] = time.monotonic_ns()
            session["stop_wall_ns"] = time.time_ns()
        self.record_saving = True
        self.btn_record_stop.config(state="disabled")
        self.btn_record_start.config(state="disabled")
        self.btn_record_folder.config(state="disabled")
        self.record_status.set("● 正在写入 ZIP，请稍候…")
        self._set_note("正从 ROS 原始 CSV 导出：不是从屏幕曲线重新采样。")

        def worker():
            try:
                result = export_recording(
                    session["prefix"], session["folder"],
                    session["start_ns"], session["stop_ns"],
                    session["start_wall_ns"], session["stop_wall_ns"])
            except Exception as exc:
                self.worker_results.put(("record_failure", str(exc)))
            else:
                self.worker_results.put((
                    "record_success", result, after_save))

        threading.Thread(
            target=worker, daemon=True, name="g10_record_export").start()

    def _record_export_failed(self, message):
        self.record_saving = False
        self.closing = False
        self.btn_record_stop.config(state="normal")
        self.btn_record_start.config(state="disabled")
        self.btn_record_folder.config(state="disabled")
        self.record_status.set("● 保存失败 · 可重试")
        self._set_note(
            "导出失败，录制区间已保留；源 CSV 未删除。"
            "检查保存目录后点击「结束并保存」重试。")
        messagebox.showerror(
            "G10 录制保存失败",
            message + "\n\nROS 原始数据仍保存在 ~/bidirectional/measurements，"
            "可以修复目录权限后重试。")

    def _record_exported(self, result, after_save=None):
        self.record_saving = False
        self.recording = None
        self.btn_record_start.config(state="normal")
        self.btn_record_stop.config(state="disabled")
        self.btn_record_folder.config(state="normal")
        self.record_status.set("✓ 已保存：%s" % Path(result["path"]).name)
        if result["onset_mean_ms"] is not None:
            self.record_latency.set(
                "已测 %d 次，起效延迟均值 %.3f ms（观测值）" %
                (result["detected"], result["onset_mean_ms"]))
        else:
            self.record_latency.set(
                "没有有效起效延迟（已保留原始记录）")
        self._set_note("实验录制已保存到：" + result["path"])
        if not self.closing:
            messagebox.showinfo(
                "G10 实验录制完成",
                "文件已保存：\n%s\n\nDSHOT：%s 行，推力：%s 行\n"
                "指令响应事件：%d；成功识别起效延迟：%d\n\n"
                "仅为 ROS 指令发布时间到估算 G10 采样时间的观测延迟。"
                % (result["path"],
                   result["counts"]["command"],
                   result["counts"]["force"],
                   result["events"], result["detected"]))
        if after_save:
            after_save()

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
        # Seal the recording interval *before* shutdown's extra zero-DSHOT
        # publishes, so the export records exactly what the user requested.
        if self.recording is not None and self.recording["stop_ns"] is None:
            self.recording["stop_ns"] = time.monotonic_ns()
            self.recording["stop_wall_ns"] = time.time_ns()
        try:
            os.killpg(self.own_proc.pid, signal.SIGINT)
            self._set_note(
                "已请求 ROS 退出；正在等待节点发送零指令并关闭数据文件。")
        except ProcessLookupError:
            pass
        except OSError as exc:
            self._set_note("停止请求失败：" + str(exc))
        self.btn_stop.config(state="disabled")
        if self.recording is not None and not self.record_saving:
            # Do not block the safety-related ROS shutdown on ZIP writing.
            self._finish_recording()

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
                self.worker_results.put(
                    ("cli", callback,
                     result.returncode == 0, output[-1800:]))
            except Exception as exc:
                self.worker_results.put((
                    "cli", callback, False, str(exc)))

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
        if self.record_saving:
            messagebox.showinfo(
                "正在保存", "请稍等 ZIP 写入完成，再关闭界面。")
            return
        owns_running = (
            self.own_proc is not None and
            self.own_proc.poll() is None)
        if not owns_running and self.recording is None:
            self.closing = True
            self.root.destroy()
            return
        if owns_running:
            question = (
                "GUI 启动的 ROS 节点还在运行。\n"
                "请先 DISARM 并切断 ESC 动力。\n\n"
                "确认请求停止 ROS、保存录制并关闭界面？"
                if self.recording is not None else
                "GUI 启动的 ROS 节点还在运行。\n"
                "请先 DISARM 并切断 ESC 动力。\n\n"
                "退出界面前请求停止该节点吗？")
        else:
            question = (
                "当前录制尚未保存。\n"
                "确认结束录制并保存 ZIP，然后关闭界面？\n"
                "其他终端启动的 ROS 节点会继续运行。")
        if not messagebox.askyesno("关闭 G10 界面", question):
            return
        self.closing = True
        if self.recording is not None and self.recording["stop_ns"] is None:
            self.recording["stop_ns"] = time.monotonic_ns()
            self.recording["stop_wall_ns"] = time.time_ns()
        if owns_running:
            try:
                os.killpg(self.own_proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            except OSError as exc:
                self.closing = False
                messagebox.showerror("退出失败", str(exc))
                return
        if self.recording is not None:
            if owns_running:
                self._finish_recording(
                    after_save=lambda: self._finish_close(
                        time.monotonic() + 30.0))
            else:
                self._finish_recording(after_save=self.root.destroy)
        elif owns_running:
            self._finish_close(time.monotonic() + 30.0)
        else:
            self.root.destroy()

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
