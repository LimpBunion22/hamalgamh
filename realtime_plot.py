#!/usr/bin/env python3
"""
Realtime plot with interactive point insertion.

Features
--------
- Importable API:
    from realtime_plot import plotter
    plotter.start()  # non-blocking; shows a Matplotlib window
    plotter.put_point(1.23, 4.56)

- CLI data sources:
    * --stdin   : read "x y" pairs from STDIN (one pair per line)
    * --tcp PORT: open a TCP line server on localhost:PORT; send "x y\n"

- Interactivity:
    * Left click on the axes to insert a point at cursor position.
    * Press 'd' and left click to delete the nearest point to the cursor.

- Processing hook:
    * Provide a callback 'on_point' to process every ingested point.
      By default it prints the point. You can override via:
        plotter.on_point = lambda x, y, source: None

Test quickly:
    python3 realtime_plot.py --stdin
    # then type: 0 0 <Enter>, 1 1 <Enter>, 2 1.5 <Enter> ...

Author: ChatGPT
License: MIT
"""
from __future__ import annotations

import argparse
import sys
import threading
import socket
import queue
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple, List

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
import signal, sys

def handler(sig, frame):
    plt.close('all')
    sys.exit(0)

signal.signal(signal.SIGINT, handler)

@dataclass
class Point:
    x: float
    y: float
    source: str  # "stdin" | "tcp" | "api" | "click"

class RealTimePlotter:
    def __init__(
        self,
        max_points: int = 200000,
        autoscale: bool = True,
        xlim: Optional[Tuple[float, float]] = [0, 60],
        ylim: Optional[Tuple[float, float]] = [0, 100],
        update_interval_ms: int = 50,
    ):
        self._q: "queue.Queue[Point]" = queue.Queue()
        self._points_x: List[float] = []
        self._points_y: List[float] = []
        self._max_points = int(max_points)
        self._autoscale = bool(autoscale)
        self._xlim_fixed = xlim
        self._ylim_fixed = ylim
        self._update_interval_ms = int(update_interval_ms)

        # Callback invoked for each point
        # Signature: on_point(x: float, y: float, source: str) -> None
        self.on_point: Callable[[float, float, str], None] = (
            lambda x, y, source: print(f"[{source}] point: {x} {y}")
        )

        # Matplotlib elements
        self._fig: Optional[plt.Figure] = None
        self._ax: Optional[plt.Axes] = None
        self._line: Optional[Line2D] = None
        self._ani: Optional[FuncAnimation] = None

        self._stdin_thread: Optional[threading.Thread] = None
        self._tcp_thread: Optional[threading.Thread] = None
        self._run_threads = threading.Event()
        self._run_threads.clear()

        self._delete_mode = False  # toggled by 'd' key press

    # ---------- Public API ----------
    def start(self, block: bool = True) -> None:
        """Start the UI loop and background threads. If block=False, returns immediately."""
        if self._fig is None:
            self._setup_plot()
        if block:
            plt.show()
        else:
            # non-blocking start via interactive mode
            plt.ion()
            self._fig.show()

    def start_in_thread(self) -> None:
        """Start the plot in a dedicated thread (non-blocking for callers)."""
        t = threading.Thread(target=self.start, kwargs={'block': True}, daemon=True)
        t.start()

    def put_point(self, x: float, y: float, source: str = "api") -> None:
        """Queue a new point (thread-safe)."""
        self._q.put(Point(float(x), float(y), source))

    # ---------- Data sources ----------
    def enable_stdin_reader(self) -> None:
        if self._stdin_thread is None:
            self._run_threads.set()
            self._stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
            self._stdin_thread.start()

    def enable_tcp_server(self, port: int) -> None:
        if self._tcp_thread is None:
            self._run_threads.set()
            self._tcp_thread = threading.Thread(target=self._tcp_loop, args=(port,), daemon=True)
            self._tcp_thread.start()

    # ---------- Internals ----------
    def _setup_plot(self) -> None:
        self._fig, self._ax = plt.subplots()
        self._fig.canvas.manager.set_window_title("Realtime XY plot")
        self._line, = self._ax.plot([], [], lw=1.5, marker='o', ms=2)

        if self._xlim_fixed:
            self._ax.set_xlim(*self._xlim_fixed)
        if self._ylim_fixed:
            self._ax.set_ylim(*self._ylim_fixed)

        self._ax.set_xlabel("x")
        self._ax.set_ylabel("y")
        self._ax.set_title("Realtime points (click to add; press 'd' then click to delete)")

        # Event handlers
        self._fig.canvas.mpl_connect("button_press_event", self._on_click)
        self._fig.canvas.mpl_connect("key_press_event", self._on_key_press)

        # Animation
        self._ani = FuncAnimation(self._fig, self._on_timer, interval=self._update_interval_ms, blit=False)

    def _on_timer(self, _frame):
        drained = 0
        while True:
            try:
                p = self._q.get_nowait()
            except queue.Empty:
                break
            else:
                self._append_point(p)
                drained += 1
        if drained:
            self._refresh_visual()
        return self._line,

    def _append_point(self, p: Point) -> None:
        if(len(self._points_x)>0):
            for i in range(len(self._points_x)):                
                if(self._points_x[i]<p.x):
                    self._points_x.insert(i, p.x)
                    self._points_y.insert(i, p.y)
                    break
                if i == len(self._points_x)-1:      
                    self._points_x.append(p.x)
                    self._points_y.append(p.y)
        else:  
            self._points_x.append(p.x)
            self._points_y.append(p.y)

        if len(self._points_x) > self._max_points:
            # drop from the front
            overflow = len(self._points_x) - self._max_points
            del self._points_x[:overflow]
            del self._points_y[:overflow]
        # user callback
        try:
            self.on_point(p.x, p.y, p.source)
        except Exception as e:
            print(f"[on_point error] {e}", file=sys.stderr)

    def _refresh_visual(self) -> None:
        self._line.set_data(self._points_x, self._points_y)
        if self._autoscale and (self._xlim_fixed is None or self._ylim_fixed is None):
            self._ax.relim()
            self._ax.autoscale_view()
            # add a small margin for nicer visuals
            x_min, x_max = self._ax.get_xlim()
            y_min, y_max = self._ax.get_ylim()
            self._ax.set_xlim(*self._add_margin(x_min, x_max))
            self._ax.set_ylim(*self._add_margin(y_min, y_max))
        self._fig.canvas.draw_idle()

    @staticmethod
    def _add_margin(a: float, b: float, frac: float = 0.05) -> Tuple[float, float]:
        span = (b - a) if (b - a) != 0 else 1.0
        m = span * frac
        return a - m, b + m

    # ---------- Interactivity ----------
    def _on_key_press(self, event):
        if event.key == 'd':
            self._delete_mode = True
            self._ax.set_title("Delete mode: click near a point to remove it (press ESC to exit)")
            self._fig.canvas.draw_idle()
        elif event.key == 'escape':
            self._delete_mode = False
            self._ax.set_title("Realtime points (click to add; press 'd' then click to delete)")
            self._fig.canvas.draw_idle()

    def _on_click(self, event):
        if event.inaxes != self._ax:
            return
        if self._delete_mode:
            self._delete_nearest(event.xdata, event.ydata)
            self._refresh_visual()
            return
        # add point at click
        self.put_point(event.xdata, event.ydata, source="click")

    def _delete_nearest(self, x: float, y: float) -> None:
        if not self._points_x:
            return
        # find nearest point by squared distance
        best_i = None
        best_d2 = None
        for i, (px, py) in enumerate(zip(self._points_x, self._points_y)):
            d2 = (px - x) ** 2 + (py - y) ** 2
            if best_d2 is None or d2 < best_d2:
                best_d2 = d2
                best_i = i
        if best_i is not None:
            del self._points_x[best_i]
            del self._points_y[best_i]

    # ---------- Background readers ----------
    def _stdin_loop(self):
        print("[stdin reader] expecting 'x y' per line. Ctrl-D to stop.", file=sys.stderr)
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                x_str, y_str = line.split()
                x = float(x_str)
                y = float(y_str)
                self.put_point(x, y, source="stdin")
            except Exception as e:
                print(f"[stdin reader] bad line '{line}': {e}", file=sys.stderr)

    def _tcp_loop(self, port: int):
        print(f"[tcp server] listening on 127.0.0.1:{port} (send 'x y\\n')", file=sys.stderr)
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", int(port)))
        srv.listen(1)
        srv.settimeout(1.0)
        conn = None
        buf = ""
        try:
            while True:
                # accept (with timeout) or reuse existing connection
                if conn is None:
                    try:
                        conn, _addr = srv.accept()
                        conn.settimeout(0.1)
                        print("[tcp server] client connected", file=sys.stderr)
                    except socket.timeout:
                        continue
                try:
                    data = conn.recv(4096)
                    if not data:
                        conn.close()
                        conn = None
                        print("[tcp server] client disconnected", file=sys.stderr)
                        continue
                    buf += data.decode("utf-8", errors="ignore")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            x_str, y_str = line.split()
                            x = float(x_str)
                            y = float(y_str)
                            self.put_point(x, y, source="tcp")
                        except Exception as e:
                            print(f"[tcp server] bad line '{line}': {e}", file=sys.stderr)
                except socket.timeout:
                    continue
        except Exception as e:
            print(f"[tcp server] fatal error: {e}", file=sys.stderr)
        finally:
            try:
                if conn is not None:
                    conn.close()
            finally:
                srv.close()

# Singleton-style helper for importers
plotter = RealTimePlotter()

def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Realtime XY plot with interactive insertion.")
    parser.add_argument("--stdin", action="store_true", help="Read 'x y' pairs from STDIN.")
    parser.add_argument("--tcp", type=int, default=None, help="Open TCP line server on localhost:PORT.")
    parser.add_argument("--max-points", type=int, default=200000, help="Maximum points kept in memory.")
    parser.add_argument("--no-autoscale", action="store_true", help="Disable autoscaling (use with --xlim/--ylim).")
    parser.add_argument("--xlim", type=float, nargs=2, default=None, help="Fixed x limits: xmin xmax.")
    parser.add_argument("--ylim", type=float, nargs=2, default=None, help="Fixed y limits: ymin ymax.")
    parser.add_argument("--fps", type=float, default=20.0, help="Target updates per second (default 20).")
    args = parser.parse_args(argv)

    # configure singleton
    plotter._max_points = args.max_points
    plotter._autoscale = not args.no_autoscale
    plotter._xlim_fixed = tuple(args.xlim) if args.xlim else None
    plotter._ylim_fixed = tuple(args.ylim) if args.ylim else None
    plotter._update_interval_ms = int(max(1.0, 1000.0 / float(args.fps)))

    # data sources
    if args.stdin:
        plotter.enable_stdin_reader()
    if args.tcp:
        plotter.enable_tcp_server(args.tcp)

    plotter.start(block=True)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
