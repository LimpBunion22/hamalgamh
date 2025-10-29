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
    * --tcp PORT: open a TCP line server on localhost:PORT; send "x y\\n"

- Interactivity:
    * Left click on the axes to insert a point at cursor position.
    * Press 'd' and left click to delete the nearest point to the cursor.

- Processing hook:
    * Provide a callback 'on_point' to process every ingested point.
      By default it prints the point. You can override via:
        plotter.on_point = lambda x, y, source: None

- Window & Theme:
    * --window-frac 0.75x1.0 sets width=75% of screen, height=100% of screen.
    * --theme industrial|dark|light (default industrial) for an "industrial" dark look.

Author: ChatGPT
License: MIT
"""
from __future__ import annotations

import argparse
import sys
import threading
import socket
import queue
from dataclasses import dataclass
from typing import Callable, Optional, Tuple, List

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
import numpy as np
from matplotlib.widgets import Button
import csv
from datetime import datetime
from itertools import zip_longest

import platform

so = platform.system()
if so == "Windows":
    import win32ui
    import win32con 
else:
    from tkinter import Tk, filedialog

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
        theme: str = 'industrial',
        window_frac: Tuple[float, float] = (0.75, 1.0),
    ):
        self._q: "queue.Queue[Point]" = queue.Queue()
        self._points_x: List[float] = []
        self._points_y: List[float] = []
        # Segunda línea (puntos del Arduino)
        self._arduino_x: List[float] = []
        self._arduino_y: List[float] = []
        self._line_arduino: Optional[Line2D] = None
        # Segundo gráfico (subplot inferior)
        self._points2_x: List[float] = []
        self._points2_y: List[float] = []
        self._line2: Optional[Line2D] = None
        self._ax_bottom: Optional[plt.Axes] = None


        self._max_points = int(max_points)
        self._autoscale = bool(autoscale)
        self._xlim_fixed = xlim
        self._ylim_fixed = ylim
        self._xlim_bottom_fixed = (0, 100)  # eje X del subplot inferior

        self._update_interval_ms = int(update_interval_ms)
        self._theme = theme
        self._window_frac = window_frac

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

        #Event system
        self.event = False
        self.eventCmd = ""

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

    def put_point_arduino(self, x: float, y: float) -> None:
        """Añade un punto a la línea del Arduino."""
        self._q.put(Point(float(x), float(y), source="arduino"))

    def set_second_graph(self, xs: List[float], ys: List[float]) -> None:
        """
        Actualiza el segundo gráfico (subplot inferior) con listas de X e Y.
        Mantiene los mismos colores que la serie principal.
        """
        if len(xs) != len(ys):
            raise ValueError("xs y ys deben tener la misma longitud")
        # Copiamos/normalizamos a float
        self._points2_x = [float(v) for v in xs]
        self._points2_y = [float(v) for v in ys]
        # Recorta si excede max_points (consistencia con el primero)
        if len(self._points2_x) > self._max_points:
            overflow = len(self._points2_x) - self._max_points
            del self._points2_x[:overflow]
            del self._points2_y[:overflow]
        # Refresco inmediato
        self._refresh_visual()

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

    # ---------- Window & Theme helpers ----------
    @staticmethod
    def _get_screen_size() -> Tuple[int, int]:
        try:
            import tkinter as tk  # lightweight way to query screen
            root = tk.Tk()
            root.withdraw()
            w = root.winfo_screenwidth()
            h = root.winfo_screenheight()
            root.destroy()
            return int(w), int(h)
        except Exception:
            return 1280, 720

    def _apply_theme(self) -> None:
        theme = (self._theme or "industrial").lower()
        if theme == "industrial":
            # reset first to avoid style accumulation
            matplotlib.rcdefaults()
            rc = matplotlib.rcParams
            rc.update({
                "figure.facecolor": "#0c0f12",
                "axes.facecolor": "#0c0f12",
                "axes.edgecolor": "#6a717d",
                "axes.labelcolor": "#cbd3dc",
                "text.color": "#cbd3dc",
                "xtick.color": "#aab2bd",
                "ytick.color": "#aab2bd",
                "grid.color": "#2a2f36",
                "grid.linestyle": (0, (3, 3)),
                "grid.linewidth": 0.8,
                "axes.grid": True,
                "font.size": 12,
                "font.family": "DejaVu Sans",
            })
        elif theme == "dark":
            matplotlib.rcdefaults()
            try:
                plt.style.use("dark_background")
            except Exception:
                pass
        elif theme == "light":
            matplotlib.rcdefaults()
        else:
            matplotlib.rcdefaults()

    def _size_window(self) -> None:
        # Resize the GUI window to (width_frac * screen_w, height_frac * screen_h)
        w_screen, h_screen = self._get_screen_size()
        frac_w, frac_h = self._window_frac
        w_px = max(200, int(w_screen * float(frac_w)))
        h_px = max(200, int(h_screen * float(frac_h)))

        try:
            mgr = self._fig.canvas.manager
            win = getattr(mgr, "window", None)
            # Try Tk
            if win is not None and hasattr(win, "wm_geometry"):
                win.wm_geometry(f"{w_px}x{h_px}+0+0")
            # Try Qt
            elif win is not None and hasattr(win, "resize"):
                try:
                    win.resize(w_px, h_px)
                    if hasattr(win, "move"):
                        win.move(0, 0)
                except Exception:
                    pass
        except Exception:
            pass

        # Also set figure inches as fallback (works across backends)
        dpi = self._fig.get_dpi()
        self._fig.set_size_inches(w_px / dpi, h_px / dpi, forward=True)

    # ---------- Internals ----------
    def _setup_plot(self) -> None:
        self._apply_theme()
        # self._fig, self._ax = plt.subplots()
        self._fig, (self._ax, self._ax_bottom) = plt.subplots(2, 1, height_ratios=[2, 1])
        self._ax2 = self._ax.twinx()
        self._ax2.patch.set_alpha(0)         # fondo transparente
        self._ax2.grid(False)                # que la rejilla no duplique

        try:
            self._fig.canvas.manager.set_window_title("HAMALGAMH TOOL")
        except Exception:
            pass
        self._line, = self._ax.plot([], [], lw=1.8, marker='o', ms=3)
        # Línea para puntos Arduino
        self._line_arduino, = self._ax2.plot([], [], lw=2, color='#ffb347', marker=None, ms=4, label='Par medido')
        self._legend = self._ax2.legend(loc='upper right', facecolor='#0c0f12', edgecolor='#6a717d', labelcolor='#cbd3dc')
        self._line2, = self._ax_bottom.plot([], [], lw=1.8, marker='o', ms=3)

        if self._xlim_fixed:
            self._ax.set_xlim(*self._xlim_fixed)
            self._ax2.set_xlim(*self._xlim_fixed)

        if hasattr(self, "_xlim_bottom_fixed") and self._xlim_bottom_fixed:
            self._ax_bottom.set_xlim(*self._xlim_bottom_fixed)

        if self._ylim_fixed:
            self._ax.set_ylim(*self._ylim_fixed)
            # self._ax_bottom.set_ylim(*self._ylim_fixed)  # usa mismo rango Y por defecto


        self._ax.set_xlabel("Tiempo [s]")
        self._ax.set_ylabel("Actuación del servo [%]")
        self._ax2.set_ylabel("Par de frenado [N·m]")
        self._ax.set_title("Puntos de actuación del servo (click to add; press 'd' then click to delete)")
        self._ax_bottom.set_xlabel("Actuación del servo [%]")
        self._ax_bottom.set_ylabel("Par de frenado [N·m]")

        # theme-specific line colors
        if (self._theme or '').lower() == 'industrial' and self._line is not None:
            self._line.set_color('#00d1d1')
            self._line.set_markerfacecolor('#0c0f12')
            self._line.set_markeredgecolor('#8bdada')

            self._line2.set_color('#00d1d1')
            self._line2.set_markerfacecolor('#0c0f12')
            self._line2.set_markeredgecolor('#8bdada')

        # Event handlers
        self._fig.canvas.mpl_connect("button_press_event", self._on_click)
        self._fig.canvas.mpl_connect("key_press_event", self._on_key_press)

        # Animation
        self._ani = FuncAnimation(self._fig, self._on_timer, interval=self._update_interval_ms, blit=False)

        # Size window after creation
        self._size_window()
        
        clear_ax = self._fig.add_axes([0.005, 0.95, 0.06, 0.03]) 
        self._btn_clear = Button(
            ax=clear_ax,
            label='CLEAR',
            color='#2a2f36',
            hovercolor='#444a55',
        )

        exit_ax = self._fig.add_axes([0.005, 0.90, 0.06, 0.03]) 
        self._btn_exit= Button(
            ax=exit_ax,
            label='EXIT',
            color='#2a2f36',
            hovercolor='#444a55',
        )

        run_ax = self._fig.add_axes([0.005, 0.85, 0.06, 0.03]) 
        self._btn_run= Button(
            ax=run_ax,
            label='RUN',
            color='#2a2f36',
            hovercolor='#444a55',
        )

        save_ax = self._fig.add_axes([0.005, 0.80, 0.06, 0.03]) 
        self._btn_save= Button(
            ax=save_ax,
            label='SAVE',
            color='#2a2f36',
            hovercolor='#444a55',
        )

        load_ax = self._fig.add_axes([0.005, 0.75, 0.06, 0.03]) 
        self._btn_load= Button(
            ax=load_ax,
            label='LOAD',
            color='#2a2f36',
            hovercolor='#444a55',
        )

        def _on_clear(event):
            # vaciar datos
            self._points_x.clear()
            self._points_y.clear()
            self._arduino_x.clear()
            self._arduino_y.clear()
            self._points2_x.clear()
            self._points2_y.clear()
            self._points_x.append(0.0)
            self._points_y.append(0.0)
            self._refresh_visual()

        def _on_exit(event):
            self.eventCmd = "[EXIT]"            
            self.event = True

        def _on_run(event):
            self.eventCmd = "[RUN]"            
            self.event = True

        def _on_save(event):

            ruta = None
            if so == "Windows":
                dlg = win32ui.CreateFileDialog(
                    0, ".csv", None, win32con.OFN_OVERWRITEPROMPT,
                    "CSV (*.csv)|*.csv|Todos los archivos (*.*)|*.*|"
                )
                dlg.SetOFNInitialDir(r"C:\Temp")
                if dlg.DoModal() == win32con.IDOK:
                    ruta = dlg.GetPathName()
            else:
                root = Tk()
                root.withdraw()  # oculta la ventana principal

                ruta = filedialog.asksaveasfilename(
                    title="Guardar archivo CSV",
                    defaultextension=".csv",
                    filetypes=[("Archivos CSV", "*.csv"), ("Todos los archivos", "*.*")]
                )

            if not ruta:
                print("Guardado cancelado.")
            else:
                with open(ruta, "w", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow(["Tiempo servo [s]", "Puntos Servo", "Tiempo[s]", "Servo[%%]", "Par[N·m]"])  # cabecera
                    writer.writerows(zip_longest(self._points_x, self._points_y, self._arduino_x, self._points2_x, self._arduino_y, fillvalue=""))
            root.destroy()

        def _on_load(event):

            ruta = None
            if so == "Windows":
                dlg = win32ui.CreateFileDialog(
                    0, ".csv", None, win32con.OFN_OVERWRITEPROMPT,
                    "CSV (*.csv)|*.csv|Todos los archivos (*.*)|*.*|"
                )
                dlg.SetOFNInitialDir(r"C:\Temp")
                if dlg.DoModal() == win32con.IDOK:
                    ruta = dlg.GetPathName()
            else:
                root = Tk()
                root.withdraw()  # oculta la ventana principal de Tk
                ruta = filedialog.askopenfilename(
                    title="Selecciona un archivo CSV",
                    filetypes=[("Archivos CSV", "*.csv"), ("Todos los archivos", "*.*")]
                )

            if not ruta:
                print("No se seleccionó ningún archivo.")
            else:
                self._points_x.clear()
                self._points_y.clear()
                self._arduino_x.clear()
                self._arduino_y.clear()
                self._points2_x.clear()
                self._points2_y.clear()

                with open(ruta, "r") as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        val = row["Tiempo servo [s]"]
                        if val:
                            self._points_x.append(float(row["Tiempo servo [s]"]) or 0.0)
                            self._points_y.append(float(row["Puntos Servo"])  or 0.0)
                        val = row["Tiempo[s]"]
                        if val:
                            self._arduino_x.append(float(row["Tiempo[s]"])  or 0.0)
                            self._arduino_y.append(float(row["Par[N·m]"])  or 0.0)
                            self._points2_x.append(float(row["Servo[%%]"])  or 0.0)
                            self._points2_y.append(float(row["Par[N·m]"])  or 0.0)
                
                self._refresh_visual()
            root.destroy()

        self._btn_clear.on_clicked(_on_clear)
        self._btn_exit.on_clicked(_on_exit)
        self._btn_run.on_clicked(_on_run)
        self._btn_save.on_clicked(_on_save)
        self._btn_load.on_clicked(_on_load)

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
        if p.source == "arduino":
            self._arduino_x.append(p.x)
            self._arduino_y.append(p.y)
            if len(self._arduino_x) > self._max_points:
                overflow = len(self._arduino_x) - self._max_points
                del self._arduino_x[:overflow]
                del self._arduino_y[:overflow]
            if self._arduino_y:
                self._update_legend_label(f"Par medido: {self._arduino_y[-1]:.2f} N·m\nPar maximo:  {np.max(self._arduino_y):.2f} N·m")
        else:
            if(len(self._points_x)>0):
                for i in range(len(self._points_x)):                
                    if(self._points_x[i]>p.x):
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
        if self._line_arduino is not None:
            self._line_arduino.set_data(self._arduino_x, self._arduino_y)

        if self._line2 is not None:
            self._line2.set_data(self._points2_x, self._points2_y)
            
        # --- Autoscaling dinámico ---
        if self._autoscale:
            # EJE SUPERIOR IZQUIERDO (principal)
            if self._xlim_fixed is None or self._ylim_fixed is None:
                self._ax.relim()
                self._ax.autoscale_view(scalex=(self._xlim_fixed is None), scaley=(self._ylim_fixed is None))
                x_min, x_max = self._ax.get_xlim()
                y_min, y_max = self._ax.get_ylim()
                # Margen suave
                self._ax.set_xlim(*self._add_margin(x_min, x_max)) if self._xlim_fixed is None else None
                self._ax.set_ylim(*self._add_margin(y_min, y_max)) if self._ylim_fixed is None else None

            # EJE SUPERIOR DERECHO (twinx): deja que matplotlib escale Y
            if getattr(self, "_ax2", None) is not None and self._line_arduino is not None:
                # primero igualamos el X con el eje principal
                x_min, x_max = self._ax.get_xlim()
                self._ax2.set_xlim(x_min, x_max)

                # ahora pedimos a matplotlib que recalcule el rango Y con TODOS esos datos
                self._ax2.relim()
                self._ax2.autoscale_view(scalex=False, scaley=True)

            # SUBPLOT INFERIOR: deja que matplotlib escale Y
            if getattr(self, "_ax_bottom", None) is not None and self._line2 is not None:
                self._ax_bottom.relim()
                self._ax_bottom.autoscale_view(scalex=False, scaley=True)

        self._fig.canvas.draw_idle()
        # x_min, x_max = self._ax.get_xlim()
        # self._ax_bottom.set_xlim(x_min, x_max)

    @staticmethod
    def _add_margin(a: float, b: float, frac: float = 0.05) -> Tuple[float, float]:
        span = (b - a) if (b - a) != 0 else 1.0
        m = span * frac
        return a - m, b + m

    def _y_to_primary(self, y: float, from_axes: plt.Axes) -> float:
        """Convierte una coordenada y de from_axes a la escala de self._ax."""
        if from_axes is self._ax:
            return y
        # Pasamos y de coords de datos -> coords de pantalla con el eje origen...
        y_disp = from_axes.transData.transform((0, y))[1]
        # ...y luego de pantalla -> datos pero del eje primario.
        return self._ax.transData.inverted().transform((0, y_disp))[1]
    
    def _update_legend_label(self, text: str) -> None:
        """Actualiza el texto de la leyenda del gráfico derecho (self._ax2)."""
        if not hasattr(self, "_legend") or self._legend is None:
            return
        try:
            # Cambia el texto del primer elemento de la leyenda
            self._legend.texts[0].set_text(text)
            self._fig.canvas.draw_idle()
        except Exception as e:
            print(f"[legend update error] {e}", file=sys.stderr)

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
        # Acepta clics en ax o ax2; ignora fuera de ambos
        if event.inaxes not in (self._ax, getattr(self, "_ax2", None)):
            return

        # Mapea y a la escala del eje primario (izquierda)
        y_primary = self._y_to_primary(event.ydata, event.inaxes)
        x_val = event.xdata

        if self._delete_mode:
            self._delete_nearest(x_val, y_primary)
            self._refresh_visual()
            return

        # añadir punto en eje primario
        self.put_point(x_val, y_primary, source="click")

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
        import socket
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
    parser.add_argument("--theme", choices=["industrial", "dark", "light"], default="industrial", help="UI theme")
    parser.add_argument("--window-frac", type=str, default="0.75x1.0", help="Window size as WIDTHxHEIGHT fractions of screen, e.g., 0.75x1.0")
    args = parser.parse_args(argv)

    # configure singleton
    plotter._max_points = args.max_points
    plotter._autoscale = not args.no_autoscale
    plotter._xlim_fixed = tuple(args.xlim) if args.xlim else None
    plotter._ylim_fixed = tuple(args.ylim) if args.ylim else None
    plotter._update_interval_ms = int(max(1.0, 1000.0 / float(args.fps)))
    plotter._theme = args.theme
    try:
        wf = args.window_frac.lower().replace(' ', '')
        w_str, h_str = wf.split('x', 1)
        plotter._window_frac = (float(w_str), float(h_str))
    except Exception:
        plotter._window_frac = (0.75, 1.0)

    # data sources
    if args.stdin:
        plotter.enable_stdin_reader()
    if args.tcp:
        plotter.enable_tcp_server(args.tcp)

    plotter.start(block=True)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
