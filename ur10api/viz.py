"""Live plots (matplotlib) of the arm, its frames and paths, and of logged signals.

Two tools, both in the UR base frame (axes: X red, Y green, Z blue):

FrameView - a single 3D view you refresh from your own loop:

    view = FrameView(robot.arm, title="Pose control", workspace=robot.workspace)
    view.add_frame(target, "T1")            # static frames, e.g. targets
    while running:
        view.update(robot.state())          # redraws at most `max_fps` times per second
    view.hold()                             # keep the window open at the end

Dashboard - an overview of the arm, a 3D close-up of the task and time plots,
with your control loop in a worker thread so that drawing never delays it; at
the end the window becomes a still summary of the whole run:

    dash = Dashboard(robot, "Demo", panels=[
        Panel("TCP height", "z [mm]", {"measured": "z", "target": "z_ref"}),
    ], paths={"TCP path": ("x", "y", "z")})
    dash.scene.add_frame(target, "T1")     # the close-up zooms on frames, paths and the TCP path
    def control(stop):                      # runs in the worker thread
        while not stop.is_set():
            ...                             # read the state, send commands
            dash.record(t, x=..., y=..., z=..., z_ref=...)   # mm
    dash.run(control)                       # returns when done, on Ctrl+C or when the window closes
    robot.stop()
    dash.finish(["RMS error 1.2 mm"])       # full-length plots + summary text, saved as PNG

Needs an interactive matplotlib backend (Tk on Windows ships with Python; on
Ubuntu: sudo apt install python3-tk). Without one, finish() still saves the PNG.
"""

import math
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

AXIS_COLORS = ("tab:red", "tab:green", "tab:blue")
ROOT = Path(__file__).resolve().parents[1]


class Scene:
    """The arm, frames, paths and markers on a matplotlib 3D axes (metres, UR base frame).

    clip=True hides whatever lies outside the axes limits (for zoomed views);
    compact=True drops the tick labels (for small overviews).
    """

    def __init__(self, ax, arm, workspace=None, extent=0.9, center=(-0.5, 0.15, 0.35), title=None,
                 clip=False, tcp_label=True, base_frame=True, compact=False):
        self.ax, self.arm, self._clip = ax, arm, clip
        self.set_limits(center, extent)
        ax.set_box_aspect((1, 1, 1))
        if compact:
            ax.set_xticklabels([])
            ax.set_yticklabels([])
            ax.set_zticklabels([])
        else:
            ax.set_xlabel("x [m]", fontsize=9)
            ax.set_ylabel("y [m]", fontsize=9)
            ax.set_zlabel("z [m]", fontsize=9)
            ax.tick_params(labelsize=7)
        ax.view_init(elev=22, azim=-125)
        if title:
            ax.set_title(title, fontsize=10)
        self.points = []     # positions of frames, paths and markers (to zoom on them)
        self._dynamic = {}   # name -> list of artists (moving frames, markers, vectors, lines)
        if base_frame:
            self._draw_axes(np.eye(4), 0.2, "UR base", lw=2.0)
        if workspace is not None:
            self._draw_box(workspace.limits)
        self._arm_line, = self._plot([], [], [], "-o", color="0.45", lw=3, ms=4, label="arm")
        self._tcp_lines = [self._plot([], [], [], color=c, lw=2.5)[0] for c in AXIS_COLORS]
        self._tcp_text = self.ax.text(0, 0, 0, "TCP", fontsize=9, axlim_clip=clip) if tcp_label else None
        self._tcp_size = 0.08

    def _plot(self, *args, **kwargs):
        return self.ax.plot(*args, axlim_clip=self._clip, **kwargs)

    def set_limits(self, center, half_width):
        """Show the cube of half width `half_width` [m] around `center`."""
        cx, cy, cz = center
        self.ax.set_xlim(cx - half_width, cx + half_width)
        self.ax.set_ylim(cy - half_width, cy + half_width)
        self.ax.set_zlim(cz - half_width, cz + half_width)
        self._tcp_size = min(0.08, 0.25 * half_width)

    def _draw_axes(self, T, size, label=None, lw=1.5, style="-", alpha=1.0):
        T = np.asarray(T, dtype=float)
        origin = T[:3, 3]
        artists = [self._plot(*zip(origin, origin + size * T[:3, i]), style, color=c, lw=lw, alpha=alpha)[0]
                   for i, c in enumerate(AXIS_COLORS)]
        if label:
            artists.append(self.ax.text(*(origin + size * np.array([0.2, 0.2, 0.35])), label, fontsize=9,
                                        axlim_clip=self._clip))
        return artists

    def _draw_box(self, limits):
        (x0, x1), (y0, y1), (z0, z1) = limits["x"], limits["y"], limits["z"]
        corners = np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)])
        for i in range(8):
            for j in range(i + 1, 8):
                if np.sum(corners[i] != corners[j]) == 1:   # corners differing in one coordinate = an edge
                    self._plot(*zip(corners[i], corners[j]), color="tab:blue", lw=0.6, alpha=0.35)

    # --- fixed items
    def add_frame(self, T, label=None, size=0.04, style="--"):
        """Draw a fixed frame (e.g. a target pose)."""
        self._draw_axes(T, size, label, lw=1.5, style=style)
        self.points.append(np.asarray(T, dtype=float)[:3, 3])

    def add_path(self, points, color="0.55", style=":", label=None, lw=1.4):
        """Draw a fixed polyline (N x 3 points [m]), e.g. a reference path."""
        points = np.asarray(points, dtype=float)
        self._plot(points[:, 0], points[:, 1], points[:, 2], style, color=color, lw=lw, label=label)
        self.points.extend(points)

    # --- moving items (call again to move them)
    def _replace(self, name, artists):
        for artist in self._dynamic.pop(name, []):
            artist.remove()
        self._dynamic[name] = artists

    def set_frame(self, name, T, label=None, size=0.06):
        """Show (or move) a highlighted frame, e.g. the current target."""
        self._replace(name, self._draw_axes(T, size, label, lw=3.5, alpha=0.6))

    def set_marker(self, name, point, color="tab:orange", marker="o", label=None, size=8):
        """Show (or move) a point [m], e.g. the moving reference of a path."""
        if name in self._dynamic:
            self._dynamic[name][0].set_data_3d([point[0]], [point[1]], [point[2]])
        else:
            self._dynamic[name] = self._plot([point[0]], [point[1]], [point[2]], marker, color=color,
                                             ms=size, label=label)

    def set_vector(self, name, origin, vector, color="tab:orange", label=None):
        """Show (or move) an arrow-like segment from `origin` along `vector` [m], e.g. a scaled force."""
        start = np.asarray(origin, dtype=float)
        end = start + np.asarray(vector, dtype=float)
        if name in self._dynamic:
            self._dynamic[name][0].set_data_3d(*zip(start, end))
            self._dynamic[name][1].set_data_3d([end[0]], [end[1]], [end[2]])
        else:
            self._dynamic[name] = [self._plot(*zip(start, end), color=color, lw=3, label=label)[0],
                                   self._plot([end[0]], [end[1]], [end[2]], "^", color=color, ms=7)[0]]

    def set_line(self, name, points, color="tab:purple", style="-", label=None, lw=1.8):
        """Show (or update) a polyline (N x 3 points [m]), e.g. the path followed so far."""
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        if name in self._dynamic:
            self._dynamic[name][0].set_data_3d(points[:, 0], points[:, 1], points[:, 2])
        else:
            self._dynamic[name] = self._plot(points[:, 0], points[:, 1], points[:, 2], style, color=color,
                                             lw=lw, label=label)

    def draw_arm(self, state):
        """Arm skeleton and TCP frame for a RobotState."""
        points = self.arm.skeleton(state.q)
        self._arm_line.set_data_3d(points[:, 0], points[:, 1], points[:, 2])
        T = state.tcp
        for i, line in enumerate(self._tcp_lines):
            line.set_data_3d(*zip(T[:3, 3], T[:3, 3] + self._tcp_size * T[:3, i]))
        if self._tcp_text is not None:
            self._tcp_text.set_position_3d(T[:3, 3] + 0.02)


class FrameView:
    """One 3D view in its own window, refreshed from your loop with update(state)."""

    def __init__(self, arm, title="UR10", workspace=None, trail=True, max_fps=8, extent=0.9,
                 center=(-0.5, 0.15, 0.35)):
        plt.ion()
        self.fig = plt.figure(title, figsize=(8, 7))
        self.scene = Scene(self.fig.add_subplot(projection="3d"), arm, workspace, extent, center, title)
        self.ax = self.scene.ax
        self._period = 1.0 / max_fps
        self._last_draw = 0.0
        self._trail = [] if trail else None

    def add_frame(self, T, label=None, size=0.06, style="--"):
        """Draw a fixed frame (e.g. a target pose)."""
        self.scene.add_frame(T, label, size, style)

    def add_path(self, points, color="0.6", style=":"):
        """Draw a fixed polyline (N x 3 points [m]), e.g. a reference path."""
        self.scene.add_path(points, color, style)

    def set_vector(self, name, origin, vector, color="tab:orange"):
        """Show (or move) an arrow-like segment from `origin` along `vector` [m], e.g. a scaled force."""
        self.scene.set_vector(name, origin, vector, color)

    def update(self, state, force=False):
        """Redraw the arm, the TCP frame and the trail for a RobotState (throttled unless force=True)."""
        if self._trail is not None:
            self._trail.append(state.tcp[:3, 3].copy())
        now = time.monotonic()
        if not force and now - self._last_draw < self._period:
            return
        self._last_draw = now
        self.scene.draw_arm(state)
        if self._trail:
            self.scene.set_line("trail", self._trail[-3000:])
        self.pause()

    def pause(self, seconds=0.001):
        """Let the window process its events (redraw, mouse rotation)."""
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        if seconds > 0.001:
            time.sleep(seconds)

    def hold(self):
        """Keep the window open until the user closes it."""
        plt.ioff()
        plt.show()


@dataclass
class Panel:
    """A plot of the Dashboard.

    kind="time": lines = {legend label: column}, plotted against time [s].
    kind="xy":   lines = {legend label: (x column, y column)}, e.g. a path in a plane.
    styles:      {legend label: matplotlib keyword arguments}, e.g. {"target": {"ls": "--"}}.
    hlines:      horizontal reference lines {label: value}, e.g. a dead band.
    """
    title: str
    ylabel: str
    lines: dict
    styles: dict = field(default_factory=dict)
    kind: str = "time"
    xlabel: str = "time [s]"
    hlines: dict = field(default_factory=dict)


class Dashboard:
    """Overview + 3D close-up + panels of logged signals; control runs in a worker thread.

    robot     the connected Robot (its arm model, workspace and live state are shown)
    panels    Panel objects, stacked on the right
    paths     {legend label: (x column, y column, z column[, style dict])}: 3D paths
              drawn from logged columns in mm, e.g. the TCP path followed
    window    seconds of history shown while running (finish() shows everything)
    fps       redraws per second while running
    window, fps and the folder of the saved figures default to the plots:
    section of ur10_config.yaml.

    dash.scene is the close-up: it zooms automatically on its frames and paths,
    the logged paths and the TCP. dash.overview shows the whole arm.
    """

    def __init__(self, robot, title, panels=(), paths=None, window=None, fps=None):
        self.robot, self.title, self.panels = robot, title, list(panels)
        self.settings = robot.config["plots"]
        self.window = self.settings["window"] if window is None else window
        self._period = 1.0 / (self.settings["fps"] if fps is None else fps)
        self.paths = dict(paths or {})
        self._lock = threading.Lock()
        self._t, self._cols = [], {}     # logged rows: times and {column: values}
        self._pending = {}               # scene updates requested by the worker, applied when drawing
        self._status = ""
        self._legends = {}               # axes -> labels in its legend
        plt.ion()
        self.fig = plt.figure(title, figsize=(16, 9))
        self.fig.suptitle(title, fontsize=13, fontweight="bold")
        outer = self.fig.add_gridspec(1, 2, width_ratios=(1.0, 1.05), left=0.01, right=0.87, top=0.93,
                                      bottom=0.07, wspace=0.12)
        left = outer[0].subgridspec(2, 1, height_ratios=(1, 2.1), hspace=0.08)
        self._grid = outer
        self._right = outer[1].subgridspec(max(len(self.panels), 1), 1, hspace=0.55)
        tcp = robot.tcp_pose()[:3, 3]
        self.overview = Scene(self.fig.add_subplot(left[0], projection="3d"), robot.arm, robot.workspace,
                              extent=0.9, center=0.5 * tcp + np.array([0.0, 0.0, 0.15]),
                              title="Overview (UR base frame)", tcp_label=False, compact=True)
        self.scene = Scene(self.fig.add_subplot(left[1], projection="3d"), robot.arm, robot.workspace,
                           extent=0.15, center=tcp, title="Close-up of the task (zooms to fit)", clip=True,
                           tcp_label=False, base_frame=False)
        self._axes, self._lines = [], []
        for i, panel in enumerate(self.panels):
            ax = self.fig.add_subplot(self._right[i])
            ax.set_title(panel.title, fontsize=10, loc="left")
            ax.set_xlabel(panel.xlabel, fontsize=9)
            ax.set_ylabel(panel.ylabel, fontsize=9)
            ax.grid(alpha=0.3)
            ax.tick_params(labelsize=8)
            if panel.kind == "xy":
                ax.set_aspect("equal", adjustable="datalim")
            lines = {label: ax.plot([], [], label=label, **{"lw": 1.4, **panel.styles.get(label, {})})[0]
                     for label in panel.lines}
            for label, value in panel.hlines.items():
                ax.axhline(value, color="0.4", ls=":", lw=1.2, label=label)
            ax.legend(fontsize=8, loc="upper left", bbox_to_anchor=(1.01, 1.0), borderaxespad=0)
            self._axes.append(ax)
            self._lines.append(lines)
        self._status_text = self.fig.text(0.015, 0.015, "", fontsize=10, family="monospace", va="bottom")

    # --- called from the control (worker) thread; they only store data
    def record(self, t, values=None, **columns):
        """Log one row at time t [s]: columns given as keywords or a dict (numbers; lengths in mm)."""
        row = dict(values or {}, **columns)
        with self._lock:
            n = len(self._t)
            self._t.append(float(t))
            for key, value in row.items():
                column = self._cols.get(key)
                if column is None:
                    column = self._cols[key] = [math.nan] * n
                column.append(float(value))
            for column in self._cols.values():
                if len(column) == n:     # not in this row
                    column.append(math.nan)

    def set_status(self, text):
        """One line of text shown under the plots."""
        self._status = text

    def set_frame(self, name, T, label=None):
        """Show (or move) a highlighted frame in the close-up, e.g. the current target."""
        self._pending[name] = ("frame", np.array(T, dtype=float), label)

    def set_marker(self, name, point, color="tab:orange", label=None):
        """Show (or move) a point [m] in the close-up, e.g. the moving reference."""
        self._pending[name] = ("marker", np.array(point, dtype=float), color, label)

    def set_vector(self, name, origin, vector, color="tab:orange", label=None):
        """Show (or move) an arrow [m] in the close-up, e.g. a scaled force."""
        self._pending[name] = ("vector", np.array(origin, dtype=float), np.array(vector, dtype=float), color, label)

    def column(self, key):
        """Logged values of a column (numpy array; NaN where a row did not have it)."""
        with self._lock:
            return np.array(self._cols.get(key, [math.nan] * len(self._t)), dtype=float)

    def times(self):
        with self._lock:
            return np.array(self._t, dtype=float)

    # --- main thread
    def _legend(self, ax, **kwargs):
        labels = tuple(ax.get_legend_handles_labels()[1])
        if labels and labels != self._legends.get(ax):   # something new to explain
            ax.legend(**kwargs)
            self._legends[ax] = labels

    def refresh(self, full=False):
        """Redraw everything: the live arm, pending updates, paths, zoom and panels."""
        with self._lock:
            t = np.array(self._t, dtype=float)
            cols = {k: np.array(v, dtype=float) for k, v in self._cols.items()}
        pending, self._pending = self._pending, {}
        for name, item in pending.items():
            if item[0] == "frame":
                self.scene.set_frame(name, item[1], item[2])
            elif item[0] == "marker":
                self.scene.set_marker(name, item[1], item[2], label=item[3])
            else:
                self.scene.set_vector(name, item[1], item[2], item[3], label=item[4])
        focus = list(self.scene.points)
        try:
            state = self.robot.state()
            self.scene.draw_arm(state)
            self.overview.draw_arm(state)
            focus.append(state.tcp[:3, 3])
        except Exception:   # connection closed at the end: keep the last drawing
            pass
        nan = np.full(len(t), math.nan)
        every = _stride(len(t))
        for label, spec in self.paths.items():
            xyz = np.c_[tuple(cols.get(c, nan) / 1000.0 for c in spec[:3])][::every]
            style = spec[3] if len(spec) > 3 else {}
            self.scene.set_line(label, xyz, label=label, **style)
            self.overview.set_line(label, xyz, **style)
            xyz = xyz[np.all(np.isfinite(xyz), axis=1)]
            if len(xyz):
                focus.extend((xyz.min(axis=0), xyz.max(axis=0)))
        if focus:   # zoom the close-up on everything of interest, with a margin
            focus = np.array(focus)
            low, high = focus.min(axis=0), focus.max(axis=0)
            self.scene.set_limits(0.5 * (low + high), max(0.6 * np.max(high - low) + 0.03, 0.08))
        self._legend(self.scene.ax, loc="upper left", fontsize=8, bbox_to_anchor=(0.0, 1.0))
        first = 0 if full or not len(t) else int(np.searchsorted(t, t[-1] - self.window))
        step = _stride(len(t) - first)
        for panel, ax, lines in zip(self.panels, self._axes, self._lines):
            for label, spec in panel.lines.items():
                if panel.kind == "xy":   # paths: always whole
                    lines[label].set_data(cols.get(spec[0], nan)[::every], cols.get(spec[1], nan)[::every])
                else:
                    lines[label].set_data(t[first::step], cols.get(spec, nan)[first::step])
            ax.relim()
            ax.autoscale_view()
        self._status_text.set_text(self._status)
        if plt.fignum_exists(self.fig.number):
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    def run(self, control):
        """Run control(stop_event) in a worker thread and keep the window live meanwhile.

        Returns True if control finished by itself, False if it was stopped (Ctrl+C
        here or the window was closed; stop_event is then set and the worker must
        return promptly). Exceptions raised by control are raised again here.
        """
        stop = threading.Event()
        error = []

        def worker():
            try:
                control(stop)
            except BaseException as exc:   # noqa: BLE001 - handed over to the main thread
                error.append(exc)

        switch = sys.getswitchinterval()
        sys.setswitchinterval(0.001)   # the worker gets the interpreter back quickly while plots draw
        thread = threading.Thread(target=worker, name="control", daemon=True)
        thread.start()
        finished = True
        try:
            while thread.is_alive():
                if not plt.fignum_exists(self.fig.number):
                    finished = False
                    break
                started = time.monotonic()
                self.refresh()
                time.sleep(max(0.01, self._period - (time.monotonic() - started)))
        except KeyboardInterrupt:
            finished = False
        finally:
            stop.set()
            thread.join(timeout=5.0)
            sys.setswitchinterval(switch)
        if error:
            raise error[0]
        return finished

    def finish(self, summary=(), save_as=None, show=True):
        """Turn the window into the still summary: all logged data, the summary lines, saved as PNG.

        save_as: file name for the PNG (None: <plots.save_dir>/<title>_<date>.png,
        relative to the repository; not saved if save_dir is empty). Blocks until
        the window is closed when show=True.
        """
        lines = list(summary)
        self._status = "\n".join(lines)
        self._grid.update(bottom=0.07 + 0.022 * max(len(lines) - 1, 0))   # room for the summary
        self.refresh(full=True)
        if save_as is None and self.settings["save_dir"]:
            name = "_".join("".join(c if c.isalnum() else " " for c in self.title.lower()).split())
            save_as = ROOT / self.settings["save_dir"] / f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.png"
        if save_as is not None:
            try:
                Path(save_as).parent.mkdir(parents=True, exist_ok=True)
                self.fig.savefig(save_as, dpi=120)
                print(f"Summary figure saved to {save_as}")
            except OSError as exc:
                print(f"Could not save the summary figure: {exc}")
        if show and plt.fignum_exists(self.fig.number):
            print("Close the plot window to exit.")
            plt.ioff()
            plt.show()


def _stride(n, max_points=4000):
    """Decimation step that keeps at most ~max_points points for drawing."""
    return max(1, n // max_points)
