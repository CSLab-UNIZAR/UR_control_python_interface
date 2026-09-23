"""Live 3D view (matplotlib) of the arm, frames and the TCP path, in the UR base frame.

    view = FrameView(robot.arm, title="Pose control", workspace=robot.workspace)
    view.add_frame(target, "T1")            # static frames, e.g. targets
    while running:
        view.update(robot.state())          # redraws at most `max_fps` times per second
    view.hold()                             # keep the window open at the end

Axes: X red, Y green, Z blue. Needs an interactive matplotlib backend (Tk on
Windows ships with Python; on Ubuntu: sudo apt install python3-tk).
"""

import time

import matplotlib.pyplot as plt
import numpy as np

AXIS_COLORS = ("tab:red", "tab:green", "tab:blue")


class FrameView:
    def __init__(self, arm, title="UR10", workspace=None, trail=True, max_fps=8, extent=0.9,
                 center=(-0.5, 0.15, 0.35)):
        plt.ion()
        self.arm = arm
        self.fig = plt.figure(title, figsize=(8, 7))
        self.ax = self.fig.add_subplot(projection="3d")
        ax = self.ax
        cx, cy, cz = center
        ax.set_xlim(cx - extent, cx + extent)
        ax.set_ylim(cy - extent, cy + extent)
        ax.set_zlim(cz - extent, cz + extent)
        ax.set_box_aspect((1, 1, 1))
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        ax.view_init(elev=22, azim=-125)
        ax.set_title(title)
        self._period = 1.0 / max_fps
        self._last_draw = 0.0
        self._trail = [] if trail else None
        self._vectors = {}
        self._draw_axes(np.eye(4), 0.2, "UR base", lw=2.0)
        if workspace is not None:
            self._draw_box(workspace.limits)
        self._arm_line, = ax.plot([], [], [], "-o", color="0.45", lw=3, ms=4)
        self._trail_line, = ax.plot([], [], [], color="tab:purple", lw=1.2)
        self._tcp_lines = [ax.plot([], [], [], color=c, lw=2.5)[0] for c in AXIS_COLORS]
        self._tcp_text = ax.text(0, 0, 0, "TCP", fontsize=9)

    def _draw_axes(self, T, size, label=None, lw=1.5, style="-"):
        T = np.asarray(T, dtype=float)
        origin = T[:3, 3]
        for i, color in enumerate(AXIS_COLORS):
            tip = origin + size * T[:3, i]
            self.ax.plot(*zip(origin, tip), style, color=color, lw=lw)
        if label:
            self.ax.text(*(origin + size * 0.15), label, fontsize=9)

    def _draw_box(self, limits):
        (x0, x1), (y0, y1), (z0, z1) = limits["x"], limits["y"], limits["z"]
        corners = np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)])
        for i in range(8):
            for j in range(i + 1, 8):
                if np.sum(corners[i] != corners[j]) == 1:   # corners differing in one coordinate = an edge
                    self.ax.plot(*zip(corners[i], corners[j]), color="tab:blue", lw=0.6, alpha=0.4)

    def add_frame(self, T, label=None, size=0.06, style="--"):
        """Draw a fixed frame (e.g. a target pose)."""
        self._draw_axes(T, size, label, lw=1.5, style=style)

    def add_path(self, points, color="0.6", style=":"):
        """Draw a fixed polyline (N x 3 points [m]), e.g. a reference path."""
        points = np.asarray(points, dtype=float)
        self.ax.plot(points[:, 0], points[:, 1], points[:, 2], style, color=color, lw=1.2)

    def set_vector(self, name, origin, vector, color="tab:orange"):
        """Show (or move) an arrow-like segment from `origin` along `vector` [m], e.g. a scaled force."""
        if name not in self._vectors:
            self._vectors[name] = self.ax.plot([], [], [], color=color, lw=3)[0]
        start, end = np.asarray(origin, dtype=float), np.asarray(origin, dtype=float) + np.asarray(vector, dtype=float)
        self._vectors[name].set_data_3d(*zip(start, end))

    def update(self, state, force=False):
        """Redraw the arm, the TCP frame and the trail for a RobotState (throttled unless force=True)."""
        if self._trail is not None:
            self._trail.append(state.tcp[:3, 3].copy())
        now = time.monotonic()
        if not force and now - self._last_draw < self._period:
            return
        self._last_draw = now
        points = self.arm.skeleton(state.q)
        self._arm_line.set_data_3d(points[:, 0], points[:, 1], points[:, 2])
        T = state.tcp
        for i, line in enumerate(self._tcp_lines):
            line.set_data_3d(*zip(T[:3, 3], T[:3, 3] + 0.08 * T[:3, i]))
        self._tcp_text.set_position_3d(T[:3, 3] + 0.02)
        if self._trail:
            trail = np.array(self._trail[-3000:])
            self._trail_line.set_data_3d(trail[:, 0], trail[:, 1], trail[:, 2])
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
