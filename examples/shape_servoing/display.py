"""The live window of example 4 (matplotlib with blitting).

Top left, the image space: the camera image with the tracked markers (blue
dots; hollow while the Kalman filter predicts a marker that was not
detected), the edges (orange arrows from each marker to the next) and the
curvature (green circles whose radius grows with the angle; filled when the
chain turns clockwise), plus the target: squares, dashed polyline and dashed
circles. With the features in the camera frame (3D), the image shares the
space with a 3D view of the chain, its curvature, the target and the recent
trajectory of every marker (drag it to rotate).

Under it: the workflow buttons (numbered in their usual order), the robot
buttons (start configuration, home, gripper), the run logger toggle, STOP,
the feature space and frame selectors, the joint jog buttons and a status card.

Right, time plots of the active frame: the three feature spaces (blue,
orange, green; one shade per marker, light to dark with increasing ID;
target values as triangles on the right edge), the joint velocities, the
feature error of the active feature set with its norm, and the condition
number of the Jacobian estimates.

Speed: the static parts (axes, ticks, labels, buttons) are drawn once into a
background; each frame only the changing artists are drawn over it and copied
to the screen (blitting). The time axes show the last `window` seconds up to
"now" = 0, so they never scroll; the y limits (and the 3D view limits) are
checked a few times per second, and only a change of limits (or of the
markers shown) redraws the whole window.
"""

import math
import sys
import threading
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, FancyArrowPatch
from matplotlib.widgets import Button, RadioButtons

from shape_servoing import features as feat

JOG_KEYS = {"q": (0, 1), "a": (0, -1), "w": (1, 1), "s": (1, -1), "e": (2, 1), "d": (2, -1),
            "r": (3, 1), "f": (3, -1), "t": (4, 1), "g": (4, -1), "y": (5, 1), "h": (5, -1)}
ACTION_KEYS = {"enter": ("record",), "b": ("start",), "p": ("probe",), "v": ("servo",), "i": ("set_start",),
               "n": ("home",), "o": ("gripper", "open"), "c": ("gripper", "close"), " ": ("stop",),
               "escape": ("stop",)}
KEY_HELP = ("Keys in [ ].  Jog: hold Q/A W/S E/D R/F T/G Y/H (J1..J6), -/+ jog speed.  "
            "1/2/3: feature space, M: 2D/3D, L: log runs.")

# Look: a light, flat theme (applied through rcParams when the window is created)
THEME = {
    "figure.facecolor": "#f3f5f8", "axes.facecolor": "white", "axes.edgecolor": "#c3cad4",
    "axes.spines.top": False, "axes.spines.right": False, "axes.titlesize": 9.5, "axes.titleweight": "semibold",
    "axes.titlecolor": "#27313d", "axes.titlelocation": "left", "axes.labelsize": 8, "axes.labelcolor": "#4a5563",
    "xtick.color": "#6b7480", "ytick.color": "#6b7480", "xtick.labelsize": 7, "ytick.labelsize": 7,
    "grid.color": "#e4e8ee", "legend.fontsize": 6.5, "legend.framealpha": 0.85, "legend.edgecolor": "none",
    "legend.handlelength": 1.2, "legend.columnspacing": 0.8, "legend.loc": "upper left",
}
TEXT, MUTED = "#27313d", "#7a8491"
MODE_COLORS = {"idle": "#8a94a3", "recording": "#3b7fc4", "moving": "#8663c2", "probing": "#e0872b",
               "servoing": "#35a150"}
JOINT_COLORS = plt.get_cmap("tab10").colors[:6]
BLUE, ORANGE, GREEN = (feat.COLORS[s] for s in feat.SPACES)
CURVATURE_FILL = (0.17, 0.63, 0.17, 0.2)
STYLES = ("-", "--", ":")     # components of a marker or edge: u/x solid, v/y dashed, z dotted
POOL = 24                     # most markers drawn
AUTOSCALE_EVERY = 0.4         # s
MAX_POINTS = 400              # points per plotted line (longer histories are decimated)
# Top-left area: the image alone (2D), or the image and the 3D view side by side (3D)
IMAGE_FULL, IMAGE_HALF, VIEW3D = (0.012, 0.44, 0.505, 0.52), (0.012, 0.44, 0.25, 0.52), (0.27, 0.44, 0.247, 0.52)


def to_view(p):
    """Camera frame [m] -> 3D view coordinates [mm]: x right, depth z forward, -y up."""
    p = np.asarray(p, dtype=float).reshape(-1, 3) * 1000.0
    return p[:, 0], p[:, 2], -p[:, 1]


class Display:
    """The window: build it with the session, then run() it (the control loop runs meanwhile)."""

    def __init__(self, session, settings, title="Shape servoing"):
        self.session, self.vision = session, session.vision
        d = settings["display"]
        self.window, self.period, self.plot_period = d["window"], 1.0 / d["fps"], 1.0 / d["plot_fps"]
        self.image_mode, self.brightness, self.downsample = d["image"], d["brightness"], d["downsample"]
        self.curvature_scale, self.min_radius = d["curvature_scale"], d["min_radius"]
        self.tolerances = {"image": settings["features"]["tolerance"], "camera": settings["features"]["tolerance_3d"]}
        self.title = title
        obs = self.vision.latest()
        self.height, self.width = obs.image.shape[:2]
        self.fps = 0.0
        self._layout = None              # (ids, space, frame, target ids) the plot lines were built for
        self._frame_shown = None         # frame the top-left area is arranged for
        self._full = True                # the next frame redraws everything (new background)
        self._backgrounds = {}
        self._next_autoscale = self._next_plots = 0.0
        self._scaled = set()             # plots whose y limits were set from their data
        self._lines = {}                 # plot name -> {column: Line2D}
        self._target_specs = {}          # feature space -> colours of the target triangles
        self._trails = {}                # marker id -> (x, y, z) recent 3D path [m], camera frame

        plt.rcParams.update(THEME)
        self.fig = plt.figure(title, figsize=(15, 8.6))
        manager = self.fig.canvas.manager
        if manager is not None and getattr(manager, "key_press_handler_id", None) is not None:
            self.fig.canvas.mpl_disconnect(manager.key_press_handler_id)   # no matplotlib shortcuts (s, q, ...)
        self._build_image()
        self._build_view3d()
        self._build_plots()
        self._build_controls()
        canvas = self.fig.canvas
        self._draw_cid = canvas.mpl_connect("draw_event", self._on_draw)
        canvas.mpl_connect("key_press_event", self._on_key_press)
        canvas.mpl_connect("key_release_event", self._on_key_release)
        canvas.mpl_connect("button_press_event", self._on_button_press)
        canvas.mpl_connect("button_release_event", lambda _event: self.session.jog.release_button())
        try:   # Tk: a key held while the window loses the focus never sends its release
            canvas.get_tk_widget().bind("<FocusOut>", lambda _event: self.session.jog.clear(), add="+")
        except AttributeError:
            pass

    # --- building ------------------------------------------------------------------------------
    def _build_image(self):
        ax = self.ax_image = self.fig.add_axes(IMAGE_FULL)
        w, h = self.width, self.height
        extent = (-0.5, w - 0.5, h - 0.5, -0.5)
        ax.set_facecolor("white" if self.image_mode == "none" else "black")
        for spine in ax.spines.values():
            spine.set_visible(False)
        self._image = None
        if self.image_mode != "none":
            shape = (len(range(0, h, self.downsample)), len(range(0, w, self.downsample)))
            if self.image_mode == "gray":
                self._image = ax.imshow(np.zeros(shape, np.uint8), cmap="gray", vmin=0,
                                        vmax=255 / max(self.brightness, 0.05), extent=extent,
                                        interpolation="nearest", animated=True)
            else:
                self._image = ax.imshow(np.zeros(shape + (3,), np.uint8), extent=extent,
                                        interpolation="nearest", animated=True)
        ax.set_xlim(-0.5, w - 0.5)
        ax.set_ylim(h - 0.5, -0.5)
        ax.set_aspect("equal")
        ax.set_title("Image space  [px]", fontsize=10.5)
        self._error_lines = ax.add_collection(LineCollection([], colors="0.85", linewidths=1.0, linestyles=":",
                                                             animated=True))
        self._target_poly, = ax.plot([], [], "--", color=ORANGE, lw=1.6, animated=True)
        self._target_circles = [ax.add_patch(Circle((0, 0), 1, fill=False, ec=GREEN, lw=1.4, ls="--",
                                                    animated=True, visible=False)) for _ in range(POOL)]
        self._target_dots, = ax.plot([], [], "s", mfc="none", mec=BLUE, mew=1.8, ms=12, ls="none", animated=True)
        self._arrows = [ax.add_patch(FancyArrowPatch((0, 0), (1, 1), arrowstyle="-|>", mutation_scale=16, lw=2.2,
                                                     color=ORANGE, shrinkA=7, shrinkB=7, animated=True,
                                                     visible=False)) for _ in range(POOL)]
        self._circles = [ax.add_patch(Circle((0, 0), 1, ec=GREEN, lw=2.0, animated=True, visible=False))
                         for _ in range(POOL)]
        self._others, = ax.plot([], [], "o", color="0.65", ms=6, ls="none", animated=True)
        self._coasting, = ax.plot([], [], "o", mfc="none", mec=BLUE, mew=2.2, ms=10, ls="none", animated=True)
        self._detected, = ax.plot([], [], "o", color=BLUE, mec="white", mew=1.0, ms=9, ls="none", animated=True)
        self._labels = [ax.text(0, 0, "", color="white", fontsize=8, fontweight="bold", clip_on=True, animated=True,
                                visible=False, bbox={"boxstyle": "round,pad=0.2", "fc": "black", "alpha": 0.5,
                                                     "ec": "none"})
                        for _ in range(POOL)]
        self._info = ax.text(0.012, 0.985, "", transform=ax.transAxes, va="top", fontsize=8.5, family="monospace",
                             color="white", animated=True,
                             bbox={"boxstyle": "round,pad=0.4", "fc": "black", "alpha": 0.55, "ec": "none"})
        legend = ax.legend(handles=[
            Line2D([], [], marker="o", color=BLUE, mec="white", ls="none", label="marker"),
            Line2D([], [], marker="o", mfc="none", mec=BLUE, mew=2, ls="none", label="predicted (not detected)"),
            Line2D([], [], color="0.65", marker="o", ls="none", label="not in the shape"),
            Line2D([], [], color=ORANGE, lw=2, marker=">", label="edge"),
            Line2D([], [], marker="o", mfc=CURVATURE_FILL, mec=GREEN, ms=10, ls="none",
                   label="curvature (filled: clockwise)"),
            Line2D([], [], marker="s", mfc="none", mec=BLUE, ls="--", color=ORANGE, label="target"),
        ], loc="lower left", fontsize=7, framealpha=0.7)
        legend.set_animated(True)              # drawn over the image, which is redrawn every frame
        self._image_legend = legend
        self._image_artists = ([] if self._image is None else [self._image]) + [
            self._error_lines, self._target_poly, *self._target_circles, self._target_dots, *self._arrows,
            *self._circles, self._others, self._coasting, self._detected, *self._labels, self._info, legend]

    def _build_view3d(self):
        ax = self.ax_3d = self.fig.add_axes(VIEW3D, projection="3d")
        ax.set_visible(False)
        ax.set_facecolor(THEME["figure.facecolor"])
        ax.set_title("Camera frame  [mm, up to scale]", fontsize=10.5)
        ax.set_xlabel("x  (right)", fontsize=7, labelpad=-4)
        ax.set_ylabel("z  (depth)", fontsize=7, labelpad=-4)
        ax.set_zlabel("-y  (up)", fontsize=7, labelpad=-4)
        ax.tick_params(labelsize=6, pad=-2)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=18, azim=-62)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.set_pane_color((0.955, 0.965, 0.975, 1.0))
        shades = plt.get_cmap("Blues")(np.linspace(0.35, 0.9, POOL))
        self._trail_lines = [ax.plot([], [], [], color=shades[k], lw=1.0, alpha=0.8, animated=True)[0]
                             for k in range(POOL)]
        self._target3d, = ax.plot([], [], [], "--s", color=ORANGE, lw=1.4, mfc="none", mec=BLUE, mew=1.5, ms=8,
                                  animated=True)
        self._chain3d, = ax.plot([], [], [], "-", color=ORANGE, lw=2.4, animated=True)
        self._curv3d = [ax.plot([], [], [], "o", mfc=CURVATURE_FILL, mec=GREEN, mew=1.8, ls="none", animated=True)[0]
                        for _ in range(POOL)]
        self._markers3d, = ax.plot([], [], [], "o", color=BLUE, mec="white", ms=7, ls="none", animated=True)
        self._labels3d = [ax.text(0, 0, 0, "", fontsize=7, fontweight="bold", color=TEXT, animated=True,
                                  visible=False) for _ in range(POOL)]
        self._view_limits = None         # (centre, half width) [mm] of the 3D view
        self._view3d_artists = [*self._trail_lines, self._target3d, self._chain3d, *self._curv3d, self._markers3d,
                                *self._labels3d]

    def _build_plots(self):
        grid = self.fig.add_gridspec(3, 2, left=0.565, right=0.992, top=0.955, bottom=0.06, wspace=0.26, hspace=0.45)
        cells = {"position": (0, 0), "edges": (1, 0), "curvature": (2, 0), "joints": (0, 1), "error": (1, 1),
                 "cond": (2, 1)}
        self.axes = {}
        for name, (row, col) in cells.items():
            ax = self.fig.add_subplot(grid[row, col])
            ax.set_xlim(-self.window, 0.04 * self.window)
            ax.axvline(0, color="#c3cad4", lw=0.8)
            ax.grid(True)
            if row == 2:
                ax.set_xlabel("time relative to now [s]")
            self.axes[name] = ax
            self._lines[name] = {}
        self.axes["joints"].set_title("Joint velocities (dotted: commanded)")
        self.axes["joints"].set_ylabel("[deg/s]")
        self.axes["cond"].set_title("Condition number of the Jacobian")
        self.axes["cond"].set_yscale("log")
        self.axes["error"].axhline(0, color="#9aa3ae", lw=0.8)
        self._targets = {space: self.axes[space].scatter([], [], marker="<", s=45, animated=True, zorder=5,
                                                         edgecolors="black", linewidths=0.5)
                         for space in feat.SPACES}
        self._tolerance_line, = self.axes["error"].plot([], [], ":", color="black", lw=1.2, animated=True)
        self._error_hint = self.axes["error"].text(0.5, 0.5, "record a target (Enter) to see the error",
                                                   transform=self.axes["error"].transAxes, ha="center", fontsize=8,
                                                   color=MUTED)
        for j in range(6):   # joint velocity lines never change
            self._add_line("joints", f"qd{j + 1}", JOINT_COLORS[j], "-", 1.3)
            self._add_line("joints", f"qc{j + 1}", JOINT_COLORS[j], ":", 1.3)
        self.axes["joints"].legend(handles=[Line2D([], [], color=c, lw=2, label=f"J{j + 1}")
                                            for j, c in enumerate(JOINT_COLORS)], ncol=6)

    def _button(self, rect, label, action, value=None, color="#e6e9ee", hover="#f1f3f6", text=TEXT, bold=False):
        """A flat button that sends session.request(action[, value]) when clicked (action None: no click)."""
        ax = self.fig.add_axes(rect)
        for spine in ax.spines.values():
            spine.set_visible(False)
        button = Button(ax, label, color=color, hovercolor=hover)
        button.label.set(fontsize=9, color=text, fontweight="bold" if bold else "normal")
        if action is not None:
            button.on_clicked(lambda _event: self.session.request(action, value))
        self._widgets.append(button)
        return button

    def _header(self, x, y, text):
        self.fig.text(x, y, text, fontsize=8, fontweight="semibold", color=MUTED)

    def _radio(self, rect, labels, active, colors, callback):
        ax = self.fig.add_axes(rect)
        ax.set_facecolor(THEME["figure.facecolor"])
        for spine in ax.spines.values():
            spine.set_visible(False)
        n = len(labels)
        radio = RadioButtons(ax, labels, active=active,
                             label_props={"color": colors, "fontsize": [9.5] * n, "fontweight": ["semibold"] * n},
                             radio_props={"s": [60] * n, "edgecolor": colors}, activecolor="#27313d")
        radio.on_clicked(callback)
        self._widgets.append(radio)
        return radio

    def _build_controls(self):
        self._widgets = []
        x0, width, gap = 0.012, 0.505, 0.006
        w4, w5 = (width - 3 * gap) / 4, (width - 4 * gap) / 5

        self._header(x0, 0.4, "WORKFLOW")
        for k, (label, action, color, hover) in enumerate((
                ("1   Record target  [Enter]", "record", "#d6e6f7", "#e6f0fa"),
                ("2   Back to start  [B]", "start", "#e1e5eb", "#eef1f4"),
                ("3   Probe Jacobian  [P]", "probe", "#fbe2c8", "#fdeedd"),
                ("4   Start control  [V]", "servo", "#d3ecd6", "#e4f4e6"))):
            self._button((x0 + k * (w4 + gap), 0.345, w4, 0.047), label, action, color=color, hover=hover)
        for k, (label, action, value) in enumerate((
                ("Start = here  [I]", "set_start", None), ("Home  [N]", "home", None),
                ("Open gripper  [O]", "gripper", "open"), ("Close gripper  [C]", "gripper", "close"))):
            color, hover = ("#e8e1f4", "#f1ecf8") if action == "gripper" else ("#eceff3", "#f5f7f9")
            self._button((x0 + k * (w5 + gap), 0.292, w5, 0.043), label, action, value, color=color, hover=hover)
        self._log_button = self._button((x0 + 4 * (w5 + gap), 0.292, w5, 0.043), "", None)
        self._log_button.on_clicked(lambda _event: self._toggle_logging())
        self._show_logging()
        self._button((x0, 0.24, width, 0.043), "STOP   [Space / Esc]", "stop", color="#e0524a", hover="#ea7770",
                     text="white", bold=True)

        status = self.session.status
        self._header(x0, 0.207, "FEATURE SPACE")
        self._space_radio = self._radio((x0, 0.108, 0.09, 0.092),
                                        [f"{s}  [{k + 1}]" for k, s in enumerate(feat.SPACES)],
                                        feat.SPACES.index(status.space), [feat.COLORS[s] for s in feat.SPACES],
                                        lambda label: self.session.request("space", label.split()[0]))
        self._header(0.108, 0.207, "FRAME  [M]")
        self._frame_radio = self._radio((0.108, 0.125, 0.085, 0.075), ["2D  image", "3D  camera"],
                                        feat.FRAMES.index(status.frame), [TEXT, TEXT],
                                        lambda label: self.session.request(
                                            "frame", "image" if label.startswith("2D") else "camera"))

        jx, jw = 0.2, 0.317                     # jog block: 6 joint columns + the speed column
        step = jw / 7
        self._header(jx, 0.207, "JOINT JOG  (hold)")
        self._jog_axes = {}
        for j in range(6):
            for sign, y in ((1, 0.158), (-1, 0.108)):
                b = self._button((jx + j * step, y, step - gap, 0.043), f"J{j + 1}  {'+' if sign > 0 else '-'}",
                                 None, color="#eceff3", hover="#f7f8fa", text=JOINT_COLORS[j], bold=True)
                self._jog_axes[b.ax] = (j, sign)
        for sign, y in ((1, 0.158), (-1, 0.108)):
            b = self._button((jx + 6 * step, y, step - gap, 0.043), f"speed {'+' if sign > 0 else '-'}",
                             "jog_speed", sign)
            b.label.set_fontsize(8)

        ax = self.ax_status = self.fig.add_axes((x0, 0.006, width, 0.088))
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_color("#dde2e9")
        self._mode_text = ax.text(0.01, 0.8, "", fontsize=8.5, fontweight="bold", color="white", va="center",
                                  animated=True, bbox={"boxstyle": "round,pad=0.35", "fc": MODE_COLORS["idle"],
                                                       "ec": "none"})
        self._status_head = ax.text(0.13, 0.8, "", fontsize=8.5, color=TEXT, va="center", animated=True)
        self._status_text = ax.text(0.01, 0.58, "", fontsize=8, color=TEXT, va="top", family="monospace",
                                    animated=True)
        self._help_text = self.fig.text(x0 + width, 0.4, KEY_HELP, fontsize=7, color=MUTED, ha="right")

    def _add_line(self, plot, column, color, style, width, alpha=1.0):
        line, = self.axes[plot].plot([], [], color=color, ls=style, lw=width, alpha=alpha, animated=True)
        self._lines[plot][column] = line

    # --- run logger toggle ---------------------------------------------------------------------
    def _toggle_logging(self):
        logger = self.session.logger
        if logger is not None:
            logger.enabled = not logger.enabled      # applies from the next servo run
        self._show_logging()

    def _show_logging(self):
        logger, button = self.session.logger, self._log_button
        on = logger is not None and logger.enabled
        button.label.set_text("Log runs: " + ("on" if on else "off" if logger is not None else "n/a") + "  [L]")
        button.color, button.hovercolor = ("#f6d9d7", "#fae6e4") if on else ("#eceff3", "#f5f7f9")
        button.ax.set_facecolor(button.color)
        self.fig.canvas.draw_idle()

    # --- plot contents that depend on the markers, the target, the space and the frame -------------
    @staticmethod
    def _groups(ids, space, frame):
        """(shades, components per group, group labels) of a feature space."""
        ids = list(ids)
        labels = {"position": [str(i) for i in ids],
                  "edges": [f"{a}->{b}" for a, b in zip(ids, ids[1:])],
                  "curvature": [f"at {b}" for b in ids[1:-1]]}[space]
        shades = plt.get_cmap(feat.COLORMAPS[space])(np.linspace(0.45, 0.95, max(len(labels), 1)))
        return shades, (1 if space == "curvature" else feat.dims(frame)), labels

    def _feature_specs(self, ids, space, frame, prefix):
        """[(column, colour, line style)] of the components of a feature set."""
        shades, per_group, _ = self._groups(ids, space, frame)
        key = feat.key(space, frame)
        return [(f"{prefix}{key}:{name}", shades[k // per_group], STYLES[k % per_group])
                for k, name in enumerate(feat.names(ids, space, frame))]

    def _arrange(self, frame):
        """Image alone (2D) or image and 3D view side by side (3D)."""
        self._frame_shown = frame
        self.ax_image.set_position(IMAGE_FULL if frame == "image" else IMAGE_HALF)
        self.ax_3d.set_visible(frame == "camera")
        self._image_legend.set_visible(frame == "image")     # too big for the half-width image
        self._view_limits = None
        self._full = True

    def _sync_layout(self, status):
        target_ids = status.target.ids if status.target is not None else None
        layout = (status.ids, status.space, status.frame, target_ids)
        if layout == self._layout:
            return
        self._layout = layout
        if status.frame != self._frame_shown:
            self._arrange(status.frame)
        frame, key = status.frame, status.key
        for name in (*feat.SPACES, "error", "cond"):
            for line in self._lines[name].values():
                line.remove()
            self._lines[name] = {}
        styles = "solid/dashed" if frame == "image" else "solid/dashed/dotted"
        titles = {"position": f"Position features   {'u/v' if frame == 'image' else 'x/y/z'}: {styles}",
                  "edges": f"Edge features   {'x/y' if frame == 'image' else 'x/y/z'}: {styles}",
                  "curvature": "Curvature features" + ("  (unsigned in 3D)" if frame == "camera" else "")}
        for space in feat.SPACES:
            ax = self.axes[space]
            ax.set_title(titles[space])
            ax.set_ylabel(f"[{feat.unit(feat.key(space, frame))}]")
            specs = self._feature_specs(status.ids, space, frame, "") if feat.size(len(status.ids), space) > 0 else []
            for column, color, style in specs:
                self._add_line(space, column, color, style, 1.3)
            self._target_specs[space] = [color for _, color, _ in specs] if target_ids is not None else []
            shades, _, labels = self._groups(status.ids, space, frame)
            legend = ax.get_legend()
            if legend is not None:
                legend.remove()
            if labels and specs:
                ax.legend(handles=[Line2D([], [], color=c, lw=2, label=lab) for c, lab in zip(shades, labels)],
                          ncol=min(len(labels), 4))
        if target_ids is not None:
            for column, color, style in self._feature_specs(target_ids, status.space, frame, "error:"):
                self._add_line("error", column, color, style, 1.0)
            self._add_line("error", f"error_norm:{key}", "black", "-", 2.2)
        self._error_hint.set_visible(target_ids is None)
        self.axes["error"].set_title(f"Error s - s* ({key}); black: norm", color=feat.COLORS[status.space])
        self.axes["error"].set_ylabel(f"[{feat.unit(key)}]")
        for space in feat.SPACES:
            active = space == status.space
            self._add_line("cond", f"cond:{feat.key(space, frame)}", feat.COLORS[space], "-", 2.2 if active else 1.0,
                           1.0 if active else 0.55)
        self.axes["cond"].legend(handles=[Line2D([], [], color=feat.COLORS[s], lw=2, label=feat.key(s, frame))
                                          for s in feat.SPACES], ncol=3)
        for radio, value in ((self._space_radio, feat.SPACES.index(status.space)),
                             (self._frame_radio, feat.FRAMES.index(status.frame))):
            if radio.index_selected != value:       # changed elsewhere: show it
                radio.eventson = False
                radio.set_active(value)
                radio.eventson = True
        self._scaled.clear()
        self._full = True

    # --- per frame -------------------------------------------------------------------------------
    def _update_image(self, obs, status):
        if self._image is not None:
            image = obs.image[::self.downsample, ::self.downsample]
            if self.image_mode == "gray":
                self._image.set_data(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
            else:
                rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                self._image.set_data(rgb if self.brightness >= 1 else cv2.convertScaleAbs(rgb, alpha=self.brightness))
        markers, ids = obs.markers, list(status.ids)
        used = [i for i in ids if i in markers]
        detected = [markers[i] for i in used if i in obs.detected]
        coasting = [markers[i] for i in used if i not in obs.detected]
        others = [markers[i] for i in sorted(markers) if i not in ids]
        for line, points in ((self._detected, detected), (self._coasting, coasting), (self._others, others)):
            line.set_data([p[0] for p in points], [p[1] for p in points])
        # edges and curvature between consecutive markers of the feature set that are tracked now
        pairs = [(markers[a], markers[b]) for a, b in zip(ids, ids[1:]) if a in markers and b in markers]
        triples = [(markers[a], markers[b], markers[c]) for a, b, c in zip(ids, ids[1:], ids[2:])
                   if a in markers and b in markers and c in markers]
        self._set_arrows(pairs)
        self._set_circles(self._circles, [(b, feat.turning_angles(np.array([b - a, c - b]))[0])
                                          for a, b, c in triples], filled=True)
        target = status.target
        if target is not None:
            points = target.points
            self._target_dots.set_data(points[:, 0], points[:, 1])
            self._target_poly.set_data(points[:, 0], points[:, 1])
            self._set_circles(self._target_circles, list(zip(points[1:-1], target.features("curvature"))))
            self._error_lines.set_segments([(markers[i], p) for i, p in zip(target.ids, points) if i in markers])
        else:
            for artist in (self._target_dots, self._target_poly):
                artist.set_data([], [])
            self._set_circles(self._target_circles, [])
            self._error_lines.set_segments([])
        tracked = sorted(markers)
        for k, label in enumerate(self._labels):
            if k < len(tracked):
                u, v = markers[tracked[k]]
                label.set_position((u + 9, v - 9))
                label.set_text(str(tracked[k]))
                label.set_visible(True)
            else:
                label.set_visible(False)
        logger = self.session.logger
        line = (("● REC | " if logger is not None and logger.active else "")
                + f"{status.mode.upper()} | {status.key}" + (" | " if self._frame_shown == "image" else "\n")
                + f"camera {obs.fps:4.1f} fps | window {self.fps:4.1f} fps | control {status.rate:4.1f} Hz")
        if self.vision.error:
            line += f"\n{self.vision.error}"
        self._info.set_text(line)
        self._info.get_bbox_patch().set_facecolor("#b3261e" if logger is not None and logger.active else "black")

    def _set_arrows(self, pairs):
        for k, arrow in enumerate(self._arrows):
            if k < len(pairs):
                arrow.set_positions(tuple(pairs[k][0]), tuple(pairs[k][1]))
                arrow.set_visible(True)
            else:
                arrow.set_visible(False)

    def _set_circles(self, circles, centres_angles, filled=False):
        for k, circle in enumerate(circles):
            if k < len(centres_angles):
                centre, angle = centres_angles[k]
                circle.set_center(tuple(centre))
                circle.set_radius(self.min_radius + self.curvature_scale * abs(angle))
                if filled:
                    circle.set_facecolor(CURVATURE_FILL if angle > 0 else "none")
                circle.set_visible(True)
            else:
                circle.set_visible(False)

    def _update_view3d(self, obs, status):
        """The chain, its curvature, the target and the marker trails in the 3D view (camera frame)."""
        markers, ids = obs.markers3d, list(status.ids)
        chain = np.array([markers[i] if i in markers else (np.nan,) * 3 for i in ids]).reshape(-1, 3)
        self._chain3d.set_data_3d(*to_view(chain))
        self._markers3d.set_data_3d(*to_view(chain))
        for k, dot in enumerate(self._curv3d):   # dot size grows with the (unsigned) angle
            if 0 < k + 1 < len(ids) - 1 and np.all(np.isfinite(chain[k:k + 3])):
                angle = feat.turning_angles(np.diff(chain[k:k + 3], axis=0))[0]
                dot.set_data_3d(*to_view(chain[k + 1]))
                dot.set_markersize(6 + 0.2 * self.curvature_scale * angle)
                dot.set_visible(True)
            else:
                dot.set_visible(False)
        target = status.target
        points = target.points3d if target is not None and target.points3d is not None else np.empty((0, 3))
        self._target3d.set_data_3d(*to_view(points))
        for k, line in enumerate(self._trail_lines):
            trail = self._trails.get(ids[k]) if k < len(ids) else None
            line.set_data_3d(*(to_view(trail) if trail is not None else ([], [], [])))
        for k, label in enumerate(self._labels3d):
            if k < len(ids) and np.all(np.isfinite(chain[k])):
                x, y, z = (v[0] for v in to_view(chain[k]))
                label.set_position_3d((x, y, z + 8))
                label.set_text(str(ids[k]))
                label.set_visible(True)
            else:
                label.set_visible(False)

    def _update_plots(self, status):
        t, data, columns = self.session.recorder.tail(self.window + 1.0)
        step = max(1, len(t) // MAX_POINTS)
        t, data = t[::-1][::step][::-1], data[::-1][::step][::-1]    # decimated, keeping the newest row
        x = t - (time.monotonic() - self.session.t0)
        for lines in self._lines.values():
            for column, line in lines.items():
                index = columns.get(column)
                if index is None:
                    line.set_data([], [])
                else:
                    line.set_data(x, data[:, index])
        if status.frame == "camera":             # recent 3D path of every marker [m]
            self._trails = {}
            for i in status.ids:
                index = [columns.get(f"position3d:{axis}{i}") for axis in "xyz"]
                if None not in index:
                    self._trails[i] = data[:, index] / 1000.0
        target = status.target
        for space, scatter in self._targets.items():
            colors = self._target_specs.get(space, [])
            values = target.features(space, status.frame) if target is not None and colors else None
            if values is None:
                scatter.set_offsets(np.empty((0, 2)))
                continue
            values = feat.to_display(values, feat.key(space, status.frame))
            scatter.set_offsets(np.c_[np.zeros(len(values)), values])
            scatter.set_facecolors(colors)
        if target is not None:
            size = feat.size(len(target.ids), status.space, status.frame)
            tolerance = self.tolerances[status.frame][status.space] * math.sqrt(size)
            self._tolerance_line.set_data([-self.window, 0], [tolerance, tolerance])
        else:
            self._tolerance_line.set_data([], [])

    def _update_status(self, status):
        self._mode_text.set_text(status.mode.upper())
        self._mode_text.get_bbox_patch().set_facecolor(MODE_COLORS[status.mode])
        head = [f"{status.key}: markers {', '.join(map(str, status.ids)) or '-'}", f"jog {status.jog_speed:g} deg/s"]
        if not status.robot:
            head.append("camera only: no robot")
        elif status.gripper:
            head.append(f"gripper {status.gripper}")
        logger = self.session.logger
        if logger is not None and logger.active:
            head.append(f"logging to {logger.run_dir.name}")
        self._status_head.set_text("   |   ".join(head))
        lines = [f"> {status.progress or status.message}", f"Jacobian: {status.jacobian}"]
        if status.progress:
            lines.insert(1, f"  last: {status.message}")
        self._status_text.set_text("\n".join(lines))

    def _autoscale(self):
        """Adapt the y limits (and the 3D view) when the data leaves them or fills a small part of them."""
        for name, ax in self.axes.items():
            values = [line.get_ydata() for line in self._lines[name].values()]
            if name in self._targets:
                values.append(self._targets[name].get_offsets()[:, 1])
            if name == "error":
                values.append(self._tolerance_line.get_ydata())
            y = np.concatenate([np.asarray(v, dtype=float).ravel() for v in values]) if values else np.empty(0)
            y = y[np.isfinite(y)]
            if name == "cond":
                y = np.log10(y[y > 0])
            if not len(y):
                continue
            lo, hi = float(y.min()), float(y.max())
            current = np.log10(ax.get_ylim()) if name == "cond" else np.array(ax.get_ylim())
            span = max(hi - lo, 0.2 if name == "cond" else 1.0)
            first = name not in self._scaled
            if first or lo < current[0] or hi > current[1] or span < 0.3 * (current[1] - current[0]):
                self._scaled.add(name)
                pad, middle = 0.15 * span, (lo + hi) / 2    # span >= hi - lo: at least the minimum range
                limits = (middle - span / 2 - pad, middle + span / 2 + pad)
                ax.set_ylim(*(10 ** np.array(limits) if name == "cond" else limits))
                self._full = True
        if self._frame_shown == "camera":
            self._autoscale_view3d()

    def _autoscale_view3d(self):
        """A cube around the markers, the target and the trails (kept while they stay well inside it)."""
        lines = [self._chain3d, self._target3d, *self._trail_lines]
        points = np.concatenate([np.column_stack(line.get_data_3d()).reshape(-1, 3) for line in lines])
        points = points[np.all(np.isfinite(points), axis=1)]
        if not len(points):
            return
        lo, hi = points.min(axis=0), points.max(axis=0)
        centre, half = (lo + hi) / 2, max(float(np.max(hi - lo)) / 2, 30.0)
        limits = self._view_limits
        if limits is not None:
            inside = np.all(lo >= limits[0] - limits[1]) and np.all(hi <= limits[0] + limits[1])
            if inside and half > 0.4 * limits[1]:
                return
        half *= 1.3
        self._view_limits = (centre, half)
        for setter, c in zip((self.ax_3d.set_xlim, self.ax_3d.set_ylim, self.ax_3d.set_zlim), centre):
            setter(c - half, c + half)
        self._full = True

    def _regions(self, plots=True):
        """(axes, artists drawn over its background): the image space (and 3D view), then plots and status."""
        regions = [(self.ax_image, self._image_artists)]
        if self._frame_shown == "camera":
            regions.append((self.ax_3d, self._view3d_artists))
        if not plots:
            return regions
        regions.append((self.ax_status, [self._mode_text, self._status_head, self._status_text]))
        for name, ax in self.axes.items():
            artists = list(self._lines[name].values())
            if name in self._targets:
                artists.append(self._targets[name])
            if name == "error":
                artists.append(self._tolerance_line)
            regions.append((ax, artists))
        return regions

    def _on_draw(self, _event):
        """After every full draw (ours, a resize, a widget, a 3D rotation): store the new backgrounds."""
        canvas = self.fig.canvas
        self._backgrounds = {ax: canvas.copy_from_bbox(ax.bbox) for ax, _ in self._regions()}
        self._next_plots = 0.0           # the plots were drawn empty: redraw their lines now

    def _blit(self, regions):
        canvas = self.fig.canvas
        for ax, artists in regions:
            background = self._backgrounds.get(ax)
            if background is None:
                continue
            canvas.restore_region(background)
            for artist in artists:
                ax.draw_artist(artist)
            canvas.blit(ax.bbox)

    def frame(self):
        """Draw one frame: the newest image (and 3D view) every time, the plots and the status at plot_fps."""
        obs, status = self.vision.latest(), self.session.status
        self._sync_layout(status)
        now = time.monotonic()
        plots = now >= self._next_plots or self._full or not self._backgrounds
        if plots:
            self._next_plots = now + self.plot_period
            self._update_plots(status)
            self._update_status(status)
        self._update_image(obs, status)
        if self._frame_shown == "camera":
            self._update_view3d(obs, status)
        if plots and now >= self._next_autoscale:
            self._next_autoscale = now + AUTOSCALE_EVERY
            self._autoscale()
        if self._full or not self._backgrounds:
            self._full = False
            self.fig.canvas.draw()       # static parts; _on_draw stores them as the background
        self._blit(self._regions(plots))
        logger = self.session.logger
        if logger is not None and logger.wants_window_frame():
            logger.window_frame(np.asarray(self.fig.canvas.buffer_rgba()).copy())

    # --- input -----------------------------------------------------------------------------------
    def _on_key_press(self, event):
        key = (event.key or "").lower()
        if key in JOG_KEYS:
            self.session.jog.press_key(*JOG_KEYS[key])
        elif key in ("1", "2", "3"):
            self._space_radio.set_active(int(key) - 1)          # its callback sends the request
        elif key == "m":
            self._frame_radio.set_active(1 - self._frame_radio.index_selected)
        elif key == "l":
            self._toggle_logging()
        elif key in ("+", "=", "-"):
            self.session.request("jog_speed", -1 if key == "-" else 1)
        elif key in ACTION_KEYS:
            self.session.request(*ACTION_KEYS[key])

    def _on_key_release(self, event):
        key = (event.key or "").lower()
        if key in JOG_KEYS:
            self.session.jog.release_key(*JOG_KEYS[key])

    def _on_button_press(self, event):
        if event.inaxes in self._jog_axes:
            self.session.jog.press_button(*self._jog_axes[event.inaxes])

    # --- running and summary -----------------------------------------------------------------
    def run(self):
        """Run the session's control loop in a worker thread and keep the window live.

        Returns True if the window was closed, False after Ctrl+C. Exceptions of the
        control loop are raised again here.
        """
        stop, errors = threading.Event(), []

        def worker():
            try:
                self.session.run(stop)
            except BaseException as exc:   # noqa: BLE001 - handed over to the main thread
                errors.append(exc)

        switch = sys.getswitchinterval()
        sys.setswitchinterval(0.001)       # the control thread gets the interpreter back quickly
        thread = threading.Thread(target=worker, name="control", daemon=True)
        thread.start()
        plt.show(block=False)
        closed, last = False, time.monotonic()
        try:
            while thread.is_alive():
                if not plt.fignum_exists(self.fig.number):
                    closed = True
                    break
                started = time.monotonic()
                self.frame()
                self.fig.canvas.flush_events()
                now = time.monotonic()
                self.fps = 0.9 * self.fps + 0.1 / max(now - last, 1e-3)
                last = now
                time.sleep(max(0.002, self.period - (now - started)))
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
            thread.join(timeout=5.0)
            sys.setswitchinterval(switch)
        if errors:
            raise errors[0]
        return closed

    def finish(self, summary, save_as=None, show=True):
        """Turn the window into a still summary of the whole session; save it as PNG.

        The time plots show everything logged (time from the start, targets
        dotted); the controls are replaced by the summary lines.
        """
        self.fig.canvas.mpl_disconnect(self._draw_cid)
        t, data, columns = self.session.recorder.tail()
        end = max(float(t[-1]), 1.0) if len(t) else 1.0
        for name, lines in self._lines.items():
            for column, line in lines.items():
                index = columns.get(column)
                line.set_data(t, data[:, index]) if index is not None else line.set_data([], [])
        _ids, _space, frame, target_ids = self._layout if self._layout is not None else (None, None, "image", None)
        for space, scatter in self._targets.items():
            scatter.set_visible(False)
            for column, color, _style in (self._feature_specs(target_ids, space, frame, "target:")
                                          if target_ids else []):
                if column in columns:
                    self.axes[space].plot(t, data[:, columns[column]], ":", color=color, lw=1.2)
        tolerance = self._tolerance_line.get_ydata()
        if len(tolerance):
            self._tolerance_line.set_data([0, end], tolerance[:2])
        for ax in self.axes.values():
            ax.set_xlim(0, end)
            if ax.get_xlabel():
                ax.set_xlabel("time since the start [s]", fontsize=8)
        self._scaled.clear()
        self._autoscale()
        for widget in self._widgets:
            widget.ax.set_visible(False)
        for text in self.fig.texts:
            text.set_visible(False)
        self.ax_status.set_visible(False)
        self._info.set_visible(False)            # live rates: meaningless in the summary
        self.fig.text(0.015, 0.4, "\n".join(summary), va="top", fontsize=9, family="monospace", color=TEXT)
        for ax, artists in self._regions():
            for artist in artists:
                artist.set_animated(False)
        if save_as is not None:
            try:
                save_as.parent.mkdir(parents=True, exist_ok=True)
                self.fig.savefig(save_as, dpi=110)
                print(f"Summary figure saved to {save_as}")
            except OSError as exc:
                print(f"Could not save the summary figure: {exc}")
        if show and plt.fignum_exists(self.fig.number):
            self.fig.canvas.draw_idle()
            print("Close the plot window to exit.")
            plt.show()
