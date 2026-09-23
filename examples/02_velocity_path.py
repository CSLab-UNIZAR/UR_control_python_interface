"""Example 2 - Velocity control: follow a flat circle or an "infinity" shape.

The path lies in a plane through the start point of the TCP: by default the
tool's XY plane (horizontal when the gripper points down), or with --plane base
the horizontal XY plane of the UR base. It starts and ends at the start point,
with a smooth speed ramp at both ends.

25 times per second (--rate) the commanded TCP velocity is

    v = v_ref(t) + kp * (p_ref(t) - p_measured)      feed-forward + P feedback
    w = kr * rotation error to the start orientation   (the orientation is held)

sent with robot.set_tcp_velocity.

Live plots: the arm with the whole reference path, the moving reference point
and the path the TCP really followed; the path in its plane (reference against
measured), the tracking error and the TCP position against the reference. When
the run ends (or Ctrl+C, or closing the window) the window becomes a summary of
the whole run with the RMS and maximum tracking error; it is saved as a PNG in runs/.

Before moving, the example checks that commands reach the arm without delay
(robot.check_command_link: wrist 3 wiggles by 1 deg); --no-link-check skips it.

The defaults are in the examples.velocity_path section of ur10_config.yaml; the
options below override them.

Run from the repository root (robot started, Play pressed, room around the TCP):
    python examples/02_velocity_path.py
    python examples/02_velocity_path.py --shape infinity --size 60 --period 10 --laps 2
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # find ur10api when run from anywhere
from ur10api import ConfigError, Rate, Robot, RobotError, load_config, transforms as tf   # noqa: E402
from ur10api.viz import Dashboard, Panel                                                  # noqa: E402


def shape(name, u, size):
    """Point (2,) and derivative d/du (2,) of the path at phase u [rad], in plane coordinates [m].

    Both shapes pass through (0, 0) at u = 0, i.e. through the start point.
    circle:   radius `size`, centred at (-size, 0)
    infinity: lemniscate of Gerono, 2*size wide and `size` high
    """
    if name == "circle":
        return (size * np.array([np.cos(u) - 1.0, np.sin(u)]),
                size * np.array([-np.sin(u), np.cos(u)]))
    return (size * np.array([np.sin(u), np.sin(u) * np.cos(u)]),
            size * np.array([np.cos(u), np.cos(2 * u)]))


def phase(t, laps, period, ramp):
    """Phase u(t) [rad] and its rate du/dt of a path run `laps` times.

    The phase speeds up smoothly (cosine ramp over `ramp` s), keeps 2*pi/period
    and slows down the same way, ending exactly at u = 2*pi*laps.
    """
    omega = 2 * np.pi / period
    total_phase = 2 * np.pi * laps
    duration = total_phase / omega + ramp
    if t <= 0:
        return 0.0, 0.0
    if t >= duration:
        return total_phase, 0.0
    if t < ramp:                                   # speeding up
        return omega * (t / 2 - ramp / (2 * np.pi) * np.sin(np.pi * t / ramp)), omega * (1 - np.cos(np.pi * t / ramp)) / 2
    if t > duration - ramp:                        # slowing down (mirror of the start)
        u_end, du_end = phase(duration - t, laps, period, ramp)
        return total_phase - u_end, du_end
    return omega * (t - ramp / 2), omega           # cruising


def main():
    cfg = load_config()                        # ur10_config.yaml
    ex = cfg["examples"]["velocity_path"]      # this example's defaults (mm, deg)
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default=cfg["robot"]["host"], help="rosbridge host (default: %(default)s)")
    parser.add_argument("--shape", choices=("circle", "infinity"), default=ex["shape"], help="(default: %(default)s)")
    parser.add_argument("--size", type=float, default=ex["size"], help="radius / half width [mm] (default: %(default)s)")
    parser.add_argument("--period", type=float, default=ex["period"], help="seconds per lap (default: %(default)s)")
    parser.add_argument("--laps", type=int, default=ex["laps"], help="(default: %(default)s)")
    parser.add_argument("--plane", choices=("tool", "base"), default=ex["plane"],
                        help="tool: XY plane of the tool at the start; base: XY plane of the UR base (default: %(default)s)")
    parser.add_argument("--kp", type=float, default=ex["kp"], help="position feedback gain [1/s] (default: %(default)s)")
    parser.add_argument("--rate", type=float, default=ex["rate"], help="control rate [Hz] (default: %(default)s)")
    parser.add_argument("--no-link-check", action="store_true", help="skip the start-up check of the command link")
    args = parser.parse_args()
    size = args.size / 1000   # m
    ramp = min(2.0, args.period / 2)
    duration = args.laps * args.period + ramp

    with Robot(args.host, config=cfg, max_linear_speed=ex["max_speed"] / 1000,
               max_angular_speed=np.radians(ex["max_angular_speed"])) as robot:
        if cfg["link_check"]["enabled"] and not args.no_link_check:
            robot.check_command_link(report=print)
        start = robot.tcp_pose()
        origin = start[:3, 3].copy()
        axes = start[:3, :2] if args.plane == "tool" else np.eye(3)[:, :2]   # plane axes in the base frame

        # The whole path, to check the workspace and the speed before moving
        u_all = np.linspace(0, 2 * np.pi, 400)
        path = np.array([origin + axes @ shape(args.shape, u, size)[0] for u in u_all])
        outside = [p for p in path if not robot.workspace.contains(tf.pose(p))]
        if outside:
            sys.exit(f"The path leaves the workspace ({len(outside)} of {len(path)} points): "
                     "move the robot, reduce --size or widen the workspace in ur10_config.yaml.")
        peak = max(np.linalg.norm(shape(args.shape, u, size)[1]) for u in u_all) * 2 * np.pi / args.period * 1000
        if peak > ex["max_speed"]:
            print(f"Note: the path needs up to {peak:.0f} mm/s but max_speed is {ex['max_speed']:.0f} mm/s: "
                  "expect a tracking lag (longer --period, smaller --size or a higher max_speed).")

        title = f"Velocity control - {args.size:.0f} mm {args.shape}, {args.plane} plane"
        xyz_styles = {"x": {"color": "tab:red"}, "y": {"color": "tab:green"}, "z": {"color": "tab:blue"},
                      "x ref": {"color": "tab:red", "ls": "--", "lw": 1},
                      "y ref": {"color": "tab:green", "ls": "--", "lw": 1},
                      "z ref": {"color": "tab:blue", "ls": "--", "lw": 1}}
        dash = Dashboard(robot, title, panels=[
            Panel(f"Path in the {args.plane} plane (start point at 0, 0)", f"{args.plane} y [mm]",
                  {"reference": ("u_ref", "v_ref"), "measured TCP": ("u", "v")},
                  styles={"reference": {"color": "0.5", "ls": "--"}, "measured TCP": {"color": "tab:purple"}},
                  kind="xy", xlabel=f"{args.plane} x [mm]"),
            Panel("Tracking error |p_ref - p|", "[mm]", {"error": "error"}, styles={"error": {"color": "tab:red"}}),
            Panel("TCP position relative to the start (UR base axes)", "[mm]",
                  {"x": "dx", "y": "dy", "z": "dz", "x ref": "dx_ref", "y ref": "dy_ref", "z ref": "dz_ref"},
                  styles=xyz_styles),
        ], paths={"TCP path": ("x", "y", "z", {"color": "tab:purple"})})
        dash.scene.add_frame(start, "start")
        dash.scene.add_path(path, color="0.35", style="--", label="reference path")

        def control(stop):
            """The path, run in a worker thread while the main thread draws."""
            rate = Rate(args.rate)
            t0 = time.monotonic()
            while not stop.is_set():
                t = time.monotonic() - t0
                u, du = phase(t, args.laps, args.period, ramp)
                point, dpoint = shape(args.shape, u, size)
                p_ref = origin + axes @ point              # reference position [m]
                v_ref = axes @ dpoint * du                  # reference velocity [m/s]
                state = robot.state()
                v = v_ref + args.kp * (p_ref - state.position)
                w = ex["kr"] * tf.pose_error(state.tcp, start)[1]   # hold the start orientation
                robot.set_tcp_velocity(v, w)
                # log in mm: base-frame positions and plane coordinates of the reference and the TCP
                error = np.linalg.norm(p_ref - state.position) * 1000
                d_ref, d = (p_ref - origin) * 1000, (state.position - origin) * 1000   # relative to the start
                plane_ref, plane = d_ref @ axes, d @ axes                            # plane coordinates
                dash.record(t, x=state.position[0] * 1000, y=state.position[1] * 1000, z=state.position[2] * 1000,
                            dx=d[0], dy=d[1], dz=d[2], dx_ref=d_ref[0], dy_ref=d_ref[1], dz_ref=d_ref[2],
                            u_ref=plane_ref[0], v_ref=plane_ref[1], u=plane[0], v=plane[1], error=error)
                dash.set_marker("reference", p_ref, label="moving reference")
                dash.set_status(f"t = {t:5.1f} / {duration:.1f} s   lap {min(int(u / (2 * np.pi)) + 1, args.laps)}"
                                f" of {args.laps}   error {error:5.1f} mm"
                                + (f"   ({robot.last_warning})" if robot.last_warning else ""))
                if t > duration + 1.0:                      # path done, 1 s to settle
                    return
                rate.sleep()

        print(f"Following a {args.size:.0f} mm {args.shape} in the {args.plane} plane, "
              f"{args.laps} laps in {duration:.1f} s. Ctrl+C or closing the window stops.")
        ending = ""
        try:
            if not dash.run(control):
                ending = "stopped by the user"
        except RobotError as exc:
            ending = f"error: {exc}"
        try:
            robot.stop()
        except RobotError:                 # connection lost: nothing to stop
            pass
        if ending:
            print(ending[0].upper() + ending[1:])

    t, error = dash.times(), dash.column("error")
    cruising = (t > ramp) & (t < duration - ramp) & np.isfinite(error)
    summary = [title + (f"  -  {ending}" if ending else "")]
    if np.any(cruising):
        stats = (f"Tracking error while cruising: RMS {np.sqrt(np.mean(error[cruising] ** 2)):.1f} mm, "
                 f"max {error[cruising].max():.1f} mm   (kp {args.kp:g} 1/s, {args.rate:g} Hz, "
                 f"{args.period:g} s per lap)")
        print(stats)
        summary.append(stats)
    dash.finish(summary)


if __name__ == "__main__":
    try:
        main()
    except (RobotError, ConfigError) as exc:   # no connection, stale data, refused motion, bad settings
        sys.exit(f"Error: {exc}")
