"""Example 1 - Pose control: align the TCP with a series of target frames.

The target frames are defined relative to the start pose of the TCP, either in
the tool frame (default: "6 cm along my own X axis, turned 20 deg about my Z") or
in the UR base frame ("6 cm along the base X axis ..."). The TCP visits them one
by one and comes back.

Two ways of reaching a target:
  --mode servo  (default) a Cartesian P-controller: 25 times per second it reads
                the pose error and commands  v = gain * position error,
                w = gain * rotation error  (robot.set_tcp_velocity). The TCP moves
                along a straight line and converges exactly.
  --mode move   one blocking joint-space motion per target (robot.move_tcp).

Live plots: the arm, the target frames (the current one highlighted), the
straight lines between them and the path the TCP really followed; the TCP
position against the target, the position/orientation errors and the commanded
speed. When the run ends (or Ctrl+C, or closing the window) the window becomes a
summary with the whole run and the final error at every target; it is saved as
a PNG in runs/.

Before moving, the example checks that commands reach the arm without delay
(robot.check_command_link: wrist 3 wiggles by 1 deg); --no-link-check skips it.

The targets, gains, tolerances and speeds are in the examples.pose_frames
section of ur10_config.yaml; the options below override them.

Run from the repository root (robot started, Play pressed, gripper clear):
    python examples/01_pose_frames.py
    python examples/01_pose_frames.py --frame base --mode move
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # find ur10api when run from anywhere
from ur10api import ConfigError, Rate, Robot, RobotError, load_config, transforms as tf   # noqa: E402
from ur10api.viz import Dashboard, Panel                                                  # noqa: E402


class Stopped(Exception):
    """The user stopped the run (Ctrl+C or closed the window)."""


def log_sample(dash, t0, origin, state, target, v=np.zeros(3), w=np.zeros(3)):
    """Record the measured TCP, the target and the errors (mm, deg) for the plots."""
    dp, drot = tf.pose_error(state.tcp, target)
    p = state.position * 1000
    d, d_ref = (state.position - origin) * 1000, (target[:3, 3] - origin) * 1000   # relative to the start
    dash.record(time.monotonic() - t0, x=p[0], y=p[1], z=p[2],
                dx=d[0], dy=d[1], dz=d[2], dx_ref=d_ref[0], dy_ref=d_ref[1], dz_ref=d_ref[2],
                e_pos=np.linalg.norm(dp) * 1000, e_rot=np.degrees(np.linalg.norm(drot)),
                v=np.linalg.norm(v) * 1000, w=np.degrees(np.linalg.norm(w)))


def servo_to(robot, target, dash, t0, origin, stop, gain, tol_mm, tol_deg, rate_hz, timeout):
    """Drive the TCP to `target` with velocity commands; True when it gets there."""
    rate = Rate(rate_hz)
    deadline = time.monotonic() + timeout
    warning = ""                                                      # last message shown
    while time.monotonic() < deadline:
        if stop.is_set():
            raise Stopped
        state = robot.state()
        dp, drot = tf.pose_error(state.tcp, target)                   # remaining motion, base frame
        if np.linalg.norm(dp) * 1000 < tol_mm and np.degrees(np.linalg.norm(drot)) < tol_deg:
            robot.set_tcp_velocity()                                  # zero velocity = brake
            log_sample(dash, t0, origin, state, target)
            return True
        v, w = gain * dp, gain * drot
        robot.set_tcp_velocity(v, w)                                  # speed is capped by the Robot
        log_sample(dash, t0, origin, state, target, v, w)
        if robot.last_warning and robot.last_warning != warning:     # refused or held at a limit:
            print("  " + robot.last_warning)                          # print each new reason once
            dash.set_status(robot.last_warning)
        warning = robot.last_warning
        rate.sleep()
    robot.stop()
    return False


def main():
    cfg = load_config()                        # ur10_config.yaml
    ex = cfg["examples"]["pose_frames"]        # this example's defaults (mm, deg)
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default=cfg["robot"]["host"], help="rosbridge host (default: %(default)s)")
    parser.add_argument("--frame", choices=("tool", "base"), default=ex["frame"],
                        help="frame in which the target offsets are defined (default: %(default)s)")
    parser.add_argument("--mode", choices=("servo", "move"), default=ex["mode"], help="(default: %(default)s)")
    parser.add_argument("--gain", type=float, default=ex["gain"], help="servo gain [1/s] (default: %(default)s)")
    parser.add_argument("--max-speed", type=float, default=ex["max_speed"],
                        help="max TCP speed in servo mode [mm/s] (default: %(default)s)")
    parser.add_argument("--joint-speed", type=float, default=ex["joint_speed"],
                        help="joint speed in move mode [deg/s] (default: %(default)s)")
    parser.add_argument("--no-link-check", action="store_true", help="skip the start-up check of the command link")
    args = parser.parse_args()

    with Robot(args.host, config=cfg, max_linear_speed=args.max_speed / 1000,
               max_angular_speed=np.radians(ex["max_angular_speed"])) as robot:
        if cfg["link_check"]["enabled"] and not args.no_link_check:
            robot.check_command_link(report=print)
        start = robot.tcp_pose()
        # target = start pose displaced by the offsets of ur10_config.yaml (mm -> m, deg -> rad)
        targets = [(name, tf.displace(start, np.array(t["position"]) / 1000, np.radians(t["rotation"]), args.frame))
                   for name, t in ex["targets"].items()]
        targets.append(("start", start))

        dash = Dashboard(robot, f"Pose control - {args.frame} frame, {args.mode} mode", panels=[
            Panel("TCP position relative to the start (UR base axes)", "[mm]",
                  {"x": "dx", "y": "dy", "z": "dz", "x target": "dx_ref", "y target": "dy_ref", "z target": "dz_ref"},
                  styles={"x": {"color": "tab:red"}, "y": {"color": "tab:green"}, "z": {"color": "tab:blue"},
                          "x target": {"color": "tab:red", "ls": "--", "lw": 1},
                          "y target": {"color": "tab:green", "ls": "--", "lw": 1},
                          "z target": {"color": "tab:blue", "ls": "--", "lw": 1}}),
            Panel("Error to the current target", "position [mm] / orientation [deg]",
                  {"position [mm]": "e_pos", "orientation [deg]": "e_rot"},
                  styles={"position [mm]": {"color": "tab:purple"}, "orientation [deg]": {"color": "tab:orange"}},
                  hlines={f"tolerance {ex['tolerance']:g} mm": ex["tolerance"]}),
            Panel("Commanded TCP speed (servo mode)", "[mm/s] / [deg/s]",
                  {"linear [mm/s]": "v", "angular [deg/s]": "w"},
                  styles={"linear [mm/s]": {"color": "tab:cyan"}, "angular [deg/s]": {"color": "tab:olive"}}),
        ], paths={"TCP path": ("x", "y", "z", {"color": "tab:purple"})})
        dash.scene.add_frame(start, "start")
        for name, T in targets[:-1]:
            dash.scene.add_frame(T, name)
        corners = np.array([T[:3, 3] for _, T in [("start", start)] + targets])
        dash.scene.add_path(corners, style="--", label="straight lines between targets")

        results = []   # (name, position error [mm], orientation error [deg], time [s])

        def control(stop):
            """The motion, run in a worker thread while the main thread draws."""
            t0 = time.monotonic()
            for name, target in targets:
                print(f"-> {name}")
                dash.set_frame("target", target, f"-> {name}")
                dash.set_status(f"Going to {name}")
                started = time.monotonic()
                if args.mode == "servo":
                    if not servo_to(robot, target, dash, t0, start[:3, 3], stop, args.gain, ex["tolerance"],
                                    ex["angle_tolerance"], ex["rate"], ex["timeout"]):
                        print("  target not reached in time")
                else:
                    def follow(state):
                        if stop.is_set():
                            raise Stopped
                        log_sample(dash, t0, start[:3, 3], state, target)
                    robot.move_tcp(target, speed=np.radians(args.joint_speed), callback=follow)
                dp, drot = tf.pose_error(robot.tcp_pose(), target)
                results.append((name, np.linalg.norm(dp) * 1000, np.degrees(np.linalg.norm(drot)),
                                time.monotonic() - started))
                print(f"  {name}: position error {results[-1][1]:5.2f} mm, orientation error {results[-1][2]:5.2f} deg")
                pause = time.monotonic() + 1.0
                while time.monotonic() < pause:                        # 1 s at the target, still logging
                    if stop.is_set():
                        raise Stopped
                    log_sample(dash, t0, start[:3, 3], robot.state(), target)
                    time.sleep(0.04)

        print(f"Visiting {len(targets) - 1} targets defined in the {args.frame} frame ({args.mode} mode). "
              "Ctrl+C or closing the window stops.")
        ending = ""
        try:
            if not dash.run(control):
                ending = "stopped by the user"
        except Stopped:
            ending = "stopped by the user"
        except RobotError as exc:          # e.g. a target outside the workspace: still show the summary
            ending = f"error: {exc}"
        try:
            robot.stop()
        except RobotError:                 # connection lost: nothing to stop
            pass
        if ending:
            print(ending[0].upper() + ending[1:])

    summary = [f"{args.mode} mode, targets in the {args.frame} frame" + (f"  -  {ending}" if ending else "")]
    summary += [f"{name:>6}: error {e_pos:5.2f} mm, {e_rot:5.2f} deg after {t:4.1f} s" for name, e_pos, e_rot, t in results]
    dash.finish(summary)


if __name__ == "__main__":
    try:
        main()
    except (RobotError, ConfigError) as exc:   # no connection, stale data, refused motion, bad settings
        sys.exit(f"Error: {exc}")
