"""Example 3 - Compliant teleoperation: push the gripper and it follows.

Admittance control with the Robotiq FT 300 on the flange:
  1. the force/torque measured in the tool frame is low-pass filtered;
  2. it is turned into the UR base frame;
  3. a dead band removes sensor noise and the residual weight of the gripper,
     and the rest becomes a velocity along the push:
         v = gain * (|F| - dead band) * F / |F|     (capped at --max-speed)
  4. the velocity is sent with robot.set_tcp_velocity, 50 times per second.
When nobody pushes, the velocity ramps down to zero and the arm holds still.

With --rotate, torques also turn the TCP (w = torque_gain * (|M| - dead band)
along M), limited to --max-rotation degrees from the start orientation, as in
controllers/force_controller.py. Rotating the tool changes how the gripper's
weight loads the sensor, so keep rotations small.

The direction convention is the one of controllers/force_controller.py (the
arm moves along the measured force); if it moves the wrong way, use --invert.

Live plots: the arm with the path the TCP followed and the measured force as an
arrow; the force (UR base frame) against the dead band, the commanded velocity
and the displacement from the start. Ctrl+C (or closing the window) ends the run
and the window becomes a summary of the whole session, saved as a PNG in runs/.

Before moving, the example checks that commands reach the arm without delay
(robot.check_command_link: wrist 3 wiggles by 1 deg); --no-link-check skips it.

The gains, dead bands and caps are in the examples.force_teleop section of
ur10_config.yaml; the options below override them.

Run from the repository root (robot started, Play pressed, gripper free):
    python examples/03_force_teleop.py
    python examples/03_force_teleop.py --rotate
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # find ur10api when run from anywhere
from ur10api import ConfigError, Rate, Robot, RobotError, load_config, transforms as tf   # noqa: E402
from ur10api.viz import Dashboard, Panel                                                  # noqa: E402


def admittance(vector, gain, dead_band):
    """Velocity along `vector`, proportional to how far its magnitude exceeds the dead band."""
    magnitude = np.linalg.norm(vector)
    if magnitude <= dead_band:
        return np.zeros(3)
    return gain * (magnitude - dead_band) * vector / magnitude


def main():
    cfg = load_config()                        # ur10_config.yaml
    ex = cfg["examples"]["force_teleop"]       # this example's defaults (mm, deg, N)
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default=cfg["robot"]["host"], help="rosbridge host (default: %(default)s)")
    parser.add_argument("--gain", type=float, default=ex["gain"],
                        help="speed per newton above the dead band [mm/s per N] (default: %(default)s)")
    parser.add_argument("--dead-band", type=float, default=ex["dead_band"],
                        help="forces below this are ignored [N] (default: %(default)s)")
    parser.add_argument("--max-speed", type=float, default=ex["max_speed"], help="TCP speed cap [mm/s] (default: %(default)s)")
    parser.add_argument("--filter", type=float, default=ex["filter"],
                        help="low-pass weight of each new sample, 0-1, smaller = smoother (default: %(default)s)")
    parser.add_argument("--rotate", action=argparse.BooleanOptionalAction, default=ex["rotate"],
                        help="also rotate with the measured torque")
    parser.add_argument("--torque-gain", type=float, default=ex["torque_gain"],
                        help="[deg/s per Nm] above the torque dead band (default: %(default)s)")
    parser.add_argument("--torque-dead-band", type=float, default=ex["torque_dead_band"], help="[Nm] (default: %(default)s)")
    parser.add_argument("--max-rotation", type=float, default=ex["max_rotation"],
                        help="max rotation from the start [deg] (default: %(default)s)")
    parser.add_argument("--invert", action=argparse.BooleanOptionalAction, default=ex["invert"],
                        help="move against the measured force")
    parser.add_argument("--no-link-check", action="store_true", help="skip the start-up check of the command link")
    args = parser.parse_args()
    sign = -1.0 if args.invert else 1.0
    gain = args.gain / 1000                      # m/s per N
    torque_gain = np.radians(args.torque_gain)   # rad/s per Nm

    with Robot(args.host, config=cfg, max_linear_speed=args.max_speed / 1000,
               max_angular_speed=np.radians(ex["max_angular_speed"])) as robot:
        if cfg["link_check"]["enabled"] and not args.no_link_check:
            robot.check_command_link(report=print)
        print("Do not touch the gripper: zeroing the force sensor in 3 s ...")
        time.sleep(3.0)
        robot.zero_force_sensor()
        start = robot.tcp_pose()

        dash = Dashboard(robot, "Compliant teleoperation" + (" with rotation" if args.rotate else ""), panels=[
            Panel("Measured force (UR base frame, filtered)", "[N]",
                  {"Fx": "fx", "Fy": "fy", "Fz": "fz", "|F|": "f"},
                  styles={"Fx": {"color": "tab:red"}, "Fy": {"color": "tab:green"}, "Fz": {"color": "tab:blue"},
                          "|F|": {"color": "black", "lw": 2}},
                  hlines={f"dead band {args.dead_band:g} N": args.dead_band}),
            Panel("Commanded TCP velocity", "[mm/s]",
                  {"vx": "vx", "vy": "vy", "vz": "vz", "|v|": "v"},
                  styles={"vx": {"color": "tab:red"}, "vy": {"color": "tab:green"}, "vz": {"color": "tab:blue"},
                          "|v|": {"color": "black", "lw": 2}}),
            Panel("TCP displacement from the start", "[mm] / rotation [deg]",
                  {"dx": "dx", "dy": "dy", "dz": "dz", "rotation [deg]": "rot"},
                  styles={"dx": {"color": "tab:red"}, "dy": {"color": "tab:green"}, "dz": {"color": "tab:blue"},
                          "rotation [deg]": {"color": "tab:orange", "ls": "--"}}),
        ], paths={"TCP path": ("x", "y", "z", {"color": "tab:purple"})})
        dash.scene.add_frame(start, "start")
        peaks = {"force": 0.0, "speed": 0.0}

        def control(stop):
            """The admittance loop, run in a worker thread while the main thread draws."""
            force = torque = np.zeros(3)
            rate = Rate(ex["rate"])
            t0 = time.monotonic()
            while not stop.is_set():
                state = robot.state()
                if state.wrench_age is None or state.wrench_age > 0.3:
                    robot.set_tcp_velocity()                         # no force data: brake and wait
                    dash.set_status("waiting for /robotiq_ft_sensor ...")
                    rate.sleep()
                    continue
                # 1. low-pass filter (exponential moving average) in the tool frame
                force = args.filter * state.force + (1 - args.filter) * force
                torque = args.filter * state.torque + (1 - args.filter) * torque
                # 2. tool frame -> base frame (the sensor axes are those of the flange)
                R = state.flange[:3, :3]
                force_base, torque_base = R @ force, R @ torque
                # 3. admittance: push -> velocity
                v = sign * admittance(force_base, gain, args.dead_band)
                w = np.zeros(3)
                offset = tf.pose_error(start, state.tcp)[1]          # rotation done since the start
                if args.rotate:
                    w = sign * admittance(torque_base, torque_gain, args.torque_dead_band)
                    if np.degrees(np.linalg.norm(offset)) > args.max_rotation and np.dot(w, offset) > 0:
                        w = w - np.dot(w, offset) / np.dot(offset, offset) * offset   # no further away
                # 4. command (the Robot caps speed and acceleration and keeps the TCP in the workspace)
                robot.set_tcp_velocity(v, w)

                p, d = state.position * 1000, (state.position - start[:3, 3]) * 1000
                f, speed = np.linalg.norm(force_base), np.linalg.norm(v) * 1000
                peaks["force"], peaks["speed"] = max(peaks["force"], f), max(peaks["speed"], speed)
                dash.record(time.monotonic() - t0, x=p[0], y=p[1], z=p[2], dx=d[0], dy=d[1], dz=d[2],
                            rot=np.degrees(np.linalg.norm(offset)),
                            fx=force_base[0], fy=force_base[1], fz=force_base[2], f=f,
                            vx=v[0] * 1000, vy=v[1] * 1000, vz=v[2] * 1000, v=speed)
                dash.set_vector("force", state.position, force_base * 0.01, label="force (1 cm per N)")
                dash.set_status(f"|F| {f:5.1f} N   |M| {np.linalg.norm(torque_base):4.2f} Nm   v {speed:5.1f} mm/s"
                                + (f"   ({robot.last_warning})" if robot.last_warning else ""))
                rate.sleep()

        print("Push the gripper to move it. Ctrl+C or closing the plot window stops.")
        ending = ""
        try:
            dash.run(control)
        except RobotError as exc:
            ending = f"error: {exc}"
            print(ending)
        try:
            robot.stop()
        except RobotError:                 # connection lost: nothing to stop
            pass
        end = robot.tcp_pose() if robot.is_ready() else start

    t = dash.times()
    moved = np.linalg.norm(end[:3, 3] - start[:3, 3]) * 1000
    xyz = np.c_[dash.column("x"), dash.column("y"), dash.column("z")]
    xyz = xyz[np.all(np.isfinite(xyz), axis=1)]
    travelled = np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)) if len(xyz) > 1 else 0.0
    summary = [dash.title + (f"  -  {ending}" if ending else ""),
               f"{t[-1] if len(t) else 0:.0f} s: path length {travelled:.0f} mm, {moved:.0f} mm from the start;"
               f" peak force {peaks['force']:.1f} N, peak speed {peaks['speed']:.0f} mm/s"
               f"   (gain {args.gain:g} mm/s/N, dead band {args.dead_band:g} N)"]
    print(summary[1])
    dash.finish(summary)


if __name__ == "__main__":
    try:
        main()
    except (RobotError, ConfigError) as exc:   # no connection, stale data, refused motion, bad settings
        sys.exit(f"Error: {exc}")
