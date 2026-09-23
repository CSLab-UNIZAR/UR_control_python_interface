"""Example 1 - Pose control: align the TCP with three target frames.

Three target frames are defined relative to the start pose of the TCP, either in
the tool frame (default: "5 cm along my own X axis, turned 20 deg about my Z") or
in the UR base frame ("5 cm along the base X axis ..."). The TCP visits them one
by one and comes back, while a live 3D view shows the arm, the targets, the
moving TCP frame and its path.

Two ways of reaching a target:
  --mode servo  (default) a Cartesian P-controller: every 40 ms it reads the pose
                error and commands  v = gain * position error,
                w = gain * rotation error  (robot.set_tcp_velocity). The TCP moves
                along a straight line and converges exactly.
  --mode move   one blocking joint-space motion per target (robot.move_tcp).

Run from the repository root (robot started, Play pressed, gripper clear):
    python examples/01_pose_frames.py --host CMP00-180723AD.local
    python examples/01_pose_frames.py --frame base --mode move
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # find ur10api when run from anywhere
from ur10api import Rate, Robot, RobotError, transforms as tf   # noqa: E402
from ur10api.viz import FrameView                    # noqa: E402

# name, translation [m] and rotation vector [deg] of each target, relative to the start pose
TARGETS = (
    ("T1", (0.06, 0.00, 0.00), (0, 0, 20)),
    ("T2", (0.00, 0.06, 0.03), (15, 0, 0)),
    ("T3", (-0.04, -0.04, 0.02), (0, -15, 0)),
)


def servo_to(robot, target, view, gain=1.5, tol_mm=1.0, tol_deg=0.3, timeout=30.0):
    """Drive the TCP to `target` with velocity commands; True when it gets there."""
    rate = Rate(25)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = robot.state()
        dp, drot = tf.pose_error(state.tcp, target)          # remaining motion, base frame
        view.update(state)
        if np.linalg.norm(dp) * 1000 < tol_mm and np.degrees(np.linalg.norm(drot)) < tol_deg:
            robot.set_tcp_velocity()                           # zero velocity = brake
            return True
        if not robot.set_tcp_velocity(gain * dp, gain * drot):   # speed is capped by the Robot
            print("  not moving:", robot.last_warning)
        rate.sleep()
    robot.stop()
    return False


def report(name, robot, target):
    dp, drot = tf.pose_error(robot.tcp_pose(), target)
    print(f"  {name}: position error {np.linalg.norm(dp) * 1000:5.2f} mm, "
          f"orientation error {np.degrees(np.linalg.norm(drot)):5.2f} deg")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="CMP00-180723AD.local")
    parser.add_argument("--frame", choices=("tool", "base"), default="tool",
                        help="frame in which the target offsets are defined (default: tool)")
    parser.add_argument("--mode", choices=("servo", "move"), default="servo")
    parser.add_argument("--speed", type=float, default=0.04, help="max TCP speed in servo mode [m/s]")
    parser.add_argument("--joint-speed", type=float, default=0.2, help="joint speed in move mode [rad/s]")
    args = parser.parse_args()

    with Robot(args.host, max_linear_speed=args.speed, max_angular_speed=0.3) as robot:
        start = robot.tcp_pose()
        targets = [(name, tf.displace(start, offset, np.radians(rotation), args.frame))
                   for name, offset, rotation in TARGETS]
        targets.append(("start", start))

        view = FrameView(robot.arm, title=f"Pose control ({args.frame} frame, {args.mode})",
                         workspace=robot.workspace)
        view.add_frame(start, "start")
        for name, T in targets[:-1]:
            view.add_frame(T, name)
        view.update(robot.state(), force=True)

        print(f"Visiting {len(TARGETS)} targets defined in the {args.frame} frame ({args.mode} mode). Ctrl+C stops.")
        try:
            for name, target in targets:
                print(f"-> {name}")
                if args.mode == "servo":
                    if not servo_to(robot, target, view):
                        print("  target not reached in time")
                else:
                    robot.move_tcp(target, speed=args.joint_speed, callback=view.update)
                report(name, robot, target)
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("Stopped by the user.")
            robot.stop()
        view.update(robot.state(), force=True)
    print("Done. Close the plot window to exit.")
    view.hold()


if __name__ == "__main__":
    try:
        main()
    except RobotError as exc:   # no connection, stale data, refused motion
        sys.exit(f"Error: {exc}")
