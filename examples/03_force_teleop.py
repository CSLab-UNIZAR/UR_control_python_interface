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

Run from the repository root (robot started, Play pressed, gripper free):
    python examples/03_force_teleop.py --host CMP00-180723AD.local
    python examples/03_force_teleop.py --rotate --view
Ctrl+C brakes and exits.
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # find ur10api when run from anywhere
from ur10api import Rate, Robot, RobotError, transforms as tf   # noqa: E402


def admittance(vector, gain, dead_band):
    """Velocity along `vector`, proportional to how far its magnitude exceeds the dead band."""
    magnitude = np.linalg.norm(vector)
    if magnitude <= dead_band:
        return np.zeros(3)
    return gain * (magnitude - dead_band) * vector / magnitude


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="CMP00-180723AD.local")
    parser.add_argument("--gain", type=float, default=0.004, help="speed per newton above the dead band [m/s/N]")
    parser.add_argument("--dead-band", type=float, default=3.0, help="forces below this are ignored [N]")
    parser.add_argument("--max-speed", type=float, default=0.05, help="TCP speed cap [m/s]")
    parser.add_argument("--filter", type=float, default=0.2,
                        help="low-pass weight of each new sample, 0-1 (smaller = smoother)")
    parser.add_argument("--rotate", action="store_true", help="also rotate with the measured torque")
    parser.add_argument("--torque-gain", type=float, default=0.15, help="[rad/s per Nm] above the dead band")
    parser.add_argument("--torque-dead-band", type=float, default=0.8, help="[Nm]")
    parser.add_argument("--max-rotation", type=float, default=30.0, help="max rotation from the start [deg]")
    parser.add_argument("--invert", action="store_true", help="move against the measured force")
    parser.add_argument("--view", action="store_true", help="live 3D view with the force vector")
    args = parser.parse_args()
    sign = -1.0 if args.invert else 1.0

    with Robot(args.host, max_linear_speed=args.max_speed, max_angular_speed=0.25) as robot:
        print("Do not touch the gripper: zeroing the force sensor in 3 s ...")
        time.sleep(3.0)
        robot.zero_force_sensor()
        start = robot.tcp_pose()
        view = None
        if args.view:
            from ur10api.viz import FrameView
            view = FrameView(robot.arm, title="Compliant teleoperation", workspace=robot.workspace)
            view.add_frame(start, "start")

        print("Push the gripper to move it. Ctrl+C to stop.")
        force = torque = np.zeros(3)
        rate = Rate(50)
        last_print = 0.0
        try:
            while True:
                state = robot.state()
                if state.wrench_age is None or state.wrench_age > 0.3:
                    robot.set_tcp_velocity()                         # no force data: brake and wait
                    print("\rwaiting for /robotiq_ft_sensor ...            ", end="")
                    rate.sleep()
                    continue
                # 1. low-pass filter (exponential moving average) in the tool frame
                force = args.filter * state.force + (1 - args.filter) * force
                torque = args.filter * state.torque + (1 - args.filter) * torque
                # 2. tool frame -> base frame (the sensor axes are those of the flange)
                R = state.flange[:3, :3]
                force_base, torque_base = R @ force, R @ torque
                # 3. admittance: push -> velocity
                v = sign * admittance(force_base, args.gain, args.dead_band)
                w = np.zeros(3)
                if args.rotate:
                    w = sign * admittance(torque_base, args.torque_gain, args.torque_dead_band)
                    offset = tf.pose_error(start, state.tcp)[1]      # rotation already done since the start
                    if np.degrees(np.linalg.norm(offset)) > args.max_rotation and np.dot(w, offset) > 0:
                        w = w - np.dot(w, offset) / np.dot(offset, offset) * offset   # no further away
                # 4. command (the Robot caps speed and acceleration and keeps the TCP in the workspace)
                robot.set_tcp_velocity(v, w)

                if view is not None:
                    view.set_vector("force", state.position, force_base * 0.01)   # 1 cm per N
                    view.update(state)
                now = time.monotonic()
                if now - last_print > 0.25:
                    last_print = now
                    note = f"  ({robot.last_warning})" if robot.last_warning else ""
                    print(f"\r|F| {np.linalg.norm(force_base):5.1f} N   |M| {np.linalg.norm(torque_base):4.2f} Nm   "
                          f"v {np.linalg.norm(v) * 1000:5.1f} mm/s{note}          ", end="")
                rate.sleep()
        except KeyboardInterrupt:
            print("\nStopping.")
            robot.stop()


if __name__ == "__main__":
    try:
        main()
    except RobotError as exc:   # no connection, stale data, refused motion
        sys.exit(f"Error: {exc}")
