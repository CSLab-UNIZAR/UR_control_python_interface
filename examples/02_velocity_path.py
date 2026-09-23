"""Example 2 - Velocity control: follow a small flat circle or an "infinity" shape.

The path lies in a plane through the start point of the TCP: by default the
tool's XY plane (horizontal when the gripper points down), or with --plane base
the horizontal XY plane of the UR base. It starts and ends at the start point,
with a smooth speed ramp at both ends.

Every 40 ms the commanded TCP velocity is

    v = v_ref(t) + kp * (p_ref(t) - p_measured)      feed-forward + P feedback
    w = kr * rotation error to the start orientation   (the orientation is held)

sent with robot.set_tcp_velocity. At the end the tracking error is reported and
the reference and measured paths are plotted.

Run from the repository root (robot started, Play pressed, room around the TCP):
    python examples/02_velocity_path.py --host CMP00-180723AD.local
    python examples/02_velocity_path.py --shape infinity --size 0.06 --period 10 --laps 2
"""

import argparse
import sys
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # find ur10api when run from anywhere
from ur10api import Rate, Robot, RobotError, transforms as tf   # noqa: E402


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
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="CMP00-180723AD.local")
    parser.add_argument("--shape", choices=("circle", "infinity"), default="circle")
    parser.add_argument("--size", type=float, default=0.05, help="radius / half width [m] (default 0.05)")
    parser.add_argument("--period", type=float, default=8.0, help="seconds per lap (default 8)")
    parser.add_argument("--laps", type=int, default=2)
    parser.add_argument("--plane", choices=("tool", "base"), default="tool",
                        help="tool: XY plane of the tool at the start; base: XY plane of the UR base")
    parser.add_argument("--kp", type=float, default=2.0, help="position feedback gain [1/s]")
    parser.add_argument("--rate", type=float, default=25.0, help="control rate [Hz]")
    args = parser.parse_args()
    ramp = min(2.0, args.period / 2)

    with Robot(args.host, max_linear_speed=0.12, max_angular_speed=0.3) as robot:
        start = robot.tcp_pose()
        origin = start[:3, 3].copy()
        axes = start[:3, :2] if args.plane == "tool" else np.eye(3)[:, :2]   # plane axes in the base frame

        # How far the path goes, to check the workspace before moving
        u_all = np.linspace(0, 2 * np.pi, 400)
        path = np.array([origin + axes @ shape(args.shape, u, args.size)[0] for u in u_all])
        outside = [p for p in path if not robot.workspace.contains(tf.pose(p))]
        if outside:
            sys.exit(f"The path leaves the workspace ({len(outside)} of {len(path)} points): "
                     "move the robot or reduce --size.")

        duration = 2 * np.pi * args.laps / (2 * np.pi / args.period) + ramp
        print(f"Following a {args.size * 1000:.0f} mm {args.shape} in the {args.plane} plane, "
              f"{args.laps} laps in {duration:.1f} s. Ctrl+C stops.")
        log = []   # time, reference point, measured point (base frame)
        rate = Rate(args.rate)
        t0 = time.monotonic()
        try:
            while True:
                t = time.monotonic() - t0
                u, du = phase(t, args.laps, args.period, ramp)
                point, dpoint = shape(args.shape, u, args.size)
                p_ref = origin + axes @ point              # reference position [m]
                v_ref = axes @ dpoint * du                  # reference velocity [m/s]
                state = robot.state()
                v = v_ref + args.kp * (p_ref - state.position)
                w = 2.0 * tf.pose_error(state.tcp, start)[1]   # hold the start orientation
                robot.set_tcp_velocity(v, w)
                log.append((t, *p_ref, *state.position))
                if t > duration + 1.0:                      # path done, 1 s to settle
                    break
                rate.sleep()
        except KeyboardInterrupt:
            print("Stopped by the user.")
        finally:
            robot.stop()

    data = np.array(log)
    t, ref, meas = data[:, 0], data[:, 1:4], data[:, 4:7]
    running = (t > ramp) & (t < duration - ramp)
    error = np.linalg.norm(ref - meas, axis=1) * 1000
    print(f"Tracking error while cruising: RMS {np.sqrt(np.mean(error[running] ** 2)):.1f} mm, "
          f"max {error[running].max():.1f} mm")

    # Plot in plane coordinates: reference vs measured path, and error over time
    ref2, meas2 = (ref - origin) @ axes * 1000, (meas - origin) @ axes * 1000
    fig, (ax_path, ax_err) = plt.subplots(1, 2, figsize=(11, 4.8))
    ax_path.plot(ref2[:, 0], ref2[:, 1], "--", color="0.5", label="reference")
    ax_path.plot(meas2[:, 0], meas2[:, 1], color="tab:blue", label="measured TCP")
    ax_path.set_aspect("equal")
    ax_path.set_xlabel(f"{args.plane} x [mm]")
    ax_path.set_ylabel(f"{args.plane} y [mm]")
    ax_path.set_title(f"{args.shape} path")
    ax_path.legend()
    ax_err.plot(t, error, color="tab:red")
    ax_err.axvspan(0, ramp, color="0.9")
    ax_err.axvspan(duration - ramp, t[-1], color="0.9")
    ax_err.set_xlabel("time [s]")
    ax_err.set_ylabel("tracking error [mm]")
    ax_err.set_title("error (grey: speed ramps)")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    try:
        main()
    except RobotError as exc:   # no connection, stale data, refused motion
        sys.exit(f"Error: {exc}")
