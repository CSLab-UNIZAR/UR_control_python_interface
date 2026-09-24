"""Example 4 - Model-free shape servoing in image or camera space (ArUco markers, RealSense camera).

A fixed camera (Intel RealSense D435 or D415, colour stream) watches ArUco
markers, e.g. along a cable held by the gripper. The markers, taken in
increasing ID order, define a shape, described in one of three feature
spaces, either in the image (2D, pixels) or in the camera frame (3D, from the
marker corners with the camera intrinsics; the nominal marker size only sets
the scale, which does not matter here):

    position    pixel positions of the N markers                2N values [px]
    edges       vector from each marker to the next              2(N-1)    [px]
    curvature   signed angle between consecutive edges           N-2       [deg]

The arm is driven so that the shape reaches a recorded target, with no model
of the camera, the arm or the object: the Jacobian J that maps joint motions
to feature changes (ds = J dq) is estimated from data. At least 3 markers are
needed (4-5 work better). Each marker is tracked by a Kalman filter that
trusts the detections much more than its motion model, so a marker missed in
a few frames is predicted instead of lost.

Workflow, with the buttons or the keys of the window:
  0. Grasp the object: Open gripper (O), place it, Close gripper (C).
  1. Teleoperate the joints (hold Q/A W/S E/D R/F T/G Y/H, or a jog button)
     until the shape is the one wanted, then Record target (Enter).
  2. Back to start (B, the configuration at start-up or the one set with I)
     or Home (N, moves.home in the settings).
  3. Probe Jacobian (P): each joint moves back and forth around the current
     configuration q0 (+-10 deg J1-J3, +-15 deg J4-J6 by default) while the
     marker motion is recorded; a least-squares fit of s - s0 = J (q - q0) + b
     gives the offline estimate J0, for the three feature spaces at once.
  4. Start control (V): damped-pseudoinverse servoing towards the target s*,
         qdot = -gain * J^T (J J^T + mu^2 I)^-1 (s - s*),   mu = damping * sigma_max(J)
     at 25 Hz, with a Broyden update of J each time the joints moved by min_step:
         J <- J + alpha * (ds - J dq) dq^T / (dq^T dq)
     It stops when the RMS feature error stays within the tolerance, on
     timeout, or with STOP (Space / Esc). If a marker is lost, the arm holds still.
  5. Repeat any step. 1/2/3 switch the feature space and M the frame (2D
     image / 3D camera) at any time, also while servoing: the loop continues
     with the Jacobian of the new feature set (all six are probed at once).

With "Log runs" on (L), every servo run is saved in its own folder,
runs/shape_servoing/<date>_<feature set>/: signals.csv (every logged signal),
meta.json (settings, target, Jacobians, outcome), window.mp4 (the whole
window) and camera.mp4 (the raw camera images).

Live window: the image space (markers, edges as arrows, curvature as circles
that grow with the angle, and the target; in 3D also a 3D view of the chain,
the target and the marker trajectories), the three feature spaces over time
(blue, orange, green), the joint velocities, the feature error with its norm,
and the condition number of the Jacobian estimates. Every joint velocity sent
(jog and servoing) keeps the TCP in the workspace box of ur10_config.yaml, and
the probing motion is checked against it before moving.

At the end (window closed, or Ctrl+C) the robot brakes; the window becomes a
summary of the session, saved as a PNG in runs/ with the logged signals
(.npz). Recorded targets are saved in runs/ too, to reuse with --target.

All parameters are in examples/04_shape_servoing.yaml (camera, markers,
Kalman filter, tolerances, probing, gains, Broyden, display).

Needs: opencv-python-headless and pyrealsense2 (in requirements.txt).
Run from the repository root (robot started, Play pressed, markers in view):
    python examples/04_shape_servoing.py
    python examples/04_shape_servoing.py --space curvature --target runs/shape_target_20260924_101500.json
    python examples/04_shape_servoing.py --camera-only          # no robot: camera, markers, features
"""

import argparse
import math
import sys
import time
from contextlib import nullcontext
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))                           # find ur10api when run from anywhere
sys.path.insert(0, str(Path(__file__).resolve().parent))   # and the shape_servoing package next to this file
from ur10api import ConfigError, Robot, RobotError, load_config            # noqa: E402
from shape_servoing import features as feat                                 # noqa: E402
from shape_servoing.display import Display                                  # noqa: E402
from shape_servoing.logger import RunLogger                                 # noqa: E402
from shape_servoing.session import Session, Target                          # noqa: E402
from shape_servoing.settings import FRAMES, SPACES, load_settings           # noqa: E402
from shape_servoing.vision import Vision, VisionError                       # noqa: E402

TITLE = "Shape servoing (model-free)"


def main():
    cfg = load_config()                        # ur10_config.yaml: robot, TCP, workspace, safety, plots
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default=cfg["robot"]["host"], help="rosbridge host (default: %(default)s)")
    parser.add_argument("--settings", type=Path, help="settings file (default: examples/04_shape_servoing.yaml)")
    parser.add_argument("--space", choices=SPACES, help="feature space at start-up (default: features.space)")
    parser.add_argument("--frame", choices=FRAMES,
                        help="features in the image (2D) or the camera frame (3D) at start-up "
                             "(default: features.frame)")
    parser.add_argument("--camera", choices=("realsense", "webcam"), help="camera to use (default: camera.source)")
    parser.add_argument("--target", type=Path,
                        help="start with a target saved by Record target (runs/shape_target_*.json)")
    parser.add_argument("--camera-only", action="store_true",
                        help="no robot: try the camera, the markers, the features and the target recording")
    parser.add_argument("--no-link-check", action="store_true", help="skip the start-up check of the command link")
    args = parser.parse_args()
    settings = load_settings(args.settings)
    save_dir = ROOT / cfg["plots"]["save_dir"] if cfg["plots"]["save_dir"] else None
    target = image_size = None
    if args.target is not None:
        try:
            target, image_size = Target.load(args.target)
        except ValueError as exc:
            sys.exit(f"Error: {exc}")
        print(f"Target loaded from {args.target}: markers {', '.join(map(str, target.ids))}")

    vision = Vision(settings, source=args.camera)
    print(f"Camera: {vision.camera.name}")
    vision.start()
    try:
        height, width = vision.wait_first().image.shape[:2]
        if image_size and tuple(image_size) != (width, height):
            print(f"Note: the target was recorded on {image_size[0]}x{image_size[1]} images, the camera gives "
                  f"{width}x{height}: its pixel positions do not match.")
        speeds = (settings["teleop"]["max_joint_speed"], settings["moves"]["speed"], settings["probe"]["speed"],
                  settings["control"]["max_joint_speed"])
        robot = None if args.camera_only else Robot(args.host, config=cfg, max_joint_speed=math.radians(max(speeds)))
        with robot if robot is not None else nullcontext():
            if robot is not None and cfg["link_check"]["enabled"] and not args.no_link_check:
                robot.check_command_link(report=print)
            session = Session(robot, vision, settings, target=target, space=args.space, frame=args.frame,
                              save_dir=save_dir)
            if save_dir is not None:              # "Log runs": every servo run to runs/<logger.folder>/
                session.logger = RunLogger(save_dir / settings["logger"]["folder"], settings, session.recorder,
                                           session.t0, vision)
            display = Display(session, settings, TITLE + (" - camera only" if robot is None else ""))
            print("Window: O/C open/close gripper, hold Q/A W/S E/D R/F T/G Y/H to jog, Enter record target, "
                  "B back to start, P probe Jacobian, V start control, 1/2/3 feature space, M 2D/3D, L log runs, "
                  "Space/Esc STOP. "
                  "Close the window (or Ctrl+C here) to end.")
            ending = ""
            try:
                if not display.run():
                    ending = "stopped with Ctrl+C"
            except RobotError as exc:
                ending = f"error: {exc}"
                print(ending)
            if robot is not None:
                try:
                    robot.stop()
                except RobotError:             # connection lost: nothing to stop
                    pass
    finally:
        vision.close()
    if session.logger is not None:
        session.logger.close()                      # wait until the last run is written

    summary = summarize(session, ending)
    print("\n".join(summary[1:]))
    stamp = time.strftime("%Y%m%d_%H%M%S")
    figure = log = None
    if save_dir is not None:
        figure, log = save_dir / f"shape_servoing_{stamp}.png", save_dir / f"shape_servoing_{stamp}.npz"
        extra = session.jacobians()
        if session.target is not None:
            extra.update(target_ids=np.array(session.target.ids), target_points=session.target.points)
        try:
            save_dir.mkdir(parents=True, exist_ok=True)
            session.recorder.save(log, **extra)
            print(f"Logged signals saved to {log}")
        except OSError as exc:
            print(f"Could not save the log: {exc}")
    display.finish(summary, save_as=figure)


def summarize(session, ending):
    """Text lines describing the session, for the terminal and the summary figure."""
    status = session.status
    lines = [TITLE + (f"  -  {ending}" if ending else ""),
             f"{status.elapsed:.0f} s, feature set at the end: {status.key}"]
    target = session.target
    lines.append("Target: " + (f"markers {', '.join(map(str, target.ids))}" if target is not None else "none recorded"))
    lines.append(f"Jacobian ({status.key}): {status.jacobian}")
    if not session.runs:
        lines.append("No servo run.")
    for k, run in enumerate(session.runs, 1):
        rms = "" if math.isnan(run.rms) else f", final RMS error {run.rms:.2f} {feat.unit(run.key)}"
        lines.append(f"Servo run {k} ({run.key}): {run.outcome} after {run.duration:.1f} s{rms}")
    return lines


if __name__ == "__main__":
    try:
        main()
    except (RobotError, ConfigError, VisionError) as exc:   # no connection, refused motion, bad settings, camera
        sys.exit(f"Error: {exc}")
