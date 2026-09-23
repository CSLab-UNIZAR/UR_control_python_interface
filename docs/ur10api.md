# ur10api – Python interface for your own control code

`ur10api` is a small Python package for writing your own control programs for the
UR10 on the Campero. One class, `Robot`, reads the robot state (joints, TCP pose,
force/torque, gripper) and sends commands (joint and pose moves, TCP or joint
velocities, gripper, sensor zeroing, stop). Helper modules cover poses, calibrated
kinematics and live plots. The robot address, TCP, workspace, safety limits and
the gains of the examples are set in one commented file, `ur10_config.yaml`.

```python
from ur10api import Robot

with Robot() as robot:                     # address, TCP, workspace, caps: ur10_config.yaml
    robot.check_command_link()             # wait until commands reach the arm promptly
    print(robot.tcp_pose())                # 4x4 TCP pose in the UR base frame
    robot.move_tcp_by(dp=(0, 0, 0.05))     # 5 cm up
    robot.close_gripper()
```

Three example programs in [`examples/`](../examples) show typical control
processes, each with a live dashboard and a saved summary figure: pose control
toward target frames, velocity control along a path, and compliant
teleoperation with the force sensor.

---

## Contents

1. [How it fits in the repository](#how-it-fits-in-the-repository)
2. [Installation and setup](#installation-and-setup)
3. [Settings file (`ur10_config.yaml`)](#settings-file-ur10_configyaml)
4. [Quick start](#quick-start)
5. [Concepts](#concepts)
6. [API reference](#api-reference)
7. [Writing a control loop](#writing-a-control-loop)
8. [Examples](#examples)
9. [Troubleshooting](#troubleshooting)

---

## How it fits in the repository

```
your script / examples/         webui/  (web control panel)
          │                        │
          └────────► ur10api ◄──────┘     Robot, kinematics, transforms, viz
                        │
                        ▼
               core/ur_control.py        UR10Control (UR_CONTROL, unchanged)
                        │  rosbridge websocket (Wi-Fi)
                        ▼
               Campero PC: /pub_ik_trajectory, /pub_gripper_control, ...
```

- Every command goes out through `UR10Control.send_trajectory` (joint targets,
  `art=True`), `send_gripper_cmd` and `zero_ft_sensor`. The Campero receives exactly
  the same messages as with the other controllers: one-point `JointTrajectory`
  messages on `/pub_ik_trajectory` and `/pub_gripper_control`.
- Cartesian targets are solved in `ur10api` with the arm's **factory calibration**
  (from `robot_model/model.json`, the Campero URDF), so poses match the teach
  pendant and RViz.
- The web panel uses the same kinematics, so the numbers you see in the panel and
  in your scripts are the same.

| Module | Contents |
|---|---|
| `ur10api.robot` | `Robot`, `RobotState`, `GripperState`, `Rate`, errors |
| `ur10api.config` | `load_config()`: the settings file `ur10_config.yaml` |
| `ur10api.transforms` | building and comparing 4×4 poses |
| `ur10api.kinematics` | `ArmModel` (FK, IK, Jacobian), `Workspace` |
| `ur10api.viz` | live plots (matplotlib): `Dashboard` + `Panel` (3D views and time plots, used by the examples) and `FrameView` (a single 3D view) |

---

## Installation and setup

`ur10api` is part of this repository and uses the same environment as the rest of
it.

1. **Create the environment** with the setup script of your system (see the
   [README](../README.md#quick-start-windows-and-linux)):
   - Windows: `setup.bat` (or `.\setup.ps1`);
   - Linux: `./setup.sh`.

   Besides installing `requirements.txt`, the script registers the repository in
   `.venv` (a `ur_control.pth` file), so `import ur10api` works from **any folder**
   with the environment's Python.
2. **Plot windows** (`ur10api.viz` and the examples) need Tk, which the python.org
   Windows installer includes. On Ubuntu: `sudo apt install python3-tk`. The setup
   script warns if it is missing.
3. **Robot side**: start the Campero as described in the
   [README](../README.md#robot-side-start-up-campero-pc) (bring-up, rosbridge,
   *Play* on the pendant) and join its Wi‑Fi network.
4. **Use it**: activate the environment (`.venv\Scripts\activate` or
   `source .venv/bin/activate`) or call its Python directly
   (`.venv\Scripts\python` or `.venv/bin/python`).

Only one `Robot` per Python process: roslibpy's event loop cannot be restarted
after a connection is closed.

---

## Settings file (`ur10_config.yaml`)

`ur10_config.yaml`, at the repository root, holds the values you may want to
tune, with a comment on every line. The web panel, `Robot()` and the examples
read it at start-up; restart them after editing.

| Section | Contents |
|---|---|
| `robot` | rosbridge host and port, reachability timeout |
| `tcp` | TCP relative to the flange: position [mm], rotation vector [deg] |
| `workspace` | box the TCP must stay in, UR base frame [mm]; the panel's workspace editor rewrites it |
| `safety` | stale joint-state limit, stop duration, streaming look-ahead, largest IK jump |
| `api` | default speed and acceleration caps of `Robot` (mm/s, deg/s, …) |
| `gripper` | gripper status topics |
| `link_check` | the start-up check of `robot.check_command_link()` |
| `plots` | history window and refresh rate of the live plots, folder of the saved figures |
| `panel` | web panel: address, jog rates, sliders, step sizes, presets |
| `examples` | defaults of the three examples: targets, gains, speeds, path shape, dead bands |

- **Units**: mm, deg, s, N and Nm, as on the pendant and in the panel. The
  Python API itself uses SI units (m, rad); `Robot` converts.
- **Checked**: an unknown key (e.g. a typo) or a wrong value stops the program
  with a `ConfigError` naming the key, so a limit is never silently ignored.
  Missing keys take the built-in defaults (`ur10api/config.py`).
- **Precedence**: keyword arguments of `Robot(...)` and command-line options of
  the examples override the file.
- **Another file**: set the environment variable `UR10_CONFIG` to its path, or
  pass `Robot(config="other.yaml")`.
- The original controllers (`controllers/`) do not read it; they keep
  UR_CONTROL's constants.

```python
from ur10api import load_config

cfg = load_config()                      # nested dicts, in the file's units
cfg["examples"]["force_teleop"]["gain"]  # 4.0 (mm/s per N)
cfg.workspace_limits()                   # {"x": (-0.8, 0.5), ...} in metres
cfg.tcp_offset()                         # (x, y, z [m], rx, ry, rz [rad])
```

Put the settings of your own programs in your own file or code: unknown keys in
`ur10_config.yaml` are rejected.

---

## Quick start

```python
import numpy as np
from ur10api import Rate, Robot
from ur10api.transforms import rotz, trans

with Robot() as robot:                                # connects and waits for /joint_states
    robot.check_command_link(report=print)             # commands reach the arm promptly (see below)
    s = robot.state()
    print("joints [deg]:", np.degrees(s.q).round(1))
    print("TCP [mm]:", (s.position * 1000).round(1))
    print("force [N]:", s.force.round(1))

    # Position commands (blocking): absolute, relative in the tool frame, joints
    robot.move_tcp(s.tcp @ trans(0.05, 0, 0) @ rotz(15), speed=0.2)
    robot.move_tcp_by(dp=(0, 0, 0.03), frame="base")
    robot.move_joints(s.q, speed=0.3)                   # back to where we started

    # Velocity command: 20 mm/s along the base X axis for 2 s
    rate = Rate(25)
    for _ in range(50):
        robot.set_tcp_velocity(v=(0.02, 0, 0))
        rate.sleep()
    robot.stop()

    robot.open_gripper()
```

Leaving the `with` block disconnects. If it is left because of an error or `Ctrl+C`,
the robot is braked first. `Robot("192.168.0.200")` connects to another address
than the one in the settings file.

---

## Concepts

### Frames and units

| Item | Convention |
|---|---|
| UR base frame | `campero_ur10_base` = the pendant's *Base* = UR_CONTROL's base frame. All poses are expressed in it |
| Flange | `campero_ur10_tool0`: what the pendant shows with an all-zero TCP |
| TCP | relative to the flange, like the pendant's TCP setting: `tcp:` in `ur10_config.yaml` (mm, deg), or `Robot(tcp=(x, y, z [m], rx, ry, rz [rad]))`. Default: 150 mm along the tool Z axis, as in UR_CONTROL |
| Joints | J1..J6 = shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, in rad |
| Poses | 4×4 numpy arrays: rotation `T[:3, :3]`, position `T[:3, 3]` [m] |
| Roll/pitch/yaw | `R = Rz(yaw) · Ry(pitch) · Rx(roll)`, UR's RPY convention |
| Force/torque | Robotiq FT 300, N and Nm, in the flange (tool) frame. `RobotState.force_base` turns the force into the base frame |

### Position commands

`move_joints`, `move_tcp` and `move_tcp_by` send **one** trajectory to the target
and, by default, wait until the robot is there. `speed` is the average speed of the
joint that moves the most [rad/s], and all joints arrive together (UR_CONTROL's
semantics). The TCP path of `move_tcp` is therefore not a straight line; for
straight paths, use velocity commands.

### Start-up delay of commands

Right after connecting, the joint states arrive at once, but the first commands
can be lost or take a few seconds to reach the arm (rosbridge and ROS are still
setting up the publisher towards the Campero nodes). `robot.check_command_link()`
waits for that to settle before your program moves anything:

1. it moves wrist 3 back and forth by 1° (0.4 s per move), turning only the gripper;
2. for each move it measures how long the measured wrist takes to get halfway,
   minus the 0.2 s the motion itself needs: the command delay; a move without an
   answer is sent again;
3. it returns once 3 moves in a row had a delay below 0.3 s and the wrist is back
   where it started (a late command must not move it again), and raises
   `NotReady` after 20 s.

The examples run it first; the values are in the `link_check:` section.

### Velocity commands (streaming)

UR_CONTROL only accepts position targets, so `set_tcp_velocity(v, w)` and
`set_joint_velocity(qd)` send a short target lying `Robot.lookahead` (0.25 s) ahead
of the measured state, with that same duration. Call them **periodically** (10–50
Hz): the robot moves at the commanded velocity while you keep calling, and stops
about 0.25 s after the last call (a dead-man behaviour). A zero velocity brakes.
Velocities are capped (`max_linear_speed`, `max_angular_speed`,
`max_joint_speed`) and ramped (`max_linear_accel`, `max_angular_accel`).

### Safety checks

- Nothing is sent unless `/joint_states` is fresh (younger than
  `Robot.stale_after`, 0.5 s); otherwise `NotReady` is raised. roslibpy would
  queue commands during a disconnection and replay them later.
- **Workspace**: Cartesian targets must be inside `robot.workspace`, a box for the
  TCP in the UR base frame (the `workspace:` section of `ur10_config.yaml`, which
  the web panel also uses), or closer to it than the current pose.
  - `move_tcp` raises `MotionRefused`.
  - A velocity command stops at a face of the box and can still slide along it;
    `last_warning` then names the limit that holds the TCP.
  - Joint commands are not checked, as in UR_CONTROL.
- IK: targets out of reach, singular, or needing a jump of more than
  `Robot.max_joint_step` (20°) in one streamed step are refused.
- `stop()` replaces whatever is running with a short braking trajectory. It is
  **not an emergency stop**: keep the teach pendant's E-stop within reach.

---

## API reference

### `Robot`

```python
Robot(host=None, port=None, *, tcp=None, workspace=None,
      max_linear_speed=None, max_angular_speed=None, max_joint_speed=None,
      max_linear_accel=None, max_angular_accel=None, config=None, timeout=10.0)
```

Every argument left as `None` comes from `ur10_config.yaml` (`robot:`, `tcp:`,
`workspace:` and `api:` sections); the ones you pass win, in SI units: `tcp` as
`(x, y, z [m], rx, ry, rz [rad])`, `workspace` as `{"x": (min, max), ...}` in
metres, speeds in m/s and rad/s, accelerations in m/s² and rad/s². `config` is
another settings file (path) or a `Config` from `load_config()`.

Connects through `UR10Control` and waits (up to `timeout` s) for the first joint
state. Like every UR_CONTROL controller, connecting **zeroes the force/torque
sensor**. Raises `NotReady` if rosbridge cannot be reached or no joint states
arrive, and `ConfigError` if the settings file is invalid.

| Attribute | Meaning |
|---|---|
| `arm` | the `ArmModel` (calibrated kinematics, with this robot's TCP) |
| `workspace` | the `Workspace` box; change it with `robot.workspace.set({...})` |
| `config` | the settings in use (`Config`, file units) |
| `lookahead` | 0.25 s, horizon of streamed velocity commands (`safety:`) |
| `stale_after` | 0.5 s, maximum age of joint states for sending commands |
| `max_joint_step` | 20° (in rad), largest IK jump accepted in one streamed step |
| `stop_brake` | 0.4 s, duration of the `stop()` trajectory |
| `max_*_speed`, `max_*_accel` | caps, SI units (can be changed) |
| `last_warning` | why the last velocity command was not sent, or which workspace limit holds the TCP ("" otherwise) |

**Reading the state**

| Method | Returns |
|---|---|
| `state()` | a `RobotState` snapshot (below) |
| `q` (property) | joint angles [rad] |
| `tcp_pose()` / `flange_pose()` | 4×4 poses in the UR base frame |
| `wrench()` | `(force [N], torque [Nm])` in the tool frame |
| `gripper()` | `GripperState` or `None` without gripper feedback |
| `is_ready()` | `True` while connected with fresh joint states |

**Position commands**

| Method | Does |
|---|---|
| `move_joints(q, speed=0.3, wait=True, timeout=None, callback=None)` | joint-space move to `q` [rad]; returns the duration [s] |
| `move_tcp(T, speed=0.3, wait=True, timeout=None, callback=None)` | TCP to pose `T` via calibrated IK (joint-space motion) |
| `move_tcp_by(dp=(0,0,0), drot=(0,0,0), frame="base", speed=0.3, wait=True, callback=None)` | relative TCP move: `dp` [m], rotation vector `drot` [rad], in `"base"` or `"tool"` axes |
| `wait_until_reached(q, tol=0.002, timeout=10.0, callback=None)` | block until the joints are within `tol` of `q` and at rest |

With `wait=True`, `callback(state)` is called about 50 times per second while
waiting (e.g. `callback=view.update`). `Ctrl+C` while waiting brakes the robot.

**Velocity commands** (call periodically)

| Method | Does |
|---|---|
| `set_tcp_velocity(v=(0,0,0), w=(0,0,0), frame="base")` | TCP linear `v` [m/s] and angular `w` [rad/s] velocity; `frame="base"` (base axes, rotation about the TCP) or `"tool"`. Returns `False` if nothing was sent (`last_warning` says why) |
| `set_joint_velocity(qd)` | joint velocities [rad/s] (6 values) |

**Other commands**

| Method | Does |
|---|---|
| `check_command_link(report=None, …)` | wiggle wrist 3 until commands reach the arm promptly (see [Start-up delay of commands](#start-up-delay-of-commands)); returns the measured delays [s], raises `NotReady` on timeout. `report=print` shows the progress |
| `stop()` | brake to a stop (0.4 s), replacing the current motion; sends nothing when the robot is at rest with nothing pending |
| `open_gripper()` / `close_gripper()` | Robotiq 2F-85 (`UR10Control.send_gripper_cmd`) |
| `zero_force_sensor(settle=0.5)` | SET ZERO of the FT 300, then wait `settle` s; nothing must touch the gripper |
| `close()` | disconnect (automatic at the end of a `with` block) |

### `RobotState`

Frozen snapshot returned by `Robot.state()`.

| Field | Meaning |
|---|---|
| `q`, `qd` | joint angles [rad] and velocities [rad/s] (`qd` is `None` if not published) |
| `tcp`, `flange` | 4×4 poses in the UR base frame |
| `position`, `rotation` | shortcuts for `tcp[:3, 3]` and `tcp[:3, :3]` |
| `force`, `torque` | [N], [Nm] in the tool frame |
| `force_base` | force in the UR base frame |
| `age` | seconds since the last `/joint_states` |
| `wrench_age` | seconds since the last force/torque message (`None`: never) |
| `gripper` | `GripperState` or `None` |

### `GripperState`

`closed` (0 = open, 85 mm … 1 = closed), `gap_mm`, `object_detected`, `moving`,
`active`, `fault` and a readable `text`, from `/robotiq_2f_gripper/input`.

### `Rate`

```python
rate = Rate(25)        # 25 Hz
while running:
    ...                # read the state, compute, command
    rate.sleep()       # waits for the rest of the 40 ms period
```

### `ur10api.transforms`

| Function | Returns |
|---|---|
| `pose(position, rotation=None)` | 4×4 pose; `rotation` is a 3×3 matrix or a rotation vector [rad] |
| `trans(x, y, z)`, `rotx(deg)`, `roty(deg)`, `rotz(deg)` | elementary poses, to compose with `@` |
| `inverse(T)` | inverse pose |
| `displace(T, dp, drot, frame="base")` | `T` moved by `dp` and rotated by `drot` about its own origin, in base or tool axes |
| `pose_error(T_current, T_target)` | `(dp, drot)`: remaining translation and rotation vector, base frame |
| `rotation_angle(T_a, T_b)` | angle between two orientations [rad] |
| `pose_to_xyzrpy(T)` / `xyzrpy_to_pose(v)` | pose ↔ `[x, y, z, roll, pitch, yaw]` |
| `rotation_vector(T)` | the pendant's RX, RY, RZ [rad] |

`T @ trans(0.05, 0, 0)` moves 5 cm along **T's own** X axis (tool frame).
`trans(0.05, 0, 0) @ T` moves along the **base** X axis.

### `ur10api.kinematics`

```python
arm = ArmModel(tcp=(0, 0, 0.15, 0, 0, 0))
T = arm.fk(q)                   # TCP pose; arm.fk(q, tcp=False) for the flange
q = arm.ik(T, seed=current_q)   # raises IKError when out of reach or singular
J = arm.jacobian(q)             # 6x6: [v; w] = J @ qdot, base frame
points = arm.skeleton(q)        # base, joints, flange, TCP (for drawing)

ws = Workspace({"x": (-1.3, -0.3), "y": (-0.45, 0.8), "z": (0.2, 0.8)})
ws.contains(T), ws.distance(T), ws.violations(T), ws.allows(T_now, T_target), ws.clamp(p)
```

The IK starts from UR_CONTROL's analytic solution, which picks the arm
configuration (shoulder up, the branch `UR10Control` uses). Newton steps on the
calibrated model then remove the remaining 3–6 mm. `robot.arm` and
`robot.workspace` are these objects.

### `ur10api.viz.Dashboard`

The live figure of the examples: an overview of the arm, a 3D close-up that
zooms on the task, and time plots on the right. Your control loop runs in a
worker thread, so drawing never delays it. When the run ends the window becomes
a still summary of the whole run, saved as a PNG.

```python
from ur10api.viz import Dashboard, Panel

dash = Dashboard(robot, "Height step", panels=[
    Panel("TCP height", "z [mm]", {"measured": "z", "target": "z_ref"},
          styles={"target": {"ls": "--"}}),
    Panel("Error", "[mm]", {"error": "e"}, hlines={"tolerance": 1.0}),
], paths={"TCP path": ("x", "y", "z")})          # 3D line from logged columns [mm]
dash.scene.add_frame(goal, "goal")                # close-up: fixed frames and paths

def control(stop):                                # runs in the worker thread
    rate, t0 = Rate(25), time.monotonic()
    while not stop.is_set():                      # set on Ctrl+C or when the window closes
        s = robot.state()
        ...                                       # compute and send commands
        p = s.position * 1000
        dash.record(time.monotonic() - t0, x=p[0], y=p[1], z=p[2], z_ref=..., e=...)
        dash.set_status(f"error {...:.1f} mm")    # line under the plots
        rate.sleep()

completed = dash.run(control)                     # False if stopped by the user
robot.stop()
dash.finish(["summary line 1", "summary line 2"]) # full plots + text; PNG in runs/
```

| Item | Meaning |
|---|---|
| `Panel(title, ylabel, lines, styles={}, kind="time", xlabel="time [s]", hlines={})` | `lines` = `{legend label: column}`; `kind="xy"` plots `{label: (x column, y column)}`, e.g. a path in a plane |
| `record(t, **columns)` | log one row at time `t` [s] (thread-safe); lengths in mm |
| `set_frame(name, T)`, `set_marker(name, p)`, `set_vector(name, origin, v)` | moving items in the close-up (e.g. current target, moving reference, force) |
| `scene`, `overview` | the two 3D views (`Scene`: `add_frame`, `add_path`, …) |
| `column(name)`, `times()` | the logged data, for your own statistics |
| `run(control)` | run `control(stop_event)` in a thread and keep the window live; exceptions of `control` are raised again here |
| `finish(summary, save_as=None)` | still summary; saved to `plots.save_dir` (`runs/`) unless `save_dir` is empty |

History window, refresh rate and folder are in the `plots:` section.

### `ur10api.viz.FrameView`

A single 3D view refreshed from your own loop (simpler, but the drawing runs in
your loop):

```python
view = FrameView(robot.arm, title="My test", workspace=robot.workspace)
view.add_frame(T, "goal")               # fixed frame (dashed)
view.add_path(points)                   # fixed polyline (N x 3)
view.set_vector("force", origin, vec)   # arrow-like segment, updated at will
view.update(robot.state())              # arm + TCP frame + trail; redraws ≤ 8 times/s
view.hold()                             # keep the window open at the end
```

`update` returns immediately between redraws, so it can be called in every cycle
of a control loop.

### Errors

| Exception | When |
|---|---|
| `RobotError` | base class; also a `move_*` that does not arrive in time |
| `NotReady` | no connection, stale joint states, or commands not reaching the arm in `check_command_link` (nothing was sent) |
| `ConfigError` | invalid `ur10_config.yaml` (names the key) |
| `MotionRefused` | workspace, IK or joint limits (nothing was sent) |
| `IKError` | `ArmModel.ik` without solution |

---

## Writing a control loop

```python
import numpy as np
from ur10api import Rate, Robot, transforms as tf

with Robot(max_linear_speed=0.05) as robot:
    robot.check_command_link()                        # commands arrive without delay
    goal = robot.tcp_pose() @ tf.trans(0, 0, 0.05)    # 5 cm along the tool Z axis
    rate = Rate(25)
    try:
        while True:
            state = robot.state()                      # 1. measure
            dp, drot = tf.pose_error(state.tcp, goal)  # 2. compute
            if np.linalg.norm(dp) < 0.001:
                break
            robot.set_tcp_velocity(1.5 * dp, 1.5 * drot)   # 3. command
            rate.sleep()                               # 4. wait for the next cycle
    finally:
        robot.stop()
```

- **Rate**: 20–50 Hz works well. Commands target 0.25 s ahead, so a late cycle is
  harmless, but the robot stops if the loop stops.
- **Stopping**: put `robot.stop()` in a `finally` block (or rely on the `with`
  block, which brakes on errors). `Ctrl+C` raises `KeyboardInterrupt` in your
  loop.
- **Check the result**: velocity commands return `False` when refused (workspace,
  IK); `robot.last_warning` says why. Position commands raise.
- **Speeds**: start slow. The caps (`api:` section, 100 mm/s and 30°/s by
  default, or the constructor's arguments) apply to every velocity command.
- **Latency**: states arrive over Wi‑Fi. A feedback loop sees the robot a few tens
  of milliseconds late, so use moderate gains (1–3 1/s on position errors). Rates
  above ~50 Hz do not help: every command replaces the trajectory being executed,
  and each message has to cross rosbridge.
- **Plots**: to watch and record a run, use a `Dashboard`: it runs your loop in a
  worker thread so that drawing never delays it.

---

## Examples

Run them from the repository root with the environment's Python. `--help` lists
all options. All three:

- take their defaults from the `examples:` section of `ur10_config.yaml`; the
  command-line options override them, **in the same units (mm, deg, N)**;
- first run `robot.check_command_link()` (wrist 3 wiggles by 1°);
  `--no-link-check` skips it;
- show a live **dashboard**: an overview of the arm, a 3D close-up of the task
  with the reference and the path the TCP really followed, and time plots of
  the target against the measured state;
- stop on `Ctrl+C` or when the window is closed (the robot brakes), then turn the
  window into a **summary** of the whole run, saved as a PNG in `runs/`.

### 1. Pose control toward target frames – `examples/01_pose_frames.py`

Target frames are defined relative to the start pose of the TCP
(`examples.pose_frames.targets`: position [mm] and rotation vector [deg]; three
by default), in the **tool frame** (default, e.g. "6 cm along my X axis, turned
20° about my Z") or in the **base frame** (`--frame base`). The TCP visits them
and returns to the start.

- `--mode servo` (default): a Cartesian P-controller, `v = gain·dp`,
  `w = gain·drot` (gain 1.5 1/s) at 25 Hz with `set_tcp_velocity`. The TCP moves
  along straight lines and settles within the tolerances (1 mm / 0.3°).
- `--mode move`: one `move_tcp` per target (joint-space motion).

Plots: the targets (the current one highlighted) and the straight lines between
them; the TCP position relative to the start against the target; the position
and orientation errors; the commanded speed. The summary lists the final error
and time for every target.

```bash
python examples/01_pose_frames.py
python examples/01_pose_frames.py --frame base --mode move --joint-speed 10
```

The default offsets stay within ±6 cm and ±20°, so start from a pose with some
room around it.

### 2. Velocity control along a path – `examples/02_velocity_path.py`

The TCP follows a small flat **circle** (default) or an **infinity** shape
(lemniscate of Gerono) through its start point, in the tool XY plane (horizontal
when the gripper points down) or in the base XY plane (`--plane base`). The speed
ramps up and down smoothly. At `--rate` (25 Hz):

```
v = v_ref(t) + kp · (p_ref(t) − p_measured)        feed-forward + P feedback
w = kr · rotation error to the start orientation    (orientation held)
```

Before moving, the script checks that the path fits in the workspace, and warns
if the path needs more speed than `max_speed`. Plots: the whole reference path,
the moving reference point and the TCP path in 3D; the path in its plane
(reference against measured); the tracking error; the TCP position against the
reference. The summary gives the RMS and maximum tracking error while cruising.

```bash
python examples/02_velocity_path.py                                             # values of ur10_config.yaml
python examples/02_velocity_path.py --shape infinity --size 60 --period 10 --plane base
```

Options: `--shape`, `--size` [mm], `--period` [s per lap], `--laps`, `--plane`,
`--kp` [1/s], `--rate` [Hz]. A few millimetres of tracking error mostly come from
the delay between measuring and moving; faster laps increase it.

### 3. Compliant teleoperation – `examples/03_force_teleop.py`

Push the gripper and the arm follows (admittance control), like
`controllers/force_controller.py` but continuous:

1. the force/torque (tool frame) is low-pass filtered (`--filter`, 0.2);
2. it is turned into the base frame;
3. above a dead band (`--dead-band`, 3 N) it becomes a velocity along the push,
   `v = gain·(|F| − dead band)` (`--gain`, 4 mm/s per N, capped at `--max-speed`,
   50 mm/s);
4. the velocity is sent with `set_tcp_velocity` at 50 Hz. Without a push, the arm
   ramps down and holds.

The sensor is zeroed after the link check: **do not touch the gripper until
"Push the gripper" appears**. `--rotate` also turns the TCP with the measured
torque (dead band 0.8 Nm, at most `--max-rotation` 30° from the start).
Rotating changes how the gripper's weight loads the sensor, so keep rotations
small and zero again if it drifts. If the arm moves against your push, add
`--invert`.

Plots: the TCP path and the force as an arrow (1 cm per N) in 3D; the force in
the base frame against the dead band; the commanded velocity; the displacement
from the start. Stop with `Ctrl+C` or by closing the window; the summary gives
the path length, the peak force and the peak speed.

```bash
python examples/03_force_teleop.py
python examples/03_force_teleop.py --rotate
```

The workspace box still applies: the TCP stops at its faces and slides along
them.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| The first commands take seconds to move the arm | Normal right after connecting; `robot.check_command_link()` waits for it (the examples do). If it times out, check the Wi‑Fi and the bring-up on the Campero. |
| `ConfigError: ur10_config.yaml: …` | The message names the key: fix its value or spelling (see the comments in the file). |
| `NotReady: cannot reach rosbridge …` | rosbridge not running, wrong network, or the `.local` name not resolved: see the README's troubleshooting, or pass the IP. |
| `NotReady: … no /joint_states arrived` | The UR bring-up is not running or *Play* was not pressed. |
| `NotReady: the joint states are … old` | The Wi‑Fi dropped or the driver stopped; commands are blocked until data is fresh again. |
| `MotionRefused: target outside the workspace` | The TCP target is out of the box; move elsewhere or widen `workspace:` in `ur10_config.yaml` (or the panel's editor). |
| `set_tcp_velocity` returns `False` | Read `robot.last_warning` (workspace, IK jump, no solution). |
| `ModuleNotFoundError: ur10api` | Use the environment's Python, run the setup script again (it writes `ur_control.pth`), or run from the repository root. |
| No plot window | Install Tk (`sudo apt install python3-tk` on Ubuntu); on Windows reinstall Python with *tcl/tk*. |
| The arm drifts in the teleop example | Zero the sensor again with the gripper free (restart the example), or raise `--dead-band`. |
| Second `Robot(...)` in the same script fails | One `Robot` per process: reuse the first one. |
