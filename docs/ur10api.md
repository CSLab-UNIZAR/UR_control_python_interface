# ur10api – Python interface for your own control code

`ur10api` is a small Python package for writing your own control programs for the
UR10 on the Campero. One class, `Robot`, reads the robot state (joints, TCP pose,
force/torque, gripper) and sends commands (joint and pose moves, TCP or joint
velocities, gripper, sensor zeroing, stop). Helper modules cover poses, calibrated
kinematics and live 3D plots.

```python
from ur10api import Robot

with Robot("CMP00-180723AD.local") as robot:
    print(robot.tcp_pose())                # 4x4 TCP pose in the UR base frame
    robot.move_tcp_by(dp=(0, 0, 0.05))     # 5 cm up
    robot.close_gripper()
```

Three example programs in [`examples/`](../examples) show typical control
processes: pose control toward target frames, velocity control along a path, and
compliant teleoperation with the force sensor.

---

## Contents

1. [How it fits in the repository](#how-it-fits-in-the-repository)
2. [Installation and setup](#installation-and-setup)
3. [Quick start](#quick-start)
4. [Concepts](#concepts)
5. [API reference](#api-reference)
6. [Writing a control loop](#writing-a-control-loop)
7. [Examples](#examples)
8. [Troubleshooting](#troubleshooting)

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
| `ur10api.transforms` | building and comparing 4×4 poses |
| `ur10api.kinematics` | `ArmModel` (FK, IK, Jacobian), `Workspace` |
| `ur10api.viz` | `FrameView`, a live 3D plot (matplotlib) |

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

## Quick start

```python
import numpy as np
from ur10api import Rate, Robot
from ur10api.transforms import rotz, trans

with Robot("CMP00-180723AD.local") as robot:          # connects and waits for /joint_states
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
the robot is braked first.

---

## Concepts

### Frames and units

| Item | Convention |
|---|---|
| UR base frame | `campero_ur10_base` = the pendant's *Base* = UR_CONTROL's base frame. All poses are expressed in it |
| Flange | `campero_ur10_tool0`: what the pendant shows with an all-zero TCP |
| TCP | set per `Robot` (`tcp=`), relative to the flange like the pendant's TCP setting: `(x, y, z [m], rx, ry, rz [rad])`. Default `(0, 0, 0.15, 0, 0, 0)`, as in UR_CONTROL |
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
  TCP in the UR base frame (default: UR_CONTROL's `WORKSPACE_LIMITS`), or closer
  to it than the current pose.
  - `move_tcp` raises `MotionRefused`.
  - A velocity command stops at a face of the box and can still slide along it.
  - Joint commands are not checked, as in UR_CONTROL.
- IK: targets out of reach, singular, or needing a jump of more than
  `Robot.max_joint_step` (0.35 rad) in one streamed step are refused.
- `stop()` replaces whatever is running with a short braking trajectory. It is
  **not an emergency stop**: keep the teach pendant's E-stop within reach.

---

## API reference

### `Robot`

```python
Robot(host="CMP00-180723AD.local", port=9090, *, tcp=(0, 0, 0.15, 0, 0, 0), workspace=None,
      max_linear_speed=0.10, max_angular_speed=0.5, max_joint_speed=0.8,
      max_linear_accel=0.5, max_angular_accel=2.0, timeout=10.0)
```

Connects through `UR10Control` and waits (up to `timeout` s) for the first joint
state. Like every UR_CONTROL controller, connecting **zeroes the force/torque
sensor**. Raises `NotReady` if rosbridge cannot be reached or no joint states
arrive. `workspace` is `{"x": (min, max), "y": ..., "z": ...}` in metres.

| Attribute | Meaning |
|---|---|
| `arm` | the `ArmModel` (calibrated kinematics, with this robot's TCP) |
| `workspace` | the `Workspace` box; change it with `robot.workspace.set({...})` |
| `lookahead` | 0.25 s, horizon of streamed velocity commands |
| `stale_after` | 0.5 s, maximum age of joint states for sending commands |
| `max_joint_step` | 0.35 rad, largest IK jump accepted in one streamed step |
| `max_*_speed`, `max_*_accel` | caps given to the constructor (can be changed) |
| `last_warning` | why the last velocity command was not sent ("" if it was) |

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

### `ur10api.viz.FrameView`

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
| `NotReady` | no connection or stale joint states (nothing was sent) |
| `MotionRefused` | workspace, IK or joint limits (nothing was sent) |
| `IKError` | `ArmModel.ik` without solution |

---

## Writing a control loop

```python
import numpy as np
from ur10api import Rate, Robot, transforms as tf

with Robot("CMP00-180723AD.local", max_linear_speed=0.05) as robot:
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
- **Speeds**: start slow. The constructor caps (`max_linear_speed=0.10` m/s,
  `max_angular_speed=0.5` rad/s) apply to every velocity command.
- **Latency**: states arrive over Wi‑Fi. A feedback loop sees the robot a few tens
  of milliseconds late, so use moderate gains (1–3 1/s on position errors).

---

## Examples

Run them from the repository root with the environment's Python. `--help` lists
all options. Each prints what it does; `Ctrl+C` brakes and exits.

### 1. Pose control toward three frames – `examples/01_pose_frames.py`

Three target frames are defined relative to the start pose of the TCP: in the
**tool frame** (default, e.g. "6 cm along my X axis, turned 20° about my Z") or in
the **base frame** (`--frame base`). The TCP visits them and returns to the
start, while a live 3D view shows the arm, the targets (dashed), the moving TCP
frame and its path. It prints the final position and orientation error at each
target.

- `--mode servo` (default): a Cartesian P-controller, `v = 1.5·dp`, `w = 1.5·drot`
  at 25 Hz with `set_tcp_velocity`. The TCP moves along straight lines and settles
  within 1 mm / 0.3°.
- `--mode move`: one `move_tcp` per target (joint-space motion).

```bash
python examples/01_pose_frames.py --host CMP00-180723AD.local
python examples/01_pose_frames.py --frame base --mode move --joint-speed 0.2
```

Edit `TARGETS` at the top of the script to change the frames. The default
offsets stay within ±6 cm and ±20°, so start from a pose with some room around it.

### 2. Velocity control along a path – `examples/02_velocity_path.py`

The TCP follows a small flat **circle** (default) or an **infinity** shape
(lemniscate of Gerono) through its start point, in the tool XY plane (horizontal
when the gripper points down) or in the base XY plane (`--plane base`). The speed
ramps up and down smoothly. Every 40 ms:

```
v = v_ref(t) + kp · (p_ref(t) − p_measured)       feed-forward + P feedback
w = 2 · rotation error to the start orientation    (orientation held)
```

The script checks that the path fits in the workspace before moving. At the end
it prints the RMS and maximum tracking error while cruising, and plots the
reference against the measured path and the error over time.

```bash
python examples/02_velocity_path.py --host CMP00-180723AD.local                  # 50 mm circle, 8 s per lap, 2 laps
python examples/02_velocity_path.py --shape infinity --size 0.06 --period 10 --plane base
```

Options: `--size` [m], `--period` [s per lap], `--laps`, `--kp` [1/s], `--rate`
[Hz]. A few millimetres of tracking error mostly come from the delay between
measuring and moving; faster laps increase it.

### 3. Compliant teleoperation – `examples/03_force_teleop.py`

Push the gripper and the arm follows (admittance control), like
`controllers/force_controller.py` but continuous:

1. the force/torque (tool frame) is low-pass filtered (`--filter`, 0.2);
2. it is turned into the base frame;
3. above a dead band (`--dead-band`, 3 N) it becomes a velocity along the push,
   `v = gain·(|F| − dead band)` (`--gain`, 0.004 m/s per N, capped at `--max-speed`,
   0.05 m/s);
4. the velocity is sent with `set_tcp_velocity` at 50 Hz. Without a push, the arm
   ramps down and holds.

The sensor is zeroed at the start: **do not touch the gripper during the first 3
s**. `--rotate` also turns the TCP with the measured torque (dead band 0.8 Nm,
at most `--max-rotation` 30° from the start). Rotating changes how the gripper's
weight loads the sensor, so keep rotations small and zero again if it drifts.
`--view` opens the 3D view with the force vector (1 cm per N). If the arm moves
against your push, add `--invert`.

```bash
python examples/03_force_teleop.py --host CMP00-180723AD.local
python examples/03_force_teleop.py --rotate --view
```

The workspace box still applies: the TCP stops at its faces and slides along
them.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `NotReady: cannot reach rosbridge …` | rosbridge not running, wrong network, or the `.local` name not resolved: see the README's troubleshooting, or pass the IP. |
| `NotReady: … no /joint_states arrived` | The UR bring-up is not running or *Play* was not pressed. |
| `NotReady: the joint states are … old` | The Wi‑Fi dropped or the driver stopped; commands are blocked until data is fresh again. |
| `MotionRefused: target outside the workspace` | The TCP target is out of the box; move elsewhere or change `robot.workspace`. |
| `set_tcp_velocity` returns `False` | Read `robot.last_warning` (workspace, IK jump, no solution). |
| `ModuleNotFoundError: ur10api` | Use the environment's Python, run the setup script again (it writes `ur_control.pth`), or run from the repository root. |
| No plot window | Install Tk (`sudo apt install python3-tk` on Ubuntu); on Windows reinstall Python with *tcl/tk*. |
| The arm drifts in the teleop example | Zero the sensor again with the gripper free (restart the example), or raise `--dead-band`. |
| Second `Robot(...)` in the same script fails | One `Robot` per process: reuse the first one. |
