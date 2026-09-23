# UR10 Control – Python interface for the Campero

Python tools to drive the **UR10 arm of the Robotnik Campero** from a remote PC
(Windows or Linux) over Wi‑Fi. The remote PC talks to the ROS workspace running
on the Campero through **rosbridge** (websocket). All robot messages are built
by one class, `UR10Control` (`core/ur_control.py`).

On top of the framework there are:

- a **web control panel** (`webui/`): jog in joint or Cartesian space (velocity or
  step), move to joint or pose targets, open and close the gripper, and read joint
  states, TCP pose, gripper state and force/torque live, all from the browser;
- the original **controllers**: terminal keyboard, 3Dconnexion SpaceMouse, Leap
  Motion hand gestures and force-guided motion.

![Web control panel](docs/webui.png)

---

## Contents

1. [How it works](#how-it-works)
2. [Repository layout](#repository-layout)
3. [Quick start on Windows](#quick-start-on-windows)
4. [Robot-side start-up (Campero PC)](#robot-side-start-up-campero-pc)
5. [Web control panel](#web-control-panel)
6. [Other controllers](#other-controllers)
7. [Framework reference](#framework-reference)
8. [Configuration](#configuration)
9. [Safety](#safety)
10. [Troubleshooting](#troubleshooting)
11. [Linux](#linux)
12. [Credits](#credits)

---

## How it works

```
 Remote PC (Windows / Linux)                           Campero PC (ROS 1)
┌──────────────────────────────────┐   Wi-Fi         ┌──────────────────────────────────┐
│ webui/ or controllers/           │   websocket     │ rosbridge_websocket (port 9090)  │
│   └── UR10Control (core/)  ──────┼──── :9090 ─────►│   /pub_ik_trajectory ──► UR10    │
│         roslibpy                 │◄────────────────┼── /joint_states                  │
│                                  │                 │   /pub_gripper_control ► gripper │
│                                  │◄────────────────┼── /robotiq_ft_sensor             │
└──────────────────────────────────┘                 └──────────────────────────────────┘
```

A node on the Campero subscribes to `/pub_ik_trajectory` and forwards the
requested joint positions to the arm. Every command is a `JointTrajectory` with
a **single point**: six joint positions and a `time_from_start`. The robot
controller interpolates from the current state to that point.

| ROS name | Type | Direction | Content |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | robot → PC | arm joint positions (the last 6 entries of the arrays) |
| `/pub_ik_trajectory` | `trajectory_msgs/JointTrajectory` | PC → robot | 1 point: joints `campero_ur10_*` + `time_from_start` |
| `/pub_gripper_control` | `trajectory_msgs/JointTrajectory` | PC → robot | `campero_robotiq_85_left_knuckle_joint`: `-0.99` open, `0.99` close, 0.4 s |
| `/robotiq_ft_sensor` | `robotiq_ft_sensor/ft_sensor` | robot → PC | `Fx Fy Fz` [N], `Mx My Mz` [Nm] (tool0 frame) |
| `/robotiq_ft_sensor_acc` (service) | `robotiq_ft_sensor/sensor_accesor` | PC → robot | `command_id: 8` = SET ZERO |
| `/clock` | `rosgraph_msgs/Clock` | robot → PC | time stamp copied into the command headers |
| `/Robotiq2FGripperRobotInput` | `robotiq_2f_gripper_control/Robotiq2FGripper_robot_input` | robot → PC | *optional*, read-only gripper feedback shown by the web panel |

`UR10Control.send_trajectory(target, speed)` accepts a 4×4 TCP pose (solved with
the analytic UR10 inverse kinematics) or, with `art=True`, six joint angles. The
trajectory duration is **largest joint displacement / `speed`**, so `speed` is
the average speed [rad/s] of the joint that moves the most.

---

## Repository layout

```
core/                 framework
  ur_control.py         UR10Control: connection, subscriptions, commands
  ur10_core.py          FK/IK wrappers with the TCP offset, workspace limits
  kinematics_utils.py   analytic UR10 kinematics (DH parameters)
  conversions.py        pose / quaternion / RPY conversions
  plots.py, spacemouse.py, leap_gestures*.py, trajectory.py, ros_structs.py
controllers/          original controllers (keyboard, SpaceMouse, Leap Motion, force)
webui/                web control panel  ->  python -m webui
  config.py             speeds, limits, rates, presets (edit here)
  robot.py              connection + monitoring + motion logic
  kinematics.py         pose helpers built on core/ur10_core.py
  server.py             local HTTP server and JSON API
  static/               page, styles and script served to the browser
other/                SpaceMouse button maps
docs/                 images for this README
requirements.txt      pinned Python dependencies
setup.bat, setup.ps1  Windows: create .venv and install the requirements
run_webui.bat         Windows: start the web panel
install.sh            Ubuntu installer (original, incl. Leap Motion bindings)
```

The panel does not modify `core/`: it calls the same `UR10Control` methods as the
other controllers, so the Campero receives exactly the same messages.

---

## Quick start on Windows

1. **Install Python 3.12** from [python.org](https://www.python.org/downloads/windows/)
   (keep the default *py launcher* option). Python 3.10–3.13 work; 3.14 does not,
   because the pinned numpy/scipy have no wheels for it.
2. **Get the code**
   ```powershell
   git clone https://github.com/nachocz/UR10_contro_python_interface_campero.git
   cd UR10_contro_python_interface_campero
   ```
3. **Create the environment**: double-click `setup.bat` (or run `.\setup.ps1` in
   PowerShell). It picks a supported Python, creates `.venv`, installs
   `requirements.txt` and checks that the framework imports.
   Options: `-Recreate` (rebuild `.venv`), `-Python 3.11` (force a version).
4. **Start the robot side** ([next section](#robot-side-start-up-campero-pc)) and
   connect the PC to the Campero's Wi‑Fi network.
5. **Start the panel**: double-click `run_webui.bat`. The browser opens
   <http://localhost:8080> and the panel connects to `CMP00-180723AD.local:9090`.
   Close the console window (or press `Ctrl+C` in it) to quit.

Manual equivalent of steps 3 and 5:

```powershell
py -3.12 -m venv .venv
.venv\Scripts\Activate.ps1          # cmd.exe: .venv\Scripts\activate.bat
python -m pip install -r requirements.txt
python -m webui --host CMP00-180723AD.local
```

> If PowerShell refuses to run `Activate.ps1`, allow local scripts once with
> `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`, or use `cmd.exe`.
> `setup.bat` and `run_webui.bat` do not need activation.

---

## Robot-side start-up (Campero PC)

Summary of the procedure in the internship report (`memoria_practicas.pdf`,
section 4.3). The VNC and pendant passwords are in the report and are not
reproduced here.

1. **Pendant program** (through VNC): run `~/start_ur_vnc.sh`, open *VNC Viewer*
   → *ARM UR10* → *Initialization screen* → *Exit* → *Run Program* → *File* →
   *Load Program* → `ros_ur_robot_driver_empty_program.urp` → *Open*.
   Do **not** press *Play* yet.
2. **Kill leftover ROS processes**
   ```bash
   cd catkin_campero_adrian_ws/
   bash kill_by_name.sh bringup
   bash kill_by_name.sh rosbridge
   ```
3. **Gripper**, in three terminals (each after `cd catkin_campero_adrian_ws/ && source devel/setup.bash`):
   ```bash
   roscore
   rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB_GRIPPER
   rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py   # type r (reset), then a (activate)
   ```
4. **Bring-up and rosbridge**, in two terminals:
   ```bash
   roslaunch campero_robot_real_bring_up campero_ur10_bringup.launch
   roslaunch rosbridge_server rosbridge_websocket.launch
   ```
5. Press **Play** on the pendant (VNC).

Tips from the report: rosbridge often misbehaves on successive connections, so
**restart only the rosbridge launch** between sessions. If the arm does not
connect to the Campero CPU, power the CPU first and the arm about 5 s later.

---

## Web control panel

```text
run_webui.bat [options]        or        python -m webui [options]

  --host HOST          rosbridge host              (default CMP00-180723AD.local)
  --port PORT          rosbridge port              (default 9090)
  --http-port PORT     port of the panel           (default 8080)
  --listen ADDRESS     panel address; 0.0.0.0 lets other devices open it (default 127.0.0.1)
  --no-connect         start without connecting (connect later from the top bar)
  --no-browser         do not open the browser
  --no-gripper-feedback   do not subscribe to /Robotiq2FGripperRobotInput
```

### Connection

The panel connects at start-up; the host and port can also be edited in the top
bar before pressing **Connect**. Connecting creates a `UR10Control`, which
(as in every controller) **zeroes the force/torque sensor**. The status pill shows:

| Status | Meaning |
|---|---|
| 🟢 Connected | joint states arrive; commands are enabled |
| 🟠 Waiting for /joint_states | rosbridge answers but the UR driver does not publish (bring-up not running, *Play* not pressed) |
| 🟠 No fresh robot data | no `/joint_states` for 0.5 s: all motion is blocked |
| 🔴 Connection lost | roslibpy reconnects automatically; motion stays blocked |
| 🔴 Not connected / error | the error explains why (name not resolved, timeout, refused, …) |

One robot connection per run: to change robot, restart the panel.

### Robot state (left column)

- **Joints**: J1–J6 in degrees and radians, plus velocity (°/s).
- **TCP pose**: position [mm] and roll/pitch/yaw [°] of the TCP in the UR10 base
  frame, and whether it lies inside the workspace limits.
- **Gripper**: last command sent, and feedback (percentage closed, object detected, faults) when
  `/Robotiq2FGripperRobotInput` is published. Otherwise it shows *no feedback topic*.
- **Force / torque**: current values and a 10 s chart; **zero sensor** calls the
  SET ZERO service.
- **copy rad / copy SI** copy the joints (rad) or the pose `[x, y, z, roll, pitch, yaw]`
  (m, rad) as a Python list.

### Jog tab: velocity and step commands

| Setting | Options |
|---|---|
| Space | **Cartesian** (TCP) or **Joint** |
| Frame (Cartesian) | **Base frame**: axes of the UR10 base, rotations about the TCP · **Tool frame**: TCP axes |
| Mode | **Continuous (hold)**: velocity command, moves while held · **Step (click)**: one increment per click |
| Speeds | linear 1–100 mm/s, angular 1–30 °/s, joint 1–30 °/s |
| Step sizes | 1–50 mm, 0.5–10 °, 0.5–10 ° (joints) |

Hold a **−/+** button, or its key. The value column shows the live coordinate of
each axis.

| Keys | Cartesian | Joint |
|---|---|---|
| `Q` / `A` | +X / −X | +J1 / −J1 |
| `W` / `S` | +Y / −Y | +J2 / −J2 |
| `E` / `D` | +Z / −Z | +J3 / −J3 |
| `R` / `F` | +Rx / −Rx | +J4 / −J4 |
| `T` / `G` | +Ry / −Ry | +J5 / −J5 |
| `Y` / `H` | +Rz / −Rz | +J6 / −J6 |
| `O` / `C` | open / close gripper | |
| `Esc` / `Space` | **STOP** | |

Jog keys work on the Jog tab when *Keyboard jog* is ticked and no text field has
focus; `Esc` always stops.

**How velocity jogging works.** `UR10Control` only accepts position targets, so a
held jog streams short targets at 20 Hz. Each target lies 0.25 s ahead of the
measured state and is sent with `time_from_start = 0.25 s`, so the robot moves at
the selected speed. Speed ramps up over 0.3 s, and the robot stops within about
0.25 s of release. Cartesian targets go through the same IK and workspace check as
`send_trajectory`. A jog stops by itself when:

- the browser stops sending its keep-alive (tab closed or frozen, window loses focus);
- joint states become stale;
- the IK fails, or jumps more than 20° in one step (singularity or arm-configuration change);
- the target leaves the workspace limits.

In Step mode, clicks made while the previous step is still moving are chained, so
N clicks give exactly N steps.

### Move to joints / Move to pose: position commands

- **Move to joints**: type J1–J6 targets (°) or load a pose from the list. The list
  holds the *Force-control start* preset (report §5.3) plus your own poses: *Save
  current…* stores the present joints in `saved_poses.json` (repository root;
  commit it to share poses).
- **Move to pose**: type X, Y, Z [mm] and roll, pitch, yaw [°] of the TCP in the
  base frame (*Copy current* fills in the present pose).
- **Max joint speed** is the `speed` argument of `send_trajectory` (in °/s).
- The **preview** shows the largest joint motion and the duration. For pose
  targets it also shows the IK solution; for joint targets, the resulting TCP
  position. Moves larger than 45°, or ending outside the workspace, ask for
  confirmation.
- A pose target is reached with one joint-space motion (like every
  `send_trajectory` call), so **the TCP does not travel in a straight line**. For
  straight paths, jog in Cartesian space.

### STOP

**STOP** (button, `Esc` or `Space`) cancels jogs. It then sends a short braking
trajectory: current joints plus half a braking distance, over 0.4 s. This replaces
whatever the robot is executing. STOP is only sent while the robot data is fresh,
and it is **not an emergency stop**: keep the teach-pendant E-stop within reach.

### Log

Commands sent, warnings (workspace, IK, stopped jogs), connection events and
messages printed by `UR10Control` (e.g. *Out of workspace limits*) are listed at
the bottom of the page and in the console.

---

## Other controllers

Run from the repository root with the environment active
(`.venv\Scripts\activate`). They connect to `CMP00-180723AD.local`; the host is
hard-coded near the top of each script.

```powershell
python -m controllers.key_controller          # terminal keyboard control
python -m controllers.spacemouse_controller   # 3Dconnexion SpaceMouse
python -m controllers.leap_controller         # Leap Motion hand gestures
python -m controllers.force_controller        # hand guiding with the F/T sensor
```

| Controller | Controls |
|---|---|
| `key_controller` | `x`/`y`/`z` select axis · `↑`/`↓` move 5 cm or rotate 0.1 rad · `r` translation ↔ rotation · `o`/`c` gripper · `q` quit |
| `spacemouse_controller` | puck moves the TCP (or the selected joint); button maps in `other/tcp_buttons.jpg` and `other/joint_buttons.jpg` |
| `leap_controller` | left hand: 👌 OKAY enable / reset reference · 🖐 OPEN_HAND open gripper · 👍 THUMBS_UP close · ☝ POINTING gripper-only mode · ✌ PEACE leave that mode / disable. Asks at start-up whether to move one axis at a time |
| `force_controller` | push the gripper: forces above 2.5 N move it 5 cm towards the force; torques above 0.7 Nm rotate it (max 30° from the start orientation). `Ctrl+C` to stop |

Before the force controller, move the arm to the *Force-control start* pose
(report §5.3; it is a preset in the web panel) to keep the camera and cables
clear.

**Device notes**

- **SpaceMouse**: `pyspacemouse` loads the native *hidapi* library. On Windows,
  download `hidapi-win.zip` from the [hidapi releases](https://github.com/libusb/hidapi/releases)
  and put `x64\hidapi.dll` in the repository folder (or any folder on `PATH`). On
  Linux, stop the device from moving the mouse pointer with
  `sudo evtest --grab /dev/input/eventXX` (find XX with `cat /proc/bus/input/devices`),
  and if permissions fail run `sudo chmod 666 /dev/hidraw*`.
- **Leap Motion**: install the Ultraleap *Hand Tracking* software, then build the
  [leapc-python-bindings](https://github.com/ultraleap/leapc-python-bindings) inside `.venv`:
  ```powershell
  git clone https://github.com/ultraleap/leapc-python-bindings.git
  cd leapc-python-bindings
  pip install -r requirements.txt
  python -m build leapc-cffi
  pip install leapc-cffi/dist/leapc_cffi-0.0.1.tar.gz
  pip install -e leapc-python-api
  ```
  On Windows this needs the Microsoft C++ Build Tools; see the bindings README if
  the Ultraleap SDK is not in its default location.

---

## Framework reference

### `UR10Control` (`core/ur_control.py`)

```python
from core.ur_control import UR10Control

ur = UR10Control(host_ip="CMP00-180723AD.local", port=9090, tcp_mode=True)
# connects, subscribes to /joint_states, /clock and /robotiq_ft_sensor, zeroes the F/T sensor

ur.joint_states      # np.ndarray(6), J1..J6 [rad]; zeros until the first /joint_states arrives
ur.T_current         # 4x4 TCP pose in the base frame (flange pose if tcp_mode=False)
ur.current_force     # np.ndarray(3) [N]
ur.current_torque    # np.ndarray(3) [Nm]

ur.send_trajectory(T_target, speed=0.07)             # Cartesian target: workspace check + IK
ur.send_trajectory(q_target, speed=0.07, art=True)   # joint target [rad], no workspace check
ur.send_trajectory_smooth(T_target, dt)              # Cartesian target reached in dt seconds (no workspace check)
ur.send_gripper_cmd("Open")                          # or "Close"
ur.zero_ft_sensor()
ur.close_connection()
```

Wait until real joint states have arrived (the controllers sleep 1–2 s after
connecting) before sending anything: until then `joint_states` is all zeros and a
command computed from it would be wrong.

### Conventions

| Item | Value |
|---|---|
| Joint order | J1 `shoulder_pan`, J2 `shoulder_lift`, J3 `elbow`, J4 `wrist_1`, J5 `wrist_2`, J6 `wrist_3` |
| Units | metres, radians, seconds (the web panel displays mm and degrees) |
| Base frame | `campero_ur10_base` (the DH base frame of the UR10) |
| TCP | `tool0` + 0.15 m along the tool Z axis (`tcp_offset` in `core/ur10_core.py`) |
| Roll/pitch/yaw | ZYX: `R = Rz(yaw) · Ry(pitch) · Rx(roll)`, as `SE3.RPY(..., order='zyx')` |
| Workspace limits | x ∈ [−1.30, −0.30] m, y ∈ [−0.45, 0.80] m, z ∈ [0.20, 0.80] m (`WORKSPACE_LIMITS`) |
| Joint limits | ±360° (`core/kinematics_utils.py`) |
| IK branch | the solution closest to the current joints with J2 < 0, J3 ≥ 0, J5 < 0 (shoulder up) |

`send_trajectory` refuses Cartesian targets outside the workspace unless they
move back towards it. Useful helpers in `core/ur10_core.py`: `ur10_fkine_tcp`,
`ur10_ikine_tcp`, `ur10_fkine_all_tcp`, `is_within_workspace`. Pose conversions
are in `webui/kinematics.py`.

### Known limitations of the framework (unchanged)

- `core/conversions.py: r2rpy()` names its argument `R`, which hides scipy's
  `Rotation`, so it raises when called. Use `webui.kinematics.pose_to_xyzrpy` or
  `spatialmath.base.tr2rpy` instead.
- `controllers/force_controller.py: force_control1()` passes a `tcp_mode` argument
  that `send_trajectory` does not accept; only `force_control2()` (the default) runs.
- Cartesian commands only work in the IK branch above. Far from it, the IK
  jumps or fails; use joint moves to go back.

---

## Configuration

Panel settings live in `webui/config.py`. The most relevant ones:

| Setting | Default | Meaning |
|---|---|---|
| `ROBOT_HOST`, `ROBOT_PORT` | `CMP00-180723AD.local`, `9090` | rosbridge address |
| `HTTP_HOST`, `HTTP_PORT` | `127.0.0.1`, `8080` | where the panel is served |
| `STALE_AFTER_S` | 0.5 s | joint-state age that blocks motion |
| `WATCHDOG_S` | 0.4 s | jog stops without a browser keep-alive for this long |
| `JOG_RATE_HZ`, `JOG_LOOKAHEAD_S`, `JOG_RAMP_S` | 20 Hz, 0.25 s, 0.3 s | velocity-jog streaming |
| `JOG_MAX_JOINT_SPEED`, `JOG_MAX_JOINT_STEP` | 0.6 rad/s, 0.35 rad | Cartesian-jog joint speed cap and IK-jump limit |
| `LINEAR_SPEED_MM_S`, `ANGULAR_SPEED_DEG_S`, `JOINT_JOG_SPEED_DEG_S`, `MOVE_SPEED_DEG_S` | (default, min, max) | slider ranges |
| `STOP_BRAKE_S` | 0.4 s | duration of the STOP trajectory |
| `CONFIRM_ABOVE_DEG` | 45° | moves larger than this ask for confirmation |
| `GRIPPER_FEEDBACK_TOPIC` | `/Robotiq2FGripperRobotInput` | set to `None` to disable |
| `PRESET_POSES_DEG` | force-control start | built-in joint presets |

---

## Safety

- Software STOP is **not** an emergency stop. Keep the teach-pendant E-stop within reach.
- Commands are only sent while `/joint_states` is fresh. roslibpy queues messages while
  disconnected and would replay them on reconnection, so the panel sends nothing in that state.
- Held jogs need a keep-alive from the browser and stop by themselves if it stops.
- The panel is served on `127.0.0.1` only. It accepts JSON commands only from its
  own page (cross-site and DNS-rebinding requests are rejected). `--listen 0.0.0.0`
  lets any device on the network open it, and so move the robot.
- Joint moves are not checked against the workspace by `UR10Control`; the panel
  warns and asks for confirmation when the TCP target is outside it.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Cannot connect … getaddrinfo failed` | The `.local` name is not resolved: check the Wi‑Fi network, try `ping CMP00-180723AD.local`, or use the IP with `--host`. |
| `Cannot connect … timed out` / `refused` | rosbridge is not running (step 4 of the robot start-up), or you are on the wrong network. |
| *Waiting for /joint_states* | The UR bring-up is not running, or *Play* was not pressed on the pendant. |
| Worked once, not after reconnecting | Restart the rosbridge launch on the Campero. |
| *No IK solution* / *IK jump* | Pose outside the IK branch or near a singularity: jog in joint space. |
| *Blocked at the workspace limit* | Limits are in `WORKSPACE_LIMITS` (`core/ur10_core.py`). |
| `setup.bat`: *No supported Python found* | Install Python 3.12 (python.org) with the py launcher. |
| `run_webui.bat`: port already in use | The panel is already open, or use `--http-port 8081`. |

---

## Linux

On Ubuntu 22.04, `sudo bash install.sh` (original installer) creates `.venv` and
installs the requirements plus the SpaceMouse and Leap Motion dependencies.
Manually:

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python3 -m webui
```

---

## Credits

Original UR_CONTROL framework and controllers by **Adrián Fortea Valencia**
(internship report *Memoria de Prácticas*, July 2025); the report credits the
kinematics functions (`kinematics_utils.py`) to **Miguel Burgh**. Web control
panel and Windows setup added on top without changing the framework.
