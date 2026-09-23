"""Robot: read the state of the Campero UR10 and command it from Python.

    from ur10api import Robot

    with Robot("CMP00-180723AD.local") as robot:
        print(robot.tcp_pose())                  # 4x4 pose of the TCP in the UR base frame
        robot.move_tcp_by(dp=(0, 0, 0.05))       # 5 cm up (blocking)
        robot.close_gripper()

Commands go out through core.ur_control.UR10Control (unchanged), so the
Campero receives the same single-point JointTrajectory messages as with every
other controller. Cartesian targets are solved here with the calibrated
kinematics and sent as joint targets.

Safety built into every command: nothing is sent unless /joint_states is fresh,
Cartesian targets must respect the workspace box, IK jumps are refused, and
speeds/accelerations are capped. STOP is a braking trajectory, not an
emergency stop: keep the teach pendant within reach.
"""

import logging
import socket
import threading
import time
from dataclasses import dataclass

import numpy as np
import roslibpy

from core.ur_control import UR10Control
from ur10api import kinematics as kin
from ur10api import transforms as tf

log = logging.getLogger("ur10api")

DEFAULT_HOST = "CMP00-180723AD.local"   # Campero PC running rosbridge_websocket
DEFAULT_PORT = 9090

# UR10Control.joint_states_callback reads J1..J6 from these indices of /joint_states
ARM_JOINT_INDEX = (-4, -5, -6, -3, -2, -1)
# Gripper status published by Robotiq2FGripperRtuNode.py (remapped by campero_ur10_bringup.launch)
GRIPPER_TOPICS = ("/robotiq_2f_gripper/input", "/Robotiq2FGripperRobotInput")
GRIPPER_TYPE = "robotiq_2f_gripper_control/Robotiq2FGripper_robot_input"

STOP_BRAKE_S = 0.4   # duration of the braking trajectory sent by stop()


class RobotError(RuntimeError):
    """Base class of the errors raised by Robot."""


class NotReady(RobotError):
    """Not connected, or the joint states are not fresh: nothing was sent."""


class MotionRefused(RobotError):
    """A motion was rejected before sending (workspace, IK, joint limits)."""


@dataclass(frozen=True)
class GripperState:
    """Status of the Robotiq 2F-85, from its driver."""
    closed: float            # 0 = fully open (85 mm gap) ... 1 = fully closed
    object_detected: bool    # the fingers stopped on an object
    moving: bool
    active: bool             # activated and ready
    fault: int               # 0 = no fault

    @classmethod
    def from_msg(cls, msg):
        """Build from a Robotiq2FGripper_robot_input message (dict)."""
        position, obj = int(msg.get("gPO", 0)), int(msg.get("gOBJ", 0))
        return cls(
            closed=min(max((position - 13) / (230 - 13), 0.0), 1.0),   # scale of pub_gripper_cmd.py
            object_detected=obj in (1, 2),
            moving=obj == 0 and int(msg.get("gGTO", 0)) == 1,
            active=int(msg.get("gSTA", 0)) == 3,
            fault=int(msg.get("gFLT", 0)),
        )

    @property
    def gap_mm(self):
        return (1.0 - self.closed) * 85.0

    @property
    def text(self):
        if self.fault:
            return f"fault 0x{self.fault:02X}"
        if not self.active:
            return "not activated"
        what = "moving" if self.moving else "object grasped" if self.object_detected else "at requested position"
        return f"gap {self.gap_mm:.0f} mm, {what}"


@dataclass(frozen=True)
class RobotState:
    """One consistent snapshot of the robot (see Robot.state())."""
    q: np.ndarray            # joint angles J1..J6 [rad]
    qd: np.ndarray           # joint velocities [rad/s], or None if not published
    tcp: np.ndarray          # 4x4 TCP pose in the UR base frame
    flange: np.ndarray       # 4x4 flange (tool0) pose in the UR base frame
    force: np.ndarray        # [Fx, Fy, Fz] in N, sensor = tool frame
    torque: np.ndarray       # [Mx, My, Mz] in Nm, sensor = tool frame
    age: float               # seconds since the last /joint_states message
    wrench_age: float        # seconds since the last force/torque message (None: never)
    gripper: GripperState    # None without gripper feedback

    @property
    def position(self):
        """TCP position [m] in the UR base frame."""
        return self.tcp[:3, 3]

    @property
    def rotation(self):
        """TCP orientation (3x3) in the UR base frame."""
        return self.tcp[:3, :3]

    @property
    def force_base(self):
        """Measured force [N] expressed in the UR base frame."""
        return self.flange[:3, :3] @ self.force


class Rate:
    """Run a loop at a fixed rate (like rospy.Rate): call sleep() once per cycle."""

    def __init__(self, hz):
        self.period = 1.0 / float(hz)
        self._next = time.monotonic() + self.period

    def sleep(self):
        delay = self._next - time.monotonic()
        if delay > 0:
            time.sleep(delay)
            self._next += self.period
        else:   # the cycle took too long: start a new schedule instead of catching up
            self._next = time.monotonic() + self.period


def _cap(vector, limit):
    """Scale `vector` down so that its norm is at most `limit`."""
    norm = float(np.linalg.norm(vector))
    return vector * (limit / norm) if norm > limit > 0 else vector


class Robot:
    """Connection to the Campero UR10 through rosbridge, with state and commands.

    Parameters (all optional):
        host, port         rosbridge address (default CMP00-180723AD.local:9090)
        tcp                TCP relative to the flange (x, y, z [m], rx, ry, rz [rad]);
                           default flange + 150 mm, as in UR_CONTROL
        workspace          TCP box {axis: (min, max)} [m]; default UR_CONTROL's limits
        max_linear_speed   cap of velocity commands [m/s]
        max_angular_speed  cap of velocity commands [rad/s]
        max_joint_speed    cap of every joint [rad/s] (moves and streaming)
        max_linear_accel   rate limit of velocity commands [m/s^2]
        max_angular_accel  rate limit of velocity commands [rad/s^2]
        timeout            seconds to wait for the connection and the first joint state

    Only one Robot per Python process: roslibpy's event loop cannot restart.
    """

    lookahead = 0.25        # s: streamed velocity targets lie this far ahead (their time_from_start)
    stale_after = 0.5       # s: commands are refused with older joint states
    max_joint_step = 0.35   # rad: a larger IK jump in one streamed step is refused

    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_PORT, *, tcp=kin.DEFAULT_TCP, workspace=None,
                 max_linear_speed=0.10, max_angular_speed=0.5, max_joint_speed=0.8,
                 max_linear_accel=0.5, max_angular_accel=2.0, timeout=10.0):
        self.arm = kin.ArmModel(tcp=tcp)
        self.workspace = kin.Workspace(workspace)
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_joint_speed = max_joint_speed
        self.max_linear_accel = max_linear_accel
        self.max_angular_accel = max_angular_accel
        self.last_warning = ""               # why the last velocity command was not sent
        self._send_lock = threading.Lock()
        self._js_time = None                 # arrival time of the last /joint_states
        self._ft_time = None                 # arrival time of the last force/torque message
        self._qd = None
        self._gripper = None                 # (arrival time, GripperState)
        self._gripper_topics = []
        self._velocity = np.zeros(6)         # last commanded [v, w] (for the acceleration limit)
        self._velocity_time = None
        self._busy_until = 0.0                # when the last sent trajectory ends
        self._ur = None
        self._connect(host, port, timeout)

    # ------------------------------------------------------------------ connection

    def _connect(self, host, port, timeout):
        try:   # fail fast, with a clear message, when the Campero is unreachable
            with socket.create_connection((host, port), timeout=3.0):
                pass
        except OSError as exc:
            raise NotReady(f"cannot reach rosbridge at {host}:{port} ({exc})") from exc
        ur = UR10Control.__new__(UR10Control)
        try:
            ur.__init__(host_ip=host, port=port)   # unchanged framework class; it zeroes the F/T sensor
        except Exception as exc:
            ros = getattr(ur, "ros", None)
            if ros is not None:   # stop roslibpy from retrying in the background
                ros.call_later(0, ros.factory.stopTrying)
            raise NotReady(f"rosbridge at {host}:{port} did not answer ({exc})") from exc
        self._ur = ur
        # Extra listeners on UR10Control's own subscriptions (roslibpy calls every
        # listener of a topic), plus the gripper status topics.
        ur.ros.on("/joint_states", self._on_joint_states)
        ur.ros.on("/robotiq_ft_sensor", self._on_wrench)
        for name in GRIPPER_TOPICS:
            topic = roslibpy.Topic(ur.ros, name, GRIPPER_TYPE)
            topic.subscribe(self._on_gripper)
            self._gripper_topics.append(topic)
        deadline = time.monotonic() + timeout
        while self._js_time is None:
            if time.monotonic() > deadline:
                self.close()
                raise NotReady("connected, but no /joint_states arrived (is the UR driver running and Play pressed?)")
            time.sleep(0.05)
        log.info("Connected to %s:%s", host, port)

    def close(self):
        """Disconnect (also called when leaving a `with Robot(...)` block)."""
        ur, self._ur = self._ur, None
        if ur is None:
            return
        for topic, callback in (("/joint_states", self._on_joint_states), ("/robotiq_ft_sensor", self._on_wrench)):
            try:
                ur.ros.off(topic, callback)
            except KeyError:
                pass
        for topic in self._gripper_topics:
            topic.unsubscribe()
        closer = threading.Thread(target=ur.close_connection, daemon=True)   # may block on a dead network
        closer.start()
        closer.join(timeout=3.0)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        if exc_type is not None:   # leaving because of an error or Ctrl+C: brake first
            try:
                self.stop()
            except RobotError:
                pass
        self.close()

    # Callbacks run in roslibpy's thread; they only store data.
    def _on_joint_states(self, msg):
        velocity = msg.get("velocity") or []
        if len(velocity) >= 6:
            self._qd = np.array([velocity[i] for i in ARM_JOINT_INDEX], dtype=float)
        self._js_time = time.monotonic()

    def _on_wrench(self, _msg):
        self._ft_time = time.monotonic()

    def _on_gripper(self, msg):
        try:
            self._gripper = (time.monotonic(), GripperState.from_msg(msg))
        except (TypeError, ValueError):
            log.debug("unexpected gripper message", exc_info=True)

    # ------------------------------------------------------------------ state

    def state(self):
        """Snapshot of joints, poses, force/torque and gripper (a RobotState)."""
        ur = self._ur
        if ur is None:
            raise NotReady("the robot connection is closed")
        now = time.monotonic()
        q = np.array(ur.joint_states, dtype=float)
        flange = self.arm.fk(q, tcp=False)
        gripper = self._gripper
        return RobotState(
            q=q,
            qd=None if self._qd is None else self._qd.copy(),
            tcp=flange @ self.arm.tcp,
            flange=flange,
            force=np.array(ur.current_force, dtype=float),
            torque=np.array(ur.current_torque, dtype=float),
            age=now - self._js_time,
            wrench_age=None if self._ft_time is None else now - self._ft_time,
            gripper=gripper[1] if gripper is not None and now - gripper[0] < 2.0 else None,
        )

    @property
    def q(self):
        """Joint angles J1..J6 [rad]."""
        return self.state().q

    def tcp_pose(self):
        """4x4 TCP pose in the UR base frame."""
        return self.state().tcp

    def flange_pose(self):
        """4x4 flange (tool0) pose in the UR base frame."""
        return self.state().flange

    def wrench(self):
        """(force [N], torque [Nm]) measured by the Robotiq FT 300, in the tool frame."""
        s = self.state()
        return s.force, s.torque

    def gripper(self):
        """GripperState from the gripper driver, or None without feedback."""
        return self.state().gripper

    def is_ready(self):
        """True while connected and receiving fresh joint states."""
        try:
            self._fresh_state()
            return True
        except NotReady:
            return False

    def _fresh_state(self):
        state = self.state()
        if not self._ur.ros.is_connected:
            raise NotReady("the connection to rosbridge is lost")
        if state.age > self.stale_after:
            raise NotReady(f"the joint states are {state.age:.1f} s old")
        return state

    # ------------------------------------------------------------------ position commands

    def move_joints(self, q, speed=0.3, wait=True, timeout=None, callback=None):
        """Move to joint angles q [rad] with one joint-space motion.

        speed: average speed [rad/s] of the joint that moves the most; all joints
        arrive together. Returns the planned duration [s]. With wait=True, blocks
        until the target is reached; `callback(state)` is called while waiting.
        """
        state = self._fresh_state()
        q = np.asarray(q, dtype=float).reshape(6)
        if not np.all(np.isfinite(q)) or np.any(np.abs(q) > kin.JOINT_LIMIT):
            raise MotionRefused("joint targets must be finite and within +/-360 deg")
        speed = float(np.clip(speed, 1e-3, self.max_joint_speed))
        duration = float(np.max(np.abs(q - state.q))) / speed
        self._reset_velocity()
        self._send_joints(q, max(duration, 1e-3))
        if wait:
            self.wait_until_reached(q, timeout=duration + 5.0 if timeout is None else timeout, callback=callback)
        return duration

    def move_tcp(self, T, speed=0.3, wait=True, timeout=None, callback=None):
        """Move the TCP to pose T (4x4, UR base frame) with one joint-space motion.

        The joints come from the calibrated IK, closest to the current ones, so the
        TCP does not travel in a straight line (use set_tcp_velocity for that).
        Raises MotionRefused outside the workspace or without an IK solution.
        """
        state = self._fresh_state()
        T = np.asarray(T, dtype=float)
        if not self.workspace.allows(state.tcp, T):
            raise MotionRefused("target outside the workspace: " + ", ".join(self.workspace.violations(T)))
        try:
            q = self.arm.ik(T, state.q)
        except kin.IKError as exc:
            raise MotionRefused(str(exc)) from exc
        return self.move_joints(q, speed, wait, timeout, callback)

    def move_tcp_by(self, dp=(0.0, 0.0, 0.0), drot=(0.0, 0.0, 0.0), frame="base", speed=0.3, wait=True,
                    callback=None):
        """Relative TCP move: dp [m] and rotation vector drot [rad], in the base or the tool frame."""
        return self.move_tcp(tf.displace(self.tcp_pose(), dp, drot, frame), speed, wait, callback=callback)

    def wait_until_reached(self, q, tol=0.002, timeout=10.0, callback=None):
        """Block until the joints are within `tol` [rad] of q and at rest. Ctrl+C brakes the robot."""
        deadline = time.monotonic() + timeout
        try:
            while True:
                state = self._fresh_state()
                if callback is not None:
                    callback(state)
                settled = state.qd is None or np.max(np.abs(state.qd)) < 0.01
                if np.max(np.abs(state.q - q)) < tol and settled:
                    return
                if time.monotonic() > deadline:
                    raise RobotError(f"target not reached within {timeout:.1f} s")
                time.sleep(0.02)
        except KeyboardInterrupt:
            self.stop()
            raise

    # ------------------------------------------------------------------ velocity commands

    def set_tcp_velocity(self, v=(0.0, 0.0, 0.0), w=(0.0, 0.0, 0.0), frame="base"):
        """Move the TCP with linear velocity v [m/s] and angular velocity w [rad/s].

        Call it periodically (10-50 Hz, see Rate). Each call sends a short target
        lying `lookahead` seconds ahead of the measured pose, so the robot keeps
        moving only while you keep calling and stops about `lookahead` s after the
        last call. frame="base" (UR base axes, rotation about the TCP) or "tool".
        Speeds are capped and ramped (max_*_speed, max_*_accel); a zero velocity
        brakes. At the workspace box the TCP stops at (and can slide along) its
        faces. Returns False when nothing could be sent; see last_warning.
        """
        state = self._fresh_state()
        command = self._limit_velocity(np.r_[np.asarray(v, dtype=float), np.asarray(w, dtype=float)])
        if not np.any(command):
            return self._brake(state)
        target = tf.displace(state.tcp, command[:3] * self.lookahead, command[3:] * self.lookahead, frame)
        if not self.workspace.contains(target):
            if self.workspace.contains(state.tcp):
                target[:3, 3] = self.workspace.clamp(target[:3, 3])   # stop at the face, keep sliding along it
            elif not self.workspace.allows(state.tcp, target):
                return self._refuse("outside the workspace, moving away from it: "
                                    + ", ".join(self.workspace.violations(state.tcp)))
        try:
            q = self.arm.ik(target, state.q)
        except kin.IKError as exc:
            return self._refuse(str(exc))
        step = float(np.max(np.abs(q - state.q)))
        if step > self.max_joint_step:
            return self._refuse(f"IK jump of {np.degrees(step):.0f} deg (singularity or arm configuration change)")
        self._send_joints(q, max(self.lookahead, step / self.max_joint_speed))
        self.last_warning = ""
        return True

    def set_joint_velocity(self, qd):
        """Move the joints with velocities qd [rad/s] (6 values). Call it periodically, like set_tcp_velocity."""
        state = self._fresh_state()
        qd = np.clip(np.asarray(qd, dtype=float).reshape(6), -self.max_joint_speed, self.max_joint_speed)
        self._reset_velocity()
        if not np.any(qd):
            return self._brake(state)
        target = np.clip(state.q + qd * self.lookahead, -kin.JOINT_LIMIT, kin.JOINT_LIMIT)
        self._send_joints(target, self.lookahead)
        return True

    def _limit_velocity(self, command):
        """Cap the speed and the change of the commanded [v, w] since the previous call."""
        now = time.monotonic()
        recent = self._velocity_time is not None and now - self._velocity_time < 0.5
        dt = now - self._velocity_time if recent else 0.04
        previous = self._velocity if recent else np.zeros(6)
        linear = _cap(command[:3], self.max_linear_speed)
        angular = _cap(command[3:], self.max_angular_speed)
        linear = previous[:3] + _cap(linear - previous[:3], self.max_linear_accel * dt)
        angular = previous[3:] + _cap(angular - previous[3:], self.max_angular_accel * dt)
        self._velocity, self._velocity_time = np.r_[linear, angular], now
        return self._velocity.copy()

    def _reset_velocity(self):
        self._velocity, self._velocity_time = np.zeros(6), None

    def _refuse(self, reason):
        if reason != self.last_warning:
            log.warning("velocity command not sent: %s", reason)
        self.last_warning = reason
        self._reset_velocity()
        return False

    # ------------------------------------------------------------------ stop, gripper, sensor

    def _brake(self, state, duration=None):
        """Short trajectory to rest: current joints plus half the braking distance."""
        duration = self.lookahead if duration is None else duration
        velocity = state.qd if state.qd is not None else np.zeros(6)
        target = np.clip(state.q + velocity * duration / 2, -kin.JOINT_LIMIT, kin.JOINT_LIMIT)
        if np.max(np.abs(target - state.q)) >= 1e-6:
            self._send_joints(target, duration)
        return True

    def stop(self):
        """Brake to a stop, replacing whatever the robot is executing (not an emergency stop).

        Sends nothing when the robot is at rest and no trajectory is pending.
        """
        self._reset_velocity()
        state = self._fresh_state()
        velocity = state.qd if state.qd is not None else np.zeros(6)
        target = np.clip(state.q + velocity * STOP_BRAKE_S / 2, -kin.JOINT_LIMIT, kin.JOINT_LIMIT)
        if np.max(np.abs(target - state.q)) < 1e-4:
            if time.monotonic() > self._busy_until + 0.5:
                return
            # Sent but maybe not started yet. UR10Control derives time_from_start as
            # distance / speed, so nudge wrist 3 by 1e-4 rad (0.006 deg) to keep a
            # non-zero duration while replacing that motion.
            target[5] += -1e-4 if target[5] > 0 else 1e-4
        self._send_joints(target, STOP_BRAKE_S)

    def open_gripper(self):
        """Open the Robotiq gripper (UR10Control.send_gripper_cmd)."""
        self._fresh_state()
        with self._send_lock:
            self._ur.send_gripper_cmd("Open")

    def close_gripper(self):
        """Close the Robotiq gripper."""
        self._fresh_state()
        with self._send_lock:
            self._ur.send_gripper_cmd("Close")

    def zero_force_sensor(self, settle=0.5):
        """Set the force/torque readings to zero (SET ZERO service), then wait `settle` s.

        Do it with nothing touching the gripper: the current load becomes the zero.
        """
        self._fresh_state()
        with self._send_lock:
            self._ur.zero_ft_sensor()
        time.sleep(settle)

    # ------------------------------------------------------------------ sending

    def _send_joints(self, q, duration):
        """One single-point trajectory: reach joint angles q [rad] in `duration` s.

        UR10Control computes time_from_start as max|q - current| / speed, so the
        matching speed is passed.
        """
        ur = self._ur
        if ur is None:
            raise NotReady("the robot connection is closed")
        q = np.asarray(q, dtype=float)
        distance = float(np.max(np.abs(q - np.array(ur.joint_states, dtype=float))))
        if distance < 1e-9:
            return
        with self._send_lock:
            ur.send_trajectory(q, speed=distance / duration, art=True)
            self._busy_until = time.monotonic() + duration
