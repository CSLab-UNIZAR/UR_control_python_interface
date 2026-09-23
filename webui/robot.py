"""Robot connection and motion logic behind the web panel.

``RobotLink`` owns the unchanged ``UR10Control`` object and adds a read-only
view of the robot (message rates, joint velocities, gripper feedback).
``Motion`` turns panel requests (jog, step, move, stop) into
``UR10Control.send_trajectory`` calls. Nothing here changes the messages the
Campero receives; it only decides *when* and *with which target* to send them.
"""

import collections
import itertools
import logging
import socket
import threading
import time
from dataclasses import dataclass, field

import numpy as np
import roslibpy

from core.ur_control import UR10Control
from ur10api.robot import ARM_JOINT_INDEX, GripperState
from webui import config
from webui import kinematics as kin

log = logging.getLogger("webui")

PHASE_TEXT = {
    "disconnected": "not connected",
    "connecting": "connecting",
    "waiting": "connected, waiting for /joint_states",
    "ready": "ready",
    "stale": "no fresh /joint_states from the robot",
    "lost": "connection lost, reconnecting",
}


class NotReady(RuntimeError):
    """Commands are refused: not connected, or the robot state is not fresh."""


class MotionError(RuntimeError):
    """A requested motion was rejected before anything was sent."""


# ----------------------------------------------------------------------------- event log

class EventLog(logging.Handler):
    """Keeps recent log records so the browser can poll them by id."""

    def __init__(self, size=300):
        super().__init__(logging.INFO)
        self._items = collections.deque(maxlen=size)
        self._ids = itertools.count(1)

    def emit(self, record):
        with self.lock:
            self._items.append({"id": next(self._ids), "time": record.created,
                                "level": record.levelname.lower(), "text": record.getMessage()})

    def since(self, last_id):
        with self.lock:
            return [item for item in self._items if item["id"] > last_id]


class _RateMeter:
    """Message rate and age from arrival times (thread-safe enough for display)."""

    def __init__(self, window=50):
        self._times = collections.deque(maxlen=window)

    def tick(self, now):
        self._times.append(now)

    def rate(self):
        times = list(self._times)
        span = times[-1] - times[0] if len(times) > 1 else 0.0
        return (len(times) - 1) / span if span > 0 else 0.0

    def age(self, now):
        times = list(self._times)
        return now - times[-1] if times else None


# ----------------------------------------------------------------------------- robot link

@dataclass
class RobotState:
    phase: str                          # one of PHASE_TEXT
    q: np.ndarray = None                # joint positions J1..J6 [rad]
    qd: np.ndarray = None               # joint velocities [rad/s], None if unknown
    T: np.ndarray = None                # TCP pose in the base frame (4x4)
    force: np.ndarray = None            # [N]
    torque: np.ndarray = None           # [Nm]
    rates: dict = field(default_factory=dict)

    @property
    def ready(self):
        return self.phase == "ready"


class RobotLink:
    """Connection to the Campero through UR10Control, plus read-only monitoring."""

    def __init__(self, gripper_topics=config.GRIPPER_FEEDBACK_TOPICS):
        self.ur = None
        self.host, self.port = config.ROBOT_HOST, config.ROBOT_PORT
        self.connecting = False
        self.error = None
        self.gripper_command = None
        self._gripper_topics = tuple(gripper_topics)
        self._gripper_subs = []
        self._gripper_feedback = None       # (arrival time, text, closed fraction 0..1)
        self._send_lock = threading.Lock()
        self._js = _RateMeter()
        self._ft = _RateMeter()
        self._qd = None
        self._last_positions = None         # for velocity estimation if not published

    # --- connection ---------------------------------------------------------------

    def connect_async(self, host, port):
        if self.ur is not None or self.connecting:
            raise MotionError("already connected; restart the panel to change robot")
        self.connecting = True     # set before the thread starts so the panel shows it at once
        threading.Thread(target=self.connect, args=(host, int(port)), daemon=True).start()

    def connect(self, host, port):
        """Blocking connect. Errors are stored in self.error and logged."""
        self.host, self.port, self.error, self.connecting = host, port, None, True
        log.info("Connecting to rosbridge at %s:%s ...", host, port)
        try:
            # Fail fast (and with a clear message) when the robot PC is unreachable.
            with socket.create_connection((host, port), timeout=config.PROBE_TIMEOUT_S):
                pass
            ur = UR10Control.__new__(UR10Control)
            try:
                ur.__init__(host_ip=host, port=port)   # unchanged framework class
            except Exception:
                ros = getattr(ur, "ros", None)
                if ros is not None:                  # stop roslibpy retrying in the background
                    ros.call_later(0, ros.factory.stopTrying)
                raise
            # Piggyback on UR10Control's own subscriptions: roslibpy dispatches each
            # message to every listener of the topic, so no extra rosbridge traffic.
            ur.ros.on("/joint_states", self._on_joint_states)
            ur.ros.on("/robotiq_ft_sensor", self._on_force)
            for topic in self._gripper_topics:
                subscription = roslibpy.Topic(ur.ros, topic, config.GRIPPER_FEEDBACK_TYPE)
                subscription.subscribe(self._on_gripper)
                self._gripper_subs.append(subscription)
            self.ur = ur
            log.info("Connected to rosbridge at %s:%s (F/T sensor zeroed by UR10Control)", host, port)
        except Exception as exc:
            self.error = f"Cannot connect to {host}:{port}: {exc or type(exc).__name__}"
            log.error(self.error)
        finally:
            self.connecting = False

    def close(self):
        ur, self.ur = self.ur, None
        if ur is None:
            return
        for topic, callback in (("/joint_states", self._on_joint_states),
                                ("/robotiq_ft_sensor", self._on_force)):
            try:
                ur.ros.off(topic, callback)
            except KeyError:
                pass
        for subscription in self._gripper_subs:
            subscription.unsubscribe()
        ur.close_connection()

    # --- monitoring callbacks (run in the roslibpy thread) ------------------------------

    def _on_joint_states(self, msg):
        now = time.monotonic()
        try:
            velocity = msg.get("velocity") or []
            if len(velocity) >= 6:
                self._qd = np.array([velocity[i] for i in ARM_JOINT_INDEX], dtype=float)
            else:
                self._qd = self._estimate_velocity(msg.get("position") or [], now)
        except (TypeError, ValueError, IndexError):
            log.debug("Unexpected /joint_states message", exc_info=True)
        self._js.tick(now)

    def _estimate_velocity(self, positions, now):
        if len(positions) < 6:
            return None
        q = np.array([positions[i] for i in ARM_JOINT_INDEX], dtype=float)
        previous, self._last_positions = self._last_positions, (q, now)
        if previous is None or now - previous[1] < 1e-3:
            return self._qd
        raw = (q - previous[0]) / (now - previous[1])
        return raw if self._qd is None else 0.5 * raw + 0.5 * self._qd

    def _on_force(self, _msg):
        self._ft.tick(time.monotonic())

    def _on_gripper(self, msg):
        try:
            gripper = GripperState.from_msg(msg)
            self._gripper_feedback = (time.monotonic(), gripper.text, gripper.closed)
        except (TypeError, ValueError):
            log.debug("Unexpected gripper feedback message", exc_info=True)

    # --- state ---------------------------------------------------------------------

    def state(self):
        ur = self.ur
        if ur is None:
            return RobotState("connecting" if self.connecting else "disconnected")
        now = time.monotonic()
        js_age = self._js.age(now)
        if not ur.ros.is_connected:
            phase = "lost"
        elif js_age is None:
            phase = "waiting"
        elif js_age > config.STALE_AFTER_S:
            phase = "stale"
        else:
            phase = "ready"
        rates = {"joint_states": self._js.rate(), "ft": self._ft.rate()}
        if js_age is None:
            return RobotState(phase, rates=rates)
        q = np.array(ur.joint_states, dtype=float)
        return RobotState(
            phase,
            q=q,
            qd=None if self._qd is None else self._qd.copy(),
            T=kin.fk(q),   # calibrated model and config.TCP (UR10Control.T_current is nominal)
            force=np.array(ur.current_force, dtype=float),
            torque=np.array(ur.current_torque, dtype=float),
            rates=rates,
        )

    def gripper_feedback(self):
        """(text, closed fraction 0..1) from the gripper driver, or None if it is silent."""
        feedback = self._gripper_feedback
        if feedback is None or time.monotonic() - feedback[0] > 2.0:
            return None
        return feedback[1:]

    # --- commands (always through UR10Control) ----------------------------------------------

    def _ready_ur(self):
        """Commands are only sent on a live link with fresh joint states: roslibpy
        queues messages while disconnected and would replay them on reconnect."""
        state = self.state()
        if not state.ready:
            raise NotReady(PHASE_TEXT[state.phase])
        return self.ur

    def send_joints(self, q, speed):
        """Joint target [rad], reached in max|q - current| / speed seconds (UR10Control semantics).

        Cartesian commands also end up here: the panel solves the IK with the
        calibrated model, so the message is the same single-point trajectory.
        """
        with self._send_lock:
            self._ready_ur().send_trajectory(np.asarray(q, dtype=float), speed=float(speed), art=True)

    def send_gripper(self, command):
        if command not in ("Open", "Close"):
            raise MotionError(f"unknown gripper command {command!r}")
        with self._send_lock:
            self._ready_ur().send_gripper_cmd(command)
            self.gripper_command = command

    def zero_ft(self):
        with self._send_lock:
            self._ready_ur().zero_ft_sensor()


# ----------------------------------------------------------------------------- motion

def _clamp(value, limits):
    """Clamp a panel value to the (default, min, max) range of a config tuple."""
    return float(min(max(float(value), limits[1]), limits[2]))


@dataclass
class Plan:
    q: np.ndarray          # joint target [rad]
    T: np.ndarray          # TCP target (4x4)
    duration: float        # [s]
    worst: int             # joint with the largest motion
    max_motion: float      # [rad]
    warnings: list


class Motion:
    """Jog streaming, step jogs, moves and STOP, all through RobotLink."""

    def __init__(self, link):
        self.link = link
        self.status = ("", "info")          # (text, level) shown under the jog pad
        self._lock = threading.Lock()
        self._jog = None                    # active jog settings, None when idle
        self._jog_refreshed = 0.0           # last keep-alive from the browser
        self._axes = np.zeros(6)            # held direction per axis (-1, 0, +1)
        self._axis_since = np.zeros(6)      # when each axis started moving (for the ramp)
        self._step = None                   # (key, target, busy_until) of the last step
        self._running = True
        self._wake = threading.Event()
        self._thread = threading.Thread(target=self._run, name="jog-stream", daemon=True)
        self._thread.start()

    # --- continuous jog ----------------------------------------------------------------

    def set_jog(self, space, frame, axes, linear_mm_s, angular_deg_s, joint_deg_s):
        """Called by the browser on every change and as keep-alive while held."""
        axes = np.clip(np.round(np.asarray(axes, dtype=float)), -1, 1)
        if axes.shape != (6,):
            raise MotionError("axes must have 6 entries")
        settings = {
            "space": "joint" if space == "joint" else "cartesian",
            "frame": "tool" if frame == "tool" else "base",
            "linear": _clamp(linear_mm_s, config.LINEAR_SPEED_MM_S) / 1000.0,
            "angular": np.radians(_clamp(angular_deg_s, config.ANGULAR_SPEED_DEG_S)),
            "joint": np.radians(_clamp(joint_deg_s, config.JOINT_JOG_SPEED_DEG_S)),
        }
        now = time.monotonic()
        with self._lock:
            same_mode = self._jog is not None and all(
                self._jog[k] == settings[k] for k in ("space", "frame"))
            changed = (axes != self._axes) if same_mode else np.ones(6, dtype=bool)
            self._axis_since[changed] = now
            self._axes = axes
            self._jog = settings if axes.any() else None
            self._jog_refreshed = now
            self._step = None
        if not axes.any() and self.status[1] == "info":
            self.status = ("", "info")

    def active_axes(self):
        with self._lock:
            return self._axes.tolist() if self._jog is not None else [0] * 6

    def _end_jog(self, reason=None):
        with self._lock:
            self._jog = None
            self._axes = np.zeros(6)
        if reason:
            self._set_status(reason, "warning")

    def _set_status(self, text, level="info"):
        if level != "info" and self.status != (text, level):
            log.warning(text)
        self.status = (text, level)

    def _run(self):
        period = 1.0 / config.JOG_RATE_HZ
        while self._running:
            started = time.monotonic()
            try:
                self._tick()
            except NotReady as exc:
                self._end_jog(f"Jog stopped: {exc}")
            except Exception:
                log.exception("Jog stream error")
                self._end_jog("Jog stopped: internal error (see console)")
            self._wake.wait(max(0.0, period - (time.monotonic() - started)))

    def _tick(self):
        with self._lock:
            jog, axes, since, refreshed = self._jog, self._axes.copy(), self._axis_since.copy(), self._jog_refreshed
        if jog is None:
            return
        now = time.monotonic()
        if now - refreshed > config.WATCHDOG_S:
            self._end_jog("Jog stopped: no keep-alive from the browser")
            return
        state = self.link.state()
        if not state.ready:
            self._end_jog(f"Jog stopped: {PHASE_TEXT[state.phase]}")
            return
        direction = axes * np.clip((now - since) / config.JOG_RAMP_S, 0.0, 1.0)
        if np.max(np.abs(direction)) < 1e-3:
            return
        if jog["space"] == "joint":
            self._stream_joints(state, direction, jog)
        else:
            self._stream_cartesian(state, direction, jog)

    def _stream_joints(self, state, direction, jog):
        horizon = config.JOG_LOOKAHEAD_S
        target = np.clip(state.q + direction * jog["joint"] * horizon, -kin.JOINT_LIMIT, kin.JOINT_LIMIT)
        motion = float(np.max(np.abs(target - state.q)))
        if motion < 1e-6:
            self._set_status("Joint software limit (+/-360 deg) reached", "warning")
            return
        self.link.send_joints(target, speed=motion / horizon)
        self._set_status(f"Jogging {self._describe(direction, 'joint')} at "
                         f"{np.degrees(jog['joint']):.0f} deg/s")

    def _stream_cartesian(self, state, direction, jog):
        horizon = config.JOG_LOOKAHEAD_S
        target = kin.displace(state.T, direction[:3] * jog["linear"] * horizon,
                              direction[3:] * jog["angular"] * horizon, jog["frame"])
        if not kin.workspace_allows(state.T, target):
            self._set_status("Blocked by the workspace limits: the TCP would reach "
                             + ", ".join(kin.workspace_violations(target)), "warning")
            return
        try:
            q_target = kin.ik(target, state.q)
        except kin.IKError as exc:
            self._end_jog(f"Jog stopped: {exc}")
            return
        motion = np.abs(q_target - state.q)
        worst = int(np.argmax(motion))
        if motion[worst] > config.JOG_MAX_JOINT_STEP:
            self._end_jog(f"Jog stopped: IK jump of {np.degrees(motion[worst]):.0f} deg on "
                          f"{kin.JOINT_NAMES[worst]} (singularity or arm configuration change)")
            return
        if motion[worst] < 1e-7:
            return
        self.link.send_joints(q_target, speed=min(motion[worst] / horizon, config.JOG_MAX_JOINT_SPEED))
        self._set_status(f"Jogging {self._describe(direction, 'cartesian')} ({jog['frame']} frame)")

    @staticmethod
    def _describe(direction, space):
        names = [f"J{i + 1}" for i in range(6)] if space == "joint" else kin.CARTESIAN_AXES
        return ", ".join(("+" if d > 0 else "-") + names[i] for i, d in enumerate(direction) if d)

    # --- step jog ----------------------------------------------------------------------

    def step(self, space, frame, axis, direction, size, linear_mm_s, angular_deg_s, joint_deg_s):
        """One increment: size in mm for X/Y/Z, degrees for rotations and joints.

        Clicks made while the previous step is still running continue from its
        target, so N quick clicks give exactly N steps.
        """
        self._end_jog()
        axis, direction = int(axis), (1 if float(direction) > 0 else -1)
        if not 0 <= axis < 6:
            raise MotionError("axis must be 0..5")
        state = self.link.state()
        if not state.ready:
            raise NotReady(PHASE_TEXT[state.phase])
        now = time.monotonic()
        key = (space, frame)
        chained = self._step is not None and self._step[0] == key and now < self._step[2]
        base = self._step[1] if chained else None

        if space == "joint":
            speed = np.radians(_clamp(joint_deg_s, config.JOINT_JOG_SPEED_DEG_S))
            target = np.array(state.q if base is None else base, dtype=float)
            target[axis] += direction * np.radians(float(size))
            if abs(target[axis]) > kin.JOINT_LIMIT:
                raise MotionError(f"{kin.JOINT_NAMES[axis]} would exceed the +/-360 deg limit")
            duration = float(np.max(np.abs(target - state.q))) / speed
            self.link.send_joints(target, speed=speed)
            label = f"J{axis + 1}"
        else:
            linear = _clamp(linear_mm_s, config.LINEAR_SPEED_MM_S) / 1000.0
            angular = np.radians(_clamp(angular_deg_s, config.ANGULAR_SPEED_DEG_S))
            dp, drot = np.zeros(3), np.zeros(3)
            if axis < 3:
                dp[axis] = direction * float(size) / 1000.0
            else:
                drot[axis - 3] = direction * np.radians(float(size))
            target = kin.displace(state.T if base is None else base, dp, drot, frame)
            if not kin.workspace_allows(state.T, target):
                raise MotionError("step blocked by the workspace limits: the TCP would reach "
                                  + ", ".join(kin.workspace_violations(target)))
            try:
                q_target = kin.ik(target, state.q)
            except kin.IKError as exc:
                raise MotionError(str(exc)) from exc
            motion = float(np.max(np.abs(q_target - state.q)))
            if motion > 2 * config.JOG_MAX_JOINT_STEP:
                raise MotionError(f"IK jump of {np.degrees(motion):.0f} deg for this step "
                                  "(singularity or arm configuration change)")
            duration = max(np.linalg.norm(target[:3, 3] - state.T[:3, 3]) / linear,
                           kin.rotation_angle(state.T, target) / angular, 0.2)
            if motion < 1e-7:
                return
            self.link.send_joints(q_target, speed=min(motion / duration, config.JOG_MAX_JOINT_SPEED))
            label = kin.CARTESIAN_AXES[axis]
        self._step = (key, target, now + duration + 0.2)
        unit = "mm" if space != "joint" and axis < 3 else "deg"
        log.info("Step %s%s %g %s", "+" if direction > 0 else "-", label, float(size), unit)

    # --- moves ---------------------------------------------------------------------------

    def plan_joints(self, q_deg, speed_deg_s):
        state = self._ready_state()
        q = np.radians(np.asarray(q_deg, dtype=float))
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise MotionError("six numeric joint targets are required")
        if np.any(np.abs(q) > kin.JOINT_LIMIT):
            raise MotionError("joint targets must be within +/-360 deg")
        T = kin.fk(q)
        warnings = kin.workspace_violations(T)
        if warnings:
            warnings = ["TCP target outside the workspace limits: " + ", ".join(warnings)]
        return self._plan(state, q, T, speed_deg_s, warnings)

    def plan_pose(self, xyz_mm, rpy_deg, speed_deg_s):
        state = self._ready_state()
        xyz = np.asarray(xyz_mm, dtype=float) / 1000.0
        rpy = np.radians(np.asarray(rpy_deg, dtype=float))
        if xyz.shape != (3,) or rpy.shape != (3,) or not np.all(np.isfinite(np.r_[xyz, rpy])):
            raise MotionError("x, y, z, roll, pitch and yaw are required")
        T = kin.xyzrpy_to_pose(np.r_[xyz, rpy])
        if not kin.workspace_allows(state.T, T):
            raise MotionError("target outside the workspace limits: "
                              + ", ".join(kin.workspace_violations(T)))
        try:
            q = kin.ik(T, state.q)
        except kin.IKError as exc:
            raise MotionError(str(exc)) from exc
        return self._plan(state, q, T, speed_deg_s, [])

    def _plan(self, state, q, T, speed_deg_s, warnings):
        speed = np.radians(_clamp(speed_deg_s, config.MOVE_SPEED_DEG_S))
        motion = np.abs(q - state.q)
        worst = int(np.argmax(motion))
        return Plan(q, T, float(motion[worst] / speed), worst, float(motion[worst]), warnings)

    def move_joints(self, q_deg, speed_deg_s):
        self._end_jog()
        plan = self.plan_joints(q_deg, speed_deg_s)
        if plan.max_motion < 1e-5:
            log.info("Joint move skipped: already at target")
            return plan
        self.link.send_joints(plan.q, speed=np.radians(_clamp(speed_deg_s, config.MOVE_SPEED_DEG_S)))
        log.info("Joint move to [%s] deg, about %.1f s",
                 ", ".join(f"{v:.2f}" for v in np.degrees(plan.q)), plan.duration)
        return plan

    def move_pose(self, xyz_mm, rpy_deg, speed_deg_s):
        self._end_jog()
        plan = self.plan_pose(xyz_mm, rpy_deg, speed_deg_s)
        if plan.max_motion < 1e-5:
            log.info("Pose move skipped: already at target")
            return plan
        self.link.send_joints(plan.q, speed=np.radians(_clamp(speed_deg_s, config.MOVE_SPEED_DEG_S)))
        log.info("Pose move to xyz [%s] mm, rpy [%s] deg, about %.1f s",
                 ", ".join(f"{v:.1f}" for v in xyz_mm), ", ".join(f"{v:.1f}" for v in rpy_deg),
                 plan.duration)
        return plan

    def _ready_state(self):
        state = self.link.state()
        if not state.ready:
            raise NotReady(PHASE_TEXT[state.phase])
        return state

    # --- stop ------------------------------------------------------------------------

    def stop(self):
        """Replace whatever the robot is executing by a short braking trajectory."""
        self._end_jog()
        self._step = None
        state = self.link.state()
        if not state.ready:
            raise NotReady(f"STOP not sent ({PHASE_TEXT[state.phase]}); use the teach pendant")
        velocity = state.qd if state.qd is not None else np.zeros(6)
        target = np.clip(state.q + velocity * config.STOP_BRAKE_S / 2, -kin.JOINT_LIMIT, kin.JOINT_LIMIT)
        if np.max(np.abs(target - state.q)) < 1e-4:
            # At rest. UR10Control derives time_from_start as distance / speed, so
            # nudge wrist 3 by 1e-4 rad (0.006 deg) to keep a non-zero duration.
            target[5] += -1e-4 if target[5] > 0 else 1e-4
        distance = float(np.max(np.abs(target - state.q)))
        self.link.send_joints(target, speed=distance / config.STOP_BRAKE_S)
        self.status = ("Stopped", "warning")
        log.warning("STOP: braking to the current position")

    def shutdown(self):
        qd = self.link.state().qd
        moving = self._jog is not None or (qd is not None and np.max(np.abs(qd)) > 0.01)
        self._running = False
        self._wake.set()
        self._thread.join(timeout=1.0)
        if moving:
            try:
                self.stop()
            except Exception as exc:
                log.warning("Could not stop the robot on exit: %s", exc)
