"""The control thread of example 4: teleoperation, target, Jacobian probing, servoing, log.

The window (display.py) never talks to the robot. It queues requests
(session.request("probe")), holds jog keys (session.jog) and reads
session.status and session.recorder. Everything that moves the robot runs in
Session.run(), one cycle per control period:

    read the newest camera observation and robot state
    handle the requests (start a task, STOP, switch the feature space, ...)
    step the running task, or teleoperate the joints when there is none
    log one row and publish the status for the window

Tasks (record a target, move, probe, servo) are generators: each `yield` ends
a control cycle, so a task reads like a sequential program, and STOP ends it
between two cycles.
"""

import json
import math
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from shape_servoing import features as feat
from shape_servoing import jacobian as jac
from ur10api import Rate, RobotError

MODES = ("idle", "recording", "moving", "probing", "servoing")
STILL = 0.002        # rad: a joint target counts as reached within this, with the joints at rest


class TaskError(RuntimeError):
    """A task cannot start or go on; the message says why."""


# --- data ------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Target:
    """The target shape: marker IDs (increasing), their centres [px] and 3D positions [m] (or None)."""
    ids: tuple
    points: np.ndarray       # N x 2, image
    points3d: object = None  # N x 3, camera frame, or None if the 3D positions were not all available

    def features(self, space, frame="image"):
        """Target features, or None in the camera frame without 3D positions."""
        points = self.points if frame == "image" else self.points3d
        return None if points is None else feat.features(points, space)

    def save(self, path, image_size):
        data = {"ids": list(self.ids), "points": np.round(self.points, 2).tolist(),
                "points3d": None if self.points3d is None else np.round(self.points3d, 5).tolist(),
                "image_size": list(image_size), "recorded": time.strftime("%Y-%m-%d %H:%M:%S")}
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")

    @classmethod
    def load(cls, path):
        """Read a target saved by save(); returns (Target, image size [w, h]). Raises ValueError."""
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            ids, points = tuple(int(i) for i in data["ids"]), np.array(data["points"], dtype=float)
            points3d = None if data.get("points3d") is None else np.array(data["points3d"], dtype=float)
            size = tuple(int(v) for v in data.get("image_size", (0, 0)))
        except KeyError as exc:
            raise ValueError(f"the target {path} has no {exc} entry") from exc
        except (OSError, ValueError, TypeError) as exc:
            raise ValueError(f"cannot read the target {path}: {exc}") from exc
        if (len(ids) < feat.MIN_MARKERS or points.shape != (len(ids), 2) or list(ids) != sorted(set(ids))
                or (points3d is not None and points3d.shape != (len(ids), 3))):
            raise ValueError(f"{path}: expected at least {feat.MIN_MARKERS} increasing marker ids and their points")
        return cls(ids, points, points3d), size


@dataclass
class Probe:
    """Samples of the probing motion:
    (joint being moved or -1 at rest, joints q [rad], {id: centre [px]}, {id: 3D position [m]})."""
    q0: np.ndarray
    ids: tuple
    samples: list


@dataclass(frozen=True)
class ServoRun:
    """Outcome of one servo run, for the summary."""
    key: str                 # feature set (space, "3d" for the camera frame) at the end of the run
    outcome: str
    duration: float          # s
    rms: float               # final RMS feature error, display units (nan if never measured)


@dataclass(frozen=True)
class Status:
    """What the window shows; replaced as a whole every cycle."""
    mode: str
    message: str             # last event
    progress: str            # what the running task is doing
    space: str               # active feature space
    frame: str               # active frame: image (2D) or camera (3D)
    key: str                 # active feature set, features.key(space, frame)
    ids: tuple               # markers defining the features: the target's, else those tracked
    target: object           # Target or None
    jog_speed: float         # deg/s
    robot: bool              # False with --camera-only
    home: bool               # a home position is configured
    jacobian: str            # the Jacobian of the active space, in words
    gripper: str             # gripper feedback, e.g. "gap 85 mm, at requested position" ("" without)
    rate: float              # measured control rate [Hz]
    elapsed: float           # time since the start [s] (the time axis of the log)


class JogInput:
    """Joint jog requests from the window: held keys (with auto-repeat) and held mouse buttons.

    Keys: a release only counts after RELEASE_GRACE (X11 auto-repeat sends
    release + press pairs), and a key that stops repeating for KEY_TIMEOUT s
    (e.g. the window lost the focus while it was held) stops the jog.
    """
    KEY_TIMEOUT = 0.8
    RELEASE_GRACE = 0.08

    def __init__(self):
        self._lock = threading.Lock()
        self._keys = {}          # (joint, sign) -> [last press time, release time or None]
        self._button = None      # (joint, sign) while a jog button is held

    def press_key(self, joint, sign):
        with self._lock:
            self._keys[(joint, sign)] = [time.monotonic(), None]

    def release_key(self, joint, sign):
        with self._lock:
            if (joint, sign) in self._keys:
                self._keys[(joint, sign)][1] = time.monotonic()

    def press_button(self, joint, sign):
        self._button = (joint, sign)

    def release_button(self):
        self._button = None

    def clear(self):
        with self._lock:
            self._keys.clear()
        self._button = None

    def direction(self):
        """-1, 0 or +1 per joint."""
        now, d = time.monotonic(), np.zeros(6)
        with self._lock:
            for (joint, sign), (pressed, released) in list(self._keys.items()):
                if (released is not None and now - released > self.RELEASE_GRACE) or now - pressed > self.KEY_TIMEOUT:
                    del self._keys[(joint, sign)]
                else:
                    d[joint] += sign
        if self._button is not None:
            d[self._button[0]] += self._button[1]
        return np.clip(d, -1, 1)


class Recorder:
    """Growing table of logged signals: one row per control cycle, named columns (NaN where absent)."""

    def __init__(self, rows=4096, columns=64):
        self._lock = threading.Lock()
        self._t = np.empty(rows)
        self._data = np.full((rows, columns), np.nan)
        self._columns = {}
        self._n = 0

    def record(self, t, row):
        with self._lock:
            for name in row:
                if name not in self._columns:
                    if len(self._columns) == self._data.shape[1]:
                        self._data = np.hstack([self._data, np.full_like(self._data, np.nan)])
                    self._columns[name] = len(self._columns)
            if self._n == len(self._t):
                self._t = np.r_[self._t, np.empty(len(self._t))]
                self._data = np.vstack([self._data, np.full_like(self._data, np.nan)])
            self._t[self._n] = t
            for name, value in row.items():
                self._data[self._n, self._columns[name]] = value
            self._n += 1

    def between(self, t_start, t_end):
        """(times, data, {column: index}) of the rows logged from t_start to t_end [s], as copies."""
        with self._lock:
            n = self._n
            first = int(np.searchsorted(self._t[:n], t_start, side="left"))
            last = int(np.searchsorted(self._t[:n], t_end, side="right"))
            return self._t[first:last].copy(), self._data[first:last, :len(self._columns)].copy(), dict(self._columns)

    def tail(self, seconds=None):
        """(times, data, {column: index}) of the last `seconds` (everything if None), as copies."""
        with self._lock:
            n = self._n
            start = 0
            if seconds is not None and n:
                start = int(np.searchsorted(self._t[:n], self._t[n - 1] - seconds))
            return self._t[start:n].copy(), self._data[start:n, :len(self._columns)].copy(), dict(self._columns)

    def save(self, path, **extra):
        """Everything logged, as a compressed .npz: t, data, columns (+ extra arrays)."""
        t, data, columns = self.tail()
        names = sorted(columns, key=columns.get)
        np.savez_compressed(path, t=t, data=data, columns=np.array(names), **extra)


# --- the session -------------------------------------------------------------------------------

class Session:
    """State and control thread of the shape servoing example.

    robot     a connected ur10api.Robot, or None (camera only: vision, targets and plots)
    vision    a started vision.Vision
    settings  from settings.load_settings()
    target    a Target to start with (e.g. loaded with --target), or None
    save_dir  folder for the recorded targets (None: not saved)
    space, frame  feature space and frame at start-up (None: from the settings)

    Attach a logger.RunLogger as session.logger to save every servo run.
    """

    def __init__(self, robot, vision, settings, target=None, space=None, frame=None, save_dir=None):
        self.robot, self.vision, self.s = robot, vision, settings
        self.save_dir = None if save_dir is None else Path(save_dir)
        self.recorder = Recorder()
        self.jog = JogInput()
        self.logger = None                        # RunLogger, or None: runs are not logged
        self.runs = []                            # ServoRun of every servo run
        self._requests = queue.SimpleQueue()
        self.t0 = time.monotonic()
        self._space = space or settings["features"]["space"]
        self._frame = frame or settings["features"]["frame"]
        self._target = target
        self._task, self._mode = None, "idle"
        self._message, self._progress = "ready", ""
        self._jog_speed = settings["teleop"]["joint_speed"]
        self._jogging = False
        self._active = np.array(settings["probe"]["amplitude"]) > 0   # joints probed and controlled
        self._probe = None
        self._J0, self._J, self._fit_text = {}, {}, {}   # per feature set: probed J0, current estimate, fit
        self._command = None                      # joint velocity sent in this cycle [rad/s]
        self._rate = 0.0
        home = settings["moves"]["home"]
        self._home = np.radians(home) if home else None
        self._q_start = robot.q.copy() if robot is not None else None
        self.obs = vision.latest()
        self.state = None
        self.status = self._make_status()

    # --- window side (thread-safe) ---------------------------------------------------------
    def request(self, action, value=None):
        """Queue an action: record, start, home, set_start, probe, servo, stop, space (value),
        frame ("image"/"camera"), jog_speed (+1/-1), gripper ("open"/"close")."""
        self._requests.put((action, value))

    def jacobians(self):
        """{name: J} probed (J0_<set>) and current estimates (J_<set>), for the saved log."""
        out = {f"J0_{key}": J for key, J in self._J0.items()}
        out.update({f"J_{key}": J for key, J in self._J.items()})
        return out

    @property
    def target(self):
        return self._target

    @property
    def key(self):
        """Active feature set, e.g. "edges" or "edges3d"."""
        return feat.key(self._space, self._frame)

    # --- control thread ----------------------------------------------------------------------
    def run(self, stop):
        """The control loop; returns when `stop` (a threading.Event) is set."""
        rate, last = Rate(self.s["control"]["rate"]), time.monotonic()
        period = 1.0 / self.s["control"]["rate"]
        try:
            while not stop.is_set():
                self._cycle()
                rate.sleep()
                now = time.monotonic()
                period = 0.9 * period + 0.1 * (now - last)    # smoothed cycle time
                self._rate, last = 1.0 / period, now
        finally:
            self._end_task()

    def _cycle(self):
        self.obs = self.vision.latest()
        self.state = self._read_state()
        self._command = None
        self._handle_requests()
        if self._task is not None:
            mode = self._mode
            try:
                next(self._task)
            except StopIteration:
                self._end_task()
            except (TaskError, RobotError) as exc:
                self._end_task(brake=True)
                self._message = f"{mode} stopped: {exc}"
        else:
            try:
                self._teleop()
            except RobotError as exc:
                self._message = f"jog: {exc}"
        self._log()
        self.status = self._make_status()

    def _read_state(self):
        """Fresh robot state, or None (no robot, connection closed or joint states too old)."""
        if self.robot is None:
            return None
        try:
            state = self.robot.state()
        except RobotError:
            return None
        return state if state.age <= self.robot.stale_after else None

    def _handle_requests(self):
        while True:
            try:
                action, value = self._requests.get_nowait()
            except queue.Empty:
                return
            if action == "stop":
                mode = self._mode
                self.jog.clear()
                self._end_task(brake=True)
                self._message = "STOP" + (f": {mode} cancelled" if mode != "idle" else "")
            elif action == "space":
                self._space = value
                self._message = f"feature space: {value}"
            elif action == "frame":
                self._frame = value
                self._message = "features in the " + ("image (2D)" if value == "image" else "camera frame (3D)")
            elif action == "jog_speed":
                speed = self._jog_speed * (1.5 if value > 0 else 1 / 1.5)
                self._jog_speed = round(min(max(speed, 0.5), self.s["teleop"]["max_joint_speed"]), 1)
                self._message = f"jog speed {self._jog_speed:g} deg/s"
            elif action == "gripper":            # "open" / "close"; allowed at any time
                if self.robot is None:
                    self._message = "no robot (started with --camera-only)"
                    continue
                try:
                    (self.robot.open_gripper if value == "open" else self.robot.close_gripper)()
                    self._message = f"gripper: {value}"
                except RobotError as exc:
                    self._message = f"gripper: {exc}"
            elif action == "set_start":
                if self.state is None:
                    self._message = "no fresh joint states: the start configuration was not changed"
                else:
                    self._q_start = self.state.q.copy()
                    self._message = "start configuration = current joints"
            elif self._task is not None:
                self._message = f"busy ({self._mode}): press STOP first"
            elif action == "record":
                self._start("recording", self._record_task())
            elif action == "start":
                self._start("moving", self._move_task(self._q_start, "the start configuration"))
            elif action == "home":
                if self._home is None:
                    self._message = "no home position: set moves.home in 04_shape_servoing.yaml"
                else:
                    self._start("moving", self._move_task(self._home, "home"))
            elif action == "probe":
                self._start("probing", self._probe_task())
            elif action == "servo":
                self._start("servoing", self._servo_task())

    def _start(self, mode, task):
        self.jog.clear()
        if self._jogging:
            self._jogging = False
            self._brake()
        self._task, self._mode, self._progress = task, mode, ""

    def _end_task(self, brake=False):
        task, self._task = self._task, None
        if task is not None:
            task.close()                 # runs the task's finally blocks
        self._mode, self._progress = "idle", ""
        if brake and self.robot is not None:
            try:
                self.robot.stop()
            except RobotError:
                pass

    # --- commands --------------------------------------------------------------------------------
    def _need_robot(self):
        if self.robot is None:
            raise TaskError("no robot (started with --camera-only)")
        return self.robot

    def _need_state(self):
        if self.state is None:
            raise TaskError("no fresh joint states")
        return self.state

    def _send_velocity(self, qd):
        """Stream joint velocities qd [rad/s], unless the TCP would leave the workspace box."""
        robot, state = self._need_robot(), self._need_state()
        T_next = robot.arm.fk(state.q + qd * robot.lookahead)
        if not robot.workspace.allows(state.tcp, T_next):
            self._brake()
            self._message = "workspace limit: " + ", ".join(robot.workspace.violations(T_next))
            return False
        robot.set_joint_velocity(qd)
        self._command = qd
        return True

    def _brake(self):
        if self.robot is not None:
            try:
                self.robot.set_joint_velocity(np.zeros(6))
            except RobotError:
                pass

    def _teleop(self):
        direction = self.jog.direction()
        if not np.any(direction):
            if self._jogging:
                self._jogging = False
                self._brake()
            return
        if self.robot is None:
            self.jog.clear()
            self._message = "no robot (started with --camera-only)"
            return
        if self.state is None:
            self._message = "no fresh joint states: cannot jog"
            return
        self._jogging = True
        self._send_velocity(direction * math.radians(self._jog_speed))

    # --- tasks ---------------------------------------------------------------------------------
    def _wait_reached(self, q_goal, duration, label, each=None):
        """Yield cycles until the joints reach q_goal and rest; each() is called every cycle.

        label: progress text, or a function returning it (called every cycle).
        """
        deadline = time.monotonic() + duration + 5.0
        while True:
            self._progress = label() if callable(label) else label
            yield
            state = self._need_state()
            if each is not None:
                each()
            settled = state.qd is None or np.max(np.abs(state.qd)) < 0.01
            if np.max(np.abs(state.q - q_goal)) < STILL and settled:
                return
            if time.monotonic() > deadline:
                raise TaskError(f"{label}: not reached in time")

    def _move_task(self, q_goal, name):
        robot, state = self._need_robot(), self._need_state()
        q_goal = np.asarray(q_goal, dtype=float)
        travel = float(np.max(np.abs(q_goal - state.q)))
        if travel < STILL:
            self._message = f"already at {name}"
            return
        outside = not robot.workspace.contains(robot.arm.fk(q_goal))
        duration = robot.move_joints(q_goal, speed=math.radians(self.s["moves"]["speed"]), wait=False)
        self._message = (f"moving to {name}: {math.degrees(travel):.1f} deg, {duration:.1f} s"
                         + (" (the TCP ends outside the workspace box)" if outside else ""))
        yield from self._wait_reached(q_goal, duration, f"moving to {name}")
        self._message = f"at {name}"

    def _record_task(self):
        """Average the tracked marker centres over record_time; they become the target."""
        obs = self.obs
        ids = sorted(obs.markers) if obs is not None else []
        if len(ids) < feat.MIN_MARKERS:
            raise TaskError(f"{len(ids)} marker(s) tracked: a target needs at least {feat.MIN_MARKERS}")
        centres, positions, last_seq = {i: [] for i in ids}, {i: [] for i in ids}, None
        end = time.monotonic() + self.s["features"]["record_time"]
        while True:
            obs = self.obs
            if obs.seq != last_seq:
                last_seq = obs.seq
                for i in ids:
                    if i in obs.markers:
                        centres[i].append(obs.markers[i])
                    if i in obs.markers3d:
                        positions[i].append(obs.markers3d[i])
            if time.monotonic() >= end:
                break
            self._progress = f"recording the target: markers {', '.join(map(str, ids))}"
            yield
        frames = max(len(c) for c in centres.values())
        kept = [i for i in ids if len(centres[i]) >= max(1, frames // 2)]
        if len(kept) < feat.MIN_MARKERS:
            raise TaskError("markers were lost while recording: try again")
        points3d = None
        if all(len(positions[i]) >= max(1, frames // 2) for i in kept):
            points3d = np.array([np.mean(positions[i], axis=0) for i in kept])
        target = Target(tuple(kept), np.array([np.mean(centres[i], axis=0) for i in kept]), points3d)
        self._target = target
        self._refit()
        self._message = f"target recorded: markers {', '.join(map(str, kept))}"
        if self.save_dir is not None:
            path = self.save_dir / f"shape_target_{time.strftime('%Y%m%d_%H%M%S')}.json"
            try:
                target.save(path, obs.image.shape[1::-1])
                self._message += f" (saved to {path.name})"
                print(f"Target saved to {path}  (reuse it with --target)")
            except OSError as exc:
                self._message += f" (not saved: {exc})"

    def _probe_task(self):
        """Excite each joint back and forth around q0 and fit J0 on the recorded marker motion."""
        robot, state = self._need_robot(), self._need_state()
        obs = self.obs
        tracked = sorted(obs.markers) if obs is not None else []
        ids = self._target.ids if self._target is not None else tuple(tracked)
        if len(ids) < feat.MIN_MARKERS:
            raise TaskError(f"{len(ids)} marker(s) tracked: probing needs at least {feat.MIN_MARKERS}")
        missing = [i for i in ids if i not in tracked]
        if missing:
            raise TaskError(f"marker(s) {', '.join(map(str, missing))} not in view")
        p = self.s["probe"]
        q0, amplitude = state.q.copy(), np.radians(p["amplitude"])
        for j in np.flatnonzero(amplitude):          # the whole excitation must respect the workspace
            for offset in np.linspace(-amplitude[j], amplitude[j], 9):
                q = q0.copy()
                q[j] += offset
                T = robot.arm.fk(q)
                if not robot.workspace.allows(state.tcp, T):
                    raise TaskError(f"J{j + 1} {math.degrees(offset):+.0f} deg would take the TCP out of the "
                                    f"workspace ({', '.join(robot.workspace.violations(T))}): reduce "
                                    "probe.amplitude or start elsewhere")
        samples, moving, last_seq = [], [-1], [None]

        def collect():
            obs, st = self.obs, self.state
            if obs is not None and st is not None and obs.seq != last_seq[0]:
                last_seq[0] = obs.seq
                samples.append((moving[0], st.q.copy(), dict(obs.markers), dict(obs.markers3d)))

        end = time.monotonic() + p["settle"]
        while time.monotonic() < end:
            collect()
            self._progress = "probing: reference samples at rest"
            yield
        plan = [(j, offset) for j in np.flatnonzero(amplitude) for _ in range(p["cycles"])
                for offset in (amplitude[j], -amplitude[j], 0.0)]
        for k, (j, offset) in enumerate(plan):
            moving[0] = j
            goal = q0.copy()
            goal[j] += offset
            duration = robot.move_joints(goal, speed=math.radians(p["speed"]), wait=False)
            label = f"probing J{j + 1}: move {k + 1} of {len(plan)}, "
            yield from self._wait_reached(goal, duration, lambda: label + f"{len(samples)} samples", collect)
        self._probe = Probe(q0, tuple(ids), samples)
        self._refit()
        self._message = "probe done: " + self._fit_text[self.key]

    def _refit(self):
        """J0 of every feature set (3 spaces x 2 frames) from the probe samples, for the current markers."""
        self._J0, self._J, self._fit_text = {}, {}, {}
        probe = self._probe
        if probe is None:
            return
        ids, active = (self._target.ids if self._target is not None else probe.ids), self._active
        for index, frame in enumerate(feat.FRAMES):
            keys = [feat.key(space, frame) for space in feat.SPACES]
            rows = []                                     # samples where all the markers were located
            for joint, q, *located in probe.samples:      # located: centres [px], 3D positions [m]
                found = located[index]
                if all(i in found for i in ids):
                    rows.append((joint, q, np.array([found[i] for i in ids])))
            per_joint = {j: sum(1 for joint, _, _ in rows if joint == j) for j in np.flatnonzero(active)}
            few = [f"J{j + 1}: {n}" for j, n in per_joint.items() if n < 5]
            if few or len(rows) < 3 * (np.sum(active) + 1):
                text = ("too few samples with all of markers " + ", ".join(map(str, ids))
                        + (f" located ({', '.join(few)})" if few else "") + ": probe again")
                self._fit_text.update(dict.fromkeys(keys, text))
                continue
            dq = np.array([q for _, q, _ in rows])[:, active] - probe.q0[active]
            for space, name in zip(feat.SPACES, keys):
                S = np.array([feat.features(points, space) for _, _, points in rows])
                J = np.zeros((S.shape[1], 6))
                J[:, active], rms = jac.fit(dq, feat.difference(S, S[0], space))
                self._J0[name] = J
                self._fit_text[name] = (f"J0 from {len(rows)} samples: fit residual "
                                        f"{feat.to_display(rms, name):.2g} {feat.unit(name)}, "
                                        f"cond {jac.condition(J[:, active]):.3g}")

    def _current_features(self, space, frame, ids):
        """Features of the markers `ids` in the newest image, or None (marker lost, image too old)."""
        obs = self.obs
        if obs is None or time.monotonic() - obs.time > self.s["control"]["max_image_age"]:
            return None
        points = obs.points(ids, frame)
        return None if points is None else feat.features(points, space)

    def _servo_task(self):
        """Damped-pseudoinverse servoing towards the target, with Broyden updates of J."""
        self._need_robot()
        self._need_state()
        if self._target is None:
            raise TaskError("record a target first (Enter)")
        if self.key not in self._J0:
            raise TaskError(self._fit_text.get(self.key) or "probe the Jacobian first (P)")
        if self._target.features(self._space, self._frame) is None:
            raise TaskError("the target has no 3D positions: record it again")
        c, b, active, ids = self.s["control"], self.s["broyden"], self._active, self._target.ids
        cap = math.radians(c["max_joint_speed"])
        key = J = ref = last_seq = good_since = None
        rms, t0, outcome = math.nan, time.monotonic(), "stopped"
        if self.logger is not None:
            self.logger.start(self.key, {"feature_set": self.key, "target_ids": ids,
                                         "target_points": self._target.points,
                                         "target_points3d": self._target.points3d, "J0": self._J0,
                                         "start_joints_deg": np.degrees(self.state.q)})
        try:
            while True:
                now = time.monotonic()
                if key != self.key:                       # start, or the user switched the space or frame
                    if self.key not in self._J0 or self._target.features(self._space, self._frame) is None:
                        raise TaskError(f"no Jacobian or target for {self.key}")
                    key, space, frame = self.key, self._space, self._frame
                    J = self._J.get(key) if b["keep"] else None
                    J = (self._J0[key] if J is None else J).copy()
                    s_target, ref, good_since = self._target.features(space, frame), None, None
                    tolerance = self.s["features"]["tolerance" if frame == "image" else "tolerance_3d"][space]
                    tolerance = feat.from_display(tolerance, key)
                    unit = feat.unit(key)
                s = self._current_features(space, frame, ids)
                if s is None:                              # a marker or the image is lost: hold still
                    self._brake()
                    ref = good_since = None
                    lost = [i for i in ids if self.obs is None or i not in self.obs.markers]
                    self._progress = (f"servoing ({key}): waiting for marker(s) {', '.join(map(str, lost))}"
                                      if lost else f"servoing ({key}): waiting for images")
                    yield
                    continue
                state, obs = self._need_state(), self.obs
                error = feat.difference(s, s_target, space)

                # Broyden: correct J along the joint motion since the last update (new images only)
                if b["enabled"] and obs.seq != last_seq:
                    last_seq = obs.seq
                    if ref is None:
                        ref = (state.q.copy(), s)
                    elif np.linalg.norm((state.q - ref[0])[active]) >= math.radians(b["min_step"]):
                        dq = np.where(active, state.q - ref[0], 0.0)
                        J = jac.broyden_update(J, dq, feat.difference(s, ref[1], space), b["alpha"])
                        ref = (state.q.copy(), s)
                self._J[key] = J

                # Damped least squares step towards the target, scaled down to the speed cap
                qd = np.zeros(6)
                qd[active] = -c["gain"] * jac.damped_pinv(J[:, active], c["damping"]) @ error
                peak = np.max(np.abs(qd))
                if peak > cap:
                    qd *= cap / peak
                self._send_velocity(qd)

                rms = float(np.sqrt(np.mean(error ** 2)))
                shown = feat.to_display(rms, key)
                if rms <= tolerance:
                    good_since = now if good_since is None else good_since
                    if now - good_since >= c["settle_time"]:
                        self._brake()
                        outcome = "target reached"
                        self._message = f"target reached in {now - t0:.1f} s ({key}, RMS error {shown:.2f} {unit})"
                        return
                else:
                    good_since = None
                if now - t0 > c["timeout"]:
                    self._brake()
                    outcome = "timeout"
                    self._message = f"timeout after {c['timeout']:g} s ({key}, RMS error {shown:.2f} {unit})"
                    return
                self._progress = (f"servoing ({key}): RMS error {shown:.2f} {unit} "
                                  f"(tolerance {feat.to_display(tolerance, key):g}), {now - t0:.0f} s")
                yield
        except (TaskError, RobotError) as exc:
            outcome = f"error: {exc}"
            raise
        finally:
            final_rms = float(feat.to_display(rms, key)) if key else math.nan
            self.runs.append(ServoRun(key or self.key, outcome, time.monotonic() - t0, final_rms))
            if self.logger is not None:
                self.logger.stop({"outcome": outcome, "final_feature_set": key, "final_rms": final_rms,
                                  "final_unit": feat.unit(key) if key else "", "J_final": dict(self._J)})

    # --- log and status ------------------------------------------------------------------------
    def _display_ids(self):
        if self._target is not None:
            return self._target.ids
        return tuple(sorted(self.obs.markers)) if self.obs is not None else ()

    def _log(self):
        row = {"mode": MODES.index(self._mode)}
        state = self.state
        if state is not None:
            for j in range(6):
                row[f"q{j + 1}"] = math.degrees(state.q[j])
                if state.qd is not None:
                    row[f"qd{j + 1}"] = math.degrees(state.qd[j])
            if self._command is not None or self._mode in ("idle", "recording"):
                command = np.zeros(6) if self._command is None else self._command
                row.update({f"qc{j + 1}": math.degrees(command[j]) for j in range(6)})
        ids, obs, target = self._display_ids(), self.obs, self._target
        for frame in feat.FRAMES:
            points = obs.points(ids, frame) if obs is not None and ids else None
            for space in feat.SPACES:
                key = feat.key(space, frame)
                names = feat.names(ids, space, frame)
                values = None
                if points is not None and names:
                    values = feat.features(points, space)
                    row.update(zip((f"{key}:{n}" for n in names), feat.to_display(values, key)))
                s_target = target.features(space, frame) if target is not None else None
                if s_target is not None:
                    row.update(zip((f"target:{key}:{n}" for n in names), feat.to_display(s_target, key)))
                    if values is not None:
                        error = feat.to_display(feat.difference(values, s_target, space), key)
                        row.update(zip((f"error:{key}:{n}" for n in names), error))
                        row[f"error_norm:{key}"] = float(np.linalg.norm(error))
                J = self._J.get(key, self._J0.get(key))
                if J is not None:
                    row[f"cond:{key}"] = jac.condition(J[:, self._active])
        self.recorder.record(time.monotonic() - self.t0, row)

    def _jacobian_text(self):
        key = self.key
        if key not in self._fit_text:
            return "no Jacobian yet: Probe Jacobian (P)"
        text = self._fit_text[key]
        J = self._J.get(key)
        if J is not None:
            text += f"; current estimate cond {jac.condition(J[:, self._active]):.3g}"
        return text

    def _make_status(self):
        gripper = getattr(self.state, "gripper", None)
        return Status(self._mode, self._message, self._progress, self._space, self._frame, self.key,
                      self._display_ids(), self._target, self._jog_speed, self.robot is not None,
                      self._home is not None, self._jacobian_text(), gripper.text if gripper is not None else "",
                      self._rate, time.monotonic() - self.t0)
