"""Calibrated kinematics of the Campero UR10 and the workspace box.

ArmModel computes forward/inverse kinematics with this arm's factory
calibration, read from robot_model/model.json (the Campero URDF, built by
tools/build_robot_model.py). Poses therefore match the teach pendant and RViz;
UR_CONTROL's own nominal model (core/ur10_core.py) is 3-6 mm off.

Workspace is a box for the TCP in the UR base frame, by default UR_CONTROL's
WORKSPACE_LIMITS.

Conventions: joints J1..J6 = shoulder_pan, shoulder_lift, elbow, wrist_1,
wrist_2, wrist_3 [rad]; poses are 4x4 transforms in the UR base frame
(campero_ur10_base, the pendant's "Base"); the TCP is given relative to the
flange (tool0) like the pendant's TCP setting: (x, y, z [m], rx, ry, rz [rad]).
"""

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from core.ur10_core import WORKSPACE_LIMITS, ur10_ikine
from ur10api import transforms as tf

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "robot_model" / "model.json"
BASE_FRAME = "campero_ur10_base"      # UR controller "Base" = pendant Base = UR_CONTROL base frame
FLANGE_FRAME = "campero_ur10_tool0"   # tool flange
ARM_JOINTS = tuple(f"campero_ur10_{name}_joint" for name in
                   ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"))
DEFAULT_TCP = (0.0, 0.0, 0.15, 0.0, 0.0, 0.0)   # UR_CONTROL's TCP: flange + 150 mm along tool Z

JOINT_LIMIT = 2 * np.pi   # software joint limit used by core/kinematics_utils.py
JOINT_NAMES = ("Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3")
CARTESIAN_AXES = ("X", "Y", "Z", "Rx", "Ry", "Rz")


class IKError(RuntimeError):
    """No usable inverse-kinematics solution for a pose."""


def _origin_transform(xyz_quat):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(xyz_quat[3:]).as_matrix()
    T[:3, 3] = xyz_quat[:3]
    return T


class ArmModel:
    """Serial chain UR base -> flange (-> TCP) with the arm calibration."""

    def __init__(self, tcp=DEFAULT_TCP, model_path=MODEL_PATH, base_frame=BASE_FRAME,
                 flange_frame=FLANGE_FRAME, joint_names=ARM_JOINTS):
        joints = {j["child"]: j for j in json.loads(Path(model_path).read_text(encoding="utf-8"))["joints"]}

        def path_from_root(link):
            path = []
            while link in joints:
                path.append(joints[link])
                link = joints[link]["parent"]
            return path[::-1]

        base_path, flange_path = path_from_root(base_frame), path_from_root(flange_frame)
        shared = 0
        while shared < min(len(base_path), len(flange_path)) and base_path[shared] is flange_path[shared]:
            shared += 1
        # base frame -> common ancestor: inverse of the fixed joints down to the base frame
        start = np.eye(4)
        for joint in base_path[shared:]:
            if joint["type"] != "fixed":
                raise ValueError(f"{base_frame} must be fixed relative to the arm root")
            start = start @ _origin_transform(joint["origin"])
        self._start = tf.inverse(start)
        # common ancestor -> flange: fixed origins and the six joint axes
        self._segments = []
        for joint in flange_path[shared:]:
            if joint["type"] == "fixed":
                self._segments.append((_origin_transform(joint["origin"]), None))
            elif joint["name"] in joint_names:
                self._segments.append((_origin_transform(joint["origin"]), np.asarray(joint["axis"], dtype=float)))
            else:
                raise ValueError(f"unexpected joint {joint['name']} between {base_frame} and {flange_frame}")
        if sum(axis is not None for _, axis in self._segments) != 6:
            raise ValueError("the arm chain must contain exactly six joints")
        self.set_tcp(tcp)

    def set_tcp(self, tcp):
        """TCP relative to the flange: (x, y, z [m], rx, ry, rz [rad, rotation vector])."""
        self.tcp_values = tuple(float(v) for v in tcp)
        self.tcp = tf.pose(self.tcp_values[:3], self.tcp_values[3:])   # flange -> TCP transform

    def chain(self, q):
        """Flange pose, joint axes (6x3) and joint positions (6x3), all in the UR base frame."""
        q = np.asarray(q, dtype=float)
        T, axes, origins, i = self._start.copy(), [], [], 0
        for origin, axis in self._segments:
            T = T @ origin
            if axis is not None:
                axes.append(T[:3, :3] @ axis)
                origins.append(T[:3, 3].copy())
                step = np.eye(4)
                step[:3, :3] = Rotation.from_rotvec(axis * q[i]).as_matrix()
                T = T @ step
                i += 1
        return T, np.array(axes), np.array(origins)

    def fk(self, q, tcp=True):
        """Pose of the TCP (or of the flange with tcp=False) for joint angles q [rad]."""
        flange = self.chain(q)[0]
        return flange @ self.tcp if tcp else flange

    def skeleton(self, q):
        """Points for a stick-figure drawing: base, joints, flange and TCP (N x 3) [m]."""
        flange, _, origins = self.chain(q)
        return np.vstack([np.zeros(3), origins, flange[:3, 3], (flange @ self.tcp)[:3, 3]])

    def jacobian(self, q, tcp=True):
        """Geometric Jacobian (6x6) of the TCP (or flange): [v; w] = J @ qdot, base frame."""
        flange, axes, origins = self.chain(q)
        point = (flange @ self.tcp)[:3, 3] if tcp else flange[:3, 3]
        return np.vstack([np.cross(axes, point - origins).T, axes.T])

    def ik(self, T, seed, tcp=True):
        """Joint angles [rad] that put the TCP (or flange) at pose T, closest to `seed`.

        UR_CONTROL's analytic solver (nominal parameters) gives the starting
        point and the arm configuration (shoulder up, same branch as UR10Control);
        a few Newton steps on the calibrated chain remove the remaining error.
        Raises IKError when the pose is out of reach or singular.
        """
        target = np.asarray(T, dtype=float) @ tf.inverse(self.tcp) if tcp else np.asarray(T, dtype=float)
        try:
            q = np.asarray(ur10_ikine(target, np.asarray(seed, dtype=float)), dtype=float)
        except Exception as exc:   # the solver raises when no solution lies in its branch
            raise IKError("no inverse-kinematics solution for this pose") from exc
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise IKError("inverse kinematics returned an invalid solution (singular pose)")
        for _ in range(10):
            flange, axes, origins = self.chain(q)
            dp = target[:3, 3] - flange[:3, 3]
            dr = Rotation.from_matrix(target[:3, :3] @ flange[:3, :3].T).as_rotvec()
            if np.linalg.norm(dp) < 1e-7 and np.linalg.norm(dr) < 1e-7:
                break
            J = np.vstack([np.cross(axes, flange[:3, 3] - origins).T, axes.T])
            q = q + np.linalg.lstsq(J, np.r_[dp, dr], rcond=None)[0]
        else:
            raise IKError("no inverse-kinematics solution: the pose is out of reach or singular")
        if np.any(np.abs(q) > JOINT_LIMIT):
            raise IKError("the solution exceeds the +/-360 deg joint limits")
        return q


# --- workspace -------------------------------------------------------------------------------

DEFAULT_WORKSPACE = {axis: (float(WORKSPACE_LIMITS[axis][0]), float(WORKSPACE_LIMITS[axis][1])) for axis in "xyz"}
MAX_WORKSPACE_EXTENT = 2.0   # m from the UR base: well beyond the reach of the UR10 + gripper


def validate_limits(limits):
    """Check {axis: (min, max)} in metres and return a clean copy. Raises ValueError."""
    clean = {}
    for axis in "xyz":
        try:
            lo, hi = (float(v) for v in limits[axis])
        except (KeyError, TypeError, ValueError):
            raise ValueError(f"{axis}: a minimum and a maximum are required") from None
        if not (np.isfinite(lo) and np.isfinite(hi)):
            raise ValueError(f"{axis}: limits must be numbers")
        if hi - lo < 0.01:
            raise ValueError(f"{axis}: the maximum must be at least 10 mm above the minimum")
        if max(abs(lo), abs(hi)) > MAX_WORKSPACE_EXTENT:
            raise ValueError(f"{axis}: limits must stay within ±{MAX_WORKSPACE_EXTENT * 1000:.0f} mm of the UR base")
        clean[axis] = (lo, hi)
    return clean


class Workspace:
    """Box that the TCP must stay in, in the UR base frame: {axis: (min, max)} in metres."""

    def __init__(self, limits=None):
        self.set(DEFAULT_WORKSPACE if limits is None else limits)

    def set(self, limits):
        self._limits = validate_limits(limits)   # single assignment: readers in other threads see a whole box

    @property
    def limits(self):
        return dict(self._limits)

    def _bounds(self):
        limits = self._limits
        return (np.array([limits[a][0] for a in "xyz"]), np.array([limits[a][1] for a in "xyz"]))

    def clamp(self, position):
        """Closest point of the box to `position` [m]."""
        lo, hi = self._bounds()
        return np.clip(np.asarray(position, dtype=float), lo, hi)

    def distance(self, T):
        """Distance [m] from the TCP of pose T to the box (0 inside)."""
        p = np.asarray(T, dtype=float)[:3, 3]
        return float(np.linalg.norm(p - self.clamp(p)))

    def contains(self, T):
        return self.distance(T) == 0.0

    def violations(self, T):
        """Limits the TCP of pose T violates, e.g. ['x = -1312 mm (limits -1300 to -300)']."""
        p, limits = np.asarray(T, dtype=float)[:3, 3], self._limits
        out = []
        for i, axis in enumerate("xyz"):
            lo, hi = limits[axis]
            if not lo <= p[i] <= hi:
                out.append(f"{axis} = {p[i] * 1000:.0f} mm (limits {lo * 1000:.0f} to {hi * 1000:.0f})")
        return out

    def allows(self, T_current, T_target):
        """A target is allowed inside the box, or when it is closer to the box than the current pose.

        This is the rule described by core.ur10_core.is_moving_towards_workspace;
        that implementation measures the current pose against the target's
        nearest box point and so also accepts some targets outside the box.
        """
        distance = self.distance(T_target)
        return distance == 0.0 or distance < self.distance(T_current) - 1e-9
