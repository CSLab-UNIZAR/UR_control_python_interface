"""Arm kinematics of the web panel.

The arm model is the Campero URDF including this arm's factory calibration
(webui/static/robot/model.json, built by tools/build_robot_model.py), so poses
match the teach pendant and RViz. UR_CONTROL's analytic solver (core.ur10_core,
nominal UR10 parameters, 3-6 mm off) provides the IK seed and the arm branch;
a few Newton steps on the calibrated model remove the remaining error.

Conventions:
  * joints J1..J6 = shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3 [rad]
  * poses are 4x4 transforms in the UR base frame (campero_ur10_base = pendant "Base")
  * the TCP is config.TCP relative to the flange (tool0), like the pendant's TCP setting
  * roll/pitch/yaw: R = Rz(yaw) @ Ry(pitch) @ Rx(roll), as SE3.RPY(..., order='zyx') and UR's RPY
"""

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from core.ur10_core import WORKSPACE_LIMITS, ur10_ikine
from webui import config

JOINT_LIMIT = 2 * np.pi   # software joint limit used by core/kinematics_utils.py
JOINT_NAMES = ("Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3")
CARTESIAN_AXES = ("X", "Y", "Z", "Rx", "Ry", "Rz")
MODEL_PATH = Path(__file__).with_name("static") / "robot" / "model.json"


class IKError(RuntimeError):
    """No usable inverse-kinematics solution for a pose."""


def _transform(xyz_quat):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(xyz_quat[3:]).as_matrix()
    T[:3, 3] = xyz_quat[:3]
    return T


def _pose(xyz, rotvec):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    T[:3, 3] = xyz
    return T


class ArmModel:
    """Serial chain base frame -> flange, read from the robot model."""

    def __init__(self, model_path, base_frame, flange_frame, joint_names, tcp):
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
        # base -> common ancestor: inverse of the fixed joints from the ancestor down to the base frame
        self._start = np.eye(4)
        for joint in base_path[shared:]:
            if joint["type"] != "fixed":
                raise ValueError(f"{base_frame} must be fixed relative to the arm root")
            self._start = self._start @ _transform(joint["origin"])
        self._start = np.linalg.inv(self._start)
        # common ancestor -> flange: fixed origins and the six joint axes
        self._segments = []
        for joint in flange_path[shared:]:
            if joint["type"] == "fixed":
                self._segments.append((_transform(joint["origin"]), None))
            elif joint["name"] in joint_names:
                self._segments.append((_transform(joint["origin"]), np.asarray(joint["axis"], dtype=float)))
            else:
                raise ValueError(f"unexpected joint {joint['name']} between {base_frame} and {flange_frame}")
        if sum(axis is not None for _, axis in self._segments) != 6:
            raise ValueError("the arm chain must contain exactly six joints")
        self.tcp = _pose(tcp[:3], tcp[3:])
        self._tcp_inv = np.linalg.inv(self.tcp)

    def chain(self, q):
        """Flange pose, joint axes (6x3) and joint positions (6x3), all in the base frame."""
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
        flange = self.chain(np.asarray(q, dtype=float))[0]
        return flange @ self.tcp if tcp else flange

    def ik(self, T, seed, tcp=True):
        target = np.asarray(T, dtype=float) @ self._tcp_inv if tcp else np.asarray(T, dtype=float)
        try:   # nominal analytic solution: picks the same arm branch as UR10Control
            q = np.asarray(ur10_ikine(target, np.asarray(seed, dtype=float)), dtype=float)
        except Exception as exc:
            raise IKError("no inverse-kinematics solution for this pose") from exc
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise IKError("inverse kinematics returned an invalid solution (singular pose)")
        for _ in range(10):   # Newton steps on the calibrated model
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


ARM = ArmModel(MODEL_PATH, config.ARM_BASE_FRAME, config.FLANGE_FRAME, config.ARM_JOINTS, config.TCP)


def fk(q, tcp=True):
    """TCP (or flange) pose for joint vector q, calibrated model."""
    return ARM.fk(q, tcp)


def ik(T, seed, tcp=True):
    """Joint solution for a TCP (or flange) pose, closest to `seed`, calibrated model."""
    return ARM.ik(T, seed, tcp)


def pose_to_xyzrpy(T):
    """4x4 pose -> [x, y, z, roll, pitch, yaw] in metres and radians."""
    T = np.asarray(T, dtype=float)
    yaw, pitch, roll = Rotation.from_matrix(T[:3, :3]).as_euler("ZYX")
    return np.array([T[0, 3], T[1, 3], T[2, 3], roll, pitch, yaw])


def rotation_vector(T):
    """Axis-angle vector [rad] of the orientation: the RX, RY, RZ shown by the UR pendant."""
    return Rotation.from_matrix(np.asarray(T, dtype=float)[:3, :3]).as_rotvec()


def xyzrpy_to_pose(xyzrpy):
    """[x, y, z, roll, pitch, yaw] in metres and radians -> 4x4 pose."""
    x, y, z, roll, pitch, yaw = (float(v) for v in xyzrpy)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("ZYX", [yaw, pitch, roll]).as_matrix()
    T[:3, 3] = (x, y, z)
    return T


def displace(T, dp, drot, frame="base"):
    """Translate by dp [m] and rotate about the TCP by rotation vector drot [rad].

    Both are expressed in the base frame (frame="base") or in the TCP frame
    (frame="tool").
    """
    T = np.asarray(T, dtype=float)
    R, p = T[:3, :3], T[:3, 3]
    dR = Rotation.from_rotvec(np.asarray(drot, dtype=float)).as_matrix()
    out = np.eye(4)
    if frame == "tool":
        out[:3, :3] = R @ dR
        out[:3, 3] = p + R @ np.asarray(dp, dtype=float)
    else:
        out[:3, :3] = dR @ R
        out[:3, 3] = p + np.asarray(dp, dtype=float)
    return out


def rotation_angle(T_a, T_b):
    """Angle [rad] of the relative rotation between two poses."""
    R_rel = np.asarray(T_a)[:3, :3].T @ np.asarray(T_b)[:3, :3]
    return float(np.linalg.norm(Rotation.from_matrix(R_rel).as_rotvec()))


# --- workspace box (TCP limits in the UR base frame, metres) ----------------------------
# The panel starts from UR_CONTROL's WORKSPACE_LIMITS and lets the user change its own
# copy; the constants in core/ur10_core.py (used by the other controllers) stay as they are.
DEFAULT_WORKSPACE = {axis: (float(WORKSPACE_LIMITS[axis][0]), float(WORKSPACE_LIMITS[axis][1])) for axis in "xyz"}
MAX_WORKSPACE_EXTENT = 2.0   # m from the UR base: well beyond the reach of the UR10 + gripper
_workspace = dict(DEFAULT_WORKSPACE)


def workspace_limits():
    """Current limits {axis: (min, max)} in metres, UR base frame."""
    return dict(_workspace)


def set_workspace_limits(limits):
    """Validate and apply {axis: (min, max)} in metres. Raises ValueError."""
    global _workspace
    new = {}
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
        new[axis] = (lo, hi)
    _workspace = new   # single assignment: the jog thread always sees a complete box


def workspace_violations(T):
    """Which workspace limits the TCP of pose T violates, e.g. 'x = -1312 mm (limits -1300 to -300)'."""
    p, limits = np.asarray(T, dtype=float)[:3, 3], _workspace
    out = []
    for i, axis in enumerate("xyz"):
        lo, hi = limits[axis]
        if not lo <= p[i] <= hi:
            out.append(f"{axis} = {p[i] * 1000:.0f} mm (limits {lo * 1000:.0f} to {hi * 1000:.0f})")
    return out


def workspace_distance(T):
    """Distance [m] from the TCP of pose T to the workspace box (0 inside)."""
    p, limits = np.asarray(T, dtype=float)[:3, 3], _workspace
    lo = np.array([limits[axis][0] for axis in "xyz"])
    hi = np.array([limits[axis][1] for axis in "xyz"])
    return float(np.linalg.norm(p - np.clip(p, lo, hi)))


def workspace_allows(T_current, T_target):
    """Allow a target inside the box, or strictly closer to it than the current pose.

    This is the rule described by core.ur10_core.is_moving_towards_workspace. Its
    implementation measures the current pose against the *target's* nearest box
    point, so from inside the box it also accepts targets just outside it; the
    panel uses the distances to the box itself.
    """
    distance = workspace_distance(T_target)
    return distance == 0.0 or distance < workspace_distance(T_current) - 1e-9
