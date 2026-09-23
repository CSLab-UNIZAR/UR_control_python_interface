"""Pose helpers built on the framework's kinematics (core.ur10_core).

Conventions (same as the rest of UR_CONTROL):
  * joints J1..J6 = shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3 [rad]
  * poses are 4x4 transforms of the TCP (tool0 + 0.15 m along tool Z) in the UR10 base frame
  * roll/pitch/yaw use the ZYX convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll),
    identical to SE3.RPY(..., order='zyx') used by the controllers
"""

import numpy as np
from scipy.spatial.transform import Rotation

from core.ur10_core import (
    WORKSPACE_LIMITS,
    is_moving_towards_workspace,
    is_within_workspace,
    ur10_fkine,
    ur10_fkine_tcp,
    ur10_ikine,
    ur10_ikine_tcp,
)

JOINT_LIMIT = 2 * np.pi   # software joint limit used by core/kinematics_utils.py
JOINT_NAMES = ("Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3")
CARTESIAN_AXES = ("X", "Y", "Z", "Rx", "Ry", "Rz")


class IKError(RuntimeError):
    """The framework's IK found no usable solution for a pose."""


def fk(q, tcp=True):
    """TCP (or flange) pose for joint vector q, as a plain 4x4 ndarray."""
    solver = ur10_fkine_tcp if tcp else ur10_fkine
    return np.asarray(solver(np.asarray(q, dtype=float)), dtype=float)


def ik(T, seed, tcp=True):
    """Joint solution closest to `seed`, using the same solver as UR10Control."""
    solver = ur10_ikine_tcp if tcp else ur10_ikine
    try:
        q = np.asarray(solver(np.asarray(T, dtype=float), np.asarray(seed, dtype=float)), dtype=float)
    except Exception as exc:  # the solver raises when no solution lies in its arm branch
        raise IKError("no inverse-kinematics solution for this pose") from exc
    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise IKError("inverse kinematics returned an invalid solution (singular pose)")
    if np.any(np.abs(q) > JOINT_LIMIT):
        raise IKError("the solution exceeds the +/-360 deg joint limits")
    return q


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


def workspace_violations(T):
    """Human-readable list of the workspace limits that pose T violates."""
    p = np.asarray(T, dtype=float)[:3, 3]
    out = []
    for i, axis in enumerate("xyz"):
        lo, hi = WORKSPACE_LIMITS[axis]
        if p[i] < lo:
            out.append(f"{axis} < {lo * 1000:.0f} mm")
        elif p[i] > hi:
            out.append(f"{axis} > {hi * 1000:.0f} mm")
    return out


def workspace_allows(T_current, T_target):
    """The rule of UR10Control.send_trajectory: inside the box, or moving towards it."""
    T_current = np.asarray(T_current, dtype=float)
    T_target = np.asarray(T_target, dtype=float)
    return is_within_workspace(T_target) or is_moving_towards_workspace(T_current, T_target)
