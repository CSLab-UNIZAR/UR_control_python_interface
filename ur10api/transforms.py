"""Small helpers to build, combine and compare poses.

A pose is a 4x4 homogeneous transform (numpy array): rotation in [:3, :3],
position in metres in [:3, 3]. Robot poses are expressed in the UR base frame
(the pendant's "Base"). Angles are radians unless a name ends in _deg.

    from ur10api.transforms import trans, rotz, pose_error

    target = start @ trans(0.05, 0, 0) @ rotz(20)   # 5 cm along the tool X axis, then 20 deg about tool Z
    dp, drot = pose_error(current, target)          # what is left to move, in the base frame
"""

import numpy as np
from scipy.spatial.transform import Rotation


def pose(position=(0.0, 0.0, 0.0), rotation=None):
    """Pose from a position [m] and a rotation (3x3 matrix or rotation vector [rad])."""
    T = np.eye(4)
    if rotation is not None:
        rotation = np.asarray(rotation, dtype=float)
        T[:3, :3] = rotation if rotation.shape == (3, 3) else Rotation.from_rotvec(rotation).as_matrix()
    T[:3, 3] = position
    return T


def trans(x=0.0, y=0.0, z=0.0):
    """Pure translation [m]."""
    return pose((x, y, z))


def rotx(angle_deg):
    """Pure rotation about X [deg]."""
    return pose(rotation=Rotation.from_euler("x", angle_deg, degrees=True).as_matrix())


def roty(angle_deg):
    """Pure rotation about Y [deg]."""
    return pose(rotation=Rotation.from_euler("y", angle_deg, degrees=True).as_matrix())


def rotz(angle_deg):
    """Pure rotation about Z [deg]."""
    return pose(rotation=Rotation.from_euler("z", angle_deg, degrees=True).as_matrix())


def inverse(T):
    """Inverse of a pose (cheaper and more exact than np.linalg.inv)."""
    T = np.asarray(T, dtype=float)
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def pose_to_xyzrpy(T):
    """Pose -> [x, y, z, roll, pitch, yaw] in metres and radians.

    Roll/pitch/yaw follow UR's convention R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    (the same as SE3.RPY(..., order='zyx') used by the controllers).
    """
    T = np.asarray(T, dtype=float)
    yaw, pitch, roll = Rotation.from_matrix(T[:3, :3]).as_euler("ZYX")
    return np.array([T[0, 3], T[1, 3], T[2, 3], roll, pitch, yaw])


def xyzrpy_to_pose(xyzrpy):
    """[x, y, z, roll, pitch, yaw] in metres and radians -> pose."""
    x, y, z, roll, pitch, yaw = (float(v) for v in xyzrpy)
    return pose((x, y, z), Rotation.from_euler("ZYX", [yaw, pitch, roll]).as_matrix())


def rotation_vector(T):
    """Axis-angle vector [rad] of the pose orientation: the RX, RY, RZ shown by the UR pendant."""
    return Rotation.from_matrix(np.asarray(T, dtype=float)[:3, :3]).as_rotvec()


def displace(T, dp=(0.0, 0.0, 0.0), drot=(0.0, 0.0, 0.0), frame="base"):
    """Translate a pose by dp [m] and rotate it by rotation vector drot [rad] about its own origin.

    frame="base": dp and drot are given in the base frame (world axes).
    frame="tool": dp and drot are given in the frame of the pose itself.
    """
    T = np.asarray(T, dtype=float)
    R, p = T[:3, :3], T[:3, 3]
    dR = Rotation.from_rotvec(np.asarray(drot, dtype=float)).as_matrix()
    if frame == "tool":
        return pose(p + R @ np.asarray(dp, dtype=float), R @ dR)
    if frame == "base":
        return pose(p + np.asarray(dp, dtype=float), dR @ R)
    raise ValueError("frame must be 'base' or 'tool'")


def pose_error(T_current, T_target):
    """What is left to move from T_current to T_target, both in the base frame.

    Returns (dp, drot): the position difference [m] and the rotation vector
    [rad] that turns the current orientation into the target one. Feeding
    (gain * dp, gain * drot) to Robot.set_tcp_velocity gives a simple pose controller.
    """
    Tc, Tt = np.asarray(T_current, dtype=float), np.asarray(T_target, dtype=float)
    dp = Tt[:3, 3] - Tc[:3, 3]
    drot = Rotation.from_matrix(Tt[:3, :3] @ Tc[:3, :3].T).as_rotvec()
    return dp, drot


def rotation_angle(T_a, T_b):
    """Angle [rad] between the orientations of two poses."""
    return float(np.linalg.norm(pose_error(T_a, T_b)[1]))
