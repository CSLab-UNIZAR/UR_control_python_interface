import numpy as np
from core.ros_structs import Pose
from spatialmath import SE3
from spatialmath.base import q2r, r2q 
from scipy.spatial.transform import Rotation as R
# import torch

# ------------------- NUMPY FUNCIONS -------------------
def np2ros(pose_np):
    """
    Converts a numpy array to a ROS Pose message.
    """
    pose_ros = Pose()
    pose_ros.position.x = pose_np[0, 3]
    pose_ros.position.y = pose_np[1, 3]
    pose_ros.position.z = pose_np[2, 3]

    R = pose_np[:3, :3]
    q = r2q(R)  # Convert rotation matrix to quaternion
    pose_ros.orientation.w = q[0]
    pose_ros.orientation.x = q[1]
    pose_ros.orientation.y = q[2]
    pose_ros.orientation.z = q[3]

    return pose_ros

def ros2np(pose_ros):
    """
    Converts a ROS Pose message to a numpy array.
    """
    p = [pose_ros.position.x, pose_ros.position.y, pose_ros.position.z]
    q = np.array([pose_ros.orientation.w, pose_ros.orientation.x, pose_ros.orientation.y, pose_ros.orientation.z])
    
    q = q / np.linalg.norm(q) # Normalize quaternion

    R = q2r(q)
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = p

    return T

def rpy2q(roll, pitch, yaw, unit='rad', order='zyx'):
    """
    Converts roll, pitch, yaw angles to a quaternion.
    """
    
    R = SE3.RPY([roll, pitch, yaw], order=order, unit=unit).R
    q = r2q(R)  

    return q

def rpy2r(roll, pitch, yaw, unit='rad', order='zyx'):
    R = SE3.RPY([roll, pitch, yaw], order=order, unit=unit).R
    return R

def q2rpy(q, deg = False): # SOLO PARA MENSAJES DE ROS, LA W ESTA AL FINAL
    """
    Converts a quaternion to roll, pitch, yaw angles.
    """

    r = R.from_quat([q[0], q[1], q[2], q[3]])  
    return r.as_euler('zyx', degrees=deg)

def q2rpy_(q, deg = False): # Resto
    """
    Converts a quaternion to roll, pitch, yaw angles.
    """

    r = R.from_quat([q[1], q[2], q[3], q[0]])  
    return r.as_euler('xyz', degrees=deg)

def r2rpy(R, deg = False):
    """
    Converts a rotation matrix to roll, pitch, yaw angles.
    """

    r = R.from_matrix(R)
    return r.as_euler('zyx', degrees=deg)

def rot_diff(R0, R1):
    """
    Computes the difference between two rotation matrices R0 and R1.
    Returns the roll, pitch, and yaw angles of the relative rotation.
    """

    R_local = R0.T @ R1

    return R.from_matrix(R_local).as_euler('zyx', degrees=True)


# ------------------- TORCH FUNCIONS -------------------
# def quat2matrix(quat, device):
#     """
#     Converts a quaternion to a rotation matrix.
#     """
#     quat = quat / torch.linalg.norm(quat)
#     w, x, y, z = quat
#     R = torch.tensor([
#         [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
#         [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
#         [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
#     ], device=device)

#     return R

# def euler2quat(roll, pitch, yaw):
#     """
#     Converts roll, pitch, yaw angles to a quaternion.
#     """
#     cr = torch.cos(roll * 0.5)
#     sr = torch.sin(roll * 0.5)
#     cp = torch.cos(pitch * 0.5)
#     sp = torch.sin(pitch * 0.5)
#     cy = torch.cos(yaw * 0.5)
#     sy = torch.sin(yaw * 0.5)

#     w = cr*cp*cy + sr*sp*sy
#     x = sr*cp*cy - cr*sp*sy
#     y = cr*sp*cy + sr*cp*sy
#     z = cr*cp*sy - sr*sp*cy

#     return torch.stack([w, x, y, z], dim=-1)

# def quat2euler(quat):
#     """
#     Converts a quaternion to roll, pitch, yaw angles.
#     """
#     w, x, y, z = quat.unbind(-1)

#     # roll (x)
#     t0 = 2.0 * (w * x + y * z)
#     t1 = 1.0 - 2.0 * (x * x + y * y)
#     roll = torch.atan2(t0, t1)

#     # pitch (y)
#     t2 = 2.0 * (w * y - z * x)
#     t2 = torch.clamp(t2, -1.0, 1.0)
#     pitch = torch.asin(t2)

#     # yaw (z)
#     t3 = 2.0 * (w * z + x * y)
#     t4 = 1.0 - 2.0 * (y * y + z * z)
#     yaw = torch.atan2(t3, t4)

#     euler = torch.stack([roll, pitch, yaw], dim=-1)
#     return euler

# def compute_sin_cos_gpu(phi: torch.Tensor):
#     sin_phi = torch.sin(phi)
#     cos_phi = torch.cos(phi)
#     return torch.cat([sin_phi, cos_phi], dim=-1)
