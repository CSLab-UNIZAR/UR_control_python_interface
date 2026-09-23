import core.kinematics_utils as ur10_kin
import numpy as np

WORKSPACE_LIMITS = {
    'x': (-1.3, -0.3),     # minimun and maximum in X
    'y': (-0.45, 0.8),      # minimum and maximum in Y
    'z': (0.20, 0.8),       # minimum and maximum in Z
}

def is_within_workspace(T_pose):
    """Check if the given pose is within the defined workspace limits."""

    return (
        WORKSPACE_LIMITS['x'][0] <= T_pose[0, 3] <= WORKSPACE_LIMITS['x'][1] and
        WORKSPACE_LIMITS['y'][0] <= T_pose[1, 3] <= WORKSPACE_LIMITS['y'][1] and
        WORKSPACE_LIMITS['z'][0] <= T_pose[2, 3] <= WORKSPACE_LIMITS['z'][1]
    )

def is_moving_towards_workspace(T_current, T_target):
    """Check if the target pose is closer to the workspace than the current pose."""
    
    def clamp(val, min_val, max_val):
        return max(min_val, min(max_val, val))

    clamped_x = clamp(T_target[0, 3], *WORKSPACE_LIMITS['x'])
    clamped_y = clamp(T_target[1, 3], *WORKSPACE_LIMITS['y'])
    clamped_z = clamp(T_target[2, 3], *WORKSPACE_LIMITS['z'])

    current_dist = ((T_current[0, 3] - clamped_x)**2 +
                    (T_current[1, 3] - clamped_y)**2 +
                    (T_current[2, 3] - clamped_z)**2) ** 0.5
    target_dist = ((T_target[0, 3] - clamped_x)**2 +
                   (T_target[1, 3] - clamped_y)**2 +
                   (T_target[2, 3] - clamped_z)**2) ** 0.5

    return target_dist < current_dist 

def ur10_ikine(eef_pose, q0):
    """
    Inverse kinematics of the ur10 robot using the Levenberg-Marquardt method.
    Returns the best solution within joint limits
    """    
    return ur10_kin.inverse(eef_pose, q0).flatten().tolist()[0]


def ur10_fkine(joint_states):
    """
    Forward kinematics of the ur10 robot.
    """
    return ur10_kin.forward(edges=joint_states.reshape(-1,1))

def ur10_fkine_all(joint_states):
    """
    Forward kinematics of the UR10 robot for all joints.
    Returns a list of transformation matrices for each joint.
    """
    return ur10_kin.forward_all(edges=joint_states.reshape(-1,1))

def ur10_fkine_tcp(joint_states, tcp_offset=0.15):
    """
    Forward kinematics of the UR10 robot with TCP offset.
    """

    T_tool0 = ur10_fkine(joint_states)

    T_tcp_offset = np.eye(4)
    T_tcp_offset[2, 3] = tcp_offset 

    T_tcp = T_tool0 @ T_tcp_offset
    return T_tcp

def ur10_fkine_all_tcp(joint_states, tcp_offset=0.15):
    """
    Forward kinematics of the UR10 robot for all joints.
    Returns a list of transformation matrices for each joint.
    """
    
    T_tcp_offset = np.eye(4)
    T_tcp_offset[2, 3] = tcp_offset 

    T_06 = ur10_kin.forward_all(edges=joint_states.reshape(-1,1))
    T_06 = np.append(T_06, T_06[-1] @ T_tcp_offset)
    
    return T_06.reshape(7, 4, 4)


def ur10_ikine_tcp(desired_pose_tcp, joint_states, tcp_offset=0.15):
    """
    Inverse kinematics of the UR10 robot with TCP offset.
    """

    T_tcp_offset_inv = np.eye(4)
    T_tcp_offset_inv[2, 3] = -tcp_offset 

    T_desired_tool0 = desired_pose_tcp @ T_tcp_offset_inv

    return ur10_ikine(T_desired_tool0, joint_states)
