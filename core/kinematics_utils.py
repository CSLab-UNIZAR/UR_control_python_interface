import numpy as np
from numpy import linalg
from math import *
from core.ros_structs import *

ZERO_THRESH = 1e-8
PI = pi

# UR10 parameters
d1 = 0.1273
a2 = -0.612
a3 = -0.5723
d4 = 0.163941
d5 = 0.1157
d6 = 0.0922

# Define DH parameters for UR10
DH_matrix_UR10 = np.matrix([[0, pi / 2.0, 0.1273],
                            [-0.612, 0, 0],
                            [-0.5723, 0, 0],
                            [0, pi / 2.0, 0.163941],
                            [0, -pi / 2.0, 0.1157],
                            [0, 0, 0.0922]])

# Joint limits for UR10 (in radians)
joint_limits = [(-2 * pi, 2 * pi),  # Joint 1
                (-2 * pi, 2 * pi),  # Joint 2
                (-2 * pi, 2 * pi),  # Joint 3
                (-2 * pi, 2 * pi),  # Joint 4
                (-2 * pi, 2 * pi),  # Joint 5
                (-2 * pi, 2 * pi)]  # Joint 6

# Auxiliary Functions
def select_solution(q_sols, q_d, w=[1]*6):
    error = []
    filtered_sols = []

    # To extend the solution to the closest one to current joints
    new_q_sols = []
    for q in q_sols:
        ang_seed = q_d
       
        ang_new = q
       
        # We extend the current solution to +2pi and -2pi
        ang_new_2pi = ang_new + 2*pi
        ang_new_m2pi = ang_new - 2*pi

        # We choose the closest angles to the current joints
        ang_sel = ang_new
        
        for i in range(6):
            if abs(ang_seed[i] - ang_new_2pi[i]) < abs(ang_seed[i] - ang_sel[i]):
                ang_sel[i]=ang_new_2pi[i]

            if abs(ang_seed[i] - ang_new_m2pi[i]) < abs(ang_seed[i] - ang_sel[i]):
                ang_sel[i]=ang_new_m2pi[i]
        new_q_sols.append(ang_sel)
        
    for q in new_q_sols:
        if (q[1].item(0) < 0.0) and (q[2].item(0) >= 0.0) and (q[4].item(0) < 0.0): # We filter the options by where the shoulder is up
            filtered_sols.append(q)
   

    for q in filtered_sols:
        error.append(sum([w[i] * (q[i] - q_d[i]) ** 2 for i in range(6)]))

    return filtered_sols[error.index(min(error))]

def mat_transform_DH(DH_matrix, n, edges=np.matrix([[0], [0], [0], [0], [0], [0]])):
    """
    Calculate the transformation matrix for a given joint using DH parameters.

    :param DH_matrix: DH parameter matrix
    :param n: Joint number (1-indexed)
    :param edges: Joint angles
    :return: Transformation matrix for the joint
    """
    n = n - 1
    t_z_theta = np.matrix([[cos(edges[n, 0]), -sin(edges[n, 0]), 0, 0],
                           [sin(edges[n, 0]), cos(edges[n, 0]), 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]], copy=False)
    t_zd = np.matrix(np.identity(4), copy=False)
    t_zd[2, 3] = DH_matrix[n, 2]
    t_xa = np.matrix(np.identity(4), copy=False)
    t_xa[0, 3] = DH_matrix[n, 0]
    t_x_alpha = np.matrix([[1, 0, 0, 0],
                           [0, cos(DH_matrix[n, 1]), -sin(DH_matrix[n, 1]), 0],
                           [0, sin(DH_matrix[n, 1]), cos(DH_matrix[n, 1]), 0],
                           [0, 0, 0, 1]], copy=False)
    transform = t_z_theta * t_zd * t_xa * t_x_alpha
    return transform

def filter_solutions(solutions, joint_limits):
    """
    Filter out solutions that exceed the joint limits.

    :param solutions: List of joint angle matrices
    :param joint_limits: List of tuples containing the min and max limits for each joint
    :return: List of valid joint angle matrices
    """
    valid_solutions = []
    for solution in solutions:
        valid = True
        for j in range(6):
            if not (joint_limits[j][0] <= solution[j, 0] <= joint_limits[j][1]):
                valid = False
                break
        if valid:
            valid_solutions.append(solution)
    return valid_solutions

def inverse_kinematic_solution(DH_matrix, transform_matrix):
    theta = np.matrix(np.zeros((6, 8)))  # Initialize joint angles matrix
    T06 = transform_matrix  # T06: desired position and orientation of the final link

    # Calculate wrist position
    P05 = T06 * np.matrix([[0], [0], [-DH_matrix[5, 2]], [1]])  # P05: pose of the desire final link wrt base
    psi = atan2(P05[1, 0], P05[0, 0])  # psi = atan2(P05y/P05x)

    #phi = acos(
    #    np.clip((DH_matrix[1, 2] + DH_matrix[3, 2] + DH_matrix[2, 2]) / sqrt(P05[0, 0] ** 2 + P05[1, 0] ** 2), -1,
    #            1))  ## psi = acos(d4/sqrt( (P05x)^2 + (P05y)^2 )

    phi = acos(np.clip(d4 / sqrt(P05[0, 0] ** 2 + P05[1, 0] ** 2), -1, 1))  ## psi = acos(d4/sqrt( (P05x)^2 + (P05y)^2 )
    theta[0, 0:4] = psi + phi + pi / 2 # theta1 -- option 1 -- shoulder left
    theta[0, 4:8] = psi - phi + pi / 2 # theta1 -- option 2 -- shoulder right

    # Calculate theta 5
    for i in {0, 4}:
        # theta5 = +- acos( (pxs1 - pyc1 -d4) /d6), s1 = sin(theta1) and c1 = cos(theta1)
        th5cos = np.clip((T06[0, 3] * sin(theta[0, i]) - T06[1, 3] * cos(theta[0, i]) - d4) / d6, -1, 1)
        th5 = acos(th5cos)
        theta[4, i:i + 2] = th5
        theta[4, i + 2:i + 4] = -th5

    # Calculate theta 6 [changes made, we add the sin(th5)]
    for i in {0, 2, 4, 6}:
        T60 = linalg.inv(T06)
        th = atan2((-T60[1, 0] * sin(theta[0, i]) + T60[1, 1] * cos(theta[0, i]))/sin(theta[4, i]),
                   (T60[0, 0] * sin(theta[0, i]) - T60[0, 1] * cos(theta[0, i]))/sin(theta[4, i]))
        theta[5, i:i + 2] = th

    # Calculate theta 3
    for i in {0, 2, 4, 6}:
        T01 = mat_transform_DH(DH_matrix, 1, theta[:, i])
        T45 = mat_transform_DH(DH_matrix, 5, theta[:, i])
        T56 = mat_transform_DH(DH_matrix, 6, theta[:, i])
        T14 = linalg.inv(T01) * T06 * linalg.inv(T45 * T56)
        P13 = T14 * np.matrix([[0], [-d4], [0], [1]])
        costh3 = np.clip(((P13[0, 0] ** 2 + P13[1, 0] ** 2 - a2 ** 2 - a3 ** 2) /
                          (2 * a2 * a3)), -1, 1)
        th3 = acos(costh3)
        theta[2, i] = th3
        theta[2, i + 1] = -th3

    # Calculate theta 2 and theta 4
    for i in range(8):
        T01 = mat_transform_DH(DH_matrix, 1, theta[:, i])
        T45 = mat_transform_DH(DH_matrix, 5, theta[:, i])
        T56 = mat_transform_DH(DH_matrix, 6, theta[:, i])
        T14 = linalg.inv(T01) * T06 * linalg.inv(T45 * T56)
        P13 = T14 * np.matrix([[0], [-d4], [0], [1]])
        theta[1, i] = -atan2(P13[1, 0], -P13[0, 0]) + asin(
            np.clip(a3 * sin(theta[2, i]) / sqrt(P13[0, 0] ** 2 + P13[1, 0] ** 2), -1, 1)
        )
        T32 = linalg.inv(mat_transform_DH(DH_matrix, 3, theta[:, i]))
        T21 = linalg.inv(mat_transform_DH(DH_matrix, 2, theta[:, i]))
        T34 = T32 * T21 * T14
        theta[3, i] = atan2(T34[1, 0], T34[0, 0])

    return [theta[:, i] for i in range(8)]

def forward(edges=np.matrix([[0], [0], [0], [0], [0], [0]])):
    """
    Calculate the forward kinematics for the robot.

    :param DH_matrix: DH parameter matrix
    :param edges: Joint angles
    :return: Transformation matrix from base to end effector
    """
    t01 = mat_transform_DH(DH_matrix_UR10, 1, edges)
    t12 = mat_transform_DH(DH_matrix_UR10, 2, edges)
    t23 = mat_transform_DH(DH_matrix_UR10, 3, edges)
    t34 = mat_transform_DH(DH_matrix_UR10, 4, edges)
    t45 = mat_transform_DH(DH_matrix_UR10, 5, edges)
    t56 = mat_transform_DH(DH_matrix_UR10, 6, edges)
    answer = t01 * t12 * t23 * t34 * t45 * t56
    return answer

def forward_all(edges=np.matrix([[0], [0], [0], [0], [0], [0]])):
    """
    Calculate the forward kinematics for the robot.

    :param DH_matrix: DH parameter matrix
    :param edges: Joint angles
    :return: Transformation matrix from base to end effector
    """
    t01 = mat_transform_DH(DH_matrix_UR10, 1, edges)
    t12 = mat_transform_DH(DH_matrix_UR10, 2, edges)
    t23 = mat_transform_DH(DH_matrix_UR10, 3, edges)
    t34 = mat_transform_DH(DH_matrix_UR10, 4, edges)
    t45 = mat_transform_DH(DH_matrix_UR10, 5, edges)
    t56 = mat_transform_DH(DH_matrix_UR10, 6, edges)
    return np.array ([t01, t01 * t12, t01 * t12 * t23, t01 * t12 * t23 * t34, t01 * t12 * t23 * t34 * t45, t01 * t12 * t23 * t34 * t45 * t56])

def inverse(T_06, q_d, i_unit='r', o_unit='r'):
    """Solve the joint values based on an HTM.
    Args:
        T_06: An HTM representing the desired pose of the end-effector.
        q_d: A list of desired joint value solution (unit: radian).
        i_unit: Output format. 'r' for radian; 'd' for degree.
        o_unit: Output format. 'r' for radian; 'd' for degree.
    Returns:
        A list of optimal joint value solution.
    """

    if i_unit == 'd':
        q_d = [radians(i) for i in q_d]

    theta = inverse_kinematic_solution(DH_matrix_UR10, T_06)

    # Filter solutions that are out of joint limits
    valid_solutions = filter_solutions(theta, joint_limits)

    # Select the most close solution
    q_sol = select_solution(valid_solutions, q_d)
  
    # Output format
    if o_unit == 'r':  # (unit: radian)
        return q_sol
    elif o_unit == 'd':  # (unit: degree)
        return [degrees(i) for i in q_sol]
