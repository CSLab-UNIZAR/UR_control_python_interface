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

T_base_to_0 = np.array([
    [-1,  0,  0,  0],
    [ 0, -1,  0,  0],
    [ 0,  0,  1,  0],
    [ 0,  0,  0,  1]
])

T_6_to_ee = np.array([
    [ 0, -1,  0,  0],
    [ 0,  0, -1,  0],
    [ 1,  0,  0,  0],
    [ 0,  0,  0,  1]
])

# Joint limits for UR10 (in radians)
joint_limits = [(-2 * pi, 2 * pi),  # Joint 1
                (-2 * pi, 2 * pi),  # Joint 2
                (-2 * pi, 2 * pi),  # Joint 3
                (-2 * pi, 2 * pi),  # Joint 4
                (-2 * pi, 2 * pi),  # Joint 5
                (-2 * pi, 2 * pi)]  # Joint 6

# Auxiliary Functions
def SIGN(x):
    return (x > 0) - (x < 0)

def select_solution(q_sols, q_d, w=None):
    if w is None:
        w = np.ones(6)
    
    error = []
    filtered_sols = []
    new_q_sols = []

    for q in q_sols:
        ang_seed = q_d
        ang_new = np.array(q)   # np.array
       
        # Extendimos la solución +2pi y -2pi
        ang_new_2pi = ang_new + 2*pi
        ang_new_m2pi = ang_new - 2*pi

        # Elegimos los ángulos más cercanos
        ang_sel = ang_new.copy()
        for i in range(6):
            if abs(ang_seed[i] - ang_new_2pi[i]) < abs(ang_seed[i] - ang_sel[i]):
                ang_sel[i] = ang_new_2pi[i]
            if abs(ang_seed[i] - ang_new_m2pi[i]) < abs(ang_seed[i] - ang_sel[i]):
                ang_sel[i] = ang_new_m2pi[i]
        new_q_sols.append(ang_sel)

    # Filtramos opciones según criterios de shoulder/wrist
    for q in new_q_sols:
        if (q[1] < 0.0) and (q[2] >= 0.0) and (q[4] < 0.0):
            filtered_sols.append(q)

    # Calculamos el error ponderado respecto a q_d
    for q in filtered_sols:
        error.append(np.sum(w * (q - q_d) ** 2))

    # Retornamos la solución con el menor error
    if filtered_sols:
        return filtered_sols[np.argmin(error)]
    else:
        return None

def filter_solutions(solutions, joint_limits):
    valid_solutions = []
    for solution in solutions:
        valid = True
        for j in range(6):
            if not (joint_limits[j][0] <= solution[j] <= joint_limits[j][1]):
                valid = False
                break
        if valid:
            valid_solutions.append(solution)
    return valid_solutions

def inverse(T_06, q_d, i_unit='r', o_unit='r'):

    T_target = T_base_to_0 @ T_06 
    T_0_6_desired = T_target @ T_6_to_ee

    if i_unit == 'd':
        q_d = [radians(i) for i in q_d]

    theta = inverse_kin_sol(T_0_6_desired, q_d)
    
    # Filter solutions that are out of joint limits
    valid_solutions = filter_solutions(theta, joint_limits)

    # Select the most close solution
    q_sol = select_solution(valid_solutions, q_d)
  
    # Output format
    if o_unit == 'r':  # (unit: radian)
        return q_sol
    elif o_unit == 'd':  # (unit: degree)
        return [degrees(i) for i in q_sol]

def forward(q):
    q1, q2, q3, q4, q5, q6 = q

    s1, c1 = sin(q1), cos(q1)
    s2, c2 = sin(q2), cos(q2)
    s3, c3 = sin(q3), cos(q3)
    s4, c4 = sin(q4), cos(q4)
    s5, c5 = sin(q5), cos(q5)
    s6, c6 = sin(q6), cos(q6)

    q23 = q2 + q3
    q234 = q2 + q3 + q4

    s23, c23 = sin(q23), cos(q23)
    s234, c234 = sin(q234), cos(q234)

    T = np.zeros((4, 4))

    T[0,0] = c234*c1*s5 - c5*s1
    T[0,1] = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6
    T[0,2] = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6
    T[0,3] = d6*c234*c1*s5 - a3*c23*c1 - a2*c1*c2 - d6*c5*s1 - d5*s234*c1 - d4*s1

    T[1,0] = c1*c5 + c234*s1*s5
    T[1,1] = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6
    T[1,2] = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1
    T[1,3] = d6*(c1*c5 + c234*s1*s5) + d4*c1 - a3*c23*s1 - a2*c2*s1 - d5*s234*s1

    T[2,0] = -s234*s5
    T[2,1] = -c234*s6 - s234*c5*c6
    T[2,2] = s234*c5*s6 - c234*c6
    T[2,3] = d1 + a3*s23 + a2*s2 - d5*(c23*c4 - s23*s4) - d6*s5*(c23*s4 + s23*c4)

    T[3,3] = 1.0

    return T_base_to_0 @ T @ np.linalg.inv(T_6_to_ee)


def forward_all(q):
    q1, q2, q3, q4, q5, q6 = q

    s1, c1 = sin(q1), cos(q1)
    s2, c2 = sin(q2), cos(q2)
    s3, c3 = sin(q3), cos(q3)
    s5, c5 = sin(q5), cos(q5)
    s6, c6 = sin(q6), cos(q6)

    q23 = q2 + q3
    q234 = q2 + q3 + q4

    s23, c23 = sin(q23), cos(q23)
    s234, c234 = sin(q234), cos(q234)

    T1 = np.eye(4)
    T2 = np.eye(4)
    T3 = np.eye(4)
    T4 = np.eye(4)
    T5 = np.eye(4)
    T6 = np.eye(4)

    # T1
    T1[0,0] = c1
    T1[0,2] = s1
    T1[1,0] = s1
    T1[1,2] = -c1
    T1[2,1] = 1
    T1[2,3] = d1

    # T2
    T2[0,0] = c1*c2
    T2[0,1] = -c1*s2
    T2[0,2] = s1
    T2[0,3] = a2*c1*c2

    T2[1,0] = c2*s1
    T2[1,1] = -s1*s2
    T2[1,2] = -c1
    T2[1,3] = a2*c2*s1

    T2[2,0] = s2
    T2[2,1] = c2
    T2[2,3] = d1 + a2*s2

    # T3
    T3[0,0] = c23*c1
    T3[0,1] = -s23*c1
    T3[0,2] = s1
    T3[0,3] = c1*(a3*c23 + a2*c2)

    T3[1,0] = c23*s1
    T3[1,1] = -s23*s1
    T3[1,2] = -c1
    T3[1,3] = s1*(a3*c23 + a2*c2)

    T3[2,0] = s23
    T3[2,1] = c23
    T3[2,3] = d1 + a3*s23 + a2*s2

    # T4
    T4[0,0] = c234*c1
    T4[0,1] = s1
    T4[0,2] = s234*c1
    T4[0,3] = c1*(a3*c23 + a2*c2) + d4*s1

    T4[1,0] = c234*s1
    T4[1,1] = -c1
    T4[1,2] = s234*s1
    T4[1,3] = s1*(a3*c23 + a2*c2) - d4*c1

    T4[2,0] = s234
    T4[2,2] = -c234
    T4[2,3] = d1 + a3*s23 + a2*s2

    # T5
    T5[0,0] = s1*s5 + c234*c1*c5
    T5[0,1] = -s234*c1
    T5[0,2] = c5*s1 - c234*c1*s5
    T5[0,3] = c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1

    T5[1,0] = c234*c5*s1 - c1*s5
    T5[1,1] = -s234*s1
    T5[1,2] = -c1*c5 - c234*s1*s5
    T5[1,3] = s1*(a3*c23 + a2*c2) - d4*c1 + d5*s234*s1

    T5[2,0] = s234*c5
    T5[2,1] = c234
    T5[2,2] = -s234*s5
    T5[2,3] = d1 + a3*s23 + a2*s2 - d5*c234

    # T6
    T6[0,0] = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6
    T6[0,1] = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6
    T6[0,2] = c5*s1 - c234*c1*s5
    T6[0,3] = d6*(c5*s1 - c234*c1*s5) + c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1

    T6[1,0] = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6
    T6[1,1] = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1
    T6[1,2] = -c1*c5 - c234*s1*s5
    T6[1,3] = s1*(a3*c23 + a2*c2) - d4*c1 - d6*(c1*c5 + c234*s1*s5) + d5*s234*s1

    T6[2,0] = c234*s6 + s234*c5*c6
    T6[2,1] = c234*c6 - s234*c5*s6
    T6[2,2] = -s234*s5
    T6[2,3] = d1 + a3*s23 + a2*s2 - d5*c234 - d6*s234*s5

    return T_base_to_0 @ T1, T_base_to_0 @ T2, T_base_to_0 @ T3, T_base_to_0 @ T4, T_base_to_0 @ T5, T_base_to_0 @ T6

def inverse_kin_sol(T, q6_des):
    T00, T01, T02, T03 = T[0,:]
    T10, T11, T12, T13 = T[1,:]
    T20, T21, T22, T23 = T[2,:]

    T02 = -T02
    T03 = -T03
    T12 = -T12
    T13 = -T13
    T20 = -T20
    T21 = -T21

    q_sols = []
    num_sols = 0

    # ---------- q1 (shoulder rotate) ----------
    q1_candidates = []
    A = d6*T12 - T13
    B = d6*T02 - T03
    R = A*A + B*B

    if fabs(A) < ZERO_THRESH:
        if fabs(fabs(d4)-fabs(B)) < ZERO_THRESH:
            div = -SIGN(d4)*SIGN(B)
        else:
            div = -d4/B
        arcsin_val = asin(div)
        if fabs(arcsin_val) < ZERO_THRESH:
            arcsin_val = 0.0
        q1_candidates.append(arcsin_val if arcsin_val>=0 else 2*PI+arcsin_val)
        q1_candidates.append(PI - arcsin_val)
    elif fabs(B) < ZERO_THRESH:
        if fabs(fabs(d4)-fabs(A)) < ZERO_THRESH:
            div = SIGN(d4)*SIGN(A)
        else:
            div = d4/A
        arccos_val = acos(div)
        q1_candidates.append(arccos_val)
        q1_candidates.append(2*PI - arccos_val)
    elif d4*d4 > R:
        return []
    else:
        arccos_val = acos(d4/sqrt(R))
        arctan_val = atan2(-B, A)
        pos = arccos_val + arctan_val
        neg = -arccos_val + arctan_val
        pos = 0.0 if fabs(pos)<ZERO_THRESH else pos
        neg = 0.0 if fabs(neg)<ZERO_THRESH else neg
        q1_candidates.append(pos if pos>=0 else 2*PI+pos)
        q1_candidates.append(neg if neg>=0 else 2*PI+neg)

    # ---------- q5 (wrist 2) ----------
    for q1 in q1_candidates:
        q5_candidates = []
        for sign in [0,1]:
            numer = (T03*sin(q1) - T13*cos(q1) - d4)
            div = SIGN(numer)*SIGN(d6) if fabs(fabs(numer)-fabs(d6))<ZERO_THRESH else numer/d6
            arccos_val = acos(div)
            q5_candidates.append(arccos_val)
            q5_candidates.append(2*PI - arccos_val)

        # ---------- q6 (wrist 3) ----------
        for q5 in q5_candidates:
            c5 = cos(q5)
            s5 = sin(q5)

            if fabs(s5) < ZERO_THRESH:
                q6 = q6_des
            else:
                q6 = atan2(SIGN(s5)*-(T01*sin(q1) - T11*cos(q1)),
                           SIGN(s5)*(T00*sin(q1) - T10*cos(q1)))
                if fabs(q6)<ZERO_THRESH:
                    q6 = 0.0
                if q6 < 0.0:
                    q6 += 2*PI

            # ---------- q2,q3,q4 ----------
            c6 = cos(q6)
            s6 = sin(q6)

            x04x = -s5*(T02*cos(q1) + T12*sin(q1)) - c5*(s6*(T01*cos(q1) + T11*sin(q1)) - c6*(T00*cos(q1) + T10*sin(q1)))
            x04y = c5*(T20*c6 - T21*s6) - T22*s5
            p13x = d5*(s6*(T00*cos(q1) + T10*sin(q1)) + c6*(T01*cos(q1) + T11*sin(q1))) - d6*(T02*cos(q1) + T12*sin(q1)) + T03*cos(q1) + T13*sin(q1)
            p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6)

            # q3
            c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3)/(2*a2*a3)
            if fabs(fabs(c3)-1.0) < ZERO_THRESH:
                c3 = SIGN(c3)
            elif fabs(c3) > 1.0:
                continue
            arccos_c3 = acos(c3)
            q3_vals = [arccos_c3, 2*PI-arccos_c3]

            # q2
            s3 = sin(arccos_c3)
            denom = a2*a2 + a3*a3 + 2*a2*a3*c3
            A = a2 + a3*c3
            B = a3*s3
            q2_vals = [
                atan2((A*p13y - B*p13x)/denom, (A*p13x + B*p13y)/denom),
                atan2((A*p13y + B*p13x)/denom, (A*p13x - B*p13y)/denom)
            ]

            # q4
            q4_vals = []
            for q2,q3 in zip(q2_vals,q3_vals):
                c23 = cos(q2+q3)
                s23 = sin(q2+q3)
                q4 = atan2(c23*x04y - s23*x04x, x04x*c23 + x04y*s23)
                if fabs(q4)<ZERO_THRESH:
                    q4 = 0.0
                elif q4<0:
                    q4 += 2*PI
                q4_vals.append(q4)

            # Guardar todas las soluciones posibles
            for k in range(2):
                q_sol = [q1, q2_vals[k], q3_vals[k], q4_vals[k], q5, q6]
                q_sols.append(q_sol)
                num_sols += 1

    return q_sols