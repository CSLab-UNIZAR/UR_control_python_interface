import time 
from core.ur_control import * 
from core.ur10_core import *
import warnings
warnings.filterwarnings("ignore")

ur10_control = UR10Control(host_ip='CMP00-180723AD.local')
time.sleep(2)

# The gripper stays in the same orientation
def force_control1():
    force_history = []   
    moving = False
 
    INIT_TRESH = 3.0       
    MOVEMENT = 0.05 

    while True:

        T_tcp = ur10_fkine_tcp(ur10_control.joint_states)

        force_history.append(ur10_control.current_force)
        if (len(force_history) > 10):
            force_history.pop(0)

        avg_force = np.mean(force_history, axis=0)

        # Filter forces
        for i in range(len(avg_force)):
            if (abs(avg_force[i]) < INIT_TRESH):
                avg_force[i] = 0.0

        magnitude = np.linalg.norm(avg_force)
                
        if (not moving and magnitude > 0): 
            moving = True
            print("Force detected, starting movement")

        elif (moving and magnitude == 0): 
            moving = False
            print("Force below threshold, stopping movement")
          
        if moving:

            R_too2base = T_tcp[:3, :3]          # Base to tcp rotation
            force_base = R_too2base @ avg_force.T 
            direct = force_base / np.linalg.norm(force_base)

            #print(direct)

            T_tcp[0, 3] += direct[0] * MOVEMENT 
            T_tcp[1, 3] += direct[1] * MOVEMENT
            T_tcp[2, 3] += direct[2] * MOVEMENT
            
            ur10_control.send_trajectory(T_tcp, tcp_mode=True)

        time.sleep(0.02)

def force_control2():

    force_history = []
    torque_history = []
   
    moving = False
    rotating = False
    block_rotation = False

    offset_force = np.zeros(3)
    offset_torque = np.zeros(3)

    INIT_TRESH = 2.5
    ROT_THESH = 0.7 
    MOVEMENT = 0.05
    ROTATION = 0.05
    ROT_MAX = 30
   
    # Get the current orientation of the gripper
   
    rot_init = ur10_control.T_current[:3, :3]
    
    while True:

        T_tcp = ur10_control.T_current

        if not moving and not rotating and len(torque_history) > 0:

            avg_torque = np.mean(torque_history, axis=0)
            avg_force = np.mean(force_history, axis=0)
        
            offset_force = avg_force
            offset_torque = avg_torque
        else:
            offset_force = np.zeros(3)
            offset_torque = np.zeros(3)

        # Siempre corregir
        current_force_corrected = ur10_control.current_force - offset_force
        current_torque_corrected = ur10_control.current_torque - offset_torque

        torque_history.append(current_torque_corrected)
        if (len(torque_history) > 20):
            torque_history.pop(0)
        avg_torque = np.mean(torque_history, axis=0)

        # Filter torques
        for i in range(len(avg_torque)):
            if (abs(avg_torque[i]) < ROT_THESH):
                avg_torque[i] = 0.0

        torque_magnitude = np.linalg.norm(avg_torque)

        force_history.append(current_force_corrected)
        if (len(force_history) > 20):
            force_history.pop(0)
        avg_force = np.mean(force_history, axis=0)

        # Filter forces
        for i in range(len(avg_force)):
            if (abs(avg_force[i]) < INIT_TRESH):
                avg_force[i] = 0.0

        magnitude = np.linalg.norm(avg_force)
        
        if (not moving and magnitude > 0): 
            moving = True
            print("Force detected, starting movement")

        elif (moving and magnitude == 0): 
            moving = False
            print("Force below threshold, stopping movement")
            
        if (not rotating and torque_magnitude > 0): 
            rotating = True
            print("Torque detected, starting rotation")
        elif (rotating and torque_magnitude == 0):
            rotating = False
            print("Torque below threshold, stopping rotation")
   
        if moving or rotating:
     
            R_too2base = T_tcp[:3, :3]          # Base to tcp rotation
            force_base = R_too2base @ avg_force.reshape(3, 1)

            direct = np.zeros(3)
            if moving:
                direct = force_base / np.linalg.norm(force_base)

            T_tcp[:3, 3] += direct * MOVEMENT 
          
            rpy_rot = np.zeros(3)
            if rotating:
                rpy_rot = (avg_torque / torque_magnitude) * ROTATION

            T_rot = np.eye(4)
            T_rot[:3, :3] = SE3.RPY(rpy_rot, unit='rad', order='zyx').R
            
            T_base_tcp_new = T_tcp @ T_rot

            # Rotation control
            diff_rot = rot_diff(T_base_tcp_new[:3, :3], rot_init)  # Calculate the difference from the initial orientation
            block_rotation = any(abs(ax) > ROT_MAX for ax in diff_rot)

            if not block_rotation:

                T_tcp = T_base_tcp_new  # Reset to the original pose if rotation is blocked

            ur10_control.send_trajectory(T_tcp)

        time.sleep(0.05)


try:
    
    print("Starting force control...")
    force_control2()

except KeyboardInterrupt:

    print("Shutting down ROS connection...")
    ur10_control.close_connection()
