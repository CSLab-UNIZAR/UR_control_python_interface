from blessed import Terminal
import copy
import time 
from core.ur_control import * 
import matplotlib.pyplot as plt


ur10_control = UR10Control(host_ip='CMP00-180723AD.local')

# -------------------------------------------- MAIN PROGRAM ---------------------------------------

term = Terminal()

def get_last_key():
    key = None
    val = term.inkey(timeout=0)
    while val:
        key = val
        val = term.inkey(timeout=0)
    return key


plt.ion()
fig = plt.figure(figsize=(7,6))
    
def keyboard_control():

    current_axis = None
    action_msg = ""
    mode = 'translation'

    print(term.clear)
    print("Controls: 'x', 'y', 'z' axis | ↑ ↓ move/rotate | 'r' mode | 'o','c' gripper | 'q' quit")

    with term.cbreak(), term.hidden_cursor():
        while True:

            key = get_last_key()

            T_base_tcp = copy.deepcopy(ur10_control.T_current)

            if key:
                k = key.lower()
            else:
                k = None

            # --- Exot ---
            if k == 'q':
                ur10_control.close_connection()
                break

            # --- Axis ---
            if k in ('x', 'y', 'z'):
                current_axis = k
                action_msg = f"Axis changed to '{current_axis}'"

            # --- Change mode ---
            if k == 'r':
                mode = 'rotation' if mode == 'translation' else 'translation'
                action_msg = f"Mode changed to {mode}"

            # --- Gripper ---
            if k == 'o':
                action_msg = "Opening gripper"
                ur10_control.send_gripper_cmd("Open")
    
            elif k == 'c':
                action_msg = "Closing gripper"
                ur10_control.send_gripper_cmd("Close")
                
            # --- Movimiento / Rotación ---
            if current_axis:
                if key and hasattr(key, "code") and key.code in (term.KEY_UP, term.KEY_DOWN):
                    direction = 1 if key.code == term.KEY_UP else -1

                    if mode == 'translation':
                        step = 0.05
                        action_msg = f"Moving {direction * step} in '{current_axis}' axis"

                        if current_axis == 'x': T_base_tcp[0, 3] += step * direction
                        if current_axis == 'y': T_base_tcp[1, 3] += step * direction
                        if current_axis == 'z': T_base_tcp[2, 3] += step * direction

                    else:  # rotation
                        rot_step = 0.10
                        rpy_delta = np.zeros(3)
                        axis_index = ['x', 'y', 'z'].index(current_axis)
                        rpy_delta[axis_index] = rot_step * direction

                        T_delta_tcp = np.eye(4)
                        T_delta_tcp[:3, :3] = SE3.RPY(rpy_delta, unit='rad', order='zyx').R
                        T_base_tcp = T_base_tcp @ T_delta_tcp
                        
                    ur10_control.send_trajectory(T_base_tcp)

            print(term.move_xy(0, 2) + f"Current axis: {current_axis if current_axis else 'none'}   ")
            print(term.move_xy(0, 4) + f"{action_msg}   ")

            time.sleep(0.01)

time.sleep(2)

keyboard_control()


