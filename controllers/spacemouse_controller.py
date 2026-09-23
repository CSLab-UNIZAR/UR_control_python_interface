import time 
from core.ur_control import * 
from core.spacemouse import *
import os
import warnings
warnings.filterwarnings("ignore")

# Execute in terminal -> sudo evtest --grab /dev/input/event20 -> see the event with cat /proc/bus/input/devices
# If there is permission error execute -> sudo chmod 666 /dev/hidraw*

# ------------------------------ UR10 CONTROL ------------------------------ # 

ur10_control = UR10Control(host_ip='CMP00-180723AD.local')
time.sleep(1)

# ------------------------------ INITIAL STATE ------------------------------ # 

state = SpaceState()
state.T_init = ur10_control.T_current.copy()
state.T_ref = ur10_control.T_current.copy()

# ------------------------------ EXECUTION CONFIG ------------------------------ # 

config = ExecConfig()
config.DEADZONE = 0.2
config.JOINT_GAIN = 0.1
config.TCP_ROTATION_GAIN = 0.5
config.TCP_TRANSLATION_GAIN = 0.5
config.LOOP_DT = 0.001

# ------------------------------ SPACEMOUSE CLASS ------------------------------ # 

spacemouse = SpaceMouse(state, ur10_control, config)

# ------------------------------ CONTROL LOOP ------------------------------ # 

while True:
    if spacemouse.running == False: os._exit(0)

    if spacemouse.state.filtered_state is None:
        continue
        
    control_func = spacemouse.tcp_control if spacemouse.state.mode == 'tcp' else spacemouse.joint_control
    control_func() 
     
    time.sleep(config.LOOP_DT)
