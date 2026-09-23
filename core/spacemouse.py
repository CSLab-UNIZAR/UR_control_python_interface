import numpy as np
from dataclasses import dataclass
from core.ur_control import * 
import pyspacemouse
import threading
import matplotlib.image as mpimg
from core.plots import *
import time 
from typing import Optional
import math

@dataclass
class SpaceState:
    T_ref : Optional[np.ndarray] = None
    T_init: Optional[np.ndarray] = None
    filtered_state: Optional[np.ndarray] = None
    mode: str = "tcp"
    mode_tcp: str = "translation"
    display: bool = False
    auto_save: bool = False
    joint: int = 0


@dataclass
class ExecConfig:
    DEADZONE = float 
    TCP_TRANSLATION_GAIN = float
    TCP_ROTATION_GAIN = float
    JOINT_GAIN = float
    LOOP_DT = float

class SpaceMouse:
    def __init__(self, space_state : SpaceState, ur10_control: UR10Control, config : ExecConfig):
        self.state = space_state
        self.ur10_control = ur10_control
        self.config = config
        self.threads = []
        self.running = True

        common_callbacks = [
            pyspacemouse.ButtonCallback(0, self.on_button_display),
            pyspacemouse.ButtonCallback(2, self.on_button_reset),
            pyspacemouse.ButtonCallback(4, self.on_button_exit),
            pyspacemouse.ButtonCallback(14, self.on_button_toggle_mode)
        ]

        self.callbacks = {
            "tcp": common_callbacks + [
                pyspacemouse.ButtonCallback(5, self.on_button_save),
                pyspacemouse.ButtonCallback([3,5], self.on_button_auto_save),
                pyspacemouse.ButtonCallback(6, self.on_button_toggle_tcp),
                pyspacemouse.ButtonCallback(7, self.on_button_open),
                pyspacemouse.ButtonCallback(8, self.on_button_close),
            ],
            "joint": common_callbacks + [
                pyspacemouse.ButtonCallback(5, self.on_button_change_joint),
                pyspacemouse.ButtonCallback(6, self.on_button_change_joint),
                pyspacemouse.ButtonCallback(7, self.on_button_change_joint),
                pyspacemouse.ButtonCallback(8, self.on_button_change_joint),
                pyspacemouse.ButtonCallback(9, self.on_button_change_joint),
                pyspacemouse.ButtonCallback(10, self.on_button_change_joint),
                pyspacemouse.ButtonCallback([3, 7], self.on_button_open),
                pyspacemouse.ButtonCallback([3, 8], self.on_button_close),
            ]
        }

        self.device = pyspacemouse.open(button_callbacks=self.callbacks[self.state.mode])
        print("Device Ready")

        # Start threads
        t1 = threading.Thread(target=self.read_mouse_thread)
        t1.daemon = True
        t1.start()
        self.threads.append(t1)

        t2 = threading.Thread(target=self.display_thread)
        t2.daemon = True
        t2.start()
        self.threads.append(t2)

    # ------------------------------------------- THREAD FUNCTIONS ------------------------------------------- #
    def read_mouse_thread(self):
        while self.running:

            state = self.device.read()
            
            new_state = np.array([
                state.x, state.y, state.z,
                state.roll, state.pitch, state.yaw
            ])

            self.state.filtered_state = np.where( abs(new_state) > self.config.DEADZONE, new_state, 0)
    
    def display_thread(self):
        
        plt.ion()
        fig = plt.figure(figsize=(10,5))

        ax_robot = fig.add_subplot(1,2,1, projection='3d')
        ax_img = fig.add_subplot(1,2,2)

        while self.running:
            if self.state.display:

                if fig is None or not plt.fignum_exists(fig.number):
                    fig = plt.figure(figsize=(10,5))
                    ax_robot = fig.add_subplot(1,2,1, projection='3d')
                    ax_img = fig.add_subplot(1,2,2)

                img = mpimg.imread(f"./other/{self.state.mode}_buttons.jpg")

                ax_robot.cla()
                ax_img.cla()

                if self.state.mode == "tcp":

                    plot_robot_tcp(ax_robot, ur10_fkine_all_tcp(self.ur10_control.joint_states), 
                                   self.state.filtered_state[:3] if self.state.mode_tcp == 'translation' else np.zeros(3), 
                                   self.state.filtered_state[3:] if self.state.mode_tcp == 'rotation' else np.zeros(3))
                else:
                    # Arreglar eje de giro
                    plot_robot_joint(ax_robot,  ur10_fkine_all_tcp(self.ur10_control.joint_states), self.state.joint, self.state.filtered_state[-1])

                ax_img.imshow(img)
                ax_img.axis("off")

                plt.draw()
                plt.pause(0.01)

            else:
                if fig is not None and plt.fignum_exists(fig.number):
                    plt.close(fig)
                fig = None
                ax_robot = None
                ax_img = None

    # ------------------------------------------- FLICKER LED ------------------------------------------- #

    def flicker(self):
        self.device.set_led(False)
        time.sleep(0.05)
        self.device.set_led(True)

    # ------------------------------------------- BUTTON CALLBACKS ------------------------------------------- #
                   
    def on_button_auto_save(self, _, __, pressed):
        
        self.state.auto_save = not self.state.auto_save
        print(f"Auto-save set to {self.state.auto_save}")
        self.flicker()

    def on_button_display(self, _, __, pressed):
       
        self.state.display = not self.state.display
        print(f"Setting display to {self.state.display}")
        self.flicker()

    def on_button_exit(self, _, __, pressed):
        
        print(f"Exiting...")

        self.ur10_control.close_connection()
        self.flicker()
        self.running = False


    def on_button_toggle_mode(self, _, __, pressed):

        self.state.mode = 'joint' if self.state.mode == 'tcp' else 'tcp'
        self.device._button_callbacks = self.callbacks[self.state.mode]
        print(f"Mode chaged to {self.state.mode}")
        self.flicker()
        
    def on_button_change_joint(self, _, __, pressed):
        
        self.state.joint = pressed - 5
        print(f"Joint changed to {pressed - 4}")
        self.flicker()

    def on_button_reset(self, _, __, pressed):

        print(f"Setting reference pose")
        self.state.T_ref = self.state.T_init.copy()
        self.flicker()

    def on_button_open(self, _, __, pressed):

        print(f"Opening gripper")
        self.ur10_control.send_gripper_cmd("Open")
        self.flicker()
        
    def on_button_close(self, _, __, pressed):

        print(f"Closing gripper")
        self.ur10_control.send_gripper_cmd("Close")
        self.flicker()

    def on_button_save(self, _, __, pressed):

        print(f"Setting reference pose")
        self.state.T_ref = self.ur10_control.T_current.copy()
        self.flicker()

    def on_button_toggle_tcp(self, _, __, pressed):

        self.state.mode_tcp = 'rotation' if self.state.mode_tcp == 'translation' else 'translation'
        print(f"TCP mode chaged to {self.state.mode_tcp}")
        self.flicker()


    # ------------------------------------------- MODE CONTROL ------------------------------------------- #
    
    def tcp_control(self):
        T_new = self.state.T_ref.copy()

        if self.state.mode_tcp == 'translation': 
            
            T_new[:3, 3] += self.state.filtered_state[:3].reshape(3, 1) * self.config.TCP_TRANSLATION_GAIN   
        
        else:

            T_delta_tcp = np.eye(4)
            T_delta_tcp[:3, :3] = SE3.RPY(self.state.filtered_state[3:] * self.config.TCP_ROTATION_GAIN, unit='rad', order='zyx').R
            T_new = T_new @ T_delta_tcp
            
        self.ur10_control.send_trajectory(T_new)

        if self.state.auto_save : self.state.T_ref = self.ur10_control.T_current.copy()

    def joint_control(self):

        joints_new = self.ur10_control.joint_states.copy()
        joints_new[self.state.joint] += self.state.filtered_state[-1] * self.config.JOINT_GAIN

        joints_clipped = np.array([ np.clip(val, -2 * math.pi, 2 * math.pi) for val in joints_new])

        self.ur10_control.send_trajectory(joints_clipped, art=True)

