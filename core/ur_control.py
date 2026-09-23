# import torch
import numpy as np
import roslibpy
from core.ur10_core import *
from core.conversions import *


class UR10Control:
    def __init__(self, host_ip= '192.168.0.200', port=9090, tcp_mode=True):
        
        # STARTING CONNECTION
        self.ros = roslibpy.Ros(host=host_ip, port=port)  #Host -> The Hostname of the ROS_MASTER_URI    
        self.ros.run()                                    #Port -> 9090 by default by ros_brige_server

        # DECLARING TOPICS
        self.joint_states_tp = roslibpy.Topic(self.ros, '/joint_states', 'sensor_msgs/JointState')
        self.pub_ik_traj_tp = roslibpy.Topic(self.ros, '/pub_ik_trajectory', 'trajectory_msgs/JointTrajectory', queue_size=1)
        self.pub_gripper_tp = roslibpy.Topic(self.ros, '/pub_gripper_control', 'trajectory_msgs/JointTrajectory', queue_size=1)
        self.clock_tp = roslibpy.Topic(self.ros, '/clock', 'rosgraph_msgs/Clock')
        self.force_tp = roslibpy.Topic(self.ros, '/robotiq_ft_sensor', 'robotiq_ft_sensor/ft_sensor')

        # DECLARING SERVICES 
        self.ft_service = roslibpy.Service(self.ros, '/robotiq_ft_sensor_acc', '/robotiq_ft_sensor/sensor_accesor')

        # GLOBAL VARIABLES
        self.joint_states = np.zeros(6)
        self.T_current = np.eye(4)
        self.current_ros_time = {'secs': 0, 'nsecs': 0}
        self.current_force = np.zeros(3)
        self.current_torque = np.zeros(3)
        self.tcp_mode = tcp_mode

        # SETTING SENSOR TO ZERO
        self.zero_ft_sensor()

        # SETTING CALLBACKS
        self.joint_states_tp.subscribe(self.joint_states_callback)
        self.clock_tp.subscribe(self.clock_callback)
        self.force_tp.subscribe(self.force_callback)

        
    def joint_states_callback(self, msg):

        # Extracting the joint states from the message
        shoulder_pan_joint = float(msg['position'][-4])
        shoulder_lift_joint = float(msg['position'][-5])
        elbow_joint  = float(msg['position'][-6])
        wrist_1_joint  = float(msg['position'][-3])
        wrist_2_joint  = float(msg['position'][-2])
        wrist_3_joint  = float(msg['position'][-1])

        self.joint_states[:] = [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
        
        fk_func = ur10_fkine_tcp if self.tcp_mode else ur10_fkine
        self.T_current = fk_func(self.joint_states) 

    def force_callback(self, msg):

        self.current_force = np.array([msg['Fx'], msg['Fy'], msg['Fz']])
        self.current_torque = np.array([msg['Mx'], msg['My'], msg['Mz']])

    def clock_callback(self, msg):
        
        self.current_ros_time['secs'] = msg['clock']['secs']
        self.current_ros_time['nsecs'] = msg['clock']['nsecs']
    
    def send_trajectory(self, target_pose, speed=0.07, art=False):

        if not art and not is_within_workspace(target_pose):
            if not is_moving_towards_workspace(self.T_current, target_pose):
                print("Out of workspace limits, not moving.")
                return
            #else:
                #print("Out of workspace limits, but entering safe area.")

        try:

            ik_func = ur10_ikine_tcp if self.tcp_mode else ur10_ikine
            ik_values = ik_func(target_pose, self.joint_states) if not art else target_pose
            
            distance = max([abs(ik_values[0]-self.joint_states[0]), abs(ik_values[1] - self.joint_states[1]), abs(ik_values[2]-self.joint_states[2]), abs(ik_values[3] - self.joint_states[3]), abs(ik_values[4] - self.joint_states[4]), abs(ik_values[5] - self.joint_states[5])])
            duration = distance / speed
            
            secs_dur = int(duration)
            nsecs_dur = int((duration - secs_dur) * 1e9)

            # Sending the trajectory
            msg = {
                'header': {
                    'stamp': {
                        'secs': self.current_ros_time['secs'],
                        'nsecs': self.current_ros_time['nsecs']
                    },
                    'frame_id': '/campero_ur10_base_link'
                },
                'joint_names': [
                    'campero_ur10_shoulder_pan_joint',
                    'campero_ur10_shoulder_lift_joint',
                    'campero_ur10_elbow_joint',
                    'campero_ur10_wrist_1_joint',
                    'campero_ur10_wrist_2_joint',
                    'campero_ur10_wrist_3_joint'
                ],
                'points': [{
                    'positions':  [float(p) for p in ik_values],
                    'time_from_start': {
                        'secs': secs_dur, 
                        'nsecs': nsecs_dur
                    }
                }]
            }
            
            self.pub_ik_traj_tp.publish(roslibpy.Message(msg))

        except Exception:
            print("Error computing trajectory")


    def send_gripper_cmd(self, cmd="Open"):

        pos = -0.99
        if (cmd == "Close"): pos = 0.99
        
        duration = 0.4
        secs_dur = int(duration)
        nsecs_dur = int((duration - secs_dur) * 1e9)
        
        # Sending gripper command 
        msg = {
            'header': {
                'stamp': {
                    'secs': self.current_ros_time['secs'],
                    'nsecs': self.current_ros_time['nsecs']
                },
                'frame_id': '/campero_ur10_ee_link'
            },
            'joint_names': [ 'campero_robotiq_85_left_knuckle_joint'],
            'points': [{
                'positions':  [pos],
                'time_from_start': {
                    'secs': secs_dur, 
                    'nsecs': nsecs_dur
                }
            }]
        }

        self.pub_gripper_tp.publish(roslibpy.Message(msg))


    def send_trajectory_smooth(self, target_pose, dt):

        # if not is_within_workspace(target_pose):
        #     if not is_moving_towards_workspace(self.T_current, target_pose):
        #         print("Out of workspace limits, not moving.")
        #         return
        #     #else:
        #         #print("Out of workspace limits, but entering safe area.")

        try:
            
            ik_func = ur10_ikine_tcp if self.tcp_mode else ur10_ikine
            ik_values = ik_func(target_pose, self.joint_states)
        
            if (dt < 0.1):
                alpha = 0.8
                ik_values = np.array(ik_values)
                ik_values = alpha * ik_values + (1 - alpha) * self.joint_states
                ik_values = ik_values.tolist()
                dt = 0.1
     
            duration = dt
            secs_dur = int(duration)
            nsecs_dur = int((duration - secs_dur) * 1e9)

            # Sending the trajectory
            msg = {
                'header': {
                    'stamp': {
                        'secs': self.current_ros_time['secs'],
                        'nsecs': self.current_ros_time['nsecs']
                    },
                    'frame_id': '/campero_ur10_base_link'
                },
                'joint_names': [
                    'campero_ur10_shoulder_pan_joint',
                    'campero_ur10_shoulder_lift_joint',
                    'campero_ur10_elbow_joint',
                    'campero_ur10_wrist_1_joint',
                    'campero_ur10_wrist_2_joint',
                    'campero_ur10_wrist_3_joint'
                ],
                'points': [{
                    'positions':  [float(p) for p in ik_values],
                    'time_from_start': {
                        'secs': secs_dur, 
                        'nsecs': nsecs_dur
                    }
                }]
            }
            
            self.pub_ik_traj_tp.publish(roslibpy.Message(msg))

        except Exception:
            print("Error computing trajectory")


    def zero_ft_sensor(self):
        request = roslibpy.ServiceRequest({'command_id': 8})

        def callback(response):
            print("Sensor set to zero")

        self.ft_service.call(request, callback=callback)


    def close_connection(self):
        self.joint_states_tp.unsubscribe()
        self.clock_tp.unsubscribe()
        self.force_tp.unsubscribe()
        self.pub_ik_traj_tp.unadvertise()
        self.pub_gripper_tp.unadvertise()
        self.ros.terminate()
        self.ros.close()
   

# ----------------- TORCH CALLBACKS -----------------
# def update_joint_states_torch(msg, joint_states, q_current):

#     # Extracting the joint states from the message
#     shoulder_pan_joint = float(msg['position'][-4])
#     shoulder_lift_joint = float(msg['position'][-5])
#     elbow_joint  = float(msg['position'][-6])
#     wrist_1_joint  = float(msg['position'][-3])
#     wrist_2_joint  = float(msg['position'][-2])
#     wrist_3_joint  = float(msg['position'][-1])

#     joint_states = [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]

#     q_current_cpu = torch.tensor(
#         r2q(ur10_fkine(joint_states)[:3, :3]),
#         dtype=torch.float32,
#         device="cpu"
#     )
#     q_current.copy_(q_current_cpu, non_blocking=True)

# def force_control_callback(msg, current_force, current_torque, force_history, torque_history):
#     # Updating force and torque history
#     force_history = torch.roll(force_history, shifts=1, dims=0)
#     torque_history = torch.roll(torque_history, shifts=1, dims=0)

#     # Inserting the new measurement
#     force_history[0, :] = torch.tensor([msg['Fx'], msg['Fy'], msg['Fz']], device=device, dtype=torch.float32)
#     torque_history[0, :] = torch.tensor([msg['Mx'], msg['My'], msg['Mz']], device=device, dtype=torch.float32)

#     # Computing the current force and torque as the mean of the history
#     current_force = force_history.mean(dim=0)
#     current_torque = torque_history.mean(dim=0)


