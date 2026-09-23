import time
import copy
from core.ur_control import * 
from core.ur10_core import *
from core.leap_gestures_kalman import *

ur10_control = UR10Control(host_ip='CMP00-180723AD.local')
time.sleep(1)

def leap_control(aid = "False"):

    recognizer = GestureRecognizer() 
    tracker = GestureTracker(recognizer)
    tracker.start()         # Tracking executes concurrently to the main thread
    
    # Reference for the movement
    initialized = False    
    last_hand_position = (0, 0, 0)
    reference = (0, 0, 0)
    smoothed_position = (0, 0, 0)
    gripper_state = None
    precision = False

    alpha = 1       
    thresholds = {'x': 0.03, 'y': 0.03, 'z': 0.03}  
    scale_factors = {'x': 0.005, 'y': 0.0075, 'z': -0.005}  

    print("Leap Control started, axis control mode is", "enabled" if aid else "disabled")
    print("Use 'OKAY' gesture to enable tracking and gesture recognition")
    print("Use 'OPEN_HAND' to open the gripper, 'THUMBS_UP' to close it, 'POINTING' to enter gripper-control mode, and 'PEACE' to exit gripper-control mode or disable tracking")
    print("Press Ctrl+C to exit Leap Control\n")
    try:
        # Main thread
        while True:

            left_data = tracker.get_hand_data('left')
          
            if (left_data.ready and left_data.gesture == "OKAY"):  # Enabling tracking and gesture recognition
                print("Enabling Tracking and Gesture recognition")
                time.sleep(1)  # Allow some time for the system to stabilize
                last_hand_position = (0, 0, 0)
                reference = left_data.position
                initialized = True
                precision = False
                continue
            
            if (initialized):                  # The robot can move
                
                if (left_data.position == None):
                    print("No hand detected")
                    continue

                # Get the current hand position
                current_hand_position = left_data.position

                smoothed_position = tuple(
                    alpha * curr + (1 - alpha) * prev
                    for curr, prev in zip(current_hand_position, smoothed_position)
                )

                # Difference from the reference
                diff_ref = tuple(
                    smoothed - ref for smoothed, ref in zip(smoothed_position, reference)
                )

                # Difference from the last hand position
                diff_last = tuple(
                    diff - last for diff, last in zip(diff_ref, last_hand_position)
                )

                # Scale the difference
                diff = tuple(
                    d * scale_factors[axis]
                    for d, axis in zip(diff_last, ['x', 'y', 'z'])
                )

                # Check if the hand is still
                hand_still = tracker.is_hand_still(smoothed_position)
                thresholds = {'x': 0.07, 'y': 0.07, 'z': 0.03}
                if hand_still:
                    thresholds = {'x': 0.17, 'y': 0.17, 'z': 0.1}
    
                # Check if the hand is dead (not moving)
                dead = all(
                    abs(d) < thresholds[axis]
                    for d, axis in zip(diff, ['x', 'y', 'z'])
                )

            
                if (left_data.ready):
                    match (left_data.gesture):
                        case "OPEN_HAND":
                            if gripper_state != "Open":
                                print("Opening Gripper")
                                ur10_control.send_gripper_cmd("Open")
                                gripper_state = "Open"
                        case "THUMBS_UP":
                            if gripper_state != "Close":
                                print("Closing Gripper")
                                ur10_control.send_gripper_cmd("Close")
                                gripper_state = "Close"
                        
                        case "POINTING":
                            print("Entering gripper-control mode")
                            precision = True
                        
                        case "PEACE":
                            if precision:
                                print("Exiting gripper-control mode")
                                time.sleep(1)  
                                precision = False
                                last_hand_position = (0, 0, 0)
                                reference = left_data.position
                            else:
                                print("Disabling Traking and Gesture recognition")
                                initialized = False
                
                if dead or precision:
                    time.sleep(0.05)
                    continue
                print(f"Movement detected: {diff}")
                
                if (aid):
                    abs_diff = [abs(d) for d in diff]
                    max_index = abs_diff.index(max(abs_diff)) 

                    diff = tuple(
                        d if i == max_index else 0.0
                        for i, d in enumerate(diff)
                    )

                # Assuming the reference is the same -> CHECKED
                T_base_tcp = ur10_control.T_current.copy()
                
                T_base_tcp[0, 3] +=  diff[0]
                T_base_tcp[1, 3] +=  diff[2]
                T_base_tcp[2, 3] +=  diff[1]
                ur10_control.send_trajectory(T_base_tcp)
               
                last_hand_position = diff_ref

                time.sleep(0.05)
                    
    except KeyboardInterrupt:
        tracker.stop()
        ur10_control.close_connection()
        print("Exiting Leap Control")

print("Use axis control mode? (only move one axis at a time) (Yes/No)")
aid = input().strip().lower() == 'yes'
leap_control(aid)

