import leap
import time
import threading
from dataclasses import dataclass
from typing import Optional, Tuple
from collections import deque
import math

@dataclass
class HandData:
    gesture: str = "UNKNOWN"
    position: Optional[Tuple[float, float, float]] = None  
    ready: bool = False
    count: int = 0


class GestureRecognizer:
    def __init__(self):
        # In mm
        self.PINCH_THRESHOLD = 30  
        self.FIST_THRESHOLD = 50   
        self.OPEN_THRESHOLD = 70  

    def distance_between(self, joint1, joint2):
        if joint1 and joint2:
            return ((joint1.x - joint2.x)**2 + 
                    (joint1.y - joint2.y)**2 + 
                    (joint1.z - joint2.z)**2)**0.5
        return float('inf')

    #[pulgar, indice, corazon, anular, meñique]

    def is_okay(self, hand):
        thumb_tip = hand.digits[0].distal.next_joint
        index_tip = hand.digits[1].distal.next_joint
        palm = hand.palm.position
        
        # Check if thumb and index finger are close enough
        if self.distance_between(thumb_tip, index_tip) > self.PINCH_THRESHOLD:
            return False
            
        # Rest of the fingers should be folded
        for digit in [2, 3, 4]: 
            tip = hand.digits[digit].distal.next_joint
            if self.distance_between(palm, tip) < self.FIST_THRESHOLD:
                return False
        return True

    def is_fist(self, hand):
        palm = hand.palm.position
        # All fingers should be close to the palm
        return all(
            self.distance_between(palm, hand.digits[i].distal.next_joint) < self.FIST_THRESHOLD
            for i in range(5))
    
    def is_open_hand(self, hand):
        palm = hand.palm.position
        # All fingers should be far from the palm -> Dependent of the depth
        return all(
            self.distance_between(palm, hand.digits[i].distal.next_joint) > self.OPEN_THRESHOLD
            for i in range(5))
    
    def is_peace(self, hand):
        palm = hand.palm.position
        extended = [1, 2]  
        folded = [0, 3, 4]  
        
        for digit in extended:
            tip = hand.digits[digit].distal.next_joint
            if self.distance_between(palm, tip) < self.OPEN_THRESHOLD:
                return False
                
        for digit in folded:
            tip = hand.digits[digit].distal.next_joint
            if self.distance_between(palm, tip) > self.FIST_THRESHOLD:
                return False
        return True
    
    def is_thumbs_up(self, hand):
        palm = hand.palm.position
        thumb_tip = hand.digits[0].distal.next_joint
        if self.distance_between(palm, thumb_tip) < self.OPEN_THRESHOLD:
            return False
            
        for digit in [1, 2, 3, 4]: 
            tip = hand.digits[digit].distal.next_joint
            if self.distance_between(palm, tip) > self.FIST_THRESHOLD:
                return False
        return True
        
    def is_pointing(self, hand):
        palm = hand.palm.position
        index_tip = hand.digits[1].distal.next_joint
        if self.distance_between(palm, index_tip) < self.OPEN_THRESHOLD:
            return False
            
        for digit in [0, 2, 3, 4]: 
            tip = hand.digits[digit].distal.next_joint
            if self.distance_between(palm, tip) > self.FIST_THRESHOLD:
                return False
        return True
    
    def recognize_gesture(self, hand):
        if not hand or not hand.digits or len(hand.digits) < 5:
            return "NO_HAND"
            
        if self.is_peace(hand): return "PEACE"
        elif self.is_thumbs_up(hand): return "THUMBS_UP"
        elif self.is_pointing(hand): return "POINTING"
        elif self.is_okay(hand): return "OKAY"
        elif self.is_fist(hand): return "FIST"
        elif self.is_open_hand(hand): return "OPEN_HAND"
        else: return "UNKNOWN"


class GesturePrinter(leap.Listener):
    def __init__(self, recognizer):
        super().__init__()
        self.recognizer = recognizer
        self.last_print_time = 0

    def on_tracking_event(self, event):
        current_time = time.time()
        if current_time - self.last_print_time < 0.3: 
            return
        
        self.last_print_time = current_time
        
        if not event.hands:
            print("No hands detected")
            return
        
        for hand in event.hands:
            hand_type = "LEFT" if str(hand.type) == "HandType.Left" else "RIGHT"
            gesture = self.recognizer.recognize_gesture(hand)
            print(f"Hand {hand_type}: {gesture}")


class GestureListener(leap.Listener):
    def __init__(self, recognizer):
        super().__init__()
        self.recognizer = recognizer
        self.left_hand = HandData()
        self.right_hand = HandData()
        self.last_update_time = 0
        self.ready_threshold = 8
        self.left_window = deque(maxlen=10)
        self.right_window = deque(maxlen=10)

    def on_tracking_event(self, event):

        current_time = time.time()
        if current_time - self.last_update_time < 0.01:
            return
        
        self.last_update_time = current_time

        if not event.hands:
            self.left_hand = HandData()
            self.right_hand = HandData()
            return

        
        for hand in event.hands:

            hand_data = self.left_hand if str(hand.type) == "HandType.Left" else self.right_hand
            
            new_gesture = self.recognizer.recognize_gesture(hand)
            new_position = (hand.palm.position.x, 
                          hand.palm.position.y, 
                          hand.palm.position.z)
            
            if str(hand.type) == "HandType.Left":
                self.left_window.append(new_position)
                hand_data.position = tuple(sum(axis) / len(self.left_window) for axis in zip(*self.left_window))
            else:
                self.right_window.append(new_position)
                hand_data.position = tuple(sum(axis) / len(self.right_window) for axis in zip(*self.right_window))
             
            if new_gesture == hand_data.gesture:
                hand_data.count += 1
                hand_data.ready = hand_data.count > self.ready_threshold
            else:
                hand_data.gesture = new_gesture
                hand_data.count = 0
                hand_data.ready = False
            
class GestureTracker:
    def __init__(self, recognizer):
        self.listener = GestureListener(recognizer)
        self.connection = leap.Connection()
        self.connection.add_listener(self.listener)
        self.thread = None
        self.running = False
        self.last_time = time.time()
        self.last_hand_position = (0.0, 0.0, 0.0)

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(
                target=self._run,
                daemon=True
            )
            self.thread.start()

    def stop(self):
        if self.running:
            self.running = False
            self.thread.join()

    def _run(self):
        with self.connection.open():
            while self.running:
                time.sleep(0.05)

    def get_hand_data(self, hand_type: str) -> HandData:
        if hand_type.lower() == 'left':
            return self.listener.left_hand
        return self.listener.right_hand

    def get_all_data(self):
        return {
            'left': self.listener.left_hand,
            'right': self.listener.right_hand
        }
         
    def is_hand_still(self, current_position, threshold=20):
    
        current_time = time.time()
        dt = current_time - self.last_time
        if dt == 0:
            return True 

        velocity = tuple((c - l) / dt for c, l in zip(current_position, self.last_hand_position))
        speed = math.sqrt(sum(v**2 for v in velocity))
     
        self.last_hand_position = current_position
        self.last_time = current_time

        return speed < threshold

def main():

    print("Gestures availiable:")
    print(" PEACE | 👍 THUMBS_UP | 👆 POINTING")
    print("✊ FIST | 🖐️ OPEN_HAND | 👌 OKAY")
    
    recognizer = GestureRecognizer() 
    tracker = GestureTracker(recognizer)
    tracker.start()        
  
    try:
        while True:

            left_data = tracker.get_hand_data('left')
            right_data = tracker.get_hand_data('right')

            print("Left Hand Data:", left_data)
            print("Right Hand Data:", right_data)
            print("")
            
            time.sleep(0.05)
                    
    except KeyboardInterrupt:
        tracker.stop()

if __name__ == "__main__":
    main()