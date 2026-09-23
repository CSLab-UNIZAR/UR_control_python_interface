import numpy as np
import copy

class Position:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

class Orientation:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def toList(self):
        return np.array([self.w, self.x, self.y, self.z])

class Pose:
    def __init__(self):
        self.position = Position()
        self.orientation = Orientation()

    def copy(self):
        return copy.deepcopy(self)
    
    def __str__(self):
        return (f"Pose:\n"
                f"  Position -> x: {self.position.x:.3f}, y: {self.position.y:.3f}, z: {self.position.z:.3f}\n"
                f"  Orientation -> x: {self.orientation.x:.3f}, "
                f"y: {self.orientation.y:.3f}, z: {self.orientation.z:.3f}, w: {self.orientation.w:.3f}")
