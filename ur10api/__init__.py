"""ur10api: a small Python interface to the Campero UR10 for your own control code.

    from ur10api import Robot, Rate

    with Robot("CMP00-180723AD.local") as robot:       # connect (default host and port)
        state = robot.state()                          # joints, TCP pose, force/torque, gripper
        robot.move_tcp_by(dp=(0, 0, 0.05))             # blocking position command
        rate = Rate(25)
        for _ in range(50):                            # 2 s of velocity control
            robot.set_tcp_velocity(v=(0.02, 0, 0))
            rate.sleep()
        robot.stop()

Modules: robot (Robot, RobotState, GripperState, Rate, errors), kinematics
(ArmModel, Workspace), transforms (pose helpers) and viz (FrameView, live 3D
plots). See docs/ur10api.md for the full guide and examples/ for demos.
"""

from ur10api import kinematics, transforms
from ur10api.kinematics import ArmModel, IKError, Workspace
from ur10api.robot import (
    DEFAULT_HOST,
    DEFAULT_PORT,
    GripperState,
    MotionRefused,
    NotReady,
    Rate,
    Robot,
    RobotError,
    RobotState,
)

__all__ = [
    "ArmModel", "DEFAULT_HOST", "DEFAULT_PORT", "GripperState", "IKError", "MotionRefused", "NotReady",
    "Rate", "Robot", "RobotError", "RobotState", "Workspace", "kinematics", "transforms",
]
