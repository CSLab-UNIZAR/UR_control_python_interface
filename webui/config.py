"""Tunable settings of the web interface.

These values only change how the interface builds its commands. The messages
themselves are always produced by ``core.ur_control.UR10Control``.
"""

# --- Connection to the robot -----------------------------------------------------
ROBOT_HOST = "CMP00-180723AD.local"   # Campero PC running rosbridge_websocket
ROBOT_PORT = 9090                     # rosbridge_websocket default port
PROBE_TIMEOUT_S = 3.0                 # TCP reachability check before connecting

# --- Local web server ------------------------------------------------------------------
HTTP_HOST = "127.0.0.1"   # only this PC can open the panel; use --listen 0.0.0.0 to share it
HTTP_PORT = 8080

# --- Safety interlocks -------------------------------------------------------------------
STALE_AFTER_S = 0.5    # /joint_states older than this blocks every motion command
WATCHDOG_S = 0.4       # a jog stops if the browser stops refreshing it for this long
STOP_BRAKE_S = 0.4     # duration of the STOP (hold position) trajectory

# --- Continuous (velocity) jog -------------------------------------------------------------
# While a jog is held, a short position target is streamed at JOG_RATE_HZ. Each
# target lies JOG_LOOKAHEAD_S ahead of the measured state and is sent with that
# same time_from_start, so the robot moves at the jog speed and stops within
# about JOG_LOOKAHEAD_S after release.
JOG_RATE_HZ = 20.0
JOG_LOOKAHEAD_S = 0.25
JOG_RAMP_S = 0.3             # speed ramp-up after pressing
JOG_MAX_JOINT_SPEED = 0.6    # rad/s cap for Cartesian jogs (slows down near singularities)
JOG_MAX_JOINT_STEP = 0.35    # rad; a larger IK jump in one streamed step aborts the jog

# --- Speed sliders shown in the panel: (default, minimum, maximum) -------------------------
LINEAR_SPEED_MM_S = (25, 1, 100)
ANGULAR_SPEED_DEG_S = (5, 1, 30)
JOINT_JOG_SPEED_DEG_S = (5, 1, 30)
MOVE_SPEED_DEG_S = (10, 1, 60)       # max joint speed of "Move to" commands

# --- Step (incremental) jog sizes: (choices, default) -------------------------------------
LINEAR_STEPS_MM = ((1, 5, 10, 25, 50), 10)
ANGULAR_STEPS_DEG = ((0.5, 1, 5, 10), 5)
JOINT_STEPS_DEG = ((0.5, 1, 5, 10), 5)

# --- Moves ----------------------------------------------------------------------------------
CONFIRM_ABOVE_DEG = 45   # "Move to" asks for confirmation above this joint displacement

# --- Tool centre point -------------------------------------------------------------------
# The arm model (robot_model/model.json) and its frames are defined in ur10api/kinematics.py.
# TCP relative to the flange, written like the pendant's Installation > TCP:
# (x, y, z [m], rx, ry, rz [rad, rotation vector]). The default is UR_CONTROL's TCP
# (tcp_offset in core/ur10_core.py). Set the same values on the pendant to compare readings.
TCP = (0.0, 0.0, 0.15, 0.0, 0.0, 0.0)

# --- Gripper feedback (read-only, optional) --------------------------------------------------
# Robotiq2FGripperRtuNode.py publishes Robotiq2FGripperRobotInput; campero_ur10_bringup.launch
# remaps it to /robotiq_2f_gripper/input. Both names are watched; use () to disable.
GRIPPER_FEEDBACK_TOPICS = ("/robotiq_2f_gripper/input", "/Robotiq2FGripperRobotInput")
GRIPPER_FEEDBACK_TYPE = "robotiq_2f_gripper_control/Robotiq2FGripper_robot_input"

# --- Joint presets (degrees, J1..J6) offered in "Move to joints" ----------------------------
# Poses saved from the panel are stored in saved_poses.json at the repository root.
PRESET_POSES_DEG = {
    "Force-control start (report 5.3)": [0.27, -90.42, 95.21, -125.32, -90.15, 0.48],
}
