"""Settings of the web interface, read from ur10_config.yaml at start-up.

Edit ur10_config.yaml (its comments explain every value), not this module, and
restart the panel. Here the values are only converted to the names and units the
panel code uses. The messages themselves are always produced by
``core.ur_control.UR10Control``.
"""

import math

from ur10api.config import load_config

SETTINGS = load_config()   # ur10_config.yaml, or the file in $UR10_CONFIG
_robot, _safety, _panel = SETTINGS["robot"], SETTINGS["safety"], SETTINGS["panel"]

# --- Connection to the robot -----------------------------------------------------
ROBOT_HOST = _robot["host"]                 # Campero PC running rosbridge_websocket
ROBOT_PORT = _robot["port"]
PROBE_TIMEOUT_S = _robot["probe_timeout"]   # TCP reachability check before connecting

# --- Local web server ------------------------------------------------------------------
HTTP_HOST = _panel["listen"]   # 127.0.0.1: only this PC can open the panel
HTTP_PORT = _panel["http_port"]

# --- Safety interlocks -------------------------------------------------------------------
STALE_AFTER_S = _safety["stale_after"]   # /joint_states older than this blocks every motion command
WATCHDOG_S = _panel["watchdog"]          # a jog stops if the browser stops refreshing it for this long
STOP_BRAKE_S = _safety["stop_brake"]     # duration of the STOP (hold position) trajectory

# --- Continuous (velocity) jog -------------------------------------------------------------
# While a jog is held, a short position target is streamed at JOG_RATE_HZ. Each
# target lies JOG_LOOKAHEAD_S ahead of the measured state and is sent with that
# same time_from_start, so the robot moves at the jog speed and stops within
# about JOG_LOOKAHEAD_S after release.
JOG_RATE_HZ = _panel["jog_rate"]
JOG_LOOKAHEAD_S = _safety["lookahead"]
JOG_RAMP_S = _panel["jog_ramp"]                                    # speed ramp-up after pressing
JOG_MAX_JOINT_SPEED = math.radians(_panel["jog_max_joint_speed"])  # rad/s cap for Cartesian jogs
JOG_MAX_JOINT_STEP = math.radians(_safety["max_joint_step"])       # rad; a larger IK jump aborts the jog

# --- Speed sliders shown in the panel: (default, minimum, maximum) -------------------------
def _slider(name):
    s = _panel[name]
    return (s["default"], s["min"], s["max"])


LINEAR_SPEED_MM_S = _slider("linear_speed")
ANGULAR_SPEED_DEG_S = _slider("angular_speed")
JOINT_JOG_SPEED_DEG_S = _slider("joint_jog_speed")
MOVE_SPEED_DEG_S = _slider("move_speed")   # max joint speed of "Move to" commands

# --- Step (incremental) jog sizes: (choices, default) -------------------------------------
LINEAR_STEPS_MM = (tuple(_panel["linear_steps"]["choices"]), _panel["linear_steps"]["default"])
ANGULAR_STEPS_DEG = (tuple(_panel["angular_steps"]["choices"]), _panel["angular_steps"]["default"])
JOINT_STEPS_DEG = (tuple(_panel["joint_steps"]["choices"]), _panel["joint_steps"]["default"])

# --- Moves ----------------------------------------------------------------------------------
CONFIRM_ABOVE_DEG = _panel["confirm_above"]   # "Move to" asks for confirmation above this

# --- Tool centre point and workspace ------------------------------------------------------
TCP = SETTINGS.tcp_offset()              # (x, y, z [m], rx, ry, rz [rad]) relative to the flange
WORKSPACE = SETTINGS.workspace_limits()  # {axis: (min, max)} [m], UR base frame; editable in the page

# --- Gripper feedback (read-only, optional) --------------------------------------------------
GRIPPER_FEEDBACK_TOPICS = tuple(SETTINGS["gripper"]["feedback_topics"])
GRIPPER_FEEDBACK_TYPE = "robotiq_2f_gripper_control/Robotiq2FGripper_robot_input"

# --- Joint presets (degrees, J1..J6) offered in "Move to joints" ----------------------------
# Poses saved from the panel are stored in saved_poses.json at the repository root.
PRESET_POSES_DEG = dict(_panel["presets"])
