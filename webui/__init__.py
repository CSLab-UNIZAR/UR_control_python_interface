"""Browser-based control panel for UR_CONTROL: jog, move, gripper and live status.

Run from the repository root with ``python -m webui`` and open the printed URL.
Every command is sent through the unchanged ``core.ur_control.UR10Control``
class, so the rosbridge node on the Campero receives exactly the same messages
as with the other controllers.
"""
