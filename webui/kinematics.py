"""Kinematics of the web panel: ur10api's calibrated model with the panel's TCP and workspace.

The panel uses one arm model (TCP from webui/config.py) and one workspace box
(editable from the page); the module-level functions below wrap them.
"""

from ur10api.kinematics import (  # noqa: F401  (re-exported for the panel)
    ARM_JOINTS,
    BASE_FRAME,
    CARTESIAN_AXES,
    DEFAULT_WORKSPACE,
    FLANGE_FRAME,
    JOINT_LIMIT,
    JOINT_NAMES,
    ArmModel,
    IKError,
    Workspace,
)
from ur10api.transforms import (  # noqa: F401
    displace,
    pose_to_xyzrpy,
    rotation_angle,
    rotation_vector,
    xyzrpy_to_pose,
)
from webui import config

ARM = ArmModel(tcp=config.TCP)
WORKSPACE = Workspace()


def fk(q, tcp=True):
    """TCP (or flange) pose for joint vector q, calibrated model."""
    return ARM.fk(q, tcp)


def ik(T, seed, tcp=True):
    """Joint solution for a TCP (or flange) pose, closest to `seed`, calibrated model."""
    return ARM.ik(T, seed, tcp)


def workspace_limits():
    return WORKSPACE.limits


def set_workspace_limits(limits):
    WORKSPACE.set(limits)


def workspace_violations(T):
    return WORKSPACE.violations(T)


def workspace_distance(T):
    return WORKSPACE.distance(T)


def workspace_allows(T_current, T_target):
    return WORKSPACE.allows(T_current, T_target)
