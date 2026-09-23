"""Settings shared by the web panel, ur10api and the examples: ur10_config.yaml.

    from ur10api.config import load_config

    cfg = load_config()                            # ur10_config.yaml (or the file in $UR10_CONFIG)
    cfg["examples"]["force_teleop"]["dead_band"]   # 3.0: values in the file's units (mm, deg, s, N)
    cfg.workspace_limits()                         # {"x": (-0.8, 0.5), ...} in metres, UR base frame
    cfg.tcp_offset()                               # (x, y, z [m], rx, ry, rz [rad])

The file uses the units of the teach pendant and the panel: mm, deg, s, N, Nm.
Missing keys take the built-in defaults of DEFAULTS below; unknown keys and
invalid values raise ConfigError, so a typo never silently disables a limit.
Robot() reads the file by itself; see the comments in ur10_config.yaml for what
each setting does.
"""

import copy
import math
import os
import re
from pathlib import Path

import yaml

from ur10api.kinematics import DEFAULT_WORKSPACE, validate_limits

ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "ur10_config.yaml"   # default settings file
ENV_VAR = "UR10_CONFIG"                   # path of another settings file, if set

DEFAULT_HOST = "CMP00-180723AD.local"     # Campero PC running rosbridge_websocket
DEFAULT_PORT = 9090


class ConfigError(ValueError):
    """The settings file cannot be read or contains an invalid value."""


def _mm(metres):
    return round(metres * 1000.0, 1)


# Built-in values, in the file's units. ur10_config.yaml mirrors this structure.
DEFAULTS = {
    "robot": {"host": DEFAULT_HOST, "port": DEFAULT_PORT, "probe_timeout": 3.0},
    "tcp": {"position": [0.0, 0.0, 150.0], "rotation": [0.0, 0.0, 0.0]},   # UR_CONTROL's TCP
    "workspace": {axis: [_mm(lo), _mm(hi)] for axis, (lo, hi) in DEFAULT_WORKSPACE.items()},
    "safety": {"stale_after": 0.5, "stop_brake": 0.4, "lookahead": 0.25, "max_joint_step": 20.0},
    "api": {"max_linear_speed": 100.0, "max_angular_speed": 30.0, "max_joint_speed": 45.0,
            "max_linear_accel": 500.0, "max_angular_accel": 115.0},
    "gripper": {"feedback_topics": ["/robotiq_2f_gripper/input", "/Robotiq2FGripperRobotInput"]},
    "link_check": {"enabled": True, "step": 1.0, "move_time": 0.4, "max_delay": 0.3, "good_moves": 3,
                   "timeout": 20.0},
    "plots": {"window": 20.0, "fps": 8.0, "save_dir": "runs"},
    "panel": {
        "listen": "127.0.0.1", "http_port": 8080, "watchdog": 0.4,
        "jog_rate": 20.0, "jog_ramp": 0.3, "jog_max_joint_speed": 35.0, "confirm_above": 45.0,
        "linear_speed": {"default": 25.0, "min": 1.0, "max": 100.0},
        "angular_speed": {"default": 5.0, "min": 1.0, "max": 30.0},
        "joint_jog_speed": {"default": 5.0, "min": 1.0, "max": 30.0},
        "move_speed": {"default": 10.0, "min": 1.0, "max": 60.0},
        "linear_steps": {"choices": [1.0, 5.0, 10.0, 25.0, 50.0], "default": 10.0},
        "angular_steps": {"choices": [0.5, 1.0, 5.0, 10.0], "default": 5.0},
        "joint_steps": {"choices": [0.5, 1.0, 5.0, 10.0], "default": 5.0},
        "presets": {"Force-control start (report 5.3)": [0.27, -90.42, 95.21, -125.32, -90.15, 0.48]},
    },
    "examples": {
        "pose_frames": {
            "frame": "tool", "mode": "servo", "gain": 1.5, "max_speed": 40.0, "max_angular_speed": 17.0,
            "tolerance": 1.0, "angle_tolerance": 0.3, "rate": 25.0, "timeout": 30.0, "joint_speed": 11.5,
            "targets": {
                "T1": {"position": [60.0, 0.0, 0.0], "rotation": [0.0, 0.0, 20.0]},
                "T2": {"position": [0.0, 60.0, 30.0], "rotation": [15.0, 0.0, 0.0]},
                "T3": {"position": [-40.0, -40.0, 20.0], "rotation": [0.0, -15.0, 0.0]},
            },
        },
        "velocity_path": {
            "shape": "circle", "size": 50.0, "period": 8.0, "laps": 2, "plane": "tool",
            "kp": 2.0, "kr": 2.0, "rate": 25.0, "max_speed": 120.0, "max_angular_speed": 17.0,
        },
        "force_teleop": {
            "gain": 4.0, "dead_band": 3.0, "max_speed": 50.0, "filter": 0.2, "rate": 50.0,
            "rotate": False, "torque_gain": 8.6, "torque_dead_band": 0.8, "max_rotation": 30.0,
            "max_angular_speed": 14.0, "invert": False,
        },
    },
}

# Sections whose keys are names chosen by the user (replaced as a whole, not merged)
FREE_KEYS = {"panel.presets", "examples.pose_frames.targets"}
# Text settings with a fixed set of values
CHOICES = {
    "examples.pose_frames.frame": ("tool", "base"),
    "examples.pose_frames.mode": ("servo", "move"),
    "examples.velocity_path.shape": ("circle", "infinity"),
    "examples.velocity_path.plane": ("tool", "base"),
}
# Text settings that may be empty
MAY_BE_EMPTY = {"plots.save_dir"}
# Numbers that may be negative or zero; every other number must be > 0
ANY_SIGN = {"tcp.position", "tcp.rotation", "workspace.x", "workspace.y", "workspace.z"}
NOT_NEGATIVE = {"panel.jog_ramp", "examples.velocity_path.kp", "examples.velocity_path.kr",
                "examples.force_teleop.dead_band", "examples.force_teleop.torque_dead_band"}


class Config(dict):
    """The settings as nested dicts, in the file's units (mm, deg, s, N).

    cfg["api"]["max_linear_speed"] -> 100.0 (mm/s). The methods convert what the
    code needs to SI units. `path` is the file that was read (None: built-in values).
    """

    def __init__(self, data, path=None):
        super().__init__(data)
        self.path = path

    def tcp_offset(self):
        """TCP relative to the flange: (x, y, z [m], rx, ry, rz [rad, rotation vector])."""
        tcp = self["tcp"]
        return tuple(v / 1000.0 for v in tcp["position"]) + tuple(math.radians(v) for v in tcp["rotation"])

    def workspace_limits(self):
        """Workspace box {axis: (min, max)} in metres, UR base frame."""
        return {axis: (lo / 1000.0, hi / 1000.0) for axis, (lo, hi) in self["workspace"].items()}

    @property
    def name(self):
        return self.path.name if self.path else "built-in defaults"


def config_path(path=None):
    """The settings file to use: `path`, else $UR10_CONFIG, else ur10_config.yaml."""
    if path is not None:
        return Path(path)
    return Path(os.environ[ENV_VAR]) if os.environ.get(ENV_VAR) else CONFIG_PATH


def load_config(path=None):
    """Read and check the settings file. Raises ConfigError with the offending key.

    Without a file at the default location the built-in values are used; a file
    named explicitly (argument or $UR10_CONFIG) must exist.
    """
    explicit = path is not None or bool(os.environ.get(ENV_VAR))
    path = config_path(path)
    try:
        text = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        if explicit:
            raise ConfigError(f"settings file not found: {path}") from None
        return Config(copy.deepcopy(DEFAULTS))
    except OSError as exc:
        raise ConfigError(f"cannot read {path}: {exc}") from exc
    return Config(_parse(text, path.name), path)


def _parse(text, name):
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ConfigError(f"{name} is not valid YAML: {exc}") from exc
    try:
        settings = _merge(DEFAULTS, {} if data is None else data, "")
        _check(settings)
    except ConfigError as exc:
        raise ConfigError(f"{name}: {exc}") from None
    return settings


# --- validation ----------------------------------------------------------------------------

def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _merge(defaults, data, where):
    """Defaults updated with `data`, checking every key and value type."""
    if not isinstance(data, dict):
        raise ConfigError(f"{where or 'the file'}: expected a section of 'key: value' lines")
    merged = copy.deepcopy(defaults)
    for key, value in data.items():
        path = f"{where}.{key}" if where else str(key)
        if key not in defaults:
            raise ConfigError(f"{path}: unknown setting (valid here: {', '.join(defaults)})")
        merged[key] = _value(defaults[key], value, path)
    return merged


def _value(default, value, path):
    if path in FREE_KEYS:
        if not isinstance(value, dict):
            raise ConfigError(f"{path}: expected 'name: value' lines")
        return value   # checked in _check
    if isinstance(default, dict):
        return _merge(default, value, path)
    if isinstance(default, bool):
        if not isinstance(value, bool):
            raise ConfigError(f"{path}: expected true or false, got {value!r}")
        return value
    if isinstance(default, int):
        if not isinstance(value, int) or isinstance(value, bool):
            raise ConfigError(f"{path}: expected a whole number, got {value!r}")
        return value
    if isinstance(default, float):
        if not _is_number(value):
            raise ConfigError(f"{path}: expected a number, got {value!r}")
        return float(value)
    if isinstance(default, str):
        if not isinstance(value, str) or (not value.strip() and path not in MAY_BE_EMPTY):
            raise ConfigError(f"{path}: expected a text value, got {value!r}")
        if path in CHOICES and value not in CHOICES[path]:
            raise ConfigError(f"{path}: must be one of {', '.join(CHOICES[path])}, got {value!r}")
        return value
    if isinstance(default, list):
        if not isinstance(value, list):
            raise ConfigError(f"{path}: expected a list [a, b, ...], got {value!r}")
        if all(isinstance(v, str) for v in default):
            if not all(isinstance(v, str) for v in value):
                raise ConfigError(f"{path}: expected a list of names, got {value!r}")
            return value
        return _numbers(value, path, None if path.endswith(".choices") else len(default))
    raise AssertionError(path)


def _numbers(value, path, length=None):
    if not isinstance(value, list) or not all(_is_number(v) for v in value):
        raise ConfigError(f"{path}: expected a list of numbers, got {value!r}")
    if length is not None and len(value) != length:
        raise ConfigError(f"{path}: expected {length} numbers, got {len(value)}")
    return [float(v) for v in value]


def _require(condition, path, message):
    if not condition:
        raise ConfigError(f"{path}: {message}")


def _check_signs(section, where):
    for key, value in section.items():
        path = f"{where}.{key}" if where else key
        if path in FREE_KEYS or path in ANY_SIGN:
            continue
        if isinstance(value, dict):
            _check_signs(value, path)
        elif _is_number(value):
            if path in NOT_NEGATIVE:
                _require(value >= 0, path, f"must be zero or positive, got {value:g}")
            else:
                _require(value > 0, path, f"must be positive, got {value:g}")
        elif isinstance(value, list) and value and all(_is_number(v) for v in value):
            _require(all(v > 0 for v in value), path, f"all values must be positive, got {value}")


def _check(cfg):
    """Ranges and relations that the types alone do not cover."""
    _check_signs(cfg, "")
    _require(cfg["robot"]["port"] <= 65535, "robot.port", "must be 1-65535")
    _require(cfg["panel"]["http_port"] <= 65535, "panel.http_port", "must be 1-65535")
    try:
        validate_limits({axis: (lo / 1000.0, hi / 1000.0) for axis, (lo, hi) in cfg["workspace"].items()})
    except ValueError as exc:
        raise ConfigError(f"workspace: {exc}") from None
    panel = cfg["panel"]
    for key in ("linear_speed", "angular_speed", "joint_jog_speed", "move_speed"):
        s = panel[key]
        _require(s["min"] <= s["default"] <= s["max"], f"panel.{key}", "needs min <= default <= max")
    for key in ("linear_steps", "angular_steps", "joint_steps"):
        s = panel[key]
        _require(len(s["choices"]) > 0, f"panel.{key}.choices", "needs at least one value")
        _require(s["default"] in s["choices"], f"panel.{key}.default", "must be one of the choices")
    for name, q in panel["presets"].items():
        q = _numbers(q, f"panel.presets.{name}", 6)
        _require(all(abs(v) <= 360 for v in q), f"panel.presets.{name}", "joint angles must be within ±360 deg")
        panel["presets"][name] = q
    targets = cfg["examples"]["pose_frames"]["targets"]
    _require(len(targets) > 0, "examples.pose_frames.targets", "needs at least one target")
    for name, target in targets.items():
        path = f"examples.pose_frames.targets.{name}"
        _require(isinstance(target, dict) and set(target) == {"position", "rotation"}, path,
                 "expected {position: [x, y, z], rotation: [rx, ry, rz]}")
        targets[name] = {key: _numbers(target[key], f"{path}.{key}", 3) for key in ("position", "rotation")}
    _require(cfg["examples"]["force_teleop"]["filter"] <= 1, "examples.force_teleop.filter", "must be between 0 and 1")


# --- writing the workspace back (web panel) --------------------------------------------------

_TOP_LEVEL_KEY = re.compile(r"[A-Za-z_][\w-]*\s*:")                 # a line starting a section
_WORKSPACE_KEY = re.compile(r"workspace\s*:\s*(#.*)?$")
_AXIS_LINE = re.compile(r"(\s+)([xyz])(\s*:\s*)\[[^\]]*\](.*)$")    # "  x: [-800, 500]   # comment"


def _format_mm(metres):
    value = _mm(metres) + 0.0   # + 0.0 turns -0.0 into 0.0
    return f"{value:.0f}" if value.is_integer() else f"{value:g}"


def save_workspace(limits, path=None):
    """Write the workspace box {axis: (min, max)} [m] into the settings file.

    Only the numbers on the x/y/z lines of the `workspace:` section change;
    comments and every other setting are kept. The new file is checked before
    it replaces the old one. Returns the path written.
    """
    limits = validate_limits(limits)
    path = config_path(path)
    try:
        with open(path, encoding="utf-8", newline="") as f:
            text = f.read()
    except FileNotFoundError:
        text = ""
    newline = "\r\n" if "\r\n" in text else "\n"
    values = {axis: "[" + ", ".join(_format_mm(v) for v in limits[axis]) + "]" for axis in "xyz"}
    lines = text.splitlines(keepends=True)
    start = next((i for i, line in enumerate(lines) if _WORKSPACE_KEY.match(line.rstrip("\r\n"))), None)
    replaced = set()
    if start is not None:
        end = next((i for i in range(start + 1, len(lines)) if _TOP_LEVEL_KEY.match(lines[i])), len(lines))
        for i in range(start + 1, end):
            body = lines[i].rstrip("\r\n")
            m = _AXIS_LINE.match(body)
            if m and m.group(2) not in replaced:
                indent, axis, colon, comment = m.groups()
                lines[i] = f"{indent}{axis}{colon}{values[axis]}{comment}" + lines[i][len(body):]
                replaced.add(axis)
    if replaced != set("xyz"):   # no section, or written in another style: write a fresh one
        block = [f"workspace:{newline}"] + [f"  {axis}: {values[axis]}{newline}" for axis in "xyz"]
        if start is None:
            if lines and not lines[-1].endswith(("\n", "\r")):
                lines[-1] += newline
            lines += ([newline] if lines else []) + block
        else:
            while end > start + 1 and (not lines[end - 1].strip() or lines[end - 1].lstrip().startswith("#")):
                end -= 1   # keep comments and blank lines that precede the next section
            lines[start:end] = block
    new_text = "".join(lines)

    check = Config(_parse(new_text, path.name)).workspace_limits()   # never write a broken file
    if any(abs(a - b) > 1e-4 for axis in "xyz" for a, b in zip(check[axis], limits[axis])):
        raise ConfigError(f"could not update the workspace section of {path.name}; edit it by hand")
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w", encoding="utf-8", newline="") as f:
        f.write(new_text)
    os.replace(tmp, path)
    return path
