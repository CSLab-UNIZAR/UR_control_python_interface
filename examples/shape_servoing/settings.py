"""Settings of example 4: examples/04_shape_servoing.yaml.

    settings = load_settings()           # the file next to the example, checked
    settings["control"]["gain"]          # values in the file's units (px, deg, s)

Like ur10_config.yaml: missing keys take the built-in values of DEFAULTS, and
unknown keys or invalid values raise SettingsError (a ur10api ConfigError)
naming the key, so a typo never silently falls back to a default.
"""

import copy
import math
from pathlib import Path

import yaml

from ur10api import ConfigError

SETTINGS_PATH = Path(__file__).resolve().parents[1] / "04_shape_servoing.yaml"
SPACES = ("position", "edges", "curvature")
FRAMES = ("image", "camera")

# Built-in values, in the file's units. 04_shape_servoing.yaml mirrors this structure.
DEFAULTS = {
    "camera": {"source": "realsense", "serial": "", "device": 0, "width": 640, "height": 480, "fps": 30},
    "markers": {"dictionary": "DICT_4X4_50", "ids": [], "refine_corners": True, "size": 40.0},
    "tracking": {"measurement_noise": 1.0, "process_noise": 3000.0, "confirm_frames": 2, "max_missing": 0.5,
                 "velocity_decay": 0.7, "measurement_noise_3d": 3.0, "process_noise_3d": 3000.0},
    "features": {"space": "position", "frame": "image", "record_time": 0.5,
                 "tolerance": {"position": 3.0, "edges": 3.0, "curvature": 1.5},
                 "tolerance_3d": {"position": 3.0, "edges": 3.0, "curvature": 1.5}},
    "teleop": {"joint_speed": 5.0, "max_joint_speed": 15.0},
    "moves": {"speed": 10.0, "home": []},
    "probe": {"amplitude": [10.0, 10.0, 10.0, 15.0, 15.0, 15.0], "speed": 5.0, "settle": 0.5, "cycles": 1},
    "control": {"rate": 25.0, "gain": 0.5, "damping": 0.05, "max_joint_speed": 8.0, "settle_time": 1.0,
                "timeout": 60.0, "max_image_age": 0.3},
    "broyden": {"enabled": True, "alpha": 0.2, "min_step": 0.5, "keep": False},
    "display": {"fps": 25.0, "plot_fps": 10.0, "window": 30.0, "image": "color", "brightness": 0.75, "downsample": 1,
                "curvature_scale": 25.0, "min_radius": 3.0},
    "logger": {"enabled": True, "folder": "shape_servoing", "window_video": True, "camera_video": True,
               "video_fps": 15.0},
}

CHOICES = {
    "camera.source": ("realsense", "webcam"),
    "features.space": SPACES,
    "features.frame": FRAMES,
    "display.image": ("color", "gray", "none"),
}
MAY_BE_EMPTY = {"camera.serial"}
# Numbers that may be zero (every other number must be > 0), and numbers with an upper bound
NOT_NEGATIVE = {"camera.device", "control.damping", "display.min_radius"}
AT_MOST_ONE = {"tracking.velocity_decay", "broyden.alpha", "display.brightness"}


class SettingsError(ConfigError):
    """04_shape_servoing.yaml cannot be read or contains an invalid value."""


def load_settings(path=None):
    """Read and check the settings file (default: 04_shape_servoing.yaml next to the example)."""
    path = Path(path) if path is not None else SETTINGS_PATH
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise SettingsError(f"settings file not found: {path}") from None
    except (OSError, yaml.YAMLError) as exc:
        raise SettingsError(f"cannot read {path.name}: {exc}") from exc
    try:
        settings = _merge(DEFAULTS, {} if data is None else data, "")
        _check(settings)
    except SettingsError as exc:
        raise SettingsError(f"{path.name}: {exc}") from None
    return settings


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _merge(defaults, data, where):
    """Defaults updated with `data`, checking every key and value type."""
    if not isinstance(data, dict):
        raise SettingsError(f"{where or 'the file'}: expected a section of 'key: value' lines")
    merged = copy.deepcopy(defaults)
    for key, value in data.items():
        path = f"{where}.{key}" if where else str(key)
        if key not in defaults:
            raise SettingsError(f"{path}: unknown setting (valid here: {', '.join(defaults)})")
        merged[key] = _value(defaults[key], value, path)
    return merged


def _value(default, value, path):
    if isinstance(default, dict):
        return _merge(default, value, path)
    if isinstance(default, bool):
        if not isinstance(value, bool):
            raise SettingsError(f"{path}: expected true or false, got {value!r}")
        return value
    if isinstance(default, int):
        if not isinstance(value, int) or isinstance(value, bool):
            raise SettingsError(f"{path}: expected a whole number, got {value!r}")
        return value
    if isinstance(default, float):
        if not _is_number(value):
            raise SettingsError(f"{path}: expected a number, got {value!r}")
        return float(value)
    if isinstance(default, str):
        if not isinstance(value, str) or (not value.strip() and path not in MAY_BE_EMPTY):
            raise SettingsError(f"{path}: expected a text value, got {value!r}")
        if path in CHOICES and value not in CHOICES[path]:
            raise SettingsError(f"{path}: must be one of {', '.join(CHOICES[path])}, got {value!r}")
        return value
    if isinstance(default, list):
        if not isinstance(value, list):
            raise SettingsError(f"{path}: expected a list [a, b, ...], got {value!r}")
        return value   # contents checked in _check
    raise AssertionError(path)


def _require(condition, path, message):
    if not condition:
        raise SettingsError(f"{path}: {message}")


def _check_numbers(section, where):
    for key, value in section.items():
        path = f"{where}.{key}" if where else key
        if isinstance(value, dict):
            _check_numbers(value, path)
        elif _is_number(value):
            if path in NOT_NEGATIVE:
                _require(value >= 0, path, f"must be zero or positive, got {value:g}")
            else:
                _require(value > 0, path, f"must be positive, got {value:g}")
            if path in AT_MOST_ONE:
                _require(value <= 1, path, f"must be at most 1, got {value:g}")


def _joint_list(value, path, check):
    _require(len(value) == 6 and all(_is_number(v) for v in value), path,
             f"expected 6 numbers (J1..J6 in deg), got {value!r}")
    _require(all(check(v) for v in value), path, f"invalid value in {value!r}")
    return [float(v) for v in value]


def _check(settings):
    """Ranges and list contents that the types alone do not cover."""
    _check_numbers(settings, "")
    ids = settings["markers"]["ids"]
    _require(all(isinstance(i, int) and not isinstance(i, bool) and i >= 0 for i in ids), "markers.ids",
             f"expected a list of marker IDs (whole numbers), got {ids!r}")
    _require(settings["markers"]["dictionary"].startswith("DICT_"), "markers.dictionary",
             "expected an OpenCV ArUco dictionary name such as DICT_4X4_50")
    probe = settings["probe"]
    probe["amplitude"] = _joint_list(probe["amplitude"], "probe.amplitude", lambda v: 0 <= v <= 45)
    _require(any(probe["amplitude"]), "probe.amplitude", "at least one joint needs a non-zero amplitude")
    home = settings["moves"]["home"]
    if home:
        settings["moves"]["home"] = _joint_list(home, "moves.home", lambda v: abs(v) <= 360)
    _require(settings["display"]["downsample"] <= 8, "display.downsample", "must be 1-8")
    _require(settings["tracking"]["confirm_frames"] <= 30, "tracking.confirm_frames", "must be 1-30")
