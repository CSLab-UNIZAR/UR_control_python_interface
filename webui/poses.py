"""Named joint poses: presets from config.py plus poses saved from the panel."""

import json
import threading
from pathlib import Path

from webui import config


class PoseStore:
    """Stores user poses as {"name": [J1..J6 in degrees]} in a JSON file."""

    def __init__(self, path):
        self.path = Path(path)
        self._lock = threading.Lock()

    def _load(self):
        try:
            return json.loads(self.path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return {}

    def all(self):
        """[{name, q_deg, builtin}], presets first."""
        with self._lock:
            saved = self._load()
        poses = [{"name": n, "q_deg": list(q), "builtin": True} for n, q in config.PRESET_POSES_DEG.items()]
        poses += [{"name": n, "q_deg": q, "builtin": False} for n, q in sorted(saved.items())]
        return poses

    def save(self, name, q_deg):
        name = str(name).strip()
        q_deg = [round(float(v), 3) for v in q_deg]
        if not name or name in config.PRESET_POSES_DEG:
            raise ValueError("choose a new, non-empty name")
        if len(q_deg) != 6:
            raise ValueError("a pose needs 6 joint values")
        with self._lock:
            saved = self._load()
            saved[name] = q_deg
            self.path.write_text(json.dumps(saved, indent=2), encoding="utf-8")

    def delete(self, name):
        with self._lock:
            saved = self._load()
            if saved.pop(name, None) is None:
                raise ValueError(f"no saved pose named {name!r}")
            self.path.write_text(json.dumps(saved, indent=2), encoding="utf-8")
