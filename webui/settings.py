"""Workspace limits chosen in the panel, kept in workspace_limits.json."""

import json
from pathlib import Path


class WorkspaceStore:
    """{"x": [min, max], "y": [...], "z": [...]} in metres, UR base frame."""

    def __init__(self, path):
        self.path = Path(path)

    def load(self):
        """Saved limits, or None when the panel uses UR_CONTROL's defaults."""
        try:
            return json.loads(self.path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return None

    def save(self, limits):
        data = {axis: [round(v, 4) for v in limits[axis]] for axis in "xyz"}
        self.path.write_text(json.dumps(data, indent=2), encoding="utf-8")

    def clear(self):
        self.path.unlink(missing_ok=True)
