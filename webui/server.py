"""Local HTTP server: serves the panel (static/) and a small JSON API."""

import json
import logging
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import numpy as np

from ur10api.config import save_workspace
from webui import config
from webui import kinematics as kin
from webui.robot import PHASE_TEXT, MotionError, NotReady

log = logging.getLogger("webui")

STATIC_DIR = Path(__file__).with_name("static").resolve()
MODEL_DIR = (Path(__file__).resolve().parents[1] / "robot_model").resolve()   # served under /robot/
CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json",
    ".glb": "model/gltf-binary",
}
CACHED_DIRS = ("vendor",)   # large files that only change with a new version (and the robot model)


def _pendant_view(T):
    """Position [mm] with RPY [deg] and rotation vector [rad], like the UR pendant."""
    pose = kin.pose_to_xyzrpy(T)
    return {"xyz_mm": (pose[:3] * 1000).tolist(), "rpy_deg": np.degrees(pose[3:]).tolist(),
            "rotvec_rad": kin.rotation_vector(T).tolist()}


def _limits_json(limits):
    return {axis: list(limits[axis]) for axis in "xyz"}


def _limits_text(limits):
    return ", ".join(f"{axis} {lo * 1000:.0f} to {hi * 1000:.0f}" for axis, (lo, hi) in limits.items()) + " mm"


class Api:
    """Maps the HTTP endpoints to RobotLink / Motion / store calls."""

    def __init__(self, link, motion, poses, events):
        self.link, self.motion, self.poses, self.events = link, motion, poses, events

    def config(self):
        return {
            "robot": {"host": self.link.host, "port": self.link.port},
            "speeds": {"linear_mm_s": config.LINEAR_SPEED_MM_S, "angular_deg_s": config.ANGULAR_SPEED_DEG_S,
                       "joint_deg_s": config.JOINT_JOG_SPEED_DEG_S, "move_deg_s": config.MOVE_SPEED_DEG_S},
            "steps": {"linear_mm": config.LINEAR_STEPS_MM, "angular_deg": config.ANGULAR_STEPS_DEG,
                      "joint_deg": config.JOINT_STEPS_DEG},
            "keepalive_ms": int(config.WATCHDOG_S * 1000 / 4),
            "confirm_above_deg": config.CONFIRM_ABOVE_DEG,
            "settings_file": config.SETTINGS.name,   # where the workspace limits are saved
            "joint_names": kin.JOINT_NAMES,
            "viewer": {   # used by the 3D view
                "arm_joints": kin.ARM_JOINTS,
                "base_frame": kin.BASE_FRAME,
                "flange_frame": kin.FLANGE_FRAME,
                "tcp": config.TCP,
                "workspace": _limits_json(kin.workspace_limits()),
                "workspace_defaults": _limits_json(kin.DEFAULT_WORKSPACE),
            },
        }

    def state(self, since):
        s = self.link.state()
        out = {
            "phase": s.phase,
            "phase_text": PHASE_TEXT[s.phase],
            "error": self.link.error,
            "robot": {"host": self.link.host, "port": self.link.port},
            "rates": s.rates,
            "gripper": self._gripper(),
            "jog": {"text": self.motion.status[0], "level": self.motion.status[1],
                    "axes": self.motion.active_axes()},
            "log": self.events.since(since),
            "limits": _limits_json(kin.workspace_limits()),   # workspace box [m], UR base frame
        }
        if s.q is not None:
            pose = kin.pose_to_xyzrpy(s.T)
            flange = kin.fk(s.q, tcp=False)
            out.update(
                q_deg=np.degrees(s.q).tolist(),
                q_rad=s.q.tolist(),
                qd_deg_s=None if s.qd is None else np.degrees(s.qd).tolist(),
                tcp_mm=(pose[:3] * 1000).tolist(),
                rpy_deg=np.degrees(pose[3:]).tolist(),
                pose_si=pose.tolist(),
                # Same poses as the teach pendant can show them (Move tab, feature Base)
                pendant={
                    "tcp": _pendant_view(s.T),
                    "flange": _pendant_view(flange),
                    "tcp_offset_mm": float(np.linalg.norm(s.T[:3, 3] - flange[:3, 3]) * 1000),
                },
                workspace=kin.workspace_violations(s.T),
                force=s.force.tolist(),
                torque=s.torque.tolist(),
            )
        return out

    def _gripper(self):
        command = self.link.gripper_command
        feedback = self.link.gripper_feedback()
        if feedback is not None:
            return {"command": command, "feedback": feedback[0], "closed": feedback[1]}
        closed = {"Open": 0.0, "Close": 1.0}.get(command)   # no feedback: assume the last command
        return {"command": command, "feedback": None, "closed": closed}

    def post(self, path, body):
        m = self.motion
        if path == "/api/connect":
            self.link.connect_async(str(body.get("host", "")).strip(), int(body.get("port", config.ROBOT_PORT)))
        elif path == "/api/jog":
            m.set_jog(body.get("space"), body.get("frame"), body.get("axes", [0] * 6),
                      body.get("linear_mm_s", 0), body.get("angular_deg_s", 0), body.get("joint_deg_s", 0))
        elif path == "/api/step":
            m.step(body.get("space"), body.get("frame"), body.get("axis"), body.get("direction"),
                   body.get("size"), body.get("linear_mm_s", 0), body.get("angular_deg_s", 0),
                   body.get("joint_deg_s", 0))
        elif path in ("/api/plan", "/api/move"):
            return self._plan_or_move(body, execute=path == "/api/move")
        elif path == "/api/stop":
            m.stop()
        elif path == "/api/gripper":
            self.link.send_gripper(body.get("command"))
            log.info("Gripper: %s", body.get("command"))
        elif path == "/api/zero_ft":
            self.link.zero_ft()
            log.info("Zeroing the force/torque sensor")
        elif path == "/api/poses/save":
            self.poses.save(body.get("name", ""), body.get("q_deg", []))
            log.info("Saved pose %r", body.get("name"))
        elif path == "/api/poses/delete":
            self.poses.delete(body.get("name", ""))
            log.info("Deleted pose %r", body.get("name"))
        elif path == "/api/workspace":   # limits in mm, UR base frame
            kin.set_workspace_limits({axis: [float(v) / 1000.0 for v in body.get(axis, [])] for axis in "xyz"})
            self._save_workspace("set to")
        elif path == "/api/workspace/reset":
            kin.set_workspace_limits(kin.DEFAULT_WORKSPACE)
            self._save_workspace("reset to UR_CONTROL's defaults:")
        else:
            return None
        return {"ok": True}

    def _save_workspace(self, what):
        """Write the panel's limits into the settings file (used by ur10api programs started later)."""
        limits = kin.workspace_limits()
        try:
            path = save_workspace(limits, config.SETTINGS.path)
        except (OSError, ValueError) as exc:
            log.error("Workspace limits %s %s, but not saved: %s", what, _limits_text(limits), exc)
            raise ValueError(f"limits applied in the panel but not saved ({exc})") from None
        log.warning("Workspace limits %s %s (UR base frame), saved in %s", what, _limits_text(limits), path.name)

    def _plan_or_move(self, body, execute):
        speed = body.get("speed_deg_s", config.MOVE_SPEED_DEG_S[0])
        if body.get("kind") == "joints":
            args = (body.get("q_deg"),)
            plan = (self.motion.move_joints if execute else self.motion.plan_joints)(*args, speed)
        else:
            args = (body.get("xyz_mm"), body.get("rpy_deg"))
            plan = (self.motion.move_pose if execute else self.motion.plan_pose)(*args, speed)
        pose = kin.pose_to_xyzrpy(plan.T)
        return {
            "ok": True,
            "q_deg": np.degrees(plan.q).tolist(),
            "tcp_mm": (pose[:3] * 1000).tolist(),
            "rpy_deg": np.degrees(pose[3:]).tolist(),
            "duration": plan.duration,
            "worst_joint": kin.JOINT_NAMES[plan.worst],
            "max_motion_deg": float(np.degrees(plan.max_motion)),
            "warnings": plan.warnings,
        }


class Handler(BaseHTTPRequestHandler):
    api = None              # set by make_server
    allowed_hosts = None    # Host header values accepted for API calls (None = any)
    server_version = "UR10WebPanel/1.0"

    def log_message(self, fmt, *args):   # the panel polls several times per second
        pass

    def do_GET(self):
        url = urlparse(self.path)
        if url.path == "/api/state":
            since = int(parse_qs(url.query).get("since", ["0"])[0] or 0)
            self._send_json(self.api.state(since))
        elif url.path == "/api/config":
            self._send_json(self.api.config())
        elif url.path == "/api/poses":
            self._send_json({"ok": True, "poses": self.api.poses.all()})
        else:
            self._send_static(url.path)

    def _send_static(self, url_path):
        base, rel = (MODEL_DIR, url_path[len("/robot/"):]) if url_path.startswith("/robot/") \
            else (STATIC_DIR, url_path.lstrip("/") or "index.html")
        path = (base / rel).resolve()
        if base not in path.parents or not path.is_file() or path.suffix not in CONTENT_TYPES:
            return self._send_json({"ok": False, "error": "not found"}, HTTPStatus.NOT_FOUND)
        cache = "max-age=3600" if base is MODEL_DIR or path.relative_to(STATIC_DIR).parts[0] in CACHED_DIRS \
            else "no-store"
        self._send(HTTPStatus.OK, path.read_bytes(), CONTENT_TYPES[path.suffix], cache)

    def do_POST(self):
        # Only the panel itself may send commands: JSON bodies force a CORS preflight
        # for cross-site requests, and the Host/Origin checks stop DNS rebinding.
        if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
            return self._send_json({"ok": False, "error": "JSON body required"}, HTTPStatus.UNSUPPORTED_MEDIA_TYPE)
        host = self.headers.get("Host", "")
        origin = self.headers.get("Origin")
        if (self.allowed_hosts is not None and host.rsplit(":", 1)[0] not in self.allowed_hosts) or \
                (origin is not None and urlparse(origin).netloc != host):
            return self._send_json({"ok": False, "error": "forbidden origin"}, HTTPStatus.FORBIDDEN)
        try:
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length) or b"{}")
            result = self.api.post(urlparse(self.path).path, body)
        except (NotReady, MotionError, ValueError, TypeError) as exc:
            return self._send_json({"ok": False, "error": str(exc)})
        except Exception as exc:
            log.exception("API error on %s", self.path)
            return self._send_json({"ok": False, "error": f"internal error: {exc}"}, HTTPStatus.INTERNAL_SERVER_ERROR)
        if result is None:
            return self._send_json({"ok": False, "error": "not found"}, HTTPStatus.NOT_FOUND)
        self._send_json(result)

    def _send_json(self, obj, status=HTTPStatus.OK):
        self._send(status, json.dumps(obj).encode("utf-8"), "application/json")

    def _send(self, status, data, content_type, cache="no-store"):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", cache)
        self.end_headers()
        self.wfile.write(data)


def make_server(api, listen, port):
    handler = type("PanelHandler", (Handler,), {
        "api": api,
        "allowed_hosts": None if listen in ("0.0.0.0", "::") else {"localhost", "127.0.0.1", "[::1]", listen},
    })
    server = ThreadingHTTPServer((listen, port), handler)
    server.daemon_threads = True
    return server
