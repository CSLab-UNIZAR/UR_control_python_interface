"""Start the web control panel:  python -m webui [--host ROBOT] [--http-port 8080]"""

import argparse
import io
import logging
import os
import sys
import threading
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:          # "core" is imported as a top-level package
    sys.path.insert(0, str(ROOT))

from webui import config                                   # noqa: E402
from webui import kinematics as kin                        # noqa: E402
from webui.poses import PoseStore                          # noqa: E402
from webui.robot import EventLog, Motion, RobotLink        # noqa: E402
from webui.server import Api, make_server                  # noqa: E402
from webui.settings import WorkspaceStore                  # noqa: E402


class _PrintsToLog(io.TextIOBase):
    """Routes print() output of UR10Control (e.g. 'Out of workspace limits') to the log."""

    def __init__(self, logger):
        self._logger = logger
        self._buffer = ""
        self._lock = threading.Lock()

    def writable(self):
        return True

    def write(self, text):
        with self._lock:
            self._buffer += text
            *lines, self._buffer = self._buffer.split("\n")
        for line in filter(None, (l.strip() for l in lines)):
            lowered = line.lower()
            level = logging.WARNING if ("error" in lowered or "out of workspace" in lowered) else logging.INFO
            self._logger.log(level, line)
        return len(text)


def _can_open_browser():
    """Without a display, webbrowser falls back to text-mode browsers (lynx, w3m...)
    that would take over this terminal, so only open one on a desktop session."""
    if sys.platform.startswith("linux"):
        return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    return True


def main():
    parser = argparse.ArgumentParser(prog="python -m webui", description="UR10 web control panel")
    parser.add_argument("--host", default=config.ROBOT_HOST, help="rosbridge host (default: %(default)s)")
    parser.add_argument("--port", type=int, default=config.ROBOT_PORT, help="rosbridge port (default: %(default)s)")
    parser.add_argument("--http-port", type=int, default=config.HTTP_PORT, help="panel port (default: %(default)s)")
    parser.add_argument("--listen", default=config.HTTP_HOST,
                        help="panel address; 0.0.0.0 lets other devices open it (default: %(default)s)")
    parser.add_argument("--no-connect", action="store_true", help="do not connect to the robot at start-up")
    parser.add_argument("--no-browser", action="store_true", help="do not open the browser")
    parser.add_argument("--no-gripper-feedback", action="store_true",
                        help="do not subscribe to the gripper feedback topics")
    args = parser.parse_args()

    events = EventLog()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(message)s", datefmt="%H:%M:%S")
    logging.getLogger().addHandler(events)
    logging.getLogger("twisted").setLevel(logging.WARNING)
    log = logging.getLogger("webui")

    link = RobotLink(gripper_topics=() if args.no_gripper_feedback else config.GRIPPER_FEEDBACK_TOPICS)
    link.host, link.port = args.host, args.port
    motion = Motion(link)
    workspace_store = WorkspaceStore(ROOT / "workspace_limits.json")
    saved_limits = workspace_store.load()
    if saved_limits:
        try:
            kin.set_workspace_limits(saved_limits)
            log.info("Workspace limits from %s: %s", workspace_store.path.name,
                     ", ".join(f"{a} {lo * 1000:.0f} to {hi * 1000:.0f}" for a, (lo, hi) in kin.workspace_limits().items()) + " mm")
        except ValueError as exc:
            log.error("Ignoring %s (%s); using UR_CONTROL's workspace limits", workspace_store.path.name, exc)
    api = Api(link, motion, PoseStore(ROOT / "saved_poses.json"), events, workspace_store)
    try:
        server = make_server(api, args.listen, args.http_port)
    except OSError as exc:
        sys.exit(f"Cannot open the panel on {args.listen}:{args.http_port} ({exc}). "
                 "Is it already running? Try --http-port 8081.")

    url = f"http://{'localhost' if args.listen in ('127.0.0.1', '0.0.0.0', '::') else args.listen}:{args.http_port}/"
    print(f"\n  UR10 control panel:  {url}\n  Press Ctrl+C here to quit.\n", flush=True)
    if args.listen in ("0.0.0.0", "::"):
        log.warning("The panel is reachable from the network: anyone who can open it can move the robot.")
    sys.stdout = _PrintsToLog(logging.getLogger("UR10Control"))

    if not args.no_connect:
        link.connect_async(args.host, args.port)
    if not args.no_browser and _can_open_browser():
        threading.Timer(0.5, webbrowser.open, (url,)).start()
    elif not args.no_browser:
        log.info("No desktop display found: open %s in a browser", url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        log.info("Shutting down ...")
        server.server_close()
        motion.shutdown()
        closer = threading.Thread(target=link.close, daemon=True)   # may block on a dead network
        closer.start()
        closer.join(timeout=5.0)
        sys.stdout = sys.__stdout__


if __name__ == "__main__":
    main()
