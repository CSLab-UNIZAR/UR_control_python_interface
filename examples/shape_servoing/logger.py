"""Run logger of example 4: every servo run saved to its own folder.

    logger = RunLogger(folder, settings, session.recorder, session.t0, vision)
    logger.enabled = True                  # the window's "Log runs" toggle (applies to the next run)
    logger.start("edges3d", meta)          # control thread: a servo run starts
    if logger.wants_window_frame():        # window thread, after each redraw
        logger.window_frame(rgba_copy)
    logger.stop(result)                    # control thread: the run ended (files written in the background)
    logger.close()                         # at exit: waits until every file is written

A run folder, <folder>/<date>_<feature set>/, holds:
    signals.csv   the rows of the session log during the run (t_run [s], then every signal)
    meta.json     settings, feature set, target, Jacobians, outcome
    window.mp4    the whole window (image space, 3D view, plots) at video_fps
    camera.mp4    the raw camera images at video_fps

The videos are written by background threads at a fixed rate (the newest
frame at each tick), so their duration is the real duration of the run and
neither the window nor the control loop waits for the encoder.
"""

import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np


class VideoWriter:
    """Writes the newest frame given by source() (BGR, or RGBA with rgba=True) to an MP4 at a fixed rate."""

    def __init__(self, path, fps, source, rgba=False):
        self.path, self.period, self.source, self.rgba = Path(path), 1.0 / fps, source, rgba
        self.frames, self.error = 0, ""
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"video {self.path.name}", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=10.0)

    def _run(self):
        writer, size, due = None, None, time.monotonic()
        try:
            while not self._stop.is_set():
                frame = self.source()
                if frame is not None:
                    if self.rgba:
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                    if writer is None:           # the first frame fixes the size (even, for the encoder)
                        size = (frame.shape[1] // 2 * 2, frame.shape[0] // 2 * 2)
                        writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"mp4v"), 1.0 / self.period,
                                                 size)
                        if not writer.isOpened():
                            self.error = f"cannot write {self.path.name}"
                            return
                    if (frame.shape[1], frame.shape[0]) != size:
                        frame = cv2.resize(frame, size)   # e.g. the window was resized
                    writer.write(frame)
                    self.frames += 1
                due += self.period
                delay = due - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                elif delay < -1.0:               # far behind (slow disk): drop instead of catching up
                    due = time.monotonic()
        finally:
            if writer is not None:
                writer.release()


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    raise TypeError(f"not JSON serializable: {type(value).__name__}")


class RunLogger:
    """Saves each servo run while `enabled` (see the module documentation)."""

    def __init__(self, folder, settings, recorder, t0, vision):
        cfg = settings["logger"]
        self.folder, self.settings, self.recorder, self.t0, self.vision = Path(folder), settings, recorder, t0, vision
        self.enabled = cfg["enabled"]
        self.fps, self.window_video, self.camera_video = cfg["video_fps"], cfg["window_video"], cfg["camera_video"]
        self.run_dir = None                  # folder of the run being logged (None: not logging)
        self.last_dir = None                 # folder of the last run logged
        self._window = None                  # newest window frame (RGBA)
        self._next_frame = 0.0
        self._writers, self._pending = [], []

    @property
    def active(self):
        return self.run_dir is not None

    def start(self, name, meta):
        """Begin logging a run (does nothing when disabled or already logging)."""
        if not self.enabled or self.active:
            return
        run_dir = self.folder / f"{time.strftime('%Y%m%d_%H%M%S')}_{name}"
        try:
            run_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            print(f"Run not logged: {exc}")
            return
        self._t_start, self._meta, self._window = time.monotonic() - self.t0, dict(meta), None
        self._writers = []
        if self.window_video:
            self._writers.append(VideoWriter(run_dir / "window.mp4", self.fps, lambda: self._window, rgba=True))
        if self.camera_video:
            self._writers.append(VideoWriter(run_dir / "camera.mp4", self.fps, self._camera_image))
        self.run_dir = run_dir

    def _camera_image(self):
        obs = self.vision.latest()
        return None if obs is None else obs.image

    def wants_window_frame(self):
        """True when the window should hand over a frame (logging, window video on, frame due)."""
        now = time.monotonic()
        if not (self.active and self.window_video) or now < self._next_frame:
            return False
        self._next_frame = now + 1.0 / self.fps
        return True

    def window_frame(self, rgba):
        self._window = rgba

    def stop(self, result):
        """End the run: videos are closed and the files written in a background thread."""
        if not self.active:
            return
        run_dir, writers, meta = self.run_dir, self._writers, self._meta
        self.run_dir, self._writers, self._window = None, [], None
        span = (self._t_start, time.monotonic() - self.t0)
        thread = threading.Thread(target=self._finish, args=(run_dir, writers, meta, dict(result), span),
                                  name="run logger")
        thread.start()
        self._pending.append(thread)
        self.last_dir = run_dir

    def close(self):
        """Wait until every run is written (call before exiting)."""
        for thread in self._pending:
            thread.join(timeout=30.0)

    def _finish(self, run_dir, writers, meta, result, span):
        for writer in writers:
            writer.stop()
        t, data, columns = self.recorder.between(*span)
        names = sorted(columns, key=columns.get)
        try:
            np.savetxt(run_dir / "signals.csv", np.c_[t - span[0], data], delimiter=",", fmt="%.6g",
                       header=",".join(["t_run"] + names), comments="")
            meta.update(result, duration=span[1] - span[0], rows=len(t), settings=self.settings,
                        videos={w.path.name: {"frames": w.frames, "fps": self.fps, "error": w.error} for w in writers})
            (run_dir / "meta.json").write_text(json.dumps(meta, indent=1, default=_jsonable), encoding="utf-8")
            print(f"Run logged in {run_dir}")
        except OSError as exc:
            print(f"Could not write the run log in {run_dir}: {exc}")
