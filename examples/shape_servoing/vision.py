"""Camera capture, ArUco detection and marker tracking (2D and 3D), in a background thread.

    vision = Vision(settings)            # opens the RealSense (or a webcam)
    vision.start()
    obs = vision.wait_first()            # newest Observation: image and filtered marker positions
    obs.points([10, 11, 20])             # (3, 2) centres [px], or None if one is not tracked
    obs.points([10, 11, 20], "camera")   # (3, 3) positions in the camera frame [m]
    vision.close()

Every frame: detect the ArUco markers; the centre is the mean of the 4 corners
[px], and the 3D position in the camera frame [m] comes from the corners with
solvePnP, the camera intrinsics and the nominal marker size (which only sets
the scale). Each is filtered by one Kalman filter per marker ID
(MarkerTracker). The observation holds the filtered values of all tracked
markers, including those predicted through a few frames without detection
("coasting").
"""

import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np


class VisionError(RuntimeError):
    """The camera cannot be opened or stopped delivering images, or bad marker settings."""


@dataclass(frozen=True)
class Observation:
    """One processed camera frame."""
    seq: int                 # frame number (increases by one per processed frame)
    time: float              # time.monotonic() when the frame arrived
    image: np.ndarray        # BGR image as captured
    markers: dict            # {id: np.array([u, v])}: filtered centres of the tracked markers [px]
    markers3d: dict          # {id: np.array([x, y, z])}: filtered positions in the camera frame [m]
    detected: frozenset      # ids detected in this very frame (the others are predicted)
    fps: float               # processed frames per second

    def points(self, ids, frame="image"):
        """Centres [px] ("image") or positions [m] ("camera") of the markers `ids`, in that order,
        or None if one of them is not tracked."""
        markers = self.markers if frame == "image" else self.markers3d
        if not all(i in markers for i in ids):
            return None
        return np.array([markers[i] for i in ids], dtype=float).reshape(len(ids), -1)


# --- cameras -------------------------------------------------------------------------------

class RealSenseCamera:
    """Colour stream of an Intel RealSense (D435, D415, ...) through pyrealsense2, with its intrinsics."""

    def __init__(self, width, height, fps, serial=""):
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise VisionError("pyrealsense2 is not installed (pip install -r requirements.txt), "
                              "or use camera.source: webcam") from exc
        if not len(rs.context().query_devices()):
            raise VisionError("no RealSense camera found: check the USB cable (use a USB 3 port)")
        config = rs.config()
        if serial:
            config.enable_device(serial)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self._pipe = rs.pipeline()
        try:
            profile = self._pipe.start(config)
        except RuntimeError as exc:
            raise VisionError(f"cannot start the RealSense colour stream {width}x{height} at {fps} fps ({exc}); "
                              "check camera.width/height/fps and camera.serial") from exc
        device = profile.get_device()
        self.name = f"{device.get_info(rs.camera_info.name)} {device.get_info(rs.camera_info.serial_number)}"
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K = np.array([[intr.fx, 0.0, intr.ppx], [0.0, intr.fy, intr.ppy], [0.0, 0.0, 1.0]])
        self.dist = np.array(intr.coeffs, dtype=float)

    def read(self):
        frames = self._pipe.wait_for_frames(2000)    # RuntimeError after 2 s without a frame
        return np.asanyarray(frames.get_color_frame().get_data()).copy()

    def close(self):
        self._pipe.stop()


class Webcam:
    """Any camera OpenCV can open (to try the vision part without the RealSense).

    Its intrinsics are unknown: a focal length equal to the image width (about
    53 deg of horizontal field of view) is assumed, enough for the 3D features.
    """

    def __init__(self, device, width, height, fps):
        self._cap = cv2.VideoCapture(device)
        if not self._cap.isOpened():
            raise VisionError(f"cannot open webcam {device} (camera.device)")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)
        self.name = f"webcam {device}"
        w, h = self._cap.get(cv2.CAP_PROP_FRAME_WIDTH) or width, self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or height
        self.K = np.array([[w, 0.0, w / 2], [0.0, w, h / 2], [0.0, 0.0, 1.0]])
        self.dist = np.zeros(5)

    def read(self):
        ok, image = self._cap.read()
        if not ok:
            raise VisionError("the webcam returned no image")
        return image

    def close(self):
        self._cap.release()


# --- detection and tracking ----------------------------------------------------------------

class MarkerDetector:
    """ArUco detection: {id: [corners (4 x 2), ...]} (a list, in case an ID is seen twice)."""

    def __init__(self, dictionary, refine_corners=True):
        code = getattr(cv2.aruco, dictionary, None)
        if not isinstance(code, int):
            raise VisionError(f"markers.dictionary: unknown ArUco dictionary {dictionary!r} (e.g. DICT_4X4_50)")
        params = cv2.aruco.DetectorParameters()
        if refine_corners:
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(code), params)

    def detect(self, image):
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = self._detector.detectMarkers(gray)
        found = {}
        if ids is not None:
            for quad, marker_id in zip(corners, ids.ravel()):
                found.setdefault(int(marker_id), []).append(quad.reshape(4, 2))
        return found


def marker_position(corners, K, dist, size):
    """Centre of a square marker of side `size` [m] in the camera frame [m], from its 4 corners
    (ArUco order: top-left, top-right, bottom-right, bottom-left), or None."""
    h = size / 2
    square = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    ok, _rvec, tvec = cv2.solvePnP(square, corners.astype(np.float64), K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return tvec.ravel() if ok and tvec[2, 0] > 0 else None


class _Track:
    """Constant-velocity Kalman filter of one marker position (2D or 3D).

    All axes share the same model and noise, so they share one 2x2 covariance P
    over (position, velocity); x holds [position, velocity] per axis.
    """

    def __init__(self, z, t, r):
        z = np.asarray(z, dtype=float)
        self.x = np.c_[z, np.zeros(len(z))]             # rows: axes; columns: position, velocity
        self.P = np.diag([r * r, 1e6 * r * r])          # velocity unknown at first
        self.t = self.last_seen = t
        self.hits = 1

    def predict(self, t, q):
        """Propagate to time t with acceleration noise q (white-noise acceleration model)."""
        dt = t - self.t
        self.t = t
        if dt <= 0:
            return
        F = np.array([[1.0, dt], [0.0, 1.0]])
        G = np.array([0.5 * dt * dt, dt])
        self.x = self.x @ F.T
        self.P = F @ self.P @ F.T + q * q * np.outer(G, G)

    def correct(self, z, r):
        """Update with a detected position z, measurement noise r."""
        gain = self.P[:, 0] / (self.P[0, 0] + r * r)    # Kalman gain for H = [1, 0]
        self.x += np.outer(np.asarray(z) - self.x[:, 0], gain)
        self.P -= np.outer(gain, self.P[0, :])


class MarkerTracker:
    """One Kalman filter per marker ID (pixel centres, or 3D positions: the units of r and q).

    With a large process noise and a small measurement noise the estimate follows
    the detections closely; when a marker is not detected, its filter predicts it
    from the last velocity (slowed by velocity_decay each frame) until it is seen
    again or max_missing seconds have passed. A new marker is reported once it was
    detected confirm_frames times, which filters out one-frame false detections.
    """

    def __init__(self, measurement_noise=1.0, process_noise=3000.0, confirm_frames=2, max_missing=0.5,
                 velocity_decay=0.7, ids=()):
        self.r, self.q = measurement_noise, process_noise
        self.confirm_frames, self.max_missing, self.decay = confirm_frames, max_missing, velocity_decay
        self.ids = set(ids)                 # empty: every ID
        self.tracks = {}

    def update(self, detections, t):
        """Feed the detections {id: [positions]} of a frame taken at t; returns (markers, detected ids)."""
        detected = set()
        for marker_id, track in list(self.tracks.items()):
            track.predict(t, self.q)
            values = detections.get(marker_id)
            if values:
                predicted = track.x[:, 0]
                track.correct(min(values, key=lambda c: np.sum((c - predicted) ** 2)), self.r)
                track.last_seen = t
                track.hits += 1
                detected.add(marker_id)
            elif t - track.last_seen > self.max_missing:
                del self.tracks[marker_id]
            else:
                track.x[:, 1] *= self.decay
        for marker_id, values in detections.items():
            if marker_id not in self.tracks and (not self.ids or marker_id in self.ids):
                self.tracks[marker_id] = _Track(values[0], t, self.r)
                detected.add(marker_id)
        markers = {i: tr.x[:, 0].copy() for i, tr in self.tracks.items() if tr.hits >= self.confirm_frames}
        return markers, frozenset(detected & set(markers))


# --- the thread ----------------------------------------------------------------------------

class Vision:
    """Grabs frames, detects and tracks the markers in a background thread; latest() is the newest result."""

    def __init__(self, settings, source=None):
        cam, mk, tr = settings["camera"], settings["markers"], settings["tracking"]
        self.detector = MarkerDetector(mk["dictionary"], mk["refine_corners"])
        common = (tr["confirm_frames"], tr["max_missing"], tr["velocity_decay"], mk["ids"])
        self.tracker = MarkerTracker(tr["measurement_noise"], tr["process_noise"], *common)
        self.tracker3d = MarkerTracker(tr["measurement_noise_3d"] / 1000, tr["process_noise_3d"] / 1000, *common)
        self.marker_size = mk["size"] / 1000                            # m
        source = source or cam["source"]
        if source == "realsense":
            self.camera = RealSenseCamera(cam["width"], cam["height"], cam["fps"], cam["serial"])
        else:
            self.camera = Webcam(cam["device"], cam["width"], cam["height"], cam["fps"])
        self.error = ""                     # last capture problem ("" while images arrive)
        self._obs = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="vision", daemon=True)

    def start(self):
        self._thread.start()

    def latest(self):
        """The newest Observation (None before the first frame)."""
        return self._obs

    def wait_first(self, timeout=5.0):
        """Wait for the first processed frame; raises VisionError after `timeout` s."""
        deadline = time.monotonic() + timeout
        while self._obs is None:
            if time.monotonic() > deadline or not self._thread.is_alive():
                raise VisionError(f"no image from {self.camera.name} within {timeout:.0f} s"
                                  + (f" ({self.error})" if self.error else ""))
            time.sleep(0.02)
        return self._obs

    def close(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=3.0)
        try:
            self.camera.close()
        except Exception:   # noqa: BLE001 - closing a camera that already failed
            pass

    def _run(self):
        seq, period, last = 0, 0.0, None
        K, dist = self.camera.K, self.camera.dist
        while not self._stop.is_set():
            try:
                image = self.camera.read()
            except Exception as exc:   # noqa: BLE001 - unplugged, timeout: report and keep trying
                self.error = f"camera: {exc}"
                time.sleep(0.2)
                continue
            now = time.monotonic()
            quads = self.detector.detect(image)
            centres = {i: [q.mean(axis=0) for q in qs] for i, qs in quads.items()}
            positions = {i: [p for p in (marker_position(q, K, dist, self.marker_size) for q in qs) if p is not None]
                         for i, qs in quads.items()}
            markers, detected = self.tracker.update(centres, now)
            markers3d, _ = self.tracker3d.update({i: p for i, p in positions.items() if p}, now)
            if last is not None:                   # smoothed frame period
                period = now - last if period == 0 else 0.9 * period + 0.1 * (now - last)
            last = now
            seq += 1
            self.error = ""
            self._obs = Observation(seq, now, image, markers, markers3d, detected,
                                    1.0 / period if period > 0 else 0.0)
