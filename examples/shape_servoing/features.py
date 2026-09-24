"""Shape features of a chain of markers, taken in increasing ID order, in 2D or 3D.

Two frames: "image" (2D marker centres [px]) and "camera" (3D marker positions
in the camera frame [m], from the marker corners; their scale is set by the
nominal marker size, which does not matter for this data-driven control).
For N marker points p_1..p_N (2D or 3D):

    position   s = [p_1, ..., p_N]                                  2N / 3N values
    edges      e_i = p_(i+1) - p_i:  s = [e_1, ..., e_(N-1)]         2(N-1) / 3(N-1)
    curvature  k_i = angle between e_i and e_(i+1), at p_(i+1)      N-2        [rad]

In the image the curvature is signed (positive when the chain turns clockwise
on the screen, the v axis points down); in 3D it is unsigned (0..pi), since
there is no plane to give it a sign. Three markers are the minimum.

A feature set is named by a key: the space for the image ("edges"), the space
followed by "3d" for the camera frame ("edges3d").

    s = features(points, "edges")
    e = difference(s, s_target, "edges")      # s - s_target (angles wrapped to +-pi)
"""

import numpy as np

SPACES = ("position", "edges", "curvature")
FRAMES = ("image", "camera")
COLORMAPS = {"position": "Blues", "edges": "Oranges", "curvature": "Greens"}
COLORS = {"position": "tab:blue", "edges": "tab:orange", "curvature": "tab:green"}
MIN_MARKERS = 3


def key(space, frame):
    """Name of a feature set: 'edges' (image) or 'edges3d' (camera)."""
    return space if frame == "image" else space + "3d"


KEYS = tuple(key(space, frame) for frame in FRAMES for space in SPACES)


def split(name):
    """(space, frame) of a feature set name."""
    return (name[:-2], "camera") if name.endswith("3d") else (name, "image")


def unit(name):
    """Unit shown for a feature set: px (image), mm (camera, up to scale) or deg (curvature)."""
    space, frame = split(name)
    return "deg" if space == "curvature" else "px" if frame == "image" else "mm"


def dims(frame):
    return 2 if frame == "image" else 3


def features(points, space):
    """Feature vector of the marker points (N x 2 or N x 3, in increasing ID order)."""
    p = np.asarray(points, dtype=float)
    if space == "position":
        return p.ravel()
    edges = np.diff(p, axis=0)
    if space == "edges":
        return edges.ravel()
    if space == "curvature":
        return turning_angles(edges)
    raise ValueError(f"unknown feature space {space!r}")


def turning_angles(edges):
    """Angle [rad] from each edge to the next (N-1 edges -> N-2 angles): signed in 2D, unsigned in 3D."""
    a, b = edges[:-1], edges[1:]
    if edges.shape[1] == 2:
        cross = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
    else:
        cross = np.linalg.norm(np.cross(a, b), axis=1)
    return np.arctan2(cross, np.sum(a * b, axis=1))


def difference(s, s_ref, space):
    """s - s_ref; curvature differences are wrapped to (-pi, pi]."""
    d = np.asarray(s, dtype=float) - np.asarray(s_ref, dtype=float)
    if space == "curvature":
        d = -((-d + np.pi) % (2 * np.pi) - np.pi)
    return d


def size(n_markers, space, frame="image"):
    """Number of features of `n_markers` markers."""
    d = dims(frame)
    return {"position": d * n_markers, "edges": d * (n_markers - 1), "curvature": n_markers - 2}[space]


def names(ids, space, frame="image"):
    """Component names: u10 v10 / x10 y10 z10 (position), x10-11 ... (edges), 10-11-20 (curvature)."""
    ids = list(ids)
    axes = "uv" if frame == "image" else "xyz"
    if space == "position":
        return [f"{axis}{i}" for i in ids for axis in axes]
    if space == "edges":
        return [f"{axis}{a}-{b}" for a, b in zip(ids, ids[1:]) for axis in axes.replace("uv", "xy")]
    return [f"{a}-{b}-{c}" for a, b, c in zip(ids, ids[1:], ids[2:])]


def to_display(values, name):
    """Values in the units shown to the user: curvature in deg, camera-frame lengths in mm."""
    space, frame = split(name)
    values = np.asarray(values, dtype=float)
    if space == "curvature":
        return np.degrees(values)
    return values * 1000.0 if frame == "camera" else values


def from_display(value, name):
    """Inverse of to_display (e.g. a tolerance given in display units)."""
    space, frame = split(name)
    if space == "curvature":
        return np.radians(value)
    return value / 1000.0 if frame == "camera" else value
