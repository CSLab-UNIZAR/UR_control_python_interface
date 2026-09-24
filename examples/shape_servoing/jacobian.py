"""Model-free Jacobian of the shape features with respect to the joints.

J (m x 6) maps a small joint motion dq [rad] to the feature change it causes:
ds = J dq. Nothing about the camera or the object is modelled; J is estimated:

  offline   fit():            least squares over probing samples, ds_k = J dq_k + b
  online    broyden_update(): rank-one correction along each observed motion
  control   damped_pinv():    J^T (J J^T + mu^2 I)^-1, mu = damping * largest singular value
"""

import numpy as np


def fit(dq, ds):
    """Least-squares J from samples: ds_k ~= J dq_k + b.

    dq: K x n joint displacements from the probing configuration [rad]
    ds: K x m feature changes from the reference features
    The intercept b absorbs noise in the reference. Returns (J (m x n), RMS residual).
    """
    dq, ds = np.asarray(dq, dtype=float), np.asarray(ds, dtype=float)
    A = np.c_[dq, np.ones(len(dq))]
    coef, *_ = np.linalg.lstsq(A, ds, rcond=None)
    residual = ds - A @ coef
    return coef[:-1].T, float(np.sqrt(np.mean(residual ** 2)))


def broyden_update(J, dq, ds, alpha=1.0):
    """Broyden's rank-one update: J + alpha (ds - J dq) dq^T / (dq^T dq).

    After it (with alpha = 1), J dq = ds exactly: J explains the last observed
    motion, and is unchanged for motions orthogonal to dq.
    """
    dq = np.asarray(dq, dtype=float)
    return J + alpha * np.outer(np.asarray(ds, dtype=float) - J @ dq, dq) / float(dq @ dq)


def damped_pinv(J, damping):
    """Damped pseudoinverse J^T (J J^T + mu^2 I)^-1 with mu = damping * sigma_max(J), via the SVD.

    Each singular value s is inverted as s / (s^2 + mu^2): like 1/s for s >> mu,
    but bounded near singular directions instead of exploding.
    """
    U, s, Vt = np.linalg.svd(J, full_matrices=False)
    mu = damping * (s[0] if len(s) else 0.0)
    denom = s * s + mu * mu
    inverse = np.divide(s, denom, out=np.zeros_like(s), where=denom > 0)
    return (Vt.T * inverse) @ U.T


def condition(J):
    """Condition number sigma_max / sigma_min of J (inf when singular)."""
    s = np.linalg.svd(J, compute_uv=False)
    if not len(s) or s[-1] <= 1e-12 * max(s[0], 1e-300):
        return float("inf")
    return float(s[0] / s[-1])
