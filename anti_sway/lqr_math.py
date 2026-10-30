"""Continuous-time LQR (algebraic Riccati) — numpy only, optional scipy."""

from __future__ import annotations

import numpy as np


def solve_continuous_are(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Return P >= 0 with A'P + PA - PBR^{-1}B'P + Q = 0."""
    try:
        from scipy.linalg import solve_continuous_are as _scipy_care

        return np.asarray(_scipy_care(A, B, Q, R), dtype=float)
    except ImportError:
        return _care_hamiltonian(A, B, Q, R)


def lqr_gain(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """State feedback u = -K @ x minimizing x'Qx + u'Ru (continuous)."""
    P = solve_continuous_are(A, B, Q, R)
    R = np.asarray(R, dtype=float)
    if R.ndim == 1:
        R = R.reshape(1, 1)
    K = np.linalg.solve(R, B.T @ P)
    return np.asarray(K, dtype=float).reshape(-1)


def _care_hamiltonian(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    Rinv = np.linalg.inv(R)
    G = B @ Rinv @ B.T
    n = A.shape[0]
    H = np.block([[A, -G], [-Q, -A.T]])
    w, V = np.linalg.eig(H)
    stable = np.where(np.real(w) < -1e-8)[0]
    if stable.size < n:
        raise ValueError('Hamiltonian CARE: insufficient stable eigenvalues')
    Vs = V[:, stable]
    # Sort by eigenvalue real part for reproducibility
    order = np.argsort(np.real(w[stable]))
    Vs = Vs[:, order]
    V1 = Vs[:n, :]
    V2 = Vs[n:, :]
    P = (V2 @ np.linalg.inv(V1)).real
    P = 0.5 * (P + P.T)
    return P
