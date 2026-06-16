"""
Utility for offline replay of flight recordings.

The :class:`FlightReplayer` class loads a ``.npz`` file produced by the
``flight_recorder`` script and provides an :meth:`evaluate` method that can
be used by unit tests or optimisation scripts.  It feeds the recorded
telemetry into an :class:`~src.estimation.ekf.ErrorStateEKF` instance and
returns root‑mean‑square errors together with the mean Normalised
Innovation Squared (NIS).
"""

import numpy as np
import src.config as config
from src.estimation.ekf import ErrorStateEKF
from typing import Dict, Any


class FlightReplayer:
    """Load a ``.npz`` flight recording and evaluate a Kalman filter.

    Parameters
    ----------
    npz_path:
        Path to the flight recording produced by :mod:`flight_recorder`.
    """

    def __init__(self, npz_path: str):
        self.data: Dict[str, Any] = np.load(npz_path, allow_pickle=True)
        # Convert to Python objects if NumPy arrays of dtype=object
        for key, val in self.data.items():
            if isinstance(val, np.ndarray) and val.dtype == np.object_:
                self.data[key] = [v.tolist() for v in val]

    @staticmethod
    def _H(up_vector: np.ndarray) -> np.ndarray:
        """Measurement matrix for altitude and velocity (4×12)."""
        H = np.zeros((4, 12))
        H[0, 0:3] = up_vector
        H[1, 3] = 1.0
        H[2, 4] = 1.0
        H[3, 5] = 1.0
        return H

    @staticmethod
    def _R() -> np.ndarray:
        """Measurement noise covariance for altimeter+velocimeter."""
        return np.diag(
            [config.SIGMA_ALT**2, config.SIGMA_VEL**2, config.SIGMA_VEL**2, config.SIGMA_VEL**2]
        )

    def evaluate(self, ekf: ErrorStateEKF, weights: Dict[str, float] | None = None):
        """Run a full re‑run of the estimator on recorded data.

        Parameters
        ----------
        ekf:
            An already‑initialised :class:`ErrorStateEKF` instance.
        weights:
            Optional dictionary of weights for the cost metric.

        Returns
        -------
        dict
            ``{"rmse_pos":…, "rmse_vel":…, "nis":…, "score":…}``
        """

        # Extract flattened arrays from the recording (pre-load everything to
        # avoid per‑iteration zip decompression — the dominant cost).
        ut = np.array(self.data["ut"])
        dt = np.array(self.data["dt"])
        gt_pos = np.array(self.data["gt_pos"])
        gt_vel = np.array(self.data["gt_vel"])
        mahony_att = np.array(self.data["mahony_attitude"])
        sf_body_noisy = np.array(self.data["sf_body_noisy"])
        raw_gyro = np.array(self.data["raw_gyro"])
        noisy_alt = np.array(self.data["noisy_alt"])
        noisy_vel = np.array(self.data["noisy_vel"])
        gravity_world = np.array(self.data["gravity_world"])

        N = len(ut)
        nis_vals = np.empty(N)
        pos_errs = np.empty((N, 3))
        vel_errs = np.empty((N, 3))

        for i in range(N):
            ekf.predict(
                sf_body_noisy[i], raw_gyro[i],
                mahony_att[i], gravity_world[i],
                float(dt[i]),
            )

            ekf.update(noisy_alt[i], noisy_vel[i])

            nis_vals[i] = ekf.get_last_nis()
            pos_errs[i] = ekf.pos - gt_pos[i]
            vel_errs[i] = ekf.vel - gt_vel[i]

        rmse_pos = float(np.sqrt(np.mean(np.sum(pos_errs**2, axis=1))))
        rmse_vel = float(np.sqrt(np.mean(np.sum(vel_errs**2, axis=1))))
        mean_nis = float(np.mean(nis_vals))

        if weights is None:
            w_pos = w_vel = w_nis = 1.0 / 3.0
        else:
            w_pos = weights.get("w_pos", 1.0 / 3.0)
            w_vel = weights.get("w_vel", 1.0 / 3.0)
            w_nis = weights.get("w_nis", 1.0 / 3.0)

        score = w_pos * rmse_pos + w_vel * rmse_vel + w_nis * mean_nis

        return {"rmse_pos": rmse_pos, "rmse_vel": rmse_vel, "nis": mean_nis, "score": score}

