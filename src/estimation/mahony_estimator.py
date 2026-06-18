"""
Mahony complementary attitude filter.

Decoupled from the EKF — this filter owns attitude estimation using
gyro integration (short-term) and accelerometer gravity-reference
correction (long-term).  Gyroscope bias is supplied by the EKF
via the ``set_gyro_bias()`` callback so the Mahony filter integrates
bias-corrected rates.

Usage
─────
    mahony = MahonyAttitudeEstimator(kp=2.0, ki=0.1)
    mahony.set_gyro_bias(bg_from_ekf)      # optional, each tick
    q = mahony.update(omega_body, f_body, gravity_ned, dt)
"""
import numpy as np
import logging
import src.config as config
from scipy.spatial.transform import Rotation as R  # type: ignore

logger = logging.getLogger(__name__)


class MahonyAttitudeEstimator:
    """
    Standalone Mahony complementary filter for attitude.

    Does **not** poll sensors — accepts raw IMU data via ``update()``.
    Gyro bias comes from the EKF via ``set_gyro_bias()``.
    """

    def __init__(
        self,
        kp: float | None = None,
        ki: float | None = None,
        up_vector: np.ndarray = np.array([0.0, 0.0, 1.0]),
        gravity_magnitude: float = 9.81,
    ) -> None:
        self.kp: float = kp if kp is not None else config.MAHONY_KP
        self.ki: float = ki if ki is not None else config.MAHONY_KI
        self.up_vector: np.ndarray = up_vector / np.linalg.norm(up_vector)
        self.g_mag: float = gravity_magnitude

        self.quaternion: np.ndarray = np.array([0.0, 0.0, 0.0, 1.0])
        self.integral_error: np.ndarray = np.zeros(3)

        self._external_bg: np.ndarray = np.zeros(3)
        self._correction_enabled: bool = True

        logger.info(f"Initialized MahonyAttitudeEstimator kp={self.kp} ki={self.ki}")

    # ────────────────────────────────────────────────────────────────
    #  CORRECTION TOGGLE
    # ────────────────────────────────────────────────────────────────

    def enable_correction(self) -> None:
        self._correction_enabled = True

    def disable_correction(self) -> None:
        self._correction_enabled = False

    # ────────────────────────────────────────────────────────────────
    #  BIAS FEEDBACK FROM EKF
    # ────────────────────────────────────────────────────────────────

    def set_gyro_bias(self, bg: np.ndarray) -> None:
        """Accept gyroscope bias estimate from the EKF [rad/s]."""
        self._external_bg = bg.copy()

    # ────────────────────────────────────────────────────────────────
    #  MAIN UPDATE
    # ────────────────────────────────────────────────────────────────

    def update(
        self,
        omega_body: np.ndarray,
        f_body: np.ndarray,
        gravity_ned: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """
        Propagate and correct the attitude estimate.

        Parameters
        ----------
        omega_body : ndarray (3,)
            Raw gyroscope reading in body frame [rad/s].
        f_body : ndarray (3,)
            Raw specific-force reading in body frame [m/s²].
        gravity_ned : ndarray (3,)
            Gravity vector in NED frame [m/s²].
        dt : float
            Time step [s].

        Returns
        -------
        quaternion : ndarray (4,)
            Updated attitude [x, y, z, w] (body→NED).
        """
        # ── 1. Subtract EKF-estimated gyro bias ─────────────────────
        omega_corr: np.ndarray = omega_body - self._external_bg

        # ── 2. Gravity-reference correction ─────────────────────────
        #   The accelerometer measures specific force f = a − g.
        #   When stationary, f = −g  →  normalised f should align
        #   with the expected gravity direction in body frame.
        g_mag: float = float(np.linalg.norm(gravity_ned))
        # In NED, gravity points along +z (Down).  Fallback to (0,0,1) if
        # the magnitude is negligible (should never happen near Kerbin).
        g_ned_unit: np.ndarray = gravity_ned / g_mag if g_mag > 1e-6 else np.array([0.0, 0.0, 1.0])

        #   Expected gravity in body frame given current quaternion.
        #   gravity_ned is NED-frame → rotate to body via q⁻¹.
        g_expected_body: np.ndarray = R.from_quat(self.quaternion).inv().apply(g_ned_unit)

        #   Measured gravity direction in body frame (inverted specific force)
        f_norm: float = float(np.linalg.norm(f_body))
        error: np.ndarray = np.zeros(3)
        if self._correction_enabled and f_norm > 0.5:
            f_unit_body: np.ndarray = -f_body / f_norm
            error = np.cross(g_expected_body, f_unit_body)

        # ── 3. PI correction on gyro ────────────────────────────────
        if self.ki > 0.0:
            self.integral_error += error * dt
            omega_corr = omega_corr + self.kp * error + self.ki * self.integral_error
        else:
            omega_corr = omega_corr + self.kp * error

        # ── 4. Quaternion integration ──────────────────────────────
        angle = float(np.linalg.norm(omega_corr))
        q = self.quaternion
        if angle < 1e-12:
            self.quaternion = q / np.linalg.norm(q)
        else:
            axis = omega_corr / angle
            ha = 0.5 * angle * dt
            dq = np.array([
                axis[0] * np.sin(ha),
                axis[1] * np.sin(ha),
                axis[2] * np.sin(ha),
                np.cos(ha),
            ])
            q_new = _quat_mul(q, dq)
            self.quaternion = q_new / np.linalg.norm(q_new)

        logger.debug(
            f"Mahony: ω=[{omega_body[0]:.3f},{omega_body[1]:.3f},{omega_body[2]:.3f}] "
            f"err=[{error[0]:.3f},{error[1]:.3f},{error[2]:.3f}] "
            f"|f|={f_norm:.3f} "
            f"q=[{self.quaternion[0]:.3f},{self.quaternion[1]:.3f},"
            f"{self.quaternion[2]:.3f},{self.quaternion[3]:.3f}]"
        )
        return self.quaternion.copy()

    # ────────────────────────────────────────────────────────────────
    #  ACCESSORS
    # ────────────────────────────────────────────────────────────────

    def get_attitude(self) -> np.ndarray:
        """Return current attitude quaternion (4,) [x,y,z,w]."""
        return self.quaternion.copy()

    def reset(self) -> None:
        """Reset to identity attitude."""
        self.quaternion = np.array([0.0, 0.0, 0.0, 1.0])
        self.integral_error = np.zeros(3)
        self._external_bg = np.zeros(3)
        self._correction_enabled = True


def _quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product of two [x,y,z,w] quaternions."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])
