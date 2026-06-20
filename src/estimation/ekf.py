"""
9-State Error-State Kalman Filter for translational + bias estimation.

Architecture
────────────
• **Mahony complementary filter** owns attitude estimation (gyro
  integration + accelerometer gravity-reference correction).
• **This EKF** owns position, velocity, gyroscope bias, and
  accelerometer bias estimation.
• The EKF consumes the Mahony-estimated attitude to rotate body-frame
  specific force into the NED frame, then adds gravity.  It remains
  frame-safe: f_corrected is rotated to NED before gravity addition.
• The EKF's bias estimates are fed back to the Mahony filter each tick
  so that the Mahony filter uses bias-corrected gyro rates.

Error state (9-dim)
───────────────────
    δpos  (3)  — position error        [m]           indices 0:3
    δvel  (3)  — velocity error        [m/s]         indices 3:6
    δbg   (3)  — gyro bias error       [rad/s]       indices 6:9
    δba   (3)  — accel bias error      [m/s²]       indices 9:12  (wait, that's 12)

Actually:
    δpos  (3)  indices 0:3
    δvel  (3)  indices 3:6
    δbg   (3)  indices 6:9
    δba   (3)  indices 9:12   → 12 states

Revised to 12 states with accel bias included per user request.

Frame convention
────────────────
• Accelerometer reads specific force (proper acceleration) in body frame.
• Gravity is in the NED frame.
• kinematic_accel_ned = R(q_mahony) · f_body_corrected + gravity_ned
  All operations on the same (NED) side before integration.

Measurement model
─────────────────
• Altitude:  z_alt = upᵀ · pos          (scalar)
• Velocity:  z_vel = vel                  (3-vector, NED frame)
"""
import numpy as np
import logging
from scipy.spatial.transform import Rotation as R  # type: ignore
import src.config as config

logger = logging.getLogger(__name__)


class ErrorStateEKF:
    """
    12-state error-state Kalman filter: pos, vel, gyro_bias, accel_bias.

    The attitude quaternion is **not** part of the state — it comes from
    the external Mahony complementary filter via ``predict()``.

    Public interface
    ────────────────
    predict(f_body, omega_body, attitude, gravity_ned, dt)
    update(noisy_alt, noisy_vel) → state_vector (6,)
    get_state()   → (6,) [x,y,z,vx,vy,vz]
    get_gyro_bias()  → (3,)
    get_accel_bias() → (3,)
    get_innovation_norm() → float
    """

    def __init__(
        self,
        initial_pos: np.ndarray,
        initial_vel: np.ndarray,
        initial_covariance: np.ndarray,
        up_vector: np.ndarray = np.array([0.0, 0.0, 1.0]),
    ) -> None:
        self.pos: np.ndarray = initial_pos.copy()
        self.vel: np.ndarray = initial_vel.copy()
        self.bg: np.ndarray = np.zeros(3)
        self.ba: np.ndarray = np.zeros(3)

        self.up_vector: np.ndarray = up_vector / np.linalg.norm(up_vector)

        assert initial_covariance.shape == (12, 12), (
            f"initial_covariance must be (12,12), got {initial_covariance.shape}"
        )
        self.P: np.ndarray = initial_covariance.copy()

        self.sigma_gyro: float = config.SIGMA_GYRO
        self.sigma_accel: float = config.SIGMA_ACCEL
        self.sigma_bg: float = config.GYRO_BIAS_INSTABILITY
        self.sigma_ba: float = config.ACCEL_BIAS_INSTABILITY
        self.thrust_coef: float = config.PROCESS_NOISE_THRUST_COEF

        self.sigma_alt: float = config.SIGMA_ALT
        self.sigma_vel: float = config.SIGMA_VEL

        self._last_innovation: np.ndarray = np.zeros(4)
        self._last_innovation_norm: float = 0.0
        self._last_nis: float = 0.0

        logger.info("Initialized ErrorStateEKF (12-state: pos+vel+bg+ba)")

    # ════════════════════════════════════════════════════════════════
    #  PREDICTION
    # ════════════════════════════════════════════════════════════════

    def predict(
        self,
        f_body: np.ndarray,
        omega_body: np.ndarray,
        attitude: np.ndarray,
        gravity_ned: np.ndarray,
        dt: float,
    ) -> None:
        """
        Propagate nominal state and error-state covariance.

        Parameters
        ----------
        f_body : ndarray (3,)
            Raw specific-force measurement in **body frame** [m/s²].
        omega_body : ndarray (3,)
            Raw angular-rate measurement in **body frame** [rad/s].
        attitude : ndarray (4,)
            Mahony-estimated quaternion [x,y,z,w] (body→NED).
        gravity_ned : ndarray (3,)
            Gravitational acceleration in the **NED frame** [m/s²].
        dt : float
            Time step [s].  Must be > 0.
        """
        if dt <= 0.0:
            return

        # ── 1. Correct IMU readings with current bias estimates ─────
        omega_corr: np.ndarray = omega_body - self.bg
        f_corr_body: np.ndarray = f_body - self.ba

        # ── 2. Rotate corrected specific force to NED frame ────────
        #    FRAME SAFETY: f_corr is body-frame → rotate to NED
        #    BEFORE adding gravity (which is NED-frame).
        rot_bw: R = R.from_quat(attitude)
        f_corr_ned: np.ndarray = rot_bw.apply(f_corr_body)

        # ── 3. Kinematic acceleration in NED frame ─────────────────
        a_ned: np.ndarray = f_corr_ned + gravity_ned

        # ── 4. Propagate nominal state ─────────────────────────────
        self.pos = self.pos + self.vel * dt + 0.5 * a_ned * dt**2
        self.vel = self.vel + a_ned * dt
        # bg and ba drift as random walks — no deterministic change

        # ── 5. State-transition Jacobian F (12×12) ─────────────────
        F: np.ndarray = self._build_F(rot_bw, dt, f_corr_body=f_corr_body)

        # ── 6. Process-noise covariance Q (12×12) ──────────────────
        Q: np.ndarray = self._build_Q(dt, a_ned)

        # ── 7. Propagate error-state covariance ────────────────────
        self.P = F @ self.P @ F.T + Q

    # ════════════════════════════════════════════════════════════════
    #  UPDATE
    # ════════════════════════════════════════════════════════════════

    def update(
        self,
        noisy_alt: float,
        noisy_vel: np.ndarray,
    ) -> np.ndarray:
        """
        Fuse altimeter + velocimeter into the error state.

        Returns
        -------
        state_vector : ndarray (6,)
            [x, y, z, vx, vy, vz] — compatible with guidance/FDI.
        """
        z: np.ndarray = np.array([
            noisy_alt,
            noisy_vel[0],
            noisy_vel[1],
            noisy_vel[2],
        ])

        z_hat: np.ndarray = np.array([
            float(np.dot(self.up_vector, self.pos)),
            self.vel[0],
            self.vel[1],
            self.vel[2],
        ])

        # H (4×12): measurements only observe pos and vel
        H: np.ndarray = np.zeros((4, 12))
        H[0, 0:3] = self.up_vector
        H[1, 3] = 1.0
        H[2, 4] = 1.0
        H[3, 5] = 1.0

        R_mat: np.ndarray = np.diag([
            self.sigma_alt**2,
            self.sigma_vel**2,
            self.sigma_vel**2,
            self.sigma_vel**2,
        ])

        innov: np.ndarray = z - z_hat
        self._last_innovation = innov.copy()
        self._last_innovation_norm = float(np.linalg.norm(innov))

        S: np.ndarray = H @ self.P @ H.T + R_mat
        try:
            K: np.ndarray = np.linalg.solve(S, H @ self.P.T).T
        except np.linalg.LinAlgError:
            logger.warning("EKF update: singular innovation covariance, skipping")
            self._last_nis = 0.0
            return self._pack_state_vector()

        # Normalised Innovation Squared — use Cholesky-style solve for stability
        nis_arg: np.ndarray = np.linalg.solve(S, innov)
        self._last_nis = float(innov @ nis_arg)

        dx: np.ndarray = K @ innov

        self.pos += dx[0:3]
        self.vel += dx[3:6]
        self.bg += dx[6:9]
        self.ba += dx[9:12]

        I12: np.ndarray = np.eye(12)
        self.P = (I12 - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)

        return self._pack_state_vector()

    # ════════════════════════════════════════════════════════════════
    #  ACCESSORS
    # ════════════════════════════════════════════════════════════════

    def get_state(self) -> np.ndarray:
        """Return (6,) [x,y,z,vx,vy,vz] for backward compatibility."""
        return self._pack_state_vector()

    def get_gyro_bias(self) -> np.ndarray:
        """Return (3,) estimated gyroscope bias [rad/s]."""
        return self.bg.copy()

    def get_accel_bias(self) -> np.ndarray:
        """Return (3,) estimated accelerometer bias [m/s²]."""
        return self.ba.copy()

    def get_innovation_norm(self) -> float:
        """Return the L2 norm of the last measurement innovation."""
        return self._last_innovation_norm

    def get_innovation(self) -> np.ndarray:
        """Return the last measurement innovation (4,)."""
        return self._last_innovation.copy()

    def get_last_nis(self) -> float:
        """Return the most recent Normalised Innovation Squared."""
        return self._last_nis

    # ════════════════════════════════════════════════════════════════
    #  INTERNAL
    # ════════════════════════════════════════════════════════════════

    def _pack_state_vector(self) -> np.ndarray:
        return np.concatenate([self.pos, self.vel])

    def _build_F(
        self, rot_bw: R, dt: float, f_corr_body: np.ndarray | None = None,
    ) -> np.ndarray:
        """
        12×12 error-state Jacobian.

        ┌────────┬────────┬──────────────────┬────────┐
        │  I₃    │  I₃·dt │        0         │   0    │  δpos
        │  0     │  I₃    │  −R·[f]×·dt²     │ −R·dt  │  δvel
        │  0     │  0     │        I₃        │   0    │  δbg
        │  0     │  0     │        0         │   I₃   │  δba
        └────────┴────────┴──────────────────┴────────┘

        Gyro bias → velocity coupling:
          δbg → integrated attitude error −δbg·dt → misrotated specific force
          → wrong a_ned → δvel = −R·[f_corr]×·δbg·dt²
        """
        F: np.ndarray = np.eye(12)

        F[0:3, 3:6] = np.eye(3) * dt

        R_mat: np.ndarray = rot_bw.as_matrix()

        # Accel bias is in body frame; when it's wrong, the rotated
        # specific force in NED frame is wrong by approximately
        # −R·δba, which propagates to velocity as −R·δba·dt.
        F[3:6, 9:12] = -R_mat * dt

        # Gyro bias → velocity: attitude error from bg accumulates as
        # δθ = −δbg·dt, which rotates f_corr_body by δθ×f_corr_body.
        # The NED-frame velocity error is R·(δθ×f_corr)·dt.
        if f_corr_body is not None:
            fx, fy, fz = f_corr_body
            f_skew: np.ndarray = np.array([
                [0.0, -fz,  fy],
                [fz,  0.0, -fx],
                [-fy,  fx,  0.0],
            ])
            F[3:6, 6:9] = -R_mat @ f_skew * (dt ** 2)

        return F

    def _build_Q(self, dt: float, a_ned: np.ndarray) -> np.ndarray:
        """
        12×12 process-noise covariance.

        • Velocity noise driven by accelerometer white noise,
          adaptively scaled by thrust magnitude.
        • Gyro bias drift modelled as random walk.
        • Accel bias drift modelled as random walk.
        """
        Q: np.ndarray = np.zeros((12, 12))

        accel_norm: float = float(np.linalg.norm(a_ned))
        vel_scale: float = 1.0 + self.thrust_coef * accel_norm**2
        Q[3:6, 3:6] = np.eye(3) * (self.sigma_accel**2 * dt * vel_scale)

        Q[6:9, 6:9] = np.eye(3) * (self.sigma_bg**2 * dt)

        Q[9:12, 9:12] = np.eye(3) * (self.sigma_ba**2 * dt)

        return Q
