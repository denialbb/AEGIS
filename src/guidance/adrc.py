import numpy as np
import logging

logger = logging.getLogger(__name__)


def fal(e: float, alpha: float, delta: float) -> float:
    """
    Nonlinear fal() function from Leblebicioglu et al. (gimbal NN-ADRC paper).
    Continuous at |e| = delta.

    Args:
        e: Estimation error.
        alpha: Nonlinearity factor (0 < alpha < 1).
        delta: Linear region width, must be > 0.

    Returns:
        fal(e, alpha, delta)
    """
    if delta <= 0.0:
        raise ValueError(f"fal() requires delta > 0, got {delta}")

    abs_e = abs(e)
    if abs_e <= delta:
        return e / (delta ** (1.0 - alpha))
    else:
        return (abs_e ** alpha) * (1.0 if e >= 0.0 else -1.0)


class PerAxisESO:
    """
    Single-axis Extended State Observer (ESO).

    Discrete-time implementation of the nonlinear ESO from the gimbal paper.
    Tracks three states:
      z1 — estimated output (tracks the measured plant output y)
      z2 — estimated output derivative (tracks y_dot)
      z3 — estimated total disturbance (lumped external + internal disturbance)

    Per-axis parameters: beta_01, beta_02, beta_03 (observer gains),
    alpha_1, alpha_2 (fal nonlinearity exponents), delta (linear region),
    b0 (control-effectiveness scaling).
    """

    def __init__(self,
                 dt: float = 0.02,
                 beta_01: float = 100.0,
                 beta_02: float = 300.0,
                 beta_03: float = 1000.0,
                 alpha_1: float = 0.5,
                 alpha_2: float = 0.25,
                 delta: float = 0.01,
                 b0: float = 1.0):
        if delta <= 0.0:
            raise ValueError(f"PerAxisESO requires delta > 0, got {delta}")
        if dt <= 0.0:
            dt = 1e-6

        self.dt = dt
        self.beta_01 = beta_01
        self.beta_02 = beta_02
        self.beta_03 = beta_03
        self.alpha_1 = alpha_1
        self.alpha_2 = alpha_2
        self.delta = delta
        self.b0 = b0

        self.z1: float = 0.0
        self.z2: float = 0.0
        self.z3: float = 0.0

    def reset(self) -> None:
        """Reset all ESO states to zero."""
        self.z1 = 0.0
        self.z2 = 0.0
        self.z3 = 0.0

    def update(self, y: float, u: float) -> None:
        """
        One discrete-time ESO step.

        ESO difference equations (per the gimbal paper):
          e   = z1 - y
          z1 += dt * (z2 - beta_01 * fal(e, alpha_1, delta))
          z2 += dt * (z3 - beta_02 * fal(e, alpha_1, delta) + b0 * u)
          z3 += dt * (-beta_03 * fal(e, alpha_2, delta))

        Args:
            y: Plant output measurement for this axis (e.g. angular error).
            u: Control input from the previous tick (e.g. torque command).
        """
        e = self.z1 - y
        fal_e1 = fal(e, self.alpha_1, self.delta)
        fal_e2 = fal(e, self.alpha_2, self.delta)

        self.z1 += self.dt * (self.z2 - self.beta_01 * fal_e1)
        self.z2 += self.dt * (self.z3 - self.beta_02 * fal_e1 + self.b0 * u)
        self.z3 += self.dt * (-self.beta_03 * fal_e2)


class CTMCalculator:
    """
    Computed-Torque Method feedforward calculator (Phase 3).

    The CTM provides a model-based feedforward torque based on the vehicle's
    rigid-body dynamics, enabling the CTM-ADRC hybrid architecture from
    Leblebicioglu et al. The CTM captures the nominal plant behavior, and
    the ESO only needs to estimate the residual disturbance.

    Feedforward torque::
        tau_ctm = J @ (kp_ctm * err - kd_ctm * omega) + omega x J @ omega

    This is the inverse-dynamics torque for the desired attitude regulation
    (err -> 0, omega -> 0). When added to the ADRC output, it pre-compensates
    for the known rigid-body dynamics, leaving the ESO to handle only
    unmodeled effects (aero torques, engine failures, fuel slosh).

    See also:
        - FDI's expected_force/mass: the translation analog of this calculator.
        - PHASE_3.md for the CTM-ADRC architecture decision.
    """

    def __init__(self,
                 inertia_tensor: np.ndarray,
                 kp_ctm: np.ndarray | None = None,
                 kd_ctm: np.ndarray | None = None):
        """
        Args:
            inertia_tensor: (3,3) Full inertia tensor in body frame.
            kp_ctm: (3,) CTM proportional gains. If None defaults to [9,9,9]
                    (omega_n=3 rad/s, zeta=1.0: k = omega_n^2 = 9).
            kd_ctm: (3,) CTM derivative gains. If None defaults to [6,6,6]
                    (omega_n=3 rad/s, zeta=1.0: d = 2*zeta*omega_n = 6).

        Raises:
            ValueError: If inertia_tensor does not have shape (3,3).
        """
        self.inertia_tensor = np.array(inertia_tensor, dtype=float)
        if self.inertia_tensor.shape != (3, 3):
            raise ValueError(
                f"inertia_tensor must have shape (3,3), "
                f"got {self.inertia_tensor.shape}"
            )

        self.kp_ctm = np.ones(3) * 9.0 if kp_ctm is None else np.array(kp_ctm, dtype=float)
        self.kd_ctm = np.ones(3) * 6.0 if kd_ctm is None else np.array(kd_ctm, dtype=float)

        if self.kp_ctm.shape != (3,) or self.kd_ctm.shape != (3,):
            raise ValueError(
                f"kp_ctm and kd_ctm must have shape (3,), "
                f"got {self.kp_ctm.shape}, {self.kd_ctm.shape}"
            )

        self._inv_inertia = np.linalg.inv(self.inertia_tensor)

    def compute_feedforward(self,
                             err_axis: np.ndarray,
                             angular_velocity: np.ndarray) -> np.ndarray:
        """
        Compute the CTM feedforward torque for attitude regulation.

        The CTM provides negative-feedback PD (matching the WSEF sign
        convention) plus gyroscopic feedforward::

            tau_ctm = J @ (-kp_ctm * err - kd_ctm * omega) + omega x J @ omega

        The `-kp_ctm * err` term provides negative feedback — positive error
        produces negative torque to reduce the error. This matches the WSEF's
        ``kp * (0 - z1)`` convention where z1 tracks err.

        The gyroscopic term ``omega x J @ omega`` provides model-based
        compensation for known cross-coupling dynamics.

        Args:
            err_axis: (3,) Angular error in body frame (rad).
            angular_velocity: (3,) Body-frame angular velocity (rad/s).

        Returns:
            tau_ctm: (3,) Feedforward torque (N-m) in body frame.

        Raises:
            ValueError: If inputs don't have shape (3,).
        """
        if err_axis.shape != (3,):
            raise ValueError(f"err_axis must have shape (3,), got {err_axis.shape}")
        if angular_velocity.shape != (3,):
            raise ValueError(
                f"angular_velocity must have shape (3,), "
                f"got {angular_velocity.shape}"
            )

        tau_inertia = self.inertia_tensor @ (
            -self.kp_ctm * err_axis - self.kd_ctm * angular_velocity
        )
        tau_gyro = np.cross(
            angular_velocity,
            self.inertia_tensor @ angular_velocity,
        )
        return tau_inertia + tau_gyro

    def expected_angular_accel(self, expected_torque: np.ndarray) -> np.ndarray:
        """
        Convert expected torque to expected angular acceleration.

        alpha_expected = J^{-1} @ tau_expected

        Args:
            expected_torque: (3,) Expected torque from engines (N-m).

        Returns:
            alpha_expected: (3,) Expected angular acceleration (rad/s^2).
        """
        if expected_torque.shape != (3,):
            raise ValueError(
                f"expected_torque must have shape (3,), "
                f"got {expected_torque.shape}"
            )
        return self._inv_inertia @ expected_torque


class ADRCController:
    """
    Active Disturbance Rejection Controller for 3-DOF attitude control (Phase 2).

    Manages three independent PerAxisESO instances (one per body axis: roll, pitch, yaw)
    and applies the WSEF (Weighted-Sum Error Format) control law to compute
    disturbance-rejecting torque commands.

    Flow per tick:
      1. For each axis, update ESO with current error and previous control.
      2. Compute WSEF control law: u = kp*(r - z1) - kd*angular_velocity - z3/b0.
      3. Return 3-DOF torque vector.

    Args:
        dt: Control loop timestep (seconds).
        kp: (3,) Proportional gains for WSEF control law (per axis).
        kd: (3,) Derivative gains for WSEF control law (per axis).
        eso_params: List of 3 dicts, one per axis, each with keys:
            beta_01, beta_02, beta_03, alpha_1, alpha_2, delta, b0.
            If None, default values are used (shared across axes).
    """

    def __init__(self,
                 dt: float = 0.02,
                 kp: np.ndarray | None = None,
                 kd: np.ndarray | None = None,
                 eso_params: list[dict] | None = None):
        self.dt = max(dt, 1e-6)

        self.kp = np.ones(3) if kp is None else np.array(kp, dtype=float)
        self.kd = np.ones(3) if kd is None else np.array(kd, dtype=float)

        if self.kp.shape != (3,) or self.kd.shape != (3,):
            raise ValueError(f"kp and kd must have shape (3,), got {self.kp.shape}, {self.kd.shape}")

        if eso_params is None:
            eso_params = [
                dict(beta_01=100.0, beta_02=300.0, beta_03=1000.0,
                     alpha_1=0.5, alpha_2=0.25, delta=0.01, b0=1.0)
                for _ in range(3)
            ]

        if len(eso_params) != 3:
            raise ValueError(f"eso_params must have length 3 (one per axis), got {len(eso_params)}")

        self.eso: list[PerAxisESO] = []
        for params in eso_params:
            self.eso.append(PerAxisESO(
                dt=self.dt,
                beta_01=params.get('beta_01', 100.0),
                beta_02=params.get('beta_02', 300.0),
                beta_03=params.get('beta_03', 1000.0),
                alpha_1=params.get('alpha_1', 0.5),
                alpha_2=params.get('alpha_2', 0.25),
                delta=params.get('delta', 0.01),
                b0=params.get('b0', 1.0),
            ))

        self.prev_u: np.ndarray = np.zeros(3)

    def reset(self) -> None:
        """Reset all ESO states and stored previous control."""
        for eso in self.eso:
            eso.reset()
        self.prev_u = np.zeros(3)

    def compute_torque(self,
                       err_axis: np.ndarray,
                       angular_velocity: np.ndarray | None = None,
                       ctm_feedforward: np.ndarray | None = None) -> np.ndarray:
        """
        Compute ADRC torque command using ESO + WSEF control law,
        with optional CTM feedforward (Phase 3).

        When CTM feedforward is NOT provided (pure ADRC mode):
          For each axis:
            1. Update ESO with (y=err_axis[i], u=prev_u[i]).
            2. WSEF: u0 = kp * (0 - z1) - kd * vel
            3. Torque = u0 - z3 / b0

        When CTM feedforward IS provided (CTM-ADRC mode):
          The CTM provides the full model-based PD torque, and the ADRC
          provides only disturbance rejection (the ESO's z3 estimate).
          The WSEF kp/kd are NOT applied — they are replaced by the CTM's
          inertia-scaled PD. This avoids double-PD cancellation.
          For each axis:
            1. Update ESO with (y=err_axis[i], u=prev_u[i]).
            2. ADRC output = -z3 / b0  (disturbance rejection only)
            3. Torque = adrc_out + ctm_feedforward[i]

        The prev_u stored for the next tick is the TOTAL torque (ADRC + CTM),
        so the ESO models the full plant response.

        Args:
            err_axis: (3,) Angular error in body frame.
            angular_velocity: (3,) Body-frame angular velocity (rad/s).
                              When provided, used directly as the velocity
                              term in the WSEF law instead of z2.
            ctm_feedforward: (3,) Optional CTM feedforward torque (N-m)
                             from CTMCalculator.compute_feedforward().
                             When provided, WSEF kp/kd are bypassed and
                             only disturbance rejection from -z3/b0 is used.

        Returns:
            torque: (3,) Commanded torque per axis (body frame).

        Raises:
            ValueError: If inputs have invalid shape.
        """
        if err_axis.shape != (3,):
            raise ValueError(f"err_axis must have shape (3,), got {err_axis.shape}")
        if ctm_feedforward is not None and ctm_feedforward.shape != (3,):
            raise ValueError(
                f"ctm_feedforward must have shape (3,), "
                f"got {ctm_feedforward.shape}"
            )

        adrc_out = np.zeros(3)

        for i in range(3):
            self.eso[i].update(err_axis[i], self.prev_u[i])

            z3 = self.eso[i].z3

            if ctm_feedforward is not None:
                # CTM-ADRC mode: CTM provides PD, ADRC provides disturbance rejection
                adrc_out[i] = -z3 / self.eso[i].b0
            else:
                # Pure ADRC mode: WSEF provides PD + disturbance rejection
                z1 = self.eso[i].z1
                vel = angular_velocity[i] if angular_velocity is not None else self.eso[i].z2
                u0 = self.kp[i] * (0.0 - z1) - self.kd[i] * vel
                adrc_out[i] = u0 - z3 / self.eso[i].b0

        if ctm_feedforward is not None:
            total_torque = adrc_out + ctm_feedforward
        else:
            total_torque = adrc_out

        self.prev_u = total_torque.copy()
        return total_torque
