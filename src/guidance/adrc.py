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
                       angular_velocity: np.ndarray | None = None) -> np.ndarray:
        """
        Compute ADRC torque command using ESO + WSEF control law.

        For each axis:
          1. Update ESO with current measurement y = err_axis[i] and previous
             control u = prev_u[i].
          2. WSEF control law (reference r=0, r_dot=0):
               e1 = 0 - z1
               vel = angular_velocity[i] if provided, else z2
               u0 = kp * e1 - kd * vel
               u  = u0 - z3 / b0

        Args:
            err_axis: (3,) Angular error in body frame (cross product of
                      nose vector and target-up vector).
            angular_velocity: (3,) Body-frame angular velocity (rad/s).
                              When provided, used directly as the velocity
                              term in the WSEF law instead of z2.

        Returns:
            torque: (3,) Commanded torque per axis (body frame).

        Raises:
            ValueError: If err_axis doesn't have shape (3,).
        """
        if err_axis.shape != (3,):
            raise ValueError(f"err_axis must have shape (3,), got {err_axis.shape}")

        torque = np.zeros(3)

        for i in range(3):
            self.eso[i].update(err_axis[i], self.prev_u[i])

            z1 = self.eso[i].z1
            z2 = self.eso[i].z2
            z3 = self.eso[i].z3

            vel = angular_velocity[i] if angular_velocity is not None else z2

            u0 = self.kp[i] * (0.0 - z1) - self.kd[i] * vel
            torque[i] = u0 - z3 / self.eso[i].b0

        self.prev_u = torque.copy()
        return torque
