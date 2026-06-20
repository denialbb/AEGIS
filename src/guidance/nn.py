import numpy as np
import logging
from typing import Any, Literal

logger = logging.getLogger(__name__)

_N_HIDDEN = 20
_N_INPUT = 9
_N_OUTPUT = 3
_DEFAULT_CLAMP = 5.0


def _relu(x: np.ndarray) -> np.ndarray:
    return np.maximum(0.0, x)


def _glorot_uniform(fan_in: int, fan_out: int) -> np.ndarray:
    limit = np.sqrt(6.0 / (fan_in + fan_out))
    return np.random.uniform(-limit, limit, (fan_in, fan_out))


class NNFeedforward:
    """
    Feedforward neural network for learned disturbance compensation.

    Architecture: 9 inputs (3 axes x [err, omega, z3]) → 20 → 20 → 3 outputs
    Hidden layers use ReLU (poslin) activation. Output layer is linear.

    The network outputs a correction in acceleration space (rad/s^2)
    that is added to the CTM base PD acceleration command. At runtime,
    the correction is clamped to ``[-clamp, +clamp]`` rad/s^2 per axis
    and multiplied by the inertia tensor to produce a torque feedforward.

    Input layout::
        [err_x, err_y, err_z,
         omega_x, omega_y, omega_z,
         z3_x, z3_y, z3_z]

    Output::
        [delta_rr_x, delta_rr_y, delta_rr_z]  (rad/s^2)

    Training target: J^{-1} @ disturbance_torque.
    The network learns to predict the total disturbance acceleration
    from the current state, providing anticipatory feedforward that
    the ESO's z3 estimate complements in real time.
    """

    def __init__(self,
                 W1: np.ndarray | None = None,
                 b1: np.ndarray | None = None,
                 W2: np.ndarray | None = None,
                 b2: np.ndarray | None = None,
                 W3: np.ndarray | None = None,
                 b3: np.ndarray | None = None,
                 clamp: float = _DEFAULT_CLAMP):
        if W1 is not None:
            self.W1 = np.array(W1, dtype=float)
            self.b1 = np.array(b1, dtype=float)
            self.W2 = np.array(W2, dtype=float)
            self.b2 = np.array(b2, dtype=float)
            self.W3 = np.array(W3, dtype=float)
            self.b3 = np.array(b3, dtype=float)
        else:
            rng = np.random.RandomState(42)
            _old_state = np.random.get_state()
            np.random.set_state(rng.get_state())
            self.W1 = _glorot_uniform(_N_INPUT, _N_HIDDEN)
            self.b1 = np.zeros(_N_HIDDEN)
            self.W2 = _glorot_uniform(_N_HIDDEN, _N_HIDDEN)
            self.b2 = np.zeros(_N_HIDDEN)
            self.W3 = _glorot_uniform(_N_HIDDEN, _N_OUTPUT)
            self.b3 = np.zeros(_N_OUTPUT)
            np.random.set_state(_old_state)

        self.clamp = float(clamp)
        self.is_trained: bool = False

    def predict(self, state: np.ndarray) -> np.ndarray:
        if state.shape != (_N_INPUT,):
            raise ValueError(
                f"NN state must have shape ({_N_INPUT},), "
                f"got {state.shape}"
            )
        h1 = _relu(state @ self.W1 + self.b1)
        h2 = _relu(h1 @ self.W2 + self.b2)
        out = h2 @ self.W3 + self.b3
        return np.clip(out, -self.clamp, self.clamp)

    def _pack(self) -> np.ndarray:
        return np.concatenate([
            self.W1.ravel(), self.b1,
            self.W2.ravel(), self.b2,
            self.W3.ravel(), self.b3,
        ])

    def _unpack(self, params: np.ndarray) -> None:
        n1 = _N_INPUT * _N_HIDDEN
        self.W1 = params[:n1].reshape(_N_INPUT, _N_HIDDEN)
        self.b1 = params[n1:n1 + _N_HIDDEN]
        o = n1 + _N_HIDDEN
        n2 = _N_HIDDEN * _N_HIDDEN
        self.W2 = params[o:o + n2].reshape(_N_HIDDEN, _N_HIDDEN)
        self.b2 = params[o + n2:o + n2 + _N_HIDDEN]
        o = o + n2 + _N_HIDDEN
        n3 = _N_HIDDEN * _N_OUTPUT
        self.W3 = params[o:o + n3].reshape(_N_HIDDEN, _N_OUTPUT)
        self.b3 = params[o + n3:]

    def _forward_batch(self, params: np.ndarray,
                       X: np.ndarray) -> np.ndarray:
        n1 = _N_INPUT * _N_HIDDEN
        W1 = params[:n1].reshape(_N_INPUT, _N_HIDDEN)
        b1 = params[n1:n1 + _N_HIDDEN]
        o = n1 + _N_HIDDEN
        n2 = _N_HIDDEN * _N_HIDDEN
        W2 = params[o:o + n2].reshape(_N_HIDDEN, _N_HIDDEN)
        b2 = params[o + n2:o + n2 + _N_HIDDEN]
        o = o + n2 + _N_HIDDEN
        n3 = _N_HIDDEN * _N_OUTPUT
        W3 = params[o:o + n3].reshape(_N_HIDDEN, _N_OUTPUT)
        b3 = params[o + n3:]

        h1 = _relu(X @ W1 + b1)
        h2 = _relu(h1 @ W2 + b2)
        return h2 @ W3 + b3

    def _residual(self, params: np.ndarray,
                  X: np.ndarray, y: np.ndarray) -> np.ndarray:
        pred = self._forward_batch(params, X)
        return (pred - y).ravel()

    def train(self,
              X: np.ndarray,
              y: np.ndarray,
              max_nfev: int = 2000,
              verbose: Literal[0, 1, 2] = 0) -> dict[str, Any]:
        from scipy.optimize import least_squares

        if X.ndim != 2 or X.shape[1] != _N_INPUT:
            raise ValueError(
                f"X must have shape (N, {_N_INPUT}), got {X.shape}"
            )
        if y.ndim != 2 or y.shape[1] != _N_OUTPUT:
            raise ValueError(
                f"y must have shape (N, {_N_OUTPUT}), got {y.shape}"
            )

        x0 = self._pack()
        # Use 'trf' method for small datasets (lm requires more residuals than variables)
        result = least_squares(
            self._residual, x0, args=(X, y),
            method='trf', max_nfev=max_nfev, verbose=verbose,
        )
        self._unpack(result.x)
        self.is_trained = True
        return {
            'success': result.success,
            'cost': result.cost,
            'nfev': result.nfev,
            'njev': getattr(result, 'njev', None),
        }


def generate_training_data(
    inertia_tensor: np.ndarray,
    n_trajectories: int = 5,
    n_steps: int = 500,
    dt: float = 0.02,
    seed: int = 42,
    stab_gains: bool = True,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """
    Generate synthetic training data from a 3-DOF kinematic attitude simulation.

    The simulation runs ``n_trajectories`` independent trajectories, each with
    a different random step-function disturbance profile. Per trajectory:
      1. A random disturbance magnitude and onset time is generated.
      2. The CTM-ADRC controller regulates the attitude under that disturbance.
      3. At each timestep the state ``[err, omega, z3]`` and the target
         ``J^{-1} @ disturbance_torque`` are recorded.

    The returned ``X`` and ``y`` are stacked across all trajectories.

    Args:
        inertia_tensor: (3,3) Inertia tensor for the simulation.
        n_trajectories: Number of independent trajectories to generate.
        n_steps: Steps per trajectory.
        dt: Simulation timestep.
        seed: Random seed for reproducibility.
        stab_gains: If True, use bandwidth-parameterized ESO gains
                    (beta_01=15, beta_02=75, beta_03=125, delta=0.1)
                    for stable 50Hz operation with large CTM feedforward.

    Returns:
        X: (N, 9) training inputs  [err, omega, z3] per axis.
        y: (N, 3) training targets  J^{-1} @ disturbance_torque.
        info: dict with metadata (disturbance profiles used, etc.).
    """
    from src.guidance.adrc import CTMCalculator, ADRCController

    inertia = np.array(inertia_tensor, dtype=float)
    inv_inertia = np.linalg.inv(inertia)

    ctm = CTMCalculator(inertia)

    if stab_gains:
        eso_params = [
            dict(beta_01=15.0, beta_02=75.0, beta_03=125.0,
                 alpha_1=0.5, alpha_2=0.25, delta=0.1, b0=1.0)
            for _ in range(3)
        ]
    else:
        eso_params = None

    adrc = ADRCController(dt=dt, eso_params=eso_params)

    rng = np.random.RandomState(seed)
    X_list: list[np.ndarray] = []
    y_list: list[np.ndarray] = []
    profiles: list[dict[str, float]] = []

    for _ in range(n_trajectories):
        err = np.zeros(3, dtype=float)
        omega = np.zeros(3, dtype=float)
        adrc.reset()

        dist_mag = rng.uniform(10.0, 100.0, 3)
        dist_axis = rng.choice([0, 1, 2])
        dist_start = rng.randint(n_steps // 4, n_steps // 2)

        def disturbance(i: int) -> np.ndarray:
            d = np.zeros(3)
            if i >= dist_start:
                d[dist_axis] = dist_mag[dist_axis]
            return d

        profiles.append({
            'axis': int(dist_axis),
            'magnitude': float(dist_mag[dist_axis]),
            'start_step': int(dist_start),
        })

        traj_X: list[np.ndarray] = []
        traj_y: list[np.ndarray] = []

        for i in range(n_steps):
            z3_before = np.array([eso.z3 for eso in adrc.eso])
            state = np.concatenate([err, omega, z3_before])
            dist = disturbance(i)
            target = inv_inertia @ dist

            ctm_ff = ctm.compute_feedforward(err, omega)
            torque = adrc.compute_torque(
                err, omega, ctm_feedforward=ctm_ff,
            )

            accel = inv_inertia @ (torque + dist - np.cross(
                omega, inertia @ omega,
            ))
            omega += dt * accel
            err += dt * omega

            traj_X.append(state)
            traj_y.append(target)

        X_list.append(np.stack(traj_X, axis=0))
        y_list.append(np.stack(traj_y, axis=0))

    X = np.concatenate(X_list, axis=0)
    y = np.concatenate(y_list, axis=0)

    info = {
        'n_trajectories': n_trajectories,
        'n_steps': n_steps,
        'dt': dt,
        'inertia_tensor': inertia,
        'profiles': profiles,
        'stab_gains': stab_gains,
    }
    return X, y, info
