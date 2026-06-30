"""
Core fixed-step RK4 physics integrator for the Digital Twin.
"""
import logging
from dataclasses import dataclass
from typing import TypedDict
import numpy as np
from scipy.spatial.transform import Rotation as R
from src.simulation.environment import EnvironmentModel
from src.simulation.vessel import VesselModel

class PhysicsDerivatives(TypedDict):
    pos: np.ndarray
    vel: np.ndarray
    q: np.ndarray
    omega: np.ndarray
    fuel_mass: float
    throttles: np.ndarray

logger = logging.getLogger(__name__)

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

@dataclass
class PhysicsState:
    """The canonical truth state vector for the Digital Twin."""
    time: float                     # Simulation time [s]
    pos: np.ndarray                 # Shape (3,) NED Position [m]
    vel: np.ndarray                 # Shape (3,) NED Velocity [m/s]
    q: np.ndarray                   # Shape (4,) Attitude quaternion [x, y, z, w]
    omega: np.ndarray               # Shape (3,) Body rates [rad/s]
    fuel_mass: float                # Current fuel mass [kg]
    throttles: np.ndarray           # Shape (N,) Actual engine throttles [0.0, 1.0]

    def __post_init__(self) -> None:
        """Validate PhysicsState invariants. Raises ValueError on bad inputs.

        Note: this is a structural / invariant check only. It does not verify
        physical reasonableness of altitude (pos[2] may be any finite value —
        NED altitude is unbounded above the pad) or energy conservation.
        """
        if not np.isfinite(self.time):
            raise ValueError(f"PhysicsState.time must be finite, got {self.time!r}")

        pos = np.asarray(self.pos)
        if pos.ndim != 1 or pos.shape[0] != 3 or not np.all(np.isfinite(pos)):
            raise ValueError(
                f"PhysicsState.pos must be a finite 1D array of length 3, got shape {pos.shape!r}"
            )

        vel = np.asarray(self.vel)
        if vel.ndim != 1 or vel.shape[0] != 3 or not np.all(np.isfinite(vel)):
            raise ValueError(
                f"PhysicsState.vel must be a finite 1D array of length 3, got shape {vel.shape!r}"
            )

        omega = np.asarray(self.omega)
        if omega.ndim != 1 or omega.shape[0] != 3 or not np.all(np.isfinite(omega)):
            raise ValueError(
                f"PhysicsState.omega must be a finite 1D array of length 3, got shape {omega.shape!r}"
            )

        q = np.asarray(self.q)
        if q.ndim != 1 or q.shape[0] != 4 or not np.all(np.isfinite(q)):
            raise ValueError(
                f"PhysicsState.q must be a finite 1D array of length 4, got shape {q.shape!r}"
            )
        q_norm = float(np.linalg.norm(q))
        if q_norm <= 1e-9:
            raise ValueError(
                f"PhysicsState.q must have norm > 1e-9, got {q_norm!r}"
            )

        throttles = np.asarray(self.throttles)
        if throttles.ndim != 1 or not np.all(np.isfinite(throttles)):
            raise ValueError(
                f"PhysicsState.throttles must be a finite 1D array, got shape {throttles.shape!r}"
            )
        if not np.all((throttles >= 0.0) & (throttles <= 1.0)):
            raise ValueError(
                f"PhysicsState.throttles values must lie in [0.0, 1.0], got {self.throttles!r}"
            )

        if not np.isfinite(self.fuel_mass) or self.fuel_mass < 0.0:
            raise ValueError(
                f"PhysicsState.fuel_mass must be finite and >= 0, got {self.fuel_mass!r}"
            )


def _euler_advance(
    base: PhysicsState,
    deriv: PhysicsDerivatives,
    dt_substep: float,
) -> PhysicsState:
    """Forward-Euler advance of `base` by `deriv * dt_substep` for an RK4 stage.

    Re-normalizes the quaternion (RK4 / Euler do not preserve unit norm
    exactly) and clips `fuel_mass` to `>= 0` and `throttles` to `[0, 1]`
    (RK4 may produce small over/undershoots). Position, velocity, and
    body rates are unbounded by the integrator.

    Args:
        base: The state at the start of the substep.
        deriv: The derivative evaluated at `base` (or a propagated stage).
        dt_substep: Time to advance. Used both for state propagation and
            as the timestamp of the returned state.

    Returns:
        A new `PhysicsState` at `base.time + dt_substep` with the
        quaternion normalized and the fuel/throttles clipped.
    """
    new_pos = base.pos + dt_substep * deriv["pos"]
    new_vel = base.vel + dt_substep * deriv["vel"]
    new_q = base.q + dt_substep * deriv["q"]
    new_omega = base.omega + dt_substep * deriv["omega"]
    new_fuel = max(0.0, base.fuel_mass + dt_substep * deriv["fuel_mass"])
    new_throttles = np.clip(
        base.throttles + dt_substep * deriv["throttles"], 0.0, 1.0
    )
    q_norm = np.linalg.norm(new_q)
    if q_norm > 1e-12:
        new_q /= q_norm
    return PhysicsState(
        time=base.time + dt_substep,
        pos=new_pos,
        vel=new_vel,
        q=new_q,
        omega=new_omega,
        fuel_mass=new_fuel,
        throttles=new_throttles,
    )


def _rk4_combine(
    base: PhysicsState,
    d1: PhysicsDerivatives,
    d2: PhysicsDerivatives,
    d3: PhysicsDerivatives,
    d4: PhysicsDerivatives,
    dt: float,
) -> PhysicsState:
    """Weighted 4th-order Runge-Kutta combination of four derivative evaluations.

    The returned state is at `base.time + dt` with the same physical
    clipping as `_euler_advance` (quaternion normalized, fuel >= 0,
    throttles in [0, 1]).
    """
    sixth = dt / 6.0
    new_pos = base.pos + sixth * (
        d1["pos"] + 2.0 * d2["pos"] + 2.0 * d3["pos"] + d4["pos"]
    )
    new_vel = base.vel + sixth * (
        d1["vel"] + 2.0 * d2["vel"] + 2.0 * d3["vel"] + d4["vel"]
    )
    new_q = base.q + sixth * (
        d1["q"] + 2.0 * d2["q"] + 2.0 * d3["q"] + d4["q"]
    )
    new_omega = base.omega + sixth * (
        d1["omega"] + 2.0 * d2["omega"] + 2.0 * d3["omega"] + d4["omega"]
    )
    new_fuel = max(
        0.0,
        base.fuel_mass
        + sixth
        * (
            d1["fuel_mass"]
            + 2.0 * d2["fuel_mass"]
            + 2.0 * d3["fuel_mass"]
            + d4["fuel_mass"]
        ),
    )
    new_throttles = np.clip(
        base.throttles
        + sixth
        * (
            d1["throttles"]
            + 2.0 * d2["throttles"]
            + 2.0 * d3["throttles"]
            + d4["throttles"]
        ),
        0.0,
        1.0,
    )
    q_norm = np.linalg.norm(new_q)
    if q_norm > 1e-12:
        new_q /= q_norm
    return PhysicsState(
        time=base.time + dt,
        pos=new_pos,
        vel=new_vel,
        q=new_q,
        omega=new_omega,
        fuel_mass=new_fuel,
        throttles=new_throttles,
    )


class DigitalTwin:
    """Fixed-step RK4 physics integrator for the Digital Twin.

    Determinism contract: when constructed with the same seed, identical
    commands, and identical `kill_engine` calls produce bit-identical state
    sequences. The seed backs a private `np.random.Generator` (exposed as
    `self.rng`) so stochastic subsystems (FDI noise injection, Gremlin,
    NN-ADRC exploration) can be made reproducible. The integrator itself
    is deterministic; the seed governs anything the caller wires through
    `self.rng`.
    """

    def __init__(
        self,
        env: EnvironmentModel,
        vessel: VesselModel,
        initial_state: PhysicsState,
        seed: int | None = None,
    ) -> None:
        self.env = env
        self.vessel = vessel
        self.state = initial_state
        self.failed_engines: set[int] = set()
        self.landed: bool = False
        self.rng: np.random.Generator = np.random.default_rng(seed)

    def reset(self, initial_state: PhysicsState, keep_failures: bool = False) -> None:
        """Restore the twin to a freshly-constructed state.

        Args:
            initial_state: The state to install as the current self.state.
            keep_failures: If True, retain any previously-failed engines
                (useful for testing recovery from a partial failure). If
                False (default), the failed_engines set is cleared.
        """
        self.state = initial_state
        if not keep_failures:
            self.failed_engines = set()
        self.landed = False

    def kill_engine(self, engine_index: int) -> None:
        """Simulates a catastrophic discrete failure."""
        if 0 <= engine_index < len(self.vessel.engines):
            self.failed_engines.add(engine_index)
            logger.info(f"[SIM] Engine {engine_index} catastrophically failed!")
        
    def step(self, commanded_throttles: np.ndarray, commanded_gimbals: np.ndarray, dt: float) -> PhysicsState:
        """Advance the simulation by `dt` using fixed-step RK4.

        The integrator evaluates derivatives at four stages
        (k1 at the current state, k2/k3 at half-step Euler advances from
        the current state, k4 at a full-step Euler advance) and combines
        them with the standard `[1, 2, 2, 1] / 6` weights. Stage state
        construction is delegated to `_euler_advance`; final combination to
        `_rk4_combine`. See ADR-031.

        No-op if `self.landed` is already set (subsequent calls return the
        frozen terminal state with no time advance).
        """
        if self.landed:
            return self.state

        # Enforce physical clamping and log warnings
        clamped_throttles = np.clip(commanded_throttles, 0.0, 1.0)
        if not np.allclose(commanded_throttles, clamped_throttles):
            logger.warning("[SIM] Commanded throttles out of physical bounds! Clamping applied.")

        # RK4 stages
        half_dt = 0.5 * dt
        deriv1 = self._compute_derivatives(self.state, clamped_throttles, commanded_gimbals)
        state_k2 = _euler_advance(self.state, deriv1, half_dt)
        deriv2 = self._compute_derivatives(state_k2, clamped_throttles, commanded_gimbals)
        state_k3 = _euler_advance(self.state, deriv2, half_dt)
        deriv3 = self._compute_derivatives(state_k3, clamped_throttles, commanded_gimbals)
        state_k4 = _euler_advance(self.state, deriv3, dt)
        deriv4 = self._compute_derivatives(state_k4, clamped_throttles, commanded_gimbals)

        self.state = _rk4_combine(self.state, deriv1, deriv2, deriv3, deriv4, dt)

        # Terminal ground contact
        self._apply_ground_contact()

        return self.state

    def _apply_ground_contact(self) -> None:
        """Snap the vessel to the pad if its bottom is at or below the ground.

        In NED coordinates, altitude is `-pos[2]`. The vessel's geometric
        origin is the bottom of the hull, so the bottom-of-vessel offset
        from the CoM is `-current_com`. We rotate this offset into the
        world frame to find the world-space altitude of the bottom.

        On contact: sets `self.landed = True`, snaps the CoM upward so the
        bottom rests on the pad (`pos[2] = -bottom_offset[2]`), zeros
        translational and rotational velocity. Subsequent `step()` calls
        are no-ops.
        """
        current_com = self.vessel.get_com_position(self.state.fuel_mass)
        bottom_offset = R.from_quat(self.state.q).apply(-current_com)
        altitude = -(self.state.pos[2] + bottom_offset[2])

        if altitude > 0.0:
            return

        self.landed = True
        self.state.pos[2] = -bottom_offset[2]
        self.state.vel = np.zeros(3)
        self.state.omega = np.zeros(3)
        logger.info(f"[SIM] Touchdown/Terminal state reached at t={self.state.time:.2f}s")
        
    def _compute_derivatives(self, state: PhysicsState, cmd_throttles: np.ndarray, cmd_gimbals: np.ndarray) -> PhysicsDerivatives:
        """
        Computes the time derivatives for all state variables.
        Used by the RK4 integrator.
        """
        # 1. Spooling dynamics
        target_throttles = np.copy(cmd_throttles)
        for idx in self.failed_engines:
            target_throttles[idx] = 0.0
            
        d_throttles = (target_throttles - state.throttles) / self.vessel.engine_tau
        
        # 2. Fuel consumption
        if state.fuel_mass > 0.0:
            fuel_burn_rate = self.vessel.get_fuel_burn_rate(state.throttles)
        else:
            fuel_burn_rate = 0.0
        d_fuel_mass = -fuel_burn_rate
        
        # 3. Thrust force and torque in body frame
        f_thrust_body = np.zeros(3)
        torque_total_body = np.zeros(3)
        
        current_com = self.vessel.get_com_position(state.fuel_mass)
        
        if state.fuel_mass > 0.0:
            for i, engine in enumerate(self.vessel.engines):
                # Clamp gimbals to physical limits
                max_rad = np.deg2rad(engine.max_gimbal_deg)
                gx = np.clip(cmd_gimbals[i, 0], -max_rad, max_rad) if engine.max_gimbal_deg > 0.0 else 0.0
                gy = np.clip(cmd_gimbals[i, 1], -max_rad, max_rad) if engine.max_gimbal_deg > 0.0 else 0.0
                
                f_dir = (engine.thrust_direction
                         + gx * engine.gimbal_y_axis
                         - gy * engine.gimbal_x_axis)
                f_norm = np.linalg.norm(f_dir)
                if f_norm > 1e-12:
                    f_dir = f_dir / f_norm
                    
                axial_thrust = state.throttles[i] * engine.max_thrust
                f_body_i = axial_thrust * f_dir
                
                f_thrust_body += f_body_i
                lever_arm = engine.position - current_com
                torque_total_body += np.cross(lever_arm, f_body_i)
                
        # 4. Translational acceleration in NED
        vessel_mass = self.vessel.total_mass(state.fuel_mass)
        rot_bw = R.from_quat(state.q)
        f_thrust_ned = rot_bw.apply(f_thrust_body)
        
        altitude = -state.pos[2]
        g_val = self.env.gravity(altitude)
        f_grav_ned = np.array([0.0, 0.0, vessel_mass * g_val])
        f_drag_ned = self.vessel.get_drag_force(state.vel, self.env.air_density(altitude))
        
        f_total_ned = f_thrust_ned + f_grav_ned + f_drag_ned
        accel_ned = f_total_ned / vessel_mass
        
        # 5. Rotational acceleration in body frame (Euler's equations)
        I = self.vessel.inertia_tensor(state.fuel_mass)
        d_omega = np.linalg.solve(I, torque_total_body - np.cross(state.omega, I @ state.omega))
        
        # 6. Quaternion derivative
        omega_quat = np.array([0.5 * state.omega[0], 0.5 * state.omega[1], 0.5 * state.omega[2], 0.0])
        dq = _quat_mul(state.q, omega_quat)
        
        return {
            "pos": state.vel,
            "vel": accel_ned,
            "q": dq,
            "omega": d_omega,
            "fuel_mass": d_fuel_mass,
            "throttles": d_throttles
        }
