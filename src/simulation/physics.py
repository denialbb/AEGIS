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

class DigitalTwin:
    def __init__(self, env: EnvironmentModel, vessel: VesselModel, initial_state: PhysicsState):
        self.env = env
        self.vessel = vessel
        self.state = initial_state
        self.failed_engines: set[int] = set()
        self.landed: bool = False
        
    def kill_engine(self, engine_index: int) -> None:
        """Simulates a catastrophic discrete failure."""
        if 0 <= engine_index < len(self.vessel.engines):
            self.failed_engines.add(engine_index)
            logger.info(f"[SIM] Engine {engine_index} catastrophically failed!")
        
    def step(self, commanded_throttles: np.ndarray, commanded_gimbals: np.ndarray, dt: float) -> PhysicsState:
        """
        Advances the simulation by dt using fixed-step RK4.
        """
        if self.landed:
            return self.state

        # Enforce physical clamping and log warnings
        clamped_throttles = np.clip(commanded_throttles, 0.0, 1.0)
        if not np.allclose(commanded_throttles, clamped_throttles):
            logger.warning("[SIM] Commanded throttles out of physical bounds! Clamping applied.")
            
        # RK4 Integration
        deriv1 = self._compute_derivatives(self.state, clamped_throttles, commanded_gimbals)
        
        # State k2
        pos_k2 = self.state.pos + 0.5 * dt * deriv1["pos"]
        vel_k2 = self.state.vel + 0.5 * dt * deriv1["vel"]
        q_k2 = self.state.q + 0.5 * dt * deriv1["q"]
        omega_k2 = self.state.omega + 0.5 * dt * deriv1["omega"]
        fuel_k2 = max(0.0, self.state.fuel_mass + 0.5 * dt * deriv1["fuel_mass"])
        throttles_k2 = np.clip(self.state.throttles + 0.5 * dt * deriv1["throttles"], 0.0, 1.0)
        q_norm2 = np.linalg.norm(q_k2)
        if q_norm2 > 1e-12:
            q_k2 /= q_norm2
        state_k2 = PhysicsState(self.state.time + 0.5 * dt, pos_k2, vel_k2, q_k2, omega_k2, fuel_k2, throttles_k2)
        
        deriv2 = self._compute_derivatives(state_k2, clamped_throttles, commanded_gimbals)
        
        # State k3
        pos_k3 = self.state.pos + 0.5 * dt * deriv2["pos"]
        vel_k3 = self.state.vel + 0.5 * dt * deriv2["vel"]
        q_k3 = self.state.q + 0.5 * dt * deriv2["q"]
        omega_k3 = self.state.omega + 0.5 * dt * deriv2["omega"]
        fuel_k3 = max(0.0, self.state.fuel_mass + 0.5 * dt * deriv2["fuel_mass"])
        throttles_k3 = np.clip(self.state.throttles + 0.5 * dt * deriv2["throttles"], 0.0, 1.0)
        q_norm3 = np.linalg.norm(q_k3)
        if q_norm3 > 1e-12:
            q_k3 /= q_norm3
        state_k3 = PhysicsState(self.state.time + 0.5 * dt, pos_k3, vel_k3, q_k3, omega_k3, fuel_k3, throttles_k3)
        
        deriv3 = self._compute_derivatives(state_k3, clamped_throttles, commanded_gimbals)
        
        # State k4
        pos_k4 = self.state.pos + dt * deriv3["pos"]
        vel_k4 = self.state.vel + dt * deriv3["vel"]
        q_k4 = self.state.q + dt * deriv3["q"]
        omega_k4 = self.state.omega + dt * deriv3["omega"]
        fuel_k4 = max(0.0, self.state.fuel_mass + dt * deriv3["fuel_mass"])
        throttles_k4 = np.clip(self.state.throttles + dt * deriv3["throttles"], 0.0, 1.0)
        q_norm4 = np.linalg.norm(q_k4)
        if q_norm4 > 1e-12:
            q_k4 /= q_norm4
        state_k4 = PhysicsState(self.state.time + dt, pos_k4, vel_k4, q_k4, omega_k4, fuel_k4, throttles_k4)
        
        deriv4 = self._compute_derivatives(state_k4, clamped_throttles, commanded_gimbals)
        
        # Update state
        new_time = self.state.time + dt
        new_pos = self.state.pos + (dt / 6.0) * (deriv1["pos"] + 2.0 * deriv2["pos"] + 2.0 * deriv3["pos"] + deriv4["pos"])
        new_vel = self.state.vel + (dt / 6.0) * (deriv1["vel"] + 2.0 * deriv2["vel"] + 2.0 * deriv3["vel"] + deriv4["vel"])
        new_q = self.state.q + (dt / 6.0) * (deriv1["q"] + 2.0 * deriv2["q"] + 2.0 * deriv3["q"] + deriv4["q"])
        new_omega = self.state.omega + (dt / 6.0) * (deriv1["omega"] + 2.0 * deriv2["omega"] + 2.0 * deriv3["omega"] + deriv4["omega"])
        new_fuel_mass = max(0.0, self.state.fuel_mass + (dt / 6.0) * (deriv1["fuel_mass"] + 2.0 * deriv2["fuel_mass"] + 2.0 * deriv3["fuel_mass"] + deriv4["fuel_mass"]))
        new_throttles = np.clip(self.state.throttles + (dt / 6.0) * (deriv1["throttles"] + 2.0 * deriv2["throttles"] + 2.0 * deriv3["throttles"] + deriv4["throttles"]), 0.0, 1.0)
        
        # Normalize attitude
        q_norm = np.linalg.norm(new_q)
        if q_norm > 1e-12:
            new_q /= q_norm
            
        self.state = PhysicsState(
            time=new_time,
            pos=new_pos,
            vel=new_vel,
            q=new_q,
            omega=new_omega,
            fuel_mass=new_fuel_mass,
            throttles=new_throttles
        )
        
        # Check for ground interaction (Altitude is -Z in NED)
        current_com = self.vessel.get_com_position(self.state.fuel_mass)
        
        # The geometric origin (0,0,0) is the bottom of the vessel.
        # Vector from CoM to bottom is -current_com.
        bottom_offset = R.from_quat(self.state.q).apply(-current_com)
        bottom_z = self.state.pos[2] + bottom_offset[2]
        altitude = -bottom_z
        
        if altitude <= 0.0:
            self.landed = True
            self.state.pos[2] = -bottom_offset[2]  # Snap bottom to pad
            self.state.vel = np.zeros(3)  # Halt movement upon touchdown
            self.state.omega = np.zeros(3)
            logger.info(f"[SIM] Touchdown/Terminal state reached at t={self.state.time:.2f}s")
            
        return self.state
        
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
        I_inv = np.linalg.inv(I)
        d_omega = I_inv @ (torque_total_body - np.cross(state.omega, I @ state.omega))
        
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
