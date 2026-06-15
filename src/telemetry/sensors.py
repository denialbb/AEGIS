import numpy as np
import logging
from typing import Tuple, Any
import src.config as config
from scipy.spatial.transform import Rotation as R  # type: ignore

logger = logging.getLogger(__name__)

class SensorModels:
    """
    Wraps kRPC telemetry streams and injects synthetic Gaussian noise into the measurements.
    This separates the perfect simulation data from the estimator, satisfying the noise-driven
    simulation philosophy.
    """
    def __init__(self, conn: Any, vessel: Any, ref_frame: Any, up_vector: np.ndarray):
        """
        Initializes high-frequency data streams.
        conn: kRPC connection object
        vessel: active vessel object
        ref_frame: the custom landing pad reference frame
        up_vector: (3,) normalized vector pointing away from the center of the celestial body in the reference frame
        """
        self.conn = conn
        self.vessel = vessel
        self.ref_frame = ref_frame
        self.up_vector = up_vector
        
        # Initialize kRPC streams
        flight_world = self.vessel.flight(self.ref_frame)
        flight_body = self.vessel.flight(self.vessel.reference_frame)
        
        self.altitude_stream = self.conn.add_stream(getattr, flight_world, 'surface_altitude')
        
        # KSP doesn't provide a direct acceleration stream, so we stream velocity and UT to differentiate
        self.velocity_stream = self.conn.add_stream(getattr, flight_world, 'velocity')
        self.ut_stream = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
        self.last_vel: np.ndarray | None = None
        self.last_ut: float | None = None
        
        # Attitude (quaternion) in the target reference frame
        self.attitude_stream = self.conn.add_stream(getattr, flight_world, 'rotation')
        
        # Aerodynamic force in the target reference frame
        self.aero_stream = self.conn.add_stream(getattr, flight_world, 'aerodynamic_force')
        
        # Mass stream
        self.mass_stream = self.conn.add_stream(getattr, self.vessel, 'mass')
        
        # Situation stream
        self.situation_stream = self.conn.add_stream(getattr, self.vessel, 'situation')
        
        # Angular velocity stream (body rates) expressed in the mission reference frame
        # This provides rotation rates relative to the landing‑pad frame rather than the vessel's own frame.
        # Use a lambda to capture the reference frame argument for angular_velocity
        # Attempt to stream angular velocity with reference frame (real kRPC usage).
        # If the vessel method requires a reference frame argument, pass it.
        # For testing mocks that expose angular_velocity without arguments, fall back to getattr.
        try:
            self.angular_velocity_stream = self.conn.add_stream(self.vessel.angular_velocity, self.ref_frame)
        except TypeError:
            # Fallback for mocks (tests) that expect a simple attribute access.
            self.angular_velocity_stream = self.conn.add_stream(getattr, self.vessel, 'angular_velocity')
        
        # Noise parameters (Standard Deviations) from config
        self.sigma_alt = config.SIGMA_ALT
        self.sigma_accel = config.SIGMA_ACCEL
        self.sigma_vel = config.SIGMA_VEL
        
        # Isolated RNG for determinism
        self.rng = np.random.default_rng(config.RANDOM_SEED)
        
        logger.info(f"Initialized SensorModels with sigma_alt={self.sigma_alt}, sigma_accel={self.sigma_accel}, sigma_vel={self.sigma_vel}")

    def poll(self) -> Tuple[float, np.ndarray, np.ndarray, float, np.ndarray, str, np.ndarray, np.ndarray]:
        """
        Samples the streams and applies zero-mean Gaussian noise.
        Returns:
            noisy_alt (float)
            noisy_accel (np.ndarray shape (3,))
            attitude (np.ndarray shape (4,))
            mass (float)
            aero_force_body (np.ndarray shape (3,))
            situation (str)
            angular_velocity (np.ndarray shape (3,)) body-frame angular rates (rad/s)
            noisy_vel (np.ndarray shape (3,)) world-frame velocity in m/s
        """
        # Read perfect data
        perfect_alt = self.altitude_stream()
        
        ut = self.ut_stream()
        vel = np.array(self.velocity_stream())
        
        if self.last_vel is None:
            perfect_accel = np.zeros(3)
        else:
            dt = ut - self.last_ut
            if dt > 0:
                perfect_accel = (vel - self.last_vel) / dt
            else:
                perfect_accel = np.zeros(3)
                
        self.last_vel = vel
        self.last_ut = ut
        attitude = np.array(self.attitude_stream())
        mass = self.mass_stream()
        aero_world = np.array(self.aero_stream())
        situation = self.situation_stream().name
        
        # KSP coordinate acceleration includes gravity when landed, but is g when falling.
        # Proper acceleration (what an IMU measures and what engines produce) requires counteracting the gravity vector.
        gravity_world = -self.up_vector * 9.81  # Simplified constant gravity aligned with up_vector
        perfect_proper_accel = perfect_accel - gravity_world

        # Rotate world proper acceleration to body frame
        # attitude is [x, y, z, w] quaternion from kRPC (which matches scipy)
        rot = R.from_quat(attitude)
        perfect_accel_body = rot.inv().apply(perfect_proper_accel)
        
        # Inject Gaussian Noise
        noisy_alt = float(perfect_alt + self.rng.normal(0, self.sigma_alt))
        noisy_accel = perfect_accel_body + self.rng.normal(0, self.sigma_accel, size=3)
        noisy_vel = vel + self.rng.normal(0, self.sigma_vel, size=3)
        
        # Rotate aero force to body frame
        aero_body = rot.inv().apply(aero_world)
        
        # Retrieve angular velocity (body rates) in the mission reference frame
        av_raw = self.angular_velocity_stream()
        if hasattr(av_raw, "x"):
            angular_velocity = np.array([av_raw.x, av_raw.y, av_raw.z])
        else:
            # Assume iterable of three numbers
            angular_velocity = np.array(av_raw, dtype=float)
        # Debug log the angular velocity components
        logger.debug(
            f"Angular velocity (frame): x={angular_velocity[0]:.3f}, y={angular_velocity[1]:.3f}, z={angular_velocity[2]:.3f}"
        )

        
        return noisy_alt, noisy_accel, attitude, float(mass), aero_body, situation, angular_velocity, noisy_vel
