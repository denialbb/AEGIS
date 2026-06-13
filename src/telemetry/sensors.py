import numpy as np
import logging
from typing import Tuple, Any
import src.config as config

logger = logging.getLogger(__name__)

class SensorModels:
    """
    Wraps kRPC telemetry streams and injects synthetic Gaussian noise into the measurements.
    This separates the perfect simulation data from the estimator, satisfying the noise-driven
    simulation philosophy.
    """
    def __init__(self, conn: Any, vessel: Any, ref_frame: Any):
        """
        Initializes high-frequency data streams.
        conn: kRPC connection object
        vessel: active vessel object
        ref_frame: the custom landing pad reference frame
        """
        self.conn = conn
        self.vessel = vessel
        self.ref_frame = ref_frame
        
        # Initialize kRPC streams
        flight_world = self.vessel.flight(self.ref_frame)
        flight_body = self.vessel.flight(self.vessel.reference_frame)
        
        self.altitude_stream = self.conn.add_stream(getattr, flight_world, 'surface_altitude')
        
        # KSP doesn't provide a direct acceleration stream, so we stream velocity and UT to differentiate
        self.velocity_stream = self.conn.add_stream(getattr, flight_world, 'velocity')
        self.ut_stream = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
        self.last_vel = None
        self.last_ut = None
        
        # Attitude (quaternion) in the target reference frame
        self.attitude_stream = self.conn.add_stream(getattr, flight_world, 'rotation')
        
        # Mass stream
        self.mass_stream = self.conn.add_stream(getattr, self.vessel, 'mass')
        
        # Noise parameters (Standard Deviations) from config
        self.sigma_alt = config.SIGMA_ALT
        self.sigma_accel = config.SIGMA_ACCEL
        
        # Isolated RNG for determinism
        self.rng = np.random.default_rng(config.RANDOM_SEED)
        
        logger.info(f"Initialized SensorModels with sigma_alt={self.sigma_alt}, sigma_accel={self.sigma_accel}")

    def poll(self) -> Tuple[float, np.ndarray, np.ndarray, float]:
        """
        Samples the streams and applies zero-mean Gaussian noise.
        Returns:
            noisy_alt (float)
            noisy_accel (np.ndarray shape (3,))
            attitude (np.ndarray shape (4,))
            mass (float)
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
        
        # Inject Gaussian Noise
        noisy_alt = float(perfect_alt + self.rng.normal(0, self.sigma_alt))
        noisy_accel = perfect_accel + self.rng.normal(0, self.sigma_accel, size=3)
        
        return noisy_alt, noisy_accel, attitude, float(mass)
