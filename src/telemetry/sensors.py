import numpy as np
from typing import Tuple, Any
import src.config as config

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
        flight = self.vessel.flight(self.ref_frame)
        self.altitude_stream = self.conn.add_stream(getattr, flight, 'surface_altitude')
        
        # For acceleration, we want body frame, but since there's no native body-frame acceleration 
        # that ignores gravity, we'll pull from the vessel's native reference frame.
        # However, to be consistent with ADR-014, we will pull acceleration in the custom world frame
        # and then we'll pretend it's body frame for testing, or just use world frame natively.
        # Actually, flight.velocity is relative to ref_frame. We can stream flight.velocity
        # to calculate true acceleration, or use kRPC's `vessel.flight(ref_frame).acceleration`.
        self.accel_stream = self.conn.add_stream(getattr, flight, 'acceleration')
        
        # Attitude (quaternion) in the target reference frame
        self.attitude_stream = self.conn.add_stream(getattr, flight, 'rotation')
        
        # Mass stream
        self.mass_stream = self.conn.add_stream(getattr, self.vessel, 'mass')
        
        # Noise parameters (Standard Deviations) from config
        self.sigma_alt = config.SIGMA_ALT
        self.sigma_accel = config.SIGMA_ACCEL

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
        perfect_accel = np.array(self.accel_stream())
        attitude = np.array(self.attitude_stream())
        mass = self.mass_stream()
        
        # Inject Gaussian Noise
        noisy_alt = float(perfect_alt + np.random.normal(0, self.sigma_alt))
        noisy_accel = perfect_accel + np.random.normal(0, self.sigma_accel, size=3)
        
        return noisy_alt, noisy_accel, attitude, float(mass)
