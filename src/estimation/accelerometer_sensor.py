import numpy as np
import logging
from typing import Any
import src.config as config

logger = logging.getLogger(__name__)


class AccelerometerSensor:
    """
    Handles accelerometer readings with noise injection.
    Provides cleaned specific force measurements (proper acceleration).
    """
    
    def __init__(self, conn: Any, vessel: Any, ref_frame: Any, up_vector: np.ndarray):
        """
        Initializes the accelerometer sensor.
        
        Args:
            conn: kRPC connection object
            vessel: active vessel object
            ref_frame: the custom landing pad reference frame
            up_vector: (3,) normalized vector pointing away from the center of the celestial body
        """
        self.conn = conn
        self.vessel = vessel
        self.ref_frame = ref_frame
        self.up_vector = up_vector
        
        # Initialize kRPC streams needed to compute acceleration
        flight_world = self.vessel.flight(self.ref_frame)
        
        # KSP doesn't provide a direct acceleration stream, so we stream velocity and UT to differentiate
        self.velocity_stream = self.conn.add_stream(getattr, flight_world, 'velocity')
        self.ut_stream = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
        self.last_vel: np.ndarray | None = None
        self.last_ut: float | None = None
        
        # Noise parameters from config
        self.sigma_accel = config.SIGMA_ACCEL if hasattr(config, 'SIGMA_ACCEL') else 0.1  # m/s^2
        self.accel_bias_instability = config.ACCEL_BIAS_INSTABILITY if hasattr(config, 'ACCEL_BIAS_INSTABILITY') else 0.001  # m/s^2/sqrt(Hz)
        
        # Bias states
        self.accel_bias = np.zeros(3)  # Estimated bias in each axis
        self.bias_update_gain = 0.001  # How quickly we adapt to bias changes
        
        # Isolated RNG for determinism
        self.rng = np.random.default_rng(config.RANDOM_SEED if hasattr(config, 'RANDOM_SEED') else 42)
        
        logger.info(f"Initialized AccelerometerSensor with sigma_accel={self.sigma_accel}, bias_instability={self.accel_bias_instability}")
    
    def poll(self, gravity_world: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Samples the velocity streams to compute acceleration, applies noise.
        Returns both world-frame and body-frame specific force.
        
        Args:
            gravity_world: Gravitational acceleration in world frame (m/s^2)
            
        Returns:
            tuple of (accel_body, accel_world) where:
            - accel_body: Specific force in body frame (m/s^2) - what an IMU measures
            - accel_world: Specific force in world frame (m/s^2)
        """
        # Read perfect data
        ut = self.ut_stream()
        vel = np.array(self.velocity_stream())
        
        if self.last_vel is None:
            perfect_accel_world = np.zeros(3)  # Coordinate acceleration
        else:
            dt = ut - self.last_ut
            if dt > 0:
                perfect_accel_world = (vel - self.last_vel) / dt
            else:
                perfect_accel_world = np.zeros(3)
                
        self.last_vel = vel
        self.last_ut = ut
        
        # Compute gravity using body's gravitational parameter and vessel's position
        body = self.vessel.orbit.body
        mu = body.gravitational_parameter
        # Vessel's position relative to body center in the reference frame
        pos_rel_body = np.array(self.vessel.position(self.ref_frame))
        distance = np.linalg.norm(pos_rel_body)
        if distance > 0:
            gravity_world = - (mu / distance**3) * pos_rel_body
        else:
            # Fallback to avoid division by zero
            gravity_world = -self.up_vector * 9.81
        
        # Proper acceleration (specific force) = coordinate acceleration - gravity
        # This is what an accelerometer actually measures
        perfect_specific_force_world = perfect_accel_world - gravity_world
        
        # Add noise to simulate real accelerometer
        noisy_specific_force_world = perfect_specific_force_world + self.rng.normal(0, self.sigma_accel, size=3)
        
        # Simple bias estimation
        bias_corrected_world = noisy_specific_force_world - self.accel_bias
        
        # Update bias estimate using low-pass filter
        self.accel_bias = self.accel_bias * (1 - self.bias_update_gain) + noisy_specific_force_world * self.bias_update_gain
        
        logger.debug(
            f"Accel: world_coord=[{perfect_accel_world[0]:.3f}, {perfect_accel_world[1]:.3f}, {perfect_accel_world[2]:.3f}] "
            f"gravity=[{gravity_world[0]:.3f}, {gravity_world[1]:.3f}, {gravity_world[2]:.3f}] "
            f"specific_force=[{perfect_specific_force_world[0]:.3f}, {perfect_specific_force_world[1]:.3f}, {perfect_specific_force_world[2]:.3f}] "
            f"noisy=[{noisy_specific_force_world[0]:.3f}, {noisy_specific_force_world[1]:.3f}, {noisy_specific_force_world[2]:.3f}] "
            f"bias=[{self.accel_bias[0]:.3f}, {self.accel_bias[1]:.3f}, {self.accel_bias[2]:.3f}] "
            f"cleaned=[{bias_corrected_world[0]:.3f}, {bias_corrected_world[1]:.3f}, {bias_corrected_world[2]:.3f}]"
        )
        
        return bias_corrected_world, gravity_world
    
    def get_bias(self) -> np.ndarray:
        """Returns the current estimated accelerometer bias."""
        return self.accel_bias.copy()