import numpy as np
import logging
from typing import Any
import src.config as config
from src.common.reference_frame import compute_gravity_ned

logger = logging.getLogger(__name__)


class AccelerometerSensor:
    """
    Handles accelerometer readings with noise injection.
    Provides cleaned specific force measurements (proper acceleration).
    """
    
    def __init__(self, conn: Any, vessel: Any, ned_frame: Any, up_vector: np.ndarray,
                 shared_ut_stream: Any = None, shared_vel_stream: Any = None):
        """
        Initializes the accelerometer sensor.
        
        Args:
            conn: kRPC connection object
            vessel: active vessel object
            ned_frame: the custom NED landing pad reference frame
            up_vector: (3,) normalized vector pointing away from the center of the celestial body
            shared_ut_stream: Optional pre-existing UT stream to avoid duplicate kRPC overhead.
            shared_vel_stream: Optional pre-existing velocity stream to avoid duplicate kRPC overhead.
        """
        self.conn = conn
        self.vessel = vessel
        self.ned_frame = ned_frame
        self.up_vector = up_vector
        
        if shared_ut_stream is not None and shared_vel_stream is not None:
            self.ut_stream = shared_ut_stream
            self.velocity_stream = shared_vel_stream
        else:
            flight_ned = self.vessel.flight(self.ned_frame)
            self.velocity_stream = self.conn.add_stream(getattr, flight_ned, 'velocity')
            self.ut_stream = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
        self.last_vel: np.ndarray | None = None
        self.last_ut: float | None = None
        
        # Noise parameters from config
        self.sigma_accel = config.SIGMA_ACCEL if hasattr(config, 'SIGMA_ACCEL') else 0.1  # m/s^2

        self.orbit_body = vessel.orbit.body
        self.position_stream = self.conn.add_stream(
            vessel.position, self.orbit_body.reference_frame
        )
        self.rng = np.random.default_rng(config.RANDOM_SEED if hasattr(config, 'RANDOM_SEED') else 42)

        logger.info(f"Initialized AccelerometerSensor with sigma_accel={self.sigma_accel}")
    
    def poll(self, gravity_ned: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Samples the velocity streams to compute acceleration, applies noise.
        Bias estimation is owned exclusively by the EKF — this sensor only
        injects noise and passes raw readings upward.
        
        Args:
            gravity_ned: Gravitational acceleration in NED frame (m/s^2)
            
        Returns:
            tuple of (accel_ned, gravity_ned) where:
            - accel_ned: Noisy specific force in NED frame (m/s^2)
            - gravity_ned: Gravitational acceleration in NED frame (m/s^2)
        """
        # Read perfect data
        ut = self.ut_stream()
        vel = np.array(self.velocity_stream())
        
        if self.last_vel is None:
            perfect_accel_ned = np.zeros(3)  # Coordinate acceleration
        else:
            dt = ut - self.last_ut
            if dt > 0:
                perfect_accel_ned = (vel - self.last_vel) / dt
            else:
                perfect_accel_ned = np.zeros(3)
                
        self.last_vel = vel
        self.last_ut = ut
        
        # Compute gravity in NED from ECEF position.
        body = self.orbit_body
        pos_ecef = np.array(self.position_stream())
        gravity_ned = compute_gravity_ned(body, pos_ecef)
        
        # Proper acceleration (specific force) = coordinate acceleration - gravity
        # This is what an accelerometer actually measures.
        # NOTE: kRPC also exposes `vessel.flight().g_force` which directly returns the
        # G-force vector (proper acceleration / specific force) in g's. This could be used
        # as a direct accelerometer reading in future revisions, rotated to the target
        # frame and scaled by 9.81 m/s².
        perfect_specific_force_ned = perfect_accel_ned - gravity_ned
        
        if config.NOISELESS_MODE:
            noisy_specific_force_ned = perfect_specific_force_ned
        else:
            noisy_specific_force_ned = perfect_specific_force_ned + self.rng.normal(0, self.sigma_accel, size=3)

        logger.debug(
            f"Accel: coord_ned=[{perfect_accel_ned[0]:.3f}, {perfect_accel_ned[1]:.3f}, {perfect_accel_ned[2]:.3f}] "
            f"gravity=[{gravity_ned[0]:.3f}, {gravity_ned[1]:.3f}, {gravity_ned[2]:.3f}] "
            f"specific_force=[{perfect_specific_force_ned[0]:.3f}, {perfect_specific_force_ned[1]:.3f}, {perfect_specific_force_ned[2]:.3f}] "
            f"noisy=[{noisy_specific_force_ned[0]:.3f}, {noisy_specific_force_ned[1]:.3f}, {noisy_specific_force_ned[2]:.3f}]"
        )

        return noisy_specific_force_ned, gravity_ned
    
