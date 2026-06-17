import numpy as np
import logging
from typing import Any
import src.config as config
from scipy.spatial.transform import Rotation as R  # type: ignore

logger = logging.getLogger(__name__)


class GyroSensor:
    """
    Handles gyroscope readings with noise injection and bias modeling.
    Provides cleaned angular velocity readings for attitude estimation.
    """
    
    def __init__(self, conn: Any, vessel: Any, ref_frame: Any, up_vector: np.ndarray):
        """
        Initializes the gyroscope sensor.
        
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
        
        # Initialize kRPC stream for angular velocity
        # Use getattr to stream vessel angular velocity
        # Stream vessel angular velocity in reference frame
        # Angular velocity stream via bound method
        # Create a bound method to avoid RPC errors
        # It will invoke vessel.angular_velocity without extra args
        self.angular_velocity_stream = self.conn.add_stream(self.vessel.angular_velocity, self.ref_frame)
        
        # Noise parameters from config
        self.sigma_gyro = config.SIGMA_GYRO if hasattr(config, 'SIGMA_GYRO') else 0.01  # rad/s
        self.gyro_bias_instability = config.GYRO_BIAS_INSTABILITY if hasattr(config, 'GYRO_BIAS_INSTABILITY') else 0.0001  # rad/s/sqrt(Hz)
        
        # Bias states (to be estimated/filtered)
        self.gyro_bias = np.zeros(3)  # Estimated bias in each axis
        self.bias_update_gain = config.GYRO_BIAS_UPDATE_GAIN if hasattr(config, 'GYRO_BIAS_UPDATE_GAIN') else 0.001
        
        # Isolated RNG for determinism
        self.rng = np.random.default_rng(config.RANDOM_SEED if hasattr(config, 'RANDOM_SEED') else 42)
        
        logger.info(f"Initialized GyroSensor with sigma_gyro={self.sigma_gyro}, bias_instability={self.gyro_bias_instability}")
    
    def _angular_velocity(self):
        """Return vessel's angular velocity (Vector3)."""
        return self.vessel.angular_velocity

    def poll(self) -> np.ndarray:
        """
        Samples the gyroscope stream and applies zero-mean Gaussian noise.
        Bias estimation is now owned by the EKF (ErrorStateEKF) — this sensor
        only injects noise and passes raw readings upward.

        Returns:
            noisy_angular_velocity (np.ndarray shape (3,)) body-frame angular rates in rad/s
        """
        av_raw = self.angular_velocity_stream()

        if hasattr(av_raw, "x"):
            perfect_angular_velocity = np.array([av_raw.x, av_raw.y, av_raw.z])
        else:
            perfect_angular_velocity = np.array(av_raw, dtype=float)

        if config.NOISELESS_MODE:
            noisy_angular_velocity = perfect_angular_velocity
        else:
            noisy_angular_velocity = perfect_angular_velocity + self.rng.normal(0, self.sigma_gyro, size=3)

        logger.debug(
            f"Gyro: raw=[{perfect_angular_velocity[0]:.3f}, {perfect_angular_velocity[1]:.3f}, {perfect_angular_velocity[2]:.3f}] "
            f"noisy=[{noisy_angular_velocity[0]:.3f}, {noisy_angular_velocity[1]:.3f}, {noisy_angular_velocity[2]:.3f}]"
        )

        return noisy_angular_velocity
    
    def get_bias(self) -> np.ndarray:
        """Returns the current estimated gyroscope bias."""
        return self.gyro_bias.copy()
