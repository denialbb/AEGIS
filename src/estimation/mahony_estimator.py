import numpy as np
import logging
from typing import Any
import src.config as config
from scipy.spatial.transform import Rotation as R  # type: ignore

logger = logging.getLogger(__name__)


class MahonyAttitudeEstimator:
    """
    Mahony attitude estimator (complementary filter) that fuses gyroscope
    and accelerometer data to estimate vessel attitude.
    
    The filter uses:
    - Gyroscope data for short-term precision (integration)
    - Accelerometer data for long-term stability (gravity vector reference)
    """
    
    def __init__(self, 
                 gyro_sensor: Any, 
                 accel_sensor: Any,
                 kp: float = 2.0, 
                 ki: float = 0.0,
                 up_vector: np.ndarray = np.array([0.0, 0.0, 1.0])):
        """
        Initializes the Mahony attitude estimator.
        
        Args:
            gyro_sensor: GyroSensor instance
            accel_sensor: AccelerometerSensor instance
            kp: Proportional gain for acceleration error
            ki: Integral gain for acceleration error (applied to gyroscope bias)
            up_vector: Reference up vector for initialization
        """
        self.gyro_sensor = gyro_sensor
        self.accel_sensor = accel_sensor
        self.kp = kp
        self.ki = ki
        self.up_vector = up_vector / np.linalg.norm(up_vector)  # Normalize
        
        # Estimated attitude quaternion [x, y, z, w] (scalar-last, matches kRPC/scipy)
        self.quaternion = np.array([0.0, 0.0, 0.0, 1.0])  # Start with identity (no rotation)
        
        # Integral error terms for bias estimation
        self.integral_error = np.zeros(3)
        
        # Timestamp for dt calculation
        self.last_update_time = None
        
        logger.info(f"Initialized MahonyAttitudeEstimator with kp={kp}, ki={ki}")
    
    def update(self, dt: float) -> np.ndarray:
        """
        Updates the attitude estimate using gyroscope and accelerometer data.
        
        Args:
            dt: Time step since last update (seconds)
            
        Returns:
            quaternion: Estimated attitude quaternion [x, y, z, w]
        """
        # Get sensor readings
        omega = self.gyro_sensor.poll()  # Angular velocity in body frame (rad/s)
        accel_world, gravity_world = self.accel_sensor.poll(np.zeros(3))  # Specific force in world frame
        
        # Normalize accelerometer reading (should align with gravity when stationary)
        accel_norm = np.linalg.norm(accel_world)
        if accel_norm > 0.1:  # Only use accelerometer if we have reasonable signal
            accel_normalized = accel_world / accel_norm
        else:
            # If accelerometer data is unreliable, skip correction
            accel_normalized = None
        
        # Current estimated gravity vector in body frame (from quaternion)
        # In body frame, gravity should be [0, 0, -1] when level
        gravity_estimate_body = R.from_quat(self.quaternion).inv().apply(np.array([0.0, 0.0, -9.81]))
        gravity_estimate_body_unit = gravity_estimate_body / np.linalg.norm(gravity_estimate_body)
        
        # Compute error between measured acceleration and expected gravity
        error = np.zeros(3)
        if accel_normalized is not None:
            # Expected accelerometer reading in body frame when level: [0, 0, -1] (gravity down)
            expected_accel_body = np.array([0.0, 0.0, -1.0])
            
            # Rotate expected acceleration to world frame using current estimate
            expected_accel_world = R.from_quat(self.quaternion).apply(expected_accel_body)
            
            # Error is cross product between expected and measured directions
            error = np.cross(expected_accel_world, accel_normalized)
        
        # Apply proportional and integral gains
        if self.ki > 0:
            self.integral_error += error * dt
            # Apply integral feedback to gyroscope
            omega_corrected = omega + (self.kp * error) + (self.ki * self.integral_error)
        else:
            omega_corrected = omega + (self.kp * error)
        
        # Convert quaternion to rate of change
        q = self.quaternion
        omega_q = np.array([omega_corrected[0], omega_corrected[1], omega_corrected[2], 0.0])
        
        # Quaternion derivative: q_dot = 0.5 * q ⊗ omega
        q_dot = 0.5 * self._quaternion_multiply(q, omega_q)
        
        # Integrate to get new quaternion
        q_new = q + q_dot * dt
        
        # Normalize quaternion
        self.quaternion = q_new / np.linalg.norm(q_new)
        
        # Store timestamp for next iteration
        self.last_update_time = dt
        
        logger.debug(
            f"Mahony: omega=[{omega[0]:.3f}, {omega[1]:.3f}, {omega[2]:.3f}] "
            f"error=[{error[0]:.3f}, {error[1]:.3f}, {error[2]:.3f}] "
            f"omega_corrected=[{omega_corrected[0]:.3f}, {omega_corrected[1]:.3f}, {omega_corrected[2]:.3f}] "
            f"q=[{self.quaternion[0]:.3f}, {self.quaternion[1]:.3f}, {self.quaternion[2]:.3f}, {self.quaternion[3]:.3f}]"
        )
        
        return self.quaternion.copy()
    
    def _quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """
        Multiply two quaternions.
        Args:
            q1, q2: Quaternions as [x, y, z, w]
        Returns:
            Result quaternion [x, y, z, w]
        """
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        
        return np.array([x, y, z, w])
    
    def get_attitude(self) -> np.ndarray:
        """Returns the current estimated attitude quaternion."""
        return self.quaternion.copy()
    
    def reset(self):
        """Resets the estimator to initial state."""
        self.quaternion = np.array([0.0, 0.0, 0.0, 1.0])
        self.integral_error = np.zeros(3)
        self.last_update_time = None