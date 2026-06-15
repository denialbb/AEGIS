import numpy as np
import logging
from typing import Tuple, Any
import src.config as config
from scipy.spatial.transform import Rotation as R  # type: ignore

# Import the new sensor classes
from src.estimation.gyro_sensor import GyroSensor
from src.estimation.accelerometer_sensor import AccelerometerSensor
from src.estimation.mahony_estimator import MahonyAttitudeEstimator

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
        
        # Initialize kRPC streams for basic telemetry
        flight_world = self.vessel.flight(self.ref_frame)
        
        self.altitude_stream = self.conn.add_stream(getattr, flight_world, 'surface_altitude')
        
        # KSP doesn't provide a direct acceleration stream, so we stream velocity and UT to differentiate
        self.velocity_stream = self.conn.add_stream(getattr, flight_world, 'velocity')
        self.ut_stream = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
        self.last_vel: np.ndarray | None = None
        self.last_ut: float | None = None
        
        # Attitude (quaternion) in the target reference frame (from kRPC, for backup/comparison)
        self.attitude_stream = self.conn.add_stream(getattr, flight_world, 'rotation')
        
        # Aerodynamic force in the target reference frame
        self.aero_stream = self.conn.add_stream(getattr, flight_world, 'aerodynamic_force')
        
        # Mass stream
        self.mass_stream = self.conn.add_stream(getattr, self.vessel, 'mass')
        
        # Situation stream
        self.situation_stream = self.conn.add_stream(getattr, self.vessel, 'situation')
        
        # Initialize the new sensor classes
        self.gyro_sensor = GyroSensor(conn, vessel, ref_frame, up_vector)
        self.accel_sensor = AccelerometerSensor(conn, vessel, ref_frame, up_vector)
        
        # Initialize Mahony attitude estimator
        self.attitude_estimator = MahonyAttitudeEstimator(
            gyro_sensor=self.gyro_sensor,
            accel_sensor=self.accel_sensor,
            kp=config.MAHONY_KP if hasattr(config, 'MAHONY_KP') else 2.0,
            ki=config.MAHONY_KI if hasattr(config, 'MAHONY_KI') else 0.0,
            up_vector=up_vector
        )
        
        # Noise parameters (Standard Deviations) from config
        self.sigma_alt = config.SIGMA_ALT
        self.sigma_accel = config.SIGMA_ACCEL
        self.sigma_vel = config.SIGMA_VEL
        
        # Isolated RNG for determinism
        self.rng = np.random.default_rng(config.RANDOM_SEED)
        
        logger.info(f"Initialized SensorModels with sigma_alt={self.sigma_alt}, sigma_accel={self.sigma_accel}, sigma_vel={self.sigma_vel}")
        logger.info("Initialized Mahony attitude estimator for IMU-based attitude estimation")

    def poll(self) -> Tuple[float, np.ndarray, np.ndarray, float, np.ndarray, str, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Samples the streams and applies zero-mean Gaussian noise.
        Uses IMU-based attitude estimation from Mahony filter.
        Returns:
            noisy_alt (float)
            noisy_accel (np.ndarray shape (3,)) - body-frame specific force from accelerometer
            attitude (np.ndarray shape (4,)) - estimated attitude quaternion [x, y, z, w] from Mahony filter
            mass (float)
            aero_force_body (np.ndarray shape (3,)) - body-frame aerodynamic force
            situation (str)
            angular_velocity (np.ndarray shape (3,)) - body-frame angular rates (rad/s) from gyroscope
            noisy_vel (np.ndarray shape (3,)) - world-frame velocity in m/s
            gravity_world (np.ndarray shape (3,)) - gravitational acceleration in world frame (m/s^2)
        """
        # Read perfect data for altitude and velocity (these are still needed)
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
        
        # Get mass and aerodynamic force
        mass = self.mass_stream()
        aero_world = np.array(self.aero_stream())
        situation = self.situation_stream().name
        
        # Get IMU-based sensor readings
        # Accelerometer gives us specific force (proper acceleration) in world frame and computes gravity
        accel_world, gravity_world = self.accel_sensor.poll(np.zeros(3))  # Pass dummy gravity, will be computed inside
        
        # Convert accelerometer reading from world frame to body frame for the EKF
        # We need the current attitude estimate to do this conversion
        current_attitude = self.attitude_estimator.get_attitude()
        rot = R.from_quat(current_attitude)
        accel_body = rot.inv().apply(accel_world)  # Convert world-frame specific force to body-frame
        
        # Get gyroscope reading (already in body frame)
        angular_velocity = self.gyro_sensor.poll()
        
        # Update attitude estimate using gyroscope and accelerometer data
        # We need to estimate dt - use a default or track time
        # For now, we'll use a fixed small dt since poll() is called regularly
        dt = 0.02  # 50Hz update rate, adjust if needed
        estimated_attitude = self.attitude_estimator.update(dt)
        
        # Compute proper acceleration (specific force) - this is what accelerometers measure
        # accel_world from accelerometer sensor IS the specific force (accelerometer reading)
        # So perfect_proper_accel = accel_world (this is what we want to pass to estimator)
        perfect_proper_accel = accel_world
        
        # Rotate aero force to body frame using estimated attitude
        aero_body = rot.inv().apply(aero_world)
        
        # Inject Gaussian Noise to simulate real sensor readings
        noisy_alt = float(perfect_alt + self.rng.normal(0, self.sigma_alt))
        # Accelerometer noise is already added in the sensor class, but we add extra for realism if needed
        noisy_accel = accel_body + self.rng.normal(0, self.sigma_accel, size=3)
        noisy_vel = vel + self.rng.normal(0, self.sigma_vel, size=3)
        
        # Debug logging
        logger.debug(
            f"Gyro: [{angular_velocity[0]:.3f}, {angular_velocity[1]:.3f}, {angular_velocity[2]:.3f}] "
            f"Accel body: [{accel_body[0]:.3f}, {accel_body[1]:.3f}, {accel_body[2]:.3f}] "
            f"Attitude: [{estimated_attitude[0]:.3f}, {estimated_attitude[1]:.3f}, {estimated_attitude[2]:.3f}, {estimated_attitude[3]:.3f}]"
        )
        logger.debug(f"Gravity world: [{gravity_world[0]:.3f}, {gravity_world[1]:.3f}, {gravity_world[2]:.3f}]")
 
         
        return noisy_alt, noisy_accel, estimated_attitude, float(mass), aero_body, situation, angular_velocity, noisy_vel, gravity_world
