import numpy as np
import logging
from filterpy.kalman import KalmanFilter  # type: ignore
from scipy.spatial.transform import Rotation as R  # type: ignore

logger = logging.getLogger(__name__)  # type: ignore

class StateEstimator:
    def __init__(self, initial_state: np.ndarray, initial_covariance: np.ndarray, process_noise: np.ndarray, measurement_noise: np.ndarray, up_vector: np.ndarray = np.array([0.0, 0.0, 1.0])):
        """
        Initializes the Discrete-Time Kalman Filter.
        initial_state: shape (6,) [X, Y, Z, Vx, Vy, Vz]
        initial_covariance: shape (6, 6)
        process_noise: shape (6, 6)
        measurement_noise: shape (1, 1) (altitude measurement variance)
        up_vector: vector pointing directly up from the planet surface
        """
        self.kf = KalmanFilter(dim_x=6, dim_z=1)
        self.kf.x = initial_state.copy()
        self.kf.P = initial_covariance.copy()
        self.kf.Q = process_noise.copy()
        self.kf.R = measurement_noise.copy()
        
        self.up_vector = up_vector
        self.gravity_world = -self.up_vector * 9.81
        
        # H matrix: z = [alt]
        # Altitude is the projection of the position state onto the up_vector.
        self.kf.H = np.zeros((1, 6))
        self.kf.H[0, :3] = self.up_vector
                
        logger.info("Initialized StateEstimator (Kalman Filter)")

    def predict(self, noisy_accel_body: np.ndarray, attitude: np.ndarray, dt: float) -> None:
        """
        Predicts the next state using the measured acceleration as the control input.
        noisy_accel_body: Accelerometer reading in vessel body frame.
        attitude: Vessel attitude required to rotate acceleration to world frame.
        """
        rot = R.from_quat(attitude)
        proper_accel_world = rot.apply(noisy_accel_body)
        
        # IMU measures proper acceleration (includes normal force, excludes gravity).
        # We need kinematic acceleration for Newtonian physics: a_kinematic = a_proper + gravity
        kinematic_accel_world = proper_accel_world + self.gravity_world
        
        # Update dynamic state transition matrix F.
        # This matrix represents the physics model x_next = F * x_current.
        # For a constant velocity model, position updates as: x_next = x + v * dt
        # The top right 3x3 block maps the velocity state (indices 3,4,5) into the position state (indices 0,1,2).
        self.kf.F = np.eye(6)
        self.kf.F[0, 3] = dt
        self.kf.F[1, 4] = dt
        self.kf.F[2, 5] = dt
        
        # Control input matrix B for acceleration.
        # This maps the control input (acceleration 'u') into the state vector.
        # According to Newtonian kinematics:
        # Position change due to acceleration: 0.5 * a * dt^2 (Top 3x3 block)
        # Velocity change due to acceleration: a * dt (Bottom 3x3 block)
        B = np.zeros((6, 3))
        B[0:3, 0:3] = 0.5 * (dt ** 2) * np.eye(3)
        B[3:6, 0:3] = dt * np.eye(3)
        
        # Predict step using kinematic_accel_world as the control input
        self.kf.predict(u=kinematic_accel_world, B=B)
        
    def update(self, noisy_alt: float) -> np.ndarray:
        """
        Fuses noisy altimeter data to correct the Z-axis state.
        Returns the updated state vector.
        """
        # Update step using alt as measurement
        z = np.array([noisy_alt])
        self.kf.update(z)
        
        return self.kf.x.copy()

    def get_state(self) -> np.ndarray:
        """
        Returns the current estimated state vector of shape (6,).
        """
        return self.kf.x.copy()
