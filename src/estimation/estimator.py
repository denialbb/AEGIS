import numpy as np
from filterpy.kalman import KalmanFilter  # type: ignore

class StateEstimator:
    def __init__(self, initial_state: np.ndarray, initial_covariance: np.ndarray, process_noise: np.ndarray, measurement_noise: np.ndarray):
        """
        Initializes the Discrete-Time Kalman Filter.
        initial_state: shape (6,) [X, Y, Z, Vx, Vy, Vz]
        initial_covariance: shape (6, 6)
        process_noise: shape (6, 6)
        measurement_noise: shape (1, 1) (altitude measurement variance)
        """
        self.kf = KalmanFilter(dim_x=6, dim_z=1)
        self.kf.x = initial_state.copy()
        self.kf.P = initial_covariance.copy()
        self.kf.Q = process_noise.copy()
        self.kf.R = measurement_noise.copy()
        
        # H matrix: z = [alt]
        # We map altitude to the Z coordinate (index 2).
        self.kf.H = np.zeros((1, 6))
        self.kf.H[0, 2] = 1.0
        
    def predict(self, noisy_accel_body: np.ndarray, attitude: np.ndarray, dt: float) -> None:
        """
        Predicts the next state using the measured acceleration as the control input.
        noisy_accel_body: Accelerometer reading in vessel body frame.
        attitude: Vessel attitude required to rotate acceleration to world frame.
        """
        # TODO: Implement actual quaternion rotation using `attitude`.
        # For now, we assume body frame is aligned with world frame.
        noisy_accel_world = noisy_accel_body.copy()
        
        # Update dynamic state transition matrix F
        self.kf.F = np.eye(6)
        self.kf.F[0, 3] = dt
        self.kf.F[1, 4] = dt
        self.kf.F[2, 5] = dt
        
        # Control input matrix B for acceleration
        B = np.zeros((6, 3))
        B[0:3, 0:3] = 0.5 * (dt ** 2) * np.eye(3)
        B[3:6, 0:3] = dt * np.eye(3)
        
        # Predict step using noisy_accel_world as the control input
        self.kf.predict(u=noisy_accel_world, B=B)
        
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
