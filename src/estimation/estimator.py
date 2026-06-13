import numpy as np
from filterpy.kalman import KalmanFilter  # type: ignore

class StateEstimator:
    def __init__(self, initial_state: np.ndarray, initial_covariance: np.ndarray, process_noise: np.ndarray, measurement_noise: np.ndarray):
        """
        Initializes the Discrete-Time Kalman Filter.
        initial_state: shape (7,) [X, Y, Z, Vx, Vy, Vz, Mass]
        initial_covariance: shape (7, 7)
        process_noise: shape (7, 7)
        measurement_noise: shape (4, 4) (fuses 3D acceleration + 1D altitude)
        """
        self.kf = KalmanFilter(dim_x=7, dim_z=4)
        self.kf.x = initial_state.copy()
        self.kf.P = initial_covariance.copy()
        self.kf.Q = process_noise.copy()
        self.kf.R = measurement_noise.copy()
        
        # H matrix: z = [alt, ax, ay, az]^T
        # We map altitude to the Z coordinate (index 2).
        # Accel measurements are zeroed out in H, but used as control input in predict.
        self.kf.H = np.zeros((4, 7))
        self.kf.H[0, 2] = 1.0
        
    def update(self, noisy_alt: float, noisy_accel: np.ndarray, dt: float) -> np.ndarray:
        """
        Fuses noisy altimeter and accelerometer data to update the state estimate.
        Returns the updated state vector.
        """
        # Update dynamic state transition matrix F
        self.kf.F = np.eye(7)
        self.kf.F[0, 3] = dt
        self.kf.F[1, 4] = dt
        self.kf.F[2, 5] = dt
        
        # Control input matrix B for acceleration
        B = np.zeros((7, 3))
        B[0:3, 0:3] = 0.5 * (dt ** 2) * np.eye(3)
        B[3:6, 0:3] = dt * np.eye(3)
        
        # Predict step using noisy_accel as the control input
        self.kf.predict(u=noisy_accel, B=B)
        
        # Update step using alt and accel as measurements
        z = np.zeros(4)
        z[0] = noisy_alt
        z[1:4] = noisy_accel
        self.kf.update(z)
        
        return self.kf.x.copy()

    def get_state(self) -> np.ndarray:
        """
        Returns the current estimated state vector of shape (7,).
        """
        return self.kf.x.copy()
