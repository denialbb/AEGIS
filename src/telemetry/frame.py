import dataclasses
from typing import Dict, Any, List
import numpy as np

@dataclasses.dataclass
class TelemetryFrame:
    """
    Telemetry frame containing vehicle state and actuator commands at a single tick.
    
    Attributes:
        timestamp: Simulation or system timestamp.
        altitude: Current vehicle altitude.
        velocity: Vehicle velocity vector. Shape: (3,)
        noisy_accel: Noisy accelerometer readings. Shape: (3,)
        throttles: Commanded throttles. Shape: (N,) where N is engine count.
        gimbals: Commanded gimbal angles (pitch, yaw). Shape: (N, 2) where N is engine count.
        skip_predict: Flag indicating if Kalman filter predict step was skipped due to dt spike.
    """
    timestamp: float
    altitude: float
    velocity: np.ndarray
    noisy_accel: np.ndarray
    throttles: np.ndarray
    fuel_state: np.ndarray
    gimbals: np.ndarray
    skip_predict: bool = False

    def flatten(self) -> Dict[str, Any]:
        """
        Flatten nested structures (like numpy arrays) into flat dictionary entries 
        suitable for a CSV row. Uses the sizes of the arrays to determine engine count.
        """
        flat_data: Dict[str, Any] = {
            "timestamp": self.timestamp,
            "altitude": self.altitude,
            "skip_predict": int(self.skip_predict),  # ISS-010: Log skip_predict for debugging
        }
        
        # Flatten velocity (3,)
        for i, axis in enumerate(['x', 'y', 'z']):
            if i < self.velocity.size:
                flat_data[f"vel_{axis}"] = float(self.velocity[i])
            
        # Flatten noisy_accel (3,)
        for i, axis in enumerate(['x', 'y', 'z']):
            if i < self.noisy_accel.size:
                flat_data[f"accel_{axis}"] = float(self.noisy_accel[i])
                
        # Flatten throttles (N,)
        num_engines = self.throttles.size
        for i in range(num_engines):
            flat_data[f"throttle_{i}"] = float(self.throttles[i])
            
        # Flatten fuel_state (N,)
        if hasattr(self, 'fuel_state') and self.fuel_state is not None:
            for i in range(num_engines):
                flat_data[f"has_fuel_{i}"] = int(self.fuel_state[i])
            
        # Flatten gimbals (N, 2) -> N engines, 2 axes (pitch, yaw typically)
        if self.gimbals.ndim == 2:
            num_gimbal_engines = self.gimbals.shape[0]
            for i in range(num_gimbal_engines):
                flat_data[f"gimbal_{i}_0"] = float(self.gimbals[i, 0])
                flat_data[f"gimbal_{i}_1"] = float(self.gimbals[i, 1])
        elif self.gimbals.ndim == 1:
            num_gimbal_engines = self.gimbals.size // 2
            for i in range(num_gimbal_engines):
                flat_data[f"gimbal_{i}_0"] = float(self.gimbals[i * 2])
                flat_data[f"gimbal_{i}_1"] = float(self.gimbals[i * 2 + 1])

        return flat_data
    
    @classmethod
    def get_csv_headers(cls, num_engines: int) -> List[str]:
        """
        Get the ordered list of CSV headers based on engine count.
        """
        headers = [
            "timestamp", "altitude", "skip_predict",
            "vel_x", "vel_y", "vel_z",
            "accel_x", "accel_y", "accel_z"
        ]
        for i in range(num_engines):
            headers.append(f"throttle_{i}")
            headers.append(f"has_fuel_{i}")
        for i in range(num_engines):
            headers.append(f"gimbal_{i}_0")
            headers.append(f"gimbal_{i}_1")
        return headers
