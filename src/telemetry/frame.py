import dataclasses
from typing import Dict, Any, List
import numpy as np

@dataclasses.dataclass
class TelemetryFrame:
    """
    Telemetry frame containing vehicle state and actuator commands at a single tick.
    
    Attributes:
        timestamp: Simulation or system timestamp.
        altitude: Current vehicle noisy altitude (kRPC measurement).
        velocity: Vehicle velocity vector. Shape: (3,)
        noisy_accel: Noisy accelerometer readings. Shape: (3,)
        throttles: Commanded throttles. Shape: (N,) where N is engine count.
        gimbals: Commanded gimbal angles (pitch, yaw). Shape: (N, 2) where N is engine count.
        skip_predict: Flag indicating if Kalman filter predict step was skipped due to dt spike.
        est_alt: EKF estimated altitude for comparison with measurement.
        a_avail: Available vertical acceleration (m/s²).
        force_body: Guidance force command body frame (3,).
        axial_forces: Per-engine axial forces (N,).
        position: EKF position (2,) for (north, east) distance-from-pad scoring.
    """
    timestamp: float
    altitude: float
    velocity: np.ndarray
    noisy_accel: np.ndarray
    throttles: np.ndarray
    fuel_state: np.ndarray
    gimbals: np.ndarray
    skip_predict: bool = False
    est_alt: float = 0.0
    a_avail: float = 0.0
    force_body: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(3))
    axial_forces: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(1))
    position: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(2))
    true_attitude: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(3))
    est_attitude: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(3))

    def flatten(self) -> Dict[str, Any]:
        """
        Flatten nested structures (like numpy arrays) into flat dictionary entries 
        suitable for a CSV row. Uses the sizes of the arrays to determine engine count.
        """
        flat_data: Dict[str, Any] = {
            "timestamp": self.timestamp,
            "altitude": self.altitude,
            "est_alt": self.est_alt,
            "a_avail": self.a_avail,
            "skip_predict": int(self.skip_predict),
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

        # Flatten guidance force body (3,)
        for i, axis in enumerate(['x', 'y', 'z']):
            if i < self.force_body.size:
                flat_data[f"fb_{axis}"] = float(self.force_body[i])

        # Flatten per-engine axial forces
        for i in range(self.axial_forces.size):
            flat_data[f"axial_{i}"] = float(self.axial_forces[i])

        # Flatten position (2,) — north, east for distance from pad
        for i, axis in enumerate(['n', 'e']):
            if i < self.position.size:
                flat_data[f"pos_{axis}"] = float(self.position[i])

        # Flatten attitude angles (pitch, yaw, roll)
        for i, axis in enumerate(['pitch', 'yaw', 'roll']):
            if i < self.true_attitude.size:
                flat_data[f"true_{axis}"] = float(self.true_attitude[i])
            if i < self.est_attitude.size:
                flat_data[f"est_{axis}"] = float(self.est_attitude[i])

        return flat_data
    
    @classmethod
    def get_csv_headers(cls, num_engines: int) -> List[str]:
        """
        Get the ordered list of CSV headers based on engine count.
        """
        headers = [
            "timestamp", "altitude", "est_alt", "a_avail", "skip_predict",
            "vel_x", "vel_y", "vel_z",
            "accel_x", "accel_y", "accel_z"
        ]
        for i in range(num_engines):
            headers.append(f"throttle_{i}")
            headers.append(f"has_fuel_{i}")
        for i in range(num_engines):
            headers.append(f"gimbal_{i}_0")
            headers.append(f"gimbal_{i}_1")
        headers.extend([
            "fb_x", "fb_y", "fb_z",
        ])
        for i in range(num_engines):
            headers.append(f"axial_{i}")
        headers.extend(["pos_n", "pos_e"])
        headers.extend(["true_pitch", "true_yaw", "true_roll"])
        headers.extend(["est_pitch", "est_yaw", "est_roll"])
        return headers
