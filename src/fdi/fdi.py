import numpy as np
from typing import List
from src.common.engine import Engine

class FaultDetectionIsolation:
    def __init__(self, threshold: float = 0.5):
        """
        threshold: Deviation in m/s^2 above which a fault is declared.
        """
        self.threshold: float = threshold

    def detect_fault(self, expected_accel: np.ndarray, measured_accel: np.ndarray) -> bool:
        """
        Compares expected vs measured acceleration.
        Returns True if the magnitude of the difference exceeds the threshold.
        """
        diff = expected_accel - measured_accel
        deviation = np.linalg.norm(diff)
        return bool(deviation > self.threshold)

    def isolate_fault(self, active_engines: List[Engine], expected_throttles: np.ndarray, 
                      measured_accel: np.ndarray, mass: float) -> List[int]:
        """
        Isolates which engine(s) failed.
        expected_throttles: array of throttle values [0.0, 1.0] commanded in the previous step, shape (N,)
        Returns a list of engine indices that have suffered a fault.
        """
        if not active_engines:
            return []
            
        expected_force = np.zeros(3)
        for i, engine in enumerate(active_engines):
            force_i = engine.thrust_direction * engine.max_thrust * expected_throttles[i]
            expected_force += force_i
            
        expected_accel = expected_force / mass
        
        if not self.detect_fault(expected_accel, measured_accel):
            return []

        missing_force = expected_force - (measured_accel * mass)
        
        failed_engines = []
        min_error = float('inf')
        best_candidate = -1
        
        for i, engine in enumerate(active_engines):
            expected_force_i = engine.thrust_direction * engine.max_thrust * expected_throttles[i]
            # Only consider engines that were actually commanded to produce thrust
            if expected_throttles[i] > 0.01:
                error = float(np.linalg.norm(missing_force - expected_force_i))
                if error < min_error:
                    min_error = error
                    best_candidate = engine.index
                    
        if best_candidate != -1:
            failed_engines.append(best_candidate)
            
        return failed_engines
