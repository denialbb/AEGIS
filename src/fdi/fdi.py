import numpy as np
from typing import List
from src.common.engine import Engine

class FaultDetectionIsolation:
    def __init__(self, threshold: float = 0.5):
        """
        threshold: Deviation in m/s^2 above which a fault is declared.
        """
        # ISS-001: placeholder — calibrate against measured KF output noise once ISS-003 is resolved.
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
        # ISS-006: mass is sourced from clean kRPC telemetry (vessel.mass), not the State Estimator. If mass is ever noised or rate-limited, this expected_accel calculation will produce spurious fault flags.
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
        force_tolerance = self.threshold * mass
        
        min_error = float('inf')
        best_combo: List[int] = []
        
        import itertools
        
        # Test all combinations of active engines (from 1 to N failures)
        for num_failed in range(1, len(active_engines) + 1):
            for combo in itertools.combinations(enumerate(active_engines), num_failed):
                combo_force = np.zeros(3)
                for i, engine in combo:
                    combo_force += engine.thrust_direction * engine.max_thrust * expected_throttles[i]
                    
                error = float(np.linalg.norm(missing_force - combo_force))
                if error < min_error:
                    min_error = error
                    best_combo = [engine.index for i, engine in combo]
                    
        # If the best matching combination is within our tolerance, return it
        if min_error < force_tolerance * 2.0: # Giving some leeway for double failures
            return best_combo
            
        # Fallback: if we can't cleanly isolate, return the best guess anyway
        return best_combo
