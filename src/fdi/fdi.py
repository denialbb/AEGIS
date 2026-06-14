import numpy as np
import logging
from typing import List, Dict, Any
from src.common.engine import Engine

logger = logging.getLogger(__name__)

class FaultDetectionIsolation:
    def __init__(self, threshold: float = 0.5):
        """
        threshold: Deviation in m/s^2 above which a fault is declared.
        """
        # ISS-001: placeholder — calibrate against measured KF output noise once ISS-003 is resolved.
        self.threshold: float = threshold
        self.persistence_ticks: int = 50  # 1 second at 50Hz to allow for engine spool up
        self.consecutive_faults: int = 0

    def detect_fault(self, expected_accel: np.ndarray, measured_accel: np.ndarray) -> bool:
        """
        Compares expected vs measured acceleration.
        Increments a persistence counter to filter out transients like engine spool-up.
        Returns True if the magnitude of the difference exceeds the threshold for N consecutive ticks.
        """
        diff = expected_accel - measured_accel
        deviation = np.linalg.norm(diff)
        if deviation > self.threshold:
            self.consecutive_faults += 1
            if self.consecutive_faults >= self.persistence_ticks:
                logger.warning(f"[FDI] Persistent Fault Confirmed! Expected: {expected_accel}, Measured: {measured_accel}, Deviation: {deviation} > {self.threshold}")
                return True
        else:
            self.consecutive_faults = 0
            
        return False

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
        
        # We assume isolate_fault is only called AFTER detect_fault has returned True.
        # We do not call detect_fault again here to avoid double-incrementing the persistence counter.

        missing_force = expected_force - (measured_accel * mass)
        force_tolerance = self.threshold * mass
        
        min_error = float('inf')
        best_combo: List[int] = []
        
        import itertools
        
        # The FDI problem is an underdetermined inverse problem. We know there is missing force,
        # but we don't know which subset of engines stopped producing it.
        # We solve this by brute-forcing all possible failure combinations (1 engine out, 2 engines out, etc.).
        # Test all combinations of active engines (from 1 to N failures)
        for num_failed in range(1, len(active_engines) + 1):
            for combo in itertools.combinations(enumerate(active_engines), num_failed):
                
                # Calculate the hypothetical force this specific combination of engines *should* have produced
                combo_force = np.zeros(3)
                for i, engine in combo:
                    combo_force += engine.thrust_direction * engine.max_thrust * expected_throttles[i]
                    
                error = float(np.linalg.norm(missing_force - combo_force))
                if error < min_error:
                    min_error = error
                    best_combo = [engine.index for i, engine in combo]
                    
        # If the best matching combination is within our tolerance, return it
        if min_error < force_tolerance * 2.0: # Giving some leeway for double failures
            logger.warning(f"[FDI] Engines {best_combo} isolated due to fault detection.")
            return best_combo
            
        # Fallback: if we can't cleanly isolate, return the best guess anyway
        return best_combo
