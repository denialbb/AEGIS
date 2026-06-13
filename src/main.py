import time
import numpy as np
from typing import Any, List

from src.common.engine import Engine
from src.estimation.estimator import StateEstimator
from src.fdi.fdi import FaultDetectionIsolation
from src.guidance.allocator import ControlAllocator

class MissionDirector:
    def __init__(self, conn: Any):
        """
        conn: kRPC connection object
        """
        self.conn = conn
        self.state: str = "DEORBIT_BURN"
        
        # Initialize submodules
        initial_state = np.zeros(7)
        initial_covariance = np.eye(7)
        process_noise = np.eye(7)
        measurement_noise = np.eye(4)
        
        self.estimator: StateEstimator = StateEstimator(
            initial_state, initial_covariance, process_noise, measurement_noise
        )
        self.fdi: FaultDetectionIsolation = FaultDetectionIsolation(threshold=0.5)
        
        self.engines: List[Engine] = [] 
        self.allocator: ControlAllocator = ControlAllocator(self.engines)
        
        # State persistence for FDI
        self.expected_throttles: np.ndarray = np.array([])
        self.expected_accel: np.ndarray = np.zeros(3)

    def run_loop(self) -> None:
        """
        Executes the main loop at 10Hz to 50Hz, polling telemetry,
        updating the estimator, running the FDI, computing control wrench,
        allocating thrust, and transitioning states.
        """
        target_hz = 50.0
        dt = 1.0 / target_hz
        
        while self.state != "HARD_ABORT":
            start_time = time.time()
            
            # 1. Poll Telemetry
            # In a complete implementation, streams via self.conn would be read here
            noisy_alt: float = 0.0
            noisy_accel: np.ndarray = np.zeros(3)
            mass: float = 1.0
            
            # 2. Update Estimator
            state_vector = self.estimator.update(noisy_alt, noisy_accel, dt)
            
            # 3. Run FDI
            active_engines = [e for e in self.engines if e.active]
            
            if len(self.expected_throttles) == 0 and len(active_engines) > 0:
                self.expected_throttles = np.zeros(len(active_engines))
                
            fault_detected = self.fdi.detect_fault(self.expected_accel, noisy_accel)
            if fault_detected and len(active_engines) > 0:
                failed_indices = self.fdi.isolate_fault(
                    active_engines, self.expected_throttles, noisy_accel, mass
                )
                for e in self.engines:
                    if e.index in failed_indices:
                        e.active = False
                
                # Re-evaluate active engines
                active_engines = [e for e in self.engines if e.active]
                
            if not active_engines and len(self.engines) > 0:
                # If we had engines and they all failed, trigger abort
                self.state = "HARD_ABORT"
                print("CRITICAL: All engines failed. HARD ABORT triggered.")
                break
            
            # 4. State Machine & Control Wrench Computation
            desired_wrench = np.zeros(6)
            
            if self.state == "DEORBIT_BURN":
                pass
            elif self.state == "HYPERSONIC_COAST":
                pass
            elif self.state == "POWERED_DESCENT":
                pass
            elif self.state == "HOVER_TARGETING":
                pass
            elif self.state == "TERMINAL_DESCENT":
                pass
            else:
                self.state = "HARD_ABORT"
                
            # 5. Allocate Thrust
            if active_engines and self.state != "HARD_ABORT":
                throttles, gimbals = self.allocator.allocate(desired_wrench, active_engines)
                self.expected_throttles = throttles
                
                # Mock expected acceleration update based on new throttles
                self.expected_accel = np.zeros(3) 
                
            # 6. Timing Enforcement
            elapsed = time.time() - start_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
            # Break in this skeleton to avoid infinite loop when tested directly
            break
