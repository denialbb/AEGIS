import time
import os
import krpc
import logging
import numpy as np
import argparse
from typing import Any, List

import src.config as config
from src.common.logger import setup_logging
from src.common.engine import Engine

logger = logging.getLogger(__name__)
from src.estimation.estimator import StateEstimator
from src.fdi.fdi import FaultDetectionIsolation
from src.guidance.allocator import ControlAllocator, AllocationDegenerateError
from src.guidance.controller import GuidanceController
from src.telemetry.frame import TelemetryFrame
from src.telemetry.writer import TelemetryWriter
from src.telemetry.sensors import SensorModels

class MissionDirector:
    def __init__(self, conn: Any):
        """
        conn: kRPC connection object
        """
        self.conn = conn
        self.state: str = "DEORBIT_BURN"
        
        # Initialize kRPC specifics
        vessel = self.conn.space_center.active_vessel
        body = vessel.orbit.body
        
        target_lat = config.TARGET_LAT
        target_lon = config.TARGET_LON
        
        # Create custom reference frame centered at target lat/lon on the body surface (ADR-017)
        self.ref_frame = self.conn.space_center.ReferenceFrame.create_relative(
            body.reference_frame,
            position=body.surface_position(target_lat, target_lon, body.reference_frame)
        )
        
        self.engines: List[Engine] = []
        
        # Discover controllable engines (ADR-016)
        # TODO: Future RCS support: When updating allocator for RCS, query with_tag("AegisRCS") from vessel.parts.rcs
        tagged_parts = vessel.parts.with_tag("AegisEngine")
        for i, part in enumerate(tagged_parts):
            if part.engine is not None:
                pos = np.array(part.position(vessel.reference_frame))
                thrust_dir = np.array([0.0, 1.0, 0.0]) # Simplified thrust vector
                e = Engine(index=i, position=pos, thrust_direction=thrust_dir, max_thrust=part.engine.max_thrust)
                part.engine.active = True
                e.active = True
                self.engines.append(e)
        logger.info(f"Discovered {len(self.engines)} Aegis engines.")

        self.allocator: ControlAllocator = ControlAllocator(self.engines)
        
        self.sensors = SensorModels(self.conn, vessel, self.ref_frame)
        
        # Initialize submodules
        initial_state = np.zeros(6)
        initial_covariance = np.eye(6)
        process_noise = np.eye(6)
        measurement_noise = np.eye(1)
        
        self.estimator: StateEstimator = StateEstimator(
            initial_state, initial_covariance, process_noise, measurement_noise
        )
        self.fdi: FaultDetectionIsolation = FaultDetectionIsolation(threshold=config.FDI_THRESHOLD)
        
        # Initialize Guidance Controller with default gains and gravity
        self.guidance: GuidanceController = GuidanceController(
            kp_pos=np.array([1.0, 1.0, 1.0]),
            kd_vel=np.array([1.0, 1.0, 1.0]),
            kp_att=np.array([10.0, 10.0, 10.0]),
            kd_att=np.array([5.0, 5.0, 5.0]),
            gravity=np.array([0.0, 0.0, -9.81])  # Default to Earth/Kerbin surface gravity
        )
        
        self.writer: TelemetryWriter = TelemetryWriter({
            "num_engines": max(len(self.engines), 1),
            "seed": config.RANDOM_SEED
        })
        self.last_tick_time: float = 0.0
        
        # State persistence for FDI
        self.expected_throttles: np.ndarray = np.array([])
        self.expected_accel: np.ndarray = np.zeros(3)

    def run_loop(self) -> None:
        """
        Executes the main loop at 10Hz to 50Hz, polling telemetry,
        updating the estimator, running the FDI, computing control wrench,
        allocating thrust, and transitioning states.
        """
        target_hz = config.TARGET_HZ
        dt = 1.0 / target_hz
        
        while self.state != "HARD_ABORT":
            start_time = time.time()
            if self.last_tick_time > 0:
                actual_dt = start_time - self.last_tick_time
                if actual_dt > 3 * dt:
                    self.writer.log_event({"type": "DT_SPIKE", "actual_dt": actual_dt, "expected_dt": dt})
                    skip_predict = True
                else:
                    skip_predict = False
            else:
                skip_predict = False
            self.last_tick_time = start_time
            
            # 1. Poll Telemetry via SensorModels wrapper
            noisy_alt, noisy_accel_body, attitude, mass = self.sensors.poll()
            
            # 2. Update Estimator
            if not skip_predict:
                self.estimator.predict(noisy_accel_body, attitude, dt)
            state_vector = self.estimator.update(noisy_alt)
            
            # 3. Run FDI
            active_engines = [e for e in self.engines if e.active]
            
            if len(self.expected_throttles) == 0 and len(active_engines) > 0:
                self.expected_throttles = np.zeros(len(active_engines))
                
            fault_detected = self.fdi.detect_fault(self.expected_accel, noisy_accel_body)
            if fault_detected and len(active_engines) > 0:
                failed_indices = self.fdi.isolate_fault(
                    active_engines, self.expected_throttles, noisy_accel_body, mass
                )
                
                # ISS-004: Handle multiple simultaneous failures
                if len(failed_indices) >= 2:
                    logger.error(f"CRITICAL: {len(failed_indices)} engines failed simultaneously. HARD ABORT triggered.")
                    self.writer.log_event({"type": "STATE_TRANSITION", "from": self.state, "to": "HARD_ABORT", "reason": "MULTIPLE_FAILURES"})
                    self.state = "HARD_ABORT"
                
                for e in self.engines:
                    if e.index in failed_indices:
                        if e.active:
                            self.writer.log_event({"type": "FAULT_DETECTED", "engine_index": e.index})
                            e.active = False
                
                # Re-evaluate active engines
                active_engines = [e for e in self.engines if e.active]
                
            if not active_engines and len(self.engines) > 0:
                # If we had engines and they all failed, trigger abort
                self.state = "HARD_ABORT"
                logger.error("CRITICAL: All engines failed. HARD ABORT triggered.")
                self.writer.log_event({"type": "STATE_TRANSITION", "from": self.state, "to": "HARD_ABORT", "reason": "ENGINE_FAILURE"})
                break
            
            # 4. State Machine & Control Wrench Computation
            # Use estimated altitude instead of raw telemetry for transitions
            est_alt = state_vector[2]
            
            target_state = np.zeros(6) # [x, y, z, vx, vy, vz]
            
            # Simple state transition for demonstration
            if self.state == "DEORBIT_BURN" and est_alt < 10000:
                logger.info("Transitioning from DEORBIT_BURN to HYPERSONIC_COAST")
                self.writer.log_event({"type": "STATE_TRANSITION", "from": self.state, "to": "HYPERSONIC_COAST"})
                self.state = "HYPERSONIC_COAST"
            elif self.state == "HYPERSONIC_COAST" and est_alt < 2000:
                logger.info("Transitioning from HYPERSONIC_COAST to POWERED_DESCENT")
                self.writer.log_event({"type": "STATE_TRANSITION", "from": self.state, "to": "POWERED_DESCENT"})
                self.state = "POWERED_DESCENT"
            elif self.state == "POWERED_DESCENT" and est_alt < 500:
                logger.info("Transitioning from POWERED_DESCENT to HOVER_TARGETING")
                self.writer.log_event({"type": "STATE_TRANSITION", "from": self.state, "to": "HOVER_TARGETING"})
                self.state = "HOVER_TARGETING"
            elif self.state == "HOVER_TARGETING" and est_alt < 50:
                logger.info("Transitioning from HOVER_TARGETING to TERMINAL_DESCENT")
                self.writer.log_event({"type": "STATE_TRANSITION", "from": self.state, "to": "TERMINAL_DESCENT"})
                self.state = "TERMINAL_DESCENT"
            elif self.state not in ["DEORBIT_BURN", "HYPERSONIC_COAST", "POWERED_DESCENT", "HOVER_TARGETING", "TERMINAL_DESCENT"]:
                self.state = "HARD_ABORT"
                
            # Define target state based on phase
            if self.state == "TERMINAL_DESCENT":
                target_state[5] = -2.0 # Descend at 2 m/s
            elif self.state == "HOVER_TARGETING":
                target_state[2] = 50.0 # Maintain altitude at 50m
                
            if self.state != "HARD_ABORT":
                desired_wrench = self.guidance.compute_wrench(
                    current_state=state_vector,
                    current_attitude=attitude,
                    mass=mass,
                    target_state=target_state,
                    dt=dt
                )
            else:
                desired_wrench = np.zeros(6)
                
            # 5. Allocate Thrust
            if active_engines and self.state != "HARD_ABORT":
                try:
                    throttles, gimbals = self.allocator.allocate(desired_wrench, active_engines)
                    self.expected_throttles = throttles
                    
                    # TODO(ISS-008): Compute actual expected acceleration using thrust allocation and mass.
                    self.expected_accel = np.zeros(3) 
                except AllocationDegenerateError as e:
                    print(f"CRITICAL: {str(e)}. HARD ABORT triggered.")
                    self.writer.log_event({"type": "STATE_TRANSITION", "from": self.state, "to": "HARD_ABORT", "reason": "DEGENERATE_ALLOCATION"})
                    self.state = "HARD_ABORT"
                
            # 5.5. Log Telemetry
            throttles = self.expected_throttles if len(self.expected_throttles) > 0 else np.zeros(max(len(self.engines), 1))
            gimbals = np.zeros((max(len(self.engines), 1), 2))
            
            frame = TelemetryFrame(
                timestamp=start_time,
                altitude=noisy_alt,
                velocity=np.zeros(3),
                noisy_accel=noisy_accel_body,
                throttles=throttles,
                gimbals=gimbals
            )
            self.writer.log_tick(frame)

            # 6. Timing Enforcement
            elapsed = time.time() - start_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        # Cleanup when loop ends
        self.writer.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AEGIS Mission Director")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    parser.add_argument("--log-to-file", action="store_true", help="Log application strings to file")
    args = parser.parse_args()
    
    if args.debug:
        config.DEBUG_LOGGING = True
    if args.log_to_file:
        config.LOG_TO_FILE = True

    setup_logging()
    
    # WSL2 Connection Topology (ADR-015)
    address = os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS)
    logger.info(f"Connecting to KSP at {address}...")
    try:
        conn = krpc.connect(name=config.KRPC_CLIENT_NAME, address=address)
        logger.info("Connected. Starting Mission Director...")
        director = MissionDirector(conn)
        director.run_loop()
    except ConnectionError:
        logger.error(f"Failed to connect to KSP at {address}. Ensure the server is running and KRPC_ADDRESS is set.")
