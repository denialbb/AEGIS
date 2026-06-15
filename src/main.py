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


def _safe_engine_access(part: Any) -> Any:
    """Safely access part.engine, returning None if part has no engine module."""
    if part is None:
        return None
    try:
        return part.engine
    except RuntimeError:
        return None


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
        self.state: str = "STANDBY"

        # Initialize kRPC specifics
        self.vessel = self.conn.space_center.active_vessel
        vessel = self.vessel
        body = vessel.orbit.body

        target_lat = config.TARGET_LAT
        target_lon = config.TARGET_LON

        # Create custom reference frame centered at target lat/lon on the body surface (ADR-017)
        self.ref_frame = self.conn.space_center.ReferenceFrame.create_relative(
            body.reference_frame,
            position=body.surface_position(
                target_lat, target_lon, body.reference_frame
            ),
        )

        # The pad's position in the celestial body's reference frame
        pad_pos = np.array(
            body.surface_position(target_lat, target_lon, body.reference_frame)
        )
        # The up vector points away from the center of the celestial body
        self.up_vector = pad_pos / np.linalg.norm(pad_pos)

        self.engines: List[Engine] = []

        # Discover controllable engines (ADR-016)
        # TODO: Future RCS support: When updating allocator for RCS, query with_tag("AegisRCS") from vessel.parts.rcs
        tagged_parts = vessel.parts.with_tag("AegisEngine")
        if not tagged_parts:
            logger.warning(
                "No parts tagged 'AegisEngine' found. Falling back to all vessel engines."
            )
            # Map kRPC Engine objects back to their parent Part objects
            tagged_parts = [engine.part for engine in vessel.parts.engines]

        for i, part in enumerate(tagged_parts):
            try:
                _ = part.engine  # This throws if part has no engine module
            except RuntimeError:
                continue
            if part.engine is not None:
                pos = np.array(part.position(vessel.reference_frame))
                thrust_dir = np.array(
                    [0.0, 1.0, 0.0]
                )  # Simplified thrust vector
                e = Engine(
                    index=i,
                    position=pos,
                    thrust_direction=thrust_dir,
                    max_thrust=part.engine.max_thrust,
                    part=part,
                )
                part.engine.thrust_limit = 1.0  # Reset thrust limit to 100% in case a previous run set it to 0
                e.active = True
                self.engines.append(e)
        logger.info(f"Discovered {len(self.engines)} Aegis engines.")

        self.allocator: ControlAllocator = ControlAllocator(self.engines)

        self.sensors = SensorModels(
            self.conn, vessel, self.ref_frame, self.up_vector
        )

        # Initialize submodules
        initial_state = np.zeros(6)
        initial_covariance = np.eye(6)
        process_noise = np.eye(6)
        measurement_noise = np.eye(1)

        self.estimator: StateEstimator = StateEstimator(
            initial_state,
            initial_covariance,
            process_noise,
            measurement_noise,
            self.up_vector,
        )
        self.fdi: FaultDetectionIsolation = FaultDetectionIsolation(
            threshold=config.FDI_THRESHOLD
        )

        # Query inertia tensor from kRPC (ADR-028), reshape from row-major list to 3x3
        # Same clean-telemetry caveat as mass (ISS-006 applies)
        self.inertia_tensor: np.ndarray = np.array(
            self.vessel.inertia_tensor
        ).reshape(3, 3)

        # Derive attitude gains from natural frequency and damping ratio (ADR-028)
        omega_n = np.array(config.GUIDANCE_ATT_NATURAL_FREQ)
        zeta = np.array(config.GUIDANCE_ATT_DAMPING_RATIO)
        kp_att = omega_n**2
        kd_att = 2.0 * zeta * omega_n

        # Initialize Guidance Controller with gains from config and proper gravity vector
        self.guidance: GuidanceController = GuidanceController(
            kp_pos_lateral=config.GUIDANCE_KP_POS_LATERAL,
            kp_pos_vertical=config.GUIDANCE_KP_POS_VERTICAL,
            kd_vel_lateral=config.GUIDANCE_KD_VEL_LATERAL,
            kd_vel_vertical=config.GUIDANCE_KD_VEL_VERTICAL,
            kp_att=kp_att,
            kd_att=kd_att,
            gravity=-self.up_vector * 9.81,
            inertia_tensor=self.inertia_tensor,
        )

        self.writer: TelemetryWriter = TelemetryWriter(
            {
                "num_engines": max(len(self.engines), 1),
                "seed": config.RANDOM_SEED,
            }
        )
        self.last_tick_time: float = 0.0

        # State persistence for FDI
        self.expected_throttles: np.ndarray = np.array([])
        self.expected_accel: np.ndarray = np.zeros(3)

        # Ensure the activation action group is toggled OFF when starting
        self.vessel.control.set_action_group(
            config.ACTIVATION_ACTION_GROUP, False
        )

    def _compute_glideslope_target(
        self,
        state_vector: np.ndarray,
        floor_alt: float,
        max_descent_rate: float,
        k_alt_gain: float,
    ) -> np.ndarray:
        """
        Generates an instantaneous target_state for vertical descent guidance
        (ISS-011 fix).

        Rather than commanding a fixed waypoint far below the vehicle's current
        position -- which produces position-error terms in the PD law far
        beyond any realizable acceleration, and gets clipped to zero thrust by
        the allocator (ISS-011) -- this:

          1. Sets the vertical position target to the vehicle's CURRENT
             altitude, so vertical pos_err is always ~0. (Lateral position
             target remains the pad center, origin, so lateral drift is still
             corrected.)
          2. Sets the velocity target to an altitude-proportional descent rate
             that shrinks to 0 as the vehicle approaches floor_alt, capped at
             max_descent_rate.

        The PD law's velocity-error term then dominates a_cmd, producing a
        continuous, bounded deceleration profile instead of an emergent
        coast-then-cliff behavior.

        Args:
            state_vector: (6,) current estimated [x, y, z, vx, vy, vz]
            floor_alt: altitude [m] this phase is descending toward (the
                altitude at which desired descent rate should reach ~0)
            max_descent_rate: cap on commanded descent speed [m/s] (positive)
            k_alt_gain: proportional gain [1/s] mapping altitude-above-floor
                to desired descent speed

        Returns:
            target_state: (6,) [x, y, z, vx, vy, vz]
        """
        est_pos = state_vector[:3]
        est_alt = float(np.dot(est_pos, self.up_vector))

        target_state = np.zeros(6)

        # Vertical pos target == current altitude -> zero vertical pos_err.
        # Lateral pos target == origin (pad center) -> still corrects drift.
        target_state[:3] = est_alt * self.up_vector

        alt_above_floor = max(est_alt - floor_alt, 0.0)
        desired_speed = min(max_descent_rate, k_alt_gain * alt_above_floor)
        target_state[3:] = -self.up_vector * desired_speed

        return target_state

    def run_loop(self) -> None:
        """
        Executes the main loop at 10Hz to 50Hz, polling telemetry,
        updating the estimator, running the FDI, computing control wrench,
        allocating thrust, and transitioning states.
        """
        target_hz = config.TARGET_HZ
        dt = 1.0 / target_hz

        while self.state not in ["HARD_ABORT", "LANDED"]:
            start_time = time.time()
            # Timing and Frame Drop Handling
            # KSP physics runs at 50Hz (20ms). If the game lags or pauses, 'actual_dt' spikes.
            # A massive dt would cause the Kalman Filter's acceleration prediction step (0.5 * a * dt^2)
            # to overshoot wildly and diverge. If we detect a spike > 3x expected dt, we skip the
            # predict step entirely and only use the altimeter update to remain stable.
            if self.last_tick_time > 0:
                actual_dt = start_time - self.last_tick_time
                if actual_dt > 3 * dt:
                    self.writer.log_event(
                        {
                            "type": "DT_SPIKE",
                            "actual_dt": actual_dt,
                            "expected_dt": dt,
                        }
                    )
                    skip_predict = True
                    self.guidance.reset()
                else:
                    skip_predict = False
            else:
                skip_predict = False
            self.last_tick_time = start_time

            # 1. Poll Telemetry
            (
                noisy_alt,
                noisy_accel_body,
                attitude,
                mass,
                aero_body,
                situation,
                angular_velocity,
            ) = self.sensors.poll()

            # Ensure vessel main throttle is at 100% so our individual engine limits work
            if self.state not in [
                "STANDBY",
                "ASCENT_COAST",
                "HARD_ABORT",
                "LANDED",
            ]:
                self.vessel.control.throttle = 1.0

            # 2. Update Estimator
            if not skip_predict:
                self.estimator.predict(noisy_accel_body, attitude, dt)
            state_vector = self.estimator.update(noisy_alt)

            # 2.5 Check fuel state
            for e in self.engines:
                engine_obj = _safe_engine_access(e.part)
                if e.active and engine_obj:
                    if not engine_obj.has_fuel:
                        self.writer.log_event(
                            {"type": "FUEL_EXHAUSTION", "engine_index": e.index}
                        )
                        logger.error(f"Engine {e.index} ran out of fuel!")
                        e.active = False

            # 3. Run FDI (Only during guided flight phases)
            active_engines = [e for e in self.engines if e.active]

            if self.state in [
                "POWERED_DESCENT",
                "HOVER_TARGETING",
                "TERMINAL_DESCENT",
            ]:
                if (
                    len(self.expected_throttles) == 0
                    and len(active_engines) > 0
                ):
                    self.expected_throttles = np.zeros(len(active_engines))

                # ISS-010: Skip FDI detection during dt spikes to avoid spurious fault flags
                # When skip_predict=True, expected_accel may be stale/invalid. Computing fault
                # detection against invalid data causes false positives (e.g., gravity during free-fall
                # looks like multiple engine failures). Hold last known good expected_accel.
                # Also skip FDI when commanded throttles are zero - expected_accel=[0,0,0] but gravity
                # is ~9.8 m/s² which would always trip the fault threshold during coasting.
                # Finally, skip FDI if the vessel is touching the ground (normal force triggers false positive).
                throttles_zero = (len(self.expected_throttles) == 0) or (
                    np.abs(self.expected_throttles).max() < 1e-6
                )
                landed = situation in ("landed", "pre_launch", "splashed")

                if not skip_predict and not throttles_zero and not landed:
                    fault_detected = self.fdi.detect_fault(
                        self.expected_accel, noisy_accel_body
                    )
                    if fault_detected and len(active_engines) > 0:
                        failed_indices = self.fdi.isolate_fault(
                            active_engines,
                            self.expected_throttles,
                            noisy_accel_body,
                            mass,
                        )

                        # ISS-004: Handle multiple simultaneous failures
                        if len(failed_indices) >= 2:
                            logger.error(
                                f"CRITICAL: {len(failed_indices)} engines failed simultaneously. HARD ABORT triggered."
                            )
                            self.writer.log_event(
                                {
                                    "type": "STATE_TRANSITION",
                                    "from": self.state,
                                    "to": "HARD_ABORT",
                                    "reason": "MULTIPLE_FAILURES",
                                }
                            )
                            self.state = "HARD_ABORT"

                        for e in self.engines:
                            if e.index in failed_indices:
                                if e.active:
                                    self.writer.log_event(
                                        {
                                            "type": "FAULT_DETECTED",
                                            "engine_index": e.index,
                                        }
                                    )
                                    e.active = False

                        # Re-evaluate active engines
                        active_engines = [e for e in self.engines if e.active]

                else:
                    logger.debug(
                        f"[FDI] Skipping detection during dt spike, holding stale expected_accel"
                    )

                if not active_engines and len(self.engines) > 0:
                    # If we had engines and they all failed, trigger abort
                    self.state = "HARD_ABORT"
                    logger.error(
                        "CRITICAL: All engines failed. HARD ABORT triggered."
                    )
                    self.writer.log_event(
                        {
                            "type": "STATE_TRANSITION",
                            "from": self.state,
                            "to": "HARD_ABORT",
                            "reason": "ENGINE_FAILURE",
                        }
                    )
                    break

            # 4. State Machine & Control Wrench Computation
            # Use estimated altitude instead of raw telemetry for transitions
            # Altitude is the projection of the state vector onto the up_vector
            est_alt = float(np.dot(state_vector[:3], self.up_vector))
            est_vz = float(np.dot(state_vector[3:], self.up_vector))

            # Smart Activation Logic
            if self.state == "STANDBY":
                activated = self.conn.space_center.active_vessel.control.get_action_group(
                    config.ACTIVATION_ACTION_GROUP
                )
                if activated:
                    logger.info("AEGIS Activated. Smart Routing initialized.")
                    self.writer.log_event({"type": "ACTIVATION"})

                    self.vessel.control.sas = True
                    self.vessel.control.sas_mode = (
                        self.conn.space_center.SASMode.stability_assist
                    )

                    if est_vz > 0:
                        self.state = "ASCENT_COAST"
                    else:
                        self.vessel.control.sas_mode = (
                            self.conn.space_center.SASMode.retrograde
                        )

                        if est_alt > config.ALT_HYPERSONIC:
                            self.state = "DEORBIT_BURN"
                        elif est_alt > config.ALT_POWERED_DESCENT:
                            self.state = "HYPERSONIC_COAST"
                        else:
                            self.state = "POWERED_DESCENT"
                    self.writer.log_event(
                        {
                            "type": "STATE_TRANSITION",
                            "from": "STANDBY",
                            "to": self.state,
                        }
                    )

            target_state = np.zeros(6)  # [x, y, z, vx, vy, vz]

            # Simple state transition cascade based on altitude above the landing pad.
            # In a real system, these would also check velocity gates and fuel margins,
            # but for this prototype we rely purely on altitude boundaries.
            if self.state == "ASCENT_COAST" and est_vz < 0:
                logger.info("Apex reached. Transitioning from ASCENT_COAST.")
                self.vessel.control.sas_mode = (
                    self.conn.space_center.SASMode.retrograde
                )

                if est_alt > config.ALT_HYPERSONIC:
                    self.state = "HYPERSONIC_COAST"
                else:
                    self.state = "POWERED_DESCENT"
                self.writer.log_event(
                    {
                        "type": "STATE_TRANSITION",
                        "from": "ASCENT_COAST",
                        "to": self.state,
                    }
                )
            elif (
                self.state == "DEORBIT_BURN" and est_alt < config.ALT_HYPERSONIC
            ):
                logger.info(
                    "Transitioning from DEORBIT_BURN to HYPERSONIC_COAST"
                )
                self.writer.log_event(
                    {
                        "type": "STATE_TRANSITION",
                        "from": self.state,
                        "to": "HYPERSONIC_COAST",
                    }
                )
                self.state = "HYPERSONIC_COAST"
            elif (
                self.state == "HYPERSONIC_COAST"
                and est_alt < config.ALT_POWERED_DESCENT
            ):
                logger.info(
                    "Transitioning from HYPERSONIC_COAST to POWERED_DESCENT"
                )
                self.writer.log_event(
                    {
                        "type": "STATE_TRANSITION",
                        "from": self.state,
                        "to": "POWERED_DESCENT",
                    }
                )
                self.state = "POWERED_DESCENT"
            elif self.state == "POWERED_DESCENT" and est_alt < config.ALT_HOVER:
                logger.info(
                    "Transitioning from POWERED_DESCENT to HOVER_TARGETING"
                )
                self.writer.log_event(
                    {
                        "type": "STATE_TRANSITION",
                        "from": self.state,
                        "to": "HOVER_TARGETING",
                    }
                )
                self.vessel.control.sas_mode = (
                    self.conn.space_center.SASMode.stability_assist
                )
                self.state = "HOVER_TARGETING"
            elif (
                self.state == "HOVER_TARGETING"
                and est_alt < config.ALT_TERMINAL
            ):
                logger.info(
                    "Transitioning from HOVER_TARGETING to TERMINAL_DESCENT"
                )
                self.writer.log_event(
                    {
                        "type": "STATE_TRANSITION",
                        "from": self.state,
                        "to": "TERMINAL_DESCENT",
                    }
                )
                self.vessel.control.sas_mode = (
                    self.conn.space_center.SASMode.retrograde
                )
                self.state = "TERMINAL_DESCENT"
            elif self.state == "TERMINAL_DESCENT":
                self.vessel.control.sas_mode = (
                    self.conn.space_center.SASMode.retrograde
                )
                if situation in ("landed", "splashed"):
                    logger.info("Touchdown detected. Transitioning to LANDED")
                    self.writer.log_event(
                        {
                            "type": "STATE_TRANSITION",
                            "from": self.state,
                            "to": "LANDED",
                        }
                    )
                    self.state = "LANDED"
                elif situation == "destroyed":
                    logger.error(
                        "Vessel destroyed during terminal descent. Transitioning to HARD_ABORT"
                    )
                    self.writer.log_event(
                        {
                            "type": "STATE_TRANSITION",
                            "from": self.state,
                            "to": "HARD_ABORT",
                            "reason": "VESSEL_DESTROYED",
                        }
                    )
                    self.state = "HARD_ABORT"
            elif self.state not in [
                "STANDBY",
                "ASCENT_COAST",
                "DEORBIT_BURN",
                "HYPERSONIC_COAST",
                "POWERED_DESCENT",
                "HOVER_TARGETING",
                "TERMINAL_DESCENT",
                "LANDED",
            ]:
                self.state = "HARD_ABORT"

            # Define instantaneous target kinematic state based on the current mission phase.
            # The GuidanceController will attempt to reduce the error between this target and current_state to 0.
            if self.state in [
                "STANDBY",
                "ASCENT_COAST",
                "DEORBIT_BURN",
                "HYPERSONIC_COAST",
            ]:
                # Unguided phases in this prototype. Zero out control authority.
                pass
            elif self.state == "POWERED_DESCENT":
                # ISS-011: glide-slope target instead of static up_vector*500.0
                target_state = self._compute_glideslope_target(
                    state_vector,
                    floor_alt=config.ALT_HOVER,
                    max_descent_rate=config.GLIDESLOPE_RATE_POWERED_DESCENT,
                    k_alt_gain=config.GLIDESLOPE_K_ALT,
                )
            elif self.state == "HOVER_TARGETING":
                # ISS-011: glide-slope target instead of static up_vector*50.0
                target_state = self._compute_glideslope_target(
                    state_vector,
                    floor_alt=config.ALT_TERMINAL,
                    max_descent_rate=config.GLIDESLOPE_RATE_HOVER,
                    k_alt_gain=config.GLIDESLOPE_K_ALT,
                )
            elif self.state == "TERMINAL_DESCENT":
                if situation in ("landed", "splashed"):
                    self.writer.log_event(
                        {
                            "type": "STATE_TRANSITION",
                            "from": self.state,
                            "to": "LANDED",
                        }
                    )
                    logger.info(
                        f"Transitioning from {self.state} to LANDED. Touchdown confirmed."
                    )
                    self.state = "LANDED"
                else:
                    # ISS-011: glide-slope target instead of static pos=0 / vel=-2
                    target_state = self._compute_glideslope_target(
                        state_vector,
                        floor_alt=0.0,
                        max_descent_rate=config.GLIDESLOPE_RATE_TERMINAL,
                        k_alt_gain=config.GLIDESLOPE_K_ALT,
                    )
            elif self.state == "LANDED":
                logger.info(
                    "Vessel is landed. Shutting down engines and concluding mission."
                )
                for engine in active_engines:
                    engine_obj = _safe_engine_access(engine.part)
                    if engine_obj:
                        engine_obj.thrust_limit = 0.0
                        engine_obj.independent_throttle = False
                break  # Exit the control loop gracefully

            if self.state == "HARD_ABORT":
                # In a hard abort, we must immediately kill all commanded thrust to prevent unpredictable spins
                self.vessel.control.throttle = 0.0
                for e in self.engines:
                    engine_obj = _safe_engine_access(e.part)
                    if engine_obj:
                        engine_obj.thrust_limit = 0.0
                        engine_obj.independent_throttle = False
                break  # Exit the control loop gracefully

            if self.state not in [
                "HARD_ABORT",
                "STANDBY",
                "ASCENT_COAST",
                "DEORBIT_BURN",
                "HYPERSONIC_COAST",
            ]:
                # Ensure engines are activated and decoupled from vessel main throttle
                for e in active_engines:
                    engine_obj = _safe_engine_access(e.part)
                    if engine_obj:
                        if not engine_obj.active:
                            engine_obj.active = True
                        # Independent throttle MUST be enabled so the engine ignores the vessel's
                        # main throttle and responds strictly to the thrust_limit we command.
                        if not engine_obj.independent_throttle:
                            engine_obj.independent_throttle = True
                        # ADR-023: Independent throttle uncouples from main vessel throttle.
                        # It defaults to 0.0, so we MUST explicitly force it to 1.0 before modulating thrust_limit.
                        engine_obj.throttle = 1.0

                        # Enable Gimbal Trim mod if present
                        for module in e.part.modules:
                            if module.name == "ModuleGimbalTrim":
                                if (
                                    "Toggle Trim" in module.events
                                    and "Gimbal X" not in module.fields
                                ):
                                    module.trigger_event("Toggle Trim")

                desired_wrench = self.guidance.compute_wrench(
                    current_state=state_vector,
                    current_attitude=attitude,
                    mass=mass,
                    target_state=target_state,
                    up_vector=self.up_vector,
                    dt=dt,
                    angular_velocity=angular_velocity,
                )
            else:
                desired_wrench = np.zeros(6)

            # 5. Allocate Thrust
            if active_engines and self.state not in [
                "HARD_ABORT",
                "STANDBY",
                "ASCENT_COAST",
                "DEORBIT_BURN",
                "HYPERSONIC_COAST",
                "LANDED",
            ]:
                try:
                    throttles, gimbals = self.allocator.allocate(
                        desired_wrench, active_engines
                    )

                    # Apply EMA to individual engine throttles to model spool-up dynamically
                    # We MUST use alpha=0.95 to match the physical KSP engine spool-up time.
                    # A faster EMA causes the expected acceleration to outpace the real engines, triggering false FDI faults!
                    alpha = 0.95
                    expected_force = np.zeros(3)
                    new_expected_throttles = []
                    current_gimbals = np.zeros((max(len(self.engines), 1), 2))

                    for i, engine in enumerate(active_engines):
                        # Update engine's internal EMA state
                        engine.expected_throttle = (
                            alpha * engine.expected_throttle
                            + (1 - alpha) * throttles[i]
                        )
                        new_expected_throttles.append(engine.expected_throttle)

                        expected_force += (
                            engine.thrust_direction
                            * engine.max_thrust
                            * engine.expected_throttle
                        )

                        gimbal_x_rad = gimbals[i, 0]
                        gimbal_y_rad = gimbals[i, 1]
                        current_gimbals[engine.index, 0] = gimbal_x_rad
                        current_gimbals[engine.index, 1] = gimbal_y_rad

                        # Apply thrust limit directly to kRPC part (critical actuation step)
                        engine_obj = _safe_engine_access(engine.part)
                        if engine_obj:
                            # Send the INSTANTANEOUS commanded throttle to the physical engine.
                            # KSP will physically spool the engine. We do not want to send the
                            # EMA 'expected_throttle' here, or it artificially double-spools!
                            engine_obj.thrust_limit = float(throttles[i])

                            # Apply independent gimbal trimming via the mod
                            for module in engine.part.modules:
                                if module.name == "ModuleGimbalTrim":
                                    if "Gimbal X" in module.fields:
                                        # Allocator outputs radians. Module takes degrees. Limit to +/- 5 degrees.
                                        g_x = np.clip(
                                            np.degrees(gimbal_x_rad), -5.0, 5.0
                                        )
                                        g_y = np.clip(
                                            np.degrees(gimbal_y_rad), -5.0, 5.0
                                        )
                                        module.set_field_float(
                                            "Gimbal X", float(g_x)
                                        )
                                        module.set_field_float(
                                            "Gimbal Y", float(g_y)
                                        )

                    self.current_gimbals = current_gimbals
                    self.expected_throttles = np.array(new_expected_throttles)

                    # Expected acceleration is total force (thrust + aerodynamic) divided by mass
                    self.expected_accel = (expected_force + aero_body) / mass
                except AllocationDegenerateError as e:
                    print(f"CRITICAL: {str(e)}. HARD ABORT triggered.")
                    self.writer.log_event(
                        {
                            "type": "STATE_TRANSITION",
                            "from": self.state,
                            "to": "HARD_ABORT",
                            "reason": "DEGENERATE_ALLOCATION",
                        }
                    )
                    self.state = "HARD_ABORT"

            # 5.5. Log Telemetry
            throttles = (
                self.expected_throttles
                if len(self.expected_throttles) > 0
                else np.zeros(max(len(self.engines), 1))
            )
            gimbals = (
                self.current_gimbals
                if hasattr(self, "current_gimbals")
                else np.zeros((max(len(self.engines), 1), 2))
            )

            fuel_state = np.zeros(max(len(self.engines), 1))
            for eng in self.engines:
                engine_obj = _safe_engine_access(eng.part)
                if engine_obj:
                    fuel_state[eng.index] = 1.0 if engine_obj.has_fuel else 0.0

            frame = TelemetryFrame(
                timestamp=start_time,
                altitude=noisy_alt,
                velocity=state_vector[
                    3:
                ],  # ISS-011 fix: was np.zeros(3), hid the KF velocity estimate
                noisy_accel=noisy_accel_body,
                throttles=throttles,
                fuel_state=fuel_state,
                gimbals=gimbals,
                skip_predict=skip_predict,  # ISS-010: Log skip_predict state for debugging
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
    parser.add_argument(
        "--debug", action="store_true", help="Enable debug logging"
    )
    parser.add_argument(
        "--log-to-file",
        action="store_true",
        help="Log application strings to file",
    )
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
        try:
            director.run_loop()
        except krpc.error.RPCError as e:
            logger.error(
                f"kRPC Error: {str(e)}. Vessel may have been destroyed. Exiting gracefully."
            )
        except Exception as e:
            logger.error(f"Unexpected error: {str(e)}. Exiting gracefully.")
        finally:
            logger.info("Mission Director shutdown. Closing telemetry streams.")
            director.writer.close()
    except ConnectionError:
        logger.error(
            f"Failed to connect to KSP at {address}. Ensure the server is running and KRPC_ADDRESS is set."
        )
