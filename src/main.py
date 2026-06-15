import math
import time
import os
import signal
import krpc
import logging
import numpy as np
import argparse
import sys
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
from src.common.hud import HudDisplay


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
                    max_gimbal_deg=part.engine.gimbal_range,
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
        # Query vessel velocity for Kalman filter initial state (one-time startup, not stream)
        initial_state = np.zeros(6)
        initial_state[3:] = np.array(vessel.flight(self.ref_frame).velocity)
        initial_covariance = np.eye(6)
        process_noise = np.eye(6)
        measurement_noise_alt = np.eye(1)
        measurement_noise_vel = np.eye(3) * (config.SIGMA_VEL**2)

        self.estimator: StateEstimator = StateEstimator(
            initial_state,
            initial_covariance,
            process_noise,
            measurement_noise_alt,
            measurement_noise_vel,
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

        # Initialize Guidance Controller with gains from config and placeholder gravity
        # Gravity will be updated each loop with dynamic gravity from kRPC
        self.guidance: GuidanceController = GuidanceController(
            kp_pos_lateral=config.GUIDANCE_KP_POS_LATERAL,
            kp_pos_vertical=config.GUIDANCE_KP_POS_VERTICAL,
            kd_vel_lateral=config.GUIDANCE_KD_VEL_LATERAL,
            kd_vel_vertical=config.GUIDANCE_KD_VEL_VERTICAL,
            kp_att=kp_att,
            kd_att=kd_att,
            gravity=np.zeros(3),
            inertia_tensor=self.inertia_tensor,
            accel_clamp_factor=config.ACCEL_CLAMP_FACTOR,
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

        # Accumulated angular motion penalty metric for Optuna tuning
        self.total_angular_motion: float = 0.0

        # Ensure the activation action group is toggled OFF when starting
        self.vessel.control.set_action_group(
            config.ACTIVATION_ACTION_GROUP, False
        )

        self._exit_requested: bool = False

        self._dt_spike_count: int = 0
        self._alloc_cond: float = 0.0
        self._saturated_engines_set: set[int] = set()

        self.hud: HudDisplay = HudDisplay(max(len(self.engines), 1))

    def _compute_glideslope_target(
        self,
        state_vector: np.ndarray,
        floor_alt: float,
        max_descent_rate: float,
        a_avail: float,
    ) -> np.ndarray:
        """
        Generates an instantaneous target_state for vertical descent guidance
        using a suicide-burn sqrt profile.

        Instead of a linear k_alt * alt_above_floor profile that saturates at
        max_descent_rate at high altitude (producing a velocity error the PD
        law cannot overcome), this uses:

            v_target = sqrt(2 * a_avail * alt_above_floor)

        which is the exact velocity of a constant-deceleration trajectory that
        reaches zero speed at floor_alt.  a_avail is the vessel's net upward
        acceleration (from actual TWR), so the target always matches the
        vehicle's braking capability.

        Args:
            state_vector: (6,) current estimated [x, y, z, vx, vy, vz]
            floor_alt: altitude [m] this phase is descending toward
            max_descent_rate: structural/terminal-velocity cap [m/s]
            a_avail: net upward acceleration available from TWR [m/s^2]

        Returns:
            target_state: (6,) [x, y, z, vx, vy, vz]
        """
        est_pos = state_vector[:3]
        est_alt = float(np.dot(est_pos, self.up_vector))

        target_state = np.zeros(6)

        target_state[:3] = est_alt * self.up_vector

        alt_above_floor = max(est_alt - floor_alt, 0.0)
        desired_speed = min(
            max_descent_rate, math.sqrt(2.0 * a_avail * alt_above_floor)
        )
        target_state[3:] = -self.up_vector * desired_speed

        return target_state

    def run_loop(self) -> bool:
        """
        Executes the main loop at 10Hz to 50Hz, polling telemetry,
        updating the estimator, running the FDI, computing control wrench,
        allocating thrust, controlling SAS and transitioning states.

        Returns:
            True if the mission landed successfully, False on HARD_ABORT or failure.
        """
        target_hz = config.TARGET_HZ
        success = False
        dt = 1.0 / target_hz
        ves_orientation = "stability"

        signal.signal(
            signal.SIGINT,
            lambda sig, frame: setattr(self, "_exit_requested", True),
        )
        signal.signal(
            signal.SIGTERM,
            lambda sig, frame: setattr(self, "_exit_requested", True),
        )

        self.hud.start()

        # Landing timer: accumulates while vessel is low and slow, decays when not
        self._landed_timer = 0.0

        while self.state not in ["HARD_ABORT", "LANDED"]:
            start_time = time.time()

            if self._exit_requested:
                logger.info("SIGINT received. Requesting graceful shutdown.")
                self.writer.log_event({"type": "USER_ABORT"})
                self.state = "HARD_ABORT"
                break

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
                    self._dt_spike_count += 1
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
                noisy_vel,
                gravity_world,
            ) = self.sensors.poll()
            # Update guidance controller with dynamic gravity
            self.guidance.gravity = gravity_world
            # Debug raw telemetry for diagnosis
            # TODO
            logger.debug(
                f"Poll: raw_alt={noisy_alt:.2f}, raw_ang_vel={np.linalg.norm(angular_velocity):.3f}"
            )

            # Universal vessel-destroyed check — must be handled for ALL states, not just TERMINAL_DESCENT
            if situation == "destroyed":
                logger.error("Vessel destroyed. Transitioning to HARD_ABORT.")
                self.writer.log_event(
                    {
                        "type": "STATE_TRANSITION",
                        "from": self.state,
                        "to": "HARD_ABORT",
                        "reason": "VESSEL_DESTROYED",
                    }
                )
                self.state = "HARD_ABORT"

            # NOTE: I think player throttle has priority
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
                self.estimator.predict(
                    noisy_accel_body, attitude, dt, gravity_world
                )
            state_vector = self.estimator.update(noisy_alt, noisy_vel)

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
                # Also skip if expected_accel is zero despite non-zero throttles (stale/initialization).
                # Finally, skip FDI if the vessel is touching the ground (normal force triggers false positive).
                throttles_zero = (len(self.expected_throttles) == 0) or (
                    np.abs(self.expected_throttles).max() < 1e-6
                )
                expected_accel_zero = np.linalg.norm(self.expected_accel) < 1e-6
                landed = situation in ("landed", "pre_launch", "splashed")

                if (
                    not skip_predict
                    and not throttles_zero
                    and not expected_accel_zero
                    and not landed
                ):
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

                # Check if remaining engines can provide full 6-DOF control
                sufficient, rank = self.allocator.is_rank_sufficient(
                    active_engines
                )
                if not sufficient:
                    logger.error(
                        f"B matrix rank {rank} < 6 with {len(active_engines)} active engines. "
                        "Insufficient control authority. HARD ABORT triggered."
                    )
                    self.writer.log_event(
                        {
                            "type": "STATE_TRANSITION",
                            "from": self.state,
                            "to": "HARD_ABORT",
                            "reason": "INSUFFICIENT_ENGINES",
                        }
                    )
                    self.state = "HARD_ABORT"
                    break

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
            # Debug KF estimates and up vector sanity
            logger.debug(
                f"KF: est_alt={est_alt:.2f}, est_vz={est_vz:.2f}, "
                f"pos_norm={np.linalg.norm(state_vector[:3]):.2f}, up_norm={np.linalg.norm(self.up_vector):.3f}"
            )
            if config.SAS_PROGRADE_ASCENT:
                if est_vz > 45 and ves_orientation != "prograde":
                    self.vessel.control.sas = True
                    self.vessel.control.sas_mode = (
                        self.conn.space_center.SASMode.prograde
                    )
                    ves_orientation = "prograde"
                elif (
                    not config.USE_SAS
                    and ves_orientation == "prograde"
                    and est_vz <= 35
                ):
                    self.vessel.control.sas = True
                    self.vessel.control.sas_mode = (
                        self.conn.space_center.SASMode.stability_assist
                    )
                    ves_orientation = "stability"
                elif (
                    not config.USE_SAS
                    and ves_orientation == "stability"
                    and est_vz < -10.0
                ):
                    self.vessel.control.sas = False
                    ves_orientation = "off"

            if config.USE_SAS:
                sas_threshold = 40
                if est_vz > sas_threshold and ves_orientation != "prograde":
                    self.vessel.control.sas_mode = (
                        self.conn.space_center.SASMode.prograde
                    )
                    ves_orientation = "prograde"
                elif (
                    sas_threshold > est_vz
                    and est_vz > -sas_threshold
                    and ves_orientation != "stability"
                ):
                    self.vessel.control.sas_mode = (
                        self.conn.space_center.SASMode.stability_assist
                    )
                    ves_orientation = "stability"
                elif (
                    est_vz < -sas_threshold and ves_orientation != "retrograde"
                ):
                    self.vessel.control.sas_mode = (
                        self.conn.space_center.SASMode.retrograde
                    )
                    ves_orientation = "retrograde"

            # Smart Activation Logic
            if self.state == "STANDBY":
                activated = self.conn.space_center.active_vessel.control.get_action_group(
                    config.ACTIVATION_ACTION_GROUP
                )
                if activated:
                    logger.info("AEGIS Activated. Smart Routing initialized.")
                    self.writer.log_event({"type": "ACTIVATION"})
                    if config.USE_SAS:
                        self.vessel.control.sas = True
                    elif config.SAS_PROGRADE_ASCENT:
                        pass  # SAS_PROGRADE_ASCENT block independently manages SAS
                    else:
                        self.vessel.control.sas = False

                    if est_vz > 0:
                        self.state = "ASCENT_COAST"
                    else:

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
                elif est_alt < 5000 and est_vz < 0:
                    logger.info("AEGIS emergency switch.")
                    self.vessel.control.set_action_group(
                        config.ACTIVATION_ACTION_GROUP, True
                    )

            target_state = np.zeros(6)  # [x, y, z, vx, vy, vz]

            # Simple state transition cascade based on altitude above the landing pad.
            # In a real system, these would also check velocity gates and fuel margins,
            # but for this prototype we rely purely on altitude boundaries.
            if self.state == "ASCENT_COAST" and est_vz < 0:
                logger.info("Apex reached. Transitioning from ASCENT_COAST.")

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
                self.state = "TERMINAL_DESCENT"
            elif self.state == "TERMINAL_DESCENT":
                # Determine if the vessel is low and slow enough to count toward landing
                vel_landed = abs(est_vz) < config.LANDED_VEL_THRESHOLD
                alt_landed = abs(noisy_alt) < config.LANDED_ALT_THRESHOLD
                if vel_landed and alt_landed:
                    self._landed_timer += dt
                    # Log the timer progress for HUD visibility
                    logger.debug(
                        f"Landed timer advancing: {self._landed_timer:.3f}s"
                    )
                else:
                    # Decay the timer when conditions are not met
                    self._landed_timer = max(0.0, self._landed_timer - dt)
                # Check if the vessel has been low and slow for the required duration
                if self._landed_timer >= 5.0:
                    self.writer.log_event(
                        {
                            "type": "STATE_TRANSITION",
                            "from": self.state,
                            "to": "LANDED",
                        }
                    )
                    logger.info(
                        f"Transitioning from {self.state} to LANDED. "
                        f"Touchdown confirmed (timer={self._landed_timer:.3f}s)."
                    )
                    self.state = "LANDED"
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

            # Refresh max_thrust from kRPC so a_avail and the allocator both use
            # the current atmospheric-pressure-adjusted thrust (ISS-012).
            for e in active_engines:
                engine_obj = _safe_engine_access(e.part)
                if engine_obj:
                    e.max_thrust = engine_obj.max_thrust

            # Compute available net upward acceleration from actual TWR.
            # Used by both the suicide-burn glideslope and the wrench clamp.
            if active_engines and mass > 0.0:
                total_max_thrust = sum(e.max_thrust for e in active_engines)
                g_mag = float(np.linalg.norm(gravity_world))
                a_avail = max(total_max_thrust / mass - g_mag, 1.0)
            else:
                a_avail = 1.0

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
                target_state = self._compute_glideslope_target(
                    state_vector,
                    floor_alt=config.ALT_HOVER,
                    max_descent_rate=config.GLIDESLOPE_RATE_POWERED_DESCENT,
                    a_avail=a_avail,
                )
            elif self.state == "HOVER_TARGETING":
                target_state = self._compute_glideslope_target(
                    state_vector,
                    floor_alt=config.ALT_TERMINAL,
                    max_descent_rate=config.GLIDESLOPE_RATE_HOVER,
                    a_avail=a_avail,
                )
            elif self.state == "TERMINAL_DESCENT":
                vel_landed = abs(est_vz) < config.LANDED_VEL_THRESHOLD
                alt_landed = abs(noisy_alt) < config.LANDED_ALT_THRESHOLD
                if vel_landed and alt_landed:
                    self._landed_timer += dt
                else:
                    self._landed_timer = max(0.0, self._landed_timer - dt)
                if self._landed_timer >= 5.0:
                    self.writer.log_event(
                        {
                            "type": "STATE_TRANSITION",
                            "from": self.state,
                            "to": "LANDED",
                        }
                    )
                    logger.info(
                        f"Transitioning from {self.state} to LANDED. "
                        f"Touchdown confirmed (timer={self._landed_timer:.3f}s)."
                    )
                    self.state = "LANDED"
                else:
                    target_state = self._compute_glideslope_target(
                        state_vector,
                        floor_alt=0.0,
                        max_descent_rate=config.GLIDESLOPE_RATE_TERMINAL,
                        a_avail=a_avail,
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
                success = True
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

                # Accumulate angular motion for Optuna rocking penalty
                self.total_angular_motion += (
                    float(np.linalg.norm(angular_velocity)) * dt
                )

                desired_wrench = self.guidance.compute_wrench(
                    current_state=state_vector,
                    current_attitude=attitude,
                    mass=mass,
                    target_state=target_state,
                    up_vector=self.up_vector,
                    dt=dt,
                    angular_velocity=angular_velocity,
                    max_a_avail=a_avail,
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

                            # TODO(ADR-029): Augment torque with reaction wheels.
                            # When gimbal authority is weak (low throttle, small
                            # moment arms), map torque_body to stock inputs:
                            #   vessel.control.pitch = clip(desired_wrench[3] * RW_GAIN, -1, 1)
                            #   vessel.control.roll  = clip(desired_wrench[4] * RW_GAIN, -1, 1)
                            #   vessel.control.yaw   = clip(desired_wrench[5] * RW_GAIN, -1, 1)
                            # RW_GAIN converts N·m to [-1, 1]; tune empirically.

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

                    self._alloc_cond = float(
                        np.linalg.cond(self.allocator._build_B(active_engines))
                    )
                    self._saturated_engines_set = set(
                        self.allocator._saturated_engines
                    )

                    # Expected acceleration is total force (thrust + aerodynamic) divided by mass
                    # Guard: if mass is invalid (0 or NaN), keep the last valid expected_accel.
                    # A stale-nonzero value is safer than zero — zero produces FDI false positives.
                    if mass > 0.0:
                        self.expected_accel = (
                            expected_force + aero_body
                        ) / mass
                except AllocationDegenerateError as e:
                    logger.error(f"CRITICAL: {str(e)}. HARD ABORT triggered.")
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

            # 5.6. Update HUD
            if self.hud.is_active:
                gimbals_deg = (
                    np.degrees(gimbals)
                    if gimbals.ndim == 2
                    else np.zeros((max(len(self.engines), 1), 2))
                )
                self.hud.update(
                    {
                        "state": self.state,
                        "altitude": noisy_alt,
                        "est_alt": est_alt,
                        "vertical_vel": est_vz,
                        "lateral_vel": state_vector[3:]
                        - self.up_vector * est_vz,
                        "position": state_vector[:3],
                        "throttles": throttles,
                        "gimbals_deg": gimbals_deg,
                        "fuel_state": fuel_state,
                        "fdi_deviation": (
                            float(
                                np.linalg.norm(
                                    self.expected_accel - noisy_accel_body
                                )
                            )
                            if np.linalg.norm(self.expected_accel) > 1e-6
                            else 0.0
                        ),
                        "alloc_cond": self._alloc_cond,
                        "saturated": self._saturated_engines_set,
                        "kf_cov_pos": (
                            float(self.estimator.kf.P[2, 2])
                            if self.estimator.kf.P.shape[0] > 2
                            else 0.0
                        ),
                        "kf_cov_vel": (
                            float(self.estimator.kf.P[5, 5])
                            if self.estimator.kf.P.shape[0] > 5
                            else 0.0
                        ),
                        "dt_spike_count": self._dt_spike_count,
                        "skip_predict": skip_predict,
                        "active_engine_count": len(active_engines),
                        "total_engine_count": len(self.engines),
                        "mass": mass,
                        "a_avail": a_avail,
                        "angular_velocity_mag": float(
                            np.linalg.norm(angular_velocity)
                        ),
                    }
                )

            # 6. Timing Enforcement
            elapsed = time.time() - start_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        # Cleanup when loop ends
        self.hud.stop()
        self.writer.close()
        return success


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
        success = False
        try:
            success = director.run_loop()
        except krpc.error.RPCError as e:
            logger.error(
                f"kRPC Error: {str(e)}. Vessel may have been destroyed. Exiting gracefully."
            )
        except Exception as e:
            logger.error(f"Unexpected error: {str(e)}. Exiting gracefully.")
        finally:
            logger.info("Mission Director shutdown. Closing telemetry streams.")
            director.writer.close()
        if not success:
            sys.exit(1)
    except ConnectionError:
        logger.error(
            f"Failed to connect to KSP at {address}. Ensure the server is running and KRPC_ADDRESS is set."
        )
        sys.exit(1)
