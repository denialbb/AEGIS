"""AEGIS Mission Director — orchestrates engines, estimation, FDI, guidance, and telemetry."""

import math
import time
import os
import signal
import argparse
import sys
from typing import Any, List, Tuple

import krpc
import logging
import numpy as np

import src.config as config
from src.common.reference_frame import build_ned_frame, get_vessel_state_ned
from src.common.logger import setup_logging
from src.common.engine import Engine
from src.estimation.ekf import ErrorStateEKF
from src.fdi.fdi import FaultDetectionIsolation
from src.guidance.allocator import ControlAllocator, AllocationDegenerateError
from src.guidance.attitude import AttitudeController
from src.guidance.controller import GuidanceController
from src.telemetry.frame import TelemetryFrame
from src.telemetry.writer import TelemetryWriter
from src.telemetry.sensors import SensorModels
from src.common.hud import HudDisplay

logger = logging.getLogger(__name__)


class MissionDirector:
    """State machine orchestrator for the AEGIS autonomous landing system."""

    def __init__(self, conn: Any) -> None:
        self.conn = conn
        self.state: str = "STANDBY"
        self.vessel = conn.space_center.active_vessel
        self._exit_requested: bool = False

        self._init_reference_frame()
        self._init_engines()
        self._init_sensors()
        self._init_estimator()
        self._init_guidance()
        self._init_telemetry()
        self._init_misc_state()

        # Ensure the activation action group is toggled OFF when starting
        self.vessel.control.set_action_group(config.ACTIVATION_ACTION_GROUP, False)

    # ------------------------------------------------------------------
    # Reference frame  (ADR-017)
    # ------------------------------------------------------------------

    def _init_reference_frame(self) -> None:
        """Create a true NED reference frame at the landing target."""
        self.ned_frame, self.up_vector = build_ned_frame(
            self.conn,
            self.vessel.orbit.body,
            config.TARGET_LAT,
            config.TARGET_LON,
        )

    # ------------------------------------------------------------------
    # Engine discovery  (ADR-016)
    # ------------------------------------------------------------------

    def _init_engines(self) -> None:
        """Discover controllable engines tagged 'AegisEngine' and initialise Engine objects."""
        self.engines: List[Engine] = []

        tagged_parts = self.vessel.parts.with_tag("AegisEngine")
        if not tagged_parts:
            logger.warning(
                "No parts tagged 'AegisEngine' found. Falling back to all vessel engines."
            )
            tagged_parts = [engine.part for engine in self.vessel.parts.engines]

        for i, part in enumerate(tagged_parts):
            engine = self._part_to_engine(i, part)
            if engine is not None:
                engine.active = True
                self.engines.append(engine)

        logger.info("Discovered %d Aegis engines.", len(self.engines))
        self.allocator = ControlAllocator(self.engines)

    def _part_to_engine(self, index: int, part: Any) -> Engine | None:
        """Convert a kRPC part to an :class:`Engine`, or None if the part has no engine module."""
        krpc_engine = self._safe_engine_access(part)
        if krpc_engine is None:
            return None

        pos = np.array(part.position(self.vessel.reference_frame))

        thrust_dir = self._resolve_thrust_direction(part, krpc_engine)
        thrust_dir /= np.linalg.norm(thrust_dir) + 1e-12

        gx_v = np.array(
            self.conn.space_center.transform_direction(
                (1.0, 0.0, 0.0), part.reference_frame, self.vessel.reference_frame,
            )
        )
        gy_v = np.array(
            self.conn.space_center.transform_direction(
                (0.0, 1.0, 0.0), part.reference_frame, self.vessel.reference_frame,
            )
        )

        krpc_engine.thrust_limit = 1.0
        e = Engine(
            index=index,
            position=pos,
            thrust_direction=thrust_dir,
            max_thrust=krpc_engine.max_thrust,
            max_gimbal_deg=krpc_engine.gimbal_range,
            part=part,
            gimbal_x_axis=gx_v / np.linalg.norm(gx_v),
            gimbal_y_axis=gy_v / np.linalg.norm(gy_v),
        )
        e.krpc_engine = krpc_engine
        for module in part.modules:
            if module.name == "ModuleGimbalTrim" and "Gimbal X" in module.fields:
                e.gimbal_module = module
                break
        logger.info("E%d %s max_gimbal=%s", e.index, e.position, e.max_gimbal_deg)
        return e

    def _resolve_thrust_direction(self, part: Any, krpc_engine: Any) -> np.ndarray:
        """Determine the thrust direction vector of an engine in the vessel frame."""
        thruster = krpc_engine.thrusters[0]
        try:
            return np.array(thruster.initial_thrust_direction(self.vessel.reference_frame))
        except Exception:
            pass
        try:
            return np.array(thruster.thrust_direction(self.vessel.reference_frame))
        except Exception:
            pass
        fallback = config.PART_THRUST_AXIS.get(part.name, config.DEFAULT_THRUST_AXIS)
        direction = np.array(
            self.conn.space_center.transform_direction(
                fallback, part.reference_frame, self.vessel.reference_frame,
            )
        )
        logger.warning(
            "Thruster API unavailable for %s, using configured axis %s \u2192 %s",
            part.name, fallback, direction,
        )
        return direction

    @staticmethod
    def _safe_engine_access(part: Any) -> Any:
        """Safely access ``part.engine``, returning None if the part has no engine module."""
        if part is None:
            return None
        try:
            return part.engine
        except RuntimeError:
            return None

    # ------------------------------------------------------------------
    # Sensors
    # ------------------------------------------------------------------

    def _init_sensors(self) -> None:
        """Create the sensor stack."""
        self.sensors = SensorModels(self.conn, self.vessel, self.ned_frame, self.up_vector)

    # ------------------------------------------------------------------
    # Estimator + FDI
    # ------------------------------------------------------------------

    def _init_estimator(self) -> None:
        """Initialise the Error-State EKF and FDI module."""
        initial_pos, initial_vel, _alt = get_vessel_state_ned(self.vessel, self.ned_frame)

        covariance = self._build_initial_covariance()

        self.estimator = ErrorStateEKF(initial_pos, initial_vel, covariance, self.up_vector)

        self.fdi = FaultDetectionIsolation(
            threshold=config.FDI_THRESHOLD,
            ekf=self.estimator,
        )

    def _build_initial_covariance(self) -> np.ndarray:
        """Return the 12x12 initial EKF covariance matrix."""
        cov = np.eye(12)
        cov[0:3, 0:3] *= config.SIGMA_ALT**2
        cov[3:6, 3:6] *= config.SIGMA_VEL**2
        cov[6:9, 6:9] *= config.EKF_INITIAL_GYRO_BIAS_UNCERTAINTY**2
        cov[9:12, 9:12] *= config.EKF_INITIAL_ACCEL_BIAS_UNCERTAINTY**2
        return cov

    # ------------------------------------------------------------------
    # Guidance controller
    # ------------------------------------------------------------------

    def _init_guidance(self) -> None:
        """Initialise the guidance controller with dynamic gravity placeholder."""
        # Inertia tensor from kRPC (ADR-028)
        self.inertia_tensor = np.array(self.vessel.inertia_tensor).reshape(3, 3)

        omega_n = np.array(config.GUIDANCE_ATT_NATURAL_FREQ)
        zeta = np.array(config.GUIDANCE_ATT_DAMPING_RATIO)
        kp_att = omega_n**2
        kd_att = 2.0 * zeta * omega_n

        self.guidance = GuidanceController(
            kp_pos_lateral=config.GUIDANCE_KP_POS_LATERAL,
            kp_pos_vertical=config.GUIDANCE_KP_POS_VERTICAL,
            kd_vel_lateral=config.GUIDANCE_KD_VEL_LATERAL,
            kd_vel_vertical=config.GUIDANCE_KD_VEL_VERTICAL,
            kp_att=kp_att,
            kd_att=kd_att,
            gravity_ned=np.zeros(3),
            inertia_tensor=self.inertia_tensor,
            accel_clamp_factor=config.ACCEL_CLAMP_FACTOR,
        )
        self.attitude = AttitudeController()

    # ------------------------------------------------------------------
    # Telemetry writer
    # ------------------------------------------------------------------

    def _init_telemetry(self) -> None:
        """Create the telemetry CSV / event-log writer."""
        self.writer = TelemetryWriter({
            "num_engines": max(len(self.engines), 1),
            "seed": config.RANDOM_SEED,
        })

    # ------------------------------------------------------------------
    # Miscellaneous state
    # ------------------------------------------------------------------

    def _init_misc_state(self) -> None:
        """Initialise remaining director attributes (tracking, debug, persistence)."""
        self.last_tick_time: float = 0.0

        # FDI persistence
        self.expected_throttles: np.ndarray = np.array([])
        self.expected_accel: np.ndarray = np.zeros(3)
        self._expected_forces: np.ndarray = np.zeros((max(len(self.engines), 1), 3))

        # Optuna-tuning metric
        self.total_angular_motion: float = 0.0

        # Tracking
        self._dt_spike_count: int = 0
        self._alloc_cond: float = 0.0
        self._saturated_engines_set: set[int] = set()
        self._diagnostic_axial_forces: np.ndarray = np.zeros(len(self.engines))

        # Debug attributes  (used by scripts/debug_telemetry_detail.py)
        self._dbg_actual_dt: float = 1.0 / config.TARGET_HZ
        self._dbg_sleep_time: float = 0.0
        self._dbg_skip_predict: bool = False
        self._dbg_raw_gyro: np.ndarray = np.zeros(3)
        self._dbg_tick_count: int = 0

        self.hud = HudDisplay(max(len(self.engines), 1))

        # Landing timer (initialised each run in loop.py)
        self._landed_timer: float = 0.0

        # Phase-entry tracking for horizontal-target blending
        self._phase_entry_ticks: int = 0
        self._phase_entry_horizontal: np.ndarray | None = None
        self._early_translation: bool = False

    # ------------------------------------------------------------------
    # Glideslope guidance
    # ------------------------------------------------------------------

    def _compute_glideslope_target(
        self,
        state_vector: np.ndarray,
        floor_alt: float,
        max_descent_rate: float,
        a_avail: float,
        horizontal_target: np.ndarray | None = None,
        min_descent_rate: float = 0.0,
    ) -> np.ndarray:
        """
        Generate a target state for vertical descent using a sqrt profile.

        ``v_target = sqrt(2 * a_avail * alt_above_floor)`` — the exact
        velocity of a constant-deceleration trajectory that reaches zero
        speed at *floor_alt*.

        Args:
            state_vector: (6,) current estimated [x, y, z, vx, vy, vz]
            floor_alt: altitude [m] this phase descends toward
            max_descent_rate: structural / terminal-velocity cap [m/s]
            a_avail: net upward acceleration from TWR [m/s²]
            horizontal_target: optional (2,) target for (north, east).
                               If None, defaults to the landing pad [0, 0].
            min_descent_rate: minimum target descent speed [m/s].
                              Applied within the FRAME-003 cap.  Ensures
                              the vehicle descends even when at near-zero
                              vertical velocity.

        Returns:
            target_state: (6,) [x, y, z, vx, vy, vz]
        """
        est_alt = float(np.dot(state_vector[:3], self.up_vector))
        target = np.zeros(6)
        if horizontal_target is not None:
            target[:2] = horizontal_target
            target[2] = est_alt * self.up_vector[2]
        else:
            target[:3] = est_alt * self.up_vector

        alt_above_floor = max(est_alt - floor_alt, 0.0)
        safe_a_avail = a_avail * 0.7  # 30% margin for PD lag
        desired_speed = min(
            max_descent_rate,
            math.sqrt(2.0 * safe_a_avail * alt_above_floor),
        )
        # FRAME-002: The glideslope target speed must never exceed the current
        # descent rate.  With the correct retrograde attitude (nose ≈ NED -Z),
        # a NED upward acceleration (negative Z) is required for braking.
        # If target_speed > current_speed, the PD velocity error commands
        # downward acceleration → a_cmd_body Y negative → reverse thrust
        # → all throttles clamped to zero (see allocator.py reverse-thrust
        # logic at line 150).
        current_speed = -float(np.dot(state_vector[3:], self.up_vector))
        # FRAME-003: target ratio 0.5 gives ~13 m/s equilibrium vs 0.9 giving ~65 m/s.
        # A lower ratio means the velocity error (target - current) is larger,
        # producing stronger braking from the PD controller.
        # min_descent_rate ensures the vehicle descends even when hovering slowly.
        desired_speed = min(desired_speed, max(current_speed * config.GLIDESLOPE_TARGET_RATIO, min_descent_rate))
        target[3:] = -self.up_vector * desired_speed
        return target

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run_loop(self) -> bool:
        """Thin wrapper delegating to the modular loop implementation."""
        from src.mission.loop import run_mission_loop
        return run_mission_loop(self)


# ======================================================================
# CLI entry-point
# ======================================================================

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AEGIS Mission Director")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    parser.add_argument("--log-to-file", action="store_true", help="Log to file")
    parser.add_argument("--hud", action="store_true", help="Enable HUD display (suppresses terminal logging)")
    return parser.parse_args()


def _apply_args(args: argparse.Namespace) -> None:
    if args.debug:
        config.DEBUG_LOGGING = True
    if args.log_to_file:
        config.LOG_TO_FILE = True
    if args.hud:
        config.HUD_ENABLED = True
        config.LOG_TO_FILE = True
    else:
        config.HUD_ENABLED = False


def _connect_krpc() -> Any:
    """Establish the kRPC connection (WSL2-aware via ADR-015)."""
    address = os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS)
    logger.info("Connecting to KSP at %s ...", address)
    return krpc.connect(name=config.KRPC_CLIENT_NAME, address=address)


def _run_mission() -> bool:
    """Connect, create the director, and run the mission loop."""
    conn = _connect_krpc()
    logger.info("Connected. Starting Mission Director ...")
    director = MissionDirector(conn)
    try:
        return director.run_loop()
    except krpc.error.RPCError as e:
        logger.error("kRPC Error: %s. Vessel may have been destroyed. Exiting.", str(e))
    except Exception as e:
        logger.error("Unexpected error: %s. Exiting.", str(e))
    return False


if __name__ == "__main__":
    _apply_args(_parse_args())
    setup_logging()
    try:
        success = _run_mission()
    except ConnectionError:
        logger.error(
            "Failed to connect to KSP at %s. Ensure the server is running "
            "and KRPC_ADDRESS is set.",
            os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS),
        )
        sys.exit(1)
    if not success:
        sys.exit(1)
