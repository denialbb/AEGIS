"""Flight-critical control functions for the mission loop.

Responsibilities: SAS, FDI, state machine, allocation, engine management,
estimator integration, throttle control, and abort handling.
"""

import logging
from typing import Any

import math
import numpy as np
from scipy.spatial.transform import Rotation as R  # type: ignore

import src.config as config
from src.guidance.allocator import AllocationDegenerateError

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
#  State-machine constants
# ---------------------------------------------------------------------------

VALID_STATES = frozenset({
    "STANDBY", "ASCENT_COAST", "DEORBIT_BURN", "HYPERSONIC_COAST",
    "SENSOR_WARMUP", "ESTIMATOR_WARMUP",
    "POWERED_DESCENT", "HOVER_TARGETING", "TERMINAL_DESCENT", "LANDED",
})

UNGUIDED_STATES = frozenset({
    "STANDBY", "ASCENT_COAST", "DEORBIT_BURN", "HYPERSONIC_COAST",
    "SENSOR_WARMUP", "ESTIMATOR_WARMUP",
})

# ---------------------------------------------------------------------------
#  Vessel-destroyed check
# ---------------------------------------------------------------------------

def check_vessel_destroyed(director: Any, data: dict) -> bool:
    """Transition to HARD_ABORT if the vessel has been destroyed.  Returns True if destroyed."""
    if data["situation"] != "destroyed":
        return False
    logger.error("Vessel destroyed. Transitioning to HARD_ABORT.")
    director.writer.log_event({
        "type": "STATE_TRANSITION",
        "from": director.state,
        "to": "HARD_ABORT",
        "reason": "VESSEL_DESTROYED",
    })
    director.state = "HARD_ABORT"
    return True


def check_catastrophic_impact(director: Any, data: dict, est_alt: float, est_vz: float) -> bool:
    """Detect catastrophic impact: ground contact with excessive velocity.
    
    Returns True if catastrophic failure detected and transitions to HARD_ABORT.
    """
    # Check if at or below ground level with high descent rate
    if est_alt <= 0.0 and est_vz < -20.0:
        logger.error(
            f"CATASTROPHIC FAILURE: Impact at {est_alt:.1f}m altitude with "
            f"{abs(est_vz):.1f} m/s descent rate (>20 m/s threshold). "
            "Transitioning to HARD_ABORT."
        )
        director.writer.log_event({
            "type": "CATASTROPHIC_FAILURE",
            "altitude": est_alt,
            "impact_velocity": est_vz,
            "state": director.state,
        })
        director.writer.log_event({
            "type": "STATE_TRANSITION",
            "from": director.state,
            "to": "HARD_ABORT",
            "reason": "CATASTROPHIC_IMPACT",
        })
        director.state = "HARD_ABORT"
        return True
    return False


def check_vessel_situation(director: Any, data: dict) -> str | None:
    """Check vessel situation and return the state to transition to, or None.

    Uses the kRPC situation directly — more reliable than estimated states
    for detecting landing or destruction.
    """
    sit = data["situation"]
    if sit == "destroyed":
        logger.error("Vessel destroyed via situation check.")
        return "HARD_ABORT"
    if sit in ("landed", "splashed", "pre_launch"):
        logger.info("Vessel situation is '%s'. Transitioning to LANDED.", sit)
        return "LANDED"
    return None

# ---------------------------------------------------------------------------
#  Master throttle
# ---------------------------------------------------------------------------

def set_throttle_for_state(director: Any) -> None:
    """Set master throttle to 1.0 during powered phases."""
    if director.state in ("STANDBY", "ASCENT_COAST", "HARD_ABORT", "LANDED"):
        return
    director.vessel.control.throttle = 1.0

# ---------------------------------------------------------------------------
#  EKF estimation
# ---------------------------------------------------------------------------

def update_estimator(
    director: Any, data: dict, ekf_dt: float, skip_predict: bool
) -> np.ndarray:
    """Run EKF predict + update.  Returns the 6-D state vector."""
    director.sensors.attitude_estimator.set_gyro_bias(
        director.estimator.get_gyro_bias()
    )
    if not skip_predict:
        director.estimator.predict(
            data["sf_body"],
            data["omega_body"],
            data["attitude"],
            data["gravity_ned"],
            ekf_dt,
        )
    return director.estimator.update(data["noisy_alt"], data["noisy_vel"])

# ---------------------------------------------------------------------------
#  IMU health  (FDI)
# ---------------------------------------------------------------------------

def check_imu_health(director: Any) -> None:
    """Log a warning if FDI detects an IMU fault via innovation monitoring."""
    if director.fdi.detect_imu_fault():
        logger.warning("IMU fault detected via EKF innovation monitoring")

# ---------------------------------------------------------------------------
#  Fuel-state check
# ---------------------------------------------------------------------------

def check_fuel_state(director: Any, alt: float = 0.0) -> None:
    """Deactivate engines that have run out of fuel."""
    for e in director.engines:
        engine_obj = director._safe_engine_access(e.part)
        if e.active and engine_obj and not engine_obj.has_fuel:
            e.active = False
            if alt > 15.0:
                director.writer.log_event({
                    "type": "FUEL_EXHAUSTION",
                    "engine_index": e.index,
                })
                logger.error("Engine %d ran out of fuel!", e.index)

# ---------------------------------------------------------------------------
#  Fault Detection & Isolation
# ---------------------------------------------------------------------------

def run_fdi(
    director: Any, data: dict, mass: float, skip_predict: bool
) -> None:
    """Run FDI during guided phases.  Deactivates failed engines and may set HARD_ABORT."""
    if director.state not in (
        "POWERED_DESCENT", "HOVER_TARGETING", "TERMINAL_DESCENT",
    ):
        return

    active_engines = [e for e in director.engines if e.active]
    if len(director.expected_throttles) == 0 and len(active_engines) > 0:
        director.expected_throttles = np.zeros(len(active_engines))

    throttles_zero = (
        len(director.expected_throttles) == 0
        or np.abs(director.expected_throttles).max() < 1e-6
    )
    accel_zero = np.linalg.norm(director.expected_accel) < 1e-6
    landed = data["situation"] in ("landed", "pre_launch", "splashed")

    if skip_predict or throttles_zero or accel_zero or landed:
        logger.debug("[FDI] Skipping detection during dt spike / stale accel / landed")
        return

    fault_detected = director.fdi.detect_fault(
        director.expected_accel, data["sf_body"]
    )
    failed_indices: set[int] = set()
    if fault_detected and len(active_engines) > 0:
        fdi_forces = np.array([
            director._expected_forces[e.index] for e in active_engines
        ])
        failed_indices = director.fdi.isolate_fault(
            active_engines,
            director.expected_throttles,
            data["sf_body"],
            mass,
            expected_forces=fdi_forces,
        )

    if len(failed_indices) >= 2:
        logger.error(
            "CRITICAL: %d engines failed simultaneously. HARD ABORT triggered.",
            len(failed_indices),
        )
        director.writer.log_event({
            "type": "STATE_TRANSITION",
            "from": director.state,
            "to": "HARD_ABORT",
            "reason": "MULTIPLE_FAILURES",
        })
        director.state = "HARD_ABORT"

    for e in director.engines:
        if e.index in failed_indices and e.active:
            director.writer.log_event({
                "type": "FAULT_DETECTED",
                "engine_index": e.index,
            })
            e.active = False

# ---------------------------------------------------------------------------
#  Control-authority checks
# ---------------------------------------------------------------------------

def check_control_authority(director: Any, active_engines: list) -> bool:
    """Verify B-matrix rank and active engine count.  Returns True if HARD_ABORT needed."""
    if director.state not in (
        "POWERED_DESCENT", "HOVER_TARGETING", "TERMINAL_DESCENT",
    ):
        return False

    sufficient, rank = director.allocator.is_rank_sufficient(active_engines)
    if not sufficient:
        logger.error(
            "B matrix rank %d < 6 with %d active engines — insufficient control authority.",
            rank, len(active_engines),
        )
        director.writer.log_event({
            "type": "STATE_TRANSITION",
            "from": director.state,
            "to": "HARD_ABORT",
            "reason": "INSUFFICIENT_ENGINES",
        })
        director.state = "HARD_ABORT"
        return True

    if not active_engines and len(director.engines) > 0:
        logger.error("CRITICAL: All engines failed. HARD ABORT triggered.")
        director.writer.log_event({
            "type": "STATE_TRANSITION",
            "from": director.state,
            "to": "HARD_ABORT",
            "reason": "ENGINE_FAILURE",
        })
        director.state = "HARD_ABORT"
        return True

    return False

# ---------------------------------------------------------------------------
#  SAS mode
# ---------------------------------------------------------------------------

def handle_sas(director: Any, est_vz: float, ves_orientation: str) -> str:
    """Switch SAS mode based on vertical velocity.  Returns the new orientation label."""
    try:
        ves_orientation = _sas_prograde(director, est_vz, ves_orientation)
        ves_orientation = _sas_standard(director, est_vz, ves_orientation)
    except RuntimeError:
        logger.warning("Couldn't set SAS")
    return ves_orientation


def _sas_prograde(director: Any, est_vz: float, ves_orientation: str) -> str:
    """SAS_PROGRADE_ASCENT branch.  Returns updated orientation label."""
    if not config.SAS_PROGRADE_ASCENT:
        return ves_orientation

    if est_vz > 45 and ves_orientation != "prograde":
        director.vessel.control.sas = True
        director.vessel.control.sas_mode = director.conn.space_center.SASMode.prograde
        return "prograde"

    if not config.USE_SAS and ves_orientation == "prograde" and est_vz <= 35:
        director.vessel.control.sas = True
        director.vessel.control.sas_mode = director.conn.space_center.SASMode.stability_assist
        return "stability"

    if not config.USE_SAS and ves_orientation == "stability" and est_vz < -10.0:
        director.vessel.control.sas = False
        return "off"

    return ves_orientation


def _sas_standard(director: Any, est_vz: float, ves_orientation: str) -> str:
    """USE_SAS branch.  Returns updated orientation label."""
    if not config.USE_SAS:
        return ves_orientation

    if director.state in ("HOVER_TARGETING", "TERMINAL_DESCENT", "TERMINAL_LANDING"):
        if ves_orientation != "stability":
            director.vessel.control.sas = True
            director.vessel.control.sas_mode = director.conn.space_center.SASMode.stability_assist
            return "stability"
        return "stability"

    threshold = 40
    if est_vz > threshold and ves_orientation != "prograde":
        director.vessel.control.sas_mode = director.conn.space_center.SASMode.prograde
        return "prograde"
    if threshold > est_vz > -threshold and ves_orientation != "stability":
        director.vessel.control.sas_mode = director.conn.space_center.SASMode.stability_assist
        return "stability"
    if est_vz < -threshold and ves_orientation != "retrograde":
        director.vessel.control.sas_mode = director.conn.space_center.SASMode.retrograde
        return "retrograde"
    return ves_orientation

# ---------------------------------------------------------------------------
#  STANDBY activation
# ---------------------------------------------------------------------------

def handle_standby(director: Any, data: dict, est_alt: float, est_vz: float) -> None:
    """Check activation action group and transition out of STANDBY."""
    if director.state != "STANDBY":
        return

    activated = director.conn.space_center.active_vessel.control.get_action_group(
        config.ACTIVATION_ACTION_GROUP
    )
    if activated:
        _do_activation(director, est_alt, est_vz)
    elif 500 < est_alt < 5000 and est_vz < 0:
        logger.info("AEGIS emergency switch.")
        director.vessel.control.set_action_group(config.ACTIVATION_ACTION_GROUP, True)


def _do_activation(director: Any, est_alt: float, est_vz: float) -> None:
    """Activate all engines and enter SENSOR_WARMUP."""
    logger.info("AEGIS Activated. Smart Routing initialized.")
    director.writer.log_event({"type": "ACTIVATION"})

    # Throttle limit at zero FIRST — prevents any burn when engines
    # are activated with throttle=1.0 below.
    for e in director.engines:
        engine_obj = director._safe_engine_access(e.part)
        if engine_obj:
            engine_obj.thrust_limit = 0.0

    for e in director.engines:
        _activate_single_engine(director, e)
        e.active = True

    # Always enter SENSOR_WARMUP first to initialise attitude from
    # truth and collect static samples for bias estimation.
    director.state = "SENSOR_WARMUP"
    director._sensor_warmup_ticks = 0
    director._warmup_bg_accum = np.zeros(3)
    director._warmup_ba_accum = np.zeros(3)
    director._warmup_sample_count = 0

    if config.USE_SAS:
        director.vessel.control.sas = True
    elif not config.SAS_PROGRADE_ASCENT:
        director.vessel.control.sas = False

    director.writer.log_event({
        "type": "STATE_TRANSITION",
        "from": "STANDBY",
        "to": director.state,
    })


def _init_mahony_from_truth(director: Any) -> None:
    """Initialise the Mahony attitude filter from the kRPC truth rotation.

    Called on the first tick of SENSOR_WARMUP so the filter starts at the
    actual vessel attitude instead of the identity quaternion, eliminating
    the ~1s convergence delay.

    Also disables the accelerometer-based correction during the warmup
    phases (SENSOR_WARMUP + ESTIMATOR_WARMUP) because the vessel is in free
    fall — specific force ≈ 0, so the correction is driven entirely by
    velocity-differentiation noise, causing rapid random-walk drift.
    Correction is re-enabled when the state transitions to POWERED_DESCENT.
    """
    truth_q = director.sensors.get_truth_attitude()
    director.sensors.attitude_estimator.quaternion = truth_q
    director.sensors.attitude_estimator.disable_correction()
    logger.info("Mahony initialised from kRPC truth attitude q=%s", truth_q)


def _run_sensor_warmup(director: Any, data: dict, est_alt: float) -> None:
    """Run one tick of the sensor-warmup phase.

    Ticks 0..N-1: accumulate gyro + accel samples for bias estimation.
    Tick 0:       initialise Mahony from kRPC truth attitude.
    Tick N-1:     finalise biases, set EKF state, transition to next state.
    """
    ticks: int = getattr(director, "_sensor_warmup_ticks", 0)

    # ── Tick 0: initialise Mahony from truth ────────────────────────
    if ticks == 0:
        _init_mahony_from_truth(director)

    # ── Accumulate samples for bias estimation ──────────────────────
    bg_accum: np.ndarray = director._warmup_bg_accum
    ba_accum: np.ndarray = director._warmup_ba_accum
    count: int = director._warmup_sample_count

    # Gyro bias: at rest the true ω is zero, so any reading is bias
    bg_accum += data["omega_body"]

    # Accel bias: at rest, f_body = -R(q)⁻¹ · g_ned.
    #   bias = f_measured - f_expected
    if data.get("situation") in ("landed", "pre_launch", "splashed"):
        rot_bw = R.from_quat(data["attitude"])
        g_body = rot_bw.inv().apply(data["gravity_ned"])
        ba_accum += data["sf_body"] + g_body
    else:
        # In free fall, expected specific force is roughly 0
        ba_accum += data["sf_body"]

    count += 1
    director._warmup_bg_accum = bg_accum
    director._warmup_ba_accum = ba_accum
    director._warmup_sample_count = count

    ticks += 1
    director._sensor_warmup_ticks = ticks

    # ── Finalise after N ticks ──────────────────────────────────────
    if ticks >= config.SENSOR_WARMUP_TICKS:
        bg_mean: np.ndarray = bg_accum / count
        ba_mean: np.ndarray = ba_accum / count

        # FRAME-004: Do NOT use the warmup gyro bias estimate.
        # The SAS retrograde hold actively rotates the vessel during warmup,
        # so the gyro measures actual rotation, not bias.  Using this as
        # bias would subtract the vessel's own rotation from future gyro
        # readings, causing the Mahony to fail to track attitude changes
        # → rapid drift → 180° attitude flip.
        # The EKF estimates gyro bias in its 12-state vector (states 7-9)
        # via the measurement update — it converges naturally.
        director.estimator.bg = np.zeros(3)
        
        # FRAME-005: Do NOT use the warmup accel bias estimate if flying.
        # When hypersonic, the vessel is decelerating due to aerodynamic drag,
        # so specific force is NOT zero. Assuming it is zero causes the EKF
        # to interpret drag as a massive accelerometer bias (e.g. 1.6 m/s²),
        # completely destroying velocity and position estimates.
        if data.get("situation") in ("landed", "pre_launch", "splashed"):
            director.estimator.ba = ba_mean
        else:
            director.estimator.ba = np.zeros(3)
        director.estimator.P[6:9, 6:9] = (
            np.eye(3) * config.SENSOR_WARMUP_GYRO_BIAS_SIGMA**2
        )
        director.estimator.P[9:12, 9:12] = (
            np.eye(3) * config.SENSOR_WARMUP_ACCEL_BIAS_SIGMA**2
        )

        logger.info(
            "Sensor warmup complete (%d samples). bg=%s, ba=%s",
            count,
            np.round(bg_mean, 6),
            np.round(ba_mean, 6),
        )
        director.writer.log_event({
            "type": "SENSOR_WARMUP_COMPLETE",
            "samples": count,
            "bg_x": float(bg_mean[0]),
            "bg_y": float(bg_mean[1]),
            "bg_z": float(bg_mean[2]),
            "ba_x": float(ba_mean[0]),
            "ba_y": float(ba_mean[1]),
            "ba_z": float(ba_mean[2]),
        })

        # Transition to the appropriate next state (same altitude/vel
        # logic that was in _do_activation before SENSOR_WARMUP existed).
        est_vz = float(np.dot(director.estimator.vel, director.up_vector))
        if est_vz > 0:
            _transition_to(director, "ASCENT_COAST")
        elif est_alt > config.ALT_HYPERSONIC:
            _transition_to(director, "DEORBIT_BURN")
        elif est_alt > config.ALT_POWERED_DESCENT:
            _transition_to(director, "HYPERSONIC_COAST")
        else:
            director.state = "ESTIMATOR_WARMUP"
            director._warmup_ticks = 0
            director.writer.log_event({
                "type": "STATE_TRANSITION",
                "from": "SENSOR_WARMUP",
                "to": "ESTIMATOR_WARMUP",
                "reason": "warmup_complete",
            })


def _activate_single_engine(director: Any, e: Any) -> None:
    """Activate a single kRPC engine: enable independent throttle."""
    engine_obj = director._safe_engine_access(e.part)
    if not engine_obj:
        return
    engine_obj.active = True
    engine_obj.independent_throttle = True
    engine_obj.throttle = 1.0
    for module in e.part.modules:
        if module.name == "ModuleGimbalTrim":
            if "Toggle Trim" in module.events and "Gimbal X" not in module.fields:
                module.trigger_event("Toggle Trim")

# ---------------------------------------------------------------------------
#  State-machine transitions
# ---------------------------------------------------------------------------

def process_state_transitions(
    director: Any, est_alt: float, est_vz: float, data: dict, dt: float
) -> None:
    """Advance the state machine based on altitude / velocity."""
    if director.state == "SENSOR_WARMUP":
        _run_sensor_warmup(director, data, est_alt)
        return

    if director.state == "ESTIMATOR_WARMUP":
        director._warmup_ticks = getattr(director, "_warmup_ticks", 0) + 1
        if director._warmup_ticks >= config.ESTIMATOR_WARMUP_TICKS:
            director.sensors.attitude_estimator.enable_correction()
            _transition_to(director, "POWERED_DESCENT")
        return

    if director.state == "ASCENT_COAST" and est_vz < 0:
        if est_alt > config.ALT_HYPERSONIC:
            _transition_to(director, "HYPERSONIC_COAST")
        else:
            director.sensors.attitude_estimator.enable_correction()
            _transition_to(director, "POWERED_DESCENT")
        return

    if director.state == "DEORBIT_BURN" and est_alt < config.ALT_HYPERSONIC:
        _transition_to(director, "HYPERSONIC_COAST")
        return

    if director.state == "HYPERSONIC_COAST" and est_alt < config.ALT_POWERED_DESCENT:
        director.sensors.attitude_estimator.enable_correction()
        _transition_to(director, "POWERED_DESCENT")
        return

    if director.state == "POWERED_DESCENT" and est_alt < config.ALT_HOVER:
        _transition_to(director, "HOVER_TARGETING")
        return

    if director.state == "HOVER_TARGETING" and est_alt < config.ALT_TERMINAL:
        _transition_to(director, "TERMINAL_DESCENT")
        return

    if director.state == "TERMINAL_DESCENT":
        _check_landed(director, est_alt, est_vz, data, dt)
        return

    if director.state not in VALID_STATES:
        director.state = "HARD_ABORT"


def _set_vessel_lights_color(director: Any, r: float, g: float, b: float) -> None:
    for part in director.vessel.parts.all:
        if part.light:
            part.light.color = (r, g, b)

def _transition_to(director: Any, new_state: str) -> None:
    """Log and perform a state transition."""
    old = director.state
    logger.info("Transitioning from %s to %s", old, new_state)
    director.writer.log_event({
        "type": "STATE_TRANSITION",
        "from": old,
        "to": new_state,
    })
    director.state = new_state

    # Reset phase-tracking flags for horizontal-target blending
    if new_state in ("POWERED_DESCENT", "HOVER_TARGETING", "TERMINAL_DESCENT"):
        director._phase_entry_horizontal = None
        director._phase_entry_ticks = director._dbg_tick_count
        director._early_translation_checked = False
        director._landed_timer = 0.0

    # Hardware Deployment & Lights
    if new_state == "POWERED_DESCENT":
        director.vessel.control.gear = True
        director.vessel.control.lights = True
        _set_vessel_lights_color(director, 1.0, 1.0, 0.0)  # Yellow
    elif new_state == "HOVER_TARGETING":
        _set_vessel_lights_color(director, 0.0, 0.5, 1.0)  # Blue
    elif new_state == "TERMINAL_DESCENT":
        _set_vessel_lights_color(director, 0.0, 1.0, 0.0)  # Green
    elif new_state == "HARD_ABORT":
        _set_vessel_lights_color(director, 1.0, 0.0, 0.0)  # Red
    elif new_state == "LANDED":
        _set_vessel_lights_color(director, 1.0, 1.0, 1.0)  # White


def _check_landed(director: Any, est_alt: float, est_vz: float, data: dict, dt: float) -> None:
    """Update the landed timer and transition to LANDED if conditions hold.

    Uses kRPC situation as an immediate landing indicator, and falls back to
    the timer-based check using estimated altitude/velocity.
    """
    if data["situation"] in ("landed", "splashed", "pre_launch"):
        if est_vz < -20.0:
            logger.error(
                "KRP reports '%s' but est_vz=%.0f — treating as crash.",
                data["situation"],
                est_vz,
            )
            director.writer.log_event(
                {
                    "type": "CATASTROPHIC_FAILURE",
                    "situation": data["situation"],
                    "descent_rate": est_vz,
                }
            )
            director.state = "HARD_ABORT"
            return
        # Allow timer to continue
        pass

    director._landed_timer += dt

    if director._landed_timer < 3.0:
        return

    # Use streams instead of calling vessel.flight() repeatedly
    pitch = director._pitch_stream()
    roll = director._roll_stream()
    if abs(pitch) > 45.0 or abs(roll) > 45.0:
        logger.error(f"Vessel tipped over on landing! pitch={pitch:.1f}, roll={roll:.1f}")
        _transition_to(director, "HARD_ABORT")
        return

    vel_ok = abs(est_vz) < config.LANDED_VEL_THRESHOLD
    alt_ok = abs(data["noisy_alt"]) < config.LANDED_ALT_THRESHOLD
    if data["situation"] in ("landed", "splashed", "pre_launch") or (vel_ok and alt_ok):
        logger.info("Touchdown confirmed. Landing.")
        _transition_to(director, "LANDED")

# ---------------------------------------------------------------------------
#  Engine-data refresh
# ---------------------------------------------------------------------------

def refresh_engine_data(director: Any, active_engines: list) -> None:
    """Update max_thrust for each active engine from kRPC.
    
    Engine positions are fixed within the vessel frame — cached at init,
    never refreshed here. This avoids a blocking part.position() RPC per
    engine per tick which was causing ~6-8ms DT spikes every iteration.
    """
    for e in active_engines:
        engine_obj = director._safe_engine_access(e.part)
        if engine_obj:
            e.max_thrust = engine_obj.max_thrust

# ---------------------------------------------------------------------------
#  Target-state computation (glideslope)
# ---------------------------------------------------------------------------

def _check_early_translation(director: Any, state_vector: np.ndarray) -> None:
    """Lazy-init the early-translation flag on first tick of POWERED_DESCENT."""
    if not getattr(director, '_early_translation_checked', False):
        offset = float(np.linalg.norm(state_vector[:2]))
        director._early_translation = offset > config.PAD_OFFSET_EARLY_THRESHOLD
        director._early_translation_checked = True


def compute_target_state(
    director: Any, state_vector: np.ndarray, a_avail: float
) -> np.ndarray:
    """Return a (6,) target state for the current phase, or zeros for unguided phases."""
    logger.debug(f"[COMPUTE_TARGET] state={director.state}, a_avail={a_avail:.2f}")
    if director.state in UNGUIDED_STATES:
        logger.debug(f"[COMPUTE_TARGET] UNGUIDED -> zeros")
        return np.zeros(6)

    if director.state == "POWERED_DESCENT":
        _check_early_translation(director, state_vector)

        # Blend from entry position to pad [0, 0] over TARGET_BLEND_TICKS.
        if director._phase_entry_horizontal is None:
            director._phase_entry_horizontal = state_vector[:2].copy()
            director._phase_entry_ticks = director._dbg_tick_count
        entry = director._phase_entry_horizontal
        elapsed = director._dbg_tick_count - director._phase_entry_ticks
        blend = min(elapsed / config.TARGET_BLEND_TICKS, 1.0)
        horiz = (1.0 - blend) * entry + blend * np.zeros(2)

        director.guidance.set_phase_gains(
            kp_pos_lateral=config.PD_KP_POS_LATERAL,
            kd_vel_lateral=config.PD_KD_VEL_LATERAL,
        )
        result = director._compute_glideslope_target(
            state_vector,
            floor_alt=config.ALT_HOVER,
            max_descent_rate=config.GLIDESLOPE_RATE_POWERED_DESCENT,
            a_avail=a_avail,
            horizontal_target=horiz,
        )
        logger.debug(f"[COMPUTE_TARGET] PD_glideslope target={result}")
        return result

    if director.state == "HOVER_TARGETING":
        # Velocity-based horizontal guidance: target_vh = APPROACH_K * to_pad, capped at APPROACH_MAX.
        # This eliminates position-blend lag and directly commands velocity toward the pad.
        to_pad = -state_vector[:2]  # vector from current pos to pad [0, 0]
        dist = float(np.linalg.norm(to_pad))
        if dist > 0.1:
            brake_accel = 0.5  # ~3 degrees tilt
            target_speed = min(math.sqrt(2.0 * brake_accel * dist), config.HOVER_APPROACH_MAX)
            target_vh = (to_pad / dist) * target_speed
        else:
            target_vh = np.zeros(2)

        director.guidance.set_phase_gains(
            kp_pos_lateral=config.HOVER_KP_POS_LATERAL,
            kd_vel_lateral=config.HOVER_KD_VEL_LATERAL,
        )
        # Build target state: horizontal position = current (we're commanding velocity, not position)
        # horizontal velocity = target_vh, vertical from glideslope
        target = np.zeros(6)
        target[:2] = state_vector[:2]  # position target = current (velocity-based)
        target[2] = state_vector[2]
        target[3:5] = target_vh
        # Vertical target from glideslope
        vs_target = director._compute_glideslope_target(
            state_vector,
            floor_alt=config.ALT_TERMINAL,
            max_descent_rate=config.GLIDESLOPE_RATE_HOVER,
            a_avail=a_avail,
            horizontal_target=None,  # vertical only
        )
        target[5] = vs_target[5]
        return target

    if director.state == "TERMINAL_DESCENT":
        # Velocity-based horizontal guidance with tighter gains
        to_pad = -state_vector[:2]
        dist = float(np.linalg.norm(to_pad))
        if dist > 0.1:
            brake_accel = 0.3  # ~2 degrees tilt
            target_speed = min(math.sqrt(2.0 * brake_accel * dist), config.TERMINAL_APPROACH_MAX)
            
            # Linearly decay max horizontal speed as we drop below 15m.
            # At 5m (ALT_LANDING), we want exactly 0 horizontal speed.
            est_alt = float(np.dot(state_vector[:3], director.up_vector))
            if est_alt < 15.0:
                speed_limit = max(0.0, (est_alt - 5.0) / 10.0 * config.TERMINAL_APPROACH_MAX)
                target_speed = min(target_speed, speed_limit)
                
            target_vh = (to_pad / dist) * target_speed
        else:
            target_vh = np.zeros(2)

        director.guidance.set_phase_gains(
            kp_pos_lateral=config.TERMINAL_KP_POS_LATERAL,
            kd_vel_lateral=config.TERMINAL_KD_VEL_LATERAL,
        )
        target = np.zeros(6)
        target[:2] = state_vector[:2]
        target[2] = state_vector[2]
        target[3:5] = target_vh
        vs_target = director._compute_glideslope_target(
            state_vector,
            floor_alt=0.0,
            max_descent_rate=config.GLIDESLOPE_RATE_TERMINAL,
            a_avail=a_avail,
            horizontal_target=None,
        )
        target[5] = vs_target[5]
        return target

    return np.zeros(6)

# ---------------------------------------------------------------------------
#  LANDED / HARD_ABORT shutdown
# ---------------------------------------------------------------------------

def handle_landed_shutdown(director: Any, active_engines: list) -> None:
    """Disable engines and thrust on landing."""
    logger.info("Vessel is landed. Shutting down engines and concluding mission.")
    for engine in active_engines:
        engine_obj = director._safe_engine_access(engine.part)
        if engine_obj:
            engine_obj.thrust_limit = 0.0
            engine_obj.independent_throttle = False


def handle_hard_abort(director: Any) -> None:
    """Emergency engine shutdown."""
    director.vessel.control.throttle = 0.0
    for e in director.engines:
        engine_obj = director._safe_engine_access(e.part)
        if engine_obj:
            engine_obj.thrust_limit = 0.0
            engine_obj.independent_throttle = False

# ---------------------------------------------------------------------------
#  Control allocation
# ---------------------------------------------------------------------------

def allocate_control(
    director: Any,
    active_engines: list,
    mass: float,
    desired_wrench: np.ndarray,
    data: dict,
    com: np.ndarray = np.zeros(3),
) -> bool:
    """Allocate wrench to throttles & gimbals. Returns False (and transitions to HARD_ABORT) on degenerate allocation."""
    if not active_engines:
        logger.warning(f"[ALLOCATE] No active engines! state={director.state}")
        return False

    logger.debug(f"[ALLOCATE] state={director.state}, active_engines={len(active_engines)}, mass={mass:.1f}")
    logger.debug(f"[ALLOCATE] desired_wrench={desired_wrench}")

    # Use cached center_of_mass, update every 5 ticks to reduce RPC calls
    director._com_update_tick += 1
    if director._com_update_tick % 5 == 0:
        # Since center_of_mass is not available in kRPC, use the cached zero vector
        # In a real implementation, we would update this based on fuel consumption
        pass
    com = director._cached_com

    try:
        throttles, gimbals, forces_out = director.allocator.allocate(
            desired_wrench, active_engines, com
        )
    except AllocationDegenerateError as e:
        logger.error("CRITICAL: %s. HARD ABORT triggered.", str(e))
        director.writer.log_event({
            "type": "STATE_TRANSITION",
            "from": director.state,
            "to": "HARD_ABORT",
            "reason": "DEGENERATE_ALLOCATION",
        })
        director.state = "HARD_ABORT"
        return False

    logger.debug(f"[ALLOCATE] throttles={throttles}")
    _apply_allocation(director, active_engines, throttles, gimbals, forces_out, mass, data)
    return True


def _apply_allocation(
    director: Any,
    active_engines: list,
    throttles: np.ndarray,
    gimbals: np.ndarray,
    forces_out: np.ndarray,
    mass: float,
    data: dict,
) -> None:
    """Write throttle/gimbal commands to kRPC and update expected state."""
    alpha = 0.95
    THROTTLE_RATE_LIMIT = 0.05
    expected_force = np.zeros(3)
    new_throttles: list[float] = []
    num_engines = max(len(director.engines), 1)
    current_gimbals = np.zeros((num_engines, 2))

    logger.debug(f"[APPLY] active_engines={len(active_engines)}, throttles={throttles}")
    
    is_first_allocation = len(director.expected_throttles) == 0
    for i, engine in enumerate(active_engines):
        throttle = throttles[i]
        if is_first_allocation:
            engine.expected_throttle = float(throttle)
        else:
            engine.expected_throttle = alpha * engine.expected_throttle + (1.0 - alpha) * throttle
        new_throttles.append(engine.expected_throttle)

        # Rate-limit throttle to prevent abrupt changes
        if not is_first_allocation and hasattr(director, '_prev_throttles') and len(director._prev_throttles) > i:
            prev = director._prev_throttles[i]
            throttle = float(np.clip(throttle, prev - THROTTLE_RATE_LIMIT, prev + THROTTLE_RATE_LIMIT))

        logger.debug(f"[APPLY] Engine {i}: throttle_cmd={throttle:.3f}, throttle_exp={engine.expected_throttle:.3f}")

        if throttle > 1e-6 and engine.max_thrust > 0:
            gimballed_dir = forces_out[i] / (throttle * engine.max_thrust)
            expected_force += gimballed_dir * engine.max_thrust * engine.expected_throttle

        # Rate-limit gimbal to prevent rapid deflections
        gimbal_rate_limit = np.deg2rad(20.0)
        if not is_first_allocation and hasattr(director, '_prev_gimbals') and i < len(director._prev_gimbals):
            prev_g = director._prev_gimbals[i]
            gimbal_xy = np.clip(gimbals[i, :], prev_g - gimbal_rate_limit, prev_g + gimbal_rate_limit)
        else:
            gimbal_xy = gimbals[i, :]

        current_gimbals[engine.index, :] = gimbal_xy
        _set_engine_throttle_and_gimbal(director, engine, throttle, gimbal_xy)

    director._prev_throttles = np.array([throttles[i] for i in range(len(active_engines))])
    director._prev_gimbals = np.array([gimbals[i, :] for i in range(len(active_engines))])

    director.current_gimbals = current_gimbals
    director.expected_throttles = np.array(new_throttles)
    logger.debug(f"[APPLY] director.expected_throttles={director.expected_throttles}")
    director._alloc_cond = float(np.linalg.cond(director.allocator._build_B(active_engines)))
    director._saturated_engines_set = set(director.allocator._saturated_engines)

    if mass > 0.0:
        director.expected_accel = (expected_force + data["aero_body"]) / mass

    for i, engine in enumerate(active_engines):
        director._expected_forces[engine.index] = forces_out[i]

    # Per-engine axial force for diagnostic logging
    director._diagnostic_axial_forces = np.array([
        float(np.dot(forces_out[i], engine.thrust_direction))
        for i, engine in enumerate(active_engines)
    ])


def _set_engine_throttle_and_gimbal(
    director: Any, engine: Any, throttle: float, gimbal: np.ndarray,
) -> None:
    """Set thrust_limit and gimbal fields on a single kRPC engine part."""
    engine_obj = director._safe_engine_access(engine.part)
    if not engine_obj:
        return
    engine_obj.thrust_limit = float(throttle)
    for module in engine.part.modules:
        if module.name == "ModuleGimbalTrim" and "Gimbal X" in module.fields:
            gclamp = engine.max_gimbal_deg
            g_x = np.clip(np.degrees(gimbal[0]), -gclamp, gclamp)
            g_y = np.clip(np.degrees(gimbal[1]), -gclamp, gclamp)
            module.set_field_float("Gimbal X", float(g_x))
            module.set_field_float("Gimbal Y", float(g_y))
