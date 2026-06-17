"""Flight-critical control functions for the mission loop.

Responsibilities: SAS, FDI, state machine, allocation, engine management,
estimator integration, throttle control, and abort handling.
"""

import logging
from typing import Any

import numpy as np

import src.config as config
from src.guidance.allocator import AllocationDegenerateError

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
#  State-machine constants
# ---------------------------------------------------------------------------

VALID_STATES = frozenset({
    "STANDBY", "ASCENT_COAST", "DEORBIT_BURN", "HYPERSONIC_COAST",
    "ESTIMATOR_WARMUP", "POWERED_DESCENT", "HOVER_TARGETING", "TERMINAL_DESCENT", "LANDED",
})

UNGUIDED_STATES = frozenset({
    "STANDBY", "ASCENT_COAST", "DEORBIT_BURN", "HYPERSONIC_COAST",
    "ESTIMATOR_WARMUP",
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

def check_fuel_state(director: Any) -> None:
    """Deactivate engines that have run out of fuel."""
    for e in director.engines:
        engine_obj = director._safe_engine_access(e.part)
        if e.active and engine_obj and not engine_obj.has_fuel:
            director.writer.log_event({
                "type": "FUEL_EXHAUSTION",
                "engine_index": e.index,
            })
            logger.error("Engine %d ran out of fuel!", e.index)
            e.active = False

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
    """Activate all engines and determine the initial mission state."""
    logger.info("AEGIS Activated. Smart Routing initialized.")
    director.writer.log_event({"type": "ACTIVATION"})

    for e in director.engines:
        _activate_single_engine(director, e)
        e.active = True

    # Throttle down during coast phases to avoid accelerating away
    if est_vz > 0:
        director.state = "ASCENT_COAST"
        for e in director.engines:
            engine_obj = director._safe_engine_access(e.part)
            if engine_obj:
                engine_obj.thrust_limit = 0.0
    elif est_alt > config.ALT_HYPERSONIC:
        director.state = "DEORBIT_BURN"
        for e in director.engines:
            engine_obj = director._safe_engine_access(e.part)
            if engine_obj:
                engine_obj.thrust_limit = 0.0
    elif est_alt > config.ALT_POWERED_DESCENT:
        director.state = "HYPERSONIC_COAST"
        for e in director.engines:
            engine_obj = director._safe_engine_access(e.part)
            if engine_obj:
                engine_obj.thrust_limit = 0.0
    else:
        director.state = "ESTIMATOR_WARMUP"
        director._warmup_ticks = 0

    if config.USE_SAS:
        director.vessel.control.sas = True
    elif not config.SAS_PROGRADE_ASCENT:
        director.vessel.control.sas = False

    director.writer.log_event({
        "type": "STATE_TRANSITION",
        "from": "STANDBY",
        "to": director.state,
    })


def _activate_single_engine(director: Any, e: Any) -> None:
    """Activate a single kRPC engine: set throttle, enable independent control, toggle trim."""
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
    if director.state == "ESTIMATOR_WARMUP":
        director._warmup_ticks = getattr(director, "_warmup_ticks", 0) + 1
        if director._warmup_ticks >= config.ESTIMATOR_WARMUP_TICKS:
            _transition_to(director, "POWERED_DESCENT")
        return

    if director.state == "ASCENT_COAST" and est_vz < 0:
        _transition_to(director, "HYPERSONIC_COAST" if est_alt > config.ALT_HYPERSONIC else "POWERED_DESCENT")
        return

    if director.state == "DEORBIT_BURN" and est_alt < config.ALT_HYPERSONIC:
        _transition_to(director, "HYPERSONIC_COAST")
        return

    if director.state == "HYPERSONIC_COAST" and est_alt < config.ALT_POWERED_DESCENT:
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


def _check_landed(director: Any, est_alt: float, est_vz: float, data: dict, dt: float) -> None:
    """Update the landed timer and transition to LANDED if conditions hold.
    
    Uses kRPC situation as an immediate landing indicator, and falls back to
    the timer-based check using estimated altitude/velocity.
    """
    if data["situation"] in ("landed", "splashed", "pre_launch"):
        logger.info("Vessel on ground (kRPC situation=%s).", data["situation"])
        _transition_to(director, "LANDED")
        return
    vel_ok = abs(est_vz) < config.LANDED_VEL_THRESHOLD
    alt_ok = abs(data["noisy_alt"]) < config.LANDED_ALT_THRESHOLD
    if vel_ok and alt_ok:
        director._landed_timer += dt
    else:
        director._landed_timer = max(0.0, director._landed_timer - dt)
    if director._landed_timer >= 5.0:
        logger.info("Touchdown confirmed (timer=%.3fs). Landing.", director._landed_timer)
        _transition_to(director, "LANDED")

# ---------------------------------------------------------------------------
#  Engine-data refresh
# ---------------------------------------------------------------------------

def refresh_engine_data(director: Any, active_engines: list) -> None:
    """Update max_thrust and position for each active engine from kRPC."""
    for e in active_engines:
        engine_obj = director._safe_engine_access(e.part)
        if engine_obj:
            e.max_thrust = engine_obj.max_thrust
            e.position = np.array(e.part.position(director.vessel.reference_frame))

# ---------------------------------------------------------------------------
#  Target-state computation (glideslope)
# ---------------------------------------------------------------------------

def compute_target_state(
    director: Any, state_vector: np.ndarray, a_avail: float
) -> np.ndarray:
    """Return a (6,) target state for the current phase, or zeros for unguided phases."""
    logger.debug(f"[COMPUTE_TARGET] state={director.state}, a_avail={a_avail:.2f}")
    if director.state in UNGUIDED_STATES:
        logger.debug(f"[COMPUTE_TARGET] UNGUIDED -> zeros")
        return np.zeros(6)

    if director.state == "POWERED_DESCENT":
        result = director._compute_glideslope_target(
            state_vector,
            floor_alt=config.ALT_HOVER,
            max_descent_rate=config.GLIDESLOPE_RATE_HOVER,
            a_avail=a_avail,
        )
        logger.debug(f"[COMPUTE_TARGET] PD_glideslope target={result}")
        return result

    if director.state == "HOVER_TARGETING":
        return director._compute_glideslope_target(
            state_vector,
            floor_alt=config.ALT_TERMINAL,
            max_descent_rate=config.GLIDESLOPE_RATE_HOVER,
            a_avail=a_avail,
        )

    if director.state == "TERMINAL_DESCENT":
        if director._landed_timer >= 5.0:
            return np.zeros(6)
        return director._compute_glideslope_target(
            state_vector,
            floor_alt=0.0,
            max_descent_rate=config.GLIDESLOPE_RATE_TERMINAL,
            a_avail=a_avail,
        )

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
) -> bool:
    """Allocate wrench to throttles & gimbals.  Returns False (and transitions to HARD_ABORT) on degenerate allocation."""
    if not active_engines:
        logger.warning(f"[ALLOCATE] No active engines! state={director.state}")
        return False

    logger.debug(f"[ALLOCATE] state={director.state}, active_engines={len(active_engines)}, mass={mass:.1f}")
    logger.debug(f"[ALLOCATE] desired_wrench={desired_wrench}")
    
    try:
        throttles, gimbals, forces_out = director.allocator.allocate(
            desired_wrench, active_engines
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

        logger.debug(f"[APPLY] Engine {i}: throttle_cmd={throttle:.3f}, throttle_exp={engine.expected_throttle:.3f}")

        if throttle > 1e-6 and engine.max_thrust > 0:
            gimballed_dir = forces_out[i] / (throttle * engine.max_thrust)
            expected_force += gimballed_dir * engine.max_thrust * engine.expected_throttle

        current_gimbals[engine.index, :] = gimbals[i, :]
        _set_engine_throttle_and_gimbal(director, engine, throttle, gimbals[i, :])

    director.current_gimbals = current_gimbals
    director.expected_throttles = np.array(new_throttles)
    logger.debug(f"[APPLY] director.expected_throttles={director.expected_throttles}")
    director._alloc_cond = float(np.linalg.cond(director.allocator._build_B(active_engines)))
    director._saturated_engines_set = set(director.allocator._saturated_engines)

    if mass > 0.0:
        director.expected_accel = (expected_force + data["aero_body"]) / mass

    for i, engine in enumerate(active_engines):
        director._expected_forces[engine.index] = forces_out[i]


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
