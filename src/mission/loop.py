"""Mission loop — thin orchestrator.

Imports specialised functions from sibling modules:
- ``flight_control`` — SAS, FDI, state machine, allocation, engine management, estimator
- ``ui`` — HUD and telemetry output
- ``helpers`` — pure utility functions
"""

import signal
import time
import logging
from typing import Any

import numpy as np

import src.config as config
from src.mission import helpers, flight_control, ui

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
#  Loop infrastructure
# ---------------------------------------------------------------------------


def _check_paused(
    director: Any, last_met: float, paused_ticks: int, dt: float
) -> tuple[float, int, bool]:
    """Detect game pause via MET stagnation.  Returns (new_last_met, new_paused_ticks, is_paused)."""
    try:
        current_met = float(director.vessel.met)
    except Exception:
        current_met = last_met + 1.0
    if abs(current_met - last_met) < 1e-6:
        paused_ticks += 1
    else:
        paused_ticks = 0
    return current_met, paused_ticks, paused_ticks > 5


def _handle_timing(
    director: Any, start_time: float, dt: float
) -> tuple[bool, float]:
    """Compute actual dt, detect dt spikes, set debug attrs.  Returns (skip_predict, ekf_dt)."""
    if director.last_tick_time > 0:
        actual_dt = start_time - director.last_tick_time
        if actual_dt > 5 * dt:
            director.writer.log_event(
                {
                    "type": "DT_SPIKE",
                    "actual_dt": actual_dt,
                    "expected_dt": dt,
                }
            )
            director.guidance.reset()
            director._dt_spike_count += 1
        ekf_dt = actual_dt
        skip_predict = False
    else:
        actual_dt = dt
        skip_predict = False
        ekf_dt = dt
    director.last_tick_time = start_time
    director._dbg_actual_dt = actual_dt
    director._dbg_skip_predict = skip_predict
    return skip_predict, ekf_dt


def _poll_telemetry(director: Any) -> dict:
    """Poll sensors, update guidance gravity, and return a named data dict."""
    try:
        poll_result = director.sensors.poll()
    except Exception:
        logger.warning("Sensor poll failed — vessel may be destroyed.")
        return {
            "noisy_alt": 0.0,
            "sf_body": np.zeros(3),
            "attitude": np.array([0.0, 0.0, 0.0, 1.0]),
            "mass": 0.0,
            "aero_body": np.zeros(3),
            "situation": "destroyed",
            "omega_body": np.zeros(3),
            "noisy_vel": np.zeros(3),
            "gravity_ned": np.zeros(3),
            "raw_gyro": np.zeros(3),
        }
    data = helpers.unpack_sensor_poll(poll_result)
    director._dbg_raw_gyro = data["raw_gyro"]
    director.guidance.gravity_ned = data["gravity_ned"]
    logger.debug(
        "Poll: raw_alt=%.2f, raw_ang_vel=%.3f",
        data["noisy_alt"],
        np.linalg.norm(data["omega_body"]),
    )
    return data


_ALLOCATION_INHIBITED = frozenset(
    {
        "HARD_ABORT",
        "STANDBY",
        "ASCENT_COAST",
        "DEORBIT_BURN",
        "HYPERSONIC_COAST",
        "SENSOR_WARMUP",
        "LANDED",
    }
)

# ---------------------------------------------------------------------------
#  Debug helpers
# ---------------------------------------------------------------------------


def _maybe_log_tick(director: Any) -> None:
    """Log tick count every 10 iterations."""
    director._dbg_tick_count += 1
    if director._dbg_tick_count % 10 == 0:
        logger.debug(
            "Tick %d, state=%s", director._dbg_tick_count, director.state
        )


def _log_sleep(director: Any, dt: float, start_time: float) -> None:
    """Sleep for the remainder of the timestep."""
    elapsed = time.time() - start_time
    sleep_time = dt - elapsed
    director._dbg_sleep_time = sleep_time
    if sleep_time > 0:
        time.sleep(sleep_time)


# ---------------------------------------------------------------------------
#  Main entry point
# ---------------------------------------------------------------------------


def run_mission_loop(director: Any) -> bool:
    """Execute the main control loop.

    Args:
        director: Instance of :class:`src.main.MissionDirector`.

    Returns:
        True if mission completed successfully (LANDED), False otherwise.
    """
    dt = 1.0 / config.TARGET_HZ
    success = False
    ves_orientation = "stability"

    signal.signal(
        signal.SIGINT,
        lambda sig, fr: setattr(director, "_exit_requested", True),
    )
    signal.signal(
        signal.SIGTERM,
        lambda sig, fr: setattr(director, "_exit_requested", True),
    )

    director.hud.start()
    director._landed_timer = 0.0
    director._mission_start_time = time.time()

    try:
        last_met = float(director.vessel.met)
    except Exception:
        last_met = 0.0
    paused_ticks = 0

    while director.state not in ("HARD_ABORT", "LANDED"):
        start_time = time.time()

        # Pause detection
        last_met, paused_ticks, is_paused = _check_paused(
            director, last_met, paused_ticks, dt
        )
        if is_paused:
            time.sleep(dt)
            continue

        # Tick log
        _maybe_log_tick(director)

        # User abort
        if director._exit_requested:
            logger.info("SIGINT received. Requesting graceful shutdown.")
            director.writer.log_event({"type": "USER_ABORT"})
            director.state = "HARD_ABORT"
            break

        # Mission timeout (safety net)
        if time.time() - director._mission_start_time > config.MAX_MISSION_TIME:
            logger.error(
                "Mission timeout after %.0f s. Aborting.",
                config.MAX_MISSION_TIME,
            )
            director.writer.log_event({"type": "MISSION_TIMEOUT"})
            director.state = "HARD_ABORT"
            break

        # Timing
        skip_predict, ekf_dt = _handle_timing(director, start_time, dt)

        # 1. Poll sensors
        data = _poll_telemetry(director)

        # 2. Vessel destroyed or landed? (kRPC situation check)
        new_state = flight_control.check_vessel_situation(director, data)
        if new_state == "HARD_ABORT":
            break
        if new_state == "LANDED":
            flight_control.handle_landed_shutdown(
                director, [e for e in director.engines if e.active]
            )
            success = True
            break

        # 3. Master throttle
        flight_control.set_throttle_for_state(director)

        # 4. EKF estimate
        state_vector = flight_control.update_estimator(
            director, data, ekf_dt, skip_predict
        )
        est_alt = float(np.dot(director.estimator.pos, director.up_vector))
        est_vz = float(np.dot(director.estimator.vel, director.up_vector))

        # 4b. Catastrophic impact detection
        if flight_control.check_catastrophic_impact(
            director, data, est_alt, est_vz
        ):
            break

        # 5. IMU health
        flight_control.check_imu_health(director)

        # 6. Fuel check
        flight_control.check_fuel_state(director)

        # 7. FDI
        flight_control.run_fdi(director, data, data["mass"], skip_predict)

        # 8. Control authority
        active = [e for e in director.engines if e.active]
        if flight_control.check_control_authority(director, active):
            break

        # 9. Estimated altitude/velocity
        est_vz = float(np.dot(state_vector[3:], director.up_vector))
        ves_orientation = flight_control.handle_sas(
            director, est_vz, ves_orientation
        )

        # 10. STANDBY activation
        est_alt = float(np.dot(state_vector[:3], director.up_vector))
        flight_control.handle_standby(director, data, est_alt, est_vz)

        # 11. State machine
        flight_control.process_state_transitions(
            director, est_alt, est_vz, data, dt
        )

        # 12. Engine data refresh
        active = [e for e in director.engines if e.active]
        flight_control.refresh_engine_data(director, active)

        # 13. Available acceleration
        a_avail = helpers.compute_a_avail(
            active, data["mass"], data["gravity_ned"]
        )

        # 14. Target state
        target = flight_control.compute_target_state(
            director, state_vector, a_avail
        )

        # 15. Terminal states
        if director.state == "LANDED":
            flight_control.handle_landed_shutdown(director, active)
            success = True
            break
        if director.state == "HARD_ABORT":
            flight_control.handle_hard_abort(director)
            break

        # 16. Angular motion metric
        director.total_angular_motion += (
            float(np.linalg.norm(data["omega_body"])) * dt
        )

        # 17. Guidance wrench
        wrench = director.guidance.compute_wrench(
            current_state=state_vector,
            current_attitude=data["attitude"],
            mass=data["mass"],
            target_state=target,
            up_vector=director.up_vector,
            dt=dt,
            angular_velocity=data["omega_body"],
            max_a_avail=a_avail,
        )
        if director.state in flight_control.UNGUIDED_STATES:
            wrench = np.zeros(6)

        # Torque is zeroed for the equal-force allocator: it only consumes
        # wrench[:3] (force).  Gimbals steer each engine toward the commanded
        # body-frame force direction (already encodes attitude correction from
        # a_cmd_ned → a_cmd_body).  Large attitude corrections are handled by
        # the AttitudeController via SAS joystick commands, which cooperates
        # with gimbal steering: gimbals provide fast local correction within
        # their authority, while the attitude controller slews the whole vessel.
        wrench[3:6] = 0.0

        # 18. Control allocation
        if active and director.state not in _ALLOCATION_INHIBITED:
            com = np.zeros(3)
            for attr in ['center_of_mass', 'CoM', 'com', 'COM']:
                try:
                    com = np.array(getattr(director.vessel, attr))
                    break
                except (AttributeError, TypeError):
                    pass
            if np.linalg.norm(com) < 0.001:
                try:
                    com = np.array(director.vessel.parts.center_of_mass())
                except (AttributeError, TypeError):
                    pass
            flight_control.allocate_control(
                director, active, data["mass"], wrench, data, com
            )

        # 19. Telemetry frame
        frame = ui.make_telemetry_frame(
            director, start_time, data, state_vector, skip_predict,
            est_alt=est_alt, a_avail=a_avail, wrench_force=wrench[:3],
        )
        director.writer.log_tick(frame)

        # 20. HUD
        ui.update_hud(
            director,
            data,
            state_vector,
            est_alt,
            est_vz,
            data["mass"],
            a_avail,
            active,
            skip_predict,
        )

        # 21. Sleep
        _log_sleep(director, dt, start_time)

    director.hud.stop()
    director.writer.close()
    return success
