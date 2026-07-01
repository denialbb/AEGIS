"""UI and telemetry-output functions for the mission loop."""

import logging
from typing import Any

import numpy as np

from scipy.spatial.transform import Rotation as R

from src.telemetry.frame import TelemetryFrame
from src.mission.helpers import build_fuel_state

logger = logging.getLogger(__name__)


def make_telemetry_frame(
    director: Any, timestamp: float, data: dict, state_vector: np.ndarray, skip_predict: bool,
    est_alt: float = 0.0, a_avail: float = 0.0, wrench_force: np.ndarray = np.zeros(3),
    target_state: np.ndarray | None = None,
    wrench: np.ndarray | None = None,
) -> TelemetryFrame:
    """Build a TelemetryFrame from the current loop state.

    Args:
        director: MissionDirector instance.
        timestamp: Wall-clock timestamp.
        data: Sensor poll dict.
        state_vector: (6,) EKF state [pos(3), vel(3)].
        skip_predict: EKF skip_predict flag.
        est_alt: EKF altitude (dot of pos with up_vector).
        a_avail: Available vertical acceleration [m/s²].
        wrench_force: (3,) guidance force body command. Accepts (3,) or
            (6,) — only the force component is used.
        target_state: (6,) full guidance target [target_pos(3), target_vel(3)].
            When provided, logged as ``target_pos`` and ``target_vel`` so the
            diagnostic CSV exposes the velocity-target sign-flip pattern
            that drives the close-vicinity oscillation.
        wrench: (6,) full guidance wrench [force(3), torque(3)]. When
            provided, ``wrench[3:6]`` is logged as ``torque_cmd`` so the
            diagnostic CSV exposes RW saturation, attitude command rate,
            and SAS interaction.
    """
    num = max(len(director.engines), 1)
    throttles = director.expected_throttles if len(director.expected_throttles) > 0 else np.zeros(num)
    gimbals = (
        director.current_gimbals
        if hasattr(director, "current_gimbals")
        else np.zeros((num, 2))
    )
    fuel_state = build_fuel_state(director, num)
    axial = getattr(director, "_diagnostic_axial_forces", np.zeros(num))
    try:
        true_q = data["attitude"]
        true_euler = R.from_quat(true_q).inv().as_euler("YXZ", degrees=True)
        true_att = np.array(true_euler)
    except Exception:
        true_att = np.zeros(3)

    try:
        q_est = director.sensors.attitude_estimator.quaternion
        # Mahony filter estimates body->NED rotation.
        # R.from_quat(q_est).inv() is NED->body.
        # We extract YXZ euler angles to roughly match (pitch, yaw, roll) convention,
        # but honestly we just want to track the filter's output.
        # However, KSP's pitch/yaw/roll are native.
        # We will log the estimated eulers:
        est_euler = R.from_quat(q_est).inv().as_euler("YXZ", degrees=True)
        est_att = np.array(est_euler)
    except Exception as e:
        logger.error(f"Failed to calculate est_attitude: {e}")
        est_att = np.zeros(3)

    target_pos = (
        target_state[:3].copy() if target_state is not None else np.zeros(3)
    )
    target_vel = (
        target_state[3:6].copy() if target_state is not None else np.zeros(3)
    )
    torque_cmd = (
        wrench[3:6].copy() if wrench is not None and wrench.size >= 6 else np.zeros(3)
    )

    return TelemetryFrame(
        timestamp=timestamp,
        altitude=data["noisy_alt"],
        velocity=state_vector[3:],
        noisy_accel=data["sf_body"],
        throttles=throttles,
        fuel_state=fuel_state,
        gimbals=gimbals,
        skip_predict=skip_predict,
        est_alt=est_alt,
        a_avail=a_avail,
        force_body=wrench_force,
        axial_forces=axial,
        position=state_vector[:2],
        true_attitude=true_att,
        est_attitude=est_att,
        target_pos=target_pos,
        target_vel=target_vel,
        torque_cmd=torque_cmd,
    )


def update_hud(
    director: Any,
    data: dict,
    state_vector: np.ndarray,
    est_alt: float,
    est_vz: float,
    mass: float,
    a_avail: float,
    active_engines: list,
    skip_predict: bool,
) -> None:
    """Push current state to the terminal HUD."""
    if not director.hud.is_active:
        return

    num = max(len(director.engines), 1)
    throttles = director.expected_throttles if len(director.expected_throttles) > 0 else np.zeros(num)
    gimbals = (
        director.current_gimbals
        if hasattr(director, "current_gimbals")
        else np.zeros((num, 2))
    )
    gimbals_deg = np.degrees(gimbals) if gimbals.ndim == 2 else np.zeros((num, 2))
    fuel_state = build_fuel_state(director, num)

    fdi_deviation = (
        float(np.linalg.norm(director.expected_accel - data["sf_body"]))
        if np.linalg.norm(director.expected_accel) > 1e-6
        else 0.0
    )
    kf_cov_pos = float(director.estimator.P[2, 2]) if director.estimator.P.shape[0] > 2 else 0.0
    kf_cov_vel = float(director.estimator.P[5, 5]) if director.estimator.P.shape[0] > 5 else 0.0

    director.hud.update({
        "state": director.state,
        "altitude": data["noisy_alt"],
        "est_alt": est_alt,
        "vertical_vel": est_vz,
        "lateral_vel": state_vector[3:] - director.up_vector * est_vz,
        "position": state_vector[:3],
        "throttles": throttles,
        "gimbals_deg": gimbals_deg,
        "fuel_state": fuel_state,
        "fdi_deviation": fdi_deviation,
        "alloc_cond": director._alloc_cond,
        "saturated": director._saturated_engines_set,
        "kf_cov_pos": kf_cov_pos,
        "kf_cov_vel": kf_cov_vel,
        "dt_spike_count": director._dt_spike_count,
        "skip_predict": skip_predict,
        "active_engine_count": len(active_engines),
        "total_engine_count": len(director.engines),
        "mass": mass,
        "a_avail": a_avail,
        "angular_velocity_mag": float(np.linalg.norm(data["omega_body"])),
    })
