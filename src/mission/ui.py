"""UI and telemetry-output functions for the mission loop."""

import logging
from typing import Any

import numpy as np

from src.telemetry.frame import TelemetryFrame
from src.mission.helpers import build_fuel_state

logger = logging.getLogger(__name__)


def make_telemetry_frame(
    director: Any, timestamp: float, data: dict, state_vector: np.ndarray, skip_predict: bool,
    est_alt: float = 0.0, a_avail: float = 0.0, wrench_force: np.ndarray = np.zeros(3),
) -> TelemetryFrame:
    """Build a TelemetryFrame from the current loop state."""
    num = max(len(director.engines), 1)
    throttles = director.expected_throttles if len(director.expected_throttles) > 0 else np.zeros(num)
    gimbals = (
        director.current_gimbals
        if hasattr(director, "current_gimbals")
        else np.zeros((num, 2))
    )
    fuel_state = build_fuel_state(director, num)
    axial = getattr(director, "_diagnostic_axial_forces", np.zeros(num))
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
