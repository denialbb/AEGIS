"""Pure helper functions for the mission loop — no side effects on the director."""

from typing import Any

import numpy as np


def unpack_sensor_poll(poll_result: tuple) -> dict:
    """Unpack the raw sensor poll tuple into a dict with named fields."""
    base = poll_result[:7]
    rest = poll_result[7:]
    return {
        "noisy_alt": base[0],
        "sf_body": base[1],
        "attitude": base[2],
        "mass": base[3],
        "aero_body": base[4],
        "situation": base[5],
        "omega_body": base[6],
        "noisy_vel": rest[0] if len(rest) > 0 else np.zeros(3),
        "gravity_ned": rest[1] if len(rest) > 1 else np.zeros(3),
        "raw_gyro": rest[2] if len(rest) > 2 else np.zeros(3),
    }


def compute_a_avail(
    active_engines: list, mass: float, gravity_ned: np.ndarray
) -> float:
    """Net upward acceleration available from TWR [m/s²]."""
    if not active_engines or mass <= 0.0:
        return 1.0
    total_max_thrust = sum(e.max_thrust for e in active_engines)
    g_mag = float(np.linalg.norm(gravity_ned))
    return max(total_max_thrust / mass - g_mag, 1.0)


def build_fuel_state(director: Any, size: int) -> np.ndarray:
    """Return a 1-D array where 1.0 = has fuel, 0.0 = empty."""
    fuel = np.zeros(size)
    for eng in director.engines:
        engine_obj = director._safe_engine_access(eng.part)
        if engine_obj:
            fuel[eng.index] = 1.0 if engine_obj.has_fuel else 0.0
    return fuel
