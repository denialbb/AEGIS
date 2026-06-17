"""kRPC reference-frame construction and helpers.

This module wraps the low-level ``ecef_to_ned()`` math from
``geometry.py`` and provides reusable functions that interact with
kRPC to build, query, and use coordinate frames.

All public functions take the kRPC objects they need as explicit
parameters — no global state, no hidden side effects.
"""

import numpy as np
from typing import Any, Tuple

from src.common.geometry import ecef_to_ned


# ---------------------------------------------------------------------------
#  Frame construction
# ---------------------------------------------------------------------------

def build_ned_frame(
    conn: Any,
    body: Any,
    target_lat: float,
    target_lon: float,
) -> Tuple[Any, np.ndarray]:
    """Build a true NED (North-East-Down) reference frame at a landing site.

    The frame is positioned at the surface point given by *target_lat* /
    *target_lon* on *body*, with axes aligned to local north, east, and
    down.  The ``up_vector`` (pointing away from the body centre) is always
    ``[0, 0, -1]`` in this frame.

    Args:
        conn: kRPC connection object.
        body: Celestial body (e.g. ``vessel.orbit.body``).
        target_lat: Target latitude in degrees.
        target_lon: Target longitude in degrees.

    Returns:
        ned_frame: A kRPC ``ReferenceFrame`` with its origin at the landing
            site and axes aligned North (X), East (Y), Down (Z).
        up_vector: ``[0, 0, -1]`` — the direction pointing away from the
            body centre, expressed in NED coordinates.
    """
    pad_ecef = get_pad_ecef(body, target_lat, target_lon)

    _R_ecef_to_ned, ned_quat, _north, _east = ecef_to_ned(pad_ecef)

    ned_frame = conn.space_center.ReferenceFrame.create_relative(
        body.reference_frame,
        position=tuple(float(v) for v in pad_ecef),
        rotation=tuple(float(v) for v in ned_quat),
    )
    up_vector: np.ndarray = np.array([0.0, 0.0, -1.0])
    return ned_frame, up_vector


def get_pad_ecef(
    body: Any,
    target_lat: float,
    target_lon: float,
) -> np.ndarray:
    """Return the surface position of a landing site in ECEF coordinates.

    Args:
        body: Celestial body.
        target_lat: Latitude in degrees.
        target_lon: Longitude in degrees.

    Returns:
        pad_ecef: ``(3,)`` array — the body-fixed (ECEF) position vector
            of the point at (*target_lat*, *target_lon*) on *body*.
    """
    return np.array(
        body.surface_position(target_lat, target_lon, body.reference_frame),
        dtype=float,
    )


# ---------------------------------------------------------------------------
#  Gravity
# ---------------------------------------------------------------------------

def compute_gravity_ned(
    body: Any,
    pos_ecef: np.ndarray,
) -> np.ndarray:
    """Compute gravitational acceleration in NED frame.

    Uses the body's gravitational parameter ``μ`` and the distance from
    the body centre.  In the NED convention gravity always points along
    the +Z (Down) axis.

    Args:
        body: Celestial body.
        pos_ecef: ``(3,)`` array — current vessel position in ECEF
            (body-centred, body-fixed) coordinates.

    Returns:
        gravity_ned: ``(3,)`` array ``[0, 0, +g]`` with g = μ / r².
    """
    mu = body.gravitational_parameter
    distance = float(np.linalg.norm(pos_ecef))
    g_mag = mu / distance ** 2 if distance > 0 else 9.81
    return np.array([0.0, 0.0, g_mag])


# ---------------------------------------------------------------------------
#  Vessel state queries
# ---------------------------------------------------------------------------

def get_vessel_position_ned(
    vessel: Any,
    ned_frame: Any,
) -> np.ndarray:
    """Vessel position in the NED frame (origin at landing pad).

    Returns:
        ``(3,)`` array — ``[north, east, down]`` in metres.
    """
    return np.array(vessel.position(ned_frame))


def get_vessel_velocity_ned(
    vessel: Any,
    ned_frame: Any,
) -> np.ndarray:
    """Vessel velocity in the NED frame.

    Returns:
        ``(3,)`` array — ``[v_north, v_east, v_down]`` in m/s.
        Downward velocity is positive.
    """
    return np.array(vessel.flight(ned_frame).velocity)


def get_vessel_altitude_ned(
    vessel: Any,
    ned_frame: Any,
) -> float:
    """Surface altitude above the terrain directly below the vessel.

    Queried in NED frame for consistency.

    Returns:
        Altitude in metres.
    """
    return float(vessel.flight(ned_frame).surface_altitude)


def get_vessel_state_ned(
    vessel: Any,
    ned_frame: Any,
) -> Tuple[np.ndarray, np.ndarray, float]:
    """Convenience — position, velocity, and altitude in a single call.

    Returns:
        ``(pos_ned, vel_ned, altitude)`` — see the individual helpers
        above for shape and meaning.
    """
    return (
        get_vessel_position_ned(vessel, ned_frame),
        get_vessel_velocity_ned(vessel, ned_frame),
        get_vessel_altitude_ned(vessel, ned_frame),
    )
