#!/usr/bin/env python3
"""
Runtime sanity check for the NED reference frame.

Connects to a running KSP instance, loads a save, and verifies that the
vessel-on-pad position, altitude, and gravity readings are consistent
with the true NED convention documented in REFERENCE_FRAMES.md.

Usage:
  KRPC_ADDRESS=172.xx.xx.xx .venv/bin/python scripts/check_ned_frame.py [save_name]

Exits with code 0 if all checks pass, 1 otherwise.
"""

import os
import sys
import time

import krpc
import numpy as np

sys.path.insert(0, os.path.abspath("."))
import src.config as config
from src.main import MissionDirector

_TOL_POS = 5.0       # metres — vessel may be slightly off pad centre
_TOL_ALT = 1.0       # metres
_TOL_GRAV = 0.5      # m/s²
_TOL_VEL = 0.1       # m/s


class NedCheckError(AssertionError):
    """Raised when a NED frame check fails."""


def _check(condition: bool, msg: str) -> None:
    if not condition:
        raise NedCheckError(msg)


def main() -> None:
    address = os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS)
    save_name = sys.argv[1] if len(sys.argv) > 1 else "quicksave"

    print(f"Connecting to KSP at {address} ...")
    conn = krpc.connect(name="AEGIS_NedCheck", address=address)

    print(f"Loading save '{save_name}' ...")
    conn.space_center.load(save_name)
    print("Waiting for vessel to load ...")
    time.sleep(2.0)

    vessel = conn.space_center.active_vessel
    if vessel is None:
        print("ERROR: No active vessel after loading save.")
        conn.close()
        sys.exit(1)

    print(f"Active vessel: {vessel.name}  situation: {vessel.situation}\n")

    # ── Create the MissionDirector (builds the NED frame) ──────────────
    director = MissionDirector(conn)
    ned = director.ned_frame
    up = director.up_vector
    print(f"up_vector in NED: {up}")
    _check(np.allclose(up, [0.0, 0.0, -1.0]),
           f"up_vector = {up}, expected (0, 0, -1)")

    # ── Check 7: vessel on pad at (0,0,0) ───────────────────────────────
    pos_ned = np.array(vessel.position(ned))
    print(f"Vessel position in NED: {pos_ned}")
    _check(np.linalg.norm(pos_ned[:2]) < _TOL_POS,
           f"Horizontal offset {np.linalg.norm(pos_ned[:2]):.2f} m > {_TOL_POS} m")
    _check(abs(pos_ned[2]) < _TOL_ALT,
           f"Vertical offset {pos_ned[2]:.2f} m > {_TOL_ALT} m (z=Down, expected ~0)")

    # ── Check 8: altitude matches -z ────────────────────────────────────
    alt = float(vessel.flight(ned).surface_altitude)
    alt_from_pos = float(-pos_ned[2])
    print(f"Surface altitude: {alt:.2f} m   -pos_z: {alt_from_pos:.2f} m")
    _check(abs(alt - alt_from_pos) < _TOL_ALT,
           f"Altitude mismatch: surface={alt:.2f}, -pos_z={alt_from_pos:.2f}")

    # ── Check 8b: velocity sign convention ──────────────────────────────
    vel_ned = np.array(vessel.flight(ned).velocity)
    vz = float(vel_ned[2])
    print(f"Velocity in NED: {vel_ned}  (vz = {vz:.2f}, positive = descending)")
    # Vessel on pad should have near-zero velocity
    _check(np.linalg.norm(vel_ned) < _TOL_VEL,
           f"Non-zero velocity on pad: {vel_ned}")

    # ── Check 9: gravity ≈ (0, 0, 9.81) on the pad ─────────────────────
    dt_poll = director.sensors.poll()
    # Index 8 in the 10-tuple is gravity_ned
    gravity_ned = np.array(dt_poll[8])
    print(f"Gravity in NED from sensor poll: {gravity_ned}")
    _check(abs(gravity_ned[0]) < _TOL_GRAV,
           f"gravity_x = {gravity_ned[0]:.3f}, expected ~0")
    _check(abs(gravity_ned[1]) < _TOL_GRAV,
           f"gravity_y = {gravity_ned[1]:.3f}, expected ~0")
    _check(gravity_ned[2] > 8.0,
           f"gravity_z = {gravity_ned[2]:.3f}, expected ~9.81")

    # ── All passed ──────────────────────────────────────────────────────
    print("\nAll NED frame sanity checks PASSED.")
    conn.close()
    sys.exit(0)


if __name__ == "__main__":
    try:
        main()
    except NedCheckError as e:
        print(f"FAIL: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)
