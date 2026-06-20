import numpy as np
from scipy.spatial.transform import Rotation as R  # type: ignore


def ecef_to_ned(
    pad_ecef: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute NED (North-East-Down) frame axes from an ECEF position vector.

    Constructs a right-handed orthonormal frame where:
      - **Down** points from the reference point toward the body centre (``-up``).
      - **East** is the cross product of the local vertical and the planet's
        rotation axis (``+Z`` in ECEF).
      - **North** completes the right-handed set (``cross(up, east)``).

    Parameters
    ----------
    pad_ecef : ndarray (3,)
        ECEF position of the reference point (e.g. landing pad).

    Returns
    -------
    R_ecef_to_ned : ndarray (3, 3)
        Rotation matrix from ECEF to NED.
    ned_quat : ndarray (4,)
        Unit quaternion ``[x, y, z, w]`` (scipy convention).
    north_ecef : ndarray (3,)
        North axis expressed in ECEF coordinates.
    east_ecef : ndarray (3,)
        East axis expressed in ECEF coordinates.
    """
    pad_norm = float(np.linalg.norm(pad_ecef))

    # ── Local vertical (radial) and Down axis ──────────────────────────
    up_vec = pad_ecef / pad_norm        # points away from body centre
    down = -up_vec                      # NED Down axis in ECEF

    # ── East = normalize(north_pole × up) ──────────────────────────────
    # Kerbin's rotation axis in ECEF is +Z.
    north_pole_ecef = np.array([0.0, 0.0, 1.0])
    east_ecef = np.cross(north_pole_ecef, up_vec)
    east_norm = float(np.linalg.norm(east_ecef))
    if east_norm < 1e-10:
        east_ecef = np.array([0.0, 1.0, 0.0])   # fallback at poles
    else:
        east_ecef = east_ecef / east_norm

    # ── North = cross(up, east)  (points along local meridian) ─────────
    north_ecef = np.cross(up_vec, east_ecef)

    # ── Build rotation: ECEF → NED ─────────────────────────────────────
    # Columns of R_ned→ecef = [north, east, down]_ecef
    # R_ecef→ned = R_ned→ecef^T
    R_ecef_to_ned = np.column_stack([north_ecef, east_ecef, down]).T
    # kRPC's create_relative rotation parameter expects a quaternion that
    # rotates from the NEW frame to the BASE frame (NED -> ECEF).
    # Therefore, we must use the transpose of R_ecef_to_ned.
    ned_quat = R.from_matrix(R_ecef_to_ned.T).as_quat()  # [x,y,z,w] scipy convention

    return R_ecef_to_ned, ned_quat, north_ecef, east_ecef
