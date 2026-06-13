import logging
import numpy as np
from typing import List, Tuple
from src.common.engine import Engine

logger = logging.getLogger(__name__)

class ControlAllocator:
    def __init__(self, engines: List[Engine]):
        self.engines: List[Engine] = engines

    def allocate(self, desired_wrench: np.ndarray, active_engines: List[Engine]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Solves the control allocation problem: W = B * u
        where B is the control effectiveness matrix of shape (6, 3N)
        and u is the 3D force vector for each engine of shape (3N,).
        Uses pseudo-inverse numpy.linalg.pinv solver to find u, then maps to throttles and gimbal angles.
        Returns:
            throttles: array of shape (N,) bounded between 0.0 and 1.0.
            gimbals: array of shape (N, 2) representing X/Y gimbal angles in radians.
        """
        N = len(active_engines)
        if N == 0:
            logger.warning("No active engines available for control allocation.")
            return np.array([]), np.empty((0, 2))

        B = np.zeros((6, 3 * N))
        for i, engine in enumerate(active_engines):
            # Force block
            B[0:3, 3 * i : 3 * i + 3] = np.eye(3)
            # Torque block = cross product matrix of position
            r = engine.position
            rx = np.array([
                [0.0, -r[2], r[1]],
                [r[2], 0.0, -r[0]],
                [-r[1], r[0], 0.0]
            ])
            B[3:6, 3 * i : 3 * i + 3] = rx

        # ISS-002: Check for rank deficiency
        rank = np.linalg.matrix_rank(B)
        if rank < 6:
            logger.warning("Control effectiveness matrix is rank-deficient (rank %d < 6). ISS-002", rank)

        # Solve for u using pseudo-inverse
        u = np.linalg.pinv(B) @ desired_wrench

        throttles = np.zeros(N)
        gimbals = np.zeros((N, 2))

        for i, engine in enumerate(active_engines):
            f_vec = u[3 * i : 3 * i + 3]
            f_mag = np.linalg.norm(f_vec)
            
            # Throttle
            throttle = float(f_mag / engine.max_thrust) if engine.max_thrust > 0 else 0.0
            throttles[i] = float(np.clip(throttle, 0.0, 1.0))
            
            # Gimbals
            if f_mag > 1e-6:
                n = f_vec / f_mag
                c = np.cross(engine.thrust_direction, n)
                s = np.linalg.norm(c)
                if s > 1e-6:
                    angle = np.arcsin(np.clip(s, -1.0, 1.0))
                    rot_vec = (c / s) * angle
                    gimbals[i, 0] = rot_vec[0]
                    gimbals[i, 1] = rot_vec[1]

        return throttles, gimbals
