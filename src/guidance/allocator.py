import logging
import numpy as np
from typing import List, Tuple
from src.common.engine import Engine

logger = logging.getLogger(__name__)

class AllocationDegenerateError(Exception):
    pass

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
        Raises:
            AllocationDegenerateError: If the condition number of B exceeds the defined threshold (1e4).
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

        # ISS-002: Check for rank deficiency using condition number
        rank = np.linalg.matrix_rank(B)
        if rank < 6:
            raise AllocationDegenerateError(f"B rank-deficient: rank={rank} < 6, active_engines={len(active_engines)}")
            
        cond = np.linalg.cond(B)
        logger.debug(f"Allocator B matrix cond: {cond}")
        if cond > 1e4:
            raise AllocationDegenerateError(f"B ill-conditioned: cond={cond:.2f}, active_engines={len(active_engines)}")

        # Solve for u using pseudo-inverse
        u = np.linalg.pinv(B) @ desired_wrench

        throttles = np.zeros(N)
        gimbals = np.zeros((N, 2))

        for i, engine in enumerate(active_engines):
            f_vec = u[3 * i : 3 * i + 3]
            f_mag = np.linalg.norm(f_vec)
            
            # Throttle
            throttle = float(f_mag / engine.max_thrust) if engine.max_thrust > 0 else 0.0
            if throttle > 1.0:
                logger.warning(f"Engine {engine.index} thrust saturated (requested: {f_mag:.2f}, max: {engine.max_thrust:.2f})")
            throttles[i] = float(np.clip(throttle, 0.0, 1.0))
            
            # Gimbals
            if f_mag > 1e-6:
                n = f_vec / f_mag
                # Use arccos for the angle to support [0, 180] range
                dot_prod = np.clip(np.dot(engine.thrust_direction, n), -1.0, 1.0)
                angle = np.arccos(dot_prod)
                
                c = np.cross(engine.thrust_direction, n)
                s = np.linalg.norm(c)
                
                if s > 1e-6:
                    rot_vec = (c / s) * angle
                    gimbals[i, 0] = rot_vec[0]
                    gimbals[i, 1] = rot_vec[1]

        return throttles, gimbals
