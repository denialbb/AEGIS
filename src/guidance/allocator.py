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
            # The control effectiveness matrix B maps individual engine 3D forces
            # to the total 6-DOF wrench on the vessel. B is composed of two blocks per engine:
            # 1. Force block (3x3): A simple identity matrix, since F_engine contributes 1:1 to F_total.
            B[0:3, 3 * i : 3 * i + 3] = np.eye(3)
            
            # 2. Torque block (3x3): The cross product matrix of the engine's position vector 'r'.
            # Since Torque = r x F, we can represent the cross product as a matrix multiplication [r_x] * F.
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
            logger.error("B matrix does not have full row rank (rank < 6).")
            raise AllocationDegenerateError("B matrix does not have full row rank (rank < 6).")
            
        cond = np.linalg.cond(B)
        logger.debug(f"Allocator B matrix cond: {cond}")
        if cond > 1e4:
            logger.error(f"[Allocator] Degenerate allocation! Condition number: {cond:.2f} > 1e4")
            raise AllocationDegenerateError(f"B ill-conditioned: cond={cond:.2f}, active_engines={len(active_engines)}")

        # Solve for the optimal force vector 'u' using the Moore-Penrose pseudo-inverse.
        # This minimizes the L2 norm of 'u', effectively distributing thrust as evenly as possible
        # among the active engines while satisfying the requested wrench
        W = np.array(desired_wrench)
        
        # Use rcond=1e-4 to ignore tiny singular values (preventing the allocator from 
        # demanding millions of Newtons by exploiting floating-point errors)
        u = np.linalg.pinv(B, rcond=1e-4) @ W

        throttles = np.zeros(N)
        gimbals = np.zeros((N, 2))

        for i, engine in enumerate(active_engines):
            f_vec = u[3 * i : 3 * i + 3]
            
            axial_force = float(np.dot(f_vec, engine.thrust_direction))
            lateral_force_vec = f_vec - axial_force * engine.thrust_direction
            lat_mag = float(np.linalg.norm(lateral_force_vec))
            
            # Constrain lateral force to physically possible gimbal limit (5 degrees)
            max_lat = max(axial_force, 0.0) * np.tan(np.radians(5))
            if lat_mag > max_lat and max_lat > 1e-6:
                lateral_force_vec = (lateral_force_vec / lat_mag) * max_lat
            elif max_lat <= 1e-6:
                lateral_force_vec = np.zeros(3)
                
            f_vec = axial_force * engine.thrust_direction + lateral_force_vec
            f_mag = float(np.linalg.norm(f_vec))
            
            # Gimbals: We need to find the rotation angles to point the engine's default 
            # thrust_direction towards the requested force direction 'n'.
            if f_mag > 1e-6:
                n = f_vec / f_mag
                
                # The dot product gives the cosine of the angle between the two vectors.
                # Use arccos for the angle to support [0, 180] range
                dot_prod = float(np.clip(np.dot(engine.thrust_direction, n), -1.0, 1.0))
                
                if dot_prod < 0:
                    # The requested force opposes the physical mounting of the engine.
                    # We cannot fire this engine to push backwards.
                    throttle = 0.0
                    f_mag = 0.0
                else:
                    throttle = float(f_mag / engine.max_thrust) if engine.max_thrust > 0 else 0.0
                    if throttle > 1.0:
                        logger.warning(f"Engine {engine.index} thrust saturated (requested: {f_mag:.2f}, max: {engine.max_thrust:.2f})")
                
                throttles[i] = float(np.clip(throttle, 0.0, 1.0))
                
                if f_mag > 1e-6:
                    angle = np.arccos(dot_prod)
                    
                    # The cross product gives the axis of rotation perpendicular to both vectors.
                    c = np.cross(engine.thrust_direction, n)
                    s = np.linalg.norm(c)
                    
                    if s > 1e-6:
                        # Scale the normalized rotation axis by the angle to get a compact rotation vector.
                        rot_vec = (c / s) * angle
                        gimbals[i, 0] = rot_vec[0]
                        gimbals[i, 1] = rot_vec[2]
            else:
                throttles[i] = 0.0

        return throttles, gimbals
