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

    def _build_B(self, active_engines: List[Engine]) -> np.ndarray:
        """Build the control effectiveness matrix B of shape (6, 3N)."""
        N = len(active_engines)
        B = np.zeros((6, 3 * N))
        for i, engine in enumerate(active_engines):
            B[0:3, 3 * i : 3 * i + 3] = np.eye(3)
            r = engine.position
            rx = np.array(
                [[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]]
            )
            B[3:6, 3 * i : 3 * i + 3] = rx
        return B

    def is_rank_sufficient(
        self, active_engines: List[Engine]
    ) -> Tuple[bool, int]:
        """
        Check if the B matrix for the given engines has full row rank (>= 6).
        Returns (is_sufficient, actual_rank).
        """
        N = len(active_engines)
        if N < 3:
            # With fewer than 3 engines, the B matrix cannot achieve rank 6
            return False, 0 if N == 0 else int(np.linalg.matrix_rank(np.zeros((6, 3 * N))))
        B = self._build_B(active_engines)
        rank = np.linalg.matrix_rank(B)
        return rank >= 6, rank

    def allocate(
        self, desired_wrench: np.ndarray, active_engines: List[Engine]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Solves the control allocation problem using an iterative saturation-aware approach.
        This method redistributes demand from saturated engines to minimize wrench error
        while respecting hard thrust limits.
        
        Args:
            desired_wrench: Desired 6DOF wrench [fx, fy, fz, tx, ty, tz] in body frame
            active_engines: List of currently active engines
            
        Returns:
            throttles: array of shape (N,) bounded between 0.0 and 1.0.
            gimbals: array of shape (N, 2) representing X/Y gimbal angles in radians.
        """
        if not active_engines:
            return np.array([]), np.empty((0, 2))
        
        # Initialize working arrays
        N = len(active_engines)
        f_desired = np.zeros((N, 3))  # Desired force per engine (before clamping)
        f_actual = np.zeros((N, 3))   # Actual force per engine (after clamping)
        saturated = np.zeros(N, dtype=bool)  # Tracks saturation per engine
        
        # Build B matrix once (constant across iterations)
        B = self._build_B(active_engines)  # Shape (6, 3N)
        
        # Residual wrench to satisfy (starts as desired_wrench)
        residual_wrench = desired_wrench.copy()
        
        for iteration in range(self._max_iterations):
            # Solve for force vectors given current residual wrench
            # u = pinv(B) @ residual_wrench  -> shape (3N,)
            u = np.linalg.pinv(B, rcond=1e-4) @ residual_wrench
            
            # Reshape u to (N, 3) force vectors
            f_desired = u.reshape((N, 3))
            
            # Check for saturation
            newly_saturated = np.zeros(N, dtype=bool)
            f_actual[:] = f_desired  # Start with desired forces
            
            for i, engine in enumerate(active_engines):
                f_mag = np.linalg.norm(f_desired[i])
                if f_mag > engine.max_thrust + 1e-6:  # Account for floating point
                    newly_saturated[i] = True
                    # Clamp to max thrust in desired direction
                    direction = f_desired[i] / f_mag if f_mag > 1e-6 else engine.thrust_direction
                    f_actual[i] = direction * engine.max_thrust
            
            # If no new saturation, we've converged
            if not np.any(newly_saturated):
                break
                
            # Update saturated set
            saturated |= newly_saturated
            
            # Compute residual wrench from clamped forces
            actual_wrench = self._compute_wrench_from_forces(f_actual, active_engines)
            residual_wrench = desired_wrench - actual_wrench
            
            # If residual is negligible, break early
            if np.linalg.norm(residual_wrench) < 1e-6:
                break
                
            # Build reduced B matrix for unsaturated engines only
            if np.any(~saturated):
                unsaturated_indices = np.where(~saturated)[0]
                B_reduced = B[:, 3*unsaturated_indices[:, None] + np.arange(3)]
                # Solve for unsaturated engines only
                u_reduced = np.linalg.pinv(B_reduced, rcond=1e-4) @ residual_wrench
                # Update f_desired for unsaturated engines
                for idx, i in enumerate(unsaturated_indices):
                    f_desired[i] = u_reduced[3*idx:3*idx+3]
            else:
                # All engines saturated - no further improvement possible
                break
        
        # Convert final forces to throttles and gimbals
        throttles, gimbals = self._forces_to_controls(f_actual, active_engines)
        
        # Log saturation events (once per engine per allocation to avoid spam)
        newly_saturated_this_call = saturated & ~self._saturated_engines
        self._saturated_engines = set(np.where(saturated)[0])
        for i in np.where(newly_saturated_this_call)[0]:
            engine = active_engines[i]
            f_mag = np.linalg.norm(f_actual[i])
            logger.warning(
                f"Engine {engine.index} thrust saturated "
                f"(requested: {f_mag:.2f}, max: {engine.max_thrust:.2f})"
            )
        
        return throttles, gimbals

    def _compute_wrench_from_forces(
        self, forces: np.ndarray, active_engines: List[Engine]
    ) -> np.ndarray:
        """
        Compute 6D wrench [fx, fy, fz, tx, ty, tz] from engine force vectors.
        
        Args:
            forces: Array of shape (N, 3) with force vectors for each engine
            active_engines: List of engines corresponding to the force vectors
            
        Returns:
            Wrench vector of shape (6,)
        """
        wrench = np.zeros(6)
        for i, engine in enumerate(active_engines):
            # Force component (simple sum)
            wrench[0:3] += forces[i]
            # Torque component: r × F
            r = engine.position
            wrench[3:6] += np.cross(r, forces[i])
        return wrench

    def _forces_to_controls(
        self, forces: np.ndarray, active_engines: List[Engine]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Convert engine force vectors to throttle and gimbal commands.
        
        Args:
            forces: Array of shape (N, 3) with force vectors for each engine
            active_engines: List of engines corresponding to the force vectors
            
        Returns:
            throttles: array of shape (N,) with values in [0.0, 1.0]
            gimbals: array of shape (N, 2) with gimbal angles in radians
        """
        N = len(active_engines)
        throttles = np.zeros(N)
        gimbals = np.zeros((N, 2))
        
        for i, engine in enumerate(active_engines):
            f_vec = forces[i]
            
            # Decompose force into axial and lateral components
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
            
            # Compute throttle
            if f_mag > 1e-6:
                throttle = (
                    float(f_mag / engine.max_thrust)
                    if engine.max_thrust > 0
                    else 0.0
                )
                # Clamp to [0, 1] range
                throttle = float(np.clip(throttle, 0.0, 1.0))
            else:
                throttle = 0.0
            throttles[i] = throttle
            
            # Compute gimbal angles
            if f_mag > 1e-6:
                n = f_vec / f_mag  # Desired thrust direction unit vector
                
                # The dot product gives the cosine of the angle between the two vectors.
                # Use arccos for the angle to support [0, 180] range
                dot_prod = float(
                    np.clip(np.dot(engine.thrust_direction, n), -1.0, 1.0)
                )
                
                if dot_prod < 0:
                    # The requested force opposes the physical mounting of the engine.
                    # We cannot fire this engine to push backwards.
                    angle = np.pi  # 180 degrees - but throttle will be 0 above
                else:
                    angle = np.arccos(dot_prod)
                
                # The cross product gives the axis of rotation perpendicular to both vectors.
                c = np.cross(engine.thrust_direction, n)
                s = np.linalg.norm(c)
                if s > 1e-6:
                    # Scale the normalized rotation axis by the angle to get a compact rotation vector.
                    rot_vec = (c / s) * angle
                    gimbals[i, 0] = rot_vec[0]
                    gimbals[i, 1] = rot_vec[1]
                else:
                    # Vectors are parallel - no rotation needed
                    gimbals[i, 0] = 0.0
                    gimbals[i, 1] = 0.0
            else:
                # Zero force - no gimbal deflection needed
                gimbals[i, 0] = 0.0
                gimbals[i, 1] = 0.0
        
        return throttles, gimbals

    @property
    def _max_iterations(self) -> int:
        """Maximum iterations for the allocation solver to prevent infinite loops."""
        return 10