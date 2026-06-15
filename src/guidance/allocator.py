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
        self._saturated_engines: set[int] = set()

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
            return False, (
                0
                if N == 0
                else int(np.linalg.matrix_rank(np.zeros((6, 3 * N))))
            )
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
        Raises:
            AllocationDegenerateError: If the B matrix is rank deficient or ill-conditioned.
        """
        if not active_engines:
            return np.array([]), np.empty((0, 2))

        # Check for rank deficiency and condition number (same as original implementation)
        B = self._build_B(active_engines)
        rank = np.linalg.matrix_rank(B)
        if rank < 6:
            logger.error("B matrix does not have full row rank (rank < 6).")
            raise AllocationDegenerateError(
                "B matrix does not have full row rank (rank < 6)."
            )

        cond = np.linalg.cond(B)
        logger.debug(f"Allocator B matrix cond: {cond}")
        if cond > 1e4:
            logger.error(
                f"[Allocator] Degenerate allocation! Condition number: {cond:.2f} > 1e4"
            )
            raise AllocationDegenerateError(
                f"B ill-conditioned: cond={cond:.2f}, active_engines={len(active_engines)}"
            )

        # Initialize working arrays
        N = len(active_engines)
        f_desired = np.zeros(
            (N, 3)
        )  # Desired force per engine (before clamping)
        f_actual = np.zeros((N, 3))  # Actual force per engine (after clamping)
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
                axial_f = float(np.dot(f_desired[i], engine.thrust_direction))
                if axial_f > engine.max_thrust + 1e-6 or axial_f < -1e-6:
                    newly_saturated[i] = True
                    if axial_f > 0:
                        f_actual[i] = (
                            engine.thrust_direction * engine.max_thrust
                        )
                    else:
                        f_actual[i] = np.zeros(3)
                else:
                    f_actual[i] = f_desired[i]

            # If no new saturation, we've converged
            if not np.any(newly_saturated):
                break

            # Update saturated set
            saturated |= newly_saturated

            # Compute residual wrench from clamped forces
            actual_wrench = self._compute_wrench_from_forces(
                f_actual, active_engines
            )
            residual_wrench = desired_wrench - actual_wrench

            # If residual is negligible, break early
            if np.linalg.norm(residual_wrench) < 1e-6:
                break

            # Build reduced B matrix for unsaturated engines only
            if np.any(~saturated):
                unsaturated_indices = np.where(~saturated)[0]
                indices = (
                    3 * unsaturated_indices[:, None] + np.arange(3)
                ).ravel()
                B_reduced = B[:, indices]
                # Solve for unsaturated engines only
                u_reduced = (
                    np.linalg.pinv(B_reduced, rcond=1e-4) @ residual_wrench
                )
                # Update f_desired for unsaturated engines
                for idx, i in enumerate(unsaturated_indices):
                    f_desired[i] = u_reduced[3 * idx : 3 * idx + 3]
            else:
                # All engines saturated - no further improvement possible
                break

        # Convert final forces to throttles and gimbals
        throttles, gimbals = self._forces_to_controls(f_actual, active_engines)

        # Log saturation events (once per engine per allocation to avoid spam)
        newly_saturated_mask = saturated & ~np.isin(
            np.arange(N), list(self._saturated_engines)
        )
        newly_saturated_indices = np.where(newly_saturated_mask)[0]
        self._saturated_engines.update(newly_saturated_indices.tolist())
        for i in newly_saturated_indices:
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

            axial_force = float(np.dot(f_vec, engine.thrust_direction))

            if axial_force <= 1e-6:
                throttles[i] = 0.0
                gimbals[i, 0] = 0.0
                gimbals[i, 1] = 0.0
                continue

            axial_force = min(axial_force, engine.max_thrust)

            lateral_force_vec = (
                f_vec
                - float(np.dot(f_vec, engine.thrust_direction))
                * engine.thrust_direction
            )
            lat_mag = float(np.linalg.norm(lateral_force_vec))

            max_lat = axial_force * np.tan(np.radians(5))
            if lat_mag > max_lat and max_lat > 1e-6:
                lateral_force_vec = (lateral_force_vec / lat_mag) * max_lat
            elif max_lat <= 1e-6:
                lateral_force_vec = np.zeros(3)

            f_vec = axial_force * engine.thrust_direction + lateral_force_vec

            f_mag = float(np.linalg.norm(f_vec))
            throttle = (
                min(f_mag / engine.max_thrust, 1.0)
                if engine.max_thrust > 0
                else 0.0
            )
            throttles[i] = float(np.clip(throttle, 0.0, 1.0))

            if f_mag > 1e-6:
                n = f_vec / f_mag
                dot_prod = float(
                    np.clip(np.dot(engine.thrust_direction, n), -1.0, 1.0)
                )
                if dot_prod < 0:
                    throttles[i] = 0.0
                    gimbals[i, 0] = 0.0
                    gimbals[i, 1] = 0.0
                    continue

                angle = np.arccos(dot_prod)
                c = np.cross(engine.thrust_direction, n)
                s = float(np.linalg.norm(c))
                if s > 1e-6:
                    rot_vec = (c / s) * angle
                    gimbals[i, 0] = rot_vec[0]
                    gimbals[i, 1] = rot_vec[1]
                else:
                    gimbals[i, 0] = 0.0
                    gimbals[i, 1] = 0.0
            else:
                gimbals[i, 0] = 0.0
                gimbals[i, 1] = 0.0

        return throttles, gimbals

    @property
    def _max_iterations(self) -> int:
        """Maximum iterations for the allocation solver to prevent infinite loops."""
        return 10

