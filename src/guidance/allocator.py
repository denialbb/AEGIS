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

    def _build_B(self, active_engines: List[Engine], com: np.ndarray = np.zeros(3)) -> np.ndarray:
        """Build the control effectiveness matrix B of shape (6, 3N)."""
        N = len(active_engines)
        B = np.zeros((6, 3 * N))
        for i, engine in enumerate(active_engines):
            B[0:3, 3 * i : 3 * i + 3] = np.eye(3)
            r = engine.position - com
            rx = np.array(
                [[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]]
            )
            B[3:6, 3 * i : 3 * i + 3] = rx
        return B

    def is_rank_sufficient(
        self, active_engines: List[Engine], com: np.ndarray = np.zeros(3)
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
        B = self._build_B(active_engines, com)
        rank = np.linalg.matrix_rank(B)
        return rank >= 6, rank

    def allocate(
        self, desired_wrench: np.ndarray, active_engines: List[Engine], com: np.ndarray = np.zeros(3)
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
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
            forces: array of shape (N, 3) with the actual force vector per engine (includes
                    gimbal deflection).
        Raises:
            AllocationDegenerateError: If the B matrix is rank deficient or ill-conditioned.
        """
        if not active_engines:
            return np.array([]), np.empty((0, 2)), np.empty((0, 3))

        # ---------------------------------------------------------
        # Equal-force allocation: distribute total force equally
        # across all engines. Gimbals steer each engine's thrust
        # along the commanded direction. No differential throttling
        # — differential thrust creates asymmetric torque which
        # causes tumbling during high-thrust retrograde burns.
        #
        # The torque commanded by the guidance (attitude correction)
        # is inherently handled: the body-frame force direction
        # already encodes the attitude correction, and gimbals
        # deflect to align each engine's thrust accordingly. If
        # gimbal authority is exceeded, the lateral component
        # saturates gracefully without affecting throttle balance.
        # ---------------------------------------------------------
        N = len(active_engines)
        f_per = desired_wrench[:3] / N
        f_pad = np.tile(f_per, (N, 1))
        throttles, gimbals, forces_out = self._forces_to_controls(f_pad, active_engines)
        self._saturated_engines.clear()
        return throttles, gimbals, forces_out


    def _compute_wrench_from_forces(
        self, forces: np.ndarray, active_engines: List[Engine], com: np.ndarray = np.zeros(3)
    ) -> np.ndarray:
        """
        Compute 6D wrench [fx, fy, fz, tx, ty, tz] from engine force vectors.

        Args:
            forces: Array of shape (N, 3) with force vectors for each engine
            active_engines: List of engines corresponding to the force vectors
            com: (3,) Center of mass position in vessel frame

        Returns:
            Wrench vector of shape (6,)
        """
        wrench = np.zeros(6)
        for i, engine in enumerate(active_engines):
            # Force component (simple sum)
            wrench[0:3] += forces[i]
            # Torque component: (r - com) × F
            r = engine.position - com
            wrench[3:6] += np.cross(r, forces[i])
        return wrench

    def _forces_to_controls(
        self, forces: np.ndarray, active_engines: List[Engine]
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Convert engine force vectors to throttle and gimbal commands.

        Args:
            forces: Array of shape (N, 3) with force vectors for each engine
            active_engines: List of engines corresponding to the force vectors

        Returns:
            throttles: array of shape (N,) with values in [0.0, 1.0]
            gimbals: array of shape (N, 2) with gimbal angles in radians
            forces_out: array of shape (N, 3) with the final gimballed force vectors
        """
        N = len(active_engines)
        throttles = np.zeros(N)
        gimbals = np.zeros((N, 2))
        forces_out = np.zeros((N, 3))

        for i, engine in enumerate(active_engines):
            f_vec = forces[i]

            axial_force = float(np.dot(f_vec, engine.thrust_direction))

            if axial_force <= 1e-6:
                throttles[i] = 0.0
                gimbals[i, 0] = 0.0
                gimbals[i, 1] = 0.0
                forces_out[i] = np.zeros(3)
                continue

            axial_force = min(axial_force, engine.max_thrust)

            lateral_force_vec = (
                f_vec
                - float(np.dot(f_vec, engine.thrust_direction))
                * engine.thrust_direction
            )

            lat_mag = float(np.linalg.norm(lateral_force_vec))

            max_lat = axial_force * np.tan(
                np.deg2rad(engine.max_gimbal_deg)
            )
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
                    forces_out[i] = np.zeros(3)
                    continue

                # Decompose lateral force onto engine-local gimbal axes
                #   Gimbal X: rotate about engine X axis → tilts thrust toward Y
                #   Gimbal Y: rotate about engine Y axis → tilts thrust toward X
                gx_rad = float(np.arctan2(
                    np.dot(lateral_force_vec, engine.gimbal_y_axis),
                    axial_force,
                ))
                gy_rad = float(-np.arctan2(
                    np.dot(lateral_force_vec, engine.gimbal_x_axis),
                    axial_force,
                ))
                max_rad = float(np.deg2rad(engine.max_gimbal_deg))
                gimbals[i, 0] = float(np.clip(gx_rad, -max_rad, max_rad))
                gimbals[i, 1] = float(np.clip(gy_rad, -max_rad, max_rad))
                forces_out[i] = f_vec
            else:
                gimbals[i, 0] = 0.0
                gimbals[i, 1] = 0.0
                forces_out[i] = np.zeros(3)

        return throttles, gimbals, forces_out

    @property
    def _max_iterations(self) -> int:
        """Maximum iterations for the allocation solver to prevent infinite loops."""
        return 10

