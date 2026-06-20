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
        Solves the 6DOF control allocation using equal-force allocation plus
        gimbal-based torque correction.

        Equal-force allocation distributes the total force equally across all
        engines. Gimbal angles are then biased from the commanded torque using
        the per-engine moment arms (r × F), producing torque without differential
        throttling. This avoids the aggressive attitude changes caused by
        differential throttle while still providing torque authority.

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

        N = len(active_engines)

        # Build 6×3N control effectiveness matrix
        B = self._build_B(active_engines, com)

        # Numerical safety: condition number check
        cond = float(np.linalg.cond(B))
        if cond > 1e4:
            raise AllocationDegenerateError(
                f"B matrix condition number {cond:.1e} > 1e4 — ill-conditioned allocation"
            )

        # Equal-force distribution: total force divided equally.
        # CRITICAL: We only distribute the AXIAL component of the desired force.
        # Rockets translate by pitching, not by gimballing laterally. If we try to produce
        # lateral force via gimbals, we create massive uncompensated torques that flip the vessel.
        # Gimbals should be reserved exclusively for attitude control (torque).
        main_thrust_dir = active_engines[0].thrust_direction if active_engines else np.array([0.0, 1.0, 0.0])
        axial_mag = max(0.0, float(np.dot(desired_wrench[:3], main_thrust_dir)))
        f_per = (axial_mag * main_thrust_dir) / N
        f_pad = np.tile(f_per, (N, 1))

        # Compute base throttles and gimbals from equal-force allocation
        throttles, gimbals, forces_out = self._forces_to_controls(f_pad, active_engines)

        # Now bias gimbals from the commanded torque.
        # Each engine at position r produces torque r × F_engine.
        # For a desired delta_torque, distribute the gimbal correction
        # across engines proportional to their moment arm.
        delta_torque = desired_wrench[3:6]
        torque_mag = float(np.linalg.norm(delta_torque))
        if torque_mag > 1e-8:
            # Per-engine moment arm magnitude (distance from CoM in XZ plane for roll,
            # X-Y plane for pitch). Use full 3D moment arm.
            moment_arms = np.array([
                float(np.linalg.norm(e.position)) + 1e-6
                for e in active_engines
            ])
            weights = moment_arms / moment_arms.sum()

            max_gimbal_bias_rad = np.deg2rad(3.0)
            for i, engine in enumerate(active_engines):
                w = weights[i]
                tau_correction = w * delta_torque
                r_vec = engine.position - com
                r_mag = float(np.linalg.norm(r_vec)) + 1e-6

                # Approximate lateral force needed: F_lat = tau / r
                # This is the force perturbation that creates the desired torque
                f_lat_vec = tau_correction / r_mag
                f_lat_mag = float(np.linalg.norm(f_lat_vec))

                if f_lat_mag > 1e-6 and engine.max_thrust > 0:
                    axial = throttles[i] * engine.max_thrust
                    max_lat = axial * np.tan(np.deg2rad(engine.max_gimbal_deg))

                    # Limit lateral force to available gimbal authority
                    f_lat_vec_limited = f_lat_vec
                    if f_lat_mag > max_lat:
                        f_lat_vec_limited = (f_lat_vec / f_lat_mag) * max_lat

                    # Project lateral force onto engine gimbal axes.
                    # Use full 3D vectors — arctan2 gives angle in the plane
                    # of the gimbal axis and thrust direction.
                    gx_rad = float(np.arctan2(
                        np.dot(f_lat_vec_limited, engine.gimbal_y_axis),
                        axial + 1e-6,
                    ))
                    gy_rad = float(-np.arctan2(
                        np.dot(f_lat_vec_limited, engine.gimbal_x_axis),
                        axial + 1e-6,
                    ))

                    max_rad = float(np.deg2rad(engine.max_gimbal_deg))
                    gx_bias = float(np.clip(gx_rad, -max_gimbal_bias_rad, max_gimbal_bias_rad))
                    gy_bias = float(np.clip(gy_rad, -max_gimbal_bias_rad, max_gimbal_bias_rad))

                    gimbals[i, 0] = np.clip(gimbals[i, 0] + gx_bias, -max_rad, max_rad)
                    gimbals[i, 1] = np.clip(gimbals[i, 1] + gy_bias, -max_rad, max_rad)

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

