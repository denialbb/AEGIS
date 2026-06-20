import numpy as np
import logging
from typing import List, Dict, Any
import src.config as config
from src.common.engine import Engine

logger = logging.getLogger(__name__)


class FaultDetectionIsolation:
    def __init__(self, threshold: float = 0.5, ekf=None):
        """
        threshold: Deviation in m/s^2 above which a fault is declared.
        ekf: Optional ErrorStateEKF instance for IMU health monitoring.
        """
        self.threshold: float = threshold
        self.persistence_ticks: int = (
            50  # 1 second at 50Hz to allow for engine spool up
        )
        self.consecutive_faults: int = 0
        self.ekf = ekf

        # IMU health monitoring parameters
        self.imu_fault_threshold: float = (
            config.EKF_INNOVATION_FAULT_THRESHOLD
            if hasattr(config, "EKF_INNOVATION_FAULT_THRESHOLD")
            else 5.0
        )
        self.imu_persistence_ticks: int = (
            10  # Shorter persistence for IMU faults
        )
        self.imu_consecutive_faults: int = 0
        self.imu_fault_reported: bool = False

    def detect_imu_fault(self) -> bool:
        """
        Check if the EKF innovation norm exceeds the threshold, indicating
        potential IMU sensor issues.

        Returns
        -------
        bool
            True if IMU fault detected, False otherwise.
        """
        if self.ekf is None:
            return False

        innovation_norm = self.ekf.get_innovation_norm()
        if innovation_norm > self.imu_fault_threshold:
            self.imu_consecutive_faults += 1
            if self.imu_consecutive_faults >= self.imu_persistence_ticks:
                if not self.imu_fault_reported:
                    logger.warning(
                        f"[FDI-IMU] Persistent IMU Fault Confirmed! Innovation norm: {innovation_norm:.3f} > {self.imu_fault_threshold}"
                    )
                    self.imu_fault_reported = True
                    return True
                else:
                    return False
        else:
            self.imu_consecutive_faults = 0
            self.imu_fault_reported = False
        return False

    def detect_fault(
        self, expected_accel: np.ndarray, measured_accel: np.ndarray
    ) -> bool:
        """
        Compares expected vs measured acceleration.
        Increments a persistence counter to filter out transients like engine spool-up.
        Returns True if the magnitude of the difference exceeds the threshold for N consecutive ticks.
        """
        diff = expected_accel - measured_accel
        deviation = np.linalg.norm(diff)
        if deviation > self.threshold:
            self.consecutive_faults += 1
            if self.consecutive_faults >= self.persistence_ticks:
                logger.warning(
                    f"[FDI] Persistent Fault Confirmed! Expected: {expected_accel}, Measured: {measured_accel}, Deviation: {deviation} > {self.threshold}"
                )
                return True
        else:
            self.consecutive_faults = 0

        return False

    def isolate_fault(
        self,
        active_engines: List[Engine],
        expected_throttles: np.ndarray,
        measured_accel: np.ndarray,
        mass: float,
        expected_forces: np.ndarray | None = None,
    ) -> List[int]:
        """
        Isolates which engine(s) failed.

        Parameters
        ----------
        active_engines : list of Engine
            Currently active engines.
        expected_throttles : ndarray (N,)
            Throttle values [0.0, 1.0] commanded in the previous step.
        measured_accel : ndarray (3,)
            Measured specific force from accelerometer.
        mass : float
            Current vessel mass.
        expected_forces : ndarray (N, 3) or None
            Pre-computed gimbal-aware force vectors per engine.  If None,
            falls back to thrust_direction * max_thrust * throttle (no gimbal).

        Returns
        -------
        list of int
            Engine indices that have suffered a fault.
        """
        if not active_engines:
            return []

        N = len(active_engines)
        if expected_forces is not None and expected_forces.shape == (N, 3):
            per_engine_forces = expected_forces
        else:
            # Fallback: no gimbal model (legacy path)
            per_engine_forces = np.zeros((N, 3))
            for i, engine in enumerate(active_engines):
                per_engine_forces[i] = (
                    engine.thrust_direction
                    * engine.max_thrust
                    * expected_throttles[i]
                )

        total_expected_force = per_engine_forces.sum(axis=0)
        missing_force = total_expected_force - (measured_accel * mass)
        force_tolerance = self.threshold * mass

        min_error = float("inf")
        best_combo: List[int] = []

        import itertools

        for num_failed in range(1, N + 1):
            for combo in itertools.combinations(
                enumerate(active_engines), num_failed
            ):
                combo_force = np.zeros(3)
                for i, engine in combo:
                    combo_force += per_engine_forces[i]

                error = float(np.linalg.norm(missing_force - combo_force))
                if error < min_error:
                    min_error = error
                    best_combo = [engine.index for i, engine in combo]

        if min_error < force_tolerance * 2.0:
            logger.warning(
                f"[FDI] Engines {best_combo} isolated due to fault detection."
            )
            return best_combo

        return best_combo
