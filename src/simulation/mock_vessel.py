import numpy as np
from src.simulation.vessel import VesselModel
from src.common.engine import Engine

class SimpleTestVessel(VesselModel):
    """Minimal 1-engine vessel used for unit tests and quick smoke runs."""
    def __init__(self):
        self.dry_mass: float = 10.0
        self.dry_com: np.ndarray = np.zeros(3)
        self.fuel_com: np.ndarray = np.zeros(3)
        self.max_thrust: float = 100.0
        self._engine_tau: float = 0.1

        # 1 engine at CoM pointing upwards (NED -Z)
        self._engines = [
            Engine(
                index=0,
                position=self.dry_com.copy(),
                thrust_direction=np.array([0.0, 0.0, -1.0]),
                max_thrust=self.max_thrust,
            )
        ]

    @property
    def engines(self) -> list[Engine]:
        return self._engines

    @property
    def engine_tau(self) -> float:
        return self._engine_tau

    def total_mass(self, fuel_mass_kg: float) -> float:
        return self.dry_mass + fuel_mass_kg

    def inertia_tensor(self, fuel_mass_kg: float) -> np.ndarray:
        return np.eye(3)

    def get_drag_force(self, velocity_ned: np.ndarray, density: float) -> np.ndarray:
        return np.zeros(3)

    def get_fuel_burn_rate(self, actual_throttles: np.ndarray) -> float:
        return 1.0 * np.sum(actual_throttles)

    def get_com_position(self, fuel_mass_kg: float) -> np.ndarray:
        total = self.dry_mass + fuel_mass_kg
        if total > 0:
            return (self.dry_mass * self.dry_com + fuel_mass_kg * self.fuel_com) / total
        return self.dry_com

class MockVessel(VesselModel):
    """A realistic 4-engine quad vessel for the visualization sandbox."""
    def __init__(self):
        self.dry_mass: float = 40.0
        self.dry_com: np.ndarray = np.array([0.0, 0.0, -4.0])
        self.fuel_com: np.ndarray = np.array([0.0, 0.0, -6.0])
        self.max_thrust: float = 500.0
        self._engine_tau: float = 0.1
        # Simplified: constant inertia ignoring fuel redistribution;
        # production vessel would compute fuel-tank contribution.
        self.inertia_diag: np.ndarray = np.array([10.0, 10.0, 15.0])

        # 4 engines placed in a square around the CoM, pointing upwards (NED -Z)
        self._engines = [
            Engine(index=0, position=np.array([ 1.0,  1.0, 0.0]), thrust_direction=np.array([0.0, 0.0, -1.0]), max_thrust=self.max_thrust),
            Engine(index=1, position=np.array([-1.0,  1.0, 0.0]), thrust_direction=np.array([0.0, 0.0, -1.0]), max_thrust=self.max_thrust),
            Engine(index=2, position=np.array([-1.0, -1.0, 0.0]), thrust_direction=np.array([0.0, 0.0, -1.0]), max_thrust=self.max_thrust),
            Engine(index=3, position=np.array([ 1.0, -1.0, 0.0]), thrust_direction=np.array([0.0, 0.0, -1.0]), max_thrust=self.max_thrust),
        ]

    @property
    def engines(self) -> list[Engine]:
        return self._engines

    @property
    def engine_tau(self) -> float:
        return self._engine_tau

    def total_mass(self, fuel_mass_kg: float) -> float:
        return self.dry_mass + fuel_mass_kg

    def inertia_tensor(self, fuel_mass_kg: float) -> np.ndarray:
        return np.diag(self.inertia_diag)

    def get_drag_force(self, velocity_ned: np.ndarray, density: float) -> np.ndarray:
        return -0.5 * density * velocity_ned

    def get_fuel_burn_rate(self, actual_throttles: np.ndarray) -> float:
        return 1.0 * np.sum(actual_throttles)

    def get_com_position(self, fuel_mass_kg: float) -> np.ndarray:
        total = self.dry_mass + fuel_mass_kg
        if total > 0:
            return (self.dry_mass * self.dry_com + fuel_mass_kg * self.fuel_com) / total
        return self.dry_com
