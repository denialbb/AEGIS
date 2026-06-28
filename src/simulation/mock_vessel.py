import numpy as np
from src.simulation.vessel import VesselModel
from src.common.engine import Engine

class SimpleTestVessel(VesselModel):
    def __init__(self):
        # 1 engine at CoM pointing upwards (NED -Z)
        self._engines = [
            Engine(index=0, position=np.zeros(3), thrust_direction=np.array([0.0, 0.0, -1.0]), max_thrust=100.0)
        ]
        
    @property
    def engines(self) -> list[Engine]:
        return self._engines
        
    @property
    def engine_tau(self) -> float:
        return 0.1
        
    def total_mass(self, fuel_mass_kg: float) -> float:
        return 10.0 + fuel_mass_kg
        
    def inertia_tensor(self, fuel_mass_kg: float) -> np.ndarray:
        return np.eye(3)
        
    def get_drag_force(self, velocity_ned: np.ndarray, density: float) -> np.ndarray:
        return np.zeros(3)
        
    def get_fuel_burn_rate(self, actual_throttles: np.ndarray) -> float:
        return 1.0 * np.sum(actual_throttles)

    def get_com_position(self, fuel_mass_kg: float) -> np.ndarray:
        return np.zeros(3)

class MockVessel(VesselModel):
    """A realistic 4-engine quad vessel for the visualization sandbox."""
    def __init__(self):
        # 4 engines placed in a square around the CoM, pointing upwards (NED -Z)
        self._engines = [
            Engine(index=0, position=np.array([ 1.0,  1.0, 0.0]), thrust_direction=np.array([0.0, 0.0, -1.0]), max_thrust=500.0),
            Engine(index=1, position=np.array([-1.0,  1.0, 0.0]), thrust_direction=np.array([0.0, 0.0, -1.0]), max_thrust=500.0),
            Engine(index=2, position=np.array([-1.0, -1.0, 0.0]), thrust_direction=np.array([0.0, 0.0, -1.0]), max_thrust=500.0),
            Engine(index=3, position=np.array([ 1.0, -1.0, 0.0]), thrust_direction=np.array([0.0, 0.0, -1.0]), max_thrust=500.0),
        ]
        
    @property
    def engines(self) -> list[Engine]:
        return self._engines
        
    @property
    def engine_tau(self) -> float:
        return 0.1
        
    def total_mass(self, fuel_mass_kg: float) -> float:
        return 40.0 + fuel_mass_kg
        
    def inertia_tensor(self, fuel_mass_kg: float) -> np.ndarray:
        return np.diag([10.0, 10.0, 15.0])
        
    def get_drag_force(self, velocity_ned: np.ndarray, density: float) -> np.ndarray:
        return -0.5 * density * velocity_ned
        
    def get_fuel_burn_rate(self, actual_throttles: np.ndarray) -> float:
        return 1.0 * np.sum(actual_throttles)

    def get_com_position(self, fuel_mass_kg: float) -> np.ndarray:
        dry_mass = 40.0
        dry_com = np.array([0.0, 0.0, -4.0])
        fuel_com = np.array([0.0, 0.0, -6.0])
        total = dry_mass + fuel_mass_kg
        if total > 0:
            return (dry_mass * dry_com + fuel_mass_kg * fuel_com) / total
        return dry_com
