"""
Vessel models for the Digital Twin physics simulation.
"""
from abc import ABC, abstractmethod
import numpy as np
from src.common.engine import Engine

class VesselModel(ABC):
    """Abstract interface for the vessel's physical properties."""
    
    @property
    @abstractmethod
    def engines(self) -> list[Engine]:
        """List of engines attached to the vessel."""
        pass
        
    @property
    @abstractmethod
    def engine_tau(self) -> float:
        """First-order spool time constant [s] for the engines."""
        pass

    @abstractmethod
    def total_mass(self, fuel_mass_kg: float) -> float:
        """Returns dry_mass + fuel_mass_kg [kg]."""
        pass

    @abstractmethod
    def inertia_tensor(self, fuel_mass_kg: float) -> np.ndarray:
        """Returns the 3x3 inertia tensor for the current fuel state [kg*m^2]."""
        pass

    @abstractmethod
    def get_drag_force(self, velocity_ned: np.ndarray, density: float) -> np.ndarray:
        """Calculates aerodynamic drag vector in NED frame [N]."""
        pass

    @abstractmethod
    def get_fuel_burn_rate(self, actual_throttles: np.ndarray) -> float:
        """Calculates mass depletion rate based on current engine throttles [kg/s]."""
        pass

    @abstractmethod
    def get_com_position(self, fuel_mass_kg: float) -> np.ndarray:
        """Returns the Center of Mass position in the geometric body frame [m]."""
        pass
