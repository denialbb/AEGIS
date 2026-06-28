"""
Environment models for the Digital Twin physics simulation.
"""
from abc import ABC, abstractmethod

class EnvironmentModel(ABC):
    """Abstract interface for the physical environment outside the vessel."""
    
    @abstractmethod
    def gravity(self, altitude: float) -> float:
        """Returns gravitational acceleration at the given altitude [m/s^2]."""
        pass
        
    @abstractmethod
    def air_density(self, altitude: float) -> float:
        """Returns atmospheric density at the given altitude [kg/m^3]."""
        pass

class VacuumEnvironment(EnvironmentModel):
    """A simple vacuum environment with uniform gravity."""
    def __init__(self, g: float = 9.80665):
        self.g = g

    def gravity(self, altitude: float) -> float:
        return self.g

    def air_density(self, altitude: float) -> float:
        return 0.0
