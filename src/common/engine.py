import numpy as np
from typing import Any

class Engine:
    def __init__(self, index: int, position: np.ndarray, thrust_direction: np.ndarray, max_thrust: float, part: Any = None):
        """
        Represents an individual propulsion unit on the vessel.
        
        Args:
            index: Unique identifier for the engine.
            position: 3D position vector relative to Center of Mass [m]
            thrust_direction: 3D unit vector of thrust direction in vessel frame
            max_thrust: Maximum thrust capacity [N]
            part: Reference to the kRPC part object for actual actuation
        """
        self.index: int = index
        self.position: np.ndarray = position
        self.thrust_direction: np.ndarray = thrust_direction
        self.max_thrust: float = max_thrust
        self.active: bool = True  # Status flag managed by FDI
        self.expected_throttle: float = 0.0 # EMA filtered throttle representing current physical state
        self.part: Any = part # Reference to the underlying kRPC part for actuation
