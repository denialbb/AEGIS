import numpy as np
from typing import Any


class Engine:
    def __init__(
        self,
        index: int,
        position: np.ndarray,
        thrust_direction: np.ndarray,
        max_thrust: float,
        max_gimbal_deg: float = 0.0,
        part: Any = None,
        gimbal_x_axis: np.ndarray | None = None,
        gimbal_y_axis: np.ndarray | None = None,
    ):
        """
        Represents an individual propulsion unit on the vessel.

        Args:
            index: Unique identifier for the engine.
            position: 3D position vector relative to Center of Mass [m]
            thrust_direction: 3D unit vector of thrust direction in vessel frame
            max_thrust: Maximum thrust capacity [N]
            max_gimbal_deg: Maximum gimbal deflection in degrees (half-cone angle)
            part: Reference to the kRPC part object for actual actuation
            gimbal_x_axis: Engine-local X axis in vessel frame (gimbal rotates
                thrust around this axis).  Must be perpendicular to thrust_direction.
            gimbal_y_axis: Engine-local Y axis in vessel frame (gimbal rotates
                thrust around this axis).  Must be perpendicular to thrust_direction
                and gimbal_x_axis.
        """
        self.index: int = index
        self.position: np.ndarray = position
        self.thrust_direction: np.ndarray = thrust_direction
        self.max_thrust: float = max_thrust
        self.active: bool = True
        self.expected_throttle: float = 0.0
        self.max_gimbal_deg: float = max_gimbal_deg
        self.part: Any = part
        if gimbal_x_axis is not None and gimbal_y_axis is not None:
            self.gimbal_x_axis = gimbal_x_axis
            self.gimbal_y_axis = gimbal_y_axis
        else:
            arbitrary = np.array([1.0, 0.0, 0.0])
            if abs(np.dot(thrust_direction, arbitrary)) > 0.9:
                arbitrary = np.array([0.0, 1.0, 0.0])
            gx = np.cross(thrust_direction, arbitrary)
            gx /= np.linalg.norm(gx) + 1e-12
            gy = np.cross(thrust_direction, gx)
            gy /= np.linalg.norm(gy) + 1e-12
            self.gimbal_x_axis = gx
            self.gimbal_y_axis = gy
