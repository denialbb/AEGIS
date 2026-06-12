# AEGIS Module Contracts & Interface Architecture

This document defines the strict API contracts, data types, inputs, outputs, and interfaces for the four core modules of AEGIS (Autonomous Estimation & Guidance Integrated System). All implementations must conform strictly to these contracts, including static type hints.

---

## 1. Common Data Structures

### Engine
Represents an individual propulsion unit on the vessel. Used by the FDI and Control Allocator modules.
```python
import numpy as np

class Engine:
    def __init__(self, index: int, position: np.ndarray, thrust_direction: np.ndarray, max_thrust: float):
        self.index: int = index
        self.position: np.ndarray = position          # 3D position vector relative to Center of Mass [m]
        self.thrust_direction: np.ndarray = thrust_direction  # 3D unit vector of thrust direction in vessel frame
        self.max_thrust: float = max_thrust            # Maximum thrust capacity [N]
        self.active: bool = True                       # Status flag managed by FDI
```

---

## 2. State Estimator (`src/estimation/estimator.py`)

Fuses noisy telemetry into a clean state vector.

### State Vector
$$\mathbf{x} = \begin{bmatrix} x & y & z & v_x & v_y & v_z & m \end{bmatrix}^T$$
- Position $[x, y, z]$ (relative to landing target or reference frame)
- Velocity $[v_x, v_y, v_z]$
- Mass $[m]$

### Interface
```python
class StateEstimator:
    def __init__(self, initial_state: np.ndarray, initial_covariance: np.ndarray, process_noise: np.ndarray, measurement_noise: np.ndarray):
        """
        Initializes the Discrete-Time Kalman Filter.
        initial_state: shape (7,)
        initial_covariance: shape (7, 7)
        process_noise: shape (7, 7)
        measurement_noise: shape (4, 4) (fuses 3D acceleration + 1D altitude)
        """
        pass

    def update(self, noisy_alt: float, noisy_accel: np.ndarray, dt: float) -> np.ndarray:
        """
        Fuses noisy altimeter and accelerometer data to update the state estimate.
        Returns the updated state vector.
        """
        pass

    def get_state(self) -> np.ndarray:
        """
        Returns the current estimated state vector of shape (7,).
        """
        pass
```

---

## 3. Fault Detection & Isolation (FDI) (`src/fdi/fdi.py`)

Monitors deviations between expected and measured acceleration to isolate failed engines.

### Interface
```python
from typing import List

class FaultDetectionIsolation:
    def __init__(self, threshold: float = 0.5):
        """
        threshold: Deviation in m/s^2 above which a fault is declared.
        """
        self.threshold: float = threshold

    def detect_fault(self, expected_accel: np.ndarray, measured_accel: np.ndarray) -> bool:
        """
        Compares expected vs measured acceleration.
        Returns True if the magnitude of the difference exceeds the threshold.
        """
        pass

    def isolate_fault(self, active_engines: List[Engine], expected_throttles: np.ndarray, 
                      measured_accel: np.ndarray, mass: float) -> List[int]:
        """
        Isolates which engine(s) failed.
        expected_throttles: array of throttle values [0.0, 1.0] commanded in the previous step, shape (N,)
        Returns a list of engine indices that have suffered a fault.
        """
        pass
```

---

## 4. Control Allocator (`src/guidance/allocator.py`)

Maps a 6-DOF Wrench vector (3 forces, 3 torques) to surviving engine throttle settings.

### Wrench Vector
$$\mathbf{W} = \begin{bmatrix} F_x & F_y & F_z & \tau_x & \tau_y & \tau_z \end{bmatrix}^T$$

### Interface
```python
class ControlAllocator:
    def __init__(self, engines: List[Engine]):
        self.engines: List[Engine] = engines

    def allocate(self, desired_wrench: np.ndarray, active_engines: List[Engine]) -> tuple[np.ndarray, np.ndarray]:
        """
        Solves the control allocation problem: W = B * u
        where B is the control effectiveness matrix of shape (6, 3N)
        and u is the 3D force vector for each engine of shape (3N,).
        Uses pseudo-inverse numpy.linalg.pinv solver to find u, then maps to throttles and gimbal angles.
        Returns:
            throttles: array of shape (N,) bounded between 0.0 and 1.0.
            gimbals: array of shape (N, 2) representing X/Y gimbal angles in radians.
        """
        pass

```

---

## 5. Mission Director (`src/main.py`)

State machine that manages nominal mission phases and handles contingencies.

### States
- `DEORBIT_BURN`
- `HYPERSONIC_COAST`
- `POWERED_DESCENT`
- `HOVER_TARGETING`
- `TERMINAL_DESCENT`
- `HARD_ABORT`

### Interface
```python
class MissionDirector:
    def __init__(self, conn):
        """
        conn: kRPC connection object
        """
        self.conn = conn
        self.state: str = "DEORBIT_BURN"
        self.estimator: StateEstimator = ...
        self.fdi: FaultDetectionIsolation = ...
        self.allocator: ControlAllocator = ...

    def run_loop(self):
        """
        Executes the main loop at 10Hz to 50Hz, polling telemetry,
        updating the estimator, running the FDI, computing control wrench,
        allocating thrust, and transitioning states.
        """
        pass
```
