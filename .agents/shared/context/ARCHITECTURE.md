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
$$\mathbf{x} = \begin{bmatrix} x & y & z & v_x & v_y & v_z \end{bmatrix}^T$$
- Position $[x, y, z]$ (relative to landing target or reference frame)
- Velocity $[v_x, v_y, v_z]$

Note: Mass is treated as an external clean telemetry parameter, not estimated in the filter state.

### Interface
```python
class StateEstimator:
    def __init__(self, initial_state: np.ndarray, initial_covariance: np.ndarray, process_noise: np.ndarray, measurement_noise: np.ndarray):
        """
        Initializes the Discrete-Time Kalman Filter.
        initial_state: shape (6,)
        initial_covariance: shape (6, 6)
        process_noise: shape (6, 6)
        measurement_noise: shape (1, 1) (altitude measurement variance)
        """
        pass

    def predict(self, noisy_accel_body: np.ndarray, attitude: np.ndarray, dt: float):
        """
        Predicts the next state using the measured acceleration as the control input (Option A).
        noisy_accel_body: Accelerometer reading in vessel body frame.
        attitude: Vessel attitude (e.g., quaternion) required to rotate acceleration to world frame.
        """
        pass

    def update(self, noisy_alt: float):
        """
        Fuses noisy altimeter data to correct the Z-axis state.
        """
        pass

    def get_state(self) -> np.ndarray:
        """
        Returns the current estimated state vector of shape (6,).
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
        Raises:
            AllocationDegenerateError: If the condition number of B exceeds the defined threshold.
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

### Contingencies
- **Single Engine Failure:** FDI flags, active engines reduced. Allocator remaps wrench.
- **Degenerate Allocation:** Control Allocator raises `AllocationDegenerateError`. MD immediately transitions to `HARD_ABORT`.
- **Multiple Simultaneous Failures:** FDI returns 2+ failed engines. MD immediately transitions to `HARD_ABORT`.
- **DT_SPIKE / KSP Pause:** If `dt > 3 * expected_dt`, the MD skips the Kalman filter predict step to avoid divergence and logs a `DT_SPIKE` event.

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
        self.writer: TelemetryWriter = ...  # Owns the logging infrastructure

    def run_loop(self):
        """
        Executes the main loop at 10Hz to 50Hz, polling telemetry,
        updating the estimator, running the FDI, computing control wrench,
        allocating thrust, and transitioning states.
        """
        pass
```

---

## 6. Telemetry Logger (`src/telemetry/`)

High-performance, non-blocking telemetry and event logging infrastructure to provide the primary debug surface. Owned exclusively by the Mission Director (ADR-013).

### TelemetryFrame
A typed dataclass (`src/telemetry/frame.py`) containing the complete system state snapshot for a single 20ms physics tick. Flattens nested arrays into CSV columns.

### Interface
```python
class TelemetryWriter:
    def __init__(self, run_config: dict):
        """
        Creates a timestamped run directory and updates the `logs/latest` symlink.
        Serializes `run_config` to `run_config.json`.
        """
        pass

    def log_tick(self, frame: TelemetryFrame):
        """
        Flattens the frame and appends it to the heavily-buffered `telemetry.csv`.
        """
        pass

    def log_event(self, event: dict):
        """
        Appends a discrete, structured event to `events.jsonl` (e.g., STATE_TRANSITION, FAULT_DETECTED).
        """
        pass
```

---

## 7. Physical Execution Topology

KSP and the kRPC server run on the Windows host. AEGIS runs in the Arch WSL2 environment. Because WSL2 operates within a Hyper-V VM with its own virtual network adapter, AEGIS cannot reach KSP via `localhost` (127.0.0.1) by default unless Windows 11 `localhostforwarding` is explicitly enabled in `.wslconfig`.

To guarantee connection reliability, the kRPC client must resolve the Windows host IP dynamically (typically found in `/etc/resolv.conf` under the `nameserver` entry) rather than hardcoding `localhost`.

### Interface Requirements
- The Python process must explicitly read the WSL host IP at startup.
- All high-frequency telemetry uses `conn.add_stream()` to push data via the kRPC stream port (50001) rather than pulling via RPC calls (50000).
