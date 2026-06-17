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

### Reference Frame Utilities (`src/common/reference_frame.py`)

Reusable kRPC-integration functions for building and querying the NED navigation frame. The pure-math rotation logic lives in `src/common/geometry.py::ecef_to_ned()`; this module wraps it for kRPC interaction.

```python
def build_ned_frame(conn, body, target_lat, target_lon) -> tuple[Any, np.ndarray]:
    """
    Build a kRPC ReferenceFrame aligned to North-East-Down at the target pad.
    Returns (ned_frame, up_vector=[0,0,-1]).
    Used by MissionDirector._init_reference_frame() and flight_recorder._init_sensors().
    """

def get_pad_ecef(body, target_lat, target_lon) -> np.ndarray:
    """Surface position of the landing site in ECEF (body-centred) coordinates."""

def compute_gravity_ned(body, pos_ecef) -> np.ndarray:
    """Gravitational acceleration [0, 0, +g] in NED from ECEF position."""

def get_vessel_position_ned(vessel, ned_frame) -> np.ndarray:
    """(3,) position in NED frame."""

def get_vessel_velocity_ned(vessel, ned_frame) -> np.ndarray:
    """(3,) velocity in NED frame. Downward positive."""

def get_vessel_altitude_ned(vessel, ned_frame) -> float:
    """Surface altitude in metres."""

def get_vessel_state_ned(vessel, ned_frame) -> tuple[np.ndarray, np.ndarray, float]:
    """Convenience — (pos, vel, alt) in one call."""
```

---

## 2. State Estimator (`src/estimation/estimator.py`)

Fuses noisy telemetry into a clean state vector using an Error-State Extended Kalman Filter (ADR-030).

### State Vector (EKF — 12 states)
$$\mathbf{x} = \begin{bmatrix} x & y & z & v_x & v_y & v_z & b_{gx} & b_{gy} & b_{gz} & b_{ax} & b_{ay} & b_{az} \end{bmatrix}^T$$
- Position $[x, y, z]$ (in NED frame, origin at landing pad)
- Velocity $[v_x, v_y, v_z]$ (in NED frame)
- Gyroscope bias $[b_{gx}, b_{gy}, b_{gz}]$ (rad/s)
- Accelerometer bias $[b_{ax}, b_{ay}, b_{az}]$ (m/s²)

Note: Mass is treated as an external clean telemetry parameter, not estimated in the filter state.

### Attitude Estimation (separate Mahony filter)
Attitude is estimated by a Mahony complementary filter (`src/estimation/mahony_estimator.py`) that fuses gyroscope and accelerometer data. The Mahony filter is initialized from kRPC truth attitude during the `SENSOR_WARMUP` phase. It produces a quaternion $q_{body \to NED}$ used to rotate body-frame specific force into the NED frame.

### Sensor Interface (`src/telemetry/sensors.py`)
```python
class SensorModels:
    def __init__(self, conn, vessel, ned_frame, up_vector):
        ...

    def poll(self) -> tuple:
        """
        Returns 10-tuple:
        0. noisy_alt      : float          — radar altitude with Gaussian noise [m]
        1. sf_body        : ndarray (3,)   — body-frame specific force (noisy) [m/s²]
        2. attitude       : ndarray (4,)   — Mahony quaternion [x,y,z,w] (scalar-last)
        3. mass           : float          — vessel mass [kg] (clean telemetry)
        4. aero_body      : ndarray (3,)   — body-frame aerodynamic force [N]
        5. situation      : str            — vessel situation (e.g. "flying", "landed")
        6. omega_body     : ndarray (3,)   — body-frame angular rates [rad/s]
        7. noisy_vel      : ndarray (3,)   — NED velocity with Gaussian noise [m/s]
        8. gravity_ned    : ndarray (3,)   — gravity in NED frame [m/s²]
        9. raw_gyro       : ndarray (3,)   — raw gyro readings (noisy, no bias correction) [rad/s]
        """
        ...

    def get_truth_attitude(self) -> np.ndarray:
        """
        Returns the raw kRPC attitude quaternion (body→NED) for filter initialization.
        Used during SENSOR_WARMUP tick 0.
        """
        ...
```

### Error-State EKF Interface (`src/estimation/ekf.py`)
```python
class ErrorStateEKF:
    def __init__(self, ...):
        """12-state error-state EKF. State: pos(3), vel(3), gyro_bias(3), accel_bias(3)."""
        pass

    def predict(self, gyro_body: np.ndarray, sf_body: np.ndarray, 
                attitude: np.ndarray, dt: float, gravity_ned: np.ndarray):
        """
        Predict step using gyroscope (for attitude propagation) and accelerometer
        (for position/velocity). Both are corrected for estimated biases.
        gravity_ned is used to reconstruct kinematic acceleration from specific force.
        """
        pass

    def update(self, noisy_alt: float, noisy_vel: np.ndarray, 
               vel_cov: np.ndarray | None = None):
        """
        Fuses noisy altimeter and velocimeter data to correct the state.
        vel_cov is optional; defaults to config.SIGMA_VEL diagonal.
        """
        pass

    def get_state(self) -> np.ndarray:
        """Returns the current estimated state vector, shape (12,)."""
        pass

    def get_innovation(self) -> np.ndarray:
        """Returns the most recent innovation vector [alt_inn, vel_inn_x, vel_inn_y, vel_inn_z]."""
        pass
```

### Sensor Sub-Modules
- `src/estimation/gyro_sensor.py` — `GyroSensor`: streams `vessel.angular_velocity(ned_frame)`, applies Gaussian noise with configurable $\sigma_{gyro}$ and bias instability.
- `src/estimation/accelerometer_sensor.py` — `AccelerometerSensor`: streams `flight(ned_frame).velocity`, differentiates to compute coordinate acceleration, subtracts gravity computed from `body.gravitational_parameter / r²` to produce specific force, applies Gaussian noise.
- `src/estimation/mahony_estimator.py` — `MahonyAttitudeEstimator`: quaternion-based complementary filter fusing gyroscope (high-rate integration) and accelerometer (gravity reference correction). Tunable $K_p$ and $K_i$ gains.

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
- `STANDBY`
- `ASCENT_COAST`
- `DEORBIT_BURN`
- `HYPERSONIC_COAST`
- `SENSOR_WARMUP`
- `ESTIMATOR_WARMUP`
- `POWERED_DESCENT`
- `HOVER_TARGETING`
- `TERMINAL_DESCENT`
- `LANDED`
- `HARD_ABORT`

### Module Structure
The Mission Director has been refactored into specialized submodules:
- `src/main.py` — `MissionDirector` class: initialization, reference frame, engines, sensors, estimator, guidance, telemetry, HUD.
- `src/mission/loop.py` — Main control loop orchestrator: timing, sensor polling, pause detection, telemetry logging.
- `src/mission/flight_control.py` — Flight-critical functions: SAS, FDI, state machine transitions, allocation, engine management, throttle, estimator predict/update, abort handling.
- `src/mission/helpers.py` — Pure helper functions: `unpack_sensor_poll()`, `compute_a_avail()`, `build_fuel_state()`.
- `src/mission/ui.py` — Telemetry-frame assembly and HUD display.
- `src/mission/states.py` — `MissionState` enum definition.

The `MissionDirector` owns all state and delegates per-tick work to imported functions from these submodules. No state is held in the submodules — they are stateless callables.

### Sensor Warmup Phase
Upon activation (if airborne and descending) the Director enters `SENSOR_WARMUP`:
1. **Tick 0:** Mahony filter initialized from kRPC truth attitude via `SensorModels.get_truth_attitude()`. Gyro/accel sample buffers start accumulating.
2. **Ticks 1–29 (3 s):** Sensor data streams, biases accumulate via `np.mean()`. No guidance or allocation runs.
3. **Tick 30:** EKF background biases (`bg`, `ba`) set to accumulated means. EKF covariance `P[6:9]` (gyro bias) reduced by `SENSOR_WARMUP_GYRO_BIAS_SIGMA`, `P[9:12]` (accel bias) by `SENSOR_WARMUP_ACCEL_BIAS_SIGMA`.
4. Transition to `ESTIMATOR_WARMUP` (100 ticks) or directly to coast/descent if altitude/velocity thresholds met.

### Contingencies
- **Single Engine Failure:** FDI flags, active engines reduced. Allocator remaps wrench.
- **Degenerate Allocation:** Control Allocator raises `AllocationDegenerateError`. MD immediately transitions to `HARD_ABORT`.
- **Multiple Simultaneous Failures:** FDI returns 2+ failed engines. MD immediately transitions to `HARD_ABORT`.
- **DT_SPIKE / KSP Pause:** If `dt > 3 * expected_dt`, the MD:
  - Skips the Kalman filter predict step to avoid divergence (velocity state may become stale)
  - **Always runs** the Guidance controller during powered descent phases (never gated on skip_predict)
  - Skips FDI fault detection, holding the last known good `expected_accel`
  - Logs a `DT_SPIKE` event with actual vs expected dt
  - **Note:** Guidance continues to command thrust to maintain control during degraded states, preventing uncontrolled free-fall.

### Interface
```python
class MissionDirector:
    def __init__(self, conn):
        """
        conn: kRPC connection object
        """
        self.conn = conn
        self.state: str = "STANDBY"
        self.estimator: StateEstimator = ...
        self.fdi: FaultDetectionIsolation = ...
        self.allocator: ControlAllocator = ...
        self.writer: TelemetryWriter = ...  # Owns the logging infrastructure

    def run_loop(self) -> bool:
        """
        Executes the main loop at 10Hz to 50Hz, polling telemetry,
        updating the estimator, running the FDI, computing control wrench,
        allocating thrust, and transitioning states.

        Returns:
            True if the mission landed successfully, False on HARD_ABORT or failure.
        """
        return True
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
