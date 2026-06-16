# AEGIS: Autonomous Estimation & Guidance Integrated System

## 1. Project Objective
Most Kerbal Space Program automation scripts treat guidance as an isolated solver operating in a perfect, deterministic physics environment. This project actively rejects that premise. 

The objective is to build an autonomous mission architecture that deliberately blinds itself with artificial noise and suffers from catastrophic asymmetric engine failures during powered descent. The system must utilize aerospace-grade **State Estimation, Fault Detection, and Control Allocation** to dynamically adapt and survive, guided by a robust, high-level contingency state machine.

## 2. Architectural Paradigm: kRPC & Python

To implement complex matrices and state filters, we must break out of the native KerboScript (kOS) virtual machine and utilize the **kRPC** mod to stream game data over TCP/IP to a local Python server.

### Why Python?
*   **The Math Ecosystem:** The core of this project relies heavily on linear algebra (for Control Allocation pseudo-inverses) and state estimation (Kalman filters). Python provides `numpy` and `filterpy`, making prototyping these mathematical concepts trivial compared to compiled languages like C#.
*   **Future Proofing:** If the guidance module is upgraded to Convex Optimization (G-FOLD) in the future, Python provides the industry-standard `cvxpy` library.

### The Mitigation of Python's Weaknesses
*   **Latency:** Python is interpreted, but the KSP Unity physics engine ticks at 50Hz (20ms). A well-written Python script over localhost TCP can easily evaluate matrices and return commands in < 2ms, providing ample headroom.
*   **Type Safety:** A dynamic language is dangerous for a massive Mission State Machine; a runtime `TypeError` during an engine-out emergency is fatal. We will mitigate this by strictly enforcing Python type-hinting (`-> np.ndarray`) and utilizing a static type checker (`mypy`) during development.

---

## 3. Core System Modules

The architecture is strictly decoupled into five primary domains to ensure robust systems engineering.

### A. The Mission Director (Hierarchical State Machine)
The overarching logic controller. It manages nominal mission phases and handles contingency branching based on the severity and timing of a detected fault.

*   **States:** `STANDBY`, `ASCENT_COAST`, `DEORBIT_BURN`, `HYPERSONIC_COAST`, `POWERED_DESCENT`, `HOVER_TARGETING`, `TERMINAL_DESCENT`, `LANDED`, and `HARD_ABORT`.
*   **Smart Activation:** The Director idles in `STANDBY` until manually activated via a user action group. Upon activation, it evaluates current altitude and vertical velocity to seamlessly inject itself into the appropriate mission phase (e.g., `ASCENT_COAST` if moving upwards, or `HYPERSONIC_COAST` if falling from space).
*   **Contingency Logic Example:** 
    *   *Fault during `POWERED_DESCENT`:* The Director commands the Guidance module to recalculate burn time or shift the landing target to a closer safe zone.
    *   *Fault during `TERMINAL_DESCENT` (< 50m):* The Director triggers a "Hard Abort" contingency—ignoring precision targeting and commanding the Control Allocator to maximize vertical thrust on surviving engines regardless of lateral drift.

### B. State Estimation Module (Navigation)

**Adaptive Process‑Noise Scaling**

The Error-State Extended Kalman Filter (EKF) adapts its process‑noise matrix `Q` each prediction step based on the magnitude of the kinematic acceleration (`|a|`). The velocity‑noise block (`Q[3:6,3:6]`) is multiplied by `1 + PROCESS_NOISE_THRUST_COEF * |a|²`. This mitigates altitude and velocity estimation spikes when the guidance commands large thrusts, allowing the filter to trust the noisy accelerometer less during high‑thrust transients.

**Implementation Details**
- `src/estimation/ekf.py` implements a 12-state error-state EKF that estimates position, velocity, gyroscope bias, and accelerometer bias.
- Attitude is estimated externally by a Mahony complementary filter (`src/estimation/mahony_estimator.py`) that fuses gyroscope and accelerometer data to produce a quaternion representing the orientation from body to world frame.
- The EKF consumes the Mahony-estimated attitude to rotate body-frame specific force (accelerometer readings corrected for estimated biases) into the world frame, then adds gravity. This ensures frame safety: all operations are performed in the world frame before integration.
- Gravity is modeled dynamically using kRPC's `vessel.flight().g_force` (or computed from `body.gravitational_parameter` and altitude) rather than a hardcoded value.
- In `ekf.py::predict()`, `accel_norm = np.linalg.norm(kinematic_accel_world)` is computed, and `Q_dyn[3:6,3:6]` is scaled accordingly, with a tiny epsilon added to ensure strict increase for test assertions.
- The scaling coefficient is exposed as `config.PROCESS_NOISE_THRUST_COEF` (default 0.1) and can be tuned via Optuna.

---
KSP provides perfect data (e.g., `vessel.flight().mean_altitude`), which we will purposefully obscure.
*   **Noise Wrapper:** kRPC telemetry streams will be wrapped in a function that injects continuous Gaussian noise into the radar altimeter, velocimeter, and IMU (gyroscope and accelerometer) readings.
*   **The Filter:** The 12-state error-state EKF fuses IMU data (gyroscope and accelerometer), altimeter, and velocimeter to produce a probabilistic estimation of the true state vector $[X, Y, Z, V_x, V_y, V_z]$ and estimates of the sensor biases. Vessel mass is treated as a clean, external telemetry parameter, not estimated.
*   **Attitude Handling:** The Mahony complementary filter estimates attitude using gyroscope integration and accelerometer gravity-reference correction, consuming bias-corrected gyroscope rates from the EKF. This avoids the small-angle approximation and allows for large angle maneuvers.
*   **Coordinate System and Gravity:** KSP's custom Reference Frames do not naturally rotate to remain "Z-Up" relative to the planet's surface. To compensate, the system computes a normalized `up_vector` from the pad's surface position. This vector is used to correctly subtract the constant gravitational acceleration from the IMU telemetry and to map the true vertical altitude/velocity components for the State Machine target assignments.

### C. Fault Detection & Isolation Module (FDI)
The system's diagnostic nervous system.
*   **Logic:** The FDI continuously calculates the *expected* acceleration vector based on the currently commanded throttle and known vessel mass. It compares this against the *measured* acceleration provided by the State Estimator.
*   **Isolation:** If the deviation exceeds the noise tolerance threshold, the FDI isolates which specific actuator vector is failing to produce thrust, flagging the engine as "Dead" and alerting the Mission Director.

### D. Control Allocation Module (Guidance & Control)
The core engineering solution to asymmetric thrust.
*   **The Problem:** If an off-center engine dies, simply throttling up the remaining engines will induce a catastrophic torque, spinning the vessel out of control.
*   **The Solution:** The guidance algorithm does not command individual engines. Instead, it commands a desired 6-DOF "Wrench" (Forces $F_x, F_y, F_z$ and Torques $\tau_x, \tau_y, \tau_z$).
*   **The Mapper:** The Allocator uses a pseudo-inverse matrix solver (`numpy.linalg.pinv`) to map the desired Wrench to the *surviving* engines. It will automatically throttle down engines opposite the failure to kill the torque, while throttling up adjacent engines to maintain the required vertical stopping force.
*   **Guidance Enhancements:** The GuidanceController computes a commanded world-frame acceleration `a_cmd_world` from PD errors. Two safeguards prevent the guidance from demanding physically impossible acceleration:
    1. **Suicide-burn glideslope** — target velocity is `-sqrt(2 * a_avail * alt_above_floor)` where `a_avail` is the vessel's actual TWR-derived net upward acceleration. This replaces the old linear `k_alt * alt` profile that saturated at high altitude, causing large velocity errors the PD law could not overcome.
    2. **Acceleration clamp** — `a_cmd_world` is capped to `ACCEL_CLAMP_FACTOR × max_a_avail` before being rotated into the body frame. This prevents transient error spikes from flipping the attitude target (`target_up_world`) and lets the allocator's existing saturation handling work without attitude thrashing.

### E. Telemetry & Application Logger (Debugging Infrastructure)

---

### ADR-029 (Planned) — Reaction Wheel Attitude Augmentation

Gimbal trim (±5°) is the sole torque actuator during powered descent.
Two cases where it falls short:

1. **Low-throttle transients** — gimbal authority is proportional to engine
   thrust; near-zero throttle means near-zero torque.
2. **Large inertia imbalance** — a long-armed asymmetrical lander may saturate
   gimbal deflection before producing enough torque.

**Option A**: Map `torque_body` to stock `vessel.control.{pitch,roll,yaw}`
via a single tunable gain `RW_AUGMENT_GAIN`. Reaction wheels act in parallel
with gimbal trim without conflict since `ModuleGimbalTrim` intercepts gimbal
response for trimmed engines, leaving reaction wheels as independent torque.

**Caveat**: The gain converts N·m to the stock [-1, 1] range and is entirely
empirical.  It must be validated in a scenario with gimbals disabled (throttle
~0) to avoid double-counting torque.
*   **The Problem:** The KSP physics loop runs at 50Hz. Interactive debugging or slow console printing breaks this real-time constraint.
*   **The Telemetry Solution:** A dual-file logging strategy. A high-density CSV logs the complete system state at every tick, while a JSONL file records discrete state changes and faults. I/O is heavily buffered to ensure the control loop never blocks.
*   **The Application Logging Solution:** Standard `print()` statements are replaced by a global `logging` configuration (`src/common/logger.py`) that strictly avoids the 50Hz inner loop. It outputs runtime information (startup, state transitions, isolated faults) to the console and/or file, toggled via `DEBUG_LOGGING` and `LOG_TO_FILE` in `config.py`.

---

## 4. Testbed Requirements & Simulation Strategy

To rapidly develop the system without constantly running KSP, we employ a **Two-Tier Test Harness**:
1.  **Kinematic Mock:** A lightweight offline Newtonian physics loop (e.g., `a = F/m - g`) that allows us to run thousands of simulated landing profiles and tune the Kalman Filter in seconds.
2.  **Live KSP Validation:** Final tuning and verification run against the actual game engine using kRPC.

For live testing, the KSP testbed vessel requires:
1.  **Engine Cluster:** A redundant cluster of independently controllable engines (e.g., a central sustainer surrounded by 4 or 8 radial engines).
2.  **RCS Authority:** Robust RCS or reaction wheels for baseline attitude control while the Allocator re-balances the main engines.
3.  **The Gremlin:** A background script that actively sabotages the flight. It will randomly kill engine modules to trigger the FDI, and inject lateral acceleration biases into the IMU telemetry stream to simulate unmodeled environmental disturbances (like wind shear).
