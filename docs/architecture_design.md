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

*   **States:** `STANDBY`, `ASCENT_COAST`, `DEORBIT_BURN`, `HYPERSONIC_COAST`, `POWERED_DESCENT`, `HOVER_TARGETING`, `TERMINAL_DESCENT`, and `HARD_ABORT`.
*   **Smart Activation:** The Director idles in `STANDBY` until manually activated via a user action group. Upon activation, it evaluates current altitude and vertical velocity to seamlessly inject itself into the appropriate mission phase (e.g., `ASCENT_COAST` if moving upwards, or `HYPERSONIC_COAST` if falling from space).
*   **Contingency Logic Example:** 
    *   *Fault during `Powered_Descent`:* The Director commands the Guidance module to recalculate burn time or shift the landing target to a closer safe zone.
    *   *Fault during `Terminal_Descent` (< 50m):* The Director triggers a "Hard Abort" contingency—ignoring precision targeting and commanding the Control Allocator to maximize vertical thrust on surviving engines regardless of lateral drift.

### B. State Estimation Module (Navigation)
KSP provides perfect data (`vessel.flight().surface_altitude`), which we will purposefully obscure.
*   **Noise Wrapper:** kRPC telemetry streams will be wrapped in a function that injects continuous Gaussian noise into the radar altimeter and accelerometer readings.
*   **The Filter:** A linear Discrete-Time Kalman Filter that fuses noisy acceleration data with noisy altitude data to produce a clean, probabilistic estimation of the true state vector $[X, Y, Z, V_x, V_y, V_z]$. (Note: Vessel mass is treated as a clean, external telemetry parameter, not estimated).
*   **Attitude Handling:** To keep the filter fast and linear, we use a small-angle approximation: attitude telemetry is treated as perfect when rotating body-frame acceleration to the world frame, and the accelerometer noise variance is artificially inflated to absorb the physical attitude uncertainty.
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
