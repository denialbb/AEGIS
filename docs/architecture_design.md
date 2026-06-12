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

The architecture is strictly decoupled into four primary domains to ensure robust systems engineering.

### A. The Mission Director (Hierarchical State Machine)
The overarching logic controller. It manages nominal mission phases and handles contingency branching based on the severity and timing of a detected fault.

*   **States:** `Deorbit_Burn`, `Hypersonic_Coast`, `Powered_Descent`, `Hover_Targeting`, `Terminal_Descent`.
*   **Contingency Logic Example:** 
    *   *Fault during `Powered_Descent`:* The Director commands the Guidance module to recalculate burn time or shift the landing target to a closer safe zone.
    *   *Fault during `Terminal_Descent` (< 50m):* The Director triggers a "Hard Abort" contingency—ignoring precision targeting and commanding the Control Allocator to maximize vertical thrust on surviving engines regardless of lateral drift.

### B. State Estimation Module (Navigation)
KSP provides perfect data (`vessel.flight().surface_altitude`), which we will purposefully obscure.
*   **Noise Wrapper:** kRPC telemetry streams will be wrapped in a function that injects continuous Gaussian noise into the radar altimeter and accelerometer readings.
*   **The Filter:** A Discrete-Time Kalman Filter (or Extended Kalman Filter) that fuses the noisy acceleration data with the noisy altitude data to produce a clean, probabilistic estimation of the true state vector $[X, Y, Z, V_x, V_y, V_z, Mass]$.

### C. Fault Detection & Isolation Module (FDI)
The system's diagnostic nervous system.
*   **Logic:** The FDI continuously calculates the *expected* acceleration vector based on the currently commanded throttle and known vessel mass. It compares this against the *measured* acceleration provided by the State Estimator.
*   **Isolation:** If the deviation exceeds the noise tolerance threshold, the FDI isolates which specific actuator vector is failing to produce thrust, flagging the engine as "Dead" and alerting the Mission Director.

### D. Control Allocation Module (Guidance & Control)
The core engineering solution to asymmetric thrust.
*   **The Problem:** If an off-center engine dies, simply throttling up the remaining engines will induce a catastrophic torque, spinning the vessel out of control.
*   **The Solution:** The guidance algorithm does not command individual engines. Instead, it commands a desired 6-DOF "Wrench" (Forces $F_x, F_y, F_z$ and Torques $\tau_x, \tau_y, \tau_z$).
*   **The Mapper:** The Allocator uses a pseudo-inverse matrix solver (`numpy.linalg.pinv`) to map the desired Wrench to the *surviving* engines. It will automatically throttle down engines opposite the failure to kill the torque, while throttling up adjacent engines to maintain the required vertical stopping force.

---

## 4. Testbed Requirements

To successfully develop and test this system, a specific testbed vessel must be constructed in KSP:

1.  **Engine Cluster:** The vessel requires a redundant cluster of independently controllable engines (e.g., a central sustainer engine surrounded by 4 or 8 radial engines).
2.  **RCS Authority:** Robust RCS or heavy reaction wheels to provide baseline attitude control during the split-second it takes the Allocator to re-balance the main engines.
3.  **The Gremlin:** We will write a lightweight background script (either in kOS or a separate Python thread) that acts as the "Gremlin"—randomly selecting an engine part module and forcing its `thrustLimit` to 0 or shutting it down entirely to trigger the FDI module during live tests.
