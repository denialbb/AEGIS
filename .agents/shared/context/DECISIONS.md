# DECISIONS.md â€” Architecture Decision Record Log
> One entry per non-obvious design decision. Written at the time the decision is made.
> The goal: when Claude flags something in a review, this file answers "yes, we know â€” here's why."
> Entries are never deleted. Superseded decisions are marked as such and linked to their replacement.

---

## How to Write an Entry

Copy the template block below. Fill every field â€” especially **Options Considered** and **Consequences**.
A decision with no considered alternatives isn't a decision, it's an assumption.

**Statuses:**
- `ACCEPTED` â€” Active, in force.
- `PROPOSED` â€” Under discussion, not yet implemented.
- `SUPERSEDED` â€” Replaced by a later decision. Link to successor.
- `DEPRECATED` â€” Removed from the system. Record why.

---

## Template

```
### ADR-XXX â€” [Short imperative title, e.g. "Use Kalman Filter over moving average"]
- **Status:** PROPOSED | ACCEPTED | SUPERSEDED by ADR-XXX | DEPRECATED
- **Date:** YYYY-MM-DD
- **Author:** [Human / Other Agent / Joint]
- **Module(s):** [State Estimator / FDI / Control Allocator / Mission Director / Cross-cutting]

**Context**
What problem or question forced this decision? What constraints were in play?

**Options Considered**
1. Option A â€” brief description and key tradeoff
2. Option B â€” brief description and key tradeoff
3. Option C â€” brief description and key tradeoff (if applicable)

**Decision**
Which option was chosen and the core reason in 1â€“2 sentences.

**Consequences**
- âœ… What this makes easier or better.
- âœ… What this makes easier or better.
- âš ï¸ What this makes harder, introduces risk, or defers to later.
- âš ï¸ What this makes harder, introduces risk, or defers to later.

**Review Notes**
Any concerns flagged during code review that are accepted as known tradeoffs.
Leave blank until a review touches this decision.
```

---
---

## Decision Log

---

### ADR-001 â€” Use kRPC + Python over native kOS for all guidance logic
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Human & Claude
- **Module(s):** Cross-cutting

**Context**
The mission requires Kalman filtering, pseudo-inverse matrix solving, and a complex hierarchical state machine. KerboScript (kOS) has no linear algebra libraries and its VM is unsuitable for the mathematical weight of this system.

**Options Considered**
1. Native kOS â€” Zero external dependencies, runs inside KSP. No matrix libraries; implementing numpy-equivalent math from scratch is impractical and unmaintainable.
2. kRPC + Python â€” Streams telemetry over TCP/IP to a local Python process. Full access to numpy, filterpy, scipy. Adds a network dependency.
3. kRPC + C# â€” Compiled, type-safe, fast. No prototyping ecosystem for Kalman filters; overkill for a research project.

**Decision**
kRPC + Python. The math ecosystem (numpy, filterpy) is the decisive factor. Localhost TCP latency is well within KSP's 20ms physics tick.

**Consequences**
- âœ… Trivial implementation of Kalman filter, pseudo-inverse solver, and state machine.
- âš ï¸ Python is interpreted; runtime TypeErrors are fatal during engine-out events. Mitigated by strict type-hinting and mypy enforcement.
- âš ï¸ Adds dependency on kRPC mod and a running Python process. System cannot operate standalone inside KSP.

**Review Notes**
None yet.

---

### ADR-002 â€” Enforce strict type-hinting and mypy across all modules
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Claude
- **Module(s):** Cross-cutting

**Context**
Python's dynamic typing is a liability for a safety-critical state machine. A silent type mismatch in the Control Allocator during an engine-out event produces wrong thrust commands with no exception raised.

**Options Considered**
1. No type enforcement â€” fastest to write, highest runtime risk.
2. Type hints only (no checker) â€” documents intent but catches nothing automatically.
3. Type hints + mypy in CI â€” catches mismatches statically before any code runs.

**Decision**
Type hints + mypy. All public functions must be fully annotated. `np.ndarray` shapes must be documented in docstrings. mypy runs as a pre-commit gate.

**Consequences**
- âœ… Type errors caught at development time, not during a live descent burn.
- âœ… Annotations serve as machine-readable interface contracts between modules.
- âš ï¸ numpy typing with mypy is notoriously verbose. Some annotations will feel bureaucratic.
- âš ï¸ Slows initial development velocity slightly.

**Review Notes**
None yet.

---

### ADR-003 â€” Decouple system into four strictly bounded modules
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Claude
- **Module(s):** Cross-cutting

**Context**
Fault detection, state estimation, guidance, and mission logic interact heavily. Without explicit boundaries, changes in one domain silently break another.

**Options Considered**
1. Monolithic script â€” Simple to prototype, impossible to test in isolation or reason about during a fault.
2. Loosely coupled modules â€” Shared global state, ad-hoc interfaces. Fast but brittle.
3. Strictly decoupled modules with explicit interface contracts â€” Each module exposes a defined API; no direct cross-module data access.

**Decision**
Strict decoupling into: Mission Director, State Estimator, FDI, Control Allocator. Each module owns its domain. Data flows through defined interfaces only.

**Consequences**
- âœ… Each module can be tested and mocked independently.
- âœ… A fault in one module cannot silently corrupt another's state.
- âœ… Enables the review workflow â€” PRs touch one module at a time.
- âš ï¸ More boilerplate at module boundaries.
- âš ï¸ Interface design must be right upfront; changing it later touches multiple modules.

**Review Notes**
None yet.

---

### ADR-004 â€” FDI uses expected vs measured acceleration delta, not thrust sensor
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Claude
- **Module(s):** FDI

**Context**
KSP does not expose a reliable per-engine thrust sensor. The FDI must detect engine failure without direct thrust telemetry.

**Options Considered**
1. Per-engine thrust sensor polling â€” Ideal, but not reliably available via kRPC for all engine types.
2. Expected vs measured acceleration delta â€” FDI computes predicted acceleration from commanded throttle + known mass, compares to State Estimator's measured acceleration. Deviation beyond noise floor flags a fault.
3. Angular rate anomaly detection â€” Detects the torque induced by asymmetric thrust. Slower to react; cannot isolate which engine failed.

**Decision**
Expected vs measured acceleration delta, using the State Estimator's clean output as the measured signal. Noise floor threshold must be calibrated against the Kalman filter's output variance.

**Consequences**
- âœ… Works with any engine type without direct sensor access.
- âœ… Leverages State Estimator output â€” already noise-filtered.
- âš ï¸ Threshold calibration is critical. Too tight â†’ false positives. Too loose â†’ late detection. Must be validated empirically per vessel configuration.
- âš ï¸ Cannot detect partial thrust loss below the noise floor. A very gradual degradation may go undetected until large.

**Review Notes**
None yet.

---

### ADR-005 â€” Control Allocator uses pseudo-inverse wrench mapping
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Joint
- **Module(s):** Control Allocator

**Context**
When an off-center engine fails, naively throttling up surviving engines induces torque that spins the vessel. A simple priority scheme cannot simultaneously satisfy force and torque constraints. Additionally, the vessel's main engines are gimbaled, meaning they can vector their thrust.

**Options Considered**
1. Priority throttle â€” Throttle up the engine nearest the failed one. Fast, simple. Cannot zero the induced torque; will spin the vessel.
2. Manual torque cancellation rules â€” Hardcoded logic per engine layout. Brittle; breaks if the vessel configuration changes.
3. Pseudo-inverse wrench mapping â€” Guidance commands a 6-DOF wrench. Allocator maps it to 3D thrust force vector for each engine.

**Decision**
Pseudo-inverse wrench mapping treating each engine's control input as a 3D thrust force vector. We solve for $\mathbf{u}$ using `numpy.linalg.pinv` and then map the 3D force vector back to physical throttle and gimbal angles.

**Consequences**
- âœ… Correctly handles any engine failure pattern without hardcoded rules.
- âœ… Automatically satisfies both force and torque constraints.
- âœ… Allows standard pseudo-inverse algorithm without non-linear optimization solvers.
- âš ï¸ If too many engines are lost, B becomes rank-deficient. The pseudo-inverse will still return a solution but it may be physically unrealisable. Must detect and report this condition.

**Review Notes**
None yet.

---

### ADR-006 â€” Arch WSL Execution Environment with `uv`
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Agent
- **Module(s):** Cross-cutting

**Context**
To run the code, we need a consistent development and execution environment.

**Options Considered**
1. Windows Native Python â€” Potential issues with some C-based dependencies or path lengths.
2. Arch WSL with pip â€” Standard, but slower environment setup.
3. Arch WSL with `uv` â€” Blazingly fast, modern package manager that handles venvs reliably.

**Decision**
Execute all python code, tests, and static analysis inside the Arch WSL distribution using `uv`.

**Consequences**
- âœ… Fast, standardized Linux environment.
- âœ… Reliable dependency resolution.
- âš ï¸ Requires users to run WSL.

**Review Notes**
None yet.

---

### ADR-007 â€” Discrete-Time Kalman Filter for State Estimation
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Agent
- **Module(s):** State Estimator

**Context**
KSP provides perfect state information. To simulate a real aerospace environment, we must deliberately obscure this telemetry and reconstruct the state from noisy inputs.

**Options Considered**
1. Moving Average Filter â€” Too laggy and cannot handle variable noise well.
2. Particle Filter â€” Computationally expensive.
3. Discrete-Time Kalman Filter â€” Optimal estimator under linear systems and Gaussian noise.

**Decision**
Wrap kRPC streams in a noise-generating function and use a Discrete-Time Kalman Filter to reconstruct the state vector $[X, Y, Z, V_x, V_y, V_z]$.
*Option A* is used for accelerometer fusion: The accelerometer reading drives the prediction step as $u = noisy\_accel$, and the altimeter is the sole measurement in the update step ($z = noisy\_alt$).

**Consequences**
- âœ… High fidelity tracking with no lag.
- âœ… Computationally efficient. Keeps math linear (no EKF needed).
- âš ï¸ Requires accurate tuning of Q and R covariance matrices.

**Review Notes**
None yet.

---

### ADR-008 â€” Target-Relative Local Cartesian Tangent Plane
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Agent
- **Module(s):** Mission Director, Control Allocator

**Context**
Guidance calculations and trajectory tracking are mathematically complex in planetary spherical or body-centric Cartesian systems.

**Options Considered**
1. Planetary Spherical â€” Complex math for local flat paths.
2. Body-centric Cartesian â€” Target coordinates change as planet rotates.
3. Target-Relative Local Cartesian Tangent Plane â€” Target is origin, fixed to surface.

**Decision**
Define the landing target on the surface as the origin $(0, 0, 0)$. Position and velocity vectors are represented in a local Cartesian tangent plane.

**Consequences**
- âœ… Simplifies state estimator and control equations.
- âš ï¸ Plane assumption breaks down at high altitudes or large ground distances.

**Review Notes**
None yet.

---

### ADR-009 â€” Local Gravity Vector Querying
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Agent
- **Module(s):** State Estimator

**Context**
Gravity changes as the vessel descends. The State Estimator needs to know the gravity vector to propagate acceleration correctly.

**Options Considered**
1. Constant gravity assumption â€” Inaccurate over large altitude changes.
2. Local gravity model implementation â€” Complex and redundant.
3. Query exact local gravity dynamically from kRPC API.

**Decision**
Query the exact local gravity vector dynamically from the kRPC API.

**Consequences**
- âœ… High accuracy without modeling complex celestial body physics.
- âš ï¸ Adds dependency on telemetry for physical constants.

**Review Notes**
None yet.

---

### ADR-010 â€” Rank-Deficiency Condition Number Threshold (`1e4`)
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Human & Agent
- **Module(s):** Control Allocator

**Context**
When an engine fails, the control allocator matrix $B$ may become rank-deficient, meaning the pseudo-inverse solver returns mathematically valid but physically impossible throttle commands. We need a numerical threshold to flag this state.

**Options Considered**
1. Default `numpy.linalg.matrix_rank` tolerance â€” Relies on floating-point precision, disconnected from physical engine layouts.
2. Hardcoded value of `1e4` â€” Catches severe ill-conditioning and near-singular matrices before physically absurd commands are issued.

**Decision**
Set condition number threshold to `1e4`. If `cond(B) > 1e4`, `AllocationDegenerateError` is raised.

**Consequences**
- âœ… Prevents the vessel from blindly trusting broken allocator logic.
- âœ… Safely routes to HARD_ABORT in the Mission Director.
- âš ï¸ `1e4` may be slightly conservative or loose depending on final thrust-to-weight ratios; may need future empirical adjustment.

**Review Notes**
Resolves Claude's finding B2 regarding degenerate allocation logic.

---

### ADR-011 â€” Simulate wind disturbance via IMU acceleration injection
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Joint
- **Module(s):** FDI (Gremlin) / State Estimator

**Context**
We need to test the control allocator's ability to reject environmental disturbances like wind shear. Stock KSP does not natively simulate wind, and while Ferram Aerospace Research (FAR) does, kRPC lacks native API wrappers to explicitly read FAR's aerodynamic data.

**Options Considered**
1. Write a custom C# kRPC plugin to read FAR wind/stall data â€” High effort, ties the architecture specifically to FAR.
2. Inject a velocity drift bias â€” Kinematically tests the controller, but does not accurately represent how wind physically acts as an external force (drag).
3. Inject a lateral acceleration bias into the IMU telemetry stream â€” Simulates unmodeled aerodynamic drag natively within the State Estimator's existing physics model.

**Decision**
Simulate wind by intercepting the `noisy_accel` telemetry stream in the Gremlin module and injecting a lateral acceleration bias before passing it to the State Estimator. The Kalman filter will naturally integrate this drift, and the control allocator will attempt to reject it, agnostic of whether the disturbance comes from FAR or the simulated Gremlin.

**Consequences**
- âœ… Validates disturbance rejection capabilities organically without requiring KSP mods.
- âœ… Keeps the guidance architecture completely agnostic to the underlying aerodynamic model (Stock vs. FAR).
- âš ï¸ Cannot test wind-induced aerodynamic stall natively (unless FAR is actually installed and the physical craft stalls).

**Review Notes**
None yet.

---

### ADR-012 â€” Two-Tier Test Harness (Unit Tests + Kinematic Mock)
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Claude (Reviewer)
- **Module(s):** Cross-cutting (Testbed)

**Context**
To iterate quickly without running full Kerbal Space Program (KSP) scenarios, we need an offline simulation/test harness (ISS-007). A proposal was made to build a full 6-DOF physics engine using `scipy.integrate.solve_ivp` to synthesize perfect KSP flight data and inject noise.

**Options Considered**
1. Full 6-DOF Analytical Physics Engine (scipy) â€” Synthesizes perfect trajectories and re-injects noise. Massive overkill, introduces a "second-system trap" that requires KSP to validate itself before validating AEGIS. Mismatched noise statistics would corrupt Kalman filter and FDI calibration.
2. Two-Tier Test Harness (Unit tests + Kinematic mock) â€” Tier 1: Pure `pytest` unit tests for module-level correctness. Tier 2: A simple 3-line Newtonian kinematic mock (`a = thrust_sum / mass - g + noise; v += a * dt; x += v * dt`) for integration loop testing.

**Decision**
Two-Tier Test Harness. We explicitly reject building a full physics engine. We will rely on pure unit tests with synthetic inputs for mathematical verification, and a lightweight kinematic mock for loop integration testing. The mock must use the exact same noise wrapper as the live system.

**Consequences**
- âœ… Prevents maintaining a complex second system (the physics engine).
- âœ… Ensures test noise statistics precisely match production noise statistics.
- âœ… Massively reduces the time required to build the test harness.
- âš ï¸ The offline kinematic mock will not capture KSP-specific transient dynamics (like gimbal slew rates or variable gravity), meaning final parameter tuning must still occur against live KSP runs.

**Review Notes**
Addresses the architectural feedback from Claude's review of the `proposal/6dof-scipy-physics-harness` proposal.

---

### ADR-013 â€” Dual-File Telemetry Logging Architecture
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Claude & Joint
- **Module(s):** Cross-cutting (Telemetry)

**Context**
Debugging live KSP runs is difficult because the console output is too fast and interactive debugging breaks the 50Hz control loop constraint. We need a telemetry logging system that can serve as the primary post-mortem debugging surface without blocking the control loop.

**Options Considered**
1. Single CSV Log â€” High resolution, but extremely difficult to parse for specific events (e.g., fault moments) since it generates ~180,000 rows per hour.
2. Dual-File Strategy (CSV + JSONL) â€” A dense `telemetry.csv` captures every physics tick for deep analysis, while `events.jsonl` captures only discrete state changes or faults.
3. Module-Level Logging â€” Each module writes its own logs. Violates ADR-003 by coupling pure math modules to I/O logic.

**Decision**
Dual-File Strategy. The `MissionDirector` is the sole owner of the `TelemetryWriter` and passes a fully assembled `TelemetryFrame` to it every tick. File I/O is heavily buffered (1MB) to prevent loop blocking. Logs are written to a timestamped folder with a `logs/latest` symlink for easy predictable access.

**Consequences**
- âœ… Preserves the 20ms physics tick budget by preventing synchronous file writes.
- âœ… Creates a highly readable "what happened" timeline via the events log.
- âœ… Keeps core modules pure and decoupled from I/O.
- âš ï¸ The `MissionDirector` has to assemble data from all modules, increasing its orchestration burden.

**Review Notes**
None yet.

---

### ADR-014 â€” Fold Small-Angle Attitude Noise into Accelerometer Noise Budget
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Joint
- **Module(s):** State Estimator, Cross-cutting

**Context**
The State Estimator requires the vessel's attitude to rotate body-frame acceleration from the IMU into the world frame. Real sensors have noise, so attitude telemetry should theoretically be noisy. However, applying noise to the attitude creates a non-linear rotation matrix. The linear Discrete-Time Kalman Filter (chosen in ADR-007) cannot handle non-linear transformations, which would strictly require upgrading to an Extended Kalman Filter (EKF).

**Options Considered**
1. Perfect Attitude (No Noise) â€” Unrealistic, breaks project simulation philosophy.
2. Upgrade to Extended Kalman Filter (EKF) â€” Accurate, but introduces significant mathematical complexity (Jacobians) and testing burden to manage a 13-state filter.
3. Small-Angle Approximation â€” Assume attitude error is small during controlled flight. Use the un-noised attitude to rotate the acceleration into the world frame, and artificially inflate the accelerometer noise parameter ($\sigma_{accel}$) to absorb the mathematical error introduced by the small rotation error.

**Decision**
Option 3: Small-Angle Approximation. We will standardize the noise wrapper to output **body-frame acceleration**. The Mission Director or Estimator will rotate this vector into the world frame using the raw, un-noised attitude telemetry, and the Kalman Filter will use an inflated accelerometer noise variance to account for the attitude uncertainty.

**Consequences**
- âœ… Maintains the simplicity and speed of the linear Discrete-Time Kalman Filter (ADR-007).
- âœ… Drastically reduces development and testing time compared to an EKF.
- âš ï¸ Assumes attitude error remains small ($\le 2-5^\circ$). If the vessel tumbles, the approximation breaks down (though if this occurs during powered descent, the mission is likely already lost).
- âš ï¸ The reference frame for telemetry is explicitly defined: `noisy_accel` is body-frame.

**Review Notes**
Resolves the architecture gap exposed by Claude regarding attitude noise in the linear Kalman filter.

---

### ADR-015 â€” WSL2 to KSP Connection Topology
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Human & Agent
- **Module(s):** Cross-cutting

**Context**
AEGIS runs in an Arch WSL2 environment, which operates inside a Hyper-V VM with its own virtual network adapter. KSP and the kRPC server run on the Windows host. Connecting to `localhost` (127.0.0.1) from within WSL2 reaches the VM's loopback interface, not the Windows host, causing kRPC connection timeouts unless Windows 11 `localhostforwarding` is explicitly enabled (which is not guaranteed).

**Options Considered**
1. Rely on `.wslconfig` `localhostforwarding=true` â€” Simple in code, but requires manual Windows configuration and relies on Windows 11 features that can be brittle across versions.
2. Dynamically resolve Windows Host IP â€” The Python process reads the host IP from `/etc/resolv.conf` (the `nameserver` entry) and connects to that IP. Works universally across WSL2 setups without manual `.wslconfig` changes.

**Decision**
Option 2: Dynamically resolve Windows Host IP. The kRPC connection setup code must dynamically determine the host IP rather than hardcoding `localhost`.

**Consequences**
- âœ… Guarantees connection reliability out-of-the-box for any WSL2 user.
- âœ… Prevents silent failure modes where the agent hangs on `krpc.connect()`.
- âš ï¸ Adds slight complexity to the initial connection bootstrapping logic.

**Review Notes**
None yet.

---

### ADR-016 â€” Engine Discovery via kOS/kRPC Tags
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Joint
- **Module(s):** Mission Director

**Context**
To allocate thrust, the Control Allocator needs to know which engines it is allowed to control. Simply querying all engines or active stages risks accidentally controlling separation motors or RCS thrusters. 

**Decision**
We will strictly use kRPC's `vessel.parts.with_tag("AegisEngine")` to discover controllable engines. 

**Consequences**
- âœ… Guarantees safety against hijacking unrelated solid rocket motors or RCS thrusters.
- âš ï¸ **Future RCS Support:** Currently, this explicitly ignores RCS thrusters. When future updates add RCS support to the Control Allocator, we will need to query `vessel.parts.with_tag("AegisRCS")` from `vessel.parts.rcs` and include them in the allocation matrix.

**Review Notes**
Requested specifically by the user.

---

### ADR-017 â€” Custom Landing Pad Reference Frame
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Agent
- **Module(s):** Cross-cutting (Math)

**Context**
Using a generic planetary surface frame places the origin $(0,0,0)$ at the center of the celestial body. This results in huge orbital state vectors ($600,000$ meters) which degrade floating-point precision inside the Kalman Filter and needlessly complicate guidance targeting.

**Decision**
Create a custom, planet-fixed reference frame centered exactly on the target landing pad's latitude and longitude, using KSP's `ReferenceFrame.create_relative`.

**Consequences**
- âœ… Drastically simplifies math. The target position is always exactly $(0,0,0)$.
- âœ… Improves Kalman Filter numerical stability by bounding position values.
- âš ï¸ Requires knowing the exact landing coordinates at startup. We will hardcode default coordinates (e.g., KSC Launchpad) for now.

**Review Notes**
None yet.

---

### ADR-018 â€” Proportional-Derivative (PD) Translation Control
- **Status:** ACCEPTED
- **Date:** 2026-06-14
- **Author:** Human & Agent
- **Module(s):** Guidance

**Context**
The Guidance module needs to compute a desired 3D force vector to track the desired trajectory during powered descent.

**Options Considered**
1. Proportional-Derivative (PD) Controller â€” Tracks position and velocity errors.
2. Apollo E-Guidance â€” Time-optimal but assumes constant gravity and no atmosphere.
3. Model Predictive Control (MPC) â€” Perfect constraint handling but computationally heavy.

**Decision**
Option 1: Proportional-Derivative (PD) Controller. Simple, robust, and computationally cheap. We retain the possibility of implementing Apollo E-Guidance in the future for atmospheric-free landings.

**Consequences**
- âœ… Easy to tune and guarantee execution within the 20ms physics tick budget.
- âœ… Highly robust to unexpected disturbances like engine-out torque.
- âš ï¸ Not strictly fuel-optimal compared to E-Guidance or MPC.

**Review Notes**
None yet.

---

### ADR-021 â€” Guidance and FDI Decoupling from skip_predict
- **Status:** ACCEPTED
- **Date:** 2026-06-14
- **Author:** Human & Agent
- **Module(s):** Mission Director, FDI

**Context**
The `skip_predict` flag was originally used to gate both the Kalman filter predict step AND the guidance controller. This caused a cascading failure during dt spikes: guidance suppression led to zero thrust, which made FDI misinterpret gravity as engine failures, triggering HARD_ABORT (ISS-010).

**Options Considered**
1. Gate guidance on skip_predict â€” Simple but dangerous. Leads to uncontrolled free-fall during lags.
2. Decouple guidance from skip_predict â€” Guidance always runs during powered descent. FDI detects faults only when predict is active. This requires telemetry to log skip_predict state for debugging.
3. Implement hover degraded mode â€” Command thrust to counter gravity during dt spikes. More complex, deferred.

**Decision**
Option 2: Decouple guidance from skip_predict. Guidance always runs during powered descent phases. FDI fault detection is skipped during dt spikes, holding last known good expected_accel. skip_predict is logged to telemetry for visibility.

**Consequences**
- âœ… Prevents catastrophic HARD_ABORT during dt spikes by ensuring continuous thrust command.
- âœ… Maintains FDI's ability to detect real failures when system is stable.
- âœ… Provides visibility into degraded states via telemetry.
- âš ï¸ FDI cannot detect faults during dt spikes (by design) â€” this is a known tradeoff.
- âš ï¸ Velocity estimate may become stale during long dt spikes (deferred to ISS-011).

**Review Notes**
Resolves ISS-010 and POSTMORTEM_2026-06-14_035508.md. Implements all HIGH priority fixes from postmortem.

---

### ADR-019 â€” Quaternion-based PD Attitude Control
- **Status:** ACCEPTED
- **Date:** 2026-06-14
- **Author:** Human & Agent
- **Module(s):** Guidance

**Context**
The Guidance module needs to compute desired torques ($\tau_x, \tau_y, \tau_z$) to track the desired orientation and feed the 6-DOF wrench to the Control Allocator.

**Options Considered**
1. Quaternion-based PD controller â€” Avoids gimbal lock, standard spacecraft control.
2. Euler-angle PID controller â€” Easy to conceptualize, susceptible to gimbal lock near 90 degrees pitch.
3. Offload to KSP SAS â€” Allocator couldn't use main engines to counter torque.

**Decision**
Option 1: Quaternion-based PD controller. Ensures stability in all orientations without gimbal lock.

**Consequences**
- âœ… Mathematically robust across the entire attitude sphere.
- âœ… Fully integrated with the Control Allocator's wrench inputs.
- âš ï¸ Quaternions are less intuitive to debug via raw numerical logs.

**Review Notes**
None yet.

---

### ADR-020 â€” Stateless Target Generation (Mission Director Driven)
- **Status:** ACCEPTED
- **Date:** 2026-06-14
- **Author:** Human & Agent
- **Module(s):** Mission Director, Guidance

**Context**
Trajectory tracking requires a moving target state (e.g. glide slope). This logic could live in the Guidance module or the Mission Director.

**Options Considered**
1. Mission Director passes instantaneous `target_state` on every tick.
2. Guidance module holds an internal state machine to track the descent profile.

**Decision**
Option 1: Mission Director passes instantaneous `target_state` directly to Guidance on every tick (`guidance.compute_wrench(current_state, target_state)`).

**Consequences**
- âœ… Strictly decouples logic. Mission Director owns "what to do", Guidance owns "how to do it".
- âœ… Guidance module remains purely a stateless mathematical function, simplifying unit testing.
- âš ï¸ Mission Director's `run_loop` becomes slightly more complex as it must interpolate the target position.

**Review Notes**
None yet.

---

### ADR-022 â€” Dynamic Glide-Slope Target Generation
- **Status:** ACCEPTED
- **Date:** 2026-06-14
- **Author:** Human & Agent
- **Module(s):** Mission Director, Guidance

**Context**
During powered descent, static waypoint targets set far below the current altitude caused massive position-error terms in the PD controller. This resulted in the Control Allocator receiving commands for downward thrust, which it correctly rejected by cutting engines to 0. The vessel would free-fall until position error decreased enough to demand upward thrust, often too late.

**Options Considered**
1. Static waypoints â€” Leads to actuator saturation and free-fall.
2. Glide-slope target generation â€” The vertical position target is set to the current altitude (zeroing vertical position error), while the velocity target is set to a descent rate proportional to altitude.
3. Path integral control â€” Too complex for the current setup.

**Decision**
Option 2: Glide-slope target generation. `main.py` explicitly zeroes vertical position error by setting target altitude to current altitude, and commands an altitude-proportional descent rate (`GLIDESLOPE_K_ALT * alt_above_floor`).

**Consequences**
- âœ… Actuator saturation is eliminated; thrust remains continuously bounded.
- âœ… Produces a smooth deceleration profile.
- âš ï¸ The controller no longer attempts to "catch up" to a specific altitude if it falls behind, strictly following the velocity profile.
- âš ï¸ Requires empirical tuning of `GLIDESLOPE_K_ALT` against actual TWR.

**Review Notes**
Resolves ISS-011.

---

### ADR-023 â€” Explicit Throttle Setting with Independent Throttle
- **Status:** ACCEPTED
- **Date:** 2026-06-14
- **Author:** Agent
- **Module(s):** Control Allocator, Mission Director

**Context**
To modulate asymmetric thrust across multiple engines, we enable `independent_throttle = True` on each engine so it can be controlled independently of the main vessel throttle via its `thrust_limit`. However, when uncoupled from the main vessel throttle, the engine's internal throttle value defaults to 0.0. Modulating `thrust_limit` while `throttle = 0.0` yields zero actual thrust output, causing silent failures (e.g., vessel dropping like a rock despite "100%" thrust_limit commands).

**Options Considered**
1. Modulate `engine.throttle` directly instead of `thrust_limit` â€” Would require overhauling the existing architecture, and KSP sometimes clamps `engine.throttle` based on other UI interactions.
2. Explicitly force `engine.throttle = 1.0` and modulate `thrust_limit` â€” Leaves the primary actuator mechanism (`thrust_limit`) intact while explicitly satisfying the API requirement for independent throttle.

**Decision**
Option 2: Whenever `independent_throttle` is enabled, explicitly force `engine.throttle = 1.0` in the same execution cycle. All fine-grained control continues to be routed through `engine.thrust_limit`.

**Consequences**
- âœ… Prevents silent 0.0 N thrust failures.
- âœ… Requires no changes to the math inside the Control Allocator (which already outputs 0 to 1 thrust limits).
- âš ï¸ Adds one extra RPC call per engine during activation/configuration.

**Review Notes**
None yet.

---

### ADR-024 â€” Engine Gimbal Trim Mod Integration for Asymmetric Control
- **Status:** ACCEPTED
- **Date:** 2026-06-14
- **Author:** Agent
- **Module(s):** Control Allocator, Mission Director

**Context**
By default, KSP's stock kRPC interface only allows global flight commands (pitch, yaw, roll) that KSP internally mixes and applies to all engines. It does not provide an API to control the gimbal angles of individual engines independently. To fully utilize 6-DOF control allocation (which requires differential gimbal deflection to generate torque/forces for yaw, pitch, and roll without relying solely on RCS or reaction wheels), we need a way to actuate each engine's gimbal deflection individually.

**Options Considered**
1. **Rely on stock reaction wheels and RCS** for attitude torque, and only use engines for differential throttling.
   - *Pros:* Simple, requires no external mods.
   - *Cons:* Reaction wheels lack authority for larger vessels, RCS fuel is limited, and this doesn't leverage the engines' active gimbal capability.
2. **Use the Gimbal Trim mod (`ModuleGimbalTrim`)** to command individual gimbals.
   - *Pros:* Exposes independent `Gimbal X` and `Gimbal Y` fields per engine. Allows the Control Allocator to specify precise, independent gimbal deflection angles (in radians, mapped/clipped to \(\pm 5^\circ\) limit) for each engine part.
   - *Cons:* Requires the Gimbal Trim mod installed in KSP; adds RPC overhead during the control loop.

**Decision**
Option 2: We use the Gimbal Trim mod (`ModuleGimbalTrim`).
- When initializing/configuring the engines in the control loop, if `ModuleGimbalTrim` is present on a part, we trigger its `"Toggle Trim"` event to enable manual control override.
- During the control loop, we query the Control Allocator for optimal gimbal angles (radians), convert them to degrees, clip them to the physical limits (\(\pm 5^\circ\)), and write them directly to the `"Gimbal X"` and `"Gimbal Y"` float fields of the `ModuleGimbalTrim` module on each active engine.

**Consequences**
- âœ… Active 6-DOF allocation is achieved using both differential throttling and differential gimbaling.
- âœ… Substantially improves attitude authority, especially under asymmetric engine failure states.
- âš ï¸ Adds dependency on the Gimbal Trim KSP mod.
- âš ï¸ Increases the number of RPC calls per engine per control tick (two field writes per engine).

**Review Notes**
None yet.


### ADR-025 — Automated Configuration Tuning Framework
- **Status:** ACCEPTED
- **Date:** 2026-06-14
- **Author:** Agent
- **Module(s):** scripts/tune_config.py

**Context**
The AEGIS flight software relies on carefully tuned configuration parameters (e.g., PD gains for position and attitude). Tuning these manually takes too long. We need an automated way to repeatedly run test flights under varying configurations and evaluate their performance (landing accuracy, fuel consumed, impact velocity) to eliminate rocking and overshoot.

**Decision**
We use a grid-search tuning script (`scripts/tune_config.py`) that utilizes the kRPC `space_center.load()` API. The script iterates over parameter permutations, loads a standardized starting state (e.g., `aegis_tune_start`), injects the parameters dynamically, triggers the Mission Director, and logs the landing metrics to a CSV.

**Consequences**
- ☑️ Automated performance characterization.
- ☑️ Enables finding the optimal attitude damping to prevent side-to-side rocking.
- ⚠️ Requires creating and maintaining a standardized "aegis_tune_start" save file.
- ⚠️ Overwrites Python module state in-memory during testing.

**Review Notes**
Superseded by ADR-026 (Optuna).

---

### ADR-026 - Optuna Hyperparameter Tuning
- **Status:** ACCEPTED
- **Date:** 2026-06-14
- **Author:** Agent
- **Module(s):** scripts/tune_config_optuna.py

**Context**
Testing all 17 configuration parameters using a combinatorial grid search is mathematically infeasible. We need an advanced method to explore the parameter space efficiently to find the absolute best setup.

**Decision**
We implemented an Optuna-based hyperparameter tuning framework (TPE algorithm). It evaluates each set of parameters by running the AEGIS flight simulation (via kRPC save loading) and computes a fitness score based on landing distance and fuel consumption. Severe penalties are applied if the vessel crashes.

**Consequences**
- ☑️ Can optimize all 17 parameters efficiently over hundreds of runs.
- ☑️ Learns which parameters cause crashes and focuses on promising areas.
- ⚠️ Requires the external `optuna` package and takes hours to converge.

**Review Notes**
None yet.

---

### ADR-027 — ESO/ADRC Ownership and Guidance/Estimator Boundary
- **Status:** ACCEPTED
- **Date:** 2026-06-14
- **Author:** Claude (Chief Code Reviewer)
- **Module(s):** Guidance, State Estimator

**Context**
The NN-ADRC plan proposed placing the Extended State Observer (ESO) in the State Estimator. The design advisory (NN_ADRC_DESIGN_ADVISORY.md §3.1) identified this as a cross-domain coupling that reopens ADR-003/ADR-014.

**Decision**
ESO lives in Guidance (`src/guidance/adrc.py`), fed by existing KF output. State Estimator remains scoped to ADR-007/014. Option 2 — confirmed by Phase 2 implementation.

**Consequences**
- ☑️ State Estimator remains scoped to ADR-007/014.
- ☑️ Phase 2 implementation confirmed Option 2 is sound.
- ⚠️ ESO tuning (β, δ) depends on ISS-001 (FDI threshold) and ISS-003 (Q/R tuning) closing first.

**Review Notes**
Recommended by NN_ADRC_DESIGN_ADVISORY.md §3.1. Phase 2 (`adrc.py`) implementation confirmed.

---

### ADR-028 — Vessel Inertia Tensor Sourcing
- **Status:** ACCEPTED
- **Date:** 2026-06-14
- **Author:** Agent
- **Module(s):** Guidance, Mission Director, Cross-cutting

**Context**
The quaternion control law upgrade (Phase 1 of the NN-ADRC roadmap) requires a 3×3 inertia tensor `J` for:
- `J·(Kp·e + Kd·ė)` inertia-scaled PD torque
- `Ω(ω)Jω` gyroscopic cross-coupling feedforward
- NN-ADRC wrench scaling (Phase 4, deferred)

AEGIS currently has no module that queries, owns, or passes around `J`.

**Options Considered**
1. Query `vessel.moment_of_inertia` (3-tuple diagonal) — Simple, but only provides principal moments; off-diagonal components are assumed zero.
2. Query `vessel.inertia_tensor` (full 3×3 matrix) — Full inertia tensor; captures off-diagonal coupling.
3. Hardcode a fixed inertia value — Brittle; changes with fuel burn and vessel config.

**Decision**
Option 2: Query `vessel.inertia_tensor` once at startup, reshape to 3×3 numpy array. Owned by Mission Director, passed to GuidanceController at construction. Same clean-telemetry pattern as `mass` (ISS-006 caveat applies).

**Consequences**
- ☑️ Unblocks quaternion feedforward (Phase 1) and NN-ADRC wrench scaling (Phase 4).
- ☑️ Same clean-telemetry dependency as `mass` (ISS-006) — documented.
- ⚠️ Inertia changes with fuel burn; queried once at startup currently. Re-polling deferred.
- ⚠️ New parameter in `controller.py` interface; must be passed from `main.py`.

**Review Notes**
Recommended by NN_ADRC_DESIGN_ADVISORY.md §3.3.

---

### ADR-029 — KF State Vector Scope for NN-ADRC Inputs (Option A vs 9-state vs EKF)
- **Status:** DEFERRED
- **Date:** 2026-06-14
- **Author:** Claude (Chief Code Reviewer)
- **Module(s):** State Estimator

**Context**
The NN-ADRC plan mentions "EKF" and implies acceleration may become part of the state vector. Advisory §3.2 identifies ambiguity: current 6-state linear KF (ADR-007 Option A) uses accelerometer as control input.

**Options Considered**
1. Option A — Keep current 6-state linear KF. Source filtered acceleration from ESO's z2 or existing `kinematic_accel_world`.
2. Option B — Extend to 9-state constant-acceleration KF (still linear).
3. Option C — Fold attitude into state; accept EKF + Jacobians. Reopens ADR-014.

**Decision**
Deferred until Phase 2. Option A is sufficient for Phase 1.

**Consequences**
- ⚠️ Decision needed before ESO/NN implementation (Phase 2+).
- ☑️ Does not block Phase 1.

**Review Notes**
Recommended by NN_ADRC_DESIGN_ADVISORY.md §3.2.
