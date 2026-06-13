# DECISIONS.md — Architecture Decision Record Log
> One entry per non-obvious design decision. Written at the time the decision is made.
> The goal: when Claude flags something in a review, this file answers "yes, we know — here's why."
> Entries are never deleted. Superseded decisions are marked as such and linked to their replacement.

---

## How to Write an Entry

Copy the template block below. Fill every field — especially **Options Considered** and **Consequences**.
A decision with no considered alternatives isn't a decision, it's an assumption.

**Statuses:**
- `ACCEPTED` — Active, in force.
- `PROPOSED` — Under discussion, not yet implemented.
- `SUPERSEDED` — Replaced by a later decision. Link to successor.
- `DEPRECATED` — Removed from the system. Record why.

---

## Template

```
### ADR-XXX — [Short imperative title, e.g. "Use Kalman Filter over moving average"]
- **Status:** PROPOSED | ACCEPTED | SUPERSEDED by ADR-XXX | DEPRECATED
- **Date:** YYYY-MM-DD
- **Author:** [Human / Other Agent / Joint]
- **Module(s):** [State Estimator / FDI / Control Allocator / Mission Director / Cross-cutting]

**Context**
What problem or question forced this decision? What constraints were in play?

**Options Considered**
1. Option A — brief description and key tradeoff
2. Option B — brief description and key tradeoff
3. Option C — brief description and key tradeoff (if applicable)

**Decision**
Which option was chosen and the core reason in 1–2 sentences.

**Consequences**
- ✅ What this makes easier or better.
- ✅ What this makes easier or better.
- ⚠️ What this makes harder, introduces risk, or defers to later.
- ⚠️ What this makes harder, introduces risk, or defers to later.

**Review Notes**
Any concerns flagged during code review that are accepted as known tradeoffs.
Leave blank until a review touches this decision.
```

---
---

## Decision Log

---

### ADR-001 — Use kRPC + Python over native kOS for all guidance logic
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Human & Claude
- **Module(s):** Cross-cutting

**Context**
The mission requires Kalman filtering, pseudo-inverse matrix solving, and a complex hierarchical state machine. KerboScript (kOS) has no linear algebra libraries and its VM is unsuitable for the mathematical weight of this system.

**Options Considered**
1. Native kOS — Zero external dependencies, runs inside KSP. No matrix libraries; implementing numpy-equivalent math from scratch is impractical and unmaintainable.
2. kRPC + Python — Streams telemetry over TCP/IP to a local Python process. Full access to numpy, filterpy, scipy. Adds a network dependency.
3. kRPC + C# — Compiled, type-safe, fast. No prototyping ecosystem for Kalman filters; overkill for a research project.

**Decision**
kRPC + Python. The math ecosystem (numpy, filterpy) is the decisive factor. Localhost TCP latency is well within KSP's 20ms physics tick.

**Consequences**
- ✅ Trivial implementation of Kalman filter, pseudo-inverse solver, and state machine.
- ⚠️ Python is interpreted; runtime TypeErrors are fatal during engine-out events. Mitigated by strict type-hinting and mypy enforcement.
- ⚠️ Adds dependency on kRPC mod and a running Python process. System cannot operate standalone inside KSP.

**Review Notes**
None yet.

---

### ADR-002 — Enforce strict type-hinting and mypy across all modules
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Claude
- **Module(s):** Cross-cutting

**Context**
Python's dynamic typing is a liability for a safety-critical state machine. A silent type mismatch in the Control Allocator during an engine-out event produces wrong thrust commands with no exception raised.

**Options Considered**
1. No type enforcement — fastest to write, highest runtime risk.
2. Type hints only (no checker) — documents intent but catches nothing automatically.
3. Type hints + mypy in CI — catches mismatches statically before any code runs.

**Decision**
Type hints + mypy. All public functions must be fully annotated. `np.ndarray` shapes must be documented in docstrings. mypy runs as a pre-commit gate.

**Consequences**
- ✅ Type errors caught at development time, not during a live descent burn.
- ✅ Annotations serve as machine-readable interface contracts between modules.
- ⚠️ numpy typing with mypy is notoriously verbose. Some annotations will feel bureaucratic.
- ⚠️ Slows initial development velocity slightly.

**Review Notes**
None yet.

---

### ADR-003 — Decouple system into four strictly bounded modules
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Claude
- **Module(s):** Cross-cutting

**Context**
Fault detection, state estimation, guidance, and mission logic interact heavily. Without explicit boundaries, changes in one domain silently break another.

**Options Considered**
1. Monolithic script — Simple to prototype, impossible to test in isolation or reason about during a fault.
2. Loosely coupled modules — Shared global state, ad-hoc interfaces. Fast but brittle.
3. Strictly decoupled modules with explicit interface contracts — Each module exposes a defined API; no direct cross-module data access.

**Decision**
Strict decoupling into: Mission Director, State Estimator, FDI, Control Allocator. Each module owns its domain. Data flows through defined interfaces only.

**Consequences**
- ✅ Each module can be tested and mocked independently.
- ✅ A fault in one module cannot silently corrupt another's state.
- ✅ Enables the review workflow — PRs touch one module at a time.
- ⚠️ More boilerplate at module boundaries.
- ⚠️ Interface design must be right upfront; changing it later touches multiple modules.

**Review Notes**
None yet.

---

### ADR-004 — FDI uses expected vs measured acceleration delta, not thrust sensor
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Claude
- **Module(s):** FDI

**Context**
KSP does not expose a reliable per-engine thrust sensor. The FDI must detect engine failure without direct thrust telemetry.

**Options Considered**
1. Per-engine thrust sensor polling — Ideal, but not reliably available via kRPC for all engine types.
2. Expected vs measured acceleration delta — FDI computes predicted acceleration from commanded throttle + known mass, compares to State Estimator's measured acceleration. Deviation beyond noise floor flags a fault.
3. Angular rate anomaly detection — Detects the torque induced by asymmetric thrust. Slower to react; cannot isolate which engine failed.

**Decision**
Expected vs measured acceleration delta, using the State Estimator's clean output as the measured signal. Noise floor threshold must be calibrated against the Kalman filter's output variance.

**Consequences**
- ✅ Works with any engine type without direct sensor access.
- ✅ Leverages State Estimator output — already noise-filtered.
- ⚠️ Threshold calibration is critical. Too tight → false positives. Too loose → late detection. Must be validated empirically per vessel configuration.
- ⚠️ Cannot detect partial thrust loss below the noise floor. A very gradual degradation may go undetected until large.

**Review Notes**
None yet.

---

### ADR-005 — Control Allocator uses pseudo-inverse wrench mapping
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Joint
- **Module(s):** Control Allocator

**Context**
When an off-center engine fails, naively throttling up surviving engines induces torque that spins the vessel. A simple priority scheme cannot simultaneously satisfy force and torque constraints. Additionally, the vessel's main engines are gimbaled, meaning they can vector their thrust.

**Options Considered**
1. Priority throttle — Throttle up the engine nearest the failed one. Fast, simple. Cannot zero the induced torque; will spin the vessel.
2. Manual torque cancellation rules — Hardcoded logic per engine layout. Brittle; breaks if the vessel configuration changes.
3. Pseudo-inverse wrench mapping — Guidance commands a 6-DOF wrench. Allocator maps it to 3D thrust force vector for each engine.

**Decision**
Pseudo-inverse wrench mapping treating each engine's control input as a 3D thrust force vector. We solve for $\mathbf{u}$ using `numpy.linalg.pinv` and then map the 3D force vector back to physical throttle and gimbal angles.

**Consequences**
- ✅ Correctly handles any engine failure pattern without hardcoded rules.
- ✅ Automatically satisfies both force and torque constraints.
- ✅ Allows standard pseudo-inverse algorithm without non-linear optimization solvers.
- ⚠️ If too many engines are lost, B becomes rank-deficient. The pseudo-inverse will still return a solution but it may be physically unrealisable. Must detect and report this condition.

**Review Notes**
None yet.

---

### ADR-006 — Arch WSL Execution Environment with `uv`
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Agent
- **Module(s):** Cross-cutting

**Context**
To run the code, we need a consistent development and execution environment.

**Options Considered**
1. Windows Native Python — Potential issues with some C-based dependencies or path lengths.
2. Arch WSL with pip — Standard, but slower environment setup.
3. Arch WSL with `uv` — Blazingly fast, modern package manager that handles venvs reliably.

**Decision**
Execute all python code, tests, and static analysis inside the Arch WSL distribution using `uv`.

**Consequences**
- ✅ Fast, standardized Linux environment.
- ✅ Reliable dependency resolution.
- ⚠️ Requires users to run WSL.

**Review Notes**
None yet.

---

### ADR-007 — Discrete-Time Kalman Filter for State Estimation
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Agent
- **Module(s):** State Estimator

**Context**
KSP provides perfect state information. To simulate a real aerospace environment, we must deliberately obscure this telemetry and reconstruct the state from noisy inputs.

**Options Considered**
1. Moving Average Filter — Too laggy and cannot handle variable noise well.
2. Particle Filter — Computationally expensive.
3. Discrete-Time Kalman Filter — Optimal estimator under linear systems and Gaussian noise.

**Decision**
Wrap kRPC streams in a noise-generating function and use a Discrete-Time Kalman Filter to reconstruct the state vector $[X, Y, Z, V_x, V_y, V_z]$.

**Consequences**
- ✅ High fidelity tracking with no lag.
- ✅ Computationally efficient.
- ⚠️ Requires accurate tuning of Q and R covariance matrices.

**Review Notes**
None yet.

---

### ADR-008 — Target-Relative Local Cartesian Tangent Plane
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Agent
- **Module(s):** Mission Director, Control Allocator

**Context**
Guidance calculations and trajectory tracking are mathematically complex in planetary spherical or body-centric Cartesian systems.

**Options Considered**
1. Planetary Spherical — Complex math for local flat paths.
2. Body-centric Cartesian — Target coordinates change as planet rotates.
3. Target-Relative Local Cartesian Tangent Plane — Target is origin, fixed to surface.

**Decision**
Define the landing target on the surface as the origin $(0, 0, 0)$. Position and velocity vectors are represented in a local Cartesian tangent plane.

**Consequences**
- ✅ Simplifies state estimator and control equations.
- ⚠️ Plane assumption breaks down at high altitudes or large ground distances.

**Review Notes**
None yet.

---

### ADR-009 — Local Gravity Vector Querying
- **Status:** ACCEPTED
- **Date:** 2026-06-13
- **Author:** Agent
- **Module(s):** State Estimator

**Context**
Gravity changes as the vessel descends. The State Estimator needs to know the gravity vector to propagate acceleration correctly.

**Options Considered**
1. Constant gravity assumption — Inaccurate over large altitude changes.
2. Local gravity model implementation — Complex and redundant.
3. Query exact local gravity dynamically from kRPC API.

**Decision**
Query the exact local gravity vector dynamically from the kRPC API.

**Consequences**
- ✅ High accuracy without modeling complex celestial body physics.
- ⚠️ Adds dependency on telemetry for physical constants.

**Review Notes**
None yet.
