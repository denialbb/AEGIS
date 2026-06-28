# AEGIS Feature Ideas & Development Directions

> **Status:** Living document — generated 27 June 2026. Integrate into roadmap as priorities shift.

---

## 1. NN-ADRC Roadmap Completion (Phases 2–5)

The existing roadmap is well-defined but only Phase 1 is complete. The highest-value next step is finishing the NN-ADRC integration.

| Phase | Deliverable | Key Challenge | Value |
|-------|-------------|---------------|-------|
| **Phase 2** | Transient Profile Generator + WSEF + NN scaffolding | Tuning β/b₀/δ per axis without live flight data | High — replaces naive PD with model-rejecting ADRC |
| **Phase 3** | Allocator integration | Minimal — allocator already handles arbitrary configs | Medium — wiring only |
| **Phase 4** | FDI rewrites to monitor ESO z₃ and NN Δr̈ | Distinguishing NN compensation from actual faults | High — enables *graceful* degradation instead of HARD_ABORT |
| **Phase 5** | "Gremlin" auto-training script | Random engine kills must be reproducible & safe | Very High — closes the learning loop |

### Concrete Next Steps
1. Build the **Transient Profile Generator** (TG) with a 5th-order polynomial filter that takes raw target-state jumps and produces dynamically feasible `(position, velocity, acceleration)` trajectories bounded by configurable jerk/snap limits. This prevents the ADRC from seeing step inputs.
2. Implement **WSEF** (`u₀ = k₁(v₁ − z₁) + k₂(v₂ − z₂)`, `u = (u₀ − z₃)/b₀`) in `src/guidance/controller.py` alongside the existing PD path, gated by a config flag.
3. Wire the NN output into the wrench pipeline: `wrench = ctm_wrench + J @ nn_output`. Ensure clamp fallback (ISS-013) logs when it fires so training data can be refined.
4. Rewrite FDI to raise events on `|z₃| > Z3_FAULT_THRESHOLD` (discrete actuator failure) and `|Δr̈| > NN_FAULT_THRESHOLD` persistent over 100 ticks (permanent engine loss). Add dt-spike guards identical to ISS-010.
5. Build the **Gremlin** script: launch KSP from a known save, randomly kill 1–3 engines at random altitudes, log `(pos, vel, accel, Δr̈)` pairs, and auto-reload the save. Run 1000+ trials overnight.

---

## 2. Real-Time Telemetry Visualization & Replay

AEGIS currently logs CSV+JSONL but has no live dashboard. A web-based real-time telemetry dashboard would dramatically accelerate debugging and tuning.

| Feature | Stack Suggestion | Complexity | Value |
|---------|-----------------|------------|-------|
| **Live 3D Trajectory Plot** | Plotly / Three.js via WebSocket | Medium | Visualize descent path, attitude drift, and pad approach in real time |
| **Attitude Quaternion Animation** | Three.js cylinder model driven by telemetry CSV | Medium | Replay any flight with scrub bar — indispensable for post-flight ANalysis |
| **Sensor Noise Dashboard** | Streamlit or FastAPI + React | Low | Show raw vs filtered_altitude, innovation norms, EKF covariance ellipsoids |
| **FDI Event Timeline** | D3.js timeline with states & faults | Low | Correlate fault detection time with actual engine kill time |
| **Allocator Force/Torque Visualization** | Matplotlib 3D arrows per engine | Medium | Debug why allocation failed or saturated |

### Concrete Implementation Path
- Add an optional `--web-dashboard` flag to `main.py` that starts a small FastAPI server on `localhost:8080`.
- Stream JSON packets of the current `TelemetryFrame` at 10 Hz via WebSocket.
- Frontend: single-page React app with Plotly for 2D plots and Three.js for 3D vessel attitude.
- Replay mode: drag-and-drop a `telemetry.csv` file, frontend parses and drives the plots.
- *MVP scope:* 2D altitude/velocity/time plot + 3D attitude sphere + event log. Skip fancy styling.

---

## 3. Telemetry Replay & Flight Sim (Digital Twin)

Turn recorded telemetry into a **replayable simulation** of the flight for algorithm validation without launching KSP.

| Component | Description |
|-----------|-------------|
| **Telemetry Replay Engine** | Load `telemetry.csv`, reconstruct state vector at each tick, and feed it into guidance/FDI modules in isolation. Assert that allocator decisions match recorded throttles. |
| **Deterministic Flight Sim** | A simple Python physics integrator (Euler/RK4) that takes recorded control outputs and initial conditions, propagates a rigid body, and compares the resulting trajectory to the recorded one. Discrepancies reveal unmodeled dynamics (aero, part flexibility, kRPC lag). |
| **Regression Testing** | Add `pytest` tests that run the replay engine against the last 10 recorded flights and assert no crash, no HARD_ABORT on nominal flights, and FDI latency < 1 s. |

### Value Proposition
- Frees developers from needing KSP running for every algorithm iteration.
- Enables CI/CD for guidance changes: replay 50 recorded flights, assert pass rate > 95%.
- The simple physics sim becomes the foundation for an eventual **Hardware-in-the-Loop** bridge to real actuators.

---

## 4. Predictive Hazard Avoidance & Landing Site Selection

Currently AEGIS targets a single pre-defined pad. A richer mission profile would evaluate landing sites dynamically based on terrain, fuel, and fault state.

| Sub-Feature | Description |
|-------------|-------------|
| **Terrain-Aware Landing** | Query KSP's `body.heightmap` or use raycasting to find flat, obstacle-free zones within a glide radius. Score by flatness, proximity, and fuel cost. |
| **Multi-Pad Targeting** | Instead of `TARGET_LAT/LON`, maintain a ranked list of candidate pads. Replan on fault: if the closest pad is no longerreachable with 2 engines out, target the next-best. |
| **Fuel-Optimal Diversion** | Replace the hard switch to `HARD_ABORT` with a fuel-optimal divert: compute reachable set with remaining Δv, pick safe site, fly there.

### Technical Approach
- Use KSP's `CelestialBody.heightmap` at descent-start to generate a coarse cost map (~1000×1000 m grid).
- Flatness = variance of height in a 10 m radius. Obstacles = local maxima above threshold.
- Reachable set = back-propagate from current state with available thrust and gravity. Use a simple shooting method.
- This is computationally cheap enough for Python and adds a level of real-world realism (like Blue Origin's pad selection or SpaceX's drone-ship targeting).

---

## 5. Multi-Vessel Coordination (Fleet Landing)

Extend AEGIS from single-vessel to **coordinated multi-vessel descent**.

| Feature | Description |
|---------|-------------|
| **Fleet Formation Control** | 2–4 vessels descend in close formation, maintaining relative position via inter-vessel telemetry. Each vessel runs its own estimator/FDI but shares fault status with the fleet. |
| **Collision Avoidance** | Minimum separation constraint (e.g., 50 m) enforced by the guidance controller: if a neighbor is closer than threshold, lateral target shifts away. |
| **Distributed Allocation** | If one vessel loses 2 engines, neighbors can dynamically assume part of its deceleration burden by adjusting their own glideslope timing (requires shared timing). |
| **Shared Landing Pad** | Multiple vessels land on the same pad in sequence, with a scheduler that staggers terminal descent by 30 s to avoid plume interaction. |

### Why This Is Impressive
- KSP with kRPC supports multiple connections; each vessel can be controlled independently.
- Demonstrates that AEGIS's modular architecture (decoupled Mission Director, Estimator, FDI, Allocator) scales beyond single-vessel monolithic scripts.
- Directly analogous to real-world Mars sample return, drone swarms, or lunar base construction with multiple landers.

---

## 6. Alternative Estimation & Sensor Fusion

The 12-state Error-State EKF + Mahony filter is solid, but several upgrades could improve robustness and accuracy.

| Approach | Description | Effort | Value |
|----------|-------------|--------|-------|
| **Unscented Kalman Filter (UKF)** | Replaces the EKF linearization with sigma-point transformation. Handles the strong nonlinearity of attitude dynamics better, especially during aggressive retrograde burns. | Medium-High | Higher accuracy during transient maneuvers; better bias estimation |
| **Particle Filter for FDI** | Use a bank of particle filters, one per hypothesized engine failure mode. The prior weight of each bank reflects the probability that a given engine set is dead. | High | More graceful handling of simultaneous/slow-degradation failures than brute-force isolation |
| **Barometer & GPS Fusion** | Add simulated barometric altimeter and GPS fix to the sensor suite. Treat them as low-rate, low-accuracy aiding sensors for the EKF. | Low | Increases realism; tests fusion under heterogeneous sensor noise |
| **Visual Odometry (kRPC screenshot → velocity)** | Capture KSP screenshots via kRPC's camera API, run optical flow or monocular depth estimation to estimate ground-relative velocity. | Very High | Demonstrates cross-domain ML (CV + GNC) integration |

---

## 7. Robustness & Edge-Case Hardening

Several open issues reveal gaps that, if closed, would make AEGIS dramatically more reliable.

| Issue | Root Cause | Proposed Fix |
|-------|------------|--------------|
| **ISS-001: FDI threshold uncalibrated** | Threshold is hardcoded `3.0` with no relation to actual noise floor | Auto-calibrate: during warm-up, record `|expected - measured|` distribution, set threshold at `mean + 5σ` |
| **ISS-003: Estimator attitude handling** | Mahony diverges when SAS rotates during warm-up, zeroing bias estimate | Fix: disable SAS during warm-up; or pre-compute gyro bias during a previous coast phase and persist it |
| **ISS-004: Multiple simultaneous failures → HARD_ABORT** | No graceful degradation path for 2+ engine failures | Implement Phase 4 NN-ADRC FDI rewrite; if still uncontrollable, execute fuel-optimal divert to nearest safe site instead of HARD_ABORT |
| **ISS-006: FDI mass from clean kRPC** | Mass is read once at start; ignores fuel burn and staging | Continuously update mass from kRPC or estimate fuel flow from throttle and Isp |
| **DT spike handling** | Current code skips predict but still runs guidance → can command unsafe throttles | Add a "dead-reckoning" mode: on dt spike, hold last safe wrench for up to 100 ms, then resume. Log all spikes for post-analysis. |

---

## 8. Distributed Training & AutoML Integration

The Gremlin script (Phase 5) produces training data, but the training itself can be accelerated.

| Feature | Description |
|---------|-------------|
| **Parallel Gremlin** | Use `multiprocessing` or `ray` to run 4+ KSP instances (one per CPU core) in parallel, each with a different random seed. Aggregate datasets. |
| **Hyperparameter Search for ADRC** | Extend existing Optuna tuning (`scripts/tune_config_optuna.py`) to search ADRC-specific parameters: `β₁, β₂, β₃`, `α₁, α₂`, `δ`, `b₀`. Define a composite score: landing accuracy × fuel efficiency × fault-survival rate. |
| **NN Architecture Search** | Use Optuna to search NN hidden-layer sizes, activation functions, and learning-rate schedules. The training metric is the L2 norm of `nn_output − disturbance_true`. |
| **Dataset Versioning** | Hash the training dataset (SHA-256 of concatenated CSVs). Tag trained NN weights with the dataset hash. Reject model deployment if dataset hash mismatches current codebase. |
| **Online Learning** | After offline training, allow the NN to fine-tune itself in real time using ESO `z₃` as a pseudo-label for disturbance. Use very small learning rate (<< 1e-5) to avoid instability. |

---

## 9. Real-World Aerospace Bridge

AEGIS's algorithms are domain-agnostic. Bridge to real-world flight software.

| Step | Description | Concrete Action |
|------|-------------|-----------------|
| **PX4 / ArduPilot Plugin** | Port the core estimator + guidance to a PX4 module or ArduPilot library. Target a VTOL or multicopter first (lower complexity). | Create `src/aero_bridge/px4_module.cpp` with C++ EKF and Python bindings for the NN. |
| **Hardware-in-the-Loop (HIL)** | Replace kRPC with MAVLink to a Pixhawk running in HIL mode (e.g., `jMAVSim`). Feed sensor noise models from AEGIS into the simulator. | Add `--hil-mode` flag to `main.py`; use `pymavlink` to send actuator commands and receive sensor data. |
| **Gazebo / Ignition Sim** | Port the physics sim to Gazebo, using a URDF of a multi-engine VTOL. Validate NN-ADRC on a high-fidelity rigid-body simulator. | Write Gazebo plugin that exposes the same API as kRPC (position, velocity, attitude, engine throttles). |
| **Flight Test** | Real-world validation on a small multi-engine drone (e.g., hexacopter with 2+ redundant motors). Inject motor kill via RC override. | Partner with a university aerospace lab. AEGIS's Python stack runs on a companion computer (e.g., Raspberry Pi + Navio2). |

---

## 10. Developer Experience & Tooling

| Tool | Description |
|------|-------------|
| **Interactive Parameter Tuning Dashboard** | Streamlit page that loads a `telemetry.csv`, lets the user drag sliders for `GUIDANCE_KP_POS_LATERAL`, etc., and replots the trajectory in real time. Saves tuned params to a new config file. |
| **Automated Flight Report Generator** | After each flight, automatically generate a PDF report (matplotlib + jinja2) with: trajectory plot, attitude error plot, FDI timeline, allocator saturation heatmap, fuel consumption, and landing score. |
| **GitHub Actions CI** | On every PR: run `mypy`, `pytest`, and the telemetry replay engine against a pinned set of recorded flights. Fail if any regression. |
| **KSP Save-State Manager** | A small CLI tool to snapshot/load KSP save files for deterministic test starts. Integrate with the Gremlin script. |

---

## Summary: Recommended Priority Order

1. **Phase 2–3 NN-ADRC core** — highest algorithmic impact; closes the largest gap in the current controller.
2. **Telemetry replay engine + regression tests** — unlocks rapid iteration without KSP running.
3. **Live web dashboard** — dramatically improves debugging speed and is visually impressive.
4. **Auto-calibrate FDI threshold + fix ISS-003 (attitude)** — closes two reliability gaps that currently cause HARD_ABORT on nominal flights.
5. **Multi-vessel coordination** — turns AEGIS from a single-lander script into a fleet-management system; highest "wow factor" for demos.
6. **Real-world bridge (PX4/HIL)** — longest-term, but positions AEGIS as a genuine aerospace research platform rather than a KSP mod.

