# OPEN_ISSUES.md — Known Issues & Technical Debt

> The honest list. Every known problem, deferred decision, and accepted risk lives here.
> An issue here is not a failure — it's controlled awareness.
> An issue NOT here that shows up in a review is a problem.

---

## Severities

- 🔴 `CRITICAL` — Will cause incorrect behavior or system failure if unresolved. Must be fixed before that module is considered stable.
- 🟡 `MAJOR` — Significant risk or limitation. Must be resolved before the full system is integrated.
- 🔵 `MINOR` — Low risk. Quality, robustness, or maintainability concern. Fix when bandwidth allows.
- ⚪ `DEFERRED` — Consciously postponed. Not a risk at current scope. Revisit trigger noted.

## Statuses

- `OPEN` — Not yet worked.
- `IN PROGRESS` — Actively being addressed. Assigned to someone.
- `RESOLVED` — Fixed. Linked to the resolving commit or review.
- `WONT FIX` — Accepted as a permanent known limitation. Rationale required.

---

## How to Add an Issue

Copy the template. Be specific — "it might break" is not an issue description.
Link to the relevant ADR if the issue stems from a known architectural tradeoff.
When resolved, do not delete the entry — update the status and add the resolution note.

---

## Template

```
### ISS-XXX — [Short title]
- **Severity:** CRITICAL | MAJOR | MINOR | DEFERRED
- **Status:** OPEN | IN PROGRESS (assigned to: ?) | RESOLVED | WONT FIX
- **Date opened:** YYYY-MM-DD
- **Module(s):** [Module name]
- **Related ADR:** ADR-XXX (or "None")
- **Related Review:** REVIEW_[timestamp] (or "None")

**Description**
Clear, specific description of the problem. What breaks, when, under what conditions.

**Acceptance Criteria**
What does "fixed" look like? What test or condition proves this is resolved?

**Resolution**
<!-- Fill in when resolved -->
```

---

---

## Issue Log

---

### ISS-001 — FDI noise threshold not yet empirically calibrated

- **Severity:** 🟡 MAJOR
- **Status:** IN PROGRESS
- **Date opened:** 2026-06-13
- **Module(s):** FDI
- **Related ADR:** ADR-004
- **Related Review:** None

**Description**
The FDI compares expected vs measured acceleration to detect engine failure. The deviation threshold that triggers a fault flag must be calibrated against actual Kalman filter output variance for this vessel. Too tight a threshold produces false positives during normal burn transients. Too loose a threshold means a dead engine goes undetected until the vessel is already spinning.

**Acceptance Criteria**

- Threshold derived from measured State Estimator output noise across at least 3 nominal descent runs.
- FDI correctly flags a simulated engine failure within 2 physics ticks under nominal noise conditions.
- FDI produces zero false positives across 5 consecutive nominal descents.

**Resolution**

The hardcoded placeholder (9999.0) was replaced with `config.FDI_THRESHOLD = 3.0` (commit 88d8853). The default 3.0 value is within the recommended [1.5, 5.0] range from `config.py`. Empirical calibration against KF output variance is still pending — downgraded from CRITICAL to MAJOR since the hardcoded-bypass bug is fixed.

---

### ISS-002 — Control Allocator behavior undefined when effectiveness matrix B is rank-deficient

- **Severity:** 🔴 CRITICAL
- **Status:** OPEN
- **Date opened:** 2026-06-13
- **Module(s):** Control Allocator
- **Related ADR:** ADR-005
- **Related Review:** None

**Description**
`numpy.linalg.pinv` always returns a result even when B is rank-deficient (too many engines lost for the requested wrench to be achievable). In this case the pseudo-inverse solution is mathematically valid but physically unrealisable — the Allocator will silently output throttle commands that cannot produce the commanded wrench. The Mission Director receives no signal that the solution is degenerate, and the vessel will behave unpredictably.

**Acceptance Criteria**

- Allocator computes and logs the condition number of B on every solve.
- If condition number exceeds a defined threshold, Allocator raises a structured `AllocationDegenerateError` rather than returning a silent bad solution.
- Mission Director handles `AllocationDegenerateError` as a contingency branch (Hard Abort or target shift).

**Resolution**

<!-- Fill in when resolved -->

---

### ISS-003 — Kalman filter Q and R matrix tuning not empirically validated

- **Severity:** 🔴 CRITICAL
- **Status:** IN PROGRESS
- **Date opened:** 2026-06-13
- **Module(s):** State Estimator
- **Related ADR:** ADR-007 (superseded by ADR-030)
- **Related Review:** None

**Description**
The process noise covariance (Q) and measurement noise covariance (R) matrices are critical inputs to the 12-state Error-State EKF (ADR-030). These values must match the actual noise characteristics of the injected Gaussian noise and the vessel's true dynamics. Incorrectly tuned Q/R causes the filter to either trust noisy measurements too much or ignore them and drift.

Current state: Q and R are no longer identity matrices — they are constructed from configured sigma values (SIGMA_ALT, SIGMA_VEL, PROCESS_NOISE_THRUST_COEF, etc.) that have been partially tuned via Optuna. However, the tuning has not been validated against actual flight recordings to confirm filter RMS errors are acceptable.

**Acceptance Criteria**

- R tuned to match the variance of the injected noise wrapper for each sensor (altimeter, accelerometer, velocimeter).
- Q tuned empirically — filter output should track true state without lag during a nominal descent.
- Filter performance documented: RMS error of estimated altitude vs true altitude across 3 test runs.

**Resolution**

<!-- Fill in when resolved -->

---

### ISS-004 — No handling for simultaneous multi-engine failure

- **Severity:** 🟡 MAJOR
- **Status:** OPEN
- **Date opened:** 2026-06-13
- **Module(s):** FDI, Mission Director
- **Related ADR:** ADR-004
- **Related Review:** None

**Description**
The FDI isolation logic currently assumes failures are sequential and singular. If two engines fail simultaneously (or in rapid succession within the same detection window), the FDI may attribute the full deviation to a single engine and misidentify the fault. The Mission Director's contingency branches also assume a single dead engine. The Gremlin script can produce this scenario.

**Acceptance Criteria**

- FDI correctly identifies and flags two simultaneous engine failures in a test where the Gremlin kills two engines at the same tick.
- Mission Director has defined behaviour for 2+ simultaneous failures (Hard Abort threshold, or reduced target set).
- Behaviour is documented in the contingency logic section of ARCHITECTURE.md.

**Resolution**

<!-- Fill in when resolved -->

---

### ISS-005 — Gremlin script timing is non-deterministic; no reproducible test seeds

- **Severity:** 🟡 MAJOR
- **Status:** OPEN
- **Date opened:** 2026-06-13
- **Module(s):** Testbed (Gremlin)
- **Related ADR:** None
- **Related Review:** None

**Description**
The Gremlin randomly selects which engine to kill and at what point in the descent. This is good for stress testing but makes failures non-reproducible. A bug observed in one run cannot be reliably reproduced in the next, which makes debugging FDI and Control Allocator interactions very difficult.

**Acceptance Criteria**

- Gremlin accepts an optional `--seed` argument that fixes the random selection.
- Gremlin logs to a structured file: which engine was killed, at what altitude, at what mission state.
- Seed and log are referenced in any test report attached to a review submission.

**Resolution**

<!-- Fill in when resolved -->

---

### ISS-006 — State vector does not include mass estimation; mass is assumed known

- **Severity:** 🟡 MAJOR
- **Status:** OPEN
- **Date opened:** 2026-06-13
- **Module(s):** State Estimator, FDI
- **Related ADR:** None
- **Related Review:** None

**Description**
The FDI's expected acceleration calculation divides commanded thrust by current vessel mass. Mass is currently read directly from kRPC (`vessel.mass`) rather than estimated. This is clean telemetry — not noisy — which is inconsistent with the project's philosophy of operating on estimated state only. More importantly, if mass telemetry is ever noised or rate-limited in future, the FDI expected acceleration becomes wrong and produces spurious fault flags.

**Acceptance Criteria**

- Evaluate whether mass should be added to the Kalman filter state vector [X, Y, Z, Vx, Vy, Vz, Mass].
- Decision recorded in a new ADR.
- If deferred, add explicit code comment in FDI flagging the clean-telemetry dependency.

**Resolution**

<!-- Fill in when resolved -->

---

### ISS-007 — No integration test harness; all testing is live KSP runs

- **Severity:** 🔵 MINOR
- **Status:** DEFERRED
- **Date opened:** 2026-06-13
- **Module(s):** Cross-cutting
- **Related ADR:** None
- **Related Review:** None

**Description**
Currently the only way to test the system is to run a full KSP descent scenario. Unit tests for individual modules (e.g. feeding synthetic telemetry into the State Estimator, or feeding a known wrench and engine layout into the Control Allocator) do not exist. This makes it slow to iterate on mathematical correctness and impossible to run in CI. Testing the Mission Director state machine requires simulating the KSP physics environment and kRPC telemetry streams.

**Acceptance Criteria**

- At minimum, the Control Allocator and FDI have pytest unit tests with synthetic inputs.
- Tests cover nominal case, single engine failure, and degenerate B matrix.
- Detailed mock structures in our tests directory to simulate telemetry streams, vessel states, and engine configurations.

**Revisit trigger:** Once all four modules have their first accepted implementation.

**Resolution**

<!-- Fill in when resolved -->

---

### ISS-008 — TCP latency and Bottlenecking

- **Severity:** 🟡 MAJOR
- **Status:** OPEN
- **Date opened:** 2026-06-13
- **Module(s):** Cross-cutting
- **Related ADR:** ADR-001
- **Related Review:** None

**Description**
Python over TCP socket communication with kRPC might suffer from jitter or packet delay, which could degrade control performance at 50Hz.

**Acceptance Criteria**

- Use kRPC stream callbacks or high-performance `add_stream` telemetry calls to prevent blocking on network reads.
- Impact analyzed under simulated packet delay.

**Resolution**

<!-- Fill in when resolved -->

---

### ISS-009 — Log File Rotation not implemented

- **Severity:** 🔵 MINOR
- **Status:** OPEN
- **Date opened:** 2026-06-14
- **Module(s):** Core Logging
- **Related ADR:** None
- **Related Review:** REVIEW_20260614_012742

**Description**
Currently, log files just append indefinitely. This can lead to excessive disk usage over time if not managed.

**Acceptance Criteria**

- Implement a `RotatingFileHandler` from python's `logging.handlers`.
- Set an appropriate max bytes and backup count limit for log rotation.

**Resolution**

<!-- Fill in when resolved -->

---

### ISS-010 — Guidance and FDI incorrectly gated on skip_predict causing HARD_ABORT during dt spikes

- **Severity:** 🔴 CRITICAL
- **Status:** IN PROGRESS
- **Date opened:** 2026-06-14
- **Module(s):** Mission Director, FDI, Telemetry
- **Related ADR:** None (see POSTMORTEM_2026-06-14_035508.md)
- **Related Review:** None

**Description**
When a dt spike occurs (e.g., game lag on activation), the `skip_predict` flag is set to True. This flag incorrectly gates both the Kalman filter predict step AND the guidance controller. With guidance suppressed, `desired_wrench = zeros`, causing zero thrust. The FDI then observes gravity in the accelerometer data (while expected_accel = 0) and misidentifies it as multiple engine failures, triggering HARD_ABORT via ISS-004.

**Acceptance Criteria**

- Guidance runs continuously during powered descent phases, regardless of `skip_predict` state.
- FDI fault detection is skipped during dt spikes to avoid spurious fault flags from stale expected_accel.
- `skip_predict` field added to telemetry for visibility into degraded state events.
- ARCHITECTURE.md updated to document correct dt spike handling behavior.
- Velocity fallback strategy documented as future enhancement (numerical differentiation of altitude when predict skipped).

**Resolution**

<!-- Fill in when resolved -->

### ISS-011 — Static far-field target setpoints saturate guidance into zero-thrust free-fall

- **Severity:** 🔴 CRITICAL
- **Status:** RESOLVED
- **Date opened:** 2026-06-14
- **Module(s):** Mission Director, Guidance
- **Related ADR:** ADR-020 (incomplete implementation), ADR-018
- **Related Review:** None (identified from live test telemetry/events log, 2026-06-14)

**Description**
During POWERED_DESCENT and HOVER_TARGETING, `main.py` assigns target_state[:3]
as a STATIC point far below the vehicle's current altitude (e.g.
`up_vector * 500.0` for POWERED_DESCENT, entered at apex ~1000m; `up_vector *
50.0` for HOVER_TARGETING, entered at ~500m). With GUIDANCE_KP_POS=1.0, a
position error of hundreds of meters produces a_cmd_world in the hundreds of
m/s^2, pointing DOWNWARD. ControlAllocator correctly identifies this as an
unrealizable thrust direction (engines can't push down) and zeroes throttle
(allocator.py dot_prod < 0 check). The vehicle then free-falls with
essentially zero thrust until the position error shrinks enough (near each
phase's static setpoint) for a_cmd to flip positive — by which point velocity
has built to near free-fall terminal values and available thrust cannot
arrest it before ground impact.

Live test evidence (2026-06-14 run): apex ~1010m at t=35.6s, throttle stayed
at 0.000 until alt=584m (t≈12.7s into descent), reached only ~0.88 by alt=7m
before the log ends (crash). Vehicle dropped 1003m in 14.19s — within 2% of
the 987m a pure free-fall (0.5*g*t^2) would predict, confirming thrust
contributed almost nothing for the bulk of the descent.

This is effectively an unimplemented part of ADR-020, which anticipated the
Mission Director "must interpolate the target position" — the current code
jumps straight to each phase's terminal setpoint instead.

A secondary defect compounds debugging this: `TelemetryFrame.velocity` is
hardcoded to `np.zeros(3)` (main.py ~line 330) instead of `state_vector[3:]`,
so the Kalman-estimated velocity is invisible in telemetry.

**Acceptance Criteria**

- target_state position/velocity for POWERED_DESCENT, HOVER_TARGETING, and
  TERMINAL_DESCENT is generated by an instantaneous glide-slope function
  (e.g. `_compute_glideslope_target`) rather than a static far-away setpoint,
  per the interpolation intent of ADR-020.
- Vertical position error (pos_err projected onto up_vector) must remain
  near zero at all times — i.e. the controller's vertical command is driven
  by velocity-profile tracking, not by closing a large altitude gap.
- A simulated/live descent starting from an apex of at least 1000m must
  reach the pad with |vz| within target tolerance at touchdown, without
  HARD_ABORT and without any sustained throttle=0 segment longer than the
  time needed for current velocity to converge to the local glide-slope
  target (a few seconds, not the whole descent).
- `TelemetryFrame.velocity` reflects `state_vector[3:]` so the estimator's
  velocity output is visible for verification.
- _Removed:_ GLIDESLOPE_K_ALT empirical calibration — the sqrt suicide-burn
  profile (`v_target = -sqrt(2 * a_avail * alt_above_floor)`) derives the
  target directly from the vessel's actual TWR each tick, eliminating the
  need for open-loop gain tuning against TWR.
- ARCHITECTURE.md / a new ADR documents the glide-slope target-generation
  algorithm (suggest ADR-022, following the ADR-021 precedent for ISS-010).

**Resolution**

Implemented `_compute_glideslope_target` to dynamically track velocity while zeroing vertical position error, avoiding actuator saturation.

Later upgraded to a suicide-burn sqrt profile: `v_target = -sqrt(2 * a_avail * alt_above_floor)`, where `a_avail` is derived from the vessel's actual TWR each tick. This eliminates the linear `k_alt * alt` profile that saturated at high altitude and required empirical tuning against TWR. The guidance's `a_cmd_world` is also clamped to `ACCEL_CLAMP_FACTOR × a_avail` to prevent attitude target flipping during saturating transients.

---

### ISS-012 — `fal()` δ=0 guard and per-axis `b0` derivation (NN-ADRC Risk)
- **Severity:** 🟡 MAJOR
- **Status:** RESOLVED
- **Date opened:** 2026-06-14
- **Module(s):** Guidance (ADRC)
- **Related ADR:** ADR-027
- **Related Review:** REVIEW_20260615_002100

**Description**
The `fal()` nonlinearity from the ESO equations has a `δ=0` edge case causing `ZeroDivisionError` for `δ**(1-α)` when `α<1`. Additionally, `b0` for attitude axes (≈1/I_axis) differs fundamentally from translation axes (≈1/mass). Per NN_ADRC_DESIGN_ADVISORY.md §4.

**Acceptance Criteria**
- `fal()` has an explicit `δ>0` guard with a clear error message. ✅ Implemented in `adrc.py` line 20-21.
- All six per-axis `b0` derivations documented with data source and clean-telemetry caveats. ⚠️ Deferred — b0 currently defaults to 1.0 per-axis.

**Resolution**
`fal()` δ>0 guard implemented (ValueError with clear message). Added `b0 != 0` guard in `PerAxisESO.__init__` to prevent silent inf/nan propagation. Per-axis `b0` derivation remains deferred; current defaults to 1.0.

---

### ISS-013 — NN output bounding and ADRC fallback mode (NN-ADRC Risk)
- **Severity:** 🟡 MAJOR
- **Status:** RESOLVED
- **Date opened:** 2026-06-14
- **Module(s):** Guidance (ADRC)
- **Related ADR:** ADR-027
- **Related Review:** REVIEW_20260615_002100

**Description**
An untrained-region NN producing NaN or out-of-range `Δr̈` is similar to a runtime TypeError during engine-out. ADR-002's safety philosophy requires clamping and fallback. Per NN_ADRC_DESIGN_ADVISORY.md §4.

**Acceptance Criteria**
- `Δr̈` clamped to a physically plausible range. ✅ Implemented in `nn.py` line 87 with `np.nan_to_num` pre-processing and `np.clip`.
- Documented fallback (pure ADRC, NN contribution = 0) when clamp triggers repeatedly. ✅ Implemented in `controller.py` with consecutive issue tracking and NN disable mechanism.

**Resolution**
Enhanced NN output bounding to handle NaN/inf values using `np.nan_to_num` before clipping. Implemented fallback mechanism that tracks consecutive NaN/clamping events and disables NN after threshold, switching to pure ADRC-CTM mode.

---

### ISS-014 — FDI/ADRC diagnostic interface and re-derived dt-spike guards
- **Severity:** 🟡 MAJOR
- **Status:** DEFERRED
- **Date opened:** 2026-06-14
- **Module(s):** FDI, Guidance
- **Related ADR:** ADR-027
- **Related Review:** None

**Description**
The FDI/ADRC interface must go through the Mission Director (ADR-013 pattern), not direct cross-module access. `z3`/`Δr̈` signals need their own dt-spike/zero-throttle guards (ISS-010 fix must be re-derived for these signals). Per NN_ADRC_DESIGN_ADVISORY.md §3.4.

**Acceptance Criteria**
- `AdrcDiagnostics` dataclass returned from `compute_wrench`, routed through main.py to FDI.
- ISS-010-style guards re-derived and validated for `z3`/`Δr̈` transient behavior.

**Resolution**
Deferred to future Phase 5 (FDI adaptation). Phase 5 is not yet implemented.

---

### ISS-015 — Quaternion convention verification (current_attitude format vs R.from_quat)
- **Severity:** 🔵 MINOR
- **Status:** RESOLVED
- **Date opened:** 2026-06-14
- **Module(s):** Guidance, Telemetry
- **Related ADR:** None
- **Related Review:** None

**Description**
`controller.py` docstring documented `current_attitude` as scalar-first `[w, x, y, z]`, but called `R.from_quat(current_attitude)` which expects scalar-last `[x, y, z, w]`. Per NN_ADRC_DESIGN_ADVISORY.md §5.

**Acceptance Criteria**
- A small unit test takes a known rotation (e.g., 90° about Z), round-trips through `R.from_quat`/`.as_quat()`, and asserts expected Euler angles. ✅ Implemented in `tests/test_quaternion.py`.
- `controller.py` docstring for `current_attitude` corrected to scalar-last convention. ✅ Fixed.
- The unit test passes on the actual `sensors.py` return format.

**Resolution**
Docstring fixed to scalar-last `[x, y, z, w]`. Quaternion unit test implemented and passing. The mismatch was a docstring error; code was always correct.

---

### ISS-017 — Thruster API crashes after `space_center.load()` (null WorldTransform)
- **Severity:** 🔴 CRITICAL
- **Status:** RESOLVED
- **Date opened:** 2026-06-17
- **Module(s):** Mission Director, Engine Configuration
- **Related ADR:** None
- **Related Review:** None

**Description**
After `conn.space_center.load("aegis_tune_start")`, all `Thruster` API methods that access `UnityEngine.Transform.get_worldPosition/worldRotation` crash with a null reference exception. This includes `initial_thrust_direction(ref_frame)`, `thrust_direction(ref_frame)`, `thrust_position(ref_frame)`, and `thrust_reference_frame` (when used with `transform_direction/position`). The transform never recovers — even 10s after load. This is a Unity-side issue where the gimbal stash/gameobject isn't re-initialised after scene reload.

Affected methods crash with:
- `UnityEngine.Transform.get_worldPosition` (null transform) for `initial_thrust_direction`
- `Object reference not set` for `thrust_direction` / `thrust_position`

**Acceptance Criteria**
- Engine discovery succeeds and produces correct `thrust_direction` even when the Thruster API is unavailable.
- `transform_direction(PART_THRUST_AXIS[part.name], part.reference_frame, vessel.reference_frame)` used as final fallback produces correct thrust direction in vessel frame.
- Fix verified by unit test running against KSP after `space_center.load()`.

**Resolution**
Added a two-level fallback in `main.py` engine discovery:
1. Try `thruster.initial_thrust_direction(vessel.reference_frame)` (original)
2. Catch → try `thruster.thrust_direction(vessel.reference_frame)` (original fallback)
3. Catch → use `transform_direction(axis, part.reference_frame, vessel.reference_frame)` with axis from `config.PART_THRUST_AXIS` dict

`config.PART_THRUST_AXIS` maps known part names to their thrust axis in part-local frame:
- `liquidEngineMini.v2` (48-7S "Spark"): `(0, 1, 0)` — thrust along part +Y
- `liquidEngine2.v2` (LV-T45 "Swivel"): `(0, 0, -1)`
- `liquidEngine3.v2` (LV-909 "Terrier"): `(0, 0, -1)`
- `liquidEngine` (LV-T30 "Reliant"): `(0, 0, -1)`
- `liquidEngineS2` (LV-T45 variant): `(0, 0, -1)`

Unknown parts fall back to `config.DEFAULT_THRUST_AXIS = (0, 0, -1)` (KSP stack-engine convention). Verified correct with Spark engines after `space_center.load("aegis_tune_start")`.

---

### ISS-016 — `max_thrust` queried once at init; stale value affects a_avail and allocator at altitude

- **Severity:** 🔴 CRITICAL
- **Status:** RESOLVED
- **Date opened:** 2026-06-15
- **Module(s):** Mission Director, Control Allocator
- **Related ADR:** None

**Description**
`Engine.max_thrust` is queried once during `MissionDirector.__init__` and stored in `e.max_thrust`. In kRPC, `part.engine.max_thrust` changes with atmospheric pressure — engines produce more thrust in thin air. The stale sea-level value causes two problems:

1. **`a_avail` underestimation** — `total_max_thrust = sum(e.max_thrust)` is too low → sqrt profile computes a slower target speed than the vehicle can actually brake to → the vehicle appears "too fast" relative to the profile → commands maximum braking.
2. **Allocator throttle overestimation** — `throttle = f_mag / engine.max_thrust` divides by the lower stale value → actual physical thrust exceeds commanded → vehicle brakes harder than expected → overshoots target speed → stops and reverses.

**Acceptance Criteria**
- `e.max_thrust` is refreshed from `part.engine.max_thrust` on every tick before `a_avail` and allocator run.
- `a_avail` reflects actual atmospheric conditions at current altitude.
- Allocator throttle correctly represents the fraction of available thrust at current altitude.

**Resolution**
Added a `max_thrust` refresh loop in `main.py` before the `a_avail` computation (lines 547-552): each tick, `e.max_thrust = engine_obj.max_thrust` is queried fresh from kRPC for all active engines. This ensures both the sqrt profile target speed and the allocator throttle calculation use the altitude-correct thrust.

In tandem, `ACCEL_CLAMP_FACTOR` raised from 1.5 to 2.5 so the clamp satisfies `clamp >= 1 + g / a_avail` for TWR >= 1.5 vehicles, allowing the profile's required `a_avail` net deceleration to be achieved through the clamp.

---

### ISS-018 — Duplicate `scipy.spatial.transform` import in `physics.py:step()`

- **Severity:** 🔵 MINOR
- **Status:** OPEN
- **Date opened:** 2026-06-30
- **Module(s):** Simulation
- **Related ADR:** ADR-031
- **Related Review:** None

**Description**
`src/simulation/physics.py` already imports `from scipy.spatial.transform import Rotation as R` at module level (line 8). The `step()` method re-imports the same name locally at line 144. This is dead code — the local import shadows the module-level one with no effect, and signals a refactor that was never finished.

**Acceptance Criteria**
- Single import of `Rotation as R` remains, at module level.
- `step()` no longer contains `from scipy.spatial.transform import Rotation as R`.
- mypy / pytest still pass.

**Resolution**
<!-- Fill in when resolved -->

---

### ISS-019 — `pyray` missing from `requirements.txt`

- **Severity:** 🔵 MINOR
- **Status:** OPEN
- **Date opened:** 2026-06-30
- **Module(s):** Visualizer
- **Related ADR:** None
- **Related Review:** None

**Description**
`scripts/visualize_physics.py` imports `pyray as pr` (line 7) but the dependency is not listed in `requirements.txt` (which currently contains only `krpc`). Anyone following the README's "sandbox execution" instructions will hit `ModuleNotFoundError: No module named 'pyray'` on first run.

**Acceptance Criteria**
- `pyray` (or the canonical Python Raylib binding) appears in `requirements.txt` with a pinned version compatible with the rest of the stack.
- A fresh `pip install -r requirements.txt` followed by `python scripts/visualize_physics.py` works on a clean environment.

**Resolution**
<!-- Fill in when resolved -->

---

### ISS-020 — `PhysicsState` has no input validation; invalid `q` / `throttles` / `fuel_mass` are accepted silently

- **Severity:** 🔵 MINOR
- **Status:** OPEN
- **Date opened:** 2026-06-30
- **Module(s):** Simulation
- **Related ADR:** ADR-031
- **Related Review:** None

**Description**
`PhysicsState` is a `@dataclass` with no `__post_init__`. A quaternion `[0,0,0,0]` is accepted: the `1e-12` norm guard at `physics.py:81` prevents division by zero but leaves the state un-normalized for that stage. Throttles outside `[0, 1]` propagate. Negative `fuel_mass` propagates. This makes bugs in callers hard to localize.

**Acceptance Criteria**
- `__post_init__` validates: `q` has finite values and `np.linalg.norm(q) > 1e-9`; `pos`, `vel`, `omega` are 1D arrays of length 3; `throttles` is 1D and all values in `[0, 1]`; `fuel_mass >= 0`.
- Invalid inputs raise `ValueError` with a clear message identifying the offending field.
- Existing tests still pass.

**Resolution**
<!-- Fill in when resolved -->

---

### ISS-021 — `DigitalTwin` lacks a `reset()` method; trials require rebuilding the twin

- **Severity:** 🔵 MINOR
- **Status:** OPEN
- **Date opened:** 2026-06-30
- **Module(s):** Simulation
- **Related ADR:** ADR-031
- **Related Review:** None

**Description**
The only way to re-run a trial today is to construct a new `DigitalTwin` and re-pass the initial `PhysicsState`. For training loops, parameter sweeps, and the Gremlin script, a `reset(initial_state)` method that clears `failed_engines`, restores `state` from the argument, and resets `landed` would be a clean primitive.

**Acceptance Criteria**
- `DigitalTwin.reset(initial_state: PhysicsState) -> None` exists and restores the twin to the same observable state as a fresh `__init__` with the given initial state.
- `failed_engines` is cleared (or accept a `keep_failures: bool = False` flag).
- `landed` is reset to `False`.
- Unit test asserts that `reset()` followed by identical commands produces a state equal to a freshly-constructed twin.

**Resolution**
<!-- Fill in when resolved -->

---

### ISS-022 — `MockVessel` mass constants are duplicated and `MockVessel` / `SimpleTestVessel` are not unit-tested

- **Severity:** 🔵 MINOR
- **Status:** OPEN
- **Date opened:** 2026-06-30
- **Module(s):** Simulation
- **Related ADR:** ADR-031
- **Related Review:** None

**Description**
`MockVessel.get_com_position` hardcodes `dry_mass = 40.0` (line 67) duplicating the `40.0` in `total_mass` (line 55). The visualizer is the only consumer; if the visualizer ever breaks because the constants drift, no test will catch it. Additionally, neither mock vessel has a dedicated unit test (they are exercised incidentally through `DigitalTwin` tests with hand-built engines).

**Acceptance Criteria**
- `MockVessel` and `SimpleTestVessel` store their constants (`dry_mass`, `dry_com`, `fuel_com`, `max_thrust`, `engine_tau`, etc.) in `__init__` and reference them everywhere.
- `inertia_tensor` and `get_com_position` reflect the stored values, not magic numbers.
- New tests in `tests/test_simulation.py` (or `tests/test_mock_vessel.py`) cover: `total_mass` linearity, `get_fuel_burn_rate` linearity, `get_com_position` linear interpolation between dry and fuel CoM, `get_drag_force` sign and magnitude.

**Resolution**
<!-- Fill in when resolved -->

---

### ISS-023 — `np.linalg.inv(I)` recomputed 4× per RK4 step; should use `np.linalg.solve`

- **Severity:** 🔵 MINOR
- **Status:** OPEN
- **Date opened:** 2026-06-30
- **Module(s):** Simulation
- **Related ADR:** ADR-031
- **Related Review:** None

**Description**
`physics.py:219` calls `np.linalg.inv(I)` on every derivative evaluation. For RK4 with 4 stages that's 4 inversions per step. `np.linalg.solve(I, b)` is both faster and numerically more stable (avoids the explicit inverse).

**Acceptance Criteria**
- `np.linalg.inv(I)` replaced with `np.linalg.solve(I, ...)`.
- `I` is cached once per `step()` call (valid because `I` depends only on `fuel_mass`, which is already propagated through the stage states — but for an immediate win, just swap the call).
- Existing physics tests still pass with the same numerical tolerance.
- A simple micro-benchmark shows ≥10% speedup on a 60 s simulated descent.

**Resolution**
<!-- Fill in when resolved -->

---

### ISS-024 — `DigitalTwin` has no determinism seed hook

- **Severity:** 🟡 MAJOR
- **Status:** OPEN
- **Date opened:** 2026-06-30
- **Module(s):** Simulation, Testbed
- **Related ADR:** ADR-031
- **Related Review:** None

**Description**
ADR-031 claims the simulation is "deterministic". That's true today only because there are no random number sources. The moment any caller (FDI injection, NN-ADRC exploration, Gremlin) introduces a stochastic call, determinism is lost. A `seed: int | None = None` parameter on `__init__` (or a `set_seed(seed)` method) that initializes a private `numpy.random.Generator` makes the contract explicit and reproducible.

**Acceptance Criteria**
- `DigitalTwin.__init__` accepts `seed: int | None = None`.
- The seed backs a private `np.random.Generator` exposed as `self.rng` (or via a `self.random() -> float` method).
- Two runs with the same seed, same commands, same `kill_engine` calls, produce bit-identical `PhysicsState` outputs.
- Document the contract in the `DigitalTwin` docstring and in ADR-031.

**Resolution**
<!-- Fill in when resolved -->

---

### ISS-025 — Visualizer overlay and command construction are hardcoded to 4 engines

- **Severity:** 🔵 MINOR
- **Status:** OPEN
- **Date opened:** 2026-06-30
- **Module(s):** Visualizer
- **Related ADR:** None
- **Related Review:** None

**Description**
`scripts/visualize_physics.py` hardcodes 4 engines: the overlay loop at line 77 iterates over `state.throttles` (which is fine), but the `cmd_throttles = np.array([hover_throttle + throttle_adj] * 4)` at line 129 and the per-engine markers in `draw_vessel` are sized for 4. Swapping in `SimpleTestVessel` (1 engine) crashes. The visualizer should derive the engine count from `vessel.engines`.

**Acceptance Criteria**
- `cmd_throttles` length matches `len(vessel.engines)`.
- `draw_vessel` iterates over `vessel.engines` (already does, but verify the indexing into `state.throttles` is consistent for any N).
- Overlay renders correctly for 1, 2, and 4 engine vessels.
- Visualizer runs against `SimpleTestVessel` and `MockVessel` without code changes.

**Resolution**
<!-- Fill in when resolved -->

---

### ISS-026 — `sys.path.append` hack in `visualize_physics.py`; the script isn't installed as a package

- **Severity:** 🔵 MINOR
- **Status:** OPEN
- **Date opened:** 2026-06-30
- **Module(s):** Visualizer
- **Related ADR:** None
- **Related Review:** None

**Description**
`scripts/visualize_physics.py:3` mutates `sys.path` to import `src.simulation.*`. This works but is fragile (depends on CWD, breaks if the script is moved, no static analysis coverage). Adding a `pyproject.toml` with `[tool.setuptools]` `packages = ["src"]` (or using `python -m scripts.visualize_physics` after making `scripts/` a package) removes the hack.

**Acceptance Criteria**
- The `sys.path.append` line is removed.
- `python scripts/visualize_physics.py` (or the `python -m` equivalent) works from the repo root without manual PYTHONPATH manipulation.
- A `pyproject.toml` exists with minimal `setuptools` config so the package is installable (`pip install -e .`).

**Resolution**
<!-- Fill in when resolved -->

---

### ISS-027 — `Engine` class couples simulation to kRPC; simulation should consume a pure `EngineSpec`

- **Severity:** 🟡 MAJOR
- **Status:** OPEN
- **Date opened:** 2026-06-30
- **Module(s):** Simulation, KSP Adapter
- **Related ADR:** ADR-031
- **Related Review:** None

**Description**
`src/simulation/vessel.py` declares the abstract interface as `engines -> list[Engine]`, but `Engine` is `src/common/engine.py`, which carries kRPC fields (`part: Any`, `gimbal_module: Any`, `krpc_engine: Any`). This makes `src/simulation/` dependent on the kRPC adapter in spirit if not in import path. The clean separation promised by ADR-031 ("`VesselModel` defines engines, mass properties, and drag properties") is undermined.

**Acceptance Criteria**
- New dataclass `src/simulation/engine_spec.py::EngineSpec` with only physics fields: `index`, `position`, `thrust_direction`, `max_thrust`, `max_gimbal_deg`, `gimbal_x_axis`, `gimbal_y_axis`.
- `VesselModel.engines -> list[EngineSpec]`.
- `MockVessel` and `SimpleTestVessel` return `EngineSpec` instances.
- KSP adapter (`main.py` or a new `src/ksp/engine_adapter.py`) converts `Engine` → `EngineSpec` before constructing the `DigitalTwin` (or the `DigitalTwin` is fed `VesselModel` instances directly, depending on the cleanest boundary).
- No new imports of `src.common.engine` from `src/simulation/`.
- Existing tests still pass.

**Resolution**
<!-- Fill in when resolved -->

---

### ISS-028 — Tests reach into private API (`_compute_derivatives`); refactor to a public `derivatives()` method or `Dynamics` object

- **Severity:** 🟡 MAJOR
- **Status:** OPEN
- **Date opened:** 2026-06-30
- **Module(s):** Simulation
- **Related ADR:** ADR-031
- **Related Review:** None

**Description**
`tests/test_simulation.py` calls `dt._compute_derivatives(...)` (lines 107, 139, 158, 189) to assert pure dynamics. The leading underscore signals "private" — a public contract is preferable. Two clean options:
1. Promote `_compute_derivatives` to a public `derivatives(...)` method.
2. Extract the dynamics into a `Dynamics` class that both `DigitalTwin.step()` and tests consume; `DigitalTwin` becomes a thin orchestrator.

Option 2 is the deeper seam and aligns with the "deep module" architecture goal: tests then exercise the same primitive the integrator does, with no privileged access.

**Acceptance Criteria**
- `Dynamics` class (or equivalent) holds a `VesselModel` and `EnvironmentModel`, exposes a `compute(state, cmd_throttles, cmd_gimbals) -> PhysicsDerivatives` method.
- `DigitalTwin.step()` uses `Dynamics.compute` for all 4 RK4 stages.
- Tests call `dynamics.compute(state, ...)` (or `dt.derivatives(state, ...)`); no `_compute_derivatives` access.
- All existing tests still pass with no semantic change.
- New `tests/test_dynamics.py` (or extension of `test_simulation.py`) covers the previously-internal cases as black-box tests.

**Resolution**
<!-- Fill in when resolved -->

---

### ISS-029 — Gimbal sign convention is implicit and untested for torque direction

- **Severity:** 🟡 MAJOR
- **Status:** OPEN
- **Date opened:** 2026-06-30
- **Module(s):** Simulation
- **Related ADR:** ADR-031
- **Related Review:** None

**Description**
`physics.py:190-195` linearizes the gimbal displacement as `f_dir = thrust_direction + gx * gimbal_y_axis - gy * gimbal_x_axis`. The sign in the `gy` term is opaque: the `Engine` constructor builds `gimbal_x_axis = thrust × arbitrary` and `gimbal_y_axis = thrust × gimbal_x_axis` (line 41-53 of `engine.py`), so both are displacement directions, not rotation axes. The minus sign is an arbitrary choice, and the existing gimbal test (`test_rotational_physics`) only verifies the *clamp* — both clamped and unclamped use the same (potentially wrong) direction, so a sign error is silent.

**Acceptance Criteria**
- A new test specifies the sign convention explicitly: e.g. "for an engine at `position = [1, 0, 0]`, a positive `gy` (gimbal x) tilts thrust in the `-y` body direction, producing a positive body-y torque". The test then asserts the actual torque matches.
- A `GIMBAL_SIGN_CONVENTION` docstring is added to `Engine` documenting which `gx`/`gy` produces which physical direction.
- A round-trip test: build a known rotation matrix R that rotates thrust by 5° about body x, convert to `(gx, gy)` by `R^{-1} @ thrust_direction`, assert the simulation's torque matches the analytical one within 1% (small-angle regime).

**Resolution**
<!-- Fill in when resolved -->

---

### ISS-030 — Quaternion integration may drift; consider `exp(0.5*ω*dt)` formulation

- **Severity:** 🟡 MAJOR
- **Status:** OPEN
- **Date opened:** 2026-06-30
- **Module(s):** Simulation
- **Related ADR:** ADR-031
- **Related Review:** None

**Description**
`physics.py:119` propagates the quaternion via `q_new = q + (h/6) * Σ dq_i` then re-normalizes once at line 125-127. Re-normalization only at the end of the step is OK for small step sizes and short horizons but is the most common source of long-term drift in rigid-body simulators. A more stable formulation is `q_new = exp(0.5 * ω_avg * dt) ⊗ q` (with re-normalization at every stage).

The current `test_rotational_physics` only runs 10 ticks (0.2 s) — long enough for RK4 to be stable, but not long enough to expose drift.

**Acceptance Criteria**
- Document the current tolerance: run a 10-minute simulated constant-rate spin and assert `|‖q‖ - 1| < 1e-9` at every tick.
- Decide between (a) accepting the current formulation with documented tolerance, (b) switching to the `exp(0.5*ω*dt)` formulation per stage.
- If switching: assert `|‖q‖ - 1| < 1e-15` at every stage, no observable difference in the existing 10 s test, and identical outputs modulo float rounding.
- Update ADR-031 with the decision and rationale.

**Resolution**
<!-- Fill in when resolved -->

---

### ISS-031 — Visualizer is a closed-loop demo; missing interactive controls and state history

- **Severity:** 🟡 MAJOR
- **Status:** OPEN
- **Date opened:** 2026-06-30
- **Module(s):** Visualizer
- **Related ADR:** None
- **Related Review:** None

**Description**
`scripts/visualize_physics.py` runs a fixed closed-loop descent with hardcoded initial state and hardcoded controller. There are no interactive controls (no key bindings to command gimbals, kill an engine, pause, change the descent target, or reset). The overlay shows only the current state — no trajectory trace, no time-series strip. For a "physics visualizer" the missing features are the main value proposition.

**Acceptance Criteria**
- Key bindings (at minimum):
  - `Space` — pause / resume the simulation
  - `R` — reset to the initial state
  - `1..4` — kill the corresponding engine
  - `WASD` — command gimbals on all engines (collective pitch/yaw)
  - `[` / `]` — reduce / increase target descent rate
  - `Esc` — close the window
- A trajectory trace: a fading line in 3D showing the CoM path over the last N seconds.
- A time-series strip in the HUD: altitude vs time, vertical speed vs time, both scrollable.
- Document the controls in the `main()` docstring and a `print` on startup.

**Resolution**
<!-- Fill in when resolved -->

---

### ISS-032 — `np.ndarray` type hints are deprecated in numpy 2.0; add a mypy config to enforce

- **Severity:** 🔵 MINOR
- **Status:** OPEN
- **Date opened:** 2026-06-30
- **Module(s):** Cross-cutting (simulation, kRPC adapter)
- **Related ADR:** None
- **Related Review:** None

**Description**
`AGENTS.md` mandates mypy. `requirements.txt` does not pin numpy, so a numpy 2.0 install will trigger deprecation warnings for bare `np.ndarray` annotations. There is no `pyproject.toml` or `mypy.ini` in the repo, so the "mypy is enforced" claim cannot be verified.

**Acceptance Criteria**
- `pyproject.toml` (or `mypy.ini`) exists with `mypy` configuration: `python_version = "3.12"`, `strict = true` (or a documented relaxation), `numpy.typing.NDArray[np.float64]` allowed.
- `requirements-dev.txt` (or `[project.optional-dependencies] dev` in `pyproject.toml`) pins `mypy` and `numpy` versions.
- `mypy src/simulation/` reports 0 errors.
- A follow-up issue can then sweep the rest of `src/` (out of scope for this issue).

**Resolution**
<!-- Fill in when resolved -->

