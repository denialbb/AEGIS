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

- **Severity:** 🔴 CRITICAL
- **Status:** OPEN
- **Date opened:** 2026-06-13
- **Module(s):** FDI
- **Related ADR:** ADR-004
- **Related Review:** None

**Description**
The FDI compares expected vs measured acceleration to detect engine failure. The deviation threshold that triggers a fault flag is currently a placeholder value. It has not been calibrated against actual Kalman filter output variance for this vessel. Too tight a threshold produces false positives during normal burn transients. Too loose a threshold means a dead engine goes undetected until the vessel is already spinning.

**Acceptance Criteria**

- Threshold derived from measured State Estimator output noise across at least 3 nominal descent runs.
- FDI correctly flags a simulated engine failure within 2 physics ticks under nominal noise conditions.
- FDI produces zero false positives across 5 consecutive nominal descents.

**Resolution**

<!-- Fill in when resolved -->

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

### ISS-003 — Kalman filter Q and R matrices are uninitialised placeholders

- **Severity:** 🔴 CRITICAL
- **Status:** OPEN
- **Date opened:** 2026-06-13
- **Module(s):** State Estimator
- **Related ADR:** ADR-007
- **Related Review:** None

**Description**
The process noise covariance (Q) and measurement noise covariance (R) matrices are required inputs to the Kalman filter and directly determine its behaviour. These values must match the actual noise characteristics of the injected Gaussian noise and the vessel's true dynamics. Incorrectly tuned Q/R causes the filter to either trust noisy measurements too much (R too high) or ignore them and drift (R too low). Currently these are set to identity matrices as a scaffold.

**Acceptance Criteria**

- R tuned to match the variance of the injected noise wrapper for each sensor (altimeter, accelerometer).
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
- GLIDESLOPE_K_ALT and per-phase max descent rates are documented as tunable
  parameters requiring empirical calibration against the vessel's actual
  thrust-to-weight ratio (similar caveat to ISS-001/ISS-003).
- ARCHITECTURE.md / a new ADR documents the glide-slope target-generation
  algorithm (suggest ADR-022, following the ADR-021 precedent for ISS-010).

**Resolution**

Implemented `_compute_glideslope_target` to dynamically track velocity while zeroing vertical position error, avoiding actuator saturation.

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
