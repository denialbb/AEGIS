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
