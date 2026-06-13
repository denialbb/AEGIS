# PENDING REVIEW
> Fill this file out completely before signaling a review is ready.
> Incomplete submissions will be returned without review.

---

## Meta
- **Branch:** `feature/state-estimator`
- **Commit hash:** `[LATEST]`
- **Timestamp:** `2026-06-13 01:25 UTC`
- **Module(s) touched:** `State Estimator, FDI, Control Allocator, Mission Director`
- **Review urgency:** `[x] Blocking  [ ] Standard  [ ] Low-priority`

---

## Summary of Changes
Implemented fixes addressing Claude's code review (REVIEW_2026-06-13_0130). Transitioned the State Estimator to a 6D state vector (omitting mass) using the "Option A" fusion strategy where noisy acceleration is fed directly into the predict step and altitude serves as the sole measurement update. Re-tuned FDI logic to accommodate simultaneous multiple-engine failures using combinatorics, while incorporating missing comments about strict kRPC mass dependency (ISS-006). Modified Control Allocator to dynamically detect physically absurd commands by monitoring the condition number (`1e4` threshold) of the configuration matrix, successfully mapping these degenerate states to a Hard Abort condition in the Mission Director. 

---

## Changed Files

| File | Change Type | Notes |
|------|-------------|-------|
| `src/estimation/estimator.py` | `Modified` | Changed state from 7D to 6D, decoupled predict/update. |
| `tests/test_estimator.py` | `Modified` | Updated for 6D state, added `test_estimator_noisy_update`. |
| `src/guidance/allocator.py` | `Modified` | Implemented `AllocationDegenerateError`, checks `cond > 1e4`. |
| `tests/test_allocator.py` | `Modified` | Updated tests to check for `AllocationDegenerateError` correctly. |
| `src/fdi/fdi.py` | `Modified` | Changed `isolate_fault` to use combinatorics to find up to N failures. Added ISS-006 comments. |
| `tests/test_fdi.py` | `Modified` | Added `test_isolate_multiple_faults`. |
| `src/main.py` | `Modified` | Refactored `run_loop` to handle multiple failures and degenerate matrices by triggering `HARD_ABORT`. |
| `.agents/shared/context/ARCHITECTURE.md` | `Modified` | Documented 6D state, 1x1 R-matrix, degenerate allocator exception, and Hard Abort conditions. |
| `.agents/shared/context/DECISIONS.md` | `Modified` | Added ADR-010 for condition number threshold, updated ADR-007 for Option A Accelerometer fusion. |

---

## Interface Contracts

- **State Estimator:** `update(noisy_alt, noisy_accel, dt)` was split into `predict(noisy_accel, dt)` and `update(noisy_alt)`. State output is `(6,)` instead of `(7,)`.
- **Control Allocator:** Now raises `AllocationDegenerateError` explicitly rather than returning unphysical bounds, changing its API contract from implicitly absorbing errors to explicitly failing loudly.
- **FDI:** `isolate_fault` now returns a list of failed engines (which may contain >= 2 engines) rather than isolating a single candidate.

---

## Mathematical / Algorithmic Notes
- **State Estimator:** Uses a linear Kalman Filter (6D state, 1D measurement) leveraging the accelerometer purely as a dynamic control input (B matrix).
- **Control Allocator:** Monitors structural stability by comparing the condition number (`np.linalg.cond(B)`) against a conservative threshold (`1e4`). If it exceeds this, the matrix is numerically degenerate, guaranteeing unphysical control bounds.
- **FDI:** To find 2+ failures simultaneously, `isolate_fault` now exhaustively checks combinations of failures up to $N$, selecting the configuration that best matches the un-accounted force deviation.

---

## Self-Identified Concerns

- [ ] `cond > 1e4` threshold: Might need tuning pending real KSP physics interactions if the scale of forces generates naturally high condition numbers.
- [ ] Combinatorial FDI: Is safe for $N \le 6$ but computationally explosive if we ever transition to massive $N$-engine platforms (e.g. 33 Raptors). For AEGIS scope this is fine.

---

## Testing Done
- Mypy strictly typed checks cleanly.
- `pytest tests/` successfully validates single faults, simultaneous multiple faults, degenerate rank allocations, and synthetic noisy Kalman filter falling estimates.

---

## Context for Reviewer
Resolves Blockers B1, B2, B3 and Majors M1-M5 from REVIEW_2026-06-13_0130.

---

## Status
- [ ] Ready for first review
- [x] Revision after review `[REVIEW_2026-06-13_0130]` — changes described above
