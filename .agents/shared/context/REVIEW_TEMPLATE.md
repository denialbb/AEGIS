# REVIEW — [YYYY-MM-DD_HHMM]
> Reviewing branch: `feature/[module-name]` | Commit: `abc1234`
> Reviewer: Claude (Chief Code Reviewer)
> Triggered by: [your name]

---

## Verdict
```
[ ] APPROVED       — No blockers. Minor notes below, apply at discretion.
[ ] APPROVED+ITER  — No blockers, but majors must be resolved before next PR.
[ ] NEEDS REVISION — One or more blockers. Do not merge. See findings below.
[ ] REJECTED       — Fundamental architectural issue. Requires design discussion before rework.
```

**Summary statement:**
<!-- 2–3 sentences on the overall state of this submission. -->

---

## Findings

### 🔴 BLOCKERS
<!-- Must be resolved before any merge. Each one numbered. -->
<!-- If none: write "None." -->

**B1 — [Short title]**
- **File:** `path/to/file.py`, line XX
- **Issue:** Clear description of what is wrong.
- **Why it matters:** What failure mode this causes (runtime crash, silent wrong output, torque divergence, etc.)
- **Suggested fix:** Concrete recommendation. Code snippet if helpful.

---

### 🟡 MAJORS
<!-- Significant issues that don't crash the system but will cause problems. Must be resolved before the module is considered stable. -->
<!-- If none: write "None." -->

**M1 — [Short title]**
- **File:** `path/to/file.py`, line XX
- **Issue:**
- **Why it matters:**
- **Suggested fix:**

---

### 🔵 MINORS
<!-- Style, maintainability, type hints, naming clarity. Apply at discretion. -->
<!-- If none: write "None." -->

**m1 — [Short title]**
- **File:** `path/to/file.py`, line XX
- **Note:**

---

## Module-Specific Checks

### Mathematical / Numerical
<!-- Kalman filter: covariance matrices positive-definite? Noise params reasonable? -->
<!-- Pseudo-inverse: condition number acceptable? Singularity handling present? -->
<!-- FDI threshold: calibrated to noise floor from State Estimator? -->

| Check | Result | Notes |
|-------|--------|-------|
| Matrix invertibility / conditioning | `Pass / Fail / N/A` | |
| Kalman covariance positive-definite | `Pass / Fail / N/A` | |
| FDI threshold vs noise floor | `Pass / Fail / N/A` | |
| Edge case: all engines dead | `Pass / Fail / N/A` | |
| Edge case: single engine surviving | `Pass / Fail / N/A` | |

### Type Safety (mypy)
| Check | Result | Notes |
|-------|--------|-------|
| All public functions type-hinted | `Pass / Fail / N/A` | |
| `np.ndarray` shapes documented | `Pass / Fail / N/A` | |
| No implicit `Any` in critical paths | `Pass / Fail / N/A` | |

### Architecture / Decoupling
| Check | Result | Notes |
|-------|--------|-------|
| Module stays within its domain | `Pass / Fail / N/A` | |
| No direct cross-module data access | `Pass / Fail / N/A` | |
| Interface contracts respected | `Pass / Fail / N/A` | |
| State machine transitions valid | `Pass / Fail / N/A` | |

---

## Positive Notes
<!-- What was done well. Not filler — only genuine observations. -->


---

## Open Questions for Human Review
<!-- Anything I'm flagging for your judgment specifically — design tradeoffs, scope questions, things that need a live test to verify. -->

- 

---

## References
- Resolves concerns from: `PENDING_REVIEW.md` — [timestamp]
- Related decisions: `DECISIONS.md` — [entry]
- Related open issues: `OPEN_ISSUES.md` — [entry]
