# AGENTS.md — AEGIS

## Critical Execution Rules

- **Always use `.venv` from Arch Linux with `uv`** for running, typechecking, and testing:
- Python: `.venv/bin/python src/main.py`
- mypy: `.venv/bin/mypy .`
- pytest: `.venv/bin/pytest`
- Do not use system Python or run from Windows directly.
- The virtual environment is managed by `uv`, the modern Python package manager, ensuring fast, reliable dependency resolution in the Arch Linux WSL2 environment.

## Architecture (4 Strictly Decoupled Modules)

| Module            | Location                      | Responsibility                                     |
| ----------------- | ----------------------------- | -------------------------------------------------- |
| Mission Director  | `src/main.py`                 | State machine, orchestrator                        |
| State Estimator   | `src/estimation/estimator.py` | Kalman filter on noisy telemetry                   |
| FDI               | `src/fdi/fdi.py`              | Fault detection & isolation                        |
| Control Allocator | `src/guidance/allocator.py`   | 6-DOF wrench → engine throttles via pseudo-inverse |

**No module may read another module's internal state.** All data flows through interfaces defined in `.agents/shared/context/ARCHITECTURE.md`.

## kRPC Conventions

- **Always use `conn.add_stream()`** for telemetry — never poll `vessel.flight()` directly (TCP latency).
- Engine parts are discovered via `vessel.parts.with_tag("AegisEngine")` (ADR-016), not by querying all engines.
- The WSL2 → Windows KSP connection requires the Windows host IP (typically from `/etc/resolv.conf` `nameserver`), not `127.0.0.1` (ADR-015).

## Type Safety

- All public functions **must** have full type hints. No implicit `Any`.
- `np.ndarray` shapes must be documented in docstrings (e.g., `shape (6,)`).
- mypy is enforced; validate with `wsl -d Arch .venv/bin/mypy .` before submitting.

## Numerical Safety

- **Pseudo-inverse condition number checked** (`cond > 1e4` → `AllocationDegenerateError`).
- B matrix rank deficiency checked (`rank < 6` → `AllocationDegenerateError`).
- Kalman filter: Q (process noise) and R (measurement noise) must be positive-definite.

## Known Issues

Tracked in `.agents/shared/context/OPEN_ISSUES.md`:

- **ISS-001**: FDI threshold placeholder (uncalibrated)
- **ISS-002**: Allocator condition number threshold
- **ISS-003**: Estimator attitude handling (TODO in `predict()`)
- **ISS-004**: Multiple simultaneous failures → HARD_ABORT
- **ISS-006**: FDI mass from clean kRPC (not noised)

Reference issue numbers in PRs rather than re-raising already-documented problems.

## State Machine States

`DEORBIT_BURN` → `HYPERSONIC_COAST` → `POWERED_DESCENT` → `HOVER_TARGETING` → `TERMINAL_DESCENT` → `HARD_ABORT`

Contingency triggers: single engine failure (allocates around it), 2+ simultaneous failures (immediate HARD_ABORT), degenerate allocation (immediate HARD_ABORT), dt spike > 3× expected (skips predict, logs DT_SPIKE).

## Multi-Agent Review Process

Code goes to `.agents/shared/queue/PENDING_REVIEW.md` following the template. Reviews land in `.agents/shared/reviews/REVIEW_[timestamp].md`. See `.agents/CLAUDE_CONTEXT.md` for severity labels (BLOCKER / MAJOR / MINOR) and verdict definitions.

## Reference Files

- Architecture contracts: `.agents/shared/context/ARCHITECTURE.md`
- Design decisions: `.agents/shared/context/DECISIONS.md`
- Open issues: `.agents/shared/context/OPEN_ISSUES.md`
- Architecture doc: `docs/architecture_design.md`

