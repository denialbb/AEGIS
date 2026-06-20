# AGENTS.md — AEGIS

## Critical Execution Rules

- **Always use `.venv` from Arch Linux with `uv`** for running, typechecking, and testing:
- Python: `.venv/bin/python src/main.py`
- mypy: `.venv/bin/mypy .`
- pytest: `.venv/bin/pytest tests/` — always specify a test file or directory; never run from the root folder.
- Data analysis: `.venv/bin/python -c "$(cat script.py)"` or `.venv/bin/python script.py` using pandas, numpy
- Do not use system Python or run from Windows directly.
- The virtual environment is managed by `uv`, the modern Python package manager, ensuring fast, reliable dependency resolution in the Arch Linux WSL2 environment.

## Integration Testing

- **Full flight test**: `scripts/run_and_score.sh` — runs the full mission against a KSP instance (kRPC at `127.0.0.1` or override via `KRPC_ADDRESS`) and scores the landing telemetry.
- **Post-flight analysis**: After a run, verify trajectory with `scripts/trajectory_analysis.py` and attitude with `scripts/attitude_analysis.py`, both reading from `logs/latest/telemetry.csv`.
- **Dashboard**: `scripts/trial_dashboard.py` for a browser-based summary of all runs in `logs/runs/`.
- **Always run a full flight test after any SAS or guidance change.** Static analysis (mypy, pytest) does not catch runtime issues in the kRPC/KSP pipeline.

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
- kRPC server ports are **50000** (RPC) and **50001** (stream), not the default 5005/5006.
- KSP save files are at `/mnt/c/Games/KSP - minimal install/saves/AEGIS/`.

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

## Current Task: Velocity-based horizontal guidance for pad targeting

### In Progress
- Replace position-blend with velocity-based guidance in `HOVER_TARGETING` and `TERMINAL_DESCENT` to eliminate pad overshoot limit cycles (best pre-fix run: dist=12.0m, vh=24.8m/s at touchdown).

### Completed
- ✅ **Velocity-based guidance**: `HOVER_TARGETING` and `TERMINAL_DESCENT` now set `result[3:5] = APPROACH_K * to_pad`, capped at `APPROACH_MAX`. kp nullified by `horizontal_target=state_vector[:2]`.
- ✅ **Config params**: Added `HOVER_APPROACH_K=0.08`, `HOVER_APPROACH_MAX=12.0`, `TERMINAL_APPROACH_K=0.12`, `TERMINAL_APPROACH_MAX=5.0`. Increased `HOVER_KD_VEL_LATERAL=2.0`. Lowered `ALT_POWERED_DESCENT=1600`, `ALT_HOVER=50`.
- ✅ **Reaction wheel authority by phase**: `_set_rw_authority()` modulates RW torque limit (0.05–0.20) per phase.
- ✅ **Catastrophic impact detection**: KSP "landed" with descent rate >20 m/s → HARD_ABORT.
- ✅ **SAS simplified**: Locked to stability assist + per-phase RW authority.
- ✅ **min_descent_rate**: Added to `_compute_glideslope_target` to ensure descent even at near-zero speed.
- ✅ **mypy stubs**: Added missing type stubs for new config params.
- ✅ **Unit tests**: 149 passed, 13 skipped (recording-dependent), 0 failures.
- ✅ **Pre-existing test failures fixed**: 149 passed, 13 skipped (recording-dependent), 0 failures

