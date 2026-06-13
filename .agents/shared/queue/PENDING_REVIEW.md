# PENDING REVIEW
> Fill this file out completely before signaling a review is ready.
> Incomplete submissions will be returned without review.

---

## Meta
- **Branch:** `feature/telemetry-logger`
- **Commit hash:** `efaffd7`
- **Timestamp:** `2026-06-14 01:31 UTC`
- **Module(s) touched:** `Mission Director, Telemetry, Core Logging`
- **Review urgency:** `[ ] Blocking  [x] Standard  [ ] Low-priority`

---

## Summary of Changes
Implemented a robust, configurable application logging infrastructure and added `--debug` / `--log-to-file` CLI flags to facilitate easier debugging and monitoring. Resolved critical issues with the kRPC acceleration stream reference frame and launchpad initialization logic. Added `.gitignore` exclusions for the telemetry logs directory. Importantly, live testing with KSP was conducted; the WSL2 to Windows kRPC connection is working flawlessly, and the system correctly triggers a `HARD_ABORT` when expected.

---

## Changed Files

| File | Change Type | Notes |
|------|-------------|-------|
| `.gitignore` | `Modified` | Ignored `logs/` directory |
| `run.sh` | `Modified` | Added `--debug` flag propagation and kRPC launchpad init fixes |
| `src/main.py` | `Modified` | Integrated new logger, added `--debug` & `--log-to-file` args, fixed init |
| `src/config.py` | `Modified` | Added logging configuration settings |
| `src/common/logger.py` | `Added` | Core logger module implementation |
| `src/telemetry/sensors.py` | `Modified` | Fixed kRPC acceleration stream reference frame |
| `src/telemetry/writer.py` | `Modified` | Integrated logger |
| `src/fdi/fdi.py` | `Modified` | Integrated logger |
| `src/estimation/estimator.py` | `Modified` | Integrated logger |
| `src/guidance/allocator.py` | `Modified` | Integrated logger |
| `tests/*` | `Modified` | Minor adjustments in tests for logger integration |

---

## Interface Contracts
No interface changes. The logging module acts as an orthogonal service utilized by existing modules.

---

## Mathematical / Algorithmic Notes
No mathematical changes.

---

## Self-Identified Concerns

- [ ] Log file rotation and size management (currently might just append indefinitely).
- [ ] Telemetry stream initialization performance with live kRPC.
- [ ] Any potential race conditions in kRPC streams if network latency spikes.

---

## Testing Done
Live testing with KSP was successfully conducted. The connection logic correctly established a TCP connection with the kRPC server on the Windows host from the WSL2 environment. Telemetry streams (including the corrected acceleration stream) were successfully initialized. The system was observed to correctly trigger a `HARD_ABORT` state as expected during the test flight.

---

## Context for Reviewer
These changes build upon the recent network connectivity resolutions and aim to make debugging the live system much easier moving forward. See ADR-015 for the WSL2 connection context.

---

## Status
- [ ] Ready for first review
- [x] Revision after review `[REVIEW_20260614_012742]` — changes described above
