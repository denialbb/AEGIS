# PENDING REVIEW
> Fill this file out completely before signaling a review is ready.
> Incomplete submissions will be returned without review.

---

## Meta
- **Branch:** `iss-011`
- **Commit hash:** `dcaf8de`
- **Timestamp:** `2026-06-14 06:50 UTC`
- **Module(s) touched:** `Control Allocator, Mission Director, Telemetry`
- **Review urgency:** `[ ] Blocking  [x] Standard  [ ] Low-priority`

---

## Summary of Changes
Integrated physical engine gimballing by actuating `ModuleGimbalTrim` fields on the vessel parts. Fixed a catastrophic issue in the pseudo-inverse allocator where mathematically unconstrained torque demands were producing requested forces of 4+ million Newtons. This was resolved by both hard-clamping the allocated lateral forces and adding an `rcond=1e-4` parameter to `np.linalg.pinv` to disregard floating-point singularities in the B-matrix. Lastly, gimbal oscillations were dampened by increasing the attitude PD derivative gain.

---

## Changed Files
| File | Change Type | Notes |
|------|-------------|-------|
| `src/main.py` | `Modified` | Added kRPC calls to toggle and set Gimbal Trim during the main loop |
| `src/guidance/allocator.py` | `Modified` | Clamped lateral force outputs to tan(5°) max to prevent throttle explosion; added rcond to pinv |
| `src/guidance/controller.py` | `Modified` | Target attitude vector uses `a_cmd_world` directly to steer into drift |
| `src/config.py` | `Modified` | Increased `GUIDANCE_KD_ATT` from 5.0 to 20.0 to dampen gimbals |
| `src/telemetry/frame.py` | `Modified` | Telemetry frame now records `gimbals` arrays correctly |
| `docs/*.md` | `Added` | Extensive markdown documentation detailing engine interfaces and control limits |

---

## Interface Contracts
No breaking interface changes. `TelemetryFrame` acquired a new `gimbals` property which is flattened automatically by the CSV writer.

---

## Mathematical / Algorithmic Notes
The B-Matrix (6xN) was assuming engines could translate forces with 360 degrees of freedom, combined with an ill-conditioned B-Matrix (all engines point exactly straight down, so lateral forces are entirely rank-deficient). We added `rcond=1e-4` to `pinv` to safely drop any singular values smaller than $10^{-4}$ of the max singular value. 
After computing the ideal 3D force vector per-engine, we hard-clamp the lateral magnitude to `axial_force * tan(5 degrees)` to guarantee the resulting computed throttle magnitude doesn't explode when the PD controller asks for unachievable torques.

---

## Self-Identified Concerns
- [x] The `GuidanceController` can theoretically output a desired acceleration pointing downwards (if it wants to rapidly descend). This causes `target_up_world` to point down, flipping the rocket. We mitigate this with high glideslope floors currently, but an explicit tilt constraint should probably be implemented in `controller.py`.
- [x] The 5-degree physical limit is hardcoded in `allocator.py` and `main.py` rather than being queried from the kRPC part (which requires digging through part metadata).

---

## Testing Done
Ran a full hardware-in-the-loop launch test via `run.sh` with live kRPC simulation. The allocator correctly managed the engines, successfully clamped the throttle explosion, and smoothly steered the vessel down to 500m where it transitioned into `HOVER_TARGETING`. The vessel effectively survived the flight and only ended because it organically exhausted all onboard fuel.

---

## Context for Reviewer
See `docs/architecture_design.md` for the overarching structural logic.

---

## Status
- [x] Ready for first review
- [ ] Revision after review `[REVIEW_timestamp]` — changes described above
