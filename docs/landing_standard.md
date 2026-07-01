# AEGIS Landing Accuracy Standard

**Status:** ACCEPTED 2026-06-30
**Scope:** HOVER_TARGETING and TERMINAL_DESCENT phases (final-approach landing).
**Source of truth:** `src/config/landing_standard.conf`

## Purpose

Quantify "did the landing work?" with measurable pass/fail thresholds, so we can iterate on controller changes against a fixed standard rather than vibes-based comparison of telemetry dumps. The standard is enforced by:

1. `scripts/score_landing.py` (extended) — prints per-metric values for the latest run.
2. `tests/test_landing_standard.py` (to be written) — pytest assertions over the latest run.

## Pre-implementation baseline (verbal, unverified)

A pre-existing record in `AGENTS.md` mentions a prior run with 12.0 m EKF touchdown distance and 24.8 m/s horizontal velocity; the user has also reported a more recent post-velocity-based-guidance run with ~5 m touchdown and 30–60s of oscillation in the close vicinity (under 100m altitude) before settling. **Neither of these is reproducible from telemetry in this freshly-cloned repo** — the data is anecdotal or from a different clone. The standard's thresholds below are derived from the physical design intent and Apollo-style references, not from the user's reported baselines. Once we have telemetry from a clean run in this repo, we will compare against the standard directly.

## Metrics

All values in `src/config/landing_standard.conf`. All distance/velocity metrics are computed on **EKF estimates** (`pos_n/e`, `vel_x/y/z` from `telemetry.csv`), not kRPC truth — the current telemetry pipeline does not record truth position/velocity. The thresholds assume an EKF-truth gap of ≤ 1.0 m at touchdown, which holds once Mahony bias has converged (see ISS-003).

| Metric | Target | Hard fail | Source columns |
|---|---|---|---|
| Touchdown horizontal distance (m) | < 2.0 | > 5.0 | `pos_n`, `pos_e` at touchdown tick |
| Touchdown horizontal velocity (m/s) | < 0.5 | > 2.0 | `vel_x`, `vel_y` at touchdown tick |
| Touchdown vertical velocity (m/s) | < 0.8 | > 2.0 | `vel_z` at touchdown tick |
| Approach time HOVER→touchdown (s) | < 30 | > 60 | `events.jsonl` (HOVER entry) → `timestamp` |
| Lateral overshoot peak (m) | < 0.5 | > 2.0 | max over phase of `sqrt(pos_n²+pos_e²)` |
| Roll at touchdown (deg) | < 5 | > 15 | `est_roll` at touchdown tick |
| EKF vs truth attitude gap (deg) | < 3 | — | `\|est_roll - true_roll\|` at touchdown |

## Touchdown detection

The "touchdown tick" is the first row in `telemetry.csv` where `est_alt <= TOUCHDOWN_ALTITUDE` (currently 20.6 m in `aegis.conf`). This is the EKF's first crossing, which is earlier than the kRPC `situation == "landed"` event by ~50–200 ms. The first crossing is the right anchor for *control* scoring (we want to know what the controller did as it crossed the ground), while the kRPC event is the right anchor for *physics* scoring. The standard uses the first crossing for controller scoring.

## Rationale for the values

- **Distance < 2.0 m, < 5.0 m hard fail:** Apollo LM Programmed Test Input (PTI) target was within ~2 m of the pad centre. SpaceX ASDS / LZ-1 declared accuracy is coarser (~10 m) but their final-phase accuracy is not public; KSP's tighter simulation tolerates the tighter number.
- **Touchdown velocity < 0.5 m/s horiz, < 0.8 m/s vert:** Apollo LM touchdown limits were 1 ft/s (~0.3 m/s) vertical, ~2 ft/s (~0.6 m/s) horizontal. The Apollo numbers assume human-rated; for an unmanned KSP vehicle, 2× Apollo is a defensible "soft" target.
- **Approach time < 30 s target, < 60 s hard fail:** Empirically, an aggressive Apollo-style approach with 1.0 m/s² brake acceleration covers 100 m in ~14 s and 50 m in ~10 s. The 30 s target is the time for a 200 m miss at 0.5 m/s² brake — well within reach of the current TWR. 60 s hard fail catches configurations that effectively stop moving.
- **Lateral overshoot < 0.5 m, < 2.0 m hard fail:** Any overshoot indicates a limit cycle in the PD or poor braking authority. 0.5 m is "you can barely see it in the trajectory plot"; 2.0 m is "obvious to a reviewer."
- **Roll at touchdown < 5°:** Vehicle must be level for gear contact. 15° is the KSP gear / leg contact failure threshold.
- **EKF vs truth attitude gap < 3°:** Catches the Mahony bias drift that's the likely cause of the "overcorrection" symptom. If gap > 3° at touchdown, the EKF is untrustworthy and the touchdown distance number is suspect.

## Out of scope (not in the standard)

- **Convex-optimization optimality** (fuel/time trade-off): tracked by `tuning_log.csv` `total_score`, not pass/fail.
- **FDI response time**: tracked by `events.jsonl` `FAULT_DETECTED` events.
- **Hard-abort correctness**: tracked by `events.jsonl` `STATE_TRANSITION` to `HARD_ABORT`.
- **Pre-HOVER phases** (DEORBIT_BURN, HYPERSONIC_COAST, POWERED_DESCENT): separate standards, not yet written.
- **Approach smoothness / oscillation count**: not currently measured. A "number of direction reversals during HOVER_TARGETING+TERMINAL_DESCENT" metric would directly characterize the hunting behaviour and is a candidate for a future metric.

## Open questions (deferred)

- Should the standard include a "minimum stable hover time" metric (e.g., must hold < 0.5 m lateral drift for 5 s before allowing TERMINAL_DESCENT)? This is currently implicit in the state-machine logic but not measured.
- Should we add a "fuel reserve at touchdown" metric to prevent the controller from optimising accuracy at the cost of safety margin? Currently `total_throttle` integral is tracked but not as a hard limit.

## Domain terms

These terms were sharpened during the design discussion that produced this standard. They are pinned here so the same vocabulary is used in code review, ADRs, and future commits.

- **Residual NN (ADRC-style)** — Neural network that *augments* a classical controller by learning an unmodeled disturbance or correction term. The classical controller remains the authority. The existing `src/guidance/nn.py` is a residual NN for attitude.
- **Policy NN (end-to-end)** — Neural network that *replaces* the classical controller, mapping state directly to control output. Not used in AEGIS; not appropriate for rocket landing translation.
- **Trajectory NN / setpoint generator** — Neural network that *replaces* a trajectory optimizer, producing a desired state trajectory. Not used; convex optimization is the correct tool.
- **Bounded-acceleration guidance** — A guidance law where the desired velocity tapers with distance: `v_target = sqrt(2 * a_brake * dist)`. Single tunable `a_brake` per phase. Already in `flight_control.py:774-776,807-820`.
- **Hybrid guidance** — Velocity-based guidance outer, position-based guidance inner when `dist < threshold`. The candidate fix for the close-vicinity oscillation if hypothesis (A) is confirmed.
- **Hunting / limit cycle** — A periodic oscillation around the desired equilibrium caused by the control law structure (sign flip, dead zone) or saturation. Quantified here as "target-velocity quadrant flips" and "lateral overshoot peak."
