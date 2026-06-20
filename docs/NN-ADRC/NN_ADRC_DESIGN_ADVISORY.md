# NN-ADRC Integration — Design Advisory
> Advisory document, not a REVIEW_*.md (no submission exists yet — this is a
> pre-implementation design consultation on `NN_ADRC_IMPLEMENTATION_PLAN.md`).
> Author: Claude (Chief Code Reviewer)
> Inputs reviewed: `NN_ADRC_IMPLEMENTATION_PLAN.md`, `ARCHITECTURE.md`,
> `DECISIONS.md`, `OPEN_ISSUES.md`, current `estimator.py` (Error-State EKF with Mahony attitude) / `controller.py` /
> `fdi.py` / `allocator.py` / `main.py`, and the three project reference papers
> (Extended Kalman Filtering — Cornman & Mei; NN-Based ADRC for a Multi-Axis
> Gimbal System — Leblebicioglu et al.; Quaternion-Based Tracking Control Law
> Design — Elbeltagy et al.)

---

## 1. Executive Assessment

The plan correctly transcribes the ESO equations and `fal()` nonlinearity from
the gimbal paper — I checked the math line-by-line and it's continuous at
`|e| = δ` as the original `fal` function requires, and the prediction/update
structure matches the source. That part is sound.

The bigger issues are not in the equations themselves but in **where this
plan proposes to put them**. As written, the plan would:
 
1. Merge two conceptually different observers (an Error-State EKF for kinematic
     state, and an ESO for control-law disturbance estimation) into one module,
     which re-opens the module-boundary debate ADR-003 already settled, with ADR-014 superseded by ADR-030.
 2. Validate a NN compensator designed in the literature for **smooth,
     continuous, parametric disturbances** (cable drag, CoG offset, friction)
     against AEGIS's actual fault mode, which is a **discrete, step-function
     actuator failure** (an engine going to zero thrust instantly). These are
     different problems and the gimbal paper's own results (Example 2) show
     that even *continuous* disturbance changes degrade an offline-trained NN
     significantly — a step-change failure is a harder case the paper doesn't
     test.
3. Add a new cross-module dependency (FDI reading Guidance's internal ESO/NN
   state) without specifying the interface, which conflicts with the "no
   module reads another module's internal state directly" rule.
4. Introduce 6-DOF ADRC + NN as a single leap, when the cited gimbal paper
   only validates a 2-DOF case — and AEGIS still has three CRITICAL open
   issues (ISS-001, ISS-003, and the just-resolved-but-unvalidated-in-NN-context
   ISS-011 glide-slope) that this plan implicitly assumes are settled inputs.

None of this means "don't do it" — the underlying technique is well-supported
and the quaternion paper in particular gives AEGIS something it currently
lacks (a principled gain-selection method and inertia-aware feedforward
terms). But the plan needs to be split into smaller, independently-reviewable
phases with explicit interface contracts before any code is written.

---

## 2. Literature-to-Plan Mapping

| Plan section | Source paper | What transfers directly | What doesn't transfer / caveat |
|---|---|---|---|
| 1.1 EKF | Cornman & Mei (EKF) | Q/R tuning-by-measured-variance methodology (Section 3 of the paper) is directly reusable for ISS-003. | The paper's "EKF" is a 6-state Euler-angle filter for head orientation — it has **no analog to AEGIS's position/velocity/bias state**, and its equations (as transcribed in the plan) are linear, not extended. AEGIS now uses an Error-State EKF with Mahony attitude estimation (ADR-030) that estimates position, velocity, gyroscope bias, and accelerometer bias, with attitude handled externally. The EKF cost is higher than a simple filter but justified by bias estimation and attitude accuracy. |
| 1.2 ESO | Leblebicioglu et al. (gimbal NN-ADRC) | ESO equations and `fal()` transcribed correctly; β/α/δ/b0 parameter set matches. | The ESO is a **single-channel** observer driven by `u` (control input) and `y` (plant output) for *that controller's* model — it is not a sensor-fusion filter and doesn't belong next to the EKF (see §3.1). |
| 2.1–2.2 NN-ADRC core | Same gimbal paper | WSEF/TG structure, NN input (pos/vel/accel) and output (Δr̈) shape, offline feedforward NN + Levenberg-Marquardt training approach. | NN was trained/validated on **sinusoidal tracking under continuous disturbance torques**, not step-function actuator loss. Example 2 in the paper shows an NN trained on one disturbance profile only partially recovers performance under a *changed but still continuous* disturbance (35.3% vs 85.4%) — a discrete engine failure is further outside the training distribution. |
| 3. Quaternion tracking | Elbeltagy et al. | Gain-selection via natural frequency/damping ratio (Eq. 37: `d=2ζωₙ`, `k=ωₙ²`) gives a principled starting point for `GUIDANCE_KP_ATT`/`KD_ATT` instead of pure grid/Optuna search. The full control law (Eq. 15) includes inertia-scaled feedforward terms AEGIS currently lacks entirely. | The paper's law includes reaction-wheel momentum terms (`h`, `Ω(ω)(Jω+h)`) — AEGIS has no reaction wheels (ADR-016 explicitly defers RCS). Only the **gyroscopic cross-coupling `Ω(ω)Jω`** and **inertia-scaled torque `J·(Kp·e+Kd·ė)`** terms are relevant; the RW-specific terms should be dropped, not transcribed. |
| 4. Allocator integration | (plan's own pseudocode) | Matches `allocator.py`'s existing `pinv`/condition-number pattern almost exactly — good news, minimal allocator change needed. | Requires a 6-DOF Wrench computed with an **inertia tensor** AEGIS doesn't currently query or own anywhere (see §3.2). |
| 5. FDI adaptation | Gimbal paper's framing (NN "masks" the disturbance) | Conceptually correct — ADRC will reduce the *acceleration-deviation signature* FDI currently keys on. | The plan doesn't specify the new interface FDI would need, and ISS-010's hard-won fix (FDI must not run on stale/invalid signals during transients) needs to be re-derived for z₃/NN-output signals, which have their own transient dynamics. |
| 6. Gremlin/NN training | — | — | "CTM" in the gimbal paper (an idealized inverse-dynamics model) is structurally identical to what `fdi.isolate_fault` already computes as `expected_force/mass`. This is a reuse opportunity, not new code (§4.4). |

---

## 3. Architectural Analysis

### 3.1 The ESO does not belong in the State Estimator

`ARCHITECTURE.md` defines the State Estimator's job as: fuse noisy telemetry
into a clean kinematic state `[x,y,z,vx,vy,vz]`. The ESO in the gimbal paper
is a different animal — it's a **controller-internal observer** whose inputs
are `u` (the *previous control command*) and `y` (plant output), and whose
purpose is to estimate `z3`, the lumped disturbance *as seen by that specific
control channel's model* (parameterized by `b0`). It has no concept of
"measurement noise" in the EKF sense; it's part of the ADRC loop.

Putting it in `estimator.py` would mean the State Estimator now needs to know
about `b0` (a control-effectiveness parameter) and the previous commanded
wrench — both Guidance concerns. That's exactly the kind of cross-domain
coupling ADR-003 was written to prevent, and it's the kind of thing a future
reviewer would flag as "FDI does not command engines, the Allocator does not
make mission decisions" — by the same logic, the State Estimator should not
own control-law internals.

**Recommendation:** ESO lives inside the Guidance module (`controller.py` or
a new `src/guidance/adrc.py`), instantiated per-axis, fed by:
- `y` = the *already-filtered* EKF position/velocity output (reuse, don't
duplicate, the Error-State EKF's position/velocity output — this also avoids
redundant filtering of the same signal by two different observers, which
would have its own lag/interaction dynamics worth avoiding).
- `u` = the previous tick's WSEF+NN output for that axis.

This keeps `estimator.py` scoped to ADR-030 (Error-State EKF with Mahony attitude estimation), and keeps the
new ADRC machinery entirely inside Guidance, where ADR-018/019 already live.

### 3.2 "EKF" — is it actually extended, and is it needed at all?

AEGIS has adopted an Error-State Extended Kalman Filter (EKF) with Mahony attitude estimation (ADR-030). This estimator estimates a 12-dimensional error state comprising position, velocity, gyroscope bias, and accelerometer bias. Attitude is estimated externally by a Mahony complementary filter that fuses gyroscope and accelerometer data, consuming bias-corrected gyroscope rates from the EKF. The EKF predicts using gyroscope and accelerometer measurements (corrected for estimated biases) and updates using altimeter and velocimeter measurements. Gravity is modeled dynamically using kRPC's `vessel.flight().g_force` (or computed from `body.gravitational_parameter` and altitude). The EKF is frame-safe: body-frame specific force is rotated to the world frame using the Mahony-attitude quaternion before gravity addition.

This design avoids the need to estimate attitude within the EKF, keeping the filter linear in the error state and eliminating the small-angle approximation (ADR-014) limitation, which is superseded by ADR-030. The EKF extends the standard Kalman filter by incorporating bias states and using complementary filter for attitude, providing robustness to sensor drift and large maneuvers.

### 3.3 Inertia tensor — a new, currently-unsourced dependency

Both the quaternion paper's feedforward terms (`J·ω̇_c`, `Ω(ω)Jω`) and the
NN-ADRC integration's "scaled by the vehicle's mass **and inertia tensor**"
(plan section 4) require a 3×3 (or at minimum a 3-vector of principal
moments) inertia tensor `J`. **AEGIS currently has no module that queries,
owns, or passes around `J` anywhere** — `controller.py`'s `torque_body` is
computed as a raw PD output in N·m with no inertia scaling at all, which is
part of why `GUIDANCE_KP_ATT`/`KD_ATT` are dimensionally awkward "tune by
feel" parameters (and why ADR-026 needs Optuna over 17 params instead of a
principled starting point).

`vessel.moment_of_inertia` is available via kRPC as clean telemetry (same
category as `vessel.mass`, which ISS-006 already flags as a clean-telemetry
dependency the project is uneasy about). Adding `J` is the same category of
dependency — it should be acquired the same way mass is, and documented with
the same caveat ISS-006 already articulates, rather than introduced as a
silent new assumption.

**Recommendation:** a small, explicit ADR: "Mission Director queries
`vessel.moment_of_inertia` once at startup (and optionally re-polls if fuel
burn meaningfully changes it — worth a sentence on whether MOI drift over a
descent burn is significant enough to matter), passes `J` to
`GuidanceController` at construction or per-tick alongside `mass`." This
single ADR unblocks both the quaternion feedforward terms (which are valuable
*independent of NN-ADRC* — see Phase 1 below) and the NN-ADRC wrench scaling.

### 3.4 FDI/ADRC interface — must go through the Mission Director

Plan section 5 proposes FDI monitor `z3` (ESO disturbance estimate) and the
NN's `Δr̈` output instead of raw `expected_accel`/`measured_accel`. Per §3.1,
`z3` and `Δr̈` are now Guidance-internal. For FDI to use them without violating
the "no direct cross-module state access" rule, `GuidanceController.compute_wrench()`
needs to return a second value — a typed diagnostics object (e.g., a small
dataclass `AdrcDiagnostics` with `z3: np.ndarray` shape `(6,)` and
`nn_output: np.ndarray` shape `(6,)`) — which `main.py` then passes to FDI
explicitly, the same way it currently assembles `TelemetryFrame` from multiple
modules' outputs (ADR-013's pattern). This is mechanically simple but **must
be specified before implementation**, or the natural path of least resistance
for whoever writes the code is "FDI just imports the Guidance instance and
reads `.eso_z3` directly" — which is exactly the drift ADR-003 exists to
prevent.

A second, more substantive issue: ISS-010's fix (FDI must hold last-known-good
`expected_accel` during dt spikes, and must not fire during zero-throttle
coasting) was hard-won and specific to the *raw acceleration* signal's
behavior during those transients. `z3` and `Δr̈` will have **different**
transient behavior during dt spikes and zero-throttle phases (the ESO's `e =
z1 - y` term depends on the KF's `y`, which itself may be frozen during
`skip_predict`). Whoever adapts FDI needs to re-derive the "skip during dt
spike / skip during zero-throttle" guards for these new signals — they don't
automatically inherit ISS-010's fix just because the surrounding control flow
looks similar.

---

## 4. Numerical & Safety Risk Register

Using the project's `OPEN_ISSUES.md` severity scale, framed as risks this plan
would introduce if implemented as written (not as a verdict on existing code):

**🔴 CRITICAL — Discrete-failure / continuous-disturbance training mismatch.**
The NN's training data (per plan §6, Gremlin forcing `thrust_limit=0`) is a
step function. The gimbal paper's NN was trained and validated only on
continuous sinusoidal-disturbance scenarios. There is no evidence in the cited
literature that this NN architecture (2 hidden layers × 20 neurons,
`poslin`, offline Levenberg-Marquardt) generalizes to step discontinuities in
its training target. A practical risk: the NN learns to "average over" the
transient, producing a `Δr̈` that is too smooth/slow exactly when a fast
response matters most (an engine just died). **Mitigation:** before training
on Gremlin data, do a quick offline check — does the NN's training-target
signal (`CTM_accel - plant_accel`) actually contain the step discontinuity in
a form the network can represent, or does the ESO's own lag smooth it away
before it reaches the NN's training set? This determines whether NN-ADRC
*helps* or *delays* fault response relative to current FDI.

**🔴 CRITICAL — ESO/NN sequencing vs. ISS-001 and ISS-003.**
The ESO's `e = z1 - y` term and the NN's position/velocity/acceleration inputs
both depend on the EKF's output quality. ISS-003 (Q/R are still
identity placeholders) and ISS-001 (FDI threshold uncalibrated) are both
CRITICAL and OPEN. Building and tuning ESO β/α/δ parameters — and especially
collecting NN training data — against an uncalibrated filter means **all of
that tuning is invalidated** the moment ISS-003 is resolved (the filter's
noise characteristics change, so `z1`'s relationship to true position changes,
so the ESO's `e` signal changes, so β/α/δ need re-tuning, so NN training data
needs regenerating). **Recommendation:** ISS-001 and ISS-003 must close before
any ESO/NN tuning work starts, or the NN-ADRC work should explicitly branch
and accept it will be redone.

**🟡 MAJOR — `fal()` and `b0` edge cases.**
The transcribed `fal()` is mathematically correct (continuous at `|e|=δ`,
verified by hand), but `δ=0` causes `δ**(1-α)` to raise `ZeroDivisionError`
(for `α<1`) or be `0**0=1` (for `α=1`) — needs an explicit `δ>0` guard with a
clear error, not a silent NaN. Separately, `b0` for **attitude** axes is
`≈1/I_axis` (inertia-dependent, see §3.3) while `b0` for **translation** axes
is `≈1/mass` — these are fundamentally different quantities computed from
different telemetry. The plan's single symbolic `b0` per axis needs to be six
*concretely different* derivations, each documented with its data source and
the ISS-006-style clean-telemetry caveat where relevant.

**🟡 MAJOR — NN output bounding / fallback.**
Per ADR-002's philosophy (a runtime TypeError during engine-out is fatal,
mitigated by strict typing), an untrained-region NN producing a `NaN` or
wildly out-of-range `Δr̈` is the numerical equivalent of that failure mode —
and it's *more* likely here because the NN will see live-flight states that
differ from its training distribution (especially during HARD_ABORT-adjacent
conditions, which is exactly when you'd want it to behave conservatively, not
erratically). **Recommendation:** `Δr̈` must be clamped to a physically
plausible range before being added to the WSEF output, and there should be a
documented fallback (pure ADRC, NN contribution = 0) if the clamp triggers
repeatedly — analogous to how `AllocationDegenerateError` gives the Allocator
a documented failure mode rather than a silent bad value (ADR-010/ISS-002).

**🔵 MINOR — Telemetry schema growth.**
6 axes × (`z1,z2,z3` + NN output + `b0`) is ~24-30 new per-tick values. Per
ADR-013, the 50Hz CSV is already dense (~180k rows/hour at 1Hz... actually
50Hz, so far more). Recommend these go behind the existing `DEBUG_LOGGING`
toggle or into `events.jsonl` at reduced rate, not unconditionally into
`telemetry.csv`.

**⚪ DEFERRED (flag, don't block) — Hyperparameter explosion vs. ADR-026.**
Per-axis ESO has 6 tunable parameters (`β01,β02,β03,α1,α2,δ`) plus `b0` —
even with shared `α1=0.5,α2=0.25` (per the paper's stated defaults, which the
plan also adopts), that's 5 new tunables × up to 6 axes = 30, on top of
ADR-026's existing 17-parameter Optuna search. This isn't blocking for a
phased rollout (Phase 2 below scopes to 3 axes with shared parameters), but
the human should be aware the eventual full search space is large, and Optuna
convergence time will grow accordingly.

---

## 5. A Convention Landmine to Resolve Before Writing Any Quaternion Code

`controller.py` line 58 documents `current_attitude` as `[w, x, y, z]`
scalar-first, but line 100 calls `R.from_quat(current_attitude)` — **scipy's
`Rotation.from_quat` expects scalar-**last** `[x, y, z, w]`** by default. I
don't have `sensors.py` in front of me, so I can't confirm whether this is a
real bug (if `attitude` actually arrives scalar-first from kRPC/`sensors.py`,
this call is silently wrong) or just a stale docstring (if `attitude` arrives
scalar-last, as kRPC's Unity-derived convention typically provides, and the
docstring is simply mislabeled).

This matters *now* because plan section 3's `calculate_attitude_error` adds
**new** `quaternion_inverse`/`quaternion_multiply` helper functions, which
must use a convention consistent with whatever `current_attitude` actually is.
If the existing convention is ambiguous or wrong, the new ADRC attitude-error
term and the NN's attitude-derived input features will both inherit that
ambiguity, and any bug will be much harder to isolate once it's buried inside
an NN's training data.

**Recommendation:** before Phase 1 (below) starts, add a small unit test that
takes a known rotation (e.g., 90° about Z) from `sensors.py`'s actual output
format, round-trips it through `R.from_quat`/`.as_quat()`, and asserts the
expected Euler angles — pinning the convention in code, not just in a
docstring. This is a 20-minute task that prevents a very hard-to-debug class
of error later.

---

## 6. Recommended Phased Roadmap

| Phase | Scope | Why this order | Risk if skipped/reordered |
|---|---|---|---|
| **0** | Close ISS-001 (FDI threshold calibration) and ISS-003 (Q/R tuning). Add the convention unit test from §5. | Everything downstream depends on filter/FDI noise characteristics being real, not placeholders. | All ESO/NN tuning in later phases is invalidated and must be redone (§4, CRITICAL risk). |
| **1** | Quaternion control law upgrade: add inertia tensor `J` (new small ADR, §3.3), add `Ω(ω)Jω` gyroscopic feedforward and `J·(Kp·e+Kd·ė)` inertia-scaled torque to `controller.py`. Re-derive `GUIDANCE_KP_ATT`/`KD_ATT` via Eq. 37 (`d=2ζωₙ`, `k=ωₙ²`) as Optuna starting points instead of raw search. | Independently valuable (improves the existing PD attitude controller), low risk, fully grounded in the quaternion paper, and **delivers the inertia tensor dependency** Phase 3 needs anyway. No NN, no ESO — small, reviewable diff. | None — this phase has value even if NN-ADRC is later deprioritized. |
| **2** | ESO + WSEF (no NN yet — "legacy ADRC" per the gimbal paper) for **attitude axes only** (3-DOF), implemented in `src/guidance/adrc.py`, fed by existing KF output. Shared `α1=0.5, α2=0.25` across axes; per-axis `β01,β02,β03,δ,b0`. Validate against the current PD attitude controller using the two-tier test harness (extend the kinematic mock per ADR-012). | Matches the gimbal paper's actual validated scope (2-3 DOF). Translation axes keep the already-tuned glide-slope PD (ADR-022) — don't destabilize what was just fixed in ISS-011. | Jumping straight to 6-DOF ADRC risks destabilizing the just-resolved glide-slope behavior with an unvalidated new translation controller simultaneously. |
| **3** | CTM-based compensation (the paper's *other* hybrid, "CTM-ADRC") as a cheaper alternative/baseline to NN — reuse FDI's existing `expected_force/mass` calculation as the CTM (§2 mapping table: CTM ≈ FDI's expected_accel). Per the paper, CTM-ADRC gets ~41% of NN-ADRC's improvement with far less infrastructure. | Establishes whether the *simple* version is "good enough" before committing to the NN pipeline (training data collection, model lifecycle, bounding/fallback per §4). Directly reuses existing code rather than duplicating it. | Without this baseline, there's no way to tell whether NN-ADRC's added complexity (Phase 4) is worth its cost. |
| **4** | NN training pipeline: data collection (likely requires extending the kinematic mock for bulk synthetic runs, since live-KSP data collection at the volumes NN training needs is slow — flag for ADR-012 extension), offline training, numpy-only inference at runtime (no heavy ML runtime in the 20ms loop), output clamping + fallback per §4. | Highest complexity, highest payoff per the paper (85.4% MTE reduction vs 40.8% for CTM-ADRC) — but only worth doing once Phases 0-3 establish it's needed and the discrete-failure training-data question (§4, first CRITICAL) is answered. | — |
| **5** | FDI adaptation: `AdrcDiagnostics` dataclass returned from `compute_wrench`, routed through `main.py` to FDI (§3.4). Re-derive ISS-010-style dt-spike/zero-throttle guards for `z3`/`Δr̈` signals. New ISS entry for calibrating the new fault thresholds against these signals (ISS-001 analog). | Depends on Phases 2-4 existing to have signals to monitor. | — |
| **6 (deferred)** | Translation-axis ADRC/NN (full 6-DOF), and/or EKF + attitude-in-state-vector (§3.2 option C) if Phase 1-5 experience shows the need to revisit attitude estimation (now using Mahony filter per ADR-030). | Only revisit ADR-014 if there's evidence it's needed — don't reopen a settled, well-reasoned decision speculatively (superseded by ADR-030). | — |

---

## 7. Suggested New ADR / ISS Entries (drafts for human consideration)

- **ADR-027 — ESO/ADRC ownership and Guidance/Estimator boundary.** Formalizes
  §3.1: ESO lives in Guidance, fed by EKF position/velocity output; State Estimator
  remains scoped to ADR-030 (Error-State EKF with Mahony attitude estimation).
- **ADR-028 — Vessel inertia tensor sourcing.** Formalizes §3.3: `J` queried
  via `vessel.moment_of_inertia`, owned/passed like `mass`, with an
  ISS-006-style caveat.
- **ADR-029 — KF state vector scope for NN-ADRC inputs (Option A vs 9-state vs
  EKF).** Formalizes §3.2: explicitly decide whether "EKF" in the plan is a
  misnomer for the current linear KF, and where the NN's acceleration input
  comes from if not from a new filter state.
- **ISS-012 — `fal()` δ=0 guard and per-axis `b0` derivation.** §4, MAJOR.
- **ISS-013 — NN output bounding and ADRC fallback mode.** §4, MAJOR.
- **ISS-014 — FDI/ADRC diagnostic interface and re-derived dt-spike guards.**
  §3.4, depends on ADR-027.
- **ISS-015 — Quaternion convention verification (`current_attitude` format
  vs `R.from_quat`).** §5 — small, but should be closed before Phase 1.

---

## 8. Open Questions for the Human

- Is the project's appetite for this work scoped to "Phase 1 only" (the
  quaternion/inertia upgrade, which stands alone and is low-risk), or is full
  NN-ADRC (Phases 2-5) an active goal for the current milestone? The answer
  changes how much of this advisory is "do now" vs "park in OPEN_ISSUES.md for
  later."
- For Phase 4 (NN training data), is live-KSP run time available at the volume
  NN training typically needs (the gimbal paper doesn't state run counts, but
  offline NN training generally wants order-100s of trajectories)? If not,
  the kinematic-mock extension becomes a hard prerequisite, not a nice-to-have
  — worth deciding before Phase 2 so the mock's fidelity requirements are
  scoped correctly from the start.
- §5's quaternion convention question needs someone with `sensors.py` open
  and a live (or replayed) telemetry sample to resolve definitively — I
  flagged it from the docstring/scipy mismatch alone and can't confirm which
  side is wrong without that file.
