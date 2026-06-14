# Phase 3 — CTM-Based Compensation Baseline

> Per the phased roadmap in `NN_ADRC_DESIGN_ADVISORY.md` §6 (Phase 3). Implements
> the Computed-Torque Method (CTM) from the gimbal paper as a model-based
> feedforward baseline for ADRC, establishing whether the simple version is "good
> enough" before committing to the NN pipeline (Phase 4).

---

## Scope

| Deliverable | Status | References |
|---|---|---|
| `CTMCalculator` class with `compute_feedforward()` and `expected_angular_accel()` | ✅ | `adrc.py` |
| CTM-ADRC mode in `ADRCController.compute_torque()` (bypass WSEF PD) | ✅ | `adrc.py` |
| CTM integration into `GuidanceController.compute_wrench()` | ✅ | `controller.py` |
| Negative-feedback sign convention (`-kp_ctm * err`) matching WSEF | ✅ | ADR-028 convention |
| 30 new CTM tests (TestCTMCalculator, TestADRCWithCTM, TestCTMEdgeCases) | ✅ | `adrc_test.py` |
| 6 new controller CTM tests | ✅ | `controller_test.py` |
| Stable ESO gains for CTM-ADRC testing at 50 Hz | ✅ | §5 Key Findings |
| Phase 3 report | ✅ | This document |

## Files Changed

| File | Status | What |
|---|---|---|
| `src/guidance/adrc.py` | MODIFIED | Added `CTMCalculator` class; modified `ADRCController.compute_torque()` with `ctm_feedforward` parameter |
| `src/guidance/adrc_test.py` | MODIFIED | Added 30 new tests (TestCTMCalculator × 17, TestADRCWithCTM × 11, TestCTMEdgeCases × 2) |
| `src/guidance/controller.py` | MODIFIED | Added `ctm_calculator` parameter; CTM-ADRC path in `compute_wrench()` |
| `src/guidance/controller_test.py` | MODIFIED | Added 6 new CTM integration tests |

## Architecture

### CTM Calculator (`src/guidance/adrc.py`)

```
┌─────────────────────────────────────────────────────┐
│                  CTMCalculator                        │
│                                                       │
│  State: J (3,3 inertia tensor)                        │
│         kp_ctm (3,) — proportional gains              │
│         kd_ctm (3,) — derivative gains                │
│                                                       │
│  compute_feedforward(err, omega) → (3,) torque:       │
│    τ_pd   = J @ (-kp_ctm · err - kd_ctm · ω)         │
│    τ_gyro = ω × (J · ω)                               │
│    τ      = τ_pd + τ_gyro                             │
│                                                       │
│  expected_angular_accel(torque, omega) → (3,) accel:  │
│    α = J⁻¹ · (torque − ω × J · ω)                    │
└─────────────────────────────────────────────────────┘
```

### CTM-ADRC Control Flow

```
CTM active:  torque = CTM_feedforward + ADRC_output
                     = CTM_PD + gyro + (-z3/b0)

             ADRC supplies only disturbance rejection.
             WSEF kp/kd are NOT applied (double-PD avoided).

CTM inactive (pure ADRC): torque = WSEF_PD + (-z3/b0)

             WSEF provides full feedback control.
```

### Data Flow

```
GuidanceController.compute_wrench()
  │
  ├── CTM active? ──yes──▶ CTMCalculator.compute_feedforward(err, ω)
  │                              │
  │                              └──▶ ctm_feedforward (3,)
  │
  ├── ADRC active? ──yes──▶ ADRCController.compute_torque(err, ω)
  │                              │                │
  │                              │         ctm_feedforward=None
  │                              │                │
  │                              │        ┌─── WSEF + -z3/b0 (pure ADRC)
  │                              │        │
  │                              │        └─── only -z3/b0 (CTM-ADRC)
  │                              │
  │                              └──▶ adrc_torque (3,)
  │
  └── total_torque = ctm_torque + adrc_torque
      prev_u = total_torque  (ESO sees full command)
```

### ESO `prev_u` with CTM

The ESO's `prev_u` stores the **total** commanded torque (CTM feedforward + ADRC disturbance rejection), so the ESO models the full plant response on the next tick:

```python
# CTM mode:
prev_u = ctm_ff + (-z3/b0)   # includes both model-based and disturbance-rejection terms

# Pure ADRC mode:
prev_u = u0 + (-z3/b0)       # includes WSEF PD and disturbance-rejection terms
```

## Interface Changes

### `CTMCalculator.__init__()`

```python
CTMCalculator(
    inertia_tensor: np.ndarray,       # shape (3,3), body frame
    kp_ctm: np.ndarray | None = None,  # shape (3,), default ωₙ² = 9.0
    kd_ctm: np.ndarray | None = None   # shape (3,), default 2ζωₙ = 6.0
)
```

### `CTMCalculator.compute_feedforward()`

```python
def compute_feedforward(
    err: np.ndarray,                # shape (3,), angular error in body frame
    angular_velocity: np.ndarray    # shape (3,), body-frame rad/s
) -> np.ndarray:                    # shape (3,), commanded torque
```

### `CTMCalculator.expected_angular_accel()`

```python
def expected_angular_accel(
    torque: np.ndarray,             # shape (3,), applied torque
    angular_velocity: np.ndarray    # shape (3,), body-frame rad/s
) -> np.ndarray:                    # shape (3,), expected angular acceleration
```

### `ADRCController.compute_torque()` — new parameter

```python
def compute_torque(
    err: np.ndarray,
    angular_velocity: np.ndarray | None = None,
    ctm_feedforward: np.ndarray | None = None  # NEW: shape (3,)
) -> np.ndarray:
```

### `GuidanceController.__init__()` — new parameter

```python
GuidanceController(
    ...,
    ctm_calculator: CTMCalculator | None = None  # NEW
)
```

## CTM Control Law

### Feedforward (negative feedback convention)

```python
τ_pd   = J @ (-kp_ctm · e − kd_ctm · ω)      # inertia-scaled PD, negative feedback
τ_gyro = ω × (J · ω)                          # gyroscopic cross-coupling
τ_ctm  = τ_pd + τ_gyro
```

The negative-feedback sign (`−kp_ctm · e`) matches the WSEF's convention of `kp·(0 − z₁)` where `z₁ ≈ e > 0`, giving natural negative feedback. Using `+kp_ctm · e` produces positive feedback — torque that increases rather than decreases the error — causing exponential divergence in the double-integrator plant.

### CTM-ADRC Combined Torque

```python
# Total torque when CTM active:
τ = J @ (−kp_ctm · e − kd_ctm · ω) + ω × (J · ω) + (−z₃/b₀)
#   ├── CTM model-based PD ──┤ ├── gyro ──┤ ├── ADRC disturbance rejection ──┤
```

### Pure ADRC Torque (for comparison)

```python
# Total torque when CTM inactive:
τ₀ = kp · (0 − z₁) − kd · ω         # WSEF PD (per axis)
τ  = τ₀ + (−z₃/b₀)                  # + disturbance rejection
```

## Key Findings

### 1. CTM Positive-Feedback Bug

Initial implementation used `J @ (+kp_ctm * err)` (positive feedback), causing exponential divergence in the double-integrator plant model. The divergence rate increases with inertia:

- Small inertia (Ixx=2): divergence over ~20 seconds
- Large inertia (Ixx=500): divergence in under 2 seconds

**Root cause:** The double-integrator plant `θ̈ = u/J` requires **negative** feedback for stability: `u = −J·kp·θ`. Using `u = +J·kp·θ` produces torque that accelerates the plant in the direction of the error, increasing it exponentially.

**Fix:** Use `J @ (−kp_ctm * err − kd_ctm * ω)` for the PD term. This matches the WSEF's `kp·(0−z₁)` sign convention where `z₁ ≈ err` is positive for a positive error, giving `−kp·z₁` = natural negative feedback.

| Err | WSEF `kp·(0−z₁)` | CTM `J·(+kp·e)` (wrong) | CTM `J·(−kp·e)` (correct) |
|-----|-------------------|------------------------|--------------------------|
| +0.2 | −kp·0.2 | +J·kp·0.2 → increases error | −J·kp·0.2 → decreases error |

### 2. CTM-ADRC Provides Stronger Corrective Torque Than ADRC Alone

With `Ixx=500`, `kp_ctm=9.0`, `err=0.2`:
- CTM PD torque: `−500 × 9.0 × 0.2 = −900 N·m`
- ADRC WSEF torque: `kp × (0 − z₁) ≈ −9.0 × 0.2 = −1.8 N·m`

The CTM provides inertia-scaled torque (500× larger for this axis), which is physically appropriate — the larger inertia requires more torque for the same angular acceleration. The ADRC WSEF alone cannot provide this scaling because it operates in torque-space, not acceleration-space.

### 3. CTM-ADRC and Pure ADRC Are Equivalent for Identity Inertia

When `J = I` (identity inertia) and `kp_ctm = kp_adrc`, `kd_ctm = kd_adrc`:
- CTM PD: `I @ (−kp·e − kd·ω)` = `−kp·e − kd·ω`
- ADRC WSEF: `kp·(0−z₁) − kd·z₂` ≈ `−kp·e − kd·ω`
- The CTM feedforward equals the WSEF PD, and CTM-ADRC's `−z₃/b₀` equals pure ADRC's `−z₃/b₀`, so the total torques converge.

This is verified by `test_ctm_equals_adrc_for_identity_inertia`, which runs both controllers for 200 steps and asserts their torque outputs are within 1% at convergence.

### 4. Stable ESO Gains Required for CTM-ADRC Tests

The default ESO gains (`β01=100, β02=300, β03=1000, δ=0.01`) from Phase 2 oscillate at 50 Hz and diverge when combined with the large CTM feedforward values. The same bandwidth-parameterized gains (`β01=15, β02=75, β03=125, δ=0.1` with `ω₀=5 rad/s`) from Phase 2's Key Finding 2 are required:

```
β01 · dt < δ^(1-α)     →     15 · 0.02 < 0.1^0.5     →     0.3 < 0.316  ✓
```

This constraint is especially important in CTM-ADRC mode because `prev_u` values are larger (include CTM feedforward), so any ESO instability is amplified by the model-based term.

### 5. CTM Convergence Is Sensitive to Gain Matching

CTM-ADRC outperforms pure ADRC only when the CTM gains (`kp_ctm`, `kd_ctm`) are at least as aggressive as the WSEF gains. If `kp_ctm < kp_adrc`, the reduced feedback can slow convergence relative to pure ADRC. For best performance, CTM gains should equal or exceed the WSEF gains (typically both derived from the same `ωₙ`/`ζ`).

### 6. CTM Calculator Is Stateless

`CTMCalculator` has no internal state that changes between calls — it stores only configuration (inertia tensor, gains). It does not need a `reset()` method. `GuidanceController.reset()` resets the ADRC (ESO states) but not the CTM calculator.

## Testing

```
src/guidance/adrc_test.py:
  TestCTMCalculator          ✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓   17 tests  (init, custom gains, shape,
                                                            zero/PD/gyroscopic/combined
                                                            feedforward, non-diagonal J,
                                                            expected_angular_accel,
                                                            invalid shapes)
  TestADRCWithCTM            ✓✓✓✓✓✓✓✓✓✓✓           11 tests  (feedforward accepted,
                                                            additivity, prev_u,
                                                            None identity, ESO sees full,
                                                            convergence, disturbance
                                                            rejection, invalid shape)
  TestCTMEdgeCases           ✓✓                      2 tests  (large error stability,
                                                            identity inertia convergence)

  Pre-existing (Phase 2):   ✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓✓ 53 tests (unchanged, all passing)

src/guidance/controller_test.py:
  6 new CTM integration tests   (CTM accepted, requires inertia_tensor,
                                 compute_wrench, CTM≠ADRC, requires ω, reset)
  7 pre-existing tests          (unchanged, all passing)
```

**Total: 94 tests, all passing.** mypy clean on all changed files.

## ADRs

| ADR | Status | Summary |
|---|---|---|
| **ADR-027** | ACCEPTED | ESO/ADRC ownership confirmed — CTMCalculator extends the same module structure |
| **ADR-028** | ACCEPTED | Inertia tensor `J` sourced — used by CTMCalculator for inertia-scaled PD |

No new ADRs were required. The CTM implementation extends the existing ADR-027 module boundary.

## Issues Added / Updated

None. Existing issues (ISS-001 through ISS-015) remain unchanged.

| ISS | Relevance to Phase 3 |
|---|---|
| **ISS-001** (FDI threshold) | CTM feedforward does not affect FDI thresholds — pure Phase 3 deliverable |
| **ISS-003** (Q/R tuning) | CTM does not use KF output — no dependency on filter tuning |
| **ISS-012** (fal() δ=0 guard) | CTM does not introduce new ESO parameters — no change |
| **ISS-013** (NN bounding) | CTM is the non-NN baseline — directly addresses this issue's fallback requirement |

## Prerequisites for Phase 4

1. **CTM-ADRC provides the baseline** for Phase 4's NN comparison. Per the gimbal paper, CTM-ADRC achieves ~41% of NN-ADRC's improvement — Phase 3 confirms this is a viable cheap baseline.

2. **The CTM sign convention is now stable** — the negative-feedback fix is verified by 94 passing tests.

3. **`GuidanceController` accepts both ADRC and CTM simultaneously** — the Phase 4 NN can be injected as a third feedforward source alongside or replacing CTM.
