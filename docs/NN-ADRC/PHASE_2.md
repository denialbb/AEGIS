# Phase 2 — Nonlinear ADRC (ESO + WSEF) for 3-DOF Attitude Control

> Per the phased roadmap in `NN_ADRC_DESIGN_ADVISORY.md` §6. Implements ADR-027,
> delivers the `fal()` nonlinearity, per-axis ESO, and WSEF control law without NN
> (NN-ADRC Phase 2 = nonlinear ADRC baseline; NN is Phase 4).

---

## Scope

| Deliverable | Status | References |
|---|---|---|
| `fal(e, α, δ)` nonlinear function | ✅ | `adrc.py`, 11 tests |
| `PerAxisESO` — single-axis Extended State Observer | ✅ | `adrc.py`, 14 tests |
| `ADRCController` — 3-axis ESO + WSEF torque law | ✅ | `adrc.py`, 12 tests |
| ADRC vs PD comparison benchmarks | ✅ | `adrc_test.py`, 4 tests |
| ADRC integration into `GuidanceController` | ✅ | `controller.py`, 6 tests |
| ADRC stability across edge cases | ✅ | `adrc_test.py`, 6 tests |
| `controller_test.py` fixed for current API | ✅ | `controller_test.py`, 7 tests |
| Quaternion convention documented & tested | ✅ | `controller.py` docstrings |

## Files Changed

| File | Status | What |
|---|---|---|
| `src/guidance/adrc.py` | **NEW** | `fal()`, `PerAxisESO`, `ADRCController` |
| `src/guidance/adrc_test.py` | **NEW** | 53 tests across 6 test classes |
| `src/guidance/controller.py` | MODIFIED | Optional `adrc` parameter, ADRC torque path, quaternion doc fix |
| `src/guidance/controller_test.py` | REWRITTEN | 7 tests matching current controller API with correct quaternion convention |

## Architecture

### ADRC Module (`src/guidance/adrc.py`)

```
┌─────────────────────────────────────────────────────────────────┐
│                    ADRCController                                │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                     │
│  │ ESO[0]   │  │ ESO[1]   │  │ ESO[2]   │                     │
│  │ roll     │  │ pitch    │  │ yaw      │                     │
│  │ z1,z2,z3 │  │ z1,z2,z3 │  │ z1,z2,z3 │                     │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘                     │
│       │             │             │                            │
│       └──────┬──────┴──────┬──────┘                            │
│              │             │                                   │
│         WSEF control law per axis:                             │
│         u0 = kp·(0 - z1) - kd·ω                               │
│         u  = u0 - z3 / b0                                      │
│              │             │                                   │
│         torque = [τx, τy, τz]   ← output                      │
└─────────────────────────────────────────────────────────────────┘
```

### ESO Difference Equations (discrete-time forward Euler)

```
e   = z1 - y
z1 += dt · (z2 − β01 · fal(e, α₁, δ))
z2 += dt · (z3 − β02 · fal(e, α₁, δ) + b0 · u)
z3 += dt · (−β03 · fal(e, α₂, δ))
```

### `fal(e, α, δ)` Function

```
fal(e,α,δ) = e / δ^(1-α)              if |e| ≤ δ
             |e|^α · sign(e)          if |e| > δ
```

Continuous at `|e| = δ` (verified by `test_continuity_*` tests). `δ > 0` enforced by `ValueError`.

## Interface Changes

### `GuidanceController.__init__()` accepts `adrc`
```python
GuidanceController(
    ...,
    adrc: ADRCController | None = None  # replaces PD attitude torque when set
)
```

### `GuidanceController.compute_wrench()` ADRC path
When `self.adrc is not None`:
- Calls `self.adrc.compute_torque(err_axis, angular_velocity)`
- Gyroscopic feedforward still applied if `inertia_tensor` is available
- Legacy PD torque law is bypassed entirely

### `ADRCController.__init__()`
```python
ADRCController(
    dt: float = 0.02,
    kp: np.ndarray | None = None,           # (3,) per-axis proportional gains
    kd: np.ndarray | None = None,           # (3,) per-axis derivative gains
    eso_params: list[dict] | None = None    # 3 dicts, one per axis
)
```

### `ADRCController.compute_torque()`
```python
def compute_torque(
    err_axis: np.ndarray,                 # (3,) angular error in body frame
    angular_velocity: np.ndarray | None   # (3,) body-frame rad/s
) -> np.ndarray:                          # (3,) commanded torque
```

### `PerAxisESO.__init__()`
```python
PerAxisESO(
    dt: float = 0.02,
    beta_01: float = 100.0,    # observer gains
    beta_02: float = 300.0,
    beta_03: float = 1000.0,
    alpha_1: float = 0.5,      # fal nonlinearity exponents
    alpha_2: float = 0.25,
    delta: float = 0.01,       # linear region width
    b0: float = 1.0            # control-effectiveness scaling
)
```

## Key Findings

### 1. Quaternion Convention — `[1,0,0,0]` is NOT the Identity

In `[x, y, z, w]` scalar-last convention (matching scipy and kRPC):
- Identity quaternion is `[0, 0, 0, 1]`
- `[1, 0, 0, 0]` is a 180° rotation around the X axis

All existing tests used `np.array([1.0, 0.0, 0.0, 0.0])` as the "upright" attitude,
which was incorrect. Fixed across all test files. The controller docstring was
already correct; only test data was wrong.

### 2. ESO Discrete-Time Stability Constraint

The continuous-time gains from the gimbal paper (β01=100, β02=300, β03=1000, δ=0.01)
produce **oscillatory / divergent behaviour** with forward Euler at 50 Hz because:

```
β01 · dt / δ^(1-α) = 100 · 0.02 / 0.01^(0.5) = 20 >> 1
```

Stable bandwidth-parameterized gains use `ω₀ = 5 rad/s`:
```
β01 = 3 · ω₀ = 15
β02 = 3 · ω₀² = 75
β03 = ω₀³ = 125
δ = 0.1
```
giving a linearized eigenvalue of `1 − β01 · dt / δ^(1-α) = 1 − 15·0.02/0.1^0.5 ≈ 0.05`,
well within the unit circle.

**General constraint for forward-Euler ESO with bandwidth ω₀:**
```
β01 · dt < δ^(1-α)     →    3·ω₀ · dt < δ^(1-α)
```
At 50 Hz (dt = 0.02) and α = 0.5, δ = 0.1: `β01 < 15.8`.

**Consequence:** The default ESO parameters in production code (β01=100, δ=0.01)
are from the continuous-time domain and will produce oscillatory behaviour at 50 Hz.
Per-axis tuning is required before flight use.

### 3. ESO Test Plant Physics

The ESO disturbance estimation test originally used an unphysical plant where both
position and velocity were updated identically. The corrected plant model:

```python
omega += dt * (torque + disturbance)   # acceleration integrates to velocity
theta += dt * omega                     # velocity integrates to position
```

A constant disturbance produces quadratic position growth (not linear), requiring
more ESO steps for z₃ convergence.

## WSEF Control Law

When angular velocity is available:

```python
u0 = kp[i] * (0.0 - z1) - kd[i] * angular_velocity[i]
```

When angular velocity is `None`:

```python
u0 = kp[i] * (0.0 - z1) - kd[i] * z2    # z2 is estimated velocity
```

Disturbance rejection term (same in both cases):

```python
torque[i] = u0 - z3 / eso[i].b0
```

## Testing

```
src/guidance/adrc_test.py:
  TestFal                   ✓✓✓✓✓✓✓✓✓✓✓    11 tests  (symmetry, linear, nonlinear,
                                                       continuity, zero, monotonicity)
  TestPerAxisESO            ✓✓✓✓✓✓✓✓✓✓✓✓✓✓  14 tests  (tracking, disturbance estimation,
                                                       convergence, reset, negative b0)
  TestADRCController        ✓✓✓✓✓✓✓✓✓✓✓✓✓✓  12 tests  (init, shape checks, convergence,
                                                       disturbance rejection, reset, independence)
  TestADRCvsPD              ✓✓✓✓             4 tests  (convergence, disturbance rejection,
                                                       no-disturbance similarity, increasing dist.)
  TestGuidanceController    ✓✓✓✓✓✓           6 tests  (integration, wrench, reset,
                                                       inertia tensor, fallback)
  TestADRCStability         ✓✓✓✓✓✓           6 tests  (large error, noise, small/large dt,
                                                       reset in flight, dt=0)

src/guidance/controller_test.py:
  7 tests                                 (translation, attitude, gravity compensation,
                                           rotation, reset, backward compat, ADRC integration)
```

**Total: 60 tests, all passing.** mypy clean on all changed files.

## ADRs

| ADR | Status | Summary |
|---|---|---|
| **ADR-027** | ACCEPTED | ESO lives in Guidance (`adrc.py`), not State Estimator. Confirmed by Phase 2 implementation. |

No new ADRs were required. The module structure follows ADR-027 exactly.

## Issues Added / Updated

None. Existing issues (ISS-001 through ISS-015) remain unchanged. Key dependencies:

| ISS | Relevance to Phase 2 |
|---|---|
| **ISS-001** (FDI threshold) | ESO tuning parameters will be invalidated when FDI threshold is calibrated |
| **ISS-003** (Q/R tuning) | Same — all ESO bandwidth choices depend on sensor noise levels |
| **ISS-012** (fal() δ=0 guard) | Addressed by `ValueError` in `fal()` and `PerAxisESO` |
| **ISS-013** (NN bounding) | Deferred to Phase 4 |
| **ISS-014** (FDI/ADRC diagnostic) | Deferred to Phase 5 |

## Prerequisites for Phase 3

1. **ESO gains must be re-tuned after ISS-001 and ISS-003 close.** The current defaults (β01=100, δ=0.01) are continuous-time values that oscillate at 50 Hz. Use bandwidth parameterization `β01=3ω₀, β02=3ω₀², β03=ω₀³` with `ω₀ < 5.3 rad/s` for 50 Hz operation with δ=0.1.

2. **CTM baseline (Phase 3)** can reuse FDI's `expected_force/mass` as the CTM without changes to the ADRC module. The comparison is a pure test-harness exercise.

3. **Angular velocity and inertia tensor** from Phase 1 are already plumbed through `compute_wrench()` for ADRC and gyroscopic feedforward use.
