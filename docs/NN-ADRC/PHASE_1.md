# Phase 1 — Quaternion Control Law Upgrade

> Per the phased roadmap in `NN_ADRC_DESIGN_ADVISORY.md` §6. Implements ADR-028,
> closes ISS-015, and delivers the inertia tensor dependency that Phase 3+ needs.

---

## Scope

| Deliverable | Status | References |
|---|---|---|
| Inertia tensor `J` sourced from kRPC | ✅ | ADR-028, `main.py` |
| Gyroscopic feedforward `Ω(ω)Jω` in torque law | ✅ | `controller.py` |
| Inertia-scaled PD torque `J·(Kp·e+Kd·ė)` | ✅ | `controller.py` |
| Gains re-derived via `ωₙ²` / `2ζωₙ` | ✅ | `config.py`, `main.py` |
| Angular velocity stream from kRPC | ✅ | `sensors.py` |
| Quaternion convention verification | ✅ | ISS-015, `tests/test_quaternion.py` |
| ADR-028 / ADR-027 / ADR-029 recorded | ✅ | `DECISIONS.md` |
| ISS-012 through ISS-015 tracked | ✅ | `OPEN_ISSUES.md` |

## Files Changed

| File | What |
|---|---|
| `src/config.py` | Added `GUIDANCE_ATT_NATURAL_FREQ` (ωₙ=3.0/axis), `GUIDANCE_ATT_DAMPING_RATIO` (ζ=1.0/axis); deprecated old `GUIDANCE_KP_ATT`/`KD_ATT` |
| `src/telemetry/sensors.py` | Added `angular_velocity` stream; `poll()` now returns 7th element `angular_velocity: np.ndarray` shape `(3,)` |
| `src/guidance/controller.py` | New torque law: `τ = J·(Kp·e − Kd·ω) + ω×J·ω`. Accepts optional `inertia_tensor` and `angular_velocity`. Fixed `current_attitude` docstring to scalar-last `[x,y,z,w]` |
| `src/main.py` | Queries `vessel.inertia_tensor` at startup → 3×3 array. Computes `Kp = ωₙ²`, `Kd = 2ζωₙ`. Passes both + `angular_velocity` through to controller |
| `tests/test_controller.py` | 6 tests: init with/without J, force computation, gyroscopic term isolation, error on missing ω, error on bad J shape |
| `tests/test_quaternion.py` | 5 tests: scalar-last convention, round-trip, Euler consistency, inverse/multiply, sensor.py convention |
| `tests/test_sensors.py` | Updated mocks for `angular_velocity`, `aerodynamic_force`, `situation` streams |

## Interface Changes

### `SensorModels.poll()` returns 7 values
```python
# Before (6):
noisy_alt, noisy_accel, attitude, mass, aero_body, situation

# After (7):
noisy_alt, noisy_accel, attitude, mass, aero_body, situation, angular_velocity
```

### `GuidanceController.__init__()` accepts `inertia_tensor`
```python
GuidanceController(
    ...,
    inertia_tensor: np.ndarray | None = None  # shape (3,3), body frame
)
```

### `GuidanceController.compute_wrench()` accepts `angular_velocity`
```python
def compute_wrench(
    ...,
    angular_velocity: np.ndarray | None = None  # shape (3,), body frame rad/s
) -> np.ndarray:
```

## Torque Law

When `inertia_tensor` is provided:

```python
τ_pd   = J @ (Kp · e − Kd · ω)      # inertia-scaled PD
τ_gyro = ω × (J · ω)                 # gyroscopic cross-coupling
τ      = τ_pd + τ_gyro
```

where gains are derived from natural frequency and damping ratio (per Elbeltagy et al. Eq. 37):
- `Kp = ωₙ²`  [rad²/s²]
- `Kd = 2ζωₙ` [rad/s]

When `inertia_tensor` is `None`, falls back to the legacy direct PD:
```python
τ = Kp · e − Kd · ė    # no inertia scaling, no gyroscopic term
```

## Testing

All pre-existing test failures (allocator gimbal angles, estimator numeric tolerances, FDI persistence, telemetry `fuel_state`) are unrelated to Phase 1.

```
tests/test_controller.py   ......  6 passed
tests/test_quaternion.py   .....   5 passed
tests/test_sensors.py      .       1 passed
```

## ADRs Added

| ADR | Status | Summary |
|---|---|---|
| **ADR-027** | DEFERRED | ESO lives in Guidance, not State Estimator (for Phase 2+) |
| **ADR-028** | ACCEPTED | Inertia tensor from `vessel.inertia_tensor`, passed like `mass` |
| **ADR-029** | DEFERRED | KF state vector stays 6-state Option A (for Phase 2+) |

## Issues Added

| ISS | Severity | Summary |
|---|---|---|
| **ISS-012** | 🟡 MAJOR | `fal()` δ=0 guard and per-axis `b0` derivation |
| **ISS-013** | 🟡 MAJOR | NN output bounding and ADRC fallback mode |
| **ISS-014** | 🟡 MAJOR | FDI/ADRC diagnostic interface and dt-spike guards |
| **ISS-015** | 🔵 MINOR | Quaternion convention verification (resolved) |

## Prerequisites for Phase 2

- ISS-001 (FDI threshold) and ISS-003 (Q/R tuning) remain OPEN — all ESO tuning
  in Phase 2 will be invalidated when these close (per CRITICAL risk in advisory §4)
- Angular velocity telemetry and inertia tensor are now available for ESO/NN use
- Quaternion arithmetic convention is pinned and tested
