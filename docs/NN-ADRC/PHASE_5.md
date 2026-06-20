# Phase 5 — Neural Network Training Pipeline & Production Wiring

> Per the phased roadmap in `NN_ADRC_DESIGN_ADVISORY.md`, this phase implements the **NN training data pipeline** (synthetic + KSP-collected), the **train orchestrator script**, and the **production wiring** that connects the ADRC/CTM/NN stack into `MissionDirector` for closed-loop validation.

## Scope

| Deliverable | Status | Reference |
|---|---|---|
| Training data pipeline (synthetic + KSP) | 📋 Plan | `nn.py` `generate_training_data()` |
| `train_nn.py` orchestrator script | 📋 Plan | new file |
| ADRC/CTM/NN wiring in `MissionDirector` | 📋 Plan | `main.py`, `controller.py` |
| Config parameters for ADRC/CTM/NN | 📋 Plan | `config.py` |
| Telemetry logging for NN signals | 📋 Plan | `frame.py`, `writer.py` |
| Closed-loop validation methodology | 📋 Plan | — |

## Files Changed

| File | Status | What |
|---|---|---|
| `scripts/train_nn.py` | **NEW** | Training pipeline orchestrator |
| `scripts/collect_nn_data.py` | **NEW** | KSP flight data collection |
| `src/main.py` | **MODIFIED** | Wire ADRC, CTM, NN into MissionDirector |
| `src/config.py` | **MODIFIED** | Expose ADRC/CTM/NN parameters |
| `src/guidance/nn.py` | **MODIFIED** | Add save/load, normalization, train/val split |
| `src/telemetry/frame.py` | **MODIFIED** | Add NN input/output signals |
| `docs/NN-ADRC/PHASE_5.md` | **NEW** | This document |

---

## 1. Training Data Pipeline

The current `generate_training_data()` creates ~2500 synthetic samples from a 3-DOF kinematic double-integrator simulation with only step-function disturbances. This is a solid baseline but needs extension for robustness.

### 1.1 Synthetic Data Augmentation

Add four disturbance profiles, selectable per trajectory:

| Profile | Equation | Physical analogy |
|---|---|---|
| Step | `d(t) = A · H(t - t₀)` | Sudden engine failure, gimbal jam |
| Sinusoidal | `d(t) = A · sin(ω · (t - t₀))` | Aero oscillation, asymmetric thrust ripple |
| Impulse | `d(t) = A · exp(-(t - t₀)² / (2σ²))` | Staging shock, engine spool-up transient |
| Ramp | `d(t) = A · clamp((t - t₀) / T, 0, 1)` | Gradual fuel imbalance, thermal derating |

Each trajectory randomly selects a profile, axis (or multi-axis combination), magnitude `A ∈ [10, 100]` Nm, and onset time. The `generate_training_data()` signature gains a `profiles` parameter:

```python
def generate_training_data(
    num_trajectories: int = 5,
    steps_per_trajectory: int = 500,
    dt: float = 0.02,
    inertia: np.ndarray | None = None,
    profiles: list[str] | None = None,
    stab_gains: bool = False,
    adrc: ADRCController | None = None,
    seed: int = 42,
) -> tuple[np.ndarray, np.ndarray, dict]:
```

Target mix: 50% step, 25% sinusoidal, 15% ramp, 10% impulse — reflecting expected real-flight frequencies.

### 1.2 KSP Flight Data Collection

For supervised learning the training target is `y = J⁻¹ · d_true` — the true disturbance acceleration. In KSP this can be obtained deterministically by injecting known failures via the Gremlin framework and computing the lost torque analytically.

#### Ground Truth Derivation

When the Gremlin kills engine `k`, the torque it was producing vanishes exactly:

```python
engine_pos = np.array(engine.part.position(vessel.reference_frame))
com = np.array(vessel.center_of_mass(vessel.reference_frame))
r_k = engine_pos - com
thrust_vector = engine.thrust_direction * engine.max_thrust * throttle_k
torque_lost = np.cross(r_k, thrust_vector)        # Nm
d_true = np.linalg.inv(inertia_tensor) @ torque_lost  # rad/s²
```

This is precise because:
- The Gremlin controls which engine fails and when
- `throttle_k` is known from the allocator's output
- `max_thrust`, `thrust_direction`, `position` are static part properties
- `vessel.center_of_mass()` and `vessel.inertia_tensor` are queried at failure time

The recorded torque is the **true disturbance**, not an ESO estimate — making this genuine supervised learning.

#### Data Collection Flight Profile

```
1. Load aegis_tune_start save
2. Run CTM-ADRC controller (no NN)
3. At random t ∈ [5, 25] s into flight:
   a. Kill engine k (random selection from active set)
   b. Record 100 ticks (2 s) of:
      - state: [err_x, err_y, err_z, ω_x, ω_y, ω_z, z3_x, z3_y, z3_z]
      - target: J⁻¹ · torque_lost(k)
4. Repeat for N flights to accumulate dataset
```

#### Data Collection Script: `scripts/collect_nn_data.py`

```python
# Pseudocode
conn = krpc.connect()
conn.space_center.load("aegis_tune_start")
time.sleep(1.0)
director = MissionDirector(conn, collect_nn_data=True)  # runs CTM-ADRC, logs NN state
# After N engine-kill events, saves logs/nn_training_data.npz
```

Output format (`logs/nn_training_data.npz`):

```
X:    (M, 9)   — [err, omega, z3] concatenated training inputs
y:    (M, 3)   — J⁻¹ · torque_lost training targets
info: dict     — disturbance profile, engine index, timestamps, inertia tensor
```

### 1.3 Normalization & Splitting

Features are normalized to zero mean and unit variance before training:

```python
mean_x = np.mean(X_train, axis=0)
std_x  = np.std(X_train, axis=0) + 1e-8
X_norm = (X - mean_x) / std_x
```

Split 80/10/10 train/validation/test, stratified by disturbance profile. Normalization parameters are stored with the model so inference can apply the same transform.

---

## 2. Training Methodology

### 2.1 Architecture (unchanged from Phase 4)

```
Input (9) → ReLU → Hidden (20) → ReLU → Hidden (20) → Linear → Output (3)
```

Total parameters: 683 (9×20 + 20 + 20×20 + 20 + 20×3 + 3).

### 2.2 Loss Function

Sum of squared residuals over all samples:

```
L = Σ_i ||y_pred_i - y_true_i||²
```

### 2.3 Optimizer

`scipy.optimize.least_squares(method='trf')` — Trust Region Reflective variant of Levenberg-Marquardt, suited for small-to-medium networks where residual vector length is tractable (N < 10k, P ≈ 700).

Max function evaluations: 2000 (configurable). Early stopping: if validation loss increases for 3 consecutive checkpoints (every 100 evals), terminate.

### 2.4 Orchestrator: `scripts/train_nn.py`

```
1. Parse arguments: --synthetic-only, --from-npz <path>, --seed <int>
2. Generate synthetic data OR load KSP-collected .npz (or both, concatenated)
3. If combining: apply optional weight to synthetic portion (default 0.3)
4. Normalize X using training-set statistics
5. Train/val/test split (80/10/10, stratified by profile)
6. Train via least_squares on train set
7. Early-stop on val loss plateau
8. Report metrics: train RMSE, val RMSE, test RMSE, R², p99 error
9. Save model + normalization params to logs/nn_model.npz
```

#### Model Save Format

```python
np.savez("logs/nn_model.npz",
    W1=model.W1, b1=model.b1,
    W2=model.W2, b2=model.b2,
    W3=model.W3, b3=model.b3,
    mean_x=mean_x, std_x=std_x,
    clamp=model.clamp,
    train_loss=train_loss, val_loss=val_loss, test_loss=test_loss,
    num_samples=X.shape[0])
```

#### NNFeedforward.save() / load()

```python
class NNFeedforward:
    def save(self, path: str) -> None:
        np.savez(path,
            W1=self.W1, b1=self.b1,
            W2=self.W2, b2=self.b2,
            W3=self.W3, b3=self.b3,
            mean_x=self.mean_x, std_x=self.std_x,
            clamp=self.clamp)

    @classmethod
    def load(cls, path: str) -> NNFeedforward:
        data = np.load(path)
        nn = cls(clamp=float(data["clamp"]))
        nn.W1, nn.b1 = data["W1"], data["b1"]
        nn.W2, nn.b2 = data["W2"], data["b2"]
        nn.W3, nn.b3 = data["W3"], data["b3"]
        nn.mean_x, nn.std_x = data["mean_x"], data["std_x"]
        nn.is_trained = True
        return nn
```

### 2.5 Validation Metrics

| Metric | Target | Interpretation |
|---|---|---|
| Train RMSE | < 0.05 rad/s² | Fitting quality |
| Val RMSE | < 0.08 rad/s² (within 1.6× train) | No severe overfit |
| Test RMSE | < 0.08 rad/s² | Generalization to unseen profiles |
| R² | > 0.95 | Variance explained |
| Max error (p99) | < 0.3 rad/s² | Worst-case bounded |

#### Pass/Fail Criteria

| Criterion | Verdict |
|---|---|
| Test RMSE < 0.1 rad/s² AND R² > 0.9 | ✅ Accept model |
| Test RMSE 0.1–0.2 rad/s² OR R² 0.8–0.9 | ⚠️ Accept with caveat; require closed-loop validation |
| Test RMSE > 0.2 rad/s² OR R² < 0.8 | ❌ Reject — need more/better data |

---

## 3. Production Wiring

This is the gating item. `GuidanceController` already supports ADRC/CTM/NN (Phase 2–4), but `MissionDirector` passes `None` for all three (lines 141–151 of `main.py`).

### 3.1 Config Parameters

Add to `src/config.py`:

```python
# ── ADRC / ESO ──
ESO_B01: float = 15.0       # ESO gain β01
ESO_B02: float = 75.0       # ESO gain β02
ESO_B03: float = 125.0      # ESO gain β03
ESO_ALPHA1: float = 0.5     # fal() α₁
ESO_ALPHA2: float = 0.25    # fal() α₂
ESO_DELTA: float = 0.1      # fal() δ
ESO_B0: float = 1.0         # Control input gain (per ADR-027, inertia-free)

# ── CTM Baseline ──
CTM_KP: float = 9.0         # Natural frequency squared: ωₙ² = 3²
CTM_KD: float = 6.0         # 2ζωₙ = 2 · 1 · 3

# ── NN Feedforward ──
NN_MODEL_PATH: str = ""     # Path to trained .npz; empty = skip NN init
NN_CLAMP: float = 5.0       # Output clamp per axis (rad/s²)

# ── Telemetry ──
LOG_NN_SIGNALS: bool = False  # Log NN/ADRC internal signals to CSV
```

### 3.2 MissionDirector Changes

In `src/main.py`, after `self.inertia_tensor` is queried (line 130) and before `GuidanceController` is instantiated (line 141):

```python
from src.guidance.adrc import ADRCController, CTMCalculator
from src.guidance.nn import NNFeedforward

# ── Phase 2: ADRC (ESO + WSEF) ──
if config.ESO_B01 > 0:  # ADRC enabled
    self.adrc = ADRCController(
        kp=[3.0, 3.0, 3.0],
        kd=[1.0, 1.0, 1.0],
        b0=[config.ESO_B0, config.ESO_B0, config.ESO_B0],
    )
    # ── Phase 3: CTM baseline ──
    self.ctm = CTMCalculator(
        kp=config.CTM_KP,
        kd=config.CTM_KD,
        inertia_tensor=self.inertia_tensor,
    )
else:
    self.adrc = None
    self.ctm = None

# ── Phase 4: NN feedforward ──
self.nn_model = None
if config.NN_MODEL_PATH:
    self.nn_model = NNFeedforward.load(config.NN_MODEL_PATH)
```

Then pass to `GuidanceController`:

```python
self.guidance = GuidanceController(
    kp_pos_lateral=config.GUIDANCE_KP_POS_LATERAL,
    kp_pos_vertical=config.GUIDANCE_KP_POS_VERTICAL,
    kd_vel_lateral=config.GUIDANCE_KD_VEL_LATERAL,
    kd_vel_vertical=config.GUIDANCE_KD_VEL_VERTICAL,
    kp_att=kp_att,
    kd_att=kd_att,
    gravity=-self.up_vector * 9.81,
    inertia_tensor=self.inertia_tensor,
    adrc=self.adrc,
    ctm_calculator=self.ctm,
    nn_model=self.nn_model,
    accel_clamp_factor=config.ACCEL_CLAMP_FACTOR,
)
```

### 3.3 Activation Flow

The ADRC/CTM/NN pipeline is active in all guided phases:

| State | Controller | SAS |
|---|---|---|
| `STANDBY` | None (idle) | Config-dependent |
| `ASCENT_COAST` | None (coasting) | SAS prograde |
| `DEORBIT_BURN` | ADRC/CTM/NN | Off |
| `HYPERSONIC_COAST` | ADRC/CTM/NN | Off |
| `POWERED_DESCENT` | ADRC/CTM/NN | Off |
| `HOVER_TARGETING` | ADRC/CTM/NN | Off |
| `TERMINAL_DESCENT` | ADRC/CTM/NN | Off |

During coasting phases (`ASCENT_COAST`, `HYPERSONIC_COAST`) the controller receives zero throttle commands and maintains attitude only. The ADRC/CTM/NN still runs to keep the ESO z3 estimate current, but the NN correction is multiplied by zero throttle so it has no effect.

---

## 4. Telemetry Logging

### 4.1 New TelemetryFrame Fields

Add to `src/telemetry/frame.py` (gated behind `LOG_NN_SIGNALS`):

```python
@dataclass
class TelemetryFrame:
    # ... existing fields ...
    # NN / ADRC signals (only populated when LOG_NN_SIGNALS=True)
    err_axis: tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    z3: tuple[float, float, float] = (0.0, 0.0, 0.0)
    nn_correction: tuple[float, float, float] = (0.0, 0.0, 0.0)
    ctm_feedforward: tuple[float, float, float] = (0.0, 0.0, 0.0)
    adrc_torque: tuple[float, float, float] = (0.0, 0.0, 0.0)
```

5 new triples = 15 new fields. Total CSV columns: `(7 + 5N + 15)` where N is engine count.

### 4.2 Populating NN Signals in compute_wrench

In `GuidanceController.compute_wrench()`, populate the telemetry fields when the NN stack is active:

```python
if config.LOG_NN_SIGNALS:
    frame.err_axis = tuple(err_axis)
    frame.angular_velocity = tuple(angular_velocity)
    if self.adrc is not None:
        frame.z3 = tuple(eso.z3 for eso in self.adrc.eso)
        frame.adrc_torque = tuple(torque_body)
    if self.ctm_calculator is not None:
        frame.ctm_feedforward = tuple(feedforward_torque)
    if self.nn_model is not None and self.nn_model.is_trained:
        frame.nn_correction = tuple(nn_correction)
```

---

## 5. Validation in Closed Loop

After training and wiring, run closed-loop trials with the Optuna tuner infrastructure to confirm the NN improves disturbance rejection.

### 5.1 Test Matrix

| Test | NN | Disturbance | Metric | Pass |
|---|---|---|---|---|
| 1 | Off | Step (engine kill) | Settling time < 0.5 s | — baseline |
| 2 | On | Step (engine kill) | Settling time < 0.3 s | 40% improvement |
| 3 | Off | No disturbance | RMS err < 0.02 rad | — baseline |
| 4 | On | No disturbance | RMS err < 0.02 rad | No degradation |
| 5 | On | Mixed multi-axis | Max deviation < 0.1 rad | Bounded |

### 5.2 Ablation Tests

Run successive trials to isolate each layer's contribution:

```
1. CTM only                     → measure settling time vs baseline
2. ADRC only (no CTM, no NN)   → measure disturbance rejection
3. CTM + ADRC (no NN)          → current best without learned compensation
4. CTM + ADRC + NN             → learned feedforward added
```

### 5.3 Flight Test Procedure

Using the existing Optuna tuner pattern (load `aegis_tune_start`, activate action group 9), run five trials per configuration. Inject a Gremlin engine kill at t = 10 s and log:

- Peak angular error after failure
- Time to return within 0.02 rad of target attitude
- Fuel consumed during recovery

---

## 6. Testing

| Test | Type | What it validates |
|---|---|---|
| `test_nn_save_load` | unit | save/load round-trips weights exactly |
| `test_nn_normalize` | unit | normalization params stored and applied to inference |
| `test_nn_train_val_split` | unit | split produces correct shapes, stratified by profile |
| `test_nn_train_from_npz` | integration | loads .npz, trains, loss decreases monotonically |
| `test_generate_training_data_augmented` | unit | all 4 profile types produce correct shapes |
| `test_generate_training_data_profiles` | unit | each profile yields non-constant target |
| `test_collect_nn_data` | integration | KSP collection produces valid .npz |
| `test_mission_director_wiring` | integration | ADRC/CTM/NN instantiated when config path set |
| `test_mission_director_no_nn_fallback` | integration | empty path → nn_model is None |
| `test_telemetry_frame_nn` | unit | NN fields populated and flattened correctly |
| `test_telemetry_frame_nn_gating` | unit | LOG_NN_SIGNALS=False → fields default to 0 |
| `test_closed_loop_baseline` | system | CTM+ADRC stabilizes after engine kill |
| `test_closed_loop_nn_improvement` | system | NN reduces settling time vs baseline |
| `test_ctm_adrc_nn_compute_wrench` | unit | all combos produce expected (3,) torque |

---

## 7. ADRs

| ADR | Status | Summary |
|---|---|---|
| ADR-027 | ACCEPTED | ESO/ADRC lives in Guidance module, not State Estimator |
| ADR-028 | ACCEPTED | Inertia tensor from kRPC; inertia-scaled PD torque |
| ADR-029 | DEFERRED | KF state vector scope (6-state retained); NN gets z3 from ESO |
| ADR-030 | **PROPOSED** | NN training pipeline: offline LM-based training on synthetic + KSP-collected data |
| ADR-031 | **PROPOSED** | NN telemetry signals logged behind `LOG_NN_SIGNALS` flag |

---

## 8. Open Issues

| ISS | Severity | Relevance to Phase 5 |
|---|---|---|
| ISS-012 | RESOLVED | NN clamp protects against divergence; clamp value now in config |
| ISS-013 | MINOR | Training data realism addressed by 4-profile augmentation + KSP collection |
| ISS-014 | DEFERRED | KF state vector scope per ADR-029; not blocking |
| ISS-015 | MINOR | NN inference < 100 µs verified; no 50 Hz risk |
| ISS-001 | MAJOR | FDI threshold calibration; closed-loop NN validation may reveal FDI tuning needs |
| ISS-003 | OPEN | Estimator attitude handling; does not affect NN training |

---

## 9. Prerequisites for Phase 6

1. **Phase 5 must pass closed-loop validation** — NN must show measurable improvement over CTM+ADRC alone.
2. **ISS-001 should be closed** before production deployment so FDI and NN don't fight each other.
3. **Phase 6 (real-time learning or adaptive gain scheduling)** would extend the NN to update weights during flight, requiring a different optimizer (SGD/Adam) and a stability guarantee.
