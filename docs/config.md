# AEGIS Configuration Guide

Configuration is organized as a Python package at `src/config/`. Each `.conf` file
is plain Python — loaded via `exec()` into the module namespace — so the
`import config; config.X` access pattern works identically to a single module.

## File Layout

| File | Purpose |
|---|---|
| `kRPC.conf` | Connection address and client name |
| `aegis.conf` | Control loop, state machine thresholds, landing target, guidance PD gains, FDI, logging, HUD |
| `sensors.conf` | Sensor noise sigmas, Mahony filter gains, bias tracking, sensor warmup |
| `ekf.conf` | EKF initial uncertainties, innovation threshold, estimator warmup |
| `glideslope.conf` | Descent-rate caps per phase, phase-specific horizontal PD gains, target blending, early translation |
| `engines.conf` | Part→thrust-axis mapping, default thrust axis |

All files are in `src/config/`.

---

## Tuning Workflow

Most guidance and EKF parameters are tuned via **Optuna** (see `scripts/tune_config_optuna.py`).
Run the tuner from the `aegis_tune_start` KSP save:

```bash
wsl -d Arch sh -c "export KRPC_ADDRESS=<ip> && .venv/bin/python scripts/tune_config_optuna.py"
```

Results persist in `logs/optuna.db` — can be stopped and resumed.

---

## 1. kRPC Connection (`kRPC.conf`)

| Parameter | Description | Default |
|---|---|---|
| `KRPC_DEFAULT_ADDRESS` | kRPC server IP (WSL2 → Windows host) | `172.22.80.1` |
| `KRPC_CLIENT_NAME` | kRPC client identifier | `AEGIS Mission Director` |

---

## 2. General Mission (`aegis.conf`)

### Control Loop

| Parameter | Description | Effect | Range |
|---|---|---|---|
| `TARGET_HZ` | Main loop frequency | Higher = smoother but more CPU/kRPC bandwidth | 20–100 |
| `USE_SAS` | Enable KSP stock SAS for attitude | `True` = SAS handles gimbal, `False` = guidance controller full authority | bool |
| `SAS_PROGRADE_ASCENT` | Prograde SAS during ascent | Keeps vessel stable during climb | bool |

### Landing Target

| Parameter | Description | Default |
|---|---|---|
| `TARGET_LAT` | Landing latitude | `-0.0972` (KSC) |
| `TARGET_LON` | Landing longitude | `-74.5577` (KSC) |
| `PAD_POSITION_ECEF` | Pad position in ECEF frame (numpy array) | `[159779.71, -1018.00, -578408.50]` |
| `LANDING_PAD_POSITION` | Pad origin in local NED frame | `[0.0, 0.0, 0.0]` |

### State Machine Thresholds

| Parameter | Description | Effect | Range |
|---|---|---|---|
| `ACTIVATION_ACTION_GROUP` | KSP action group to trigger AEGIS | 0–9 | 1–10 |
| `ALT_HYPERSONIC` | Entry to HYPERSONIC_COAST | Higher = more free-fall drag braking | 5000–30000 m |
| `ALT_POWERED_DESCENT` | Ignite engines for primary braking | **Critical** — too low = crash, too high = fuel waste | 1000–5000 m |
| `ALT_HOVER` | Entry to HOVER_TARGETING | Buffer to translate over pad | 100–1000 m |
| `ALT_TERMINAL` | Final slow-descent phase | Touchdown preparation | 5–50 m |
| `MAX_MISSION_TIME` | Hard abort timeout | Prevents infinite loops | 300 s |
| `LANDED_VEL_THRESHOLD` | Vertical speed below which landed timer accumulates | 0.5 m/s |
| `LANDED_ALT_THRESHOLD` | Altitude below which landed timer accumulates | 1.0 m |

### Fault Detection

| Parameter | Description | Effect | Range |
|---|---|---|---|
| `FDI_THRESHOLD` | Engine failure sensitivity | Higher = fewer false positives, lower = faster detection | 1.5–5.0 |

### Simulation

| Parameter | Description | Default |
|---|---|---|
| `RANDOM_SEED` | Seed for noise RNG | 42 |
| `NOISELESS_MODE` | Skip noise injection (used during Optuna tuning) | False |

### Guidance PD Gains

| Parameter | Description | Effect | Range |
|---|---|---|---|
| `GUIDANCE_KP_POS_LATERAL` | Lateral position stiffness | Higher = snappier translation | 0.1–5.0 |
| `GUIDANCE_KP_POS_VERTICAL` | Vertical position stiffness | 0.1–5.0 |
| `GUIDANCE_KD_VEL_LATERAL` | Lateral velocity damping | Higher = less overshoot | 2.0–100.0 |
| `GUIDANCE_KD_VEL_VERTICAL` | Vertical velocity damping | 2.0–100.0 |
| `GUIDANCE_KP_ATT` | Direct attitude Kp (deprecated) | 2.0–50.0 |
| `GUIDANCE_KD_ATT` | Direct attitude Kd (deprecated) | 1.0–40.0 |
| `GUIDANCE_ATT_NATURAL_FREQ` | Attitude natural frequency [pitch, yaw, roll] | Kp = ωₙ² | 1.0–6.0 rad/s |
| `GUIDANCE_ATT_DAMPING_RATIO` | Attitude damping ratio [pitch, yaw, roll] | ζ < 1 underdamped, ζ = 1 critical | 0.5–2.0 |
| `ACCEL_CLAMP_FACTOR` | Multiplier on max available acceleration | Must be ≥ 1 + g/a_avail for TWR=2 → ≥ 2.0 | 1.0–4.0 |
| `PROCESS_NOISE_THRUST_COEF` | Adaptive Q scaling by commanded acceleration² | 0.05–0.2 |
| `GUIDANCE_MAX_TORQUE` | Max torque per axis [N·m] | Prevents asymmetric thrust saturation | [3200, 3200, 3200] |

### Logging & HUD

| Parameter | Description | Default |
|---|---|---|
| `DEBUG_LOGGING` | Enable DEBUG-level logs | True |
| `LOG_TO_FILE` | Write logs to file | True |
| `LOG_FILE_PATH` | Log file path | `logs/aegis.log` |
| `HUD_ENABLED` | Enable in-terminal HUD | True |
| `HUD_REFRESH_HZ` | HUD update rate | 10.0 |

---

## 3. Sensors (`sensors.conf`)

### Noise Standard Deviations

| Parameter | Description | Effect | Range |
|---|---|---|---|
| `SIGMA_ALT` | Altimeter noise (m) | Higher → estimator trusts IMU more for vertical position | 0.1–10.0 |
| `SIGMA_ACCEL` | Accelerometer noise (m/s²) | Higher → estimator trusts altimeter more | 0.05–2.0 |
| `SIGMA_VEL` | Velocimeter noise (m/s) | Higher → smoother but slower velocity response | 0.1–5.0 |
| `SIGMA_GYRO` | Gyroscope noise (rad/s) | Higher → less gyro trust for attitude | 0.001–0.1 |

### Bias Instability

| Parameter | Description | Range |
|---|---|---|
| `GYRO_BIAS_INSTABILITY` | Gyro bias random walk (rad/s/√Hz) | 1e-6–0.01 |
| `ACCEL_BIAS_INSTABILITY` | Accel bias random walk (m/s²/√Hz) | 1e-5–0.01 |

### Mahony Filter

| Parameter | Description | Effect | Range |
|---|---|---|---|
| `MAHONY_KP` | Proportional gain | Higher = faster correction from accelerometer | 0.1–10.0 |
| `MAHONY_KI` | Integral gain (gyro bias estimation) | Higher = faster bias tracking | 0.0–0.1 |

### Bias Tracking (LPF)

| Parameter | Description | Range |
|---|---|---|
| `GYRO_BIAS_UPDATE_GAIN` | Gyro bias low-pass gain | 0.0001–0.01 |
| `ACCEL_BIAS_UPDATE_GAIN` | Accel bias low-pass gain | 0.0001–0.01 |

### Sensor Warmup

| Parameter | Description | Default |
|---|---|---|
| `SENSOR_WARMUP_TICKS` | Ticks to collect static bias samples before EKF start | 30 (3 s at 10 Hz) |
| `SENSOR_WARMUP_GYRO_BIAS_SIGMA` | Post-warmup gyro bias uncertainty (rad/s, 1-σ) | 0.003 |
| `SENSOR_WARMUP_ACCEL_BIAS_SIGMA` | Post-warmup accel bias uncertainty (m/s², 1-σ) | 0.03 |

---

## 4. EKF (`ekf.conf`)

| Parameter | Description | Effect | Range |
|---|---|---|---|
| `EKF_INITIAL_ATT_UNCERTAINTY` | Initial attitude uncertainty (rad, 1-σ) | Higher → slower convergence | 0.01–1.0 |
| `EKF_INITIAL_GYRO_BIAS_UNCERTAINTY` | Initial gyro bias uncertainty (rad/s, 1-σ) | 1e-5–0.1 |
| `EKF_INITIAL_ACCEL_BIAS_UNCERTAINTY` | Initial accel bias uncertainty (m/s², 1-σ) | 1e-4–1.0 |
| `EKF_INNOVATION_FAULT_THRESHOLD` | Normalised innovation threshold for IMU health monitoring | 3.0–10.0 |
| `ESTIMATOR_WARMUP_TICKS` | Ticks to run EKF before enabling guidance | 100 (10 s at 10 Hz) |

---

## 5. Glideslope (`glideslope.conf`)

Descent uses a suicide-burn sqrt profile:
```
v_target = -sqrt(2 * a_avail * alt_above_floor)
```
where `a_avail = total_max_thrust / mass - g`. The `GLIDESLOPE_RATE_*` values
are structural/terminal-velocity caps, not the profile itself.

### Descent-Rate Caps

| Parameter | Description | Range |
|---|---|---|
| `GLIDESLOPE_K_ALT` | **Deprecated** — linear profile constant, unused | — |
| `GLIDESLOPE_RATE_POWERED_DESCENT` | Max descent speed during braking (m/s) | 20–500 |
| `GLIDESLOPE_RATE_HOVER` | Max descent speed during pad search (m/s) | 5–30 |
| `GLIDESLOPE_RATE_TERMINAL` | Touchdown speed (m/s) — must be below leg destruction limit | 0.5–5.0 |

### Phase-Specific Horizontal PD Gains

| Phase | `KP_POS_LATERAL` | `KD_VEL_LATERAL` | Role |
|---|---|---|---|
| `PD_*` (POWERED_DESCENT) | 0.2 | 0.5 | Gentle drift damping; thrust dedicated to braking |
| `HOVER_*` (HOVER_TARGETING) | 0.5 | 3.0 | Smooth translate to pad with damped convergence |
| `TERMINAL_*` (TERMINAL_DESCENT) | 0.8 | 4.0 | Precision position hold over pad |

### Target Smoothing

| Parameter | Description | Default |
|---|---|---|
| `TARGET_BLEND_TICKS` | Ticks to blend horizontal target from phase-entry position to pad | 30 (3 s) |

### Early Translation

| Parameter | Description | Default |
|---|---|---|
| `PAD_OFFSET_EARLY_THRESHOLD` | Horizontal offset (m) triggering early nudge toward pad during braking | 500.0 |
| `PAD_OFFSET_EARLY_ALPHA` | Fraction of remaining offset corrected per tick | 0.03 |

---

## 6. Engines (`engines.conf`)

### Part Thrust Axes

Maps KSP part names to their thrust direction in the part-local frame.
Used as fallback when the kRPC Thruster API is unavailable (e.g. after
`space_center.load()`).

| Part | Thrust Axis |
|---|---|
| `liquidEngineMini.v2` (48-7S "Spark") | `(0, +1, 0)` — along part Y |
| `liquidEngine2.v2` (LV-T45 "Swivel") | `(0, 0, -1)` |
| `liquidEngine3.v2` (LV-909 "Terrier") | `(0, 0, -1)` |
| `liquidEngine` (LV-T30 "Reliant") | `(0, 0, -1)` |
| `liquidEngineS2` (LV-T45 variant) | `(0, 0, -1)` |

| Parameter | Description | Default |
|---|---|---|
| `DEFAULT_THRUST_AXIS` | Fallback axis for unknown parts | `(0, 0, -1)` |

---

## Adding a New Parameter

1. Add the variable to the appropriate `src/config/*.conf` file.
2. Add the corresponding declaration with type to `src/config/__init__.pyi`
   (required for mypy static analysis).
3. Reference as `config.YOUR_NEW_VAR` in application code — no import changes
   needed.
