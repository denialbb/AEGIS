---
name: run-auto-tuner
description: >-
  Executes the AEGIS configuration tuning scripts, waits for completion, parses the results to find the best configuration, and updates the appropriate `.conf` files automatically.
---

# Run Auto-Tuner

## Overview
AEGIS has two independent Optuna-based tuning scripts:

| Script | Scope | KSP required? | Sampler |
|---|---|---|---|
| `tune_config_optuna.py` | Guidance gains, state-machine thresholds, glideslope caps, phase-specific PD gains, Mahony | **Yes** — live KSP flight per trial | CMA‑ES |
| `tune_estimator_optuna.py` | Sensor noise (SIGMA_*), bias instability, EKF process‑noise coefficient | **No** — replays recorded `.npz` flights | TPE |

Both write best results directly into the appropriate `.conf` file under `src/config/`.

## Phase 1 — Record Flights (for estimator tuning)
Run alongside `apogee_test.sh` to create `.npz` telemetry recordings:

```
wsl -d Arch .venv/bin/python scripts/flight_recorder.py
```

Toggle kRPC **Action Group 1** to start/stop recording. Repeat 3–5 times. Files land in `recordings/`.

## Phase 2 — Run the Estimator Tuner
```
wsl -d Arch .venv/bin/python scripts/tune_estimator_optuna.py [n_trials]
```

- Default: 150 trials × number of recordings.
- Offline (no KSP needed once recordings exist).
- Writes best params to `sensors.conf` and `aegis.conf`.

## Phase 3 — Run the Config Tuner
```
wsl -d Arch sh -c "export KRPC_ADDRESS=172.22.80.1 && .venv/bin/python scripts/tune_config_optuna.py"
```

- Each trial = a live KSP flight. Keep KSP open.
- Indefinite (Ctrl+C to stop). ~1–2 min per trial.
- Writes best params across `aegis.conf`, `glideslope.conf`, and `sensors.conf`.
- Best params also saved to `logs/best_params.json`.

## Output Files
| File | Contents |
|---|---|
| `logs/config-optuna.db` | Config‑tuner trial history (SQLite) |
| `logs/ekf-tuning.db` | Estimator‑tuner trial history (SQLite) |
| `logs/best_params.json` | Best config‑tuner parameters (JSON) |
| `logs/best_ekf_params.json` | Best estimator‑tuner parameters (JSON) |
| `src/config/*.conf` | Tuned values applied in‑place |

## Common Mistakes
- **Hanging on execution**: Do not run the config tuner synchronously — it runs for hours. Launch in the background or in a separate terminal.
- **Applying crashed configs**: The tuner only applies best params from LANDED trials. Crashed trials score ≥ 10 000 and are never selected as best.
- **Config‑tuner study name**: `aegis_tuning` (not `aegis_full_tuning`). Database at `logs/config-optuna.db` (not `logs/optuna.db`).
- **Default values outside Optuna ranges**: If you change a default in `.conf` to a value outside the tuner's `suggest_*` range, the tuner will not explore it. Update the range in the script to match.
