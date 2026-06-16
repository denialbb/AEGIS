#!/usr/bin/env python3
"""
Run Optuna optimisation of the Error‑State EKF hyper‑parameters using recorded
flight telemetry.  The script searches over sensor noise levels, bias random
walk rates and process‑noise scaling, and evaluates the resulting filter on
all recordings present in the ``recordings/`` directory.

The cost metric is a weighted sum of the mean position RMSE, the mean velocity
RMSE, and the mean Normalised Innovation Squared (NIS).
"""

import optuna
import glob
import os
import numpy as np
from optuna.samplers import TPESampler
from optuna.pruners import MedianPruner

import src.config as config
from src.estimation.ekf import ErrorStateEKF
from src.simulation.flights import FlightReplayer

# ---------------------------------------------------------------------------
# Objective function
# ---------------------------------------------------------------------------

def _build_ekf(trial: optuna.trial.Trial) -> ErrorStateEKF:
    """Construct an EKF instance with hyper‑parameters sampled by a trial."""
    # Hyper‑parameters
    sigma_gyro = trial.suggest_loguniform("sigma_gyro", 0.0005, 0.02)
    sigma_accel = trial.suggest_loguniform("sigma_accel", 0.02, 1.0)
    sigma_alt = trial.suggest_loguniform("sigma_alt", 0.1, 5.0)
    sigma_vel = trial.suggest_loguniform("sigma_vel", 0.05, 2.0)
    bg_inst = trial.suggest_loguniform("bg_inst", 1e-6, 0.02)
    ba_inst = trial.suggest_loguniform("ba_inst", 1e-4, 0.1)
    process_coef = trial.suggest_loguniform("process_coef", 0.01, 0.2)

    # Initial covariance – use default values for position/velocity
    init_cov = np.eye(12)
    init_cov[0:3, 0:3] *= 1.0  # position uncertainty 1 m²
    init_cov[3:6, 3:6] *= 1.0  # velocity uncertainty 1 (m/s)²
    init_cov[6:9, 6:9] *= bg_inst**2
    init_cov[9:12, 9:12] *= ba_inst**2

    ekf = ErrorStateEKF(np.zeros(3), np.zeros(3), init_cov, np.array([0.0, 0.0, 1.0]))

    ekf.sigma_gyro = sigma_gyro
    ekf.sigma_accel = sigma_accel
    ekf.sigma_bg = bg_inst
    ekf.sigma_ba = ba_inst
    ekf.sigma_alt = sigma_alt
    ekf.sigma_vel = sigma_vel
    ekf.thrust_coef = process_coef

    return ekf

def objective(trial: optuna.trial.Trial) -> float:
    ekf = _build_ekf(trial)

    # Accumulate metrics across all recordings
    total_rmse_pos = 0.0
    total_rmse_vel = 0.0
    total_nis = 0.0
    count = 0

    for rec_path in glob.glob("recordings/*.npz"):
        replay = FlightReplayer(rec_path)
        metrics = replay.evaluate(ekf)
        total_rmse_pos += metrics["rmse_pos"]
        total_rmse_vel += metrics["rmse_vel"]
        total_nis += metrics["nis"]
        count += 1

    if count == 0:
        return 1e6  # no recordings – penalise heavily

    avg_rmse_pos = total_rmse_pos / count
    avg_rmse_vel = total_rmse_vel / count
    avg_nis = total_nis / count

    # Weighted cost – the exact weights can be tuned, here we use 1/3 each
    score = 0.33 * avg_rmse_pos + 0.33 * avg_rmse_vel + 0.34 * avg_nis
    return score

# ---------------------------------------------------------------------------
# Main execution
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    os.makedirs("logs", exist_ok=True)
    study_name = "ekf_optuna_tuning"
    storage = f"sqlite:///logs/{study_name}.db"
    study = optuna.create_study(
        name=study_name,
        direction="minimize",
        storage=storage,
        sampler=TPESampler(seed=config.RANDOM_SEED),
        pruner=MedianPruner(),
        load_if_exists=True,
    )
    print("Starting optimisation – database:", storage)
    study.optimize(objective, n_trials=None, n_jobs=-1)
    print("Best trial:")
    print("  Value:", study.best_trial.value)
    print("  Params:")
    for k, v in study.best_trial.params.items():
        print(f"    {k}: {v}")
