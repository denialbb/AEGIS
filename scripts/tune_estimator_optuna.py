#!/usr/bin/env python3
"""
Run Optuna optimisation of the Error‑State EKF hyper‑parameters using recorded
flight telemetry.  The script searches over sensor noise levels, bias random
walk rates and process‑noise scaling, and evaluates the resulting filter on
all recordings present in the ``recordings/`` directory.

The cost metric is a weighted sum of the mean position RMSE, the mean velocity
RMSE, and the mean Normalised Innovation Squared (NIS), each normalised by a
running median so all three components contribute equally regardless of scale.
The displayed score is a percentage where 100% = the running median
(lower is better).
"""

import optuna
import optuna.logging as optuna_logging

import glob
import os
import sys
import time
import signal
import numpy as np
from optuna.samplers import TPESampler
from optuna.pruners import MedianPruner
import src.config as config
from src.estimation.ekf import ErrorStateEKF
from src.simulation.flights import FlightReplayer
from scripts.trial_dashboard import TrialDashboard

_dashboard: TrialDashboard | None = None

optuna_logging.set_verbosity(optuna_logging.WARNING)
_TERMINAL_WIDTH = 72
_recordings_cache: list[str] = []


def _hr(char: str = "─") -> str:
    return char * _TERMINAL_WIDTH


def _center(text: str, width: int = _TERMINAL_WIDTH) -> str:
    return text.center(width)


def _fmt(val: float, width: int = 9, prec: int = 4) -> str:
    return f"{val:{width}.{prec}f}"


def _fmt_pct(val: float, width: int = 7, prec: int = 1) -> str:
    return f"{val * 100:{width}.{prec}f}%"


# ---------------------------------------------------------------------------
# Objective function
# ---------------------------------------------------------------------------


def _build_ekf(trial: optuna.trial.Trial) -> ErrorStateEKF:
    """Construct an EKF instance with hyper‑parameters sampled by a trial."""
    # Hyper‑parameters
    sigma_gyro = trial.suggest_float("sigma_gyro", 0.0005, 0.02)
    sigma_accel = trial.suggest_float("sigma_accel", 0.02, 1.0)
    sigma_alt = trial.suggest_float("sigma_alt", 0.5, 50.0, log=True)
    sigma_vel = trial.suggest_float("sigma_vel", 0.5, 20.0, log=True)
    bg_inst = trial.suggest_float("bg_inst", 1e-6, 0.02)
    ba_inst = trial.suggest_float("ba_inst", 1e-4, 0.1)
    process_coef = trial.suggest_float("process_coef", 0.01, 2.0, log=True)

    # Initial covariance – use default values for position/velocity
    init_cov = np.eye(12)
    init_cov[0:3, 0:3] *= 1.0  # position uncertainty 1 m²
    init_cov[3:6, 3:6] *= 1.0  # velocity uncertainty 1 (m/s)²
    init_cov[6:9, 6:9] *= bg_inst**2
    init_cov[9:12, 9:12] *= ba_inst**2

    ekf = ErrorStateEKF(
        np.zeros(3), np.zeros(3), init_cov, np.array([0.0, 0.0, 1.0])
    )

    ekf.sigma_gyro = sigma_gyro
    ekf.sigma_accel = sigma_accel
    ekf.sigma_bg = bg_inst
    ekf.sigma_ba = ba_inst
    ekf.sigma_alt = sigma_alt
    ekf.sigma_vel = sigma_vel
    ekf.thrust_coef = process_coef

    return ekf


def _flight_banner(n_flights: int, rec_paths: list[str]) -> None:
    print()
    print(_hr("═"))
    print(_center("EKF HYPER-PARAMETER TUNING (Optuna)"))
    print(_hr("═"))
    print(f"  Flight recordings : {n_flights}")

    total_ticks = 0
    total_dur = 0.0
    total_bytes = 0

    for rp in rec_paths:
        data = np.load(rp, allow_pickle=True)
        ticks = int(len(data["ut"]))
        dt_arr = np.array(data["dt"])
        dur = float(np.sum(dt_arr))
        fsize = os.path.getsize(rp)
        total_ticks += ticks
        total_dur += dur
        total_bytes += fsize
        print(
            f"    · {os.path.basename(rp):40s}  "
            f"{ticks:>5d} ticks  {dur:>7.1f}s  "
            f"{fsize / 1024:>6.0f} KB"
        )
        data.close()

    print(_hr("─"))
    print(
        f"  Total : {total_ticks} ticks  |  "
        f"{total_dur:.1f}s flight time  |  "
        f"{total_bytes / (1024 * 1024):.1f} MB on disk"
    )
    print(_hr("─"))


def _trial_banner(
    trial: optuna.trial.Trial,
    trial_num: int,
    n_total: int | None,
    params: dict[str, float],
) -> None:
    print(_hr("─"))
    print(f"Trial {trial_num + 1}")


def _trial_result(
    trial_num: int,
    score: float,
    rmse_pos: float,
    rmse_vel: float,
    nis: float,
    best_score: float,
    elapsed: float,
) -> None:
    improved = "NEW BEST" if score <= best_score else ""
    assert _dashboard is not None
    _dashboard.report_trial(
        trial_num, score, rmse_pos, rmse_vel, nis, best_score, elapsed
    )
    print(f"  ┌─ Trial {trial_num + 1} result{improved}")
    print(f"  │  Score    = {_fmt_pct(score)}")
    print(f"  │  RMSE pos = {_fmt(rmse_pos)}")
    print(f"  │  RMSE vel = {_fmt(rmse_vel)}")
    print(f"  │  NIS      = {_fmt(nis)}")
    print(f"  │  Best     = {_fmt_pct(best_score)}")
    print(f"  │  Time     = {elapsed:.1f}s")
    print(f"  └{_hr('─')[1:]}")


def objective(trial: optuna.trial.Trial) -> float:
    ekf = _build_ekf(trial)
    trial_num = trial.number
    _trial_banner(trial, trial_num, _N_TOTAL, trial.params)
    t0 = time.perf_counter()
    total_rmse_pos = 0.0
    total_rmse_vel = 0.0
    total_nis = 0.0

    for i, rec in enumerate(_recordings_cache):
        replay = FlightReplayer(rec)
        metrics = replay.evaluate(ekf)
        total_rmse_pos += metrics["rmse_pos"]
        total_rmse_vel += metrics["rmse_vel"]
        total_nis += metrics["nis"]
        if _dashboard is not None:
            _dashboard.advance(1)

    if n_flights == 0:
        print("  No recordings found — returning penalty score 1e6")
        return 1e6

    avg_rmse_pos = total_rmse_pos / n_flights
    avg_rmse_vel = total_rmse_vel / n_flights
    avg_nis = total_nis / n_flights

    # Store per-component metrics for running-normalization
    trial.set_user_attr("rmse_pos", avg_rmse_pos)
    trial.set_user_attr("rmse_vel", avg_rmse_vel)
    trial.set_user_attr("nis", avg_nis)

    # Compute running medians from completed trials
    completed = [
        t
        for t in trial.study.trials
        if t.state == optuna.trial.TrialState.COMPLETE
        and t.number != trial.number
    ]
    if len(completed) >= 20:
        pos_vals = [t.user_attrs.get("rmse_pos", np.nan) for t in completed]
        vel_vals = [t.user_attrs.get("rmse_vel", np.nan) for t in completed]
        nis_vals = [t.user_attrs.get("nis", np.nan) for t in completed]
        median_pos = float(np.nanmedian(pos_vals))
        median_vel = float(np.nanmedian(vel_vals))
        median_nis = float(np.nanmedian(nis_vals))
    else:
        median_pos, median_vel, median_nis = 5000.0, 50.0, 5000.0

    norm_pos = avg_rmse_pos / max(median_pos, 1e-12)
    norm_vel = avg_rmse_vel / max(median_vel, 1e-12)
    norm_nis = avg_nis / max(median_nis, 1e-12)
    score = 0.33 * norm_pos + 0.33 * norm_vel + 0.34 * norm_nis

    if not np.isfinite(score) or score < 0:
        print(_hr("═"))
        print("  FATAL: Invalid score detected in estimator tuning.")
        print(f"  Score     = {score}")
        print(f"  RMSE pos  = {avg_rmse_pos}")
        print(f"  RMSE vel  = {avg_rmse_vel}")
        print(f"  NIS       = {avg_nis}")
        print(f"  Trial     = {trial_num}")
        print(f"  Params    = {trial.params}")
        print("  This indicates EKF divergence or study corruption.")
        print("  Fix the root cause before re-running.")
        print(_hr("═"))
        sys.exit(1)

    elapsed = time.perf_counter() - t0
    best_so_far = trial.study.best_value if trial.number > 0 else score
    _trial_result(
        trial_num,
        score,
        total_rmse_pos / max(n_flights, 1),
        total_rmse_vel / max(n_flights, 1),
        total_nis / max(n_flights, 1),
        best_so_far,
        elapsed,
    )
    if _dashboard is not None:
        _dashboard.report_trial(
            trial_num,
            score,
            avg_rmse_pos,
            avg_rmse_vel,
            avg_nis,
            best_so_far,
            elapsed,
        )
    return score


# ---------------------------------------------------------------------------
# Main execution
# ---------------------------------------------------------------------------
_N_TOTAL: int | None = None

_PARAM_NAMES = [
    "sigma_gyro",
    "sigma_accel",
    "sigma_alt",
    "sigma_vel",
    "bg_inst",
    "ba_inst",
    "process_coef",
]

if __name__ == "__main__":
    os.makedirs("logs", exist_ok=True)

    _recordings_cache = sorted(glob.glob("recordings/*.npz"))
    n_flights = len(_recordings_cache)

    study_name = "ekf-tuning"
    storage = f"sqlite:///logs/{study_name}.db"

    n_trials_arg = int(sys.argv[1]) if len(sys.argv) > 1 else 500
    _N_TOTAL = n_trials_arg * n_flights

    study = optuna.create_study(
        study_name=study_name,
        direction="minimize",
        storage=storage,
        sampler=TPESampler(seed=config.RANDOM_SEED),
        pruner=MedianPruner(),
        load_if_exists=True,
    )

    completed = len(study.trials)
    if completed > 0:
        n_trials_arg = max(n_trials_arg - completed, 0)

    _flight_banner(n_flights, _recordings_cache)

    if completed > 0:
        print(
            f"  Resuming from trial {completed}  "
            f"(best so far: {_fmt_pct(study.best_value)})"
        )
        print(_hr("─"))

    print(f"  Target trials : {_N_TOTAL}")
    print(f"  Remaining     : {n_trials_arg}")
    print(f"  CTRL+C to stop gracefully")
    print(_hr("═"))

    _stop = [False]

    def _sigint_handler(signum: int, frame: object) -> None:
        print("\nCTRL+C received — finishing current trial and stopping…")
        _stop[0] = True

    original_handler = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, _sigint_handler)

    try:

        def _should_stop(
            study: optuna.Study, trial: optuna.trial.FrozenTrial
        ) -> None:
            if _stop[0]:
                study.stop()

        with TrialDashboard(
            total_steps=_N_TOTAL,
            history_rows=10,
            title="EKF Hyperparameter Search",
        ) as dash:
            globals()["_dashboard"] = dash
            study.optimize(
                objective,
                n_trials=n_trials_arg,
                n_jobs=1,
                callbacks=[_should_stop],
            )

    except KeyboardInterrupt:
        print("\n  interrupted.")
    finally:
        signal.signal(signal.SIGINT, original_handler)

    print()
    print(_hr("═"))
    print(_center("OPTIMISATION COMPLETE"))
    print(_hr("═"))
    completed_final = len(study.trials)
    total_expected = _N_TOTAL
    percent = (completed_final / total_expected * 100) if total_expected else 0
    print(
        f"  Completed trials : {completed_final}/{total_expected} ({percent:.1f}%)"
    )
    print(f"  Best score       : {_fmt_pct(study.best_value)}")
    print(f"  Best trial       : #{study.best_trial.number}")
    print()
    print("  Best hyper-parameters:")
    for k, v in study.best_trial.params.items():
        print(f"    {k:>14s} = {v:.6f}")
    print()
    print(f"  Database : {storage}")
    print(_hr("═"))
