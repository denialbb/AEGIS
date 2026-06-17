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
import functools
import numpy as np
from optuna.samplers import TPESampler
from optuna.pruners import MedianPruner
from optuna.storages import RDBStorage
from optuna.exceptions import StorageInternalError
import src.config as config
from src.estimation.ekf import ErrorStateEKF
from src.simulation.flights import FlightReplayer
from scripts.trial_dashboard import TrialDashboard

# Pin BLAS to 1 thread so parallel Optuna workers don't oversubscribe cores.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")


def _retry_write(method):
    """Decorator: retry *method* with exponential backoff on SQLite lock contention."""
    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        last_exc: Exception | None = None
        for attempt in range(self._max_retries):
            try:
                return method(self, *args, **kwargs)
            except StorageInternalError as e:
                last_exc = e
                if attempt < self._max_retries - 1:
                    time.sleep(self._base_delay * (2.0 ** attempt))
        if last_exc is not None:
            raise last_exc
        return None
    return wrapper


class RetryableRDBStorage(RDBStorage):
    """RDBStorage with exponential-backoff retry on SQLite commit conflicts.

    Optuna's SQLite backend raises ``StorageInternalError`` when parallel
    workers contend for the same database file.  This wrapper catches the
    error on every write operation and retries with an exponentially
    increasing delay, making multi-worker tuning practical without
    switching to PostgreSQL.
    """

    def __init__(
        self,
        max_retries: int = 5,
        base_delay: float = 0.1,
        **kwargs,
    ) -> None:
        self._max_retries = max_retries
        self._base_delay = base_delay
        super().__init__(**kwargs)

    # ── Write operations: all retry-wrapped ──────────────────────────

    @_retry_write
    def create_new_study(self, *args, **kwargs):
        return super().create_new_study(*args, **kwargs)

    @_retry_write
    def create_new_trial(self, *args, **kwargs):
        return super().create_new_trial(*args, **kwargs)

    @_retry_write
    def set_trial_param(self, *args, **kwargs):
        return super().set_trial_param(*args, **kwargs)

    @_retry_write
    def set_trial_state(self, *args, **kwargs):
        return super().set_trial_state(*args, **kwargs)

    @_retry_write
    def set_trial_user_attr(self, *args, **kwargs):
        return super().set_trial_user_attr(*args, **kwargs)

    @_retry_write
    def set_trial_system_attr(self, *args, **kwargs):
        return super().set_trial_system_attr(*args, **kwargs)

    @_retry_write
    def set_study_user_attr(self, *args, **kwargs):
        return super().set_study_user_attr(*args, **kwargs)

    @_retry_write
    def set_trial_intermediate_value(self, *args, **kwargs):
        return super().set_trial_intermediate_value(*args, **kwargs)

    @_retry_write
    def set_trial_values(self, *args, **kwargs):
        return super().set_trial_values(*args, **kwargs)


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
    assert _dashboard is not None
    _dashboard.status(f"Trial {trial_num + 1} / {n_total} running …")


_best_raw_score: float = float("inf")


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

    trial.set_user_attr("raw_score", score)

    global _best_raw_score
    if score < _best_raw_score:
        _best_raw_score = score
    best_so_far = _best_raw_score

    elapsed = time.perf_counter() - t0
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
        _dashboard.status("")
    return abs(score - 1.0)


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

    _recordings_cache = sorted(
        f for f in glob.glob("recordings/*.npz")
        if np.load(f, allow_pickle=True)["ut"].size > 0
    )
    n_flights = len(_recordings_cache)

    study_name = "ekf-tuning"
    storage = RetryableRDBStorage(
        url=f"sqlite:///logs/{study_name}.db",
        max_retries=5,
        base_delay=0.1,
    )

    n_trials_arg = int(sys.argv[1]) if len(sys.argv) > 1 else 150
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
        raw = study.best_trial.user_attrs.get("raw_score")
        best_raw: float = (
            raw if isinstance(raw, (int, float)) else study.best_value or 0.0
        )
        print(
            f"  Resuming from trial {completed}  "
            f"(best so far: {_fmt_pct(best_raw)})"
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
            dash.advance(completed)
            study.optimize(
                objective,
                n_trials=n_trials_arg,
                n_jobs=10,
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
    completed_trials = [
        t for t in study.trials if t.state == optuna.trial.TrialState.COMPLETE
    ]
    if completed_trials:
        best_trial = study.best_trial
        raw = best_trial.user_attrs.get("raw_score")
        final_best: float = (
            raw if isinstance(raw, (int, float)) else (best_trial.value or 0.0)
        )
        print(f"  Best score       : {_fmt_pct(final_best)}")
        print(f"  Best trial       : #{best_trial.number}")
        print()
        print("  Best hyper-parameters:")
        for k, v in best_trial.params.items():
            print(f"    {k:>14s} = {v:.6f}")
        print()
    print(f"  Database : {storage}")
    print(_hr("═"))
