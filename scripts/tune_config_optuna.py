import os
import time
import sys
import json
import re

sys.path.insert(0, os.path.abspath("."))

import optuna
from optuna.samplers import CmaEsSampler
from optuna.pruners import MedianPruner
import krpc
import numpy as np

import src.config as config
from src.main import MissionDirector

# Make optuna quieter
optuna.logging.set_verbosity(optuna.logging.INFO)


def run_simulation(trial: optuna.Trial) -> float:
    # 1. Sample hyperparameters
    config.ALT_HYPERSONIC = trial.suggest_float(
        "ALT_HYPERSONIC", 5000.0, 30000.0
    )
    config.ALT_POWERED_DESCENT = trial.suggest_float(
        "ALT_POWERED_DESCENT", 3000.0, 15000.0
    )
    config.ALT_HOVER = trial.suggest_float("ALT_HOVER", 100.0, 1000.0)
    config.ALT_TERMINAL = trial.suggest_float("ALT_TERMINAL", 10.0, 200.0)

    # Sensor & EKF hyper-parameters (SIGMA_*, GYRO_*, MAHONY_*, etc.) are
    # tuned separately via scripts/tune_estimator_optuna.py using recorded
    # flight telemetry and kept fixed during guidance tuning.
    config.GLIDESLOPE_RATE_POWERED_DESCENT = trial.suggest_float(
        "GLIDESLOPE_RATE_POWERED_DESCENT", 20.0, 500.0
    )
    config.GLIDESLOPE_RATE_HOVER = trial.suggest_float(
        "GLIDESLOPE_RATE_HOVER", 5.0, 30.0
    )
    config.GLIDESLOPE_RATE_TERMINAL = trial.suggest_float(
        "GLIDESLOPE_RATE_TERMINAL", 0.5, 5.0
    )

    config.GUIDANCE_KP_POS_LATERAL = trial.suggest_float(
        "GUIDANCE_KP_POS_LATERAL", 0.1, 5.0
    )
    config.GUIDANCE_KP_POS_VERTICAL = trial.suggest_float(
        "GUIDANCE_KP_POS_VERTICAL", 0.1, 5.0
    )

    # Attitude control uses natural-frequency/damping-ratio parameterization
    # (ADR-028). The MissionDirector derives Kp = ωₙ², Kd = 2ζωₙ internally.
    # NOTE: the deprecated GUIDANCE_KP_ATT / GUIDANCE_KD_ATT are NOT read by
    # the controller; tuning them via Optuna was a no-op.
    nat_freq = trial.suggest_float("GUIDANCE_ATT_NATURAL_FREQ_SCALAR", 1.0, 6.0)
    config.GUIDANCE_ATT_NATURAL_FREQ = [nat_freq, nat_freq, nat_freq]
    damping = trial.suggest_float("GUIDANCE_ATT_DAMPING_RATIO_SCALAR", 0.5, 2.0)
    config.GUIDANCE_ATT_DAMPING_RATIO = [damping, damping, damping]

    # Acceleration clamp factor limits a_cmd_world to ACCEL_CLAMP_FACTOR × a_avail.
    # Prevents attitude target flip during saturating transients.
    config.ACCEL_CLAMP_FACTOR = trial.suggest_float(
        "ACCEL_CLAMP_FACTOR", 2.0, 4.0
    )

    # Adaptive process‑noise scaling for the StateEstimator (see docs).
    config.PROCESS_NOISE_THRUST_COEF = trial.suggest_float(
        "PROCESS_NOISE_THRUST_COEF", 0.01, 2.0, log=True
    )

    # Kd spans 2 orders — log scale helps find the right neighborhood faster
    config.GUIDANCE_KD_VEL_LATERAL = trial.suggest_float(
        "GUIDANCE_KD_VEL_LATERAL", 2.0, 100.0, log=True
    )
    config.GUIDANCE_KD_VEL_VERTICAL = trial.suggest_float(
        "GUIDANCE_KD_VEL_VERTICAL", 2.0, 100.0, log=True
    )

    # 2. Connect to kRPC and load save
    address = os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS)
    try:
        conn = krpc.connect(name=config.KRPC_CLIENT_NAME, address=address)
    except Exception as e:
        print(f"Failed to connect to kRPC: {e}")
        return 1e6  # Extreme penalty if connection fails

    try:
        conn.space_center.load("aegis_tune_start")
    except Exception as e:
        print(f"Failed to load 'aegis_tune_start': {e}")
        conn.close()
        return 1e6

    time.sleep(0.5)  # Let physics settle
    vessel = conn.space_center.active_vessel

    # 3. Disable sensor noise for clean guidance tuning (Option C)
    config.NOISELESS_MODE = True

    # 3. Instantiate Director
    director = MissionDirector(conn)
    initial_mass = vessel.mass

    # Activate
    vessel.control.toggle_action_group(config.ACTIVATION_ACTION_GROUP)

    start_time = time.time()
    max_duration = 300.0  # 5 minutes max per test

    # 4. Run loop
    try:
        director.run_loop()
    except Exception as e:
        print(f"Error during simulation: {e}")
        conn.close()
        return 1e6

    if director._exit_requested:
        print("Trial interrupted by user.")
        conn.close()
        raise KeyboardInterrupt()

    end_time = time.time()
    angular_motion = director.total_angular_motion
    elapsed = min(end_time - start_time, max_duration)

    if director.state != "LANDED":
        # Vessel crashed or was destroyed — vessel queries will fail.
        # Score with a hard penalty, no normalization.
        conn.close()
        time_bonus = elapsed * 10.0
        return max(1e4, 1e5 - time_bonus + 0.01 * angular_motion)

    # ── LANDED: vessel is intact, safe to query ───────────────────────
    fuel_used = initial_mass - vessel.mass
    pad_pos = np.array(
        vessel.orbit.body.surface_position(
            config.TARGET_LAT,
            config.TARGET_LON,
            vessel.orbit.body.reference_frame,
        )
    )
    current_pos = np.array(vessel.position(vessel.orbit.body.reference_frame))
    distance_to_pad = float(np.linalg.norm(current_pos - pad_pos))

    conn.close()

    trial.set_user_attr("landing_distance", round(distance_to_pad, 2))
    trial.set_user_attr("fuel_used", round(fuel_used, 2))
    trial.set_user_attr("angular_motion", round(angular_motion, 2))

    # ── Normalized score (equal-weight contributions) ─────────────────
    # Each component is divided by its running median so all three
    # contribute equally regardless of scale, matching
    # scripts/tune_estimator_optuna.py.
    landed = [
        t
        for t in trial.study.trials
        if t.state == optuna.trial.TrialState.COMPLETE
        and t.number != trial.number
        and "landing_distance" in t.user_attrs
    ]
    if len(landed) >= 10:
        med_dist = float(
            np.nanmedian([t.user_attrs["landing_distance"] for t in landed])
        )
        med_fuel = float(
            np.nanmedian([t.user_attrs["fuel_used"] for t in landed])
        )
        med_ang = float(
            np.nanmedian([t.user_attrs["angular_motion"] for t in landed])
        )
    else:
        med_dist, med_fuel, med_ang = 500.0, 5000.0, 500.0

    norm_dist = distance_to_pad / max(med_dist, 1e-12)
    norm_fuel = fuel_used / max(med_fuel, 1e-12)
    norm_ang = angular_motion / max(med_ang, 1e-12)

    fitness = 0.34 * norm_dist + 0.33 * norm_fuel + 0.33 * norm_ang
    trial.report(fitness, step=0)
    return fitness


def _apply_best_params_to_config(
    params: dict[str, float],
    config_path: str = "src/config.py",
) -> None:
    """Overwrite config.py with best Optuna params, preserving comments & structure."""
    with open(config_path) as f:
        lines = f.readlines()

    replacements: dict[str, str] = {}
    for key, value in params.items():
        if key == "GUIDANCE_ATT_NATURAL_FREQ_SCALAR":
            replacements.setdefault(
                "GUIDANCE_ATT_NATURAL_FREQ", f"[{value}, {value}, {value}]"
            )
        elif key == "GUIDANCE_ATT_DAMPING_RATIO_SCALAR":
            replacements.setdefault(
                "GUIDANCE_ATT_DAMPING_RATIO", f"[{value}, {value}, {value}]"
            )
        else:
            if isinstance(value, bool):
                s = str(value)
            elif isinstance(value, int):
                s = str(value)
            else:
                s = f"{value!r}"
                if len(s) > 10:
                    s = f"{value:.6g}"
            replacements[key] = s

    new_lines: list[str] = []
    for line in lines:
        m = re.match(r"^(\w+)\s*=", line.lstrip())
        if m and m.group(1) in replacements:
            key = m.group(1)
            indent = line[: len(line) - len(line.lstrip())]
            cmt = re.search(r"(#.*)$", line)
            suffix = f"  {cmt.group(1)}" if cmt else ""
            new_lines.append(f"{indent}{key} = {replacements[key]}{suffix}\n")
        else:
            new_lines.append(line)

    with open(config_path, "w") as f:
        f.writelines(new_lines)

    print(f"\n  ✓ Applied best parameters to {config_path}")


if __name__ == "__main__":
    db_path = "sqlite:///logs/config-optuna.db"
    study_name = "aegis_tuning"

    os.makedirs("logs", exist_ok=True)

    print(f"Starting Optuna hyperparameter optimization. Database: {db_path}")
    print(f"Sampler: CMA-ES (n_startup_trials=15 random init), Pruner: Median")

    study = optuna.create_study(
        study_name=study_name,
        storage=db_path,
        load_if_exists=True,
        direction="minimize",
        sampler=CmaEsSampler(
            seed=config.RANDOM_SEED,
            n_startup_trials=15,
            consider_pruned_trials=True,
        ),
        pruner=MedianPruner(
            n_startup_trials=10,
            n_warmup_steps=0,
        ),
    )

    try:
        study.optimize(run_simulation, n_trials=None)
    except KeyboardInterrupt:
        print("\nOptimization interrupted by user.")

    completed = [
        t for t in study.trials if t.state == optuna.trial.TrialState.COMPLETE
    ]
    print(f"Number of finished trials: {len(completed)}")

    if len(completed) > 0:
        best_trial = study.best_trial
        print(f"\n{'='*60}")
        print(f"  Best trial value: {best_trial.value:.4f}")
        print(f"{'='*60}")
        print(f"  Parameters:")
        for key, value in best_trial.params.items():
            print(f"    {key}: {value}")
        print(f"{'='*60}")

        # Write full params to JSON for programmatic access
        params_path = "logs/best_params.json"
        with open(params_path, "w") as f:
            json.dump(best_trial.params, f, indent=2)
        print(f"\n  Full params written to {params_path}")

        # Auto-apply best params to src/config.py
        _apply_best_params_to_config(best_trial.params)

        print(f"\n{'='*60}")
        print(
            f"  Best params applied to src/config.py and logs/best_params.json"
        )
        print(f"{'='*60}\n")
