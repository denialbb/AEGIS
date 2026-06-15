import math
import os
import time
import sys
sys.path.insert(0, os.path.abspath('.'))

import optuna
from optuna.samplers import CmaEsSampler
from optuna.pruners import MedianPruner
import krpc
import numpy as np

import src.config as config
from src.main import MissionDirector
from src.telemetry.sensors import SensorModels
from src.estimation.estimator import StateEstimator
from src.guidance.allocator import ControlAllocator
from src.telemetry.writer import TelemetryWriter

# Make optuna quieter
optuna.logging.set_verbosity(optuna.logging.INFO)

def run_simulation(trial: optuna.Trial) -> float:
    # 1. Sample hyperparameters
    config.ALT_HYPERSONIC = trial.suggest_float("ALT_HYPERSONIC", 5000.0, 30000.0)
    config.ALT_POWERED_DESCENT = trial.suggest_float("ALT_POWERED_DESCENT", 3000.0, 15000.0)
    config.ALT_HOVER = trial.suggest_float("ALT_HOVER", 100.0, 1000.0)
    config.ALT_TERMINAL = trial.suggest_float("ALT_TERMINAL", 10.0, 200.0)
    
    # GLIDESLOPE_K_ALT removed — sqrt suicide-burn profile self-tunes from TWR.
    # Rate limits serve as structural caps; POWERED_DESCENT range widened.
    config.GLIDESLOPE_RATE_POWERED_DESCENT = trial.suggest_float("GLIDESLOPE_RATE_POWERED_DESCENT", 20.0, 500.0)
    config.GLIDESLOPE_RATE_HOVER = trial.suggest_float("GLIDESLOPE_RATE_HOVER", 5.0, 30.0)
    config.GLIDESLOPE_RATE_TERMINAL = trial.suggest_float("GLIDESLOPE_RATE_TERMINAL", 0.5, 5.0)
    
    config.GUIDANCE_KP_POS_LATERAL = trial.suggest_float("GUIDANCE_KP_POS_LATERAL", 0.1, 5.0)
    config.GUIDANCE_KP_POS_VERTICAL = trial.suggest_float("GUIDANCE_KP_POS_VERTICAL", 0.1, 5.0)
    
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
    config.ACCEL_CLAMP_FACTOR = trial.suggest_float("ACCEL_CLAMP_FACTOR", 1.0, 3.0)
    
    # Log-scale for params spanning 1+ order of magnitude
    config.SIGMA_ALT = trial.suggest_float("SIGMA_ALT", 0.1, 10.0, log=True)
    config.SIGMA_ACCEL = trial.suggest_float("SIGMA_ACCEL", 0.05, 2.0, log=True)
    config.FDI_THRESHOLD = trial.suggest_float("FDI_THRESHOLD", 1.5, 5.0)
    
    # Kd spans 2 orders — log scale helps find the right neighborhood faster
    config.GUIDANCE_KD_VEL_LATERAL = trial.suggest_float("GUIDANCE_KD_VEL_LATERAL", 2.0, 100.0, log=True)
    config.GUIDANCE_KD_VEL_VERTICAL = trial.suggest_float("GUIDANCE_KD_VEL_VERTICAL", 2.0, 100.0, log=True)

    # 2. Connect to kRPC and load save
    address = os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS)
    try:
        conn = krpc.connect(name=config.KRPC_CLIENT_NAME, address=address)
    except Exception as e:
        print(f"Failed to connect to kRPC: {e}")
        return 1e6 # Extreme penalty if connection fails

    try:
        conn.space_center.load("aegis_tune_start")
    except Exception as e:
        print(f"Failed to load 'aegis_tune_start': {e}")
        conn.close()
        return 1e6
        
    time.sleep(0.5) # Let physics settle
    vessel = conn.space_center.active_vessel
    
    # 3. Instantiate Director
    director = MissionDirector(conn)
    initial_mass = vessel.mass

    # Activate
    vessel.control.toggle_action_group(config.ACTIVATION_ACTION_GROUP)

    start_time = time.time()
    max_duration = 300.0 # 5 minutes max per test
    
    # 4. Run loop
    try:
        director.run_loop()
    except Exception as e:
        print(f"Error during simulation: {e}")
        conn.close()
        return 1e6

    end_time = time.time()
    
    # 5. Evaluate results
    fuel_used = initial_mass - vessel.mass
    pad_pos = np.array(vessel.orbit.body.surface_position(config.TARGET_LAT, config.TARGET_LON, vessel.orbit.body.reference_frame))
    current_pos = np.array(vessel.position(vessel.orbit.body.reference_frame))
    distance_to_pad = float(np.linalg.norm(current_pos - pad_pos))
    
    conn.close()

    if director.state != "LANDED":
        # Vessel crashed or aborted.  Penalty scales with how much simulation
        # time was wasted — a trial that survives 200 s and nearly reaches
        # the pad is punished less than one that falls out of the sky in 10 s.
        elapsed = min(end_time - start_time, max_duration)
        time_bonus = elapsed * 10.0  # subtract ~1/s of survival
        return max(1e4, 1e5 + distance_to_pad - time_bonus)

    # Fitness: Primary minimize distance. Secondary minimize fuel.
    # 1 kg of fuel is penalized equivalent to 0.1 meters of distance error.
    fitness = distance_to_pad + (fuel_used * 0.1)
    trial.report(fitness, step=0)
    return fitness

if __name__ == "__main__":
    db_path = "sqlite:///logs/optuna.db"
    study_name = "aegis_full_tuning"
    
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
        # Run indefinitely. Can be interrupted with Ctrl+C.
        study.optimize(run_simulation, n_trials=None)
    except KeyboardInterrupt:
        print("\nOptimization interrupted by user. Best parameters found so far:")
        
    print(f"Number of finished trials: {len(study.trials)}")
    
    if len(study.trials) > 0:
        best_trial = study.best_trial
        print("  Value: ", best_trial.value)
        print("  Params: ")
        for key, value in best_trial.params.items():
            print(f"    {key}: {value}")
