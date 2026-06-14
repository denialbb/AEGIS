import os
import time
import sys
sys.path.insert(0, os.path.abspath('.'))

import optuna
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
    
    config.GLIDESLOPE_K_ALT = trial.suggest_float("GLIDESLOPE_K_ALT", 0.1, 0.8)
    config.GLIDESLOPE_RATE_POWERED_DESCENT = trial.suggest_float("GLIDESLOPE_RATE_POWERED_DESCENT", 20.0, 150.0)
    config.GLIDESLOPE_RATE_HOVER = trial.suggest_float("GLIDESLOPE_RATE_HOVER", 5.0, 30.0)
    config.GLIDESLOPE_RATE_TERMINAL = trial.suggest_float("GLIDESLOPE_RATE_TERMINAL", 0.5, 5.0)
    
    config.GUIDANCE_KP_POS_LATERAL = trial.suggest_float("GUIDANCE_KP_POS_LATERAL", 0.1, 5.0)
    config.GUIDANCE_KP_POS_VERTICAL = trial.suggest_float("GUIDANCE_KP_POS_VERTICAL", 0.1, 5.0)
    config.GUIDANCE_KD_VEL_LATERAL = trial.suggest_float("GUIDANCE_KD_VEL_LATERAL", 2.0, 100.0)
    config.GUIDANCE_KD_VEL_VERTICAL = trial.suggest_float("GUIDANCE_KD_VEL_VERTICAL", 2.0, 100.0)
    
    kp_att_val = trial.suggest_float("GUIDANCE_KP_ATT_SCALAR", 2.0, 50.0)
    config.GUIDANCE_KP_ATT = [kp_att_val, kp_att_val, kp_att_val]
    
    kd_att_val = trial.suggest_float("GUIDANCE_KD_ATT_SCALAR", 1.0, 40.0)
    config.GUIDANCE_KD_ATT = [kd_att_val, kd_att_val, kd_att_val]
    
    config.SIGMA_ALT = trial.suggest_float("SIGMA_ALT", 0.1, 10.0)
    config.SIGMA_ACCEL = trial.suggest_float("SIGMA_ACCEL", 0.05, 2.0)
    config.FDI_THRESHOLD = trial.suggest_float("FDI_THRESHOLD", 1.5, 5.0)

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
    
    # Enable trims
    for part in vessel.parts.all:
        if part.modules:
            for module in part.modules:
                if module.name == 'ModuleGimbalTrim':
                    module.set_action('Toggle Event', True)

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
        # Vessel crashed or aborted
        return 1e5 + distance_to_pad

    # Fitness: Primary minimize distance. Secondary minimize fuel.
    # 1 kg of fuel is penalized equivalent to 0.1 meters of distance error.
    fitness = distance_to_pad + (fuel_used * 0.1)
    return fitness

if __name__ == "__main__":
    db_path = "sqlite:///logs/optuna.db"
    study_name = "aegis_full_tuning"
    
    os.makedirs("logs", exist_ok=True)
    
    print(f"Starting Optuna hyperparameter optimization. Database: {db_path}")
    
    study = optuna.create_study(
        study_name=study_name, 
        storage=db_path, 
        load_if_exists=True,
        direction="minimize"
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
