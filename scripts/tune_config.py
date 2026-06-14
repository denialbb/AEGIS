import os
import time
import csv
import itertools
import sys
sys.path.insert(0, os.path.abspath('.'))

import krpc
import numpy as np

# Modify config directly before instantiating MissionDirector
import src.config as config
from src.main import MissionDirector
from src.common.logger import setup_logging

def run_tuning():
    setup_logging()
    
    address = os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS)
    
    # Grid Search Definition
    kp_att_values = [5.0, 10.0, 15.0]
    kd_att_values = [10.0, 20.0, 30.0, 40.0]
    
    combinations = list(itertools.product(kp_att_values, kd_att_values))
    print(f"Starting auto-tuning with {len(combinations)} combinations.")
    
    results = []
    
    for i, (kp, kd) in enumerate(combinations):
        print(f"\n--- Iteration {i+1}/{len(combinations)} ---")
        print(f"Testing KP_ATT: {kp}, KD_ATT: {kd}")
        
        # Connect to kRPC
        try:
            conn = krpc.connect(name=f"AEGIS Tuner {i}", address=address)
        except Exception as e:
            print(f"Failed to connect to kRPC: {e}")
            break
            
        try:
            # Load the standardized save file
            print("Loading 'aegis_tune_start'...")
            conn.space_center.load("aegis_tune_start")
            time.sleep(2) # Wait for physics to settle
            
            # Fetch the new active vessel after load
            vessel = conn.space_center.active_vessel
            initial_mass = vessel.mass
            
            # Inject parameters into config
            config.GUIDANCE_KP_ATT = [kp, kp, kp]
            config.GUIDANCE_KD_ATT = [kd, kd, kd]
            
            # Instantiate Director
            director = MissionDirector(conn)
            
            # Trigger activation action group
            print("Activating AEGIS...")
            vessel.control.set_action_group(config.ACTIVATION_ACTION_GROUP, True)
            
            # Run the mission loop (blocking until LANDED or HARD_ABORT)
            print("Running Mission Director loop...")
            start_time = time.time()
            director.run_loop()
            duration = time.time() - start_time
            
            # Collect metrics
            final_state = director.state
            final_mass = vessel.mass
            fuel_used = initial_mass - final_mass
            
            # Distance from target
            body = vessel.orbit.body
            pad_pos = body.surface_position(config.TARGET_LAT, config.TARGET_LON, body.reference_frame)
            vessel_pos = vessel.position(body.reference_frame)
            dist = np.linalg.norm(np.array(pad_pos) - np.array(vessel_pos))
            
            print(f"Result: {final_state} | Distance: {dist:.2f}m | Fuel: {fuel_used:.2f}kg | Time: {duration:.1f}s")
            
            results.append({
                "KP_ATT": kp,
                "KD_ATT": kd,
                "State": final_state,
                "Distance_m": dist,
                "Fuel_kg": fuel_used,
                "Duration_s": duration
            })
            
        except Exception as e:
            print(f"Error during iteration: {e}")
            results.append({
                "KP_ATT": kp,
                "KD_ATT": kd,
                "State": f"ERROR: {e}",
                "Distance_m": -1,
                "Fuel_kg": -1,
                "Duration_s": -1
            })
        finally:
            conn.close()
            
    # Write results to CSV
    os.makedirs("logs", exist_ok=True)
    csv_file = "logs/tuning_results.csv"
    with open(csv_file, mode='w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=["KP_ATT", "KD_ATT", "State", "Distance_m", "Fuel_kg", "Duration_s"])
        writer.writeheader()
        for r in results:
            writer.writerow(r)
            
    print(f"\n✅ Tuning complete! Results saved to {csv_file}")

if __name__ == "__main__":
    run_tuning()
