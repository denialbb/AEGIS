import csv
import json
import argparse
import sys
import os

def find_best_run(csv_file: str):
    if not os.path.exists(csv_file):
        print(f"Error: {csv_file} not found.", file=sys.stderr)
        sys.exit(1)
        
    best_run = None
    
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["State"] == "LANDED":
                distance = float(row["Distance_m"])
                fuel = float(row["Fuel_kg"])
                
                if best_run is None:
                    best_run = row
                else:
                    best_distance = float(best_run["Distance_m"])
                    best_fuel = float(best_run["Fuel_kg"])
                    
                    if distance < best_distance:
                        best_run = row
                    elif abs(distance - best_distance) < 0.1 and fuel < best_fuel:
                        # Tie-breaker on fuel if distance is nearly identical
                        best_run = row

    if best_run:
        print(json.dumps({
            "status": "success",
            "best_run": best_run
        }, indent=2))
    else:
        print(json.dumps({
            "status": "failure",
            "message": "No successful runs found."
        }, indent=2))
        sys.exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Parse AEGIS tuning results.")
    parser.add_argument("--input", type=str, default="logs/tuning_results.csv", help="Path to tuning_results.csv")
    args = parser.parse_args()
    
    find_best_run(args.input)
