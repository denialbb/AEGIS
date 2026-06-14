---
name: run-auto-tuner
description: >-
  Executes the AEGIS configuration tuning script, waits for completion, parses the results to find the best configuration based on distance/fuel, and updates config.py automatically.
---

# Run Auto-Tuner

## Overview
This skill orchestrates the automated testing of AEGIS configuration parameters. It runs the grid search script asynchronously, waits for it to finish, parses the CSV results to find the run that achieved the safest and most accurate landing, and directly patches `src/config.py` with those winning parameters.

## Dependencies
None.

## Quick Start
"Please run the auto-tuner and find the best attitude parameters."

## Workflow
### 1. Execute the Tuner Script
- Use the `run_command` tool to execute `wsl -d Arch .venv/bin/python scripts/tune_config.py`.
- Run the command asynchronously (set `WaitMsBeforeAsync` to `5000` or similar).

### 2. Wait for Completion
- Use the `schedule` tool to set a 10-minute timer (`DurationSeconds=600`, `Prompt="The 10 minute tuning period has elapsed. Please check on the script or proceed to parse the results."`).
- Stop calling tools and wait for a message indicating the task has completed or the timer has fired.

### 3. Parse and Evaluate Results
- Once complete, use the `run_command` tool to execute `wsl -d Arch .venv/bin/python scripts/parse_tuning.py --input logs/tuning_results.csv`.
- This script will automatically filter for safe landings and output the JSON data of the most accurate run.
- Note the optimal `KP_ATT` and `KD_ATT` values from the output.

### 4. Update Configuration
- Use the `replace_file_content` tool on `src/config.py`.
- Replace the `GUIDANCE_KP_ATT` and `GUIDANCE_KD_ATT` lists with the winning values (e.g. `[10.0, 10.0, 10.0]`).
- Present the winning combination and metrics to the user in a short summary.

## Common Mistakes
- **Hanging on execution**: Do not wait for the script synchronously. Ensure it's launched in the background.
- **Applying crashed configs**: Never apply parameters if the `State` was `HARD_ABORT` or if the vessel crashed, regardless of distance or fuel.
