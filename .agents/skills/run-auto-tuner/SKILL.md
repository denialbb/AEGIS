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
- Use the `run_command` tool to execute `wsl -d Arch sh -c "export KRPC_ADDRESS=172.22.80.1 && .venv/bin/python scripts/tune_config_optuna.py"`.
- Run the command asynchronously (set `WaitMsBeforeAsync` to `5000` or similar).

### 2. Wait for Completion
- Let the script run. It persists data to `logs/optuna.db`. You can kill it to pause it, and running it again will resume it.
- Once you decide it has run long enough or the user stops it, proceed.

### 3. Parse and Evaluate Results
- The script automatically outputs the best parameters from the database when it exits.
- You can also use Optuna's CLI to read the best run: `wsl -d Arch .venv/bin/python -c "import optuna; study = optuna.load_study(study_name='aegis_full_tuning', storage='sqlite:///logs/optuna.db'); print(study.best_params)"`

### 4. Update Configuration
- Use the `replace_file_content` tool on `src/config.py`.
- Replace the `GUIDANCE_KP_ATT` and `GUIDANCE_KD_ATT` lists with the winning values (e.g. `[10.0, 10.0, 10.0]`).
- Present the winning combination and metrics to the user in a short summary.

## Common Mistakes
- **Hanging on execution**: Do not wait for the script synchronously. Ensure it's launched in the background.
- **Applying crashed configs**: Never apply parameters if the `State` was `HARD_ABORT` or if the vessel crashed, regardless of distance or fuel.
