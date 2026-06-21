---
name: flight-test
description: Executes flight simulations against a live KSP instance and analyzes landing trajectory and attitude telemetry.
---

# `flight-test` Skill

This skill automates running landing flight tests against a live Kerbal Space Program (KSP) simulation and performing post-flight trajectory and attitude telemetry analyses.

## When to Use This Skill
Use this skill when changes are made to the guidance algorithms, state estimation filters, sensor configurations, or control allocation policies. A full flight test is necessary as static typing and unit tests do not catch dynamic runtime anomalies in the kRPC/KSP control loop.

## Prerequisites
- A running KSP instance with the kRPC server active.
- The vessel must be positioned and ready for flight initialization (or save file reloaded).

## Instructions

When asked to run a flight test, evaluate a mission, or score landing telemetry:

1. **Verify environment and configuration:**
   Check target connection settings (e.g. kRPC server IP/port configurations).

2. **Execute Flight Simulation:**
   Run the mission execution script.
   - **Linux / macOS / WSL:** Run `wsl -d Arch ./scripts/run_and_score.sh` (or `./scripts/run_and_score.sh` if native).
   - **Windows:** Run `./scripts/run_and_score.sh` (via Bash shell) or execute the underlying python scoring script if scripting platform-specific wrappers.

3. **Verify Trajectory Telemetry:**
   Analyze trajectory invariants after flight completion. Telemetry data is stored under `logs/latest/telemetry.csv`.
   - **Linux / macOS / WSL:** `wsl -d Arch .venv/bin/python scripts/trajectory_analysis.py`
   - **Windows:** `.venv\Scripts\python scripts\trajectory_analysis.py`

4. **Verify Attitude Telemetry:**
   Analyze attitude tracking and rates.
   - **Linux / macOS / WSL:** `wsl -d Arch .venv/bin/python scripts/attitude_analysis.py`
   - **Windows:** `.venv\Scripts\python scripts\attitude_analysis.py`

5. **Summarize Telemetry & Scoring:**
   Generate a summary containing key flight variables:
   - Touchdown Position Error (target vs. actual pad coordinates)
   - Velocity at Touchdown (vertical and horizontal speeds)
   - Maximum G-Force and Attitude Error peaks
   - Total Fuel Consumption
   
6. **Detect Catastrophic Impacts:**
   Check telemetry to see if KSP reported the vessel as "landed" but with a descent rate exceeding `20 m/s`. Alert user of a simulated crash or hard abort situation.
