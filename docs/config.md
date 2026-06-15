# AEGIS Configuration Guide (`src/config.py`)

This document explains the practical effects of the various configuration parameters in AEGIS, along with examples for typical minimum and maximum values.

---

## 🚀 Automated Tuning (Optuna)
Because tuning 17 interdependent parameters manually is extremely difficult, AEGIS includes an automated hyperparameter tuning script using **Optuna**.
To run the automated tuner:
1. Ensure you have the `aegis_tune_start` save in KSP.
2. Run the tuning agent skill or the script directly:
   `wsl -d Arch sh -c "export KRPC_ADDRESS=<your_ip> && .venv/bin/python scripts/tune_config_optuna.py"`
3. The script will run iteratively, persisting results to `logs/optuna.db`. You can stop it with `Ctrl+C` and restart it later to resume.

---

## 1. Control Loop & System
| Parameter | Description | Practical Effect | Min/Max Examples |
|-----------|-------------|------------------|------------------|
| `TARGET_HZ` | Main loop execution frequency in Hertz. | Higher values give smoother, more reactive control but require more CPU/kRPC bandwidth. Lower values (e.g., 10 Hz) may cause lag and attitude oscillations. | **Min:** 20.0 (Sluggish)<br>**Max:** 100.0 (CPU heavy) |
| `ACTIVATION_ACTION_GROUP` | The KSP Action Group (0-9) to trigger AEGIS. | Defines which key the user presses to transition from `STANDBY` to `ASCENT_COAST`. | **Min:** 1<br>**Max:** 10 |

## 2. State Machine Altitude Thresholds
These define the altitudes (in meters) where the mission director transitions flight states.
| Parameter | Description | Practical Effect | Min/Max Examples |
|-----------|-------------|------------------|------------------|
| `ALT_HYPERSONIC` | Threshold to enter `HYPERSONIC_COAST`. | A high value gives the vessel more time to free-fall and bleed velocity via aerodynamic drag before starting engines. | **Min:** 5000.0<br>**Max:** 30000.0 |
| `ALT_POWERED_DESCENT` | Altitude to ignite engines for primary braking. | **Critical:** If set too low, the vessel will not have enough time/altitude to slow down and will crash. If too high, it wastes fuel hovering down. | **Min:** 3000.0 (Suicide burn)<br>**Max:** 15000.0 (Fuel heavy) |
| `ALT_HOVER` | Altitude to enter `HOVER_TARGETING` and zero out lateral drift. | Gives the vessel a buffer to precisely align over the pad. | **Min:** 100.0<br>**Max:** 1000.0 |
| `ALT_TERMINAL` | Final slow descent phase threshold. | Sets the height of the final slow-touchdown phase. | **Min:** 10.0<br>**Max:** 200.0 |

## 3. Sensor Noise & State Estimation
Used by the Kalman Filter to fuse noisy telemetry.
| Parameter | Description | Practical Effect | Min/Max Examples |
|-----------|-------------|------------------|------------------|
| `SIGMA_ALT` | Standard deviation of altitude noise (meters). | Higher values make the estimator trust the IMU (accelerometer) more for vertical position. | **Min:** 0.1 (Perfect radar)<br>**Max:** 10.0 (Noisy radar) |
| `SIGMA_ACCEL` | Standard deviation of accelerometer noise (m/s²). | Higher values make the estimator trust the altitude sensor more, slowing down velocity reaction time. | **Min:** 0.05 (High-end IMU)<br>**Max:** 2.0 (Cheap IMU) |

## 4. Fault Detection & Isolation (FDI)
| Parameter | Description | Practical Effect | Min/Max Examples |
|-----------|-------------|------------------|------------------|
| `FDI_THRESHOLD` | Sensitivity threshold for detecting an engine failure. | A higher value avoids false positives from aerodynamic noise or gimbal swinging. A lower value catches partial-thrust failures faster. | **Min:** 1.5 (High sensitivity)<br>**Max:** 5.0 (Low sensitivity) |

## 5. Glide-Slope Guidance
These parameters dictate the target velocities during descent.
| Parameter | Description | Practical Effect | Min/Max Examples |
|-----------|-------------|------------------|------------------|
| `GLIDESLOPE_K_ALT` | **DEPRECATED** — no longer used. The sqrt suicide-burn profile replaces the old linear profile. | Kept for backward compat only. | — |
| `GLIDESLOPE_RATE_POWERED_DESCENT` | Structural/terminal-velocity cap for powered descent (m/s). | With the sqrt profile, this is a hard upper bound, not the guidance profile itself. Set high enough (e.g., 300) so the sqrt formula dominates over the altitude range of interest. | **Min:** 20.0<br>**Max:** 500.0 |
| `GLIDESLOPE_RATE_HOVER` | Max descent speed while searching for the pad (m/s). | Slower speeds give the vessel more time to move laterally to the pad. | **Min:** 5.0<br>**Max:** 30.0 |
| `GLIDESLOPE_RATE_TERMINAL` | Touchdown speed (m/s). | **Critical:** Must be lower than the landing leg destruction limit. | **Min:** 0.5<br>**Max:** 5.0 |

The target profile is now `v_target = -sqrt(2 * a_avail * alt_above_floor)`, where
`a_avail = total_max_thrust / mass - g` is computed each tick from the vessel's
actual TWR.  This is a proper suicide-burn that matches the target speed to the
vehicle's braking capability at every altitude, eliminating the PD saturation
that occurred with the old linear profile at high altitude.  `GLIDESLOPE_RATE_*`
serve as structural/terminal-velocity caps to protect against the sqrt producing
unreachably high speeds for a particular vessel design.

## 6. Proportional-Derivative (PD) Guidance Gains
These tune the vessel's translation and attitude behavior. If the vessel is sluggish or oscillates, these need tuning.
| Parameter | Description | Practical Effect | Min/Max Examples |
|-----------|-------------|------------------|------------------|
| `GUIDANCE_KP_POS_LATERAL` / `_VERTICAL` | Positional stiffness. | Higher values make the vessel snap to its target position aggressively. Too high causes overshoot. | **Min:** 0.1 (Sluggish)<br>**Max:** 5.0 (Jittery) |
| `GUIDANCE_KD_VEL_LATERAL` / `_VERTICAL` | Velocity dampening (Derivative). | Higher values prevent overshoot and strictly enforce speed limits. Vertical is typically higher than lateral to fight gravity aggressively. | **Min:** 2.0<br>**Max:** 100.0 |
| `GUIDANCE_KP_ATT` | Attitude stiffness (Pitch, Yaw, Roll). | Higher values command more torque to point the nose. Too high causes thrust windup and allocator saturation. | **Min:** 2.0<br>**Max:** 50.0 |
| `GUIDANCE_KD_ATT` | Attitude dampening (Gimbal oscillation control). | **Critical for Gimbals:** Higher values heavily dampen "wobbling" or "rocking" by resisting rotational speed. | **Min:** 1.0 (Wobbly)<br>**Max:** 40.0 (Stiff) |
| `ACCEL_CLAMP_FACTOR` | Multiplier on `a_avail` to cap `a_cmd_world` magnitude. | Clamps the commanded acceleration before it is projected into force and attitude target. Must satisfy `clamp >= 1 + g / a_avail` so the profile's required net deceleration can be achieved. For TWR=2 this requires `>= 2.0`. Prevents the attitude target from flipping during saturating transients. | **Min:** 2.0 (Marginal)<br>**Max:** 4.0 (Aggressive) |
