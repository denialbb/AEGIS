# AEGIS Configuration Guide (`src/config.py`)

This document explains the practical effects of the various configuration parameters in AEGIS, along with examples for typical minimum and maximum values.

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
| `GLIDESLOPE_K_ALT` | Proportional multiplier for descent speed. | Determines how sharply velocity drops as altitude drops. e.g., `0.3` means at 100m, target speed is 30m/s. | **Min:** 0.1 (Slow drop)<br>**Max:** 0.8 (Aggressive) |
| `GLIDESLOPE_RATE_POWERED_DESCENT` | Max descent speed during initial braking (m/s). | Higher values mean faster falling during the braking phase. | **Min:** 20.0<br>**Max:** 150.0 |
| `GLIDESLOPE_RATE_HOVER` | Max descent speed while searching for the pad. | Slower speeds give the vessel more time to move laterally to the pad. | **Min:** 5.0<br>**Max:** 30.0 |
| `GLIDESLOPE_RATE_TERMINAL` | Touchdown speed (m/s). | **Critical:** Must be lower than the landing leg destruction limit. | **Min:** 0.5<br>**Max:** 5.0 |

## 6. Proportional-Derivative (PD) Guidance Gains
These tune the vessel's translation and attitude behavior. If the vessel is sluggish or oscillates, these need tuning.
| Parameter | Description | Practical Effect | Min/Max Examples |
|-----------|-------------|------------------|------------------|
| `GUIDANCE_KP_POS_LATERAL` / `_VERTICAL` | Positional stiffness. | Higher values make the vessel snap to its target position aggressively. Too high causes overshoot. | **Min:** 0.1 (Sluggish)<br>**Max:** 5.0 (Jittery) |
| `GUIDANCE_KD_VEL_LATERAL` / `_VERTICAL` | Velocity dampening (Derivative). | Higher values prevent overshoot and strictly enforce speed limits. Vertical is typically higher than lateral to fight gravity aggressively. | **Min:** 2.0<br>**Max:** 100.0 |
| `GUIDANCE_KP_ATT` | Attitude stiffness (Pitch, Yaw, Roll). | Higher values command more torque to point the nose. Too high causes thrust windup and allocator saturation. | **Min:** 2.0<br>**Max:** 50.0 |
| `GUIDANCE_KD_ATT` | Attitude dampening (Gimbal oscillation control). | **Critical for Gimbals:** Higher values heavily dampen "wobbling" or "rocking" by resisting rotational speed. | **Min:** 1.0 (Wobbly)<br>**Max:** 40.0 (Stiff) |
