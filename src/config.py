"""
Global Configuration for AEGIS Mission Director.
This file contains the configuration parameters for the flight software.
Tuning these values directly affects the performance, stability, and behavior of the landing system.
"""

# ---------------------------------------------------------
# kRPC Connection
# ---------------------------------------------------------
KRPC_DEFAULT_ADDRESS = "127.0.0.1"
KRPC_CLIENT_NAME = "AEGIS Mission Director"

# ---------------------------------------------------------
# Control Loop & System
# ---------------------------------------------------------
# Main loop execution frequency in Hertz.
# Higher values (e.g., 100.0) give smoother, more reactive control but require more CPU/network bandwidth.
# Lower values (e.g., 20.0) may cause lag and attitude oscillations.
# Min: 20.0, Max: 100.0
TARGET_HZ = 50.0

# ---------------------------------------------------------
# Landing Target (Default: KSC Launchpad)
# ---------------------------------------------------------
TARGET_LAT = -0.0972
TARGET_LON = -74.5577

# ---------------------------------------------------------
# State Machine Parameters
# ---------------------------------------------------------
# The KSP Action Group (1-10) to trigger AEGIS from STANDBY to ASCENT_COAST.
ACTIVATION_ACTION_GROUP = 9

# Threshold to enter HYPERSONIC_COAST.
# A high value gives the vessel more time to free-fall and bleed velocity via aerodynamic drag.
# Min: 5000.0, Max: 30000.0
ALT_HYPERSONIC = 10000.0

# Altitude to ignite engines for primary braking (POWERED_DESCENT).
# CRITICAL: If set too low (e.g., 3000.0 suicide burn), the vessel will not have enough time to slow down and will crash.
# If too high (e.g., 15000.0), it wastes fuel hovering down.
ALT_POWERED_DESCENT = 1500.0

# Altitude to enter HOVER_TARGETING and zero out lateral drift.
# Gives the vessel a buffer to precisely align over the pad before final descent.
# Min: 100.0, Max: 1000.0
ALT_HOVER = 200.0

# Final slow descent phase threshold. Sets the height of the final touchdown phase.
# Min: 10.0, Max: 200.0
ALT_TERMINAL = 20.0

# ---------------------------------------------------------
# Sensor Noise Modeling (Standard Deviations)
# ---------------------------------------------------------
# Standard deviation of altitude noise (meters).
# Higher values make the estimator trust the IMU (accelerometer) more for vertical position.
# Min: 0.1 (Perfect radar), Max: 10.0 (Noisy radar)
SIGMA_ALT = 2.0  # meters

# Standard deviation of accelerometer noise (m/s^2).
# Higher values make the estimator trust the altitude sensor more, slowing velocity reaction time.
# Min: 0.05 (High-end IMU), Max: 2.0 (Cheap IMU)
SIGMA_ACCEL = 0.5  # m/s^2

# ---------------------------------------------------------
# Fault Detection & Isolation
# ---------------------------------------------------------
# Sensitivity threshold for detecting an engine failure.
# A higher value avoids false positives from aerodynamic noise or gimbal swinging.
# A lower value catches partial-thrust failures faster.
# Min: 1.5 (High sensitivity), Max: 5.0 (Low sensitivity)
FDI_THRESHOLD = 3.0

# ---------------------------------------------------------
# Simulation Determinism
# ---------------------------------------------------------
RANDOM_SEED = 42

# ---------------------------------------------------------
# Glide-Slope Guidance
# ---------------------------------------------------------
# Vertical position target = current altitude (zero vertical pos_err).
# Vertical velocity target = -min(max_rate, K_ALT * (current_alt - floor_alt)).
# These need empirical tuning against the vessel's actual TWR.

# Proportional multiplier for descent speed. Determines how sharply velocity drops as altitude drops.
# e.g., 0.3 means at 100m above floor, target speed is 30m/s.
# Min: 0.1 (Slow drop), Max: 0.8 (Aggressive drop)
GLIDESLOPE_K_ALT = 0.8  # [1/s]

# Max descent speed during initial braking phase (m/s).
# Min: 20.0, Max: 150.0
GLIDESLOPE_RATE_POWERED_DESCENT = 150.0  # [m/s]

# Max descent speed while searching for the pad.
# Slower speeds give the vessel more time to move laterally to the pad.
# Min: 5.0, Max: 30.0
GLIDESLOPE_RATE_HOVER = 10.0  # [m/s]

# Touchdown speed (m/s). CRITICAL: Must be lower than the landing leg destruction limit.
# Min: 0.5, Max: 5.0
GLIDESLOPE_RATE_TERMINAL = 5.0  # [m/s]

# ---------------------------------------------------------
# Guidance Controller Gains (PD)
# ---------------------------------------------------------
# Translation gains are broken into lateral and vertical components to correctly
# apply aggressive braking along the true gravity vector, regardless of latitude.

# Positional stiffness. Higher values snap the vessel to the target aggressively. Too high causes overshoot.
# Min: 0.1 (Sluggish), Max: 5.0 (Jittery)
GUIDANCE_KP_POS_LATERAL = 0.8
GUIDANCE_KP_POS_VERTICAL = 0.5

# Velocity dampening (Derivative). Higher values strictly enforce speed limits and prevent overshoot.
# Vertical is typically higher than lateral to fight gravity aggressively.
# Min: 2.0, Max: 100.0
GUIDANCE_KD_VEL_LATERAL = 10.0
GUIDANCE_KD_VEL_VERTICAL = 20.0

# Attitude stiffness (Pitch, Yaw, Roll).
# Higher values command more torque to point the nose. Too high causes thrust windup and allocator saturation.
# Min: 2.0, Max: 50.0
# DEPRECATED in favor of GUIDANCE_ATT_NATURAL_FREQ / GUIDANCE_ATT_DAMPING_RATIO (ADR-028).
# When inertia-scaled torque is active (inertia_tensor passed to controller), Kp/Kd are
# computed from: Kp = ωₙ², Kd = 2ζωₙ. These raw Kp/Kd values are used only as a fallback.
GUIDANCE_KP_ATT = [10.0, 10.0, 10.0]

# Attitude dampening (Gimbal oscillation control).
# CRITICAL FOR GIMBALS: Higher values heavily dampen "wobbling" or "rocking" by resisting rotational speed.
# Min: 1.0 (Wobbly), Max: 40.0 (Stiff)
# DEPRECATED in favor of GUIDANCE_ATT_NATURAL_FREQ / GUIDANCE_ATT_DAMPING_RATIO (ADR-028).
GUIDANCE_KD_ATT = [20.0, 20.0, 20.0]

# Natural frequency (rad/s) for attitude control per axis [pitch, yaw, roll].
# Replaces direct Kp/Kd gains when inertia-scaled torque is active (ADR-028).
# Kp = ωₙ², Kd = 2ζωₙ.
# Min: 1.0 (slow), Max: 6.0 (fast)
GUIDANCE_ATT_NATURAL_FREQ = [3.0, 3.0, 3.0]

# Damping ratio for attitude control per axis [pitch, yaw, roll].
# ζ < 1 = underdamped, ζ = 1 = critically damped, ζ > 1 = overdamped.
# Min: 0.5, Max: 2.0
GUIDANCE_ATT_DAMPING_RATIO = [1.0, 1.0, 1.0]

GRAVITY = [0.0, 0.0, -9.81]

# ---------------------------------------------------------
# Application Logging
# ---------------------------------------------------------
DEBUG_LOGGING = False
LOG_TO_FILE = False
LOG_FILE_PATH = "logs/aegis.log"
