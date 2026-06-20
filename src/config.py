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

# Use KSP stock SAS for attitude stabilisation.
# When False, the guidance controller handles attitude entirely via gimbal trim
# (inertia-scaled PD + gyroscopic feedforward per ADR-028).  SAS and our
# controller fight for the same engine gimbals, so disabling SAS gives us
# full authority — at the cost of requiring well-tuned attitude gains.
USE_SAS = False

# Enable SAS in prograde mode during ascent for a stable test start.
# Operates independently of USE_SAS — this is always active unless set False.
# Ensures the vessel points prograde while climbing, preventing tumbling
# before the guidance controller takes over on descent.
SAS_PROGRADE_ASCENT = True

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
ALT_POWERED_DESCENT = 4000.0

# Altitude to enter HOVER_TARGETING and zero out lateral drift.
# Gives the vessel a buffer to precisely align over the pad before final descent.
# Min: 100.0, Max: 1000.0
ALT_HOVER = 200.0

# Final slow descent phase threshold. Sets the height of the final touchdown phase.
# Min: 10.0, Max: 200.0
ALT_TERMINAL = 20.0

# ---------------------------------------------------------
# Landed Timer Parameters
# ---------------------------------------------------------
# Vertical velocity threshold (m/s) below which the vessel counts as "landed"
# The timer only accumulates when |vertical_velocity| < LANDED_VEL_THRESHOLD.
# Min: 0.01, Max: 5.0
LANDED_VEL_THRESHOLD = 0.5  # m/s

# Altitude above ground threshold (m) within which the vessel counts as "at land level"
# The timer only accumulates when altitude < LANDED_ALT_THRESHOLD.
# Min: 0.01, Max: 10.0
LANDED_ALT_THRESHOLD = 1.0  # m

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

# Standard deviation of velocity noise (m/s).
# Higher values make the estimator trust the altitude sensor more for velocity.
# Min: 0.1 (High-end GPS), Max: 5.0 (Noisy GPS)
SIGMA_VEL = 1.0  # m/s

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
# IMU & Inertial Navigation Parameters
# ---------------------------------------------------------
# Standard deviation of gyroscope noise (rad/s).
# Higher values make the attitude estimate more reliant on accelerometer (slower but less drift).
# Min: 0.001 (High-end gyro), Max: 0.1 (Low-end gyro)
SIGMA_GYRO = 0.01  # rad/s

# Gyroscope bias instability (rad/s/sqrt(Hz)).
# Models random walk of gyroscope bias over time.
# Min: 1e-6 (Very stable), Max: 0.01 (Unstable)
GYRO_BIAS_INSTABILITY = 0.0001  # rad/s/sqrt(Hz)

# Standard deviation of accelerometer noise (m/s^2).
# Already defined above as SIGMA_ACCEL, keeping for clarity
# ACCEL_BIAS_INSTABILITY models random walk of accelerometer bias
ACCEL_BIAS_INSTABILITY = 0.001  # m/s^2/sqrt(Hz)

# Mahony filter proportional gain.
# Higher values increase correction from accelerometer (reduces drift but increases noise sensitivity).
# Min: 0.1 (Sluggish correction), Max: 10.0 (Aggressive correction)
MAHONY_KP = 2.0

# Mahony filter integral gain (for gyroscope bias estimation).
# Higher values better estimate and remove gyroscope bias but can cause overshoot.
# Min: 0.0 (No integral action), Max: 0.1 (Aggressive bias estimation)
MAHONY_KI = 0.0

# ---------------------------------------------------------
# Error-State EKF Parameters
# ---------------------------------------------------------
# Initial attitude uncertainty (rad, 1-σ).
# Controls how quickly the EKF trusts accelerometer-based attitude correction.
# Min: 0.01 (Well-known attitude), Max: 1.0 (Large initial error)
EKF_INITIAL_ATT_UNCERTAINTY = 0.1  # rad

# Initial gyroscope bias uncertainty (rad/s, 1-σ).
# Min: 1e-5 (Factory-calibrated), Max: 0.1 (Uncalibrated)
EKF_INITIAL_GYRO_BIAS_UNCERTAINTY = 0.01  # rad/s

# Initial accelerometer bias uncertainty (m/s², 1-σ).
# Min: 1e-4, Max: 1.0
EKF_INITIAL_ACCEL_BIAS_UNCERTAINTY = 0.1  # m/s²

# Innovation magnitude threshold for IMU health monitoring.
# When the normalised innovation exceeds this, an sensor anomaly is flagged.
# Min: 3.0 (Sensitive), Max: 10.0 (Tolerant)
EKF_INNOVATION_FAULT_THRESHOLD = 5.0

# ---------------------------------------------------------
# Glide-Slope Guidance
# ---------------------------------------------------------
# Vertical velocity target = -sqrt(2 * a_avail * alt_above_floor), where
# a_avail is the vessel's net upward acceleration from actual TWR.  This is a
# proper suicide-burn profile: at every altitude the target speed is exactly
# the velocity a constant-deceleration trajectory would have, so the vehicle
# reaches zero speed precisely at floor_alt.  Altitude above floor is capped
# at max_descent_rate (structural/terminal-velocity limit).

# DEPRECATED — no longer used.  The sqrt suicide-burn profile replaces the
# old linear k_alt * alt_above_floor profile.  Kept for backward compat only.
# Min: 0.1 (Slow drop), Max: 0.8 (Aggressive drop)
GLIDESLOPE_K_ALT = 0.8  # [1/s]

# Max descent speed during initial braking phase (m/s).
# With the suicide-burn sqrt profile, this is a structural/terminal-velocity
# cap, not the guidance profile itself. Set high enough to not interfere.
# Min: 20.0, Max: 500.0
GLIDESLOPE_RATE_POWERED_DESCENT = 300.0  # [m/s]

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
GUIDANCE_KD_VEL_VERTICAL = 2.0

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

# ---------------------------------------------------------
# Acceleration Command Clamp
# ---------------------------------------------------------
# Multiplier on max_a_avail (the vessel's net upward acceleration from TWR)
# used to cap a_cmd_world before it enters force_body and target_up_world.
# A value of 2.5 means the guidance can command up to 250 % of the vessel's
# net upward accelerating capability (a_avail).  The sqrt profile requires
# a_avail NET deceleration, so the clamp must satisfy:
#   clamp_factor >= 1 + g / a_avail
# For TWR=2 (a_avail ~ g) this requires clamp_factor >= 2.0.
# Higher = more aggressive (risks attitude flip during saturating transients);
# lower = more conservative (risks the vehicle always lagging the profile).
# Min: 2.0, Max: 4.0
ACCEL_CLAMP_FACTOR = 2.5  # multiplier on max_a_avail, see GuidanceController

# Scale factor for adaptive process noise in the StateEstimator.
# Larger value makes the filter increase velocity‑noise covariance when the
# commanded acceleration magnitude grows (helps during high‑thrust phases).
# Recommended range: 0.05 – 0.2 (tune experimentally).
PROCESS_NOISE_THRUST_COEF = 0.1  # (1/(m/s)^2) scaling of Q based on |a|^2

GRAVITY = [0.0, 0.0, -9.81]

# ---------------------------------------------------------
# Reaction Wheel Attitude Augmentation (TODO: ADR-029)
# ---------------------------------------------------------
# Gain that maps torque_body (N·m) to the stock [-1, 1] pitch/yaw/roll
# range for reaction wheels.  Only active when gimbal authority is weak
# (low throttle, small moment arms).  Tune empirically.
# Min: 0.0 (Off), Max: 1e-3 (Very aggressive)
# RW_AUGMENT_GAIN = 0.0

# ---------------------------------------------------------
# Application Logging
# ---------------------------------------------------------
DEBUG_LOGGING = False
LOG_TO_FILE = False
LOG_FILE_PATH = "logs/aegis.log"

# ---------------------------------------------------------
# HUD Display
# ---------------------------------------------------------
# Enable the in-terminal heads-up display that updates in-place
# without scrolling. Automatically disabled if stdout is not a TTY.
HUD_ENABLED = True
# HUD refresh rate in Hz. Decoupled from the control loop to reduce
# terminal overhead. 10 Hz is smooth enough for monitoring.
HUD_REFRESH_HZ = 10.0
