"""
Global Configuration for AEGIS Mission Director.
"""

# kRPC Connection
KRPC_DEFAULT_ADDRESS = "127.0.0.1"
KRPC_CLIENT_NAME = "AEGIS Mission Director"

# Control Loop
TARGET_HZ = 50.0

# Landing Target (Default: KSC Launchpad)
TARGET_LAT = -0.0972
TARGET_LON = -74.5577

# State Machine Parameters
ACTIVATION_ACTION_GROUP = 9
ALT_HYPERSONIC = 10000.0
ALT_POWERED_DESCENT = 6000.0
ALT_HOVER = 500.0
ALT_TERMINAL = 50.0

# Sensor Noise Modeling (Standard Deviations)
SIGMA_ALT = 2.0     # meters
SIGMA_ACCEL = 0.5   # m/s^2

# Fault Detection & Isolation
FDI_THRESHOLD = 3.0

# Simulation Determinism
RANDOM_SEED = 42

# Glide-Slope Guidance (ISS-011)
# Vertical position target = current altitude (zero vertical pos_err).
# Vertical velocity target = -min(max_rate, K_ALT * (current_alt - floor_alt)).
# These need empirical tuning against the vessel's actual TWR.
GLIDESLOPE_K_ALT = 0.3                  # [1/s] -- desired-speed-per-meter-above-floor
GLIDESLOPE_RATE_POWERED_DESCENT = 50.0  # [m/s] -- matches prior "descend at 50 m/s" intent
GLIDESLOPE_RATE_HOVER = 10.0            # [m/s]
GLIDESLOPE_RATE_TERMINAL = 2.0          # [m/s] -- matches prior "2 m/s" intent

# Guidance Controller Gains
# Translation gains are broken into lateral and vertical components to correctly
# apply aggressive braking along the true gravity vector, regardless of latitude.
GUIDANCE_KP_POS_LATERAL = 1.0
GUIDANCE_KP_POS_VERTICAL = 1.0
GUIDANCE_KD_VEL_LATERAL = 10.0
GUIDANCE_KD_VEL_VERTICAL = 40.0
GUIDANCE_KP_ATT = [10.0, 10.0, 10.0]
GUIDANCE_KD_ATT = [20.0, 20.0, 20.0]
GRAVITY = [0.0, 0.0, -9.81]

# Application Logging
DEBUG_LOGGING = False
LOG_TO_FILE = False
LOG_FILE_PATH = "logs/aegis.log"
