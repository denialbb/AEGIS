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
ALT_POWERED_DESCENT = 2000.0
ALT_HOVER = 500.0
ALT_TERMINAL = 50.0

# Sensor Noise Modeling (Standard Deviations)
SIGMA_ALT = 2.0     # meters
SIGMA_ACCEL = 0.5   # m/s^2

# Fault Detection & Isolation
FDI_THRESHOLD = 10.0

# Simulation Determinism
RANDOM_SEED = 42

# Guidance Controller Gains
GUIDANCE_KP_POS = [1.0, 1.0, 1.0]
GUIDANCE_KD_VEL = [1.0, 1.0, 1.0]
GUIDANCE_KP_ATT = [10.0, 10.0, 10.0]
GUIDANCE_KD_ATT = [5.0, 5.0, 5.0]
GRAVITY = [0.0, 0.0, -9.81]

# Application Logging
DEBUG_LOGGING = False
LOG_TO_FILE = False
LOG_FILE_PATH = "logs/aegis.log"
