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

# Sensor Noise Modeling (Standard Deviations)
SIGMA_ALT = 2.0     # meters
SIGMA_ACCEL = 0.5   # m/s^2

# Fault Detection & Isolation
FDI_THRESHOLD = 0.5

# Simulation Determinism
RANDOM_SEED = 42
