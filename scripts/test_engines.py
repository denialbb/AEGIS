import os
import sys
import krpc
import time
import argparse

# Ensure we can import from src
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import src.config as config


def test_engines():
    address = os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS)
    print(f"Connecting to KSP at {address}...")
    try:
        conn = krpc.connect(name="Engine Test", address=address)
    except ConnectionError:
        print(f"Failed to connect to KSP at {address}.")
        return

    vessel = conn.space_center.active_vessel
    print(f"Connected to vessel: {vessel.name}")

    tagged_parts = vessel.parts.with_tag("AegisEngine")
    if not tagged_parts:
        print(
            "No parts tagged 'AegisEngine' found. Falling back to all engines."
        )
        engines = [
            part.engine
            for part in vessel.parts.engines
            if part.engine is not None
        ]
    else:
        engines = [
            part.engine for part in tagged_parts if part.engine is not None
        ]

    print(f"Found {len(engines)} engines to test.")

    # Configure each engine
    for i, engine in enumerate(engines):
        print(
            f"Configuring Engine {i}: independent_throttle=True, thrust_limit=0.0"
        )
        engine.active = True
        engine.independent_throttle = True
        engine.throttle = 1.0
        engine.thrust_limit = 0.0

    # Must be 1, we modulate engines via thrust_limit
    vessel.control.throttle = 1.0

    print("Beginning thrust limit modulation test...")
    time.sleep(2)

    # Test sequence: modulate each engine one by one
    try:
        for i, engine in enumerate(engines):
            print(f"--- Testing Engine {i} ---")

            print(f"Engine {i}: thrust_limit = 1.0")
            engine.thrust_limit = 1.0
            time.sleep(0.5)
            print(
                f"   -> active: {engine.active}, has_fuel: {engine.has_fuel}, current_thrust: {engine.thrust:.2f} N"
            )
            time.sleep(0.5)

            print(f"Engine {i}: thrust_limit = 0.0")
            engine.thrust_limit = 0.0
            time.sleep(0.5)

        print("Test complete. Safing engines.")
    except KeyboardInterrupt:
        print("Test interrupted. Safing engines.")
    finally:
        for i, engine in enumerate(engines):
            engine.thrust_limit = 0.0
            engine.independent_throttle = False
        vessel.control.throttle = 0.0
        print("Vessel throttle set to 0.0. All engines reset.")


if __name__ == "__main__":
    test_engines()
