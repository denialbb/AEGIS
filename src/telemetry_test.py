import krpc
import time
import os


def main():
    # To connect from WSL2:
    # 1. In KSP kRPC settings, change the Address to 'Any' or '0.0.0.0'.
    # 2. Leave protocol as 'Protobuf over TCP'.
    # 3. Set the KRPC_ADDRESS env var to the Windows host IP when running.
    # Example: export KRPC_ADDRESS=$(ip -4 route show default | awk '{print $3}')
    address = os.environ.get("KRPC_ADDRESS", "127.0.0.1")
    print(f"Connecting to KSP at {address}...")
    # Establishes the TCP connection. Fails if server is not running.
    conn = krpc.connect(name="G-FOLD Prototype", address=address)

    vessel = conn.space_center.active_vessel
    print(f"Connected to: {vessel.name}")

    # Set up a high-performance stream
    # Instead of requesting data, the server streams it to us automatically
    altitude_stream = conn.add_stream(
        getattr, vessel.flight(), "surface_altitude"
    )
    velocity_stream = conn.add_stream(
        getattr, vessel.flight(vessel.orbit.body.reference_frame), "velocity"
    )

    print("Streaming telemetry...")
    try:
        while True:
            alt = altitude_stream()
            vel = velocity_stream()
            print(f"Alt: {alt:.1f}m | Vel: {vel}")
            time.sleep(0.1)  # 10Hz control loop

    except KeyboardInterrupt:
        print("Terminating connection.")
        conn.close()


if __name__ == "__main__":
    main()
