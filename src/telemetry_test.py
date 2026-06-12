import krpc
import time


def main():
    print("Connecting to KSP...")
    # Establishes the TCP connection. Fails if server is not running.
    conn = krpc.connect(name="G-FOLD Prototype")

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
