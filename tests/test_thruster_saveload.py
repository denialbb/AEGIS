"""Debug script: test thruster API after save-load with a live KSP connection.

This is NOT a pytest test — run manually while KSP is running:
    .venv/bin/python tests/test_thruster_saveload.py
"""
if __name__ == "__main__":
    import os, time
    os.environ['KRPC_ADDRESS'] = '172.22.80.1'
    import krpc

    conn = krpc.connect(name='test_call', address='172.22.80.1')
    conn.space_center.load('aegis_tune_start')
    time.sleep(0.5)

    vessel = conn.space_center.active_vessel
    parts = vessel.parts.with_tag('AegisEngine')

    for i, part in enumerate(parts):
        assert part.engine is not None
        thruster = part.engine.thrusters[0]
        print(f'Engine {i}:')
        try:
            d = thruster.initial_thrust_direction(vessel.reference_frame)
            print(f'  initial_thrust_direction(vessel.rf) = {d}')
        except Exception as e:
            print(f'  initial_thrust_direction(vessel.rf) ERROR: {type(e).__name__}: {str(e)[:80]}')
        try:
            d = thruster.initial_thrust_direction(vessel.reference_frame)  # type: ignore
            print(f'  initial_thrust_direction() = {d}')
        except Exception as e:
            print(f'  initial_thrust_direction() ERROR: {type(e).__name__}: {str(e)[:80]}')
        try:
            d = thruster.thrust_direction(vessel.reference_frame)
            print(f'  thrust_direction(vessel.rf) = {d}')
        except Exception as e:
            print(f'  thrust_direction(vessel.rf) ERROR: {type(e).__name__}: {str(e)[:80]}')
        try:
            p = thruster.thrust_position(vessel.reference_frame)
            print(f'  thrust_position(vessel.rf) = {p}')
        except Exception as e:
            print(f'  thrust_position(vessel.rf) ERROR: {type(e).__name__}: {str(e)[:80]}')
        print()

    conn.close()
