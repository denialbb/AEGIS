"""Debug script: inspect gimbal thruster attributes with a live KSP connection.

This is NOT a pytest test — run manually while KSP is running:
    .venv/bin/python tests/test_gimbal_thruster.py
"""
if __name__ == "__main__":
    import os, time
    os.environ['KRPC_ADDRESS'] = '172.22.80.1'
    import krpc

    conn = krpc.connect(name='test_gimbal_axes', address='172.22.80.1')
    conn.space_center.load('aegis_tune_start')
    time.sleep(0.5)

    vessel = conn.space_center.active_vessel
    parts = vessel.parts.with_tag('AegisEngine')

    for i, part in enumerate(parts):
        assert part.engine is not None
        thruster = part.engine.thrusters[0]
        print(f'Engine {i}:')
        for attr in dir(thruster):
            if not attr.startswith('_'):
                try:
                    val = getattr(thruster, attr)
                    is_call = callable(val)
                    if not is_call:
                        print(f'  thruster.{attr} = {repr(val)}')
                    else:
                        print(f'  thruster.{attr}() - callable')
                except Exception as e:
                    print(f'  thruster.{attr} ERROR: {str(e)[:60]}')
        print()
    conn.close()
