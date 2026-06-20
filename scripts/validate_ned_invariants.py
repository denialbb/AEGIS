"""Live KSP validation of EKF/NED/ECEF invariants.

Called from landing_pad_test.sh after loading the save.
Prints pass/fail for each invariant and returns non-zero on failure.
"""
import os
import sys
import time
import numpy as np

import src.config as config
from src.common.geometry import ecef_to_ned
from src.common.reference_frame import build_ned_frame, get_pad_ecef, compute_gravity_ned


def main() -> int:
    import krpc

    address = os.environ.get("KRPC_ADDRESS", config.KRPC_DEFAULT_ADDRESS)
    print(f"Connecting to KSP at {address}...")

    try:
        conn = krpc.connect(name="AEGIS_Validate", address=address)
        print("Connected.")
    except Exception as e:
        print(f"Failed to connect to kRPC: {e}")
        return 1

    vessel = conn.space_center.active_vessel
    body = vessel.orbit.body
    mu = body.gravitational_parameter

    sep = "=" * 60
    print(f"\n{sep}")
    print("  EKF / NED / ECEF — Live Invariant Validation")
    print(sep)

    target_lat = config.TARGET_LAT
    target_lon = config.TARGET_LON
    pad_ecef = get_pad_ecef(body, target_lat, target_lon)
    pad_r = float(np.linalg.norm(pad_ecef))
    g_mag_local = mu / pad_r**2

    print(f"\n  Pad: lat={target_lat:.4f}  lon={target_lon:.4f}")
    print(f"  Pad ECEF: [{pad_ecef[0]:.1f}  {pad_ecef[1]:.1f}  {pad_ecef[2]:.1f}]")
    print(f"  Distance from centre: {pad_r:.1f} m")
    print(f"  Local gravity:       {g_mag_local:.4f} m/s\xb2")

    # ── Build NED frame (same method as main.py) ────────────────────────
    R_ecef_to_ned, ned_quat, north_e, east_e = ecef_to_ned(pad_ecef)
    ned_frame, _up_vector = build_ned_frame(conn, body, target_lat, target_lon)

    # ── Read vessel state in NED frame ──────────────────────────────────
    flight = vessel.flight(ned_frame)
    pos_ned = np.array(vessel.position(ned_frame))
    vel_ned = np.array(flight.velocity)

    # Vessel position in body-centred frame -> gravitational force
    vessel_pos_ecef = np.array(vessel.position(body.reference_frame))
    vessel_r = float(np.linalg.norm(vessel_pos_ecef))
    vessel_g_mag = mu / vessel_r**2
    vessel_gravity_ned = compute_gravity_ned(body, vessel_pos_ecef)

    # Proper acceleration (g-force) in NED frame.
    # kRPC Flight.g_force returns magnitude in Gs (multiples of 9.80665 m/s²).
    # Convert to m/s² if < 100 (clearly in Gs, not m/s²).
    raw_g = flight.g_force
    if hasattr(raw_g, "__len__"):
        proper_accel_ned = np.array([float(v) for v in raw_g])
    else:
        g_val = float(raw_g) * 9.80665 if float(raw_g) < 100.0 else float(raw_g)
        proper_accel_ned = np.array([0.0, 0.0, g_val])

    print(f"\n  --- Position -------------------------------")
    print(f"  pos_NED:          [{pos_ned[0]:.3f}, {pos_ned[1]:.3f}, {pos_ned[2]:.3f}] m")
    print(f"\n  --- Velocity -------------------------------")
    print(f"  vel_NED:          [{vel_ned[0]:.3f}, {vel_ned[1]:.3f}, {vel_ned[2]:.3f}] m/s")
    print(f"\n  --- Gravity (computed via mu/r\xb2) ---------")
    print(f"  gravity_NED:      [{vessel_gravity_ned[0]:.6f}, {vessel_gravity_ned[1]:.6f}, {vessel_gravity_ned[2]:.6f}] m/s\xb2")
    print(f"\n  --- Proper Acceleration (flight.g_force) ----")
    print(f"  proper_NED:       [{proper_accel_ned[0]:.4f}, {proper_accel_ned[1]:.4f}, {proper_accel_ned[2]:.4f}] m/s\xb2")

    # ── VALIDATION ──────────────────────────────────────────────────────
    print(f"\n{sep}")
    print("  VALIDATION RESULTS")
    print(sep)

    errors = 0
    warnings = 0

    # 1. Rotation orthonormality
    R_test = R_ecef_to_ned @ R_ecef_to_ned.T - np.eye(3)
    ortho_err = float(np.max(np.abs(R_test)))
    if ortho_err < 1e-12:
        print(f"  [PASS] R_ecef->ned is orthonormal (max|R·R^T - I| = {ortho_err:.2e})")
    else:
        print(f"  [FAIL] R_ecef->ned orthonormality error: {ortho_err:.2e}")
        errors += 1

    # 2. Determinant = +1
    det = float(np.linalg.det(R_ecef_to_ned))
    if abs(det - 1.0) < 1e-12:
        print(f"  [PASS] det(R) = {det:.14f}")
    else:
        print(f"  [FAIL] det(R) = {det:.4f}")
        errors += 1

    # 3. Quaternion is unit norm
    q_norm = float(np.linalg.norm(ned_quat))
    if abs(q_norm - 1.0) < 1e-12:
        print(f"  [PASS] NED quaternion unit norm = {q_norm:.14f}")
    else:
        print(f"  [FAIL] NED quaternion norm = {q_norm:.6f}")
        errors += 1

    # 4. Gravity has near-zero N/E components in NED
    if abs(vessel_gravity_ned[0]) < 1e-4 and abs(vessel_gravity_ned[1]) < 1e-4:
        print(f"  [PASS] Gravity N/E components < 1e-4 "
              f"(N={vessel_gravity_ned[0]:.2e}, E={vessel_gravity_ned[1]:.2e})")
    else:
        print(f"  [FAIL] Gravity has non-zero N/E: "
              f"N={vessel_gravity_ned[0]:.4f}, E={vessel_gravity_ned[1]:.4f}")
        errors += 1

    # 5. Gravity Down component = +g_local
    g_err = abs(vessel_gravity_ned[2] - vessel_g_mag)
    if g_err < 1e-3:
        print(f"  [PASS] Gravity down component = {vessel_gravity_ned[2]:.4f} "
              f"(local g = {vessel_g_mag:.4f}, err = {g_err:.2e})")
    else:
        print(f"  [FAIL] Gravity down component mismatch: {vessel_gravity_ned[2]:.4f} "
              f"vs expected {vessel_g_mag:.4f}")
        errors += 1

    # 6. North x East = Down (right-handed frame)
    north_cross_east = np.cross(north_e, east_e)
    down_expected = -pad_ecef / np.linalg.norm(pad_ecef)
    cross_err = float(np.linalg.norm(north_cross_east - down_expected))
    if cross_err < 1e-12:
        print(f"  [PASS] North x East = Down (cross err = {cross_err:.2e})")
    else:
        print(f"  [FAIL] North x East != Down: err = {cross_err:.2e}")
        errors += 1

    # 7. At rest on pad: proper accel = [0, 0, -g]
    if abs(proper_accel_ned[0]) < 0.1 and abs(proper_accel_ned[1]) < 0.1:
        approx_g = abs(proper_accel_ned[2])
        if abs(approx_g - g_mag_local) < 0.5:
            print(f"  [PASS] Proper accel N/E near-zero at rest "
                  f"(N={proper_accel_ned[0]:.3f}, E={proper_accel_ned[1]:.3f})")
        else:
            print(f"  [WARN] |proper_z| = {approx_g:.2f} vs g = {g_mag_local:.2f} "
                  f"(vessel may not be at rest)")
            warnings += 1
    else:
        print(f"  [WARN] Proper accel has significant N/E: "
              f"N={proper_accel_ned[0]:.3f}, E={proper_accel_ned[1]:.3f} "
              f"(vessel may be moving/tilting)")
        warnings += 1

    # 8. Third row of R_ecef->ned = -pad_ecef / |pad_ecef| (Down direction)
    row2 = R_ecef_to_ned[2, :]
    down_vec = -pad_ecef / pad_r
    row_match = float(np.linalg.norm(row2 - down_vec))
    if row_match < 1e-12:
        print(f"  [PASS] R[2,:] = -pad_ecef/|pad_ecef| (err = {row_match:.2e})")
    else:
        print(f"  [FAIL] R[2,:] mismatch: err = {row_match:.2e}")
        errors += 1

    # 9. Rotation consistency: R @ down_ecef = [0,0,1]
    down_ecef = -pad_ecef / pad_r
    ned_down = R_ecef_to_ned @ down_ecef
    if abs(ned_down[0]) < 1e-12 and abs(ned_down[1]) < 1e-12 and abs(ned_down[2] - 1.0) < 1e-12:
        print(f"  [PASS] R @ down_ecef = [0, 0, 1] (down in NED is +z)")
    else:
        print(f"  [FAIL] R @ down_ecef = [{ned_down[0]:.4f}, {ned_down[1]:.4f}, {ned_down[2]:.4f}]")
        errors += 1

    # 10. Rotation consistency: R @ up_ecef = [0,0,-1]
    up_ecef = pad_ecef / pad_r
    ned_up = R_ecef_to_ned @ up_ecef
    if abs(ned_up[0]) < 1e-12 and abs(ned_up[1]) < 1e-12 and abs(ned_up[2] + 1.0) < 1e-12:
        print(f"  [PASS] R @ up_ecef = [0, 0, -1] (up in NED is -z)")
    else:
        print(f"  [FAIL] R @ up_ecef = [{ned_up[0]:.4f}, {ned_up[1]:.4f}, {ned_up[2]:.4f}]")
        errors += 1

    print(f"\n{'-' * 60}")
    print(f"  Results: {errors} failures, {warnings} warnings")
    if errors > 0:
        print("  OUTCOME: FAIL -- invariant(s) violated")
    else:
        print("  OUTCOME: PASS -- all invariants satisfied")
    print(f"{'-' * 60}")

    conn.close()
    return 0 if errors == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
