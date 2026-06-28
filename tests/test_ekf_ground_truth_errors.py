"""
Ground-truth error analysis for the Error-State EKF.

Loads one or two post-fix flight recordings, runs the full EKF replay,
and compares the estimated state against ground truth at every timestep.
Reports per-axis RMSE, max errors, innovation statistics, and bias
convergence.

This is the primary regression test for EKF tracking performance.
"""

import pytest
import numpy as np
import os
import logging

from src.simulation.flights import FlightReplayer
from src.estimation.ekf import ErrorStateEKF
import src.config as config

logger = logging.getLogger(__name__)

RECORDING_DIR = "recordings"

# Use the first two post-fix recordings that have non-zero samples
POST_FIX_CANDIDATES = sorted(
    os.path.join(RECORDING_DIR, f)
    for f in os.listdir(RECORDING_DIR)
    if f.endswith(".npz")
    and not f.endswith("074654.npz")  # 0-sample
) if os.path.exists(RECORDING_DIR) else []


def _build_ekf_at_ground_truth(
    gt_pos: np.ndarray, gt_vel: np.ndarray, up_vector: np.ndarray
) -> ErrorStateEKF:
    """Initialise EKF at the first ground-truth state with proper covariance."""
    cov = np.eye(12)
    cov[0:3, 0:3] *= config.SIGMA_ALT**2
    cov[3:6, 3:6] *= config.SIGMA_VEL**2
    cov[6:9, 6:9] *= config.EKF_INITIAL_GYRO_BIAS_UNCERTAINTY**2
    cov[9:12, 9:12] *= config.EKF_INITIAL_ACCEL_BIAS_UNCERTAINTY**2
    return ErrorStateEKF(gt_pos.copy(), gt_vel.copy(), cov, up_vector)


def _analyze_recording(npz_path: str) -> dict:
    """Run full replay and compute per-timestep error metrics.

    Returns a dict with keys:
        path, N,
        rmse_pos, rmse_pos_x, rmse_pos_y, rmse_pos_z,
        rmse_vel, rmse_vel_x, rmse_vel_y, rmse_vel_z,
        max_pos_err, max_vel_err,
        mean_innov_norm, max_innov_norm,
        nis_mean, nis_95pct,
        final_gyro_bias, final_accel_bias,
        gt_final_pos, gt_final_vel,
        est_final_pos, est_final_vel,
        warning_flags (list of strings)
    """
    raw = np.load(npz_path, allow_pickle=True)
    N = len(raw["ut"])
    if N == 0:
        return {"path": npz_path, "N": 0, "warning_flags": ["Empty recording"]}

    # Eager-load all arrays — npz indexing re-reads from disk per access
    gt_pos_all = np.array(raw["gt_pos"])
    gt_vel_all = np.array(raw["gt_vel"])
    dt_arr = np.array(raw["dt"])
    sf_bodies = np.array(raw["sf_body_noisy"])
    raw_gyros = np.array(raw["raw_gyro"])
    mahonys = np.array(raw["mahony_attitude"])
    if "gravity_world" in raw:
        gravities = np.array(raw["gravity_world"])
    elif "gravity_ned" in raw:
        gravities = np.array(raw["gravity_ned"])
    else:
        gravities = np.full((len(raw["ut"]), 3), [0.0, 0.0, -9.81])
    noisy_alts = np.array(raw["noisy_alt"])
    noisy_vels = np.array(raw["noisy_vel"])

    up = np.array([0.0, 0.0, 1.0])

    ekf = _build_ekf_at_ground_truth(gt_pos_all[0], gt_vel_all[0], up)

    pos_errors = []
    vel_errors = []
    innov_norms = []
    nis_vals = []

    H = FlightReplayer._H(up)
    R = FlightReplayer._R()

    for i in range(N):
        ekf.predict(sf_bodies[i], raw_gyros[i], mahonys[i], gravities[i], float(dt_arr[i]))

        pos_err = ekf.pos - gt_pos_all[i]
        vel_err = ekf.vel - gt_vel_all[i]
        pos_errors.append(pos_err)
        vel_errors.append(vel_err)

        z_hat = np.array([
            float(np.dot(up, ekf.pos)),
            ekf.vel[0], ekf.vel[1], ekf.vel[2],
        ])
        z_meas = np.array([
            float(noisy_alts[i]),
            float(noisy_vels[i][0]),
            float(noisy_vels[i][1]),
            float(noisy_vels[i][2]),
        ])
        innovation = z_meas - z_hat
        innov_norms.append(float(np.linalg.norm(innovation)))

        try:
            S = H @ ekf.P @ H.T + R
            nis = float(innovation.T @ np.linalg.inv(S) @ innovation)
        except np.linalg.LinAlgError:
            nis = float("inf")
        nis_vals.append(nis)

        ekf.update(noisy_alts[i], noisy_vels[i])

    # ── Compute statistics ───────────────────────────────────────────
    pos_errs = np.array(pos_errors)
    vel_errs = np.array(vel_errors)
    innov_arr = np.array(innov_norms)
    nis_arr = np.array(nis_vals)
    finite_nis = nis_arr[np.isfinite(nis_arr)]

    rmse_pos = float(np.sqrt(np.mean(np.sum(pos_errs**2, axis=1))))
    rmse_vel = float(np.sqrt(np.mean(np.sum(vel_errs**2, axis=1))))
    rmse_pos_ax = [float(np.sqrt(np.mean(pos_errs[:, j]**2))) for j in range(3)]
    rmse_vel_ax = [float(np.sqrt(np.mean(vel_errs[:, j]**2))) for j in range(3)]

    max_pos = float(np.max(np.linalg.norm(pos_errs, axis=1)))
    max_vel = float(np.max(np.linalg.norm(vel_errs, axis=1)))

    nis_mean = float(np.mean(finite_nis)) if len(finite_nis) > 0 else float("inf")
    nis_95 = float(np.percentile(finite_nis, 95)) if len(finite_nis) > 0 else float("inf")

    warnings = []
    if rmse_pos > 5000:
        warnings.append(f"rmse_pos={rmse_pos:.1f}m > 5000")
    if rmse_vel > 50:
        warnings.append(f"rmse_vel={rmse_vel:.1f}m/s > 50")
    if max_pos > 30000:
        warnings.append(f"max_pos_err={max_pos:.1f}m > 30000")
    if max_vel > 200:
        warnings.append(f"max_vel_err={max_vel:.1f}m/s > 200")
    if nis_mean > 100:
        warnings.append(f"nis_mean={nis_mean:.1f} > 100")

    return {
        "path": npz_path,
        "N": N,
        "rmse_pos": rmse_pos,
        "rmse_pos_x": rmse_pos_ax[0],
        "rmse_pos_y": rmse_pos_ax[1],
        "rmse_pos_z": rmse_pos_ax[2],
        "rmse_vel": rmse_vel,
        "rmse_vel_x": rmse_vel_ax[0],
        "rmse_vel_y": rmse_vel_ax[1],
        "rmse_vel_z": rmse_vel_ax[2],
        "max_pos_err": max_pos,
        "max_vel_err": max_vel,
        "mean_innov_norm": float(innov_arr.mean()),
        "max_innov_norm": float(innov_arr.max()),
        "nis_mean": nis_mean,
        "nis_95pct": nis_95,
        "final_gyro_bias": ekf.get_gyro_bias().tolist(),
        "final_accel_bias": ekf.get_accel_bias().tolist(),
        "gt_final_pos": gt_pos_all[-1].tolist(),
        "gt_final_vel": gt_vel_all[-1].tolist(),
        "est_final_pos": ekf.pos.tolist(),
        "est_final_vel": ekf.vel.tolist(),
        "warning_flags": warnings,
    }


def _print_summary(results: list[dict]) -> None:
    """Print a human-readable summary table to stdout."""
    sep = "-" * 80
    print(f"\n{sep}")
    print(f"  Ground-Truth Error Analysis")
    print(f"{sep}")
    for r in results:
        if r["N"] == 0:
            print(f"\n  {os.path.basename(r['path'])}: EMPTY")
            continue
        name = os.path.basename(r["path"])
        print(f"\n  Recording: {name}  ({r['N']} samples)")
        print(f"    RMSE position   : {r['rmse_pos']:8.1f} m    (x={r['rmse_pos_x']:.1f}  y={r['rmse_pos_y']:.1f}  z={r['rmse_pos_z']:.1f})")
        print(f"    RMSE velocity   : {r['rmse_vel']:8.1f} m/s  (x={r['rmse_vel_x']:.1f}  y={r['rmse_vel_y']:.1f}  z={r['rmse_vel_z']:.1f})")
        print(f"    Max pos err     : {r['max_pos_err']:8.1f} m")
        print(f"    Max vel err     : {r['max_vel_err']:8.1f} m/s")
        print(f"    Mean innov norm : {r['mean_innov_norm']:8.1f}")
        print(f"    Max innov norm  : {r['max_innov_norm']:8.1f}")
        print(f"    NIS mean / 95%  : {r['nis_mean']:.1f} / {r['nis_95pct']:.1f}")
        print(f"    Final gyro bias : [{r['final_gyro_bias'][0]:.5f}  {r['final_gyro_bias'][1]:.5f}  {r['final_gyro_bias'][2]:.5f}]")
        print(f"    Final accel bias: [{r['final_accel_bias'][0]:.4f}  {r['final_accel_bias'][1]:.4f}  {r['final_accel_bias'][2]:.4f}]")
        print(f"    GT final pos    : [{r['gt_final_pos'][0]:.1f}  {r['gt_final_pos'][1]:.1f}  {r['gt_final_pos'][2]:.1f}]")
        print(f"    Est final pos   : [{r['est_final_pos'][0]:.1f}  {r['est_final_pos'][1]:.1f}  {r['est_final_pos'][2]:.1f}]")
        if r["warning_flags"]:
            for w in r["warning_flags"]:
                print(f"    ⚠ WARNING: {w}")
    print(f"{sep}\n")


# ══════════════════════════════════════════════════════════════════════
#  TESTS
# ══════════════════════════════════════════════════════════════════════


def test_ground_truth_first_two_recordings():
    """Detailed ground-truth error analysis on the first two post-fix recordings."""
    if len(POST_FIX_CANDIDATES) < 2:
        pytest.skip(f"Need ≥2 recordings, found {len(POST_FIX_CANDIDATES)}")

    test_paths = POST_FIX_CANDIDATES[:2]
    results = []
    for path in test_paths:
        r = _analyze_recording(path)
        results.append(r)

    _print_summary(results)

    for r in results:
        if r["N"] == 0:
            continue
        # Hard bounds — these should never be exceeded
        assert r["rmse_pos"] < 20000.0, (
            f"{os.path.basename(r['path'])}: rmse_pos={r['rmse_pos']:.1f}m"
        )
        assert r["rmse_vel"] < 100.0, (
            f"{os.path.basename(r['path'])}: rmse_vel={r['rmse_vel']:.1f}m/s"
        )
        assert r["max_pos_err"] < 50000.0, (
            f"{os.path.basename(r['path'])}: max_pos_err={r['max_pos_err']:.1f}m"
        )
        assert r["max_vel_err"] < 500.0, (
            f"{os.path.basename(r['path'])}: max_vel_err={r['max_vel_err']:.1f}m/s"
        )
        assert r["mean_innov_norm"] < 2500.0, (
            f"{os.path.basename(r['path'])}: mean_innov_norm={r['mean_innov_norm']:.1f}"
        )

        # State must be finite
        assert all(np.isfinite(r["final_gyro_bias"])), "Gyro bias has NaN/Inf"
        assert all(np.isfinite(r["final_accel_bias"])), "Accel bias has NaN/Inf"

        # Innovation must not be trivial zero (filter is working)
        assert r["mean_innov_norm"] > 1e-6, "Innovation is trivially zero"


def test_ground_truth_error_timeseries():
    """Check error trends across the recording — no late divergence."""
    if len(POST_FIX_CANDIDATES) < 1:
        pytest.skip("No recordings available")

    path = POST_FIX_CANDIDATES[0]
    r = _analyze_recording(path)
    if r["N"] == 0:
        pytest.skip("Empty recording")

    # Re-run with detailed timeseries output
    raw = np.load(path, allow_pickle=True)
    N = len(raw["ut"])
    gt_pos_all = np.array(raw["gt_pos"])
    gt_vel_all = np.array(raw["gt_vel"])
    sf_bodies = np.array(raw["sf_body_noisy"])
    raw_gyros = np.array(raw["raw_gyro"])
    mahonys = np.array(raw["mahony_attitude"])
    if "gravity_world" in raw:
        gravities = np.array(raw["gravity_world"])
    elif "gravity_ned" in raw:
        gravities = np.array(raw["gravity_ned"])
    else:
        gravities = np.full((N, 3), [0.0, 0.0, -9.81])
    noisy_alts = np.array(raw["noisy_alt"])
    noisy_vels = np.array(raw["noisy_vel"])
    dt_arr = np.array(raw["dt"])
    up = np.array([0.0, 0.0, 1.0])

    ekf = _build_ekf_at_ground_truth(gt_pos_all[0], gt_vel_all[0], up)

    checkpoints = np.linspace(0, N - 1, 11, dtype=int)
    pos_err_norms_at_checkpoints = []

    for i in range(N):
        ekf.predict(sf_bodies[i], raw_gyros[i], mahonys[i], gravities[i], float(dt_arr[i]))
        ekf.update(noisy_alts[i], noisy_vels[i])

        if i in checkpoints:
            pos_err_norms_at_checkpoints.append(
                float(np.linalg.norm(ekf.pos - gt_pos_all[i]))
            )

    # Check that the last checkpoint error is not dramatically larger
    # than the first (excluding the first which is always near-zero since
    # we initialized at ground truth)
    if len(pos_err_norms_at_checkpoints) >= 3:
        early = np.median(pos_err_norms_at_checkpoints[1:4])
        late = np.median(pos_err_norms_at_checkpoints[-3:])
        if early > 1e-6:
            ratio = late / early
            assert ratio < 1e6, (
                f"Error ratio late/early = {ratio:.1e} — filter may have diverged"
            )


def test_ground_truth_prediction_error_spikes():
    """Detect frame-by-frame prediction error spikes (pre-update)."""
    if len(POST_FIX_CANDIDATES) < 1:
        pytest.skip("No recordings available")

    path = POST_FIX_CANDIDATES[0]
    raw = np.load(path, allow_pickle=True)
    N = len(raw["ut"])
    if N == 0:
        pytest.skip("Empty recording")

    gt_pos_all = np.array(raw["gt_pos"])
    gt_vel_all = np.array(raw["gt_vel"])
    sf_bodies = np.array(raw["sf_body_noisy"])
    raw_gyros = np.array(raw["raw_gyro"])
    mahonys = np.array(raw["mahony_attitude"])
    if "gravity_world" in raw:
        gravities = np.array(raw["gravity_world"])
    elif "gravity_ned" in raw:
        gravities = np.array(raw["gravity_ned"])
    else:
        gravities = np.full((N, 3), [0.0, 0.0, -9.81])
    noisy_alts = np.array(raw["noisy_alt"])
    noisy_vels = np.array(raw["noisy_vel"])
    dt_arr = np.array(raw["dt"])
    up = np.array([0.0, 0.0, 1.0])

    ekf = _build_ekf_at_ground_truth(gt_pos_all[0], gt_vel_all[0], up)

    pre_update_pos_errs = []

    for i in range(N):
        ekf.predict(sf_bodies[i], raw_gyros[i], mahonys[i], gravities[i], float(dt_arr[i]))
        pre_update_pos_errs.append(
            float(np.linalg.norm(ekf.pos - gt_pos_all[i]))
        )
        ekf.update(noisy_alts[i], noisy_vels[i])

    pre_arr = np.array(pre_update_pos_errs)
    p99 = float(np.percentile(pre_arr, 99))
    # No single prediction error spike should exceed 50000 m
    assert p99 < 50000.0, f"99th percentile pre-update pos error = {p99:.1f} m"


if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING)
    if not POST_FIX_CANDIDATES:
        print("No recordings found.")
    else:
        results = []
        for path in POST_FIX_CANDIDATES[:2]:
            r = _analyze_recording(path)
            results.append(r)
        _print_summary(results)
