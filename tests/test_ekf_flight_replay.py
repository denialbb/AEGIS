"""
Flight-replay integration tests for the Error-State EKF.

Loads each recorded flight from ``recordings/``, runs the full EKF loop
through ``FlightReplayer``, and checks that:

1. The filter does not crash on any recording.
2. Innovation norms are bounded for recordings made AFTER the gravity-bug fix
   (accelerometer_sensor.py now uses body-centered position).
3. For legacy recordings (pre-fix), innovation may be large but the EKF must
   still produce finite state and not crash.
4. Replay is deterministic.

Gravity-bug fix reference
─────────────────────────
Older recordings had gravity_world at ~1e12 m/s² because
``vessel.position(ref_frame)`` used the pad-relative origin instead of the
body-centered frame.  This was fixed in ``accelerometer_sensor.py:85``.
"""

import numpy as np
import os
import pytest

from src.simulation.flights import FlightReplayer
from src.estimation.ekf import ErrorStateEKF


RECORDING_DIR = "recordings"
ALL_RECORDINGS = sorted(
    os.path.join(RECORDING_DIR, f)
    for f in os.listdir(RECORDING_DIR)
    if f.endswith(".npz")
)

NO_RECORDINGS_REASON = (
    "No recordings found in recordings/. "
    "Use KSP + flight_recorder.py to record new flights after the "
    "gravity-bug fix in accelerometer_sensor.py."
)

# ── Recording classification ──────────────────────────────────────────
# After the gravity fix (body-centered position for gravity computation),
# new recordings should have gravity_world norms ~9.81 m/s².
# Legacy pre-fix recordings had norms in the 1e4–1e12 range.
# We detect post-fix recordings by checking gravity norm << 1e4.
_POST_FIX_GRAVITY_NORM_MAX = 1e4

POST_FIX_RECORDINGS = []
LEGACY_RECORDINGS = []

for p in ALL_RECORDINGS:
    try:
        d = np.load(p, allow_pickle=True)
        g = np.array(d["gravity_world"])
        g_mean_norm = float(np.mean(np.linalg.norm(g.reshape(-1, 3), axis=1)))
        if g_mean_norm < _POST_FIX_GRAVITY_NORM_MAX:
            POST_FIX_RECORDINGS.append(p)
        else:
            LEGACY_RECORDINGS.append(p)
    except Exception:
        LEGACY_RECORDINGS.append(p)


# ══════════════════════════════════════════════════════════════════════
#  HELPERS
# ══════════════════════════════════════════════════════════════════════


def _build_ekf() -> ErrorStateEKF:
    """Return a default EKF initialised at the origin with modest cov."""
    return ErrorStateEKF(
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0]),
        np.eye(12) * 0.1,
    )


def _require_nonempty(npz_path: str) -> FlightReplayer:
    """Load a recording and skip if it has zero samples."""
    rp = FlightReplayer(npz_path)
    if len(rp.data["ut"]) == 0:
        pytest.skip(f"{os.path.basename(npz_path)} has 0 samples")
    return rp


def _compute_innovation_stats(replayer: FlightReplayer,
                              ekf: ErrorStateEKF) -> dict:
    """Run a replay and return per-step innovation stats.

    Returns
    -------
    dict with keys: mean_innov_norm, max_innov_norm, innov_norms (list),
    nis_90pct, nis_mean, final_gyro_bias, final_accel_bias.
    """
    data = replayer.data
    ut = np.array(data["ut"])
    dt_arr = np.array(data["dt"])
    sf_body_noisy = data["sf_body_noisy"]
    raw_gyro = data["raw_gyro"]
    mahony_att = np.array(data["mahony_attitude"])
    if "gravity_world" in data:
        gravity_world = np.array(data["gravity_world"])
    elif "gravity_ned" in data:
        gravity_world = np.array(data["gravity_ned"])
    else:
        gravity_world = np.full((len(ut), 3), [0.0, 0.0, -9.81])
    noisy_alt = np.array(data["noisy_alt"])
    noisy_vel = np.array(data["noisy_vel"])

    up = np.array([0.0, 0.0, 1.0])
    H = replayer._H(up)
    R = replayer._R()

    innov_norms = []
    nis_vals = []
    N = len(ut)

    for i in range(N):
        f_body = np.array(sf_body_noisy[i])
        omega = np.array(raw_gyro[i])
        attitude = np.array(mahony_att[i])
        g_world = np.array(gravity_world[i])
        dt_i = float(dt_arr[i])

        ekf.predict(f_body, omega, attitude, g_world, dt_i)

        z_hat = np.array([
            float(np.dot(up, ekf.pos)),
            ekf.vel[0], ekf.vel[1], ekf.vel[2],
        ])
        z_meas = np.array([
            noisy_alt[i],
            noisy_vel[i, 0], noisy_vel[i, 1], noisy_vel[i, 2],
        ])
        innovation = z_meas - z_hat
        innov_norm = float(np.linalg.norm(innovation))
        innov_norms.append(innov_norm)

        try:
            S = H @ ekf.P @ H.T + R
            nis = float(innovation.T @ np.linalg.inv(S) @ innovation)
        except np.linalg.LinAlgError:
            nis = float("inf")
        nis_vals.append(nis)

        ekf.update(noisy_alt[i], noisy_vel[i])

    innov_arr = np.array(innov_norms)
    nis_arr = np.array(nis_vals)
    finite_nis = nis_arr[np.isfinite(nis_arr)]

    return {
        "mean_innov_norm": float(innov_arr.mean()),
        "max_innov_norm": float(innov_arr.max()),
        "innov_norms": innov_arr,
        "nis_mean": float(finite_nis.mean()) if len(finite_nis) > 0 else float("inf"),
        "nis_90pct": float(np.percentile(finite_nis, 90)) if len(finite_nis) > 0 else float("inf"),
        "final_gyro_bias": ekf.get_gyro_bias().copy(),
        "final_accel_bias": ekf.get_accel_bias().copy(),
        "final_pos": ekf.pos.copy(),
        "final_vel": ekf.vel.copy(),
    }


# ══════════════════════════════════════════════════════════════════════
#  TESTS — ALL RECORDINGS
# ══════════════════════════════════════════════════════════════════════


@pytest.mark.skipif(not ALL_RECORDINGS, reason=NO_RECORDINGS_REASON)
class TestFlightReplaySmoke:
    """Each recording: run a full replay and verify completeness + finite state."""

    @pytest.mark.parametrize("npz_path", ALL_RECORDINGS,
                             ids=os.path.basename)
    def test_replay(self, npz_path: str):
        replayer = _require_nonempty(npz_path)
        ekf = _build_ekf()
        replayer.evaluate(ekf)
        # Verify finite state (reuses the same replay)
        assert np.all(np.isfinite(ekf.pos)), f"Position not finite: {ekf.pos}"
        assert np.all(np.isfinite(ekf.vel)), f"Velocity not finite: {ekf.vel}"
        assert np.all(np.isfinite(ekf.bg)), f"Gyro bias not finite: {ekf.bg}"
        assert np.all(np.isfinite(ekf.ba)), f"Accel bias not finite: {ekf.ba}"


# ══════════════════════════════════════════════════════════════════════
#  TESTS — POST-FIX RECORDINGS (gravity is physically correct)
# ══════════════════════════════════════════════════════════════════════


@pytest.mark.skipif(not POST_FIX_RECORDINGS, reason=(
    "No post-fix recordings found. "
    "Record new flights after the gravity-bug fix using flight_recorder.py."
))
class TestFlightReplayPostFix:
    """Post-fix recordings should produce bounded estimation errors."""

    @pytest.mark.parametrize("npz_path", POST_FIX_RECORDINGS[:2],
                             ids=os.path.basename)
    def test_innovation_not_diverged(self, npz_path: str):
        """Mean innovation norm should be below 1e4 (not diverged to infinity)."""
        replayer = _require_nonempty(npz_path)
        ekf = _build_ekf()
        stats = _compute_innovation_stats(replayer, ekf)
        assert stats["mean_innov_norm"] < 1e4, (
            f"{os.path.basename(npz_path)}: mean innov {stats['mean_innov_norm']:.3f} "
            f"exceeds 1e4"
        )

    @pytest.mark.parametrize("npz_path", POST_FIX_RECORDINGS[:2],
                             ids=os.path.basename)
    def test_innovation_settles(self, npz_path: str):
        """The last 10% of innovation norms should not be dramatically
        larger than the first 10% (no late divergence)."""
        replayer = _require_nonempty(npz_path)
        ekf = _build_ekf()
        stats = _compute_innovation_stats(replayer, ekf)
        innov_arr = stats["innov_norms"]
        N = len(innov_arr)
        first_decile = np.median(innov_arr[:max(N // 10, 1)])
        last_decile = np.median(innov_arr[-max(N // 10, 1):])
        ratio = last_decile / max(first_decile, 1e-10)
        assert ratio < 1e6, (
            f"{os.path.basename(npz_path)}: last/first innov ratio = {ratio:.1e} "
            f"— filter may have diverged"
        )

    @pytest.mark.parametrize("npz_path", POST_FIX_RECORDINGS[:2],
                             ids=os.path.basename)
    def test_rmse_reasonable(self, npz_path: str):
        """Position RMSE should be below 20000 m; velocity RMSE below 100 m/s."""
        replayer = _require_nonempty(npz_path)
        ekf = _build_ekf()
        result = replayer.evaluate(ekf)
        assert result["rmse_pos"] < 20000.0, (
            f"{os.path.basename(npz_path)}: rmse_pos={result['rmse_pos']:.1f} m"
        )
        assert result["rmse_vel"] < 100.0, (
            f"{os.path.basename(npz_path)}: rmse_vel={result['rmse_vel']:.1f} m/s"
        )

    def test_nonzero_innovation(self):
        """At least one post-fix recording should have non-zero innovation."""
        any_nonzero = False
        for npz_path in POST_FIX_RECORDINGS[:2]:
            replayer = _require_nonempty(npz_path)
            ekf = _build_ekf()
            stats = _compute_innovation_stats(replayer, ekf)
            if stats["mean_innov_norm"] > 1e-6:
                any_nonzero = True
                break
        assert any_nonzero, "All recordings had near-zero innovation"


# ══════════════════════════════════════════════════════════════════════
#  TESTS — LEGACY (pre-fix) RECORDINGS
# ══════════════════════════════════════════════════════════════════════


@pytest.mark.skipif(not LEGACY_RECORDINGS, reason="No legacy recordings.")
class TestFlightReplayLegacy:
    """Legacy (pre-fix) recordings have corrupted gravity but must not
    crash the EKF."""

    @pytest.mark.parametrize("npz_path", LEGACY_RECORDINGS,
                             ids=os.path.basename)
    def test_legacy_does_not_crash(self, npz_path: str):
        replayer = _require_nonempty(npz_path)
        ekf = _build_ekf()
        replayer.evaluate(ekf)

    @pytest.mark.parametrize("npz_path", LEGACY_RECORDINGS,
                             ids=os.path.basename)
    def test_legacy_state_finite(self, npz_path: str):
        replayer = _require_nonempty(npz_path)
        ekf = _build_ekf()
        stats = _compute_innovation_stats(replayer, ekf)
        assert np.all(np.isfinite(stats["final_pos"]))
        assert np.all(np.isfinite(stats["final_vel"]))
        assert np.all(np.isfinite(stats["final_gyro_bias"]))
        assert np.all(np.isfinite(stats["final_accel_bias"]))

    @pytest.mark.parametrize("npz_path", LEGACY_RECORDINGS,
                             ids=os.path.basename)
    def test_legacy_large_innovation(self, npz_path: str):
        """Legacy recordings have corrupted gravity → large innovation."""
        replayer = _require_nonempty(npz_path)
        ekf = _build_ekf()
        stats = _compute_innovation_stats(replayer, ekf)
        assert stats["mean_innov_norm"] > 1.0, (
            f"{os.path.basename(npz_path)}: mean innov {stats['mean_innov_norm']:.1f} "
            f"should be large for legacy data"
        )


# ══════════════════════════════════════════════════════════════════════
#  TESTS — DETERMINISM
# ══════════════════════════════════════════════════════════════════════


@pytest.mark.skipif(not ALL_RECORDINGS, reason=NO_RECORDINGS_REASON)
class TestFlightReplayDeterminism:
    """Replaying the same recording twice must produce identical results."""

    @pytest.mark.parametrize("npz_path", ALL_RECORDINGS[:5],
                             ids=os.path.basename)
    def test_deterministic_replay(self, npz_path: str):
        replayer = FlightReplayer(npz_path)
        ekf_a = _build_ekf()
        ekf_b = _build_ekf()
        result_a = replayer.evaluate(ekf_a)
        result_b = replayer.evaluate(ekf_b)
        for key in ("rmse_pos", "rmse_vel", "nis", "score"):
            np.testing.assert_allclose(result_a[key], result_b[key],
                                       err_msg=f"{key} differs between runs")


# ══════════════════════════════════════════════════════════════════════
#  SUMMARY
# ══════════════════════════════════════════════════════════════════════


class TestFlightReplaySummary:
    """Ensures at least some recordings exist to make the tests useful."""

    def test_post_fix_recordings_should_exist(self):
        if len(ALL_RECORDINGS) == 0:
            pytest.skip(
                "No recordings found.  Run flight_recorder.py in KSP after "
                "the gravity-bug fix to create usable test data."
            )
        # Warn if there are no post-fix recordings
        if len(POST_FIX_RECORDINGS) == 0:
            import warnings
            warnings.warn(
                "All existing recordings are legacy (pre-fix).  Re-record after "
                "the gravity-bug fix for meaningful innovation/RMSE bounds."
            )
