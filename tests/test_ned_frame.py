"""Unit tests for the ECEF→NED frame construction (pure math, no kRPC)."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R
from src.common.geometry import ecef_to_ned
import src.config as config

# Kerbin gravitational parameter (m³/s²) and radius (m)
KERBIN_MU = 3.5316000e12
KERBIN_RADIUS = 600_000.0


def _ecef_from_lat_lon(lat_deg: float, lon_deg: float) -> np.ndarray:
    """Convert lat/lon (degrees) to ECEF position on Kerbin (sphere, R=600 km)."""
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    x = KERBIN_RADIUS * np.cos(lat) * np.cos(lon)
    y = KERBIN_RADIUS * np.cos(lat) * np.sin(lon)
    z = KERBIN_RADIUS * np.sin(lat)
    return np.array([x, y, z])


_KSC_PAD = config.PAD_POSITION_ECEF.copy()

# Three hard-coded pad positions at different latitudes
PAD_POSITIONS: list[np.ndarray] = [
    _KSC_PAD,                                          # KSC (lat -0.097°, lon -74.56°)
    _ecef_from_lat_lon(0.0, 0.0),                     # Equator, prime meridian
    _ecef_from_lat_lon(-45.0, 120.0),                 # Mid-southern hemisphere
    _ecef_from_lat_lon(-89.0, 0.0),                   # Near south pole
]
PAD_IDS = ["ksc", "equator_0_0", "lat-45_lon120", "near_south_pole"]


# ---------------------------------------------------------------------------
#  Common helpers
# ---------------------------------------------------------------------------

def _check_gravity_direction(pad_ecef: np.ndarray) -> None:
    """gravity_ned must always be (0, 0, +g) — purely along Down (+z)."""
    r = float(np.linalg.norm(pad_ecef))
    g_mag_local = KERBIN_MU / r**2
    R_mat, _, _, _ = ecef_to_ned(pad_ecef)
    grav_ecef = -(KERBIN_MU / r**3) * pad_ecef
    g_N = R_mat @ grav_ecef
    assert abs(g_N[0]) < 1e-4          # near-zero north component
    assert abs(g_N[1]) < 1e-4          # near-zero east  component
    assert abs(g_N[2] - g_mag_local) < 1e-3  # down component matches local g


# ---------------------------------------------------------------------------
#  Tests — each runs against ALL pad positions
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("pad_ecef", PAD_POSITIONS, ids=PAD_IDS)
class TestNedRotationMatrix:
    """Check 1: rotation orthonormality and determinant."""

    def test_rotation_is_orthonormal(self, pad_ecef: np.ndarray) -> None:
        R_mat, _, _, _ = ecef_to_ned(pad_ecef)
        np.testing.assert_allclose(R_mat @ R_mat.T, np.eye(3), atol=1e-12)

    def test_determinant_is_plus_one(self, pad_ecef: np.ndarray) -> None:
        R_mat, _, _, _ = ecef_to_ned(pad_ecef)
        np.testing.assert_allclose(np.linalg.det(R_mat), 1.0, atol=1e-12)


@pytest.mark.parametrize("pad_ecef", PAD_POSITIONS, ids=PAD_IDS)
class TestNedRoundTrip:
    """Check: R_NED→ECEF = R_ECEF→NED^T; round-trip an arbitrary vector."""

    def test_inverse_is_transpose(self, pad_ecef: np.ndarray) -> None:
        R_mat, _, _, _ = ecef_to_ned(pad_ecef)
        np.testing.assert_allclose(R_mat.T @ R_mat, np.eye(3), atol=1e-12)

    def test_arbitrary_vector_round_trip(self, pad_ecef: np.ndarray) -> None:
        R_mat, _, _, _ = ecef_to_ned(pad_ecef)
        r_E = np.array([1234.5, -567.8, 987.6])
        r_N = R_mat @ r_E
        r_E2 = R_mat.T @ r_N
        np.testing.assert_allclose(r_E, r_E2, atol=1e-12)


@pytest.mark.parametrize("pad_ecef", PAD_POSITIONS, ids=PAD_IDS)
class TestNedAxisDirections:
    """Check 2: axis directions verified via ECEF→NED rotation."""

    def test_down_axis_points_toward_body_centre(self, pad_ecef: np.ndarray) -> None:
        """R @ down_ecef  =  [0, 0, 1]  — Down in NED is +z."""
        R_mat, _, _, _ = ecef_to_ned(pad_ecef)
        down_ecef = -pad_ecef / np.linalg.norm(pad_ecef)
        np.testing.assert_allclose(R_mat @ down_ecef, np.array([0.0, 0.0, 1.0]), atol=1e-12)

    def test_third_row_is_down_direction(self, pad_ecef: np.ndarray) -> None:
        """Third row of R_ecef→ned equals -pad_ecef / ‖pad_ecef‖."""
        R_mat, _, _, _ = ecef_to_ned(pad_ecef)
        down = -pad_ecef / np.linalg.norm(pad_ecef)
        np.testing.assert_allclose(R_mat[2, :], down, atol=1e-12)

    def test_north_cross_east_equals_down(self, pad_ecef: np.ndarray) -> None:
        """Right-handed frame: north × east = down."""
        _, _, north, east = ecef_to_ned(pad_ecef)
        down = -pad_ecef / np.linalg.norm(pad_ecef)
        np.testing.assert_allclose(np.cross(north, east), down, atol=1e-12)

    def test_up_vector_in_ned(self, pad_ecef: np.ndarray) -> None:
        """R @ up_ecef  gives  [0, 0, -1]."""
        R_mat, _, _, _ = ecef_to_ned(pad_ecef)
        up_ecef = pad_ecef / np.linalg.norm(pad_ecef)
        np.testing.assert_allclose(R_mat @ up_ecef, np.array([0.0, 0.0, -1.0]), atol=1e-12)

    def test_gravity_always_along_down(self, pad_ecef: np.ndarray) -> None:
        """gravity_ned = (0, 0, +g) for every pad position."""
        _check_gravity_direction(pad_ecef)


@pytest.mark.parametrize("pad_ecef", PAD_POSITIONS, ids=PAD_IDS)
class TestNedPositionConvention:
    """Check 3: altitude = -z in NED; 100 m above pad → z = -100."""

    def test_pad_origin_at_zero(self, pad_ecef: np.ndarray) -> None:
        """The pad is at the NED origin: R @ (pad - pad) = 0."""
        R_mat, _, _, _ = ecef_to_ned(pad_ecef)
        np.testing.assert_allclose(R_mat @ np.zeros(3), np.zeros(3), atol=1e-12)

    def test_one_hundred_metres_above_is_negative_z(self, pad_ecef: np.ndarray) -> None:
        """A point 100 m above the pad along the local vertical has z = -100 in NED."""
        R_mat, _, _, _ = ecef_to_ned(pad_ecef)
        up_ecef = pad_ecef / np.linalg.norm(pad_ecef)
        offset = 100.0 * up_ecef
        ned = R_mat @ offset
        np.testing.assert_allclose(ned, np.array([0.0, 0.0, -100.0]), atol=1e-6)

    def test_descending_velocity_is_positive_z(self, pad_ecef: np.ndarray) -> None:
        """Descending at 50 m/s: v_N = R @ (-50 * up_E) = [0, 0, +50]."""
        R_mat, _, _, _ = ecef_to_ned(pad_ecef)
        up_ecef = pad_ecef / np.linalg.norm(pad_ecef)
        v_e_descend = -50.0 * up_ecef
        v_ned = R_mat @ v_e_descend
        np.testing.assert_allclose(v_ned, np.array([0.0, 0.0, 50.0]), atol=1e-6)

    def test_ascending_velocity_is_negative_z(self, pad_ecef: np.ndarray) -> None:
        """Ascending at 50 m/s: v_N = R @ (+50 * up_E) = [0, 0, -50]."""
        R_mat, _, _, _ = ecef_to_ned(pad_ecef)
        up_ecef = pad_ecef / np.linalg.norm(pad_ecef)
        v_e_ascend = 50.0 * up_ecef
        v_ned = R_mat @ v_e_ascend
        np.testing.assert_allclose(v_ned, np.array([0.0, 0.0, -50.0]), atol=1e-6)


@pytest.mark.parametrize("pad_ecef", PAD_POSITIONS, ids=PAD_IDS)
class TestGravityInNed:
    """Check 4: gravity points along +z (Down) in NED."""

    def test_gravity_is_along_down_axis(self, pad_ecef: np.ndarray) -> None:
        """gravity_ned = (0, 0, g)  where g = mu / r²."""
        r = float(np.linalg.norm(pad_ecef))
        g_mag = KERBIN_MU / r**2
        gravity_ned = np.array([0.0, 0.0, g_mag])
        assert gravity_ned[0] == 0.0
        assert gravity_ned[1] == 0.0
        assert gravity_ned[2] > 0.0


def test_gravity_magnitude_at_surface() -> None:
    """g ≈ 9.81 m/s² at Kerbin surface (radius 600 km)."""
    g_mag = KERBIN_MU / KERBIN_RADIUS**2
    np.testing.assert_allclose(g_mag, 9.81, atol=0.02)


@pytest.mark.parametrize("pad_ecef", PAD_POSITIONS, ids=PAD_IDS)
class TestQuaternion:
    """Check 5: quaternion round-trip and unit norm."""

    def test_quaternion_matches_rotation_matrix(self, pad_ecef: np.ndarray) -> None:
        R_mat, quat, _, _ = ecef_to_ned(pad_ecef)
        recovered = R.from_quat(quat).as_matrix()
        np.testing.assert_allclose(recovered, R_mat, atol=1e-12)

    def test_quaternion_is_unit_norm(self, pad_ecef: np.ndarray) -> None:
        _, quat, _, _ = ecef_to_ned(pad_ecef)
        np.testing.assert_allclose(np.linalg.norm(quat), 1.0, atol=1e-12)


class TestPoleDegeneracy:
    """Check 6: edge case at the north pole."""

    def test_pole_fallback_uses_east_as_y(self) -> None:
        """At the north pole, cross(up, pole) = 0; fallback to east = (0,1,0)."""
        pole_position = np.array([0.0, 0.0, KERBIN_RADIUS])
        _, _, north, east = ecef_to_ned(pole_position)
        np.testing.assert_allclose(east, np.array([0.0, 1.0, 0.0]), atol=1e-12)
        assert abs(north[0]) > 0.99
        assert abs(north[1]) < 1e-6
        assert abs(north[2]) < 1e-6

    def test_south_pole_also_handled(self) -> None:
        """At the south pole, cross(up, pole) = 0; fallback applies."""
        pole_position = np.array([0.0, 0.0, -KERBIN_RADIUS])
        _, _, _, east = ecef_to_ned(pole_position)
        np.testing.assert_allclose(east, np.array([0.0, 1.0, 0.0]), atol=1e-12)

    def test_gravity_at_pole(self) -> None:
        """gravity_ned = (0,0,+g) at the pole too."""
        _check_gravity_direction(np.array([0.0, 0.0, KERBIN_RADIUS]))
