import numpy as np
from src.common.engine import Engine
from src.simulation.mock_vessel import MockVessel, SimpleTestVessel


def test_simple_total_mass_linearity():
    vessel = SimpleTestVessel()
    dry = vessel.dry_mass
    assert vessel.total_mass(0.0) == dry
    assert vessel.total_mass(5.0) == dry + 5.0
    assert vessel.total_mass(10.0) == dry + 10.0


def test_simple_fuel_burn_rate_zero_at_zero_throttle():
    vessel = SimpleTestVessel()
    assert vessel.get_fuel_burn_rate(np.zeros(1)) == 0.0


def test_simple_fuel_burn_rate_linear_in_throttles():
    vessel = SimpleTestVessel()
    full = np.array([1.0])
    half = np.array([0.5])
    assert np.isclose(vessel.get_fuel_burn_rate(full), 2.0 * vessel.get_fuel_burn_rate(half))
    assert np.isclose(vessel.get_fuel_burn_rate(full), 1.0)


def test_simple_get_com_position_at_zero_fuel_is_dry_com():
    vessel = SimpleTestVessel()
    np.testing.assert_allclose(vessel.get_com_position(0.0), vessel.dry_com)


def test_simple_get_drag_force_is_anti_parallel_to_velocity():
    vessel = SimpleTestVessel()
    for v in [
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, -1.0]),
        np.array([1.0, -1.0, 1.0]),
    ]:
        assert np.dot(vessel.get_drag_force(v, density=1.225), v) <= 0.0


def test_simple_engines_property_returns_single_engine_instance():
    vessel = SimpleTestVessel()
    engines = vessel.engines
    assert isinstance(engines, list)
    assert len(engines) == 1
    for e in engines:
        assert isinstance(e, Engine)


def test_simple_engine_tau_is_positive_float():
    vessel = SimpleTestVessel()
    assert isinstance(vessel.engine_tau, float)
    assert vessel.engine_tau > 0.0


def test_mock_total_mass_linearity():
    vessel = MockVessel()
    dry = vessel.dry_mass
    assert vessel.total_mass(0.0) == dry
    assert vessel.total_mass(5.0) == dry + 5.0
    assert vessel.total_mass(10.0) == dry + 10.0


def test_mock_fuel_burn_rate_zero_at_zero_throttle():
    vessel = MockVessel()
    assert vessel.get_fuel_burn_rate(np.zeros(4)) == 0.0


def test_mock_fuel_burn_rate_linear_in_throttles():
    vessel = MockVessel()
    full = np.array([0.5, 0.5, 0.5, 0.5])
    half = np.array([0.25, 0.25, 0.25, 0.25])
    assert np.isclose(vessel.get_fuel_burn_rate(full), 2.0 * vessel.get_fuel_burn_rate(half))
    assert np.isclose(vessel.get_fuel_burn_rate(full), 2.0)


def test_mock_get_com_position_at_zero_fuel_is_dry_com():
    vessel = MockVessel()
    np.testing.assert_allclose(vessel.get_com_position(0.0), vessel.dry_com)


def test_mock_get_com_position_at_equal_fuel_interpolates():
    vessel = MockVessel()
    expected = (vessel.dry_com + vessel.fuel_com) / 2.0
    np.testing.assert_allclose(vessel.get_com_position(vessel.dry_mass), expected)


def test_mock_get_com_position_weighted_by_mass():
    vessel = MockVessel()
    fuel_mass = 2.0 * vessel.dry_mass
    expected = (vessel.dry_mass * vessel.dry_com + fuel_mass * vessel.fuel_com) / (vessel.dry_mass + fuel_mass)
    np.testing.assert_allclose(vessel.get_com_position(fuel_mass), expected)


def test_mock_get_drag_force_is_anti_parallel_to_velocity():
    vessel = MockVessel()
    density = 1.225
    for v in [
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, -1.0]),
        np.array([1.0, -1.0, 1.0]),
        np.array([0.0, 0.0, 0.0]),
    ]:
        assert np.dot(vessel.get_drag_force(v, density), v) <= 0.0


def test_mock_engines_property_returns_four_engine_instances():
    vessel = MockVessel()
    engines = vessel.engines
    assert isinstance(engines, list)
    assert len(engines) == 4
    for e in engines:
        assert isinstance(e, Engine)


def test_mock_engines_use_stored_max_thrust():
    vessel = MockVessel()
    for e in vessel.engines:
        assert e.max_thrust == vessel.max_thrust


def test_mock_engine_tau_is_positive_float():
    vessel = MockVessel()
    assert isinstance(vessel.engine_tau, float)
    assert vessel.engine_tau > 0.0


def test_mock_inertia_tensor_uses_stored_diag():
    vessel = MockVessel()
    I = vessel.inertia_tensor(0.0)
    assert I.shape == (3, 3)
    np.testing.assert_allclose(np.diag(I), vessel.inertia_diag)


def test_mock_inertia_tensor_is_diag():
    vessel = MockVessel()
    I = vessel.inertia_tensor(0.0)
    np.testing.assert_allclose(I, np.diag(np.diag(I)))
