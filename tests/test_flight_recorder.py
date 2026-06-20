import pytest
import numpy as np
from pathlib import Path
from unittest.mock import MagicMock
import scripts.flight_recorder as flight_recorder

# Pytest will use the timeout plugin; each test will be killed after the given
# time limit if it runs longer.

@pytest.mark.timeout(3)
def test_flight_recorder_basic(tmp_path, monkeypatch):
    """Verify that the flight recorder writes a single .npz file with the expected keys."""
    # ---------------------------------------------------------------------
    # 1.  Mock the kRPC connection and vessel
    # ---------------------------------------------------------------------
    mock_conn = MagicMock()
    mock_space_center = MagicMock()
    # ``ut`` attribute will be mutated by the recorder; for the test we keep it
    # constant because all timestamps will be identical and still sufficient for
    # the assertion of key existence.
    mock_space_center.ut = 0.0
    mock_conn.space_center = mock_space_center

    # Vessel mock
    mock_vessel = MagicMock()

    class StopLoop(Exception):
        """Raised to break the recorder infinite loop for unit-testing."""
        pass

    # action_group 1 sequence: start recording (True) → stop recording (False)
    # → raise StopLoop to break the infinite while loop gracefully.
    _action_seq = [True, False, "stop"]
    def _get_action_group_side_effect(_):
        val = _action_seq.pop(0)
        if val == "stop":
            raise StopLoop()
        return val
    mock_vessel.control.get_action_group.side_effect = _get_action_group_side_effect
    mock_vessel.control.set_action_group = MagicMock()
    mock_vessel.name = "Test Vessel"
    flight_mock = MagicMock()
    flight_mock.position = [0.0, 0.0, 600000.0]
    flight_mock.velocity = [0.0, 0.0, 0.0]
    mock_vessel.flight = MagicMock(return_value=flight_mock)
    mock_vessel.orbit.body.reference_frame = MagicMock()
    mock_vessel.orbit.body.surface_position = MagicMock(return_value=[0.0, 0.0, 600000.0])
    mock_vessel.parts.with_tag.return_value = []
    mock_vessel.parts.engines.return_value = []
    mock_conn.space_center.active_vessel = mock_vessel

    # -----------------------------------------------------------------
    # 2.  Patch global objects in the recorder module
    # -----------------------------------------------------------------
    # Replace krpc.connect to return our mock connection
    monkeypatch.setattr(
        flight_recorder.krpc,
        "connect",
        lambda *_, **__: mock_conn,
    )

    # Point RECORD_DIR to a temporary directory.
    monkeypatch.setattr(flight_recorder, "RECORD_DIR", str(tmp_path))

    # Replace SensorModels with a deterministic stub.
    class DummySensor:
        def __init__(self):
            # The recorder accesses sensors.gyro_sensor.angular_velocity_stream()
            gyro_mock = MagicMock()
            gyro_mock.angular_velocity_stream = MagicMock(return_value=MagicMock())
            gyro_mock.angular_velocity_stream().x = 0.0
            gyro_mock.angular_velocity_stream().y = 0.0
            gyro_mock.angular_velocity_stream().z = 0.0
            self.gyro_sensor = gyro_mock

        def _read_krpc_quaternion(self):
            return np.array([0.0, 0.0, 0.0, 1.0])

        def get_truth_attitude(self):
            return np.array([0.0, 0.0, 0.0, 1.0])

        def poll(self):
            return (
                1000.0,  # noisy_alt
                np.array([0.1, 0.1, 0.1]),  # sf_body_noisy
                np.array([1.0, 0.0, 0.0, 0.0]),  # mahony_attitude
                5000.0,  # mass
                np.array([0.0, 0.0, 0.0]),  # aero_body
                "flying",  # situation
                np.array([0.0, 0.0, 0.0]),  # omega_body
                np.array([0.0, 0.0, 0.0]),  # noisy_vel
                np.array([0.0, 0.0, -9.81]),  # gravity_world
                np.array([0.0, 0.0, 0.0]),  # raw_gyro
            )

    # The original constructor expects arguments; provide a factory that ignores them
    def dummy_factory(*_, **__) -> DummySensor:
        """Factory returning a DummySensor instance for tests."""
        return DummySensor()

    monkeypatch.setattr(flight_recorder, "SensorModels", dummy_factory)

    # -----------------------------------------------------------------
    # 3.  Execute the recorder – it should start, capture a tick, stop, and exit
    # -----------------------------------------------------------------
    try:
        flight_recorder.main()
    except StopLoop:
        pass

    # ---------------------------------------------------------------------
    # 4.  Assertions
    # ---------------------------------------------------------------------
    files = list(tmp_path.glob("flight_*.npz"))
    assert len(files) == 1, "Recorder did not produce a single .npz file"

    recorded = np.load(files[0])
    expected_keys = {
        "ut",
        "dt",
        "gt_pos",
        "gt_vel",
        "gt_att",
        "noisy_alt",
        "noisy_vel",
        "sf_body_noisy",
        "mahony_attitude",
        "gravity_ned",
        "raw_gyro",
        "mass",
        "aero_body",
        "situation",
        "clean_alt",
        "clean_angular_vel",
        "throttle",
        "active_engine_count",
    }
    assert set(recorded.files) == expected_keys, "Missing keys in the recording"
    # Verify that the gravity vector matches the stubbed value.
    np.testing.assert_allclose(
        recorded["gravity_ned"][0], np.array([0.0, 0.0, -9.81]), rtol=1e-6
    )

    # Ensure the data arrays are non‑empty
    for name in expected_keys:
        assert recorded[name].size > 0, f"Array {name} is empty"
