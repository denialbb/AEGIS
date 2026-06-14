import pytest
import numpy as np
import os
import json
import tempfile
import shutil
from src.telemetry.frame import TelemetryFrame
from src.telemetry.writer import TelemetryWriter

def test_telemetry_frame_flatten():
    frame = TelemetryFrame(
        timestamp=100.5,
        altitude=1000.0,
        velocity=np.array([1.0, 2.0, 3.0]),
        noisy_accel=np.array([0.1, 0.2, 0.3]),
        throttles=np.array([0.5, 0.6]),
        gimbals=np.array([[0.1, 0.2], [0.3, 0.4]])
    )
    flat = frame.flatten()
    
    assert flat["timestamp"] == 100.5
    assert flat["altitude"] == 1000.0
    assert flat["vel_x"] == 1.0
    assert flat["vel_y"] == 2.0
    assert flat["vel_z"] == 3.0
    assert flat["accel_x"] == 0.1
    assert flat["throttle_0"] == 0.5
    assert flat["throttle_1"] == 0.6
    assert flat["gimbal_0_0"] == 0.1
    assert flat["gimbal_0_1"] == 0.2
    assert flat["gimbal_1_0"] == 0.3
    assert flat["gimbal_1_1"] == 0.4

def test_telemetry_frame_flatten_mismatched_arrays():
    # 2 throttles, but only 1 gimbal configured
    frame = TelemetryFrame(
        timestamp=100.5,
        altitude=1000.0,
        velocity=np.array([1.0, 2.0]), # intentionally short 2D velocity vector
        noisy_accel=np.array([0.1, 0.2, 0.3, 0.4]), # intentionally long 4D accel vector
        throttles=np.array([0.5, 0.6]),
        gimbals=np.array([[0.1, 0.2]]) # only 1 engine has gimbal
    )
    flat = frame.flatten()
    
    assert flat["timestamp"] == 100.5
    assert flat["vel_x"] == 1.0
    assert flat["vel_y"] == 2.0
    assert "vel_z" not in flat
    
    assert flat["accel_x"] == 0.1
    assert flat["accel_y"] == 0.2
    assert flat["accel_z"] == 0.3
    # 4th accel component is ignored
    
    assert flat["throttle_0"] == 0.5
    assert flat["throttle_1"] == 0.6
    
    assert flat["gimbal_0_0"] == 0.1
    assert flat["gimbal_0_1"] == 0.2
    assert "gimbal_1_0" not in flat

def test_telemetry_writer_creates_files():
    temp_dir = tempfile.mkdtemp()
    try:
        config = {"seed": 42, "num_engines": 2}
        with TelemetryWriter(run_config=config, base_dir=temp_dir) as writer:
            assert os.path.exists(writer.run_dir_path)
            assert os.path.exists(os.path.join(writer.run_dir_path, "run_config.json"))
            assert os.path.exists(os.path.join(writer.run_dir_path, "telemetry.csv"))
            assert os.path.exists(os.path.join(writer.run_dir_path, "events.jsonl"))
            
            # test latest symlink or directory existence
            latest_path = os.path.join(temp_dir, "latest")
            assert os.path.exists(latest_path)
            
            # verify run config
            with open(os.path.join(writer.run_dir_path, "run_config.json"), "r") as f:
                loaded_config = json.load(f)
                assert loaded_config["seed"] == 42
                
            # verify writing
            frame = TelemetryFrame(
                timestamp=1.0, altitude=100.0,
                velocity=np.array([0., 0., 0.]),
                noisy_accel=np.array([0., 0., 0.]),
                throttles=np.array([1.0, 1.0]),
                gimbals=np.array([[0., 0.], [0., 0.]])
            )
            writer.log_tick(frame)
            writer.log_event({"event": "test"})
            
        # check contents after close
        with open(os.path.join(writer.run_dir_path, "events.jsonl"), "r") as f:
            lines = f.readlines()
            assert len(lines) == 1
            assert json.loads(lines[0])["event"] == "test"
            
        with open(os.path.join(writer.run_dir_path, "telemetry.csv"), "r") as f:
            lines = f.readlines()
            assert len(lines) == 2 # header + 1 row
            assert "timestamp" in lines[0]
            assert "gimbal_1_1" in lines[0]
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)
