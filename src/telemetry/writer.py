import json
import os
import csv
import logging
from datetime import datetime
import types
from typing import Dict, Any, Optional, TextIO

logger = logging.getLogger(__name__)
from .frame import TelemetryFrame

class TelemetryWriter:
    """
    Manages telemetry writing for AEGIS.
    """
    def __init__(self, run_config: Dict[str, Any], base_dir: str = "logs"):
        """
        Initializes the writer. Creates the timestamped directory and setup files.
        """
        self.run_config = run_config
        self.base_dir = base_dir
        
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        seed = run_config.get("seed", 0)
        self.run_dir_name = f"{timestamp_str}_seed{seed}"
        self.run_dir_path = os.path.join(self.base_dir, "runs", self.run_dir_name)
        
        os.makedirs(self.run_dir_path, exist_ok=True)
        
        self._setup_symlink()
        
        # Write run config
        config_path = os.path.join(self.run_dir_path, "run_config.json")
        with open(config_path, "w") as f:
            json.dump(run_config, f, indent=4)
            
        # We need num_engines for CSV headers. Assume it's in run_config.
        self.num_engines = run_config.get("num_engines", 1)
        
        self.telemetry_file: Optional[TextIO] = None
        self.events_file: Optional[TextIO] = None
        self.csv_writer: Optional[csv.DictWriter] = None

        self._init_files()
        
        logger.info(f"Telemetry logging initialized at {self.run_dir_path}")

    def _setup_symlink(self) -> None:
        """
        Creates an atomic symlink logs/latest pointing to the new run dir.
        If atomic replace fails (e.g. on Windows without dev mode), it falls back
        to standard symlink overwrite.
        """
        latest_path = os.path.join(self.base_dir, "latest")
        temp_link = f"{latest_path}_tmp"
        
        # Make the target relative so the symlink is portable
        target = os.path.join("runs", self.run_dir_name)
        
        try:
            if os.path.exists(temp_link) or os.path.islink(temp_link):
                if os.path.isdir(temp_link) and not os.path.islink(temp_link):
                    import shutil
                    shutil.rmtree(temp_link)
                else:
                    os.unlink(temp_link)
                
            os.symlink(target, temp_link, target_is_directory=True)
            # Atomic replace
            os.replace(temp_link, latest_path)
        except OSError as e:
            # Fallback if os.replace fails over symlinks or we have limited permissions
            logger.warning(f"Atomic symlink replace failed: {e}. Falling back to standard replace.")
            if os.path.exists(latest_path) or os.path.islink(latest_path):
                try:
                    if os.path.isdir(latest_path) and not os.path.islink(latest_path):
                        import shutil
                        shutil.rmtree(latest_path)
                    else:
                        os.unlink(latest_path)
                except OSError as e2:
                    logger.warning(f"Failed to remove existing latest symlink: {e2}")
            try:
                os.symlink(target, latest_path, target_is_directory=True)
            except OSError as e3:
                logger.warning(f"Failed to create standard symlink: {e3}")

    def _init_files(self) -> None:
        """
        Initializes the output files.
        Important Decision: We use a 1MB buffer (buffering=1048576) because the control loop
        runs at 50Hz, and blocking I/O calls can induce jitter and violate real-time constraints.
        By buffering, we defer actual disk writes until the buffer is full.
        """
        telemetry_path = os.path.join(self.run_dir_path, "telemetry.csv")
        events_path = os.path.join(self.run_dir_path, "events.jsonl")
        
        # 1MB buffer = 1048576 bytes
        self.telemetry_file = open(telemetry_path, "w", buffering=1048576, newline='')
        self.events_file = open(events_path, "w", buffering=1048576)
        
        headers = TelemetryFrame.get_csv_headers(self.num_engines)
        # Using typing Any for writer to keep it simple, but specify strictly:
        self.csv_writer = csv.DictWriter(self.telemetry_file, fieldnames=headers)
        self.csv_writer.writeheader()

    def log_tick(self, frame: TelemetryFrame) -> None:
        """
        Logs a single telemetry frame to the CSV file.
        """
        if self.csv_writer is not None:
            self.csv_writer.writerow(frame.flatten())

    def log_event(self, event_dict: Dict[str, Any]) -> None:
        """
        Logs an event dict as JSON to the JSONL events file.
        """
        if self.events_file is not None:
            self.events_file.write(json.dumps(event_dict) + "\n")

    def close(self) -> None:
        """
        Flushes and closes the files.
        """
        if self.telemetry_file is not None:
            self.telemetry_file.flush()
            self.telemetry_file.close()
            self.telemetry_file = None
        if self.events_file is not None:
            self.events_file.flush()
            self.events_file.close()
            self.events_file = None

    def __enter__(self) -> 'TelemetryWriter':
        return self

    def __exit__(self, exc_type: Optional[type], exc_val: Optional[BaseException], exc_tb: Optional[types.TracebackType]) -> None:
        self.close()
