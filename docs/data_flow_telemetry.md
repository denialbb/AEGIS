# AEGIS Data Flow and Telemetry Guide

## 1. Physical Execution Topology

KSP is a Windows application. kRPC is a mod that starts a TCP server inside that Windows process — by default on ports 50000 (RPC) and 50001 (streams). The Python code runs in Arch WSL2 (ADR-006). WSL2 is not WSL1 — it runs in a lightweight Hyper-V virtual machine with its own virtual network adapter and its own IP address space, separate from Windows localhost.

This means the Python `krpc.connect(address='localhost', ...)` call inside WSL will NOT reach 127.0.0.1 on Windows by default. It reaches the WSL2 VM's own loopback, which has nothing listening on port 50000. 

The fix is either:
- Enable WSL2 localhost forwarding in `.wslconfig` (`localhostforwarding=true`), 
- Connect to the Windows host IP, which WSL2 exposes via the nameserver entry in `/etc/resolv.conf` (typically 172.x.x.x), or
- Set the kRPC server to accept "Any IP Address" rather than just localhost.

## 2. Data Flow, Tick by Tick

Once connected, kRPC operates on two channels simultaneously:
1. **RPC channel**: Handles imperative calls (set throttle, query vessel mass, command gimbal angle).
2. **Stream channel**: A push feed. KSP pushes updated values to Python at the physics tick rate without Python having to ask. 

The agent code uses `add_stream()` for all high-frequency telemetry (altitude, acceleration, velocity) and reserves direct RPC calls for low-frequency writes (throttle commands, gimbal set).

### A single tick through the control loop:
**KSP physics tick (50Hz, 20ms budget)**
- stream: `noisy_alt` arrives via kRPC stream port
- stream: `noisy_vel` arrives via kRPC stream port
- stream: `sf_body` (specific force) arrives
- stream: `omega_body` (angular velocity) arrives
- stream: `vessel.mass` arrives
- stream: `attitude` (Mahony quaternion) arrives
- stream: `gravity_world` arrives

**Python `run_loop()` wakes:**
- `noise_wrapper(raw_alt)` → `noisy_alt` [inject Gaussian]
- `noise_wrapper(raw_vel)` → `noisy_vel` [inject Gaussian]
- `noise_wrapper(raw_sf_body)` → `sf_body` [inject Gaussian]
- `noise_wrapper(raw_omega_body)` → `omega_body` [inject Gaussian]
- `noise_wrapper(raw_attitude)` → `attitude` [Mahony update uses raw gyro and accel]
- `noise_wrapper(raw_gravity)` → `gravity_world` [if needed, but gravity is from kRPC]

- `mahony_attitude = attitude_estimator.update(omega_body, sf_body, gravity_world, dt)` [Mahony predict step]
- `estimator.predict(sf_body, omega_body, mahony_attitude, gravity_world, dt)` [EKF predict step]
- `estimator.update(noisy_alt, noisy_vel)` [EKF update step]
- `state = estimator.get_state()` 
- `fdi.detect_fault(expected_accel, measured_accel)` 
- if fault: `fdi.isolate_fault(...)` 
- guidance: compute `desired_wrench` from state
- allocator: `allocator.allocate(desired_wrench, active_engines)`
  - build B matrix
  - check cond(B) vs 1e4 threshold
  - pinv(B) @ desired_wrench → throttles, gimbals
- kRPC RPC call: set engine throttles [TCP round trip ~1ms]
- kRPC RPC call: set gimbal angles [TCP round trip ~1ms]
- Mission Director: evaluate state transitions

Total per-tick Python execution time: approximately 3–5ms against a 20ms budget. 

---

## 3. The Telemetry Log: The Real Debug Surface

Traditional interactive debugging (e.g. `pdb` breakpoint) is incompatible with a real-time control loop. If the Python process pauses, KSP physics continues, throwing off the Kalman filter `dt` calculations. 

The agent's primary debug surface is a **structured telemetry file** written every tick in buffered mode. 

### File Structure
Three files per run are written to a timestamped directory:
```text
logs/
 └── runs/
      └── 20260613_153042_seed42/
           ├── telemetry.csv     ← per-tick structured data, one row per loop iteration
           ├── events.jsonl      ← discrete events only (faults, transitions, dt spikes)
           └── run_config.json   ← initial conditions: noise params, thresholds, seed
```
A symlink `logs/latest/` always points to the most recent run. The agent can continuously monitor this folder via `tail -f logs/latest/telemetry.csv`.

### The Events Log
One JSON object per line, written only when something significant happens:
```json
{"type": "STATE_TRANSITION", "from": "POWERED_DESCENT", "to": "TERMINAL_DESCENT"}
{"type": "FAULT_DETECTED", "engine_index": 2}
{"type": "STATE_TRANSITION", "from": "TERMINAL_DESCENT", "to": "HARD_ABORT", "reason": "VESSEL_DESTROYED"}
```
This is the high-signal view. The agent reads this file after a run to understand the timeline, and dips into `telemetry.csv` for high-resolution tick data around specific moments (e.g., exactly when an engine failed).

### KSP Pause / DT Spike Handling
When a human presses Space in KSP, the game pauses. The kRPC connection stays open, but no physics ticks fire. When KSP resumes, `dt` will reflect however long the game was paused — potentially many seconds. 
If `dt > 3 * expected_dt`, the Mission Director skips the Kalman predict step to avoid divergence and logs a `DT_SPIKE` event in the JSONL log.
