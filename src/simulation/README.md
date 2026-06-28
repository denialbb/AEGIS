# Digital Twin Physics Simulation

This module implements a headless, deterministic, and blazing-fast rigid-body physics simulator for AEGIS. Its primary purpose is to train the Active Disturbance Rejection Controller (ADRC) and test the Fault Detection and Isolation (FDI) module without requiring a live KSP instance.

## Architectural Decisions (ADR-031)

### 1. Domain Boundary: Brain vs. Universe

This module represents the **Universe** (Truth). The Control Allocator represents the **Brain**.

- The simulation receives exactly what physical actuators receive: **commands** (`throttles`, `gimbals`).
- The simulation recalculates thrust and wrench from these commands. _We never reuse the allocator's forward math._ This ensures we can simulate actuator lag and engine failures that the Brain is unaware of.
- The simulation strictly outputs perfect mathematical **Truth** state. Sensor noise is handled externally by a `SimulatedSensors` wrapper to allow 1:1 parity with the live game's `SensorModels`.

### 2. Physics Rules

- **Coordinate Frame:** Local NED (North-East-Down) assuming a flat, non-rotating planet. (No orbital mechanics).
- **Integration Method:** Fixed-step RK4 (Runge-Kutta 4th Order). Required for stability and determinism in headless training.
- **Actuators:** First-order exponential lag (spooling). Engines do not reach commanded thrust instantly. Physical limits are strictly enforced (silent clamping with `logger.warning()`).
- **Aerodynamics:** Simple quadratic translational drag applied at the Center of Mass.
- **Mass:** Dynamic mass depletion. Mass and inertia decrease as fuel is burned.
- **Ground Interaction:** Terminal state. The simulation halts cleanly the moment altitude drops to `<= 0.0`. We do not simulate bouncing.
- **Faults:** Binary kill switches (`engine.active = False`). The engine spools down to 0 thrust regardless of commanded throttle.

### 3. Dependency Injection

To avoid hardcoding Kerbin or a specific lander:

- `EnvironmentModel` defines gravity and atmospheric density.
- `VesselModel` defines engines, mass properties, and drag properties.
