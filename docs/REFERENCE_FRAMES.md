# AEGIS Reference Frames

## 1. Frame Definitions

| Name   | kRPC Source                                                    | Origin        | Axes                                                    | Rotates with Planet | Used For                 |
| ------ | -------------------------------------------------------------- | ------------- | ------------------------------------------------------- | ------------------- | ------------------------ |
| **ECEF** | `body.reference_frame`                                         | Planet center | +X = equator/prime meridian, +Y = 90°E, +Z = north pole | Yes                 | Gravity computation, pad position, building NED |
| **BODY** | `vessel.reference_frame`                                       | Vessel CoM    | Nose, right, up (standard KSP vessel frame)             | No (body-fixed)     | IMU readings (accelerometer, gyroscope), thrust direction, torque, wrench |
| **NED**  | Built from `body.surface_position()`                           | Landing pad   | North, East, Down (local navigation frame)              | Yes (follows ECEF rotation) | Position/velocity guidance, state estimation |

### 1.1 ECEF — Planet-Centered, Planet-Fixed

kRPC's `body.reference_frame` is a right-handed frame centered at the celestial body's center, rotating with the planet.

```
+Z   north pole
▲
│
│
│
│
╰───→ +Y  90° east longitude
╲
╲
╲
▼
+X  equator / prime meridian
```

All vectors returned in this frame are expressed in **planet-fixed coordinates**. The frame rotates with the body — a stationary point on the surface has constant coordinates in ECEF.

### 1.2 BODY — Vessel Body Frame

kRPC's `vessel.reference_frame` is centered at the vessel's center of mass, with axes fixed to the vessel's orientation:

- **+X** — Right (starboard)
- **+Y** — Forward (nose)
- **+Z** — Up (top)

This is the natural frame for:
- **Gyroscope** — measures angular velocity of the vessel about its own axes
- **Accelerometer** — measures specific force along vessel axes
- **Engines** — thrust direction is in vessel-axis coordinates
- **Torque / wrench** — computed in body frame, allocated to engines

### 1.3 NED — North-East-Down Navigation Frame

A local-level frame with origin at the landing pad, following the aviation convention:

| Axis | Direction  |
| ---- | ---------- |
| +X   | North      |
| +Y   | East       |
| +Z   | Down       |

---

## 2. NED Construction (Verified)

Given target latitude/longitude on the celestial body:

```python
# 1) Pad position in ECEF (planet-centered, planet-fixed)
pad_ECEF = np.array(body.surface_position(lat, lon, body.reference_frame))

# 2) Radial outward (up) direction
up = pad_ECEF / np.linalg.norm(pad_ECEF)

# 3) North = polar axis projected onto local tangent plane
#    [0,0,1] is the ECEF Z axis (planet's rotation axis)
north_axis = np.array([0.0, 0.0, 1.0]) - up * np.dot(np.array([0.0, 0.0, 1.0]), up)
north = north_axis / np.linalg.norm(north_axis)

# 4) East completes the right-handed frame
east = np.cross(north, up)

# 5) Rotation matrix: ECEF → NED  (rows = NED basis in ECEF coords)
R_E2NED = np.vstack([north, east, -up])

# 6) Transform a position from ECEF to NED
r_NED = R_E2NED @ (r_ECEF - pad_ECEF)
```

### 2.1 Verification

| Test                                    | Result                                      |
| --------------------------------------- | ------------------------------------------- |
| At pad: `r_ECEF == pad_ECEF`            | `r_NED = [0, 0, 0]`                        |
| 100 m above pad: `r_ECEF += 100 × up`   | `r_NED = [0, 0, -100]` (positive D = down) |
| `north × east`                          | `= down = -up` ✓ right-handed NED           |

The third row of `R_E2NED` is `-up`. At 100 m above the pad:
```
r_NED[2] = (-up) · (100 × up) = -100
```
This is correct: **Down = +Z**, so being above ground gives a negative Z.

### 2.2 Degenerate Case: Poles

At the north pole, `pad_ECEF ≈ [0, 0, R]`, so `up ≈ [0, 0, 1]`. The polar axis projects to zero on the tangent plane, making `north` undefined. This is the standard NED gimbal-lock problem. For equatorial/mid-latitude landing sites this is not an issue; for polar landings use an alternate construction (e.g., use `east = [0,1,0]`, `north = cross(up, east)`).

---

## 3. Frame Mapping: Current → New Convention

| Current Name             | New Name    | Where Used                                                        |
| ------------------------ | ----------- | ----------------------------------------------------------------- |
| `ref_frame` / world frame / custom pad-relative frame | **N/A** (replaced by NED) | `src/main.py:60`, `src/telemetry/sensors.py:33`, `src/estimation/gyro_sensor.py:28`, `src/estimation/accelerometer_sensor.py:27` |
| `body.reference_frame`   | **ECEF**    | Gravity computation, pad position, NED building |
| `vessel.reference_frame` | **BODY**    | Engine positions, thrust direction, gimbal axes |
| `up_vector`              | **Up (ECEF axes)** | |
| `gravity_world`          | **Gravity (ECEF axes)** | |
| `omega_body`             | **ω_ECEF** (see §4) | |
| `sf_body` / `sf_world`   | **sf_ECEF**, **sf_BODY** | |

---

## 4. Data Flow & Transformations

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        kRPC  Simulator                                   │
│                                                                         │
│  body.reference_frame (ECEF)     vessel.reference_frame (BODY)         │
│  vessel.position(ECEF)           part.position(BODY)                    │
│  vessel.flight(ECEF).velocity    Engine thrust direction (BODY)         │
│  flight.rotation (body→ECEF q)   Gimbal axes (BODY)                    │
│  vessel.angular_velocity(ECEF)                                          │
└────────────────────────────────┬────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────┐
│  SensorModels (sensors.py)                                              │
│                                                                         │
│  ┌──────────────┐    ┌────────────────────────┐                        │
│  │ Accelerometer │───→│  sf_ECEF = a_ECEF − g  │                        │
│  │ (diff vel)    │    └─────────┬──────────────┘                        │
│  └──────────────┘              │ rot_bw.inv().apply()                   │
│                                ▼                                        │
│  ┌──────────────┐    ┌─────────────────────┐                            │
│  │ Gyroscope    │───→│  ω_ECEF ← BUG → ω_body ← need rot_bw.inv()     │
│  └──────────────┘    └─────────────────────┘                            │
│                                │                                        │
│  ┌──────────────┐             │                                        │
│  │ Flight       │───→ q_body→ECEF (attitude)                            │
│  └──────────────┘                                                       │
│                                │                                        │
│  ┌──────────────┐             │                                        │
│  │ Mahony       │◄──── omega_body (BUG: actually ω_ECEF)               │
│  │              │◄──── sf_body (correctly rotated)                     │
│  │              │◄──── gravity_ECEF                                     │
│  └──────────────┘                                                       │
│       │ q_body→ECEF                                                     │
│       ▼                                                                 │
│  ┌──────────────┐                                                       │
│  │ EKF          │                                                       │
│  │ pos/vel in   │                                                       │
│  │ pad-relative │  (should be NED)                                      │
│  │ ECEF axes    │                                                       │
│  └──────────────┘                                                       │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 5. Known Issues

### ~~5.1 FRAME-001: Gyroscope ω Not Rotated to Body Frame~~ (RESOLVED)

**Resolution**: `src/telemetry/sensors.py:146-150` now rotates `omega_ned` to body frame:

```python
omega_ned = self.gyro_sensor.poll()
omega_body = rot_bw.inv().apply(omega_ned)
```

The gyro_sensor still returns ω in NED-frame axes (as its docstring notes), but the caller correctly applies the body→NED rotation. The Mahony filter receives proper body-frame angular rates. Fixed in the SENSOR_WARMUP / refactor round.

### ~~5.2 FRAME-002: NED Not Yet Implemented~~ (RESOLVED)

**Resolution**: The NED reference frame is fully implemented. `src/main.py::_init_reference_frame` constructs a true NED frame using `ecef_to_ned()` from `src/common/geometry.py`. The frame is created via `ReferenceFrame.create_relative` with a rotation quaternion aligned to local north/east/down. All guidance and estimation operate in NED coordinates. Verified by 68 parametrized unit tests (`tests/test_ned_frame.py`) and a 10-check live KSP validation script (`scripts/validate_ned_invariants.py`).

### 5.3 Pole Degeneracy

The NED construction fails at/near the poles. For AEGIS's KSC landing site (lat ≈ -0.1°) this is irrelevant.

---

## 6. Transformation Summary

| Transform                              | Matrix / Quaternion                          |
| -------------------------------------- | -------------------------------------------- |
| ECEF → BODY                            | `q_body→ECEF.inv()` (or `rot_bw.inv()`)      |
| BODY → ECEF                            | `q_body→ECEF` (or `rot_bw`)                  |
| ECEF → NED                             | `R_E2NED = stack(north, east, -up)`          |
| NED → ECEF                             | `R_E2NED.T`                                  |

All frames are right-handed. NED follows the aviation convention (North-East-Down, +Z down).
