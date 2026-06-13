# AEGIS Test Vehicle (ATV) Design Document

This document outlines the recommended vessel design for testing the AEGIS (Autonomous Estimation & Guidance Integrated System) in Kerbal Space Program. To effectively test 6-DOF control allocation, differential throttling, engine gimbals, and Fault Detection & Isolation (FDI), the vessel needs specific physical characteristics and redundancy.

## 1. Primary Objectives
* **Redundancy:** Must have enough engines to maintain 6-DOF control even if 1 or 2 engines fail.
* **Torque Authority:** Engines must be spread wide from the Center of Mass (CoM) to provide strong pitch/yaw/roll authority via differential throttling.
* **Pure Engine Control:** The vessel should rely **entirely** on main engines for attitude control. Reaction wheels and RCS should be disabled or absent to truly test the Control Allocator.

## 2. Mass & Dimensions
* **Target Mass:** ~12 to 15 tons (fully fueled). This provides a stable inertia tensor, making the vessel less twitchy and easier for the State Estimator to track smoothly.
* **Profile:** Wide and squat (like a flying saucer or a heavy lunar lander). A low Center of Mass ensures that asymmetric thrust from engine failures doesn't easily overpower the vessel's rotational inertia.

## 3. Engine Configuration
* **Number of Engines:** 8 Main Engines.
* **Engine Type:** `LV-1R "Spider"` or `48-7S "Spark"` (if surface mounted underneath), or `Mk-55 "Thud"` (if radially mounted). We recommend **8x "Spark" engines** or similar highly-gimbaled engines for a 15-ton vessel.
* **Placement (The "Octagon" Layout):**
  * Mount the 8 engines in a circular/octagonal pattern around the outer edge of the bottom stage.
  * Radius: Place them as far out as possible (e.g., using structural outriggers or a wide Rockomax tank). A radius of ~1.5 to 2.5 meters from the central axis is ideal.
* **Gimbal:** Enable gimbals on all engines. AEGIS expects to compute and control gimbal X/Y vectors to assist with roll/pitch/yaw.
* **Angle:** Point the engines strictly downward (0° outward cant) or with a very slight 5° outward cant to give slightly more natural roll authority via gimbaling.

## 4. Required Hardware
1. **Probe Core:** Any advanced probe core (e.g., RC-L01 Remote Guidance Unit). 
   * *CRITICAL:* Disable its internal Reaction Wheel in the right-click menu before launch.
2. **Fuel Tanks:** A central, wide tank (e.g., Rockomax X200-16 or X200-32).
3. **Power:** Solar panels and sufficient batteries to keep the probe core and kRPC connection alive.
4. **kRPC Server:** Ensure the vessel has the necessary parts to establish a kRPC connection.
5. **Telemetry Sensors:** 
   * The kRPC mod provides orbital and surface telemetry dynamically, so no specific scientific parts are required for state estimation.

## 5. Summary of KSP Tweaks
Before running AEGIS on the pad or in flight, ensure the following vessel settings:
* **Reaction Wheels:** `Disabled` (Test the engines, not the magic torque wheels).
* **RCS:** `Off` (No monopropellant thrusters).
* **Engine Gimbals:** `Free` (Not locked, so the Allocator can command them).
* **Fuel Flow:** Ensure fuel drains evenly (symmetric tanks) so the Center of Mass doesn't shift wildly off the central Z-axis during flight.

## 6. Testing Scenarios for this Vessel
Once built in Sandbox mode, you can use this vessel to run the following test matrices:
1. **Hover Test (Nominal):** Vessel lifts off to 100m and maintains altitude.
2. **Single Engine Failure:** During hover, manually right-click an engine and click "Shutdown". AEGIS FDI should detect the drop in acceleration, isolate the engine, and the Control Allocator should dynamically throttle the opposite engines and gimbal the adjacent engines to maintain the hover without tumbling.
3. **Double Asymmetric Failure:** Shutdown two engines on the same side. The vessel should throttle down the opposite side significantly and use maximum gimbal to stay upright, possibly initiating the `HARD_ABORT` phase if it exceeds the condition threshold.

## 7. Real-Life Inspiration
For design and aesthetic inspiration, refer to these real-world spacecraft using similar multi-engine redundant layouts:

![SpaceX Crew Dragon](images/crew_dragon.jpg)
*SpaceX Crew Dragon with its 8x SuperDraco engine layout.*

![Blue Moon Lander](images/blue_moon.png)
*Blue Origin Blue Moon MK2 Lunar Lander concept.*
