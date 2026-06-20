#!/usr/bin/env python3
"""
Suicide Burn Calculator for AEGIS

Calculates optimal powered descent ignition altitude and glideslope parameters
based on vessel performance and manual test data.

Manual test observations (2026-06-17):
- Vessel aerobrakes to ~150 m/s at 3000m altitude
- Manual suicide burn successful starting from 2000m
- Vessel has sufficient thrust margin for gimbal authority at this ignition point

Usage:
    python scripts/calculate_suicide_burn.py
"""

import math

# ============================================================
# VESSEL SPECIFICATIONS (from KSP telemetry)
# ============================================================
MASS = 5125.69  # kg (wet mass at powered descent)
ENGINE_THRUST = 18303.6  # N per engine (liquidEngineMini.v2 "Spark")
NUM_ENGINES = 5
GIMBAL_RANGE_DEG = 4.5  # degrees

# Derived values
TOTAL_THRUST = ENGINE_THRUST * NUM_ENGINES
G = 9.81  # m/s^2 (Kerbin gravity)
MAX_ACCEL = TOTAL_THRUST / MASS
NET_ACCEL = MAX_ACCEL - G  # net upward acceleration capability
TWR = MAX_ACCEL / G


def altitude_to_stop(velocity: float, deceleration: float) -> float:
    """Calculate altitude required to stop from given velocity.
    
    Args:
        velocity: Current descent velocity (m/s, positive downward)
        deceleration: Available net deceleration (m/s^2)
    
    Returns:
        Altitude required to reach zero velocity (m)
    """
    return velocity**2 / (2 * deceleration)


def velocity_at_altitude(velocity_start: float, alt_start: float, alt_end: float, 
                         deceleration: float) -> float:
    """Calculate velocity at a given altitude during constant deceleration.
    
    Args:
        velocity_start: Initial velocity (m/s)
        alt_start: Starting altitude (m)
        alt_end: Target altitude (m)
        deceleration: Net deceleration (m/s^2)
    
    Returns:
        Velocity at target altitude (m/s)
    """
    delta_h = alt_start - alt_end
    v_squared = velocity_start**2 - 2 * deceleration * delta_h
    if v_squared < 0:
        return 0.0  # Would have stopped already
    return math.sqrt(v_squared)


def main():
    print("=" * 60)
    print("AEGIS Suicide Burn Calculator")
    print("=" * 60)
    print()
    
    # Vessel specs
    print("VESSEL SPECIFICATIONS")
    print("-" * 40)
    print(f"Mass:                    {MASS:.1f} kg")
    print(f"Engines:                 {NUM_ENGINES}× liquidEngineMini.v2")
    print(f"Total thrust:            {TOTAL_THRUST:.0f} N")
    print(f"TWR:                     {TWR:.2f}")
    print(f"Max acceleration:        {MAX_ACCEL:.2f} m/s²")
    print(f"Net deceleration:        {NET_ACCEL:.2f} m/s²")
    print(f"Gimbal range:            {GIMBAL_RANGE_DEG:.1f}° ({GIMBAL_RANGE_DEG * math.pi / 180:.4f} rad)")
    print()
    
    # Manual test data
    print("MANUAL TEST OBSERVATIONS")
    print("-" * 40)
    print("From manual testing (2026-06-17):")
    print("  - Aerobraking to ~150 m/s at 3000m altitude")
    print("  - Successful suicide burn from 2000m ignition")
    print("  - Sufficient thrust margin for gimbal control")
    print()
    
    # Stopping distance analysis
    print("STOPPING DISTANCE ANALYSIS")
    print("-" * 40)
    print(f"{'Velocity (m/s)':<20} {'Altitude Required (m)':<25} {'Margin from 2000m':<20}")
    print("-" * 65)
    
    test_velocities = [50, 75, 100, 125, 150, 175, 200]
    for v in test_velocities:
        h_required = altitude_to_stop(v, NET_ACCEL)
        margin = 2000 - h_required
        print(f"{v:<20.0f} {h_required:<25.1f} {margin:<20.1f}{'✓' if margin > 0 else '✗ CRASH'}")
    print()
    
    # Recommended parameters
    print("RECOMMENDED CONFIGURATION")
    print("-" * 40)
    
    # Target: ignite at altitude where velocity is ~100-120 m/s
    # This provides margin for lateral corrections and gimbal authority
    target_velocity = 110.0  # m/s at ignition
    safety_margin = 300.0  # m
    
    h_required = altitude_to_stop(target_velocity, NET_ACCEL)
    h_ignition = h_required + safety_margin
    
    print(f"Target ignition velocity: {target_velocity:.0f} m/s")
    print(f"Required altitude:        {h_required:.0f} m")
    print(f"Safety margin:            {safety_margin:.0f} m")
    print(f"Recommended ignition:     {h_ignition:.0f} m")
    print()
    
    # Glideslope rate tuning
    print("GLIDESLOPE RATE TUNING")
    print("-" * 40)
    print("Maximum descent rate cap for powered descent phase:")
    print("(Should allow aerobraking to work, but prevent excessive speed)")
    print()
    
    print(f"{'Max Rate (m/s)':<20} {'Stop Altitude (m)':<20} {'Recommendation':<20}")
    print("-" * 60)
    
    for rate in [80, 100, 120, 150, 200]:
        h_stop = altitude_to_stop(rate, NET_ACCEL)
        rec = "✓ Good" if 400 < h_stop < 1000 else "Too low" if h_stop < 400 else "Too high"
        print(f"{rate:<20.0f} {h_stop:<20.1f} {rec:<20}")
    print()
    
    # Output config snippet
    print("CONFIGURATION SNIPPET (src/config.py)")
    print("-" * 40)
    print("# Auto-generated by calculate_suicide_burn.py")
    print(f"ALT_POWERED_DESCENT = {h_ignition:.0f}  # Ignite below this altitude")
    print(f"GLIDESLOPE_RATE_POWERED_DESCENT = 100.0  # Cap descent rate")
    print(f"ALT_HYPERSONIC = 10000.0  # Aerobrake from this altitude")
    print()
    
    # Trajectory simulation
    print("SAMPLE TRAJECTORY (from 10000m coast)")
    print("-" * 40)
    print("Assumes free-fall from 10000m with aerodynamic drag")
    print("(Drag modeled as reducing terminal velocity to ~150 m/s at 3000m)")
    print()
    
    # Simplified model: velocity increases with sqrt(2*g*h) but capped by drag
    altitudes = [10000, 8000, 6000, 4000, 3000, 2500, h_ignition]
    for alt in altitudes:
        if alt > 3000:
            # Free fall with drag approximation
            v = min(150, math.sqrt(2 * G * (10000 - alt)) * 0.7)
        elif alt > h_ignition:
            # Continuing to accelerate
            v = 150 * math.sqrt(alt / 3000)
        else:
            # Powered descent (not calculated here)
            v = 0
        
        status = "IGNITION" if abs(alt - h_ignition) < 50 else ""
        print(f"Altitude {alt:6.0f}m: velocity ~{v:6.1f} m/s {status}")
    
    print()
    print("=" * 60)
    print("Run 'scripts/run-auto-tuner.py' to fine-tune these parameters")
    print("=" * 60)


if __name__ == "__main__":
    main()