# AEGIS Vessel Specifications

## Current Configuration (as of 2026-06-17)

### Mass & Propulsion
- **Wet mass:** 5125.69 kg
- **Engine:** 5× liquidEngineMini.v2 (48-7S "Spark")
- **Thrust per engine:** 18,303.6 N (vacuum)
- **Total thrust:** 91,518 N
- **TWR:** 1.82
- **Max acceleration:** 17.85 m/s²
- **Net upward acceleration (a_avail):** 8.04 m/s²

### Gimbal Authority
- **Gimbal range:** 4.5° (0.0785 rad) per engine
- **Configuration:** 5-engine radial mount (assumed symmetric)

### Performance Limitations

#### Suicide Burn Analysis
At powered descent ignition (3000m altitude):
- **Typical descent velocity:** ~270-300 m/s (from quicksave)
- **Required deceleration:** v²/(2h) = 300²/(2×3000) = **15 m/s²**
- **Available net acceleration:** **8.04 m/s²**
- **Shortfall:** **7 m/s²** (insufficient thrust)

**Conclusion:** The vessel cannot perform a suicide burn from high-velocity descent. The current ALT_POWERED_DESCENT = 3000m is too low for the available TWR.

#### Gimbal Saturation Root Cause
1. Suicide burn profile demands maximum deceleration (15 m/s² required vs 8.04 m/s² available)
2. All 5 engines at 100% throttle for braking → zero thrust margin for gimbal torque
3. Limited 4.5° gimbal range provides minimal torque authority
4. Guidance controller commands lateral corrections that require asymmetric thrust
5. Allocator saturates gimbals trying to produce commanded torque with no thrust margin

### Recommended Fixes

1. **Increase ALT_POWERED_DESCENT** to 8000-10000m
   - Allows longer, gentler deceleration profile
   - Reduces peak velocity before engine ignition
   - Gives vessel time to bleed velocity via drag during coast

2. **Reduce GLIDESLOPE_RATE_POWERED_DESCENT** from 300 m/s to 150-200 m/s
   - Caps descent rate to achievable deceleration
   - Prevents guidance from demanding impossible braking

3. **Add thrust-margin awareness to allocator**
   - Reserve ~10-20% thrust for gimbal authority
   - Prioritize vertical braking over lateral corrections when thrust-limited

4. **Consider craft redesign** (long-term)
   - Higher TWR engine configuration (TWR ≥ 2.5 recommended)
   - Larger gimbal range (±10° minimum)
   - More engines or higher-thrust engines

### Configuration Values to Tune

```python
# src/config.py
ALT_POWERED_DESCENT = 8000.0  # was 3000.0 - ignite earlier
GLIDESLOPE_RATE_POWERED_DESCENT = 150.0  # was 300.0 - cap descent rate
ACCEL_CLAMP_FACTOR = 0.9  # was 1.0 - reserve 10% thrust for gimbal authority
```

### Reference Mission Profile (Recommended)

1. **Deorbit burn:** Circularize/retrograde at apoapsis
2. **Hypersonic coast:** 10000m → 8000m (free fall, drag braking)
3. **Powered descent:** Ignite at 8000m, target -150 m/s descent
4. **Hover targeting:** 200m altitude, zero lateral velocity
5. **Terminal descent:** 5 m/s touchdown

---

## Historical Configurations

### Previous Test Configurations
- Mass: ~2531 kg (different craft variant)
- Engine count: 5× liquidEngineMini.v2
- TWR: ~3.6 (much higher performance)

### Notes
- Craft specs may vary between saves and quicksaves
- Always verify mass and engine configuration before tuning guidance gains
- TWR < 2.0 requires significantly higher ignition altitude