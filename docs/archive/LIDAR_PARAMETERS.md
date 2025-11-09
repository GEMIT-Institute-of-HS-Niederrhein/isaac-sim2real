# RTX LiDAR Obstacle Avoidance - Parameter Reference

## Quick Configuration

Edit these parameters in `wearhaus_room_jetbot_avoidance.py`:

```python
AVOIDANCE_CONFIG = {
    # Main toggle
    "enabled": True,                    # Set False to disable avoidance
    
    # Distance thresholds (meters)
    "obstacle_threshold": 0.8,          # Trigger avoidance at this distance
    "clear_margin": 0.3,                # Resume when clear by this much more
    
    # Maneuver timing (seconds)
    "reverse_duration": 1.0,            # How long to reverse
    "cooldown_time": 0.5,               # Wait between maneuvers
    
    # Movement parameters
    "turn_angle": np.pi / 3,            # 60° turn (in radians)
    "reverse_speed": 0.1,               # Linear speed when reversing (m/s)
    "turn_speed": np.pi / 6,            # Angular speed when turning (rad/s)
    "forward_speed": 0.15,              # Normal forward speed (m/s)
    
    # Sensor processing
    "moving_avg_window": 5,             # Samples for moving average
    "lidar_fov": 60.0,                  # Forward arc to monitor (degrees)
    
    # Safety
    "emergency_stop_key": "e",          # Key for emergency stop
}
```

## Common Adjustments

### Robot too cautious?
```python
"obstacle_threshold": 0.5,  # Decrease (more aggressive)
```

### Robot hits obstacles?
```python
"obstacle_threshold": 1.2,  # Increase (more conservative)
"reverse_duration": 1.5,    # Reverse longer
```

### Oscillating behavior?
```python
"clear_margin": 0.5,        # Increase hysteresis
"cooldown_time": 1.0,       # Wait longer between maneuvers
```

### Turn not enough?
```python
"turn_angle": np.pi / 2,    # 90° instead of 60°
```

### Unstable distance readings?
```python
"moving_avg_window": 10,    # More smoothing
```

### Need wider detection?
```python
"lidar_fov": 90.0,          # Monitor wider arc
```

## State Machine Behavior

```
FORWARD: distance >= threshold
   ↓ (distance < threshold)
STOP: brief pause (0.2s)
   ↓
REVERSE: back up for reverse_duration
   ↓
TURN: rotate until distance > (threshold + margin)
   ↓
FORWARD: resume normal movement
```

## Thresholds Explained

**Obstacle Threshold (0.8m):**
- Distance at which avoidance starts
- Lower = more aggressive
- Higher = more cautious

**Clear Threshold (1.1m):**
- Calculated as: threshold + margin
- Distance at which robot resumes forward
- Prevents rapid trigger/clear cycles

**Hysteresis (0.3m):**
- Difference between trigger and clear
- Prevents oscillation
- Increase if robot keeps stopping/starting

## Emergency Stop

**Activation:**
- Press 'e' key at any time
- Immediately stops all motors
- Overrides all other commands

**Reset:**
- Press 'e' again
- Returns to FORWARD state
- Resumes normal operation

## Hardware Safety

When Dynamixel servos are connected:

1. **Velocity Clamping:** All velocities clamped to safe range
2. **Override Mode:** Hardware commands blocked during avoidance
3. **Emergency Stop:** Direct motor stop command sent
4. **Cleanup:** All motors stopped on exit

## Testing Parameters

For initial testing, use these safe values:

```python
AVOIDANCE_CONFIG = {
    "enabled": True,
    "obstacle_threshold": 1.0,      # Conservative
    "clear_margin": 0.4,            # Good hysteresis
    "reverse_duration": 1.5,        # Longer reverse
    "turn_angle": np.pi / 2,        # 90° turn
    "reverse_speed": 0.08,          # Slow reverse
    "turn_speed": np.pi / 8,        # Slow turn
    "forward_speed": 0.12,          # Slow forward
    "cooldown_time": 1.0,           # Long cooldown
    "moving_avg_window": 10,        # More smoothing
    "lidar_fov": 60.0,
    "emergency_stop_key": "e",
}
```

## Speed Conversions

**Linear speeds (m/s):**
- 0.05 = very slow
- 0.10 = slow
- 0.15 = moderate (default)
- 0.20 = fast
- 0.25 = very fast

**Angular speeds (rad/s):**
- π/12 = 15°/s = slow turn
- π/6  = 30°/s = moderate turn (default)
- π/4  = 45°/s = fast turn
- π/2  = 90°/s = very fast turn

**Turn angles (radians → degrees):**
- π/6 = 30°
- π/4 = 45°
- π/3 = 60° (default)
- π/2 = 90°
- 2π/3 = 120°

## Performance Tuning

**For smooth operation:**
- Increase moving_avg_window (more filtering)
- Increase clear_margin (less jitter)
- Decrease speeds (more control)

**For responsive operation:**
- Decrease moving_avg_window (faster response)
- Decrease cooldown_time (react faster)
- Increase speeds (faster maneuvers)

**For narrow spaces:**
- Increase obstacle_threshold (detect earlier)
- Increase turn_angle (turn more)
- Increase reverse_duration (back up more)

## Troubleshooting

**Robot doesn't stop at obstacles:**
- Check sensor warm-up (wait 2 seconds)
- Increase obstacle_threshold
- Check LiDAR data in console

**Robot stops too early:**
- Decrease obstacle_threshold
- Check for false detections (ground, noise)

**Robot oscillates:**
- Increase clear_margin
- Increase cooldown_time
- Increase moving_avg_window

**Robot doesn't clear obstacles:**
- Increase turn_angle
- Increase reverse_duration
- Check turn_speed (may be too slow)

**Jerky movement:**
- Increase moving_avg_window
- Decrease speeds
- Check hardware connection

## Files Reference

**Main demo:** `examples/wearhaus_room_jetbot_avoidance.py`
**Test script:** `tests/test_lidar_avoidance.py`
**Quick start:** `scripts/run_avoidance_demo.sh`
**Menu option:** `scripts/run_isaac_dxl.sh` (option 9)

## Console Output

Watch for these indicators:

```
[AVOIDANCE] Obstacle detected at 0.75m - STOPPING
[AVOIDANCE] Starting REVERSE maneuver
[AVOIDANCE] Starting TURN maneuver
[AVOIDANCE] Path clear (1.15m) - Resuming FORWARD
[AVOIDANCE] ⚠️  EMERGENCY STOP ACTIVATED
```

Status format:
```
[Step 1234] Pos: [+1.23, -0.45] | Vel: L=+2.345 R=+2.345 | Avoid: FORWARD | Dist: 1.50m
```

---

For more details, see `LIDAR_AVOIDANCE_IMPLEMENTATION.md`
