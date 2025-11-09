# Sensor Switch: RTX LiDAR → PhysX Rotating LiDAR

## Problem Summary
RTX LiDAR proved too complex for basic obstacle avoidance:
- Event-driven callback system difficult to debug
- Annotator setup required
- No clear `.update()` method for manual polling
- Robot stuck in "WARMING UP" state - no data flowing

## Solution
Switched to **PhysX Rotating LiDAR** - much simpler API:

### Old (RTX LiDAR - Complex)
```python
from isaacsim.sensors.rtx import LidarRtx

# Create sensor
omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path=lidar_path,
    config="Example_Solid_State",
    ...
)

# Wrap in class
lidar_sensor = LidarRtx(prim_path=lidar_path)
my_world.scene.add(lidar_sensor)
lidar_sensor.add_linear_depth_data_to_frame()
lidar_sensor.initialize()

# Read via callback/frame system
current_frame = lidar_sensor.get_current_frame()
depth_data = current_frame.get("linear_depth_data")
```

### New (PhysX Rotating LiDAR - Simple!)
```python
from omni.isaac.range_sensor import _range_sensor

# Create sensor
omni.kit.commands.execute(
    "RangeSensorCreateLidar",
    path=lidar_path,
    horizontal_fov=270.0,
    max_range=100.0,
    horizontal_resolution=1.0,
    ...
)

# Read directly - no initialization needed!
lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
depth_data = lidar_interface.get_linear_depth_data(lidar_path)
```

## Key Differences

| Feature | RTX LiDAR | PhysX Rotating LiDAR |
|---------|-----------|---------------------|
| **API Complexity** | High (callbacks, annotators) | Low (direct reads) |
| **Initialization** | `.initialize()` + annotators | Auto (just step sim) |
| **Data Reading** | Event-driven callbacks | Synchronous polling |
| **Integration** | Must add to scene | Just create and use |
| **Debugging** | Difficult (async flow) | Easy (direct calls) |
| **Use Case** | Advanced/realistic sims | Basic distance sensing |

## What Changed
1. **Import**: `isaacsim.sensors.rtx.LidarRtx` → `omni.isaac.range_sensor._range_sensor`
2. **Creation Command**: `IsaacSensorCreateRtxLidar` → `RangeSensorCreateLidar`
3. **Configuration**: Removed annotator setup, simplified params
4. **Data Reading**: Direct API call instead of frame/callback system
5. **FOV**: Changed from 120° (solid-state) to 270° (rotating)

## Files Updated
- ✅ `examples/wearhaus_room_jetbot_avoidance.py` - Main demo
- ✅ `tests/test_lidar_avoidance.py` - Test script

## State Machine Logic
**Unchanged!** All avoidance behavior, state transitions, hysteresis, and hardware integration remain identical. Only the sensor interface changed.

## Next Steps
1. **Test**: Run `./run_isaac_dxl.sh` (option 9)
2. **Verify**: Check if distance readings appear
3. **Observe**: Robot should detect obstacle at 1.5m and execute avoidance
4. **Tune**: Adjust `AVOIDANCE_CONFIG` if needed

## Expected Behavior
```
[Sensor] Distance: 5.23m - State: FORWARD
[Sensor] Distance: 2.45m - State: FORWARD
[Sensor] Distance: 0.75m - State: STOP      ← Below threshold!
[Sensor] Distance: 0.75m - State: REVERSE   ← Backing up
[Sensor] Distance: 1.20m - State: TURN      ← Turning away
[Sensor] Distance: 3.50m - State: FORWARD   ← Clear path found
```

---

**Why This Works**: PhysX LiDAR is designed for simple robotics applications where you just need distance measurements. RTX LiDAR is for high-fidelity simulations requiring realistic sensor modeling. We need the former!
