# RTX LiDAR - Sensor Not Reading Data Fix

## Problem
Robot stuck in "WARMING UP" state - sensor never becomes ready because `lidar_data` is always `None`.

```
# RTX LiDAR - Final Working Solution

## Problem
Robot stuck in "WARMING UP" state - sensor never getting data.

## Root Cause
Misunderstood the `LidarRtx` API. The sensor uses an **event-driven callback system**, not manual update calls.

## Correct Implementation

### ❌ WRONG (What We Tried):
```python
lidar_sensor.update(dt=0.0)  # ❌ This method doesn't exist!
current_frame = lidar_sensor.get_current_frame()
```

### ✅ CORRECT (What Actually Works):
```python
# Just read the data - it's updated automatically!
current_frame = lidar_sensor.get_current_frame()
depth_data = current_frame.get("linear_depth_data")
```

## How LidarRtx Actually Works

The `LidarRtx` class uses an automatic callback system:

1. **Initialization**:
   ```python
   lidar_sensor = LidarRtx(prim_path=lidar_path, name="jetbot_lidar")
   my_world.scene.add(lidar_sensor)  # Important!
   lidar_sensor.add_linear_depth_data_to_frame()  # Enable annotator
   ```

2. **After world.reset() and initialize()**:
   ```python
   my_world.reset()
   lidar_sensor.initialize()  # Registers callback with event dispatcher
   ```

3. **Automatic Data Collection**:
   - The sensor registers `_data_acquisition_callback` with the event dispatcher
   - **Every frame**, the callback automatically fires
   - Callback updates `_current_frame` with latest data
   - No manual trigger needed!

4. **Reading Data (each loop iteration)**:
   ```python
   current_frame = lidar_sensor.get_current_frame()
   depth_data = current_frame.get("linear_depth_data")
   # Data is fresh - updated automatically by callback
   ```

## Complete Working Code

```python
# 1. Create sensor
result, sensor_prim = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path=lidar_path,
    parent="/World/Jetbot/chassis",
    config="Example_Solid_State",
    translation=(0.08, 0.0, 0.06),
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
)

# 2. Wrap and configure
lidar_sensor = LidarRtx(prim_path=lidar_path, name="jetbot_lidar")
my_world.scene.add(lidar_sensor)  # Add to world scene!
lidar_sensor.add_linear_depth_data_to_frame()  # Enable annotator

# 3. Initialize
my_world.reset()
lidar_sensor.initialize()  # Starts automatic data collection
timeline.play()

# 4. Main loop - just read the data
while running:
    my_world.step(render=True)
    
    # Data is updated automatically - just read it
    current_frame = lidar_sensor.get_current_frame()
    if current_frame:
        depth_data = current_frame.get("linear_depth_data")
        if depth_data is not None:
            # Process depth_data
            pass
```

## Key Points

1. **No manual `.update()` method** - LidarRtx doesn't have this
2. **Callback-based** - Data collection happens automatically via event dispatcher
3. **Must add to world.scene** - `my_world.scene.add(lidar_sensor)`
4. **Must enable annotator** - `lidar_sensor.add_linear_depth_data_to_frame()`
5. **Must call initialize()** - Registers the callback

## Files Updated

1. **examples/wearhaus_room_jetbot_avoidance.py**
   - Removed incorrect `.update(dt=0.0)` call
   - Data now read directly from `get_current_frame()`

2. **tests/test_lidar_avoidance.py**
   - Same fix

## Testing

```bash
cd ~/Desktop/isaac-sim2real/scripts
./run_isaac_dxl.sh  # Choose option 9
```

Expected output:
```
[✓] LiDAR sensor initialized
[✓] Sensor warm-up complete
[Step   20] Pos: [+0.00, +0.00] | Vel: L=+2.345 R=+2.345 | Avoid: FORWARD | Dist: 1.50m
```

## API Reference

**LidarRtx Data Collection Flow:**

```python
# Internal (automatic):
lidar_sensor.initialize()
  └─> lidar_sensor.resume()
      └─> Registers _data_acquisition_callback with event dispatcher
          └─> Callback fires automatically each frame
              └─> Updates _current_frame dict

# External (manual):
current_frame = lidar_sensor.get_current_frame()
  └─> Returns _current_frame dict (already up-to-date)
```

**Key Methods:**
- `initialize()` - Starts data collection (registers callback)
- `get_current_frame()` - Returns latest data dict
- `pause()` - Stops data collection
- `resume()` - Resumes data collection
- `is_paused()` - Check if paused

**No `.update()` method exists!**
```

## Root Cause
The `LidarRtx.get_current_frame()` method returns stale/cached data. You must call `.update()` first to trigger new data collection.

## Solution

### Before (Not Working):
```python
def get_lidar_data():
    current_frame = lidar_sensor.get_current_frame()  # Returns stale data!
    depth_data = current_frame.get("linear_depth_data")
    return depth_data
```

### After (Working):
```python
def get_lidar_data():
    # ⭐ KEY FIX: Update sensor to collect new data frame
    lidar_sensor.update(dt=0.0)
    
    # Now get_current_frame() returns fresh data
    current_frame = lidar_sensor.get_current_frame()
    depth_data = current_frame.get("linear_depth_data")
    return depth_data
```

## Why This Is Needed

The `LidarRtx` sensor workflow:
1. **Create sensor** → Sensor exists but isn't collecting data
2. **Initialize sensor** → Sensor is ready but needs to be triggered
3. **Update sensor** → Triggers data collection for current frame ⭐
4. **Get current frame** → Returns the data from step 3

Without calling `.update()`, the sensor never collects new data, so `get_current_frame()` returns `None` or stale data.

## Files Updated

1. **examples/wearhaus_room_jetbot_avoidance.py**
   - Added `lidar_sensor.update(dt=0.0)` in `get_lidar_data()`

2. **tests/test_lidar_avoidance.py**
   - Added `lidar_sensor.update(dt=0.0)` in `get_lidar_data()`

## Expected Behavior After Fix

```
[Sensor] Stepping simulation for sensor warm-up...
[✓] Sensor warm-up complete

[Step   20] Pos: [+0.00, +0.00] | Vel: L=+2.345 R=+2.345 | Avoid: FORWARD | Dist: 1.50m
[Step   40] Pos: [+0.10, +0.00] | Vel: L=+2.345 R=+2.345 | Avoid: FORWARD | Dist: 1.40m
[Step   60] Pos: [+0.20, +0.00] | Vel: L=+2.345 R=+2.345 | Avoid: FORWARD | Dist: 1.30m
...
[AVOIDANCE] Obstacle detected at 0.75m - STOPPING
[AVOIDANCE] Starting REVERSE maneuver
[AVOIDANCE] Starting TURN maneuver
[AVOIDANCE] Path clear (1.15m) - Resuming FORWARD
```

Robot should now:
- ✅ Show distance readings
- ✅ Move forward normally
- ✅ Detect obstacles
- ✅ Execute avoidance maneuvers

## API Reference

**LidarRtx Key Methods:**

```python
# Initialize sensor (call once after world.reset())
lidar_sensor.initialize()

# Update sensor (call every frame to collect new data)
lidar_sensor.update(dt=0.0)  # dt=0.0 uses simulation dt

# Get latest data (after update)
frame = lidar_sensor.get_current_frame()
distances = frame["linear_depth_data"]
```

## Testing

Run the updated script:
```bash
cd ~/Desktop/isaac-sim2real/scripts
./run_isaac_dxl.sh  # Choose option 9
```

Should now see:
- Distance readings updating
- State changing from WARMING UP → FORWARD
- Robot moving toward obstacle
- Avoidance behavior triggering

## Lesson Learned

RTX sensors in Isaac Sim 5.1 require explicit `.update()` calls to collect new frame data. This is different from some other sensor types that automatically update with the simulation step.

**Always remember:**
```python
lidar_sensor.update(dt=0.0)  # Collect new frame
frame = lidar_sensor.get_current_frame()  # Read the data
```
