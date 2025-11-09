# RTX LiDAR Implementation - Bug Fixes

## Final Solution

### Root Cause
The initial implementation attempted to use PhysX LiDAR interface (`_range_sensor`) to read RTX LiDAR data, which are incompatible. RTX LiDAR uses a raytracing-based approach with a different API.

### Correct Implementation

**Import the proper class:**
```python
from isaacsim.sensors.rtx import LidarRtx
```

**Create sensor and wrap in LidarRtx:**
```python
# Create the sensor prim
result, sensor_prim = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path=lidar_path,
    parent="/World/Jetbot/chassis",
    config="Example_Solid_State",
    translation=(0.08, 0.0, 0.06),
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # w, i, j, k
)

# Wrap in LidarRtx class for data access
lidar_sensor = LidarRtx(prim_path=lidar_path, name="jetbot_lidar")

# Enable the annotator for distance data
lidar_sensor.add_linear_depth_data_to_frame()
```

**Initialize properly:**
```python
my_world.reset()
lidar_sensor.initialize()  # Initialize sensor
timeline.play()

# Step simulation to let sensor collect first data
for i in range(20):
    my_world.step(render=True)
```

**Read data correctly:**
```python
def get_lidar_data():
    current_frame = lidar_sensor.get_current_frame()
    if current_frame is None:
        return None
    
    depth_data = current_frame.get("linear_depth_data")
    # ... process depth_data
```

## Issues Fixed

### Issue 1: Wrong Sensor Interface ✅
**Before:**
```python
from omni.isaac.range_sensor import _range_sensor
lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
depth_data = lidar_interface.get_linear_depth_data(lidar_path)
```
**Error:** `Lidar Sensor does not exist`

**After:**
```python
from isaacsim.sensors.rtx import LidarRtx
lidar_sensor = LidarRtx(prim_path=lidar_path, name="jetbot_lidar")
depth_data = lidar_sensor.get_current_frame().get("linear_depth_data")
```

### Issue 2: Missing Annotator ✅
**Before:** Tried to read data without enabling annotator

**After:** Enable the flat scan annotator:
```python
lidar_sensor.add_linear_depth_data_to_frame()
```

### Issue 3: No Sensor Initialization ✅
**Before:** Sensor not initialized before use

**After:**
```python
lidar_sensor.initialize()
```

### Issue 4: Quaternion Format ✅
**Before:** `orientation=(0.0, 0.0, 0.0, 1.0)`

**After:** `orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0)`

## Files Updated

1. **examples/wearhaus_room_jetbot_avoidance.py**
   - Changed to use `LidarRtx` class
   - Added proper initialization
   - Fixed data reading method

2. **tests/test_lidar_avoidance.py**
   - Same fixes as above

## API Reference

The correct API for RTX LiDAR in Isaac Sim 5.1:

**Class:** `isaacsim.sensors.rtx.LidarRtx`

**Key methods:**
- `initialize()` - Initialize sensor before use
- `add_linear_depth_data_to_frame()` - Enable distance data
- `add_intensities_data_to_frame()` - Enable intensity data
- `get_current_frame()` - Get latest sensor data

**Data access:**
- `current_frame["linear_depth_data"]` - Distance array
- `current_frame["intensities_data"]` - Intensity array
- `current_frame["horizontal_resolution"]` - Resolution
- `current_frame["azimuth_range"]` - Angular range

## Testing

Run the fixed version:
```bash
cd ~/Desktop/isaac-sim2real/scripts
./run_isaac_dxl.sh  # Choose option 9
```

Expected output:
```
[✓] RTX LiDAR sensor prim created
[✓] RTX LiDAR configured
[✓] LiDAR sensor initialized
[Sensor] Stepping simulation for sensor warm-up...
[✓] Sensor warm-up complete
```

No more "Lidar Sensor does not exist" errors!

## Reference

Official Isaac Sim RTX LiDAR example:
- `~/Desktop/isaacsim/source/standalone_examples/api/isaacsim.util.debug_draw/rtx_lidar.py`
- `~/Desktop/isaacsim/source/extensions/isaacsim.sensors.rtx/python/impl/lidar_rtx.py`
