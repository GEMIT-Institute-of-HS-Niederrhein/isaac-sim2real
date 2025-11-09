# Isaac Sim Examples

This directory contains example scripts demonstrating Isaac Sim capabilities and serving as templates for your projects.

## Available Examples

### 📦 sample_room_robot.py
**Purpose:** Create a warehouse environment with NVIDIA robot examples

**What it does:**
- Loads NVIDIA warehouse environment or creates a simple room
- Adds a wheeled robot (Jetbot, Carter, or Nova Carter)
- Demonstrates basic robot movement
- Serves as a base for Dynamixel servo integration

**How to run:**
```bash
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/isaac-sim2real/examples/sample_room_robot.py
```

**Or use the menu:**
```bash
cd ~/Desktop/isaac-sim2real/scripts
./run_isaac_dxl.sh  # Choose option 6
```

**Configuration:**
Edit the script to change:
- `ROBOT_TYPE`: Choose "jetbot", "carter", or "nova_carter"
- Room size and obstacles
- Camera viewpoint
- Movement patterns

**What you'll see:**
- A 3D environment (warehouse or custom room)
- A wheeled robot performing autonomous movements
- Console output showing robot position and velocity

**Future Integration:**
This script serves as a template for:
1. Connecting Isaac Sim robot joints to Dynamixel servos
2. Reading servo feedback and updating simulation
3. Testing control algorithms before deploying to hardware
4. Sim-to-real transfer workflows

## Creating Your Own Examples

### Template Structure

```python
#!/usr/bin/env python3
"""
Your Example Description
"""

from isaacsim import SimulationApp

# Configuration
CONFIG = {
    "headless": False,
    "width": 1920,
    "height": 1080,
}

simulation_app = SimulationApp(CONFIG)

# Import Isaac Sim modules AFTER SimulationApp
from isaacsim.core.api import World
# ... other imports

# Create your world
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# Add objects, robots, etc.
# ...

# Initialize
my_world.reset()

# Main loop
while simulation_app.is_running():
    my_world.step(render=True)
    # Your logic here

# Cleanup
my_world.stop()
simulation_app.close()
```

### Tips for Isaac Sim Scripts

1. **Always import `SimulationApp` first**
   ```python
   from isaacsim import SimulationApp
   simulation_app = SimulationApp({"headless": False})
   # Now import other Isaac Sim modules
   ```

2. **Use Isaac Sim's Python interpreter**
   ```bash
   # ✅ Correct
   ./python.sh your_script.py
   
   # ❌ Wrong
   python your_script.py
   ```

3. **Reset the world before simulation**
   ```python
   my_world.reset()  # Call this before your main loop
   ```

4. **Handle cleanup properly**
   ```python
   try:
       while simulation_app.is_running():
           my_world.step(render=True)
   except KeyboardInterrupt:
       print("Interrupted")
   finally:
       my_world.stop()
       simulation_app.close()
   ```

## Available NVIDIA Robot Assets

Check what's available in your Isaac Sim installation:

```bash
# List all robot assets
find ~/Desktop/isaacsim -name "*.usd" -path "*Robots*" 2>/dev/null

# Common robots in Isaac Sim:
# - Jetbot (2-wheel differential drive)
# - Carter (4-wheel differential drive) 
# - Nova Carter (advanced autonomous platform)
# - Franka Panda (robotic arm)
# - UR10 (robotic arm)
# - And many more!
```

## Learning Resources

### Official NVIDIA Examples
Located in: `~/Desktop/isaacsim/source/standalone_examples/`

```bash
# Browse examples
ls ~/Desktop/isaacsim/source/standalone_examples/api/
ls ~/Desktop/isaacsim/source/standalone_examples/tutorials/

# Run an example
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/isaacsim/source/standalone_examples/tutorials/getting_started_robot.py
```

### Useful Example Categories

1. **Tutorials** - Basic concepts and getting started
   - `tutorials/getting_started_robot.py`
   - `tutorials/hello_world.py`

2. **Wheeled Robots** - Differential drive and holonomic robots
   - `api/isaacsim.robot.wheeled_robots.examples/jetbot_differential_move.py`
   - `api/isaacsim.robot.wheeled_robots.examples/kaya_holonomic_move.py`

3. **Robot Manipulators** - Robotic arms
   - `api/isaacsim.robot.manipulators/`

4. **ROS2 Integration** - ROS2 bridge examples
   - `api/isaacsim.ros2.bridge/`

## Documentation

- **Servo Control Guide:** `../SERVO_CONTROL_GUIDE.md` - Complete guide on connecting Dynamixel servos
- **How to Run:** `../HOW_TO_RUN.md` - Python interpreter and execution guide
- **Main README:** `../README.md` - Project documentation
- **Official Docs:** https://docs.omniverse.nvidia.com/isaacsim/latest/

## Troubleshooting

### "ModuleNotFoundError: No module named 'isaacsim'"
**Solution:** Use Isaac Sim's Python interpreter:
```bash
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh your_script.py
```

### "Could not find Isaac Sim assets folder"
**Solution:** Check if Isaac Sim is properly installed and Nucleus is running.
You can work around this by using local USD files instead of Nucleus assets.

### Robot not visible
**Solution:** 
- Press `F` key to frame camera on robot
- Check console for USD loading errors
- Verify asset path exists

### Simulation runs slow
**Solution:**
- Reduce physics substeps
- Use simpler robot models
- Close other GPU-intensive applications
- Check GPU utilization

## Next Steps

1. **Run the sample room example** to see a complete scene
2. **Modify the script** to add your own objects or behaviors
3. **Read SERVO_CONTROL_GUIDE.md** to learn about hardware integration
4. **Create your own example** based on the template
5. **Integrate with Dynamixel servos** using the bridge

---

For questions or issues, see the main project README or TROUBLESHOOTING.md
