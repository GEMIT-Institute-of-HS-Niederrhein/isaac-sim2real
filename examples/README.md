# Isaac Sim Examples

Example scripts demonstrating Isaac Sim capabilities with hardware integration and intelligent obstacle avoidance.

## ⭐ Main Demos

### 🚦 wearhaus_room_jetbot_avoidance.py (RECOMMENDED)
**Purpose:** Dual-robot intelligent obstacle avoidance with hardware integration

**Features:**
- 🤖 **2 Jetbots** with differential drive (4 Dynamixel motors total)
- 🚦 **Intelligent Obstacle Avoidance**:
  - PhysX raycast-based sensing (60° FOV, 8m range)
  - Gradual slowdown as obstacles approach
  - Smart turn direction (analyzes left/right clearance)
  - Smooth acceleration/deceleration
  - 5-state machine: FORWARD, STOP, REVERSE, TURN, EMERGENCY_STOP
- 🔄 **Mutual Avoidance**: Robots detect and avoid each other
- 🏢 **Realistic Environment**: Full warehouse with shelves and props
- 🛡️ **Hardware Safety**: Velocity limiting, emergency stop, error handling

**Hardware Configuration:**
- **Robot 1**: Motor IDs 1 (Left) & 2 (Right)
- **Robot 2**: Motor IDs 3 (Left) & 4 (Right)
- Port: /dev/ttyUSB0 @ 57600 baud
- Works in simulation-only mode if hardware not connected

**How to run:**
```bash
# Method 1: Direct execution
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/isaac-sim2real/examples/wearhaus_room_jetbot_avoidance.py

# Method 2: Use menu (recommended)
cd ~/Desktop/isaac-sim2real/scripts
./run_isaac_dxl.sh  # Choose option 9
```

**Controls:**
- `E` : Toggle emergency stop (both robots)
- `Ctrl+C` : Quit application
- Robots move forward autonomously and avoid obstacles

**Console Output:**
```
[Step  370]
  Robot 1: Pos: [+0.73, -0.41] | Vel: L=+5.000 R=+5.000 | Avoid: FORWARD | Dist: 4.70m | Motors: L= 202 R= 207
  Robot 2: Pos: [+1.33, -0.72] | Vel: L=+5.000 R=+5.000 | Avoid: FORWARD | Dist: 4.10m | Motors: L= 198 R= 195
```

**Tunable Parameters** (in script):
```python
AVOIDANCE_CONFIG = {
    "obstacle_threshold": 0.8,      # Stop distance (meters)
    "clear_margin": 0.5,            # Hysteresis margin (meters)
    "reverse_speed": 0.15,          # Reverse velocity (m/s)
    "reverse_duration": 1.8,        # Reverse time (seconds)
    "turn_speed": math.pi / 4,      # Turn angular velocity (rad/s)
    "turn_angle": math.pi / 2,      # Turn amount (radians, ~90°)
    "lidar_fov": 60.0,              # Forward arc coverage (degrees)
    "forward_speed": 0.5,           # Normal forward velocity (m/s)
    # ... 13 parameters total
}
```

---

### 🏢 wearhaus_room_jetbot.py
**Purpose:** Single Jetbot with manual control in warehouse environment

**Features:**
- Single Jetbot robot with differential drive
- Autonomous movement pattern
- Optional Dynamixel integration (Motor IDs 1 & 2)
- Realistic warehouse environment
- Works in simulation-only mode

**How to run:**
```bash
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/isaac-sim2real/examples/wearhaus_room_jetbot.py

# Or use menu:
cd ~/Desktop/isaac-sim2real/scripts
./run_isaac_dxl.sh  # Choose option 8
```

---

### 📦 sample_room_robot.py
**Purpose:** Basic example with NVIDIA robot in warehouse

**Features:**
- Loads NVIDIA warehouse environment or creates simple room
- Supports multiple robot types (Jetbot, Carter, Nova Carter)
- Demonstrates basic robot movement
- Template for hardware integration

**How to run:**
```bash
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/isaac-sim2real/examples/sample_room_robot.py

# Or use menu:
cd ~/Desktop/isaac-sim2real/scripts
./run_isaac_dxl.sh  # Choose option 6
```

---

### 🤖 minimal_jetbot.py
**Purpose:** Minimal working example with Jetbot

**Features:**
- Simplest possible Jetbot setup
- No hardware dependencies
- Great starting point for learning

**How to run:**
```bash
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/isaac-sim2real/examples/minimal_jetbot.py
```

---

### 🔌 jetbot_servo_bridge.py
**Purpose:** Hardware integration example

**Features:**
- Jetbot with Dynamixel servo integration
- Bidirectional sync (Sim ↔ Hardware)
- Template for custom hardware bridges

**How to run:**
```bash
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/isaac-sim2real/examples/jetbot_servo_bridge.py
```

## 🎯 Which Example Should I Use?

| Use Case | Example | Description |
|----------|---------|-------------|
| **Demo showcase** | `wearhaus_room_jetbot_avoidance.py` | Best for demonstrations, intelligent behaviors |
| **Learning Isaac Sim** | `minimal_jetbot.py` → `sample_room_robot.py` | Start simple, build complexity |
| **Hardware integration** | `jetbot_servo_bridge.py` | Connect simulation to real motors |
| **Testing navigation** | `wearhaus_room_jetbot.py` | Test autonomous movement patterns |
| **Custom development** | `sample_room_robot.py` | Template for custom projects |

## 📁 Project Structure

```
examples/
├── wearhaus_room_jetbot_avoidance.py  # ⭐ Main demo (dual-robot obstacle avoidance)
├── wearhaus_room_jetbot.py            # Single jetbot with manual control
├── sample_room_robot.py               # Basic warehouse example
├── minimal_jetbot.py                  # Minimal working example
├── jetbot_servo_bridge.py             # Hardware integration template
└── README.md                          # This file
```

## 🔧 Creating Your Own Examples

### Basic Template

```python
#!/usr/bin/env python3
"""Your Example Description"""

from isaacsim import SimulationApp

# Configuration
CONFIG = {"headless": False, "width": 1920, "height": 1080}
simulation_app = SimulationApp(CONFIG)

# Import Isaac Sim modules AFTER SimulationApp
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
import numpy as np

# Create world
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# Add your objects/robots here
# ...

# Initialize
my_world.reset()

# Main loop
while simulation_app.is_running():
    my_world.step(render=True)
    # Your logic here

# Cleanup
simulation_app.close()
```

### Key Principles

1. **Import order matters:**
   ```python
   from isaacsim import SimulationApp
   app = SimulationApp({})  # Create first
   # Now import other Isaac modules
   from isaacsim.core.api import World
   ```

2. **Use Isaac Sim's Python:**
   ```bash
   # ✅ Correct
   cd ~/Desktop/isaacsim/_build/linux-x86_64/release
   ./python.sh your_script.py
   
   # ❌ Wrong
   python your_script.py
   ```

3. **Reset before simulation:**
   ```python
   my_world.reset()  # Always call before main loop
   ```

4. **Handle cleanup:**
   ```python
   try:
       while simulation_app.is_running():
           my_world.step(render=True)
   finally:
       simulation_app.close()
   ```

## 📚 Learning Resources

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

### Documentation
- **Main README:** `../README.md` - Complete project guide
- **How to Run:** `../HOW_TO_RUN.md` - Python interpreter guide
- **Servo Control:** `../SERVO_CONTROL_GUIDE.md` - Hardware integration
- **Troubleshooting:** `../docs/TROUBLESHOOTING.md` - Common issues
- **Official Docs:** https://docs.omniverse.nvidia.com/isaacsim/latest/

## 🐛 Troubleshooting

### Common Issues

**"ModuleNotFoundError: No module named 'isaacsim'"**
```bash
# Solution: Use Isaac Sim's Python interpreter
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh your_script.py
```

**Robot not visible in viewport**
```bash
# Solution: Press 'F' to frame camera on robot
# Or check console for USD loading errors
```

**Motors not responding**
```bash
# Solution: Check hardware connections
python tools/hardware/dxl_idscan.py  # Verify motor IDs
```

**Obstacle avoidance not working**
```bash
# Check console for:
# - LiDAR initialization (20-step warm-up)
# - Distance readings (should show ~10 rays)
# - State transitions (FORWARD → STOP → REVERSE → TURN)
```

**Permission denied on /dev/ttyUSB0**
```bash
sudo usermod -aG dialout $USER
newgrp dialout
```

See [TROUBLESHOOTING.md](../docs/TROUBLESHOOTING.md) for more solutions.

## 🚀 Next Steps

1. **Run the main demo:** Start with `wearhaus_room_jetbot_avoidance.py`
2. **Explore simpler examples:** Try `minimal_jetbot.py` to understand basics
3. **Modify parameters:** Tune obstacle avoidance thresholds and behaviors
4. **Add hardware:** Connect Dynamixel motors and test bidirectional sync
5. **Create your own:** Use templates to build custom applications

---

For questions or contributions, see the main project [README](../README.md) or [CONTRIBUTING.md](../CONTRIBUTING.md).
