# Isaac Sim2Real 🤖

Real-time bidirectional control bridge between NVIDIA Isaac Sim 5.1 and Dynamixel XL430-W250-T servo motors. **Learn in simulation, transfer to hardware** - enabling seamless sim-to-real transfer for robotics research and development.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.11+](https://img.shields.io/badge/python-3.11+-blue.svg)](https://www.python.org/downloads/)
[![Isaac Sim 5.1](https://img.shields.io/badge/Isaac%20Sim-5.1-green.svg)](https://developer.nvidia.com/isaac-sim)

## ✨ Features

- 🎮 **Real-time Control**: Keyboard-driven control with <100ms latency
- �️ **Autonomous Navigation**: Point-and-go waypoint navigation with path planning
- �🔄 **Bi-directional**: Sim → Hardware and Hardware → Sim feedback loops
- 🚗 **4-Wheel Drive**: Independent wheel control with variable speed (10%-90%)
- 🛡️ **Safe Operation**: Emergency stop, velocity limiting, error handling
- 🧪 **Modular Testing**: Test hardware and simulation independently
- 📊 **Live Monitoring**: Real-time status display and diagnostics
- 🐍 **Clean Python**: Type hints, docstrings, PEP 8 compliant

## 📋 Prerequisites

### Hardware

**Primary System (Dynamixel):**
- 4× Dynamixel XL430-W250-T servo motors ([e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/))
- 1× U2D2 USB communication adapter
- 12V power supply (SMPS recommended)

**Alternative System (ESP32 PWM):**
- ESP32 development board with motor control firmware
- DC motors with H-bridge drivers
- USB cable for ESP32 connection

**Common:**
- Computer with NVIDIA GPU (Compute Capability ≥ 5.0)

### Software
- Ubuntu 20.04+ (tested on 24.04 LTS)
- NVIDIA Isaac Sim 5.1
- Python 3.11+
- CUDA 12.0+ with compatible drivers

## 🚀 Quick Start

### 1. Clone and Setup Virtual Environment

```bash
cd ~/Desktop/isaac-sim2real

# Create and activate virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install Python dependencies
pip install --upgrade pip
pip install -r requirements.txt
```

### 2. Configure Hardware

```bash
# Grant serial port permissions
sudo usermod -aG dialout $USER
newgrp dialout  # or logout/login

# Verify motors are detected (use hardware utility tools)
python tools/hardware/dxl_idscan.py
```

### 3. Install Isaac Sim Dependencies

```bash
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh -m pip install --upgrade pip
./python.sh -m pip install dynamixel-sdk pynput
```

### 4. Verify Setup

```bash
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/isaac-sim2real/scripts/verify_setup.py
```

### 5. Run the Bridge! 🎯

**⚠️ CRITICAL: Must use Isaac Sim's Python interpreter!**

```bash
# ✅ CORRECT - Use Isaac Sim's python.sh
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/isaac-sim2real/src/isaac_dxl_bridge.py

# Or use the helper script (recommended):
cd ~/Desktop/isaac-sim2real/scripts
./run_isaac_dxl.sh  # Choose option 5
```

**❌ DO NOT run with regular Python:**
```bash
# This will fail with "ModuleNotFoundError: No module named 'isaacsim'"
python src/isaac_dxl_bridge.py
source .venv/bin/activate && python src/isaac_dxl_bridge.py
```

> **Why?** The `isaacsim` module is only available in Isaac Sim's Python environment.
> See [HOW_TO_RUN.md](HOW_TO_RUN.md) for detailed explanation.

**Keyboard Controls:**

**Mode Toggle:**
- `N` : Toggle between Keyboard and Navigation modes

**Keyboard Mode (Manual Control):**
- `↑` / `↓` : Forward / Backward
- `←` / `→` : Turn Left / Right (4-Wheel Skid Steering)
- `1-9` : Speed Control (10%-90%)
- `SPACE` : Emergency Stop

**Navigation Mode (Autonomous):**
- `W` : Navigate forward 2 meters
- `A` : Navigate left 2 meters
- `S` : Navigate backward 2 meters
- `D` : Navigate right 2 meters

**Common:**
- `ESC` : Quit Application

### 🗺️ Autonomous Navigation

The bridge now supports autonomous waypoint navigation with automatic path planning:

1. **Press `N`** to toggle Navigation Mode
2. **In Navigation Mode**, use `W`, `A`, `S`, `D` keys to set relative waypoints:
   - `W` = Navigate 2 meters forward
   - `A` = Navigate 2 meters to the left
   - `S` = Navigate 2 meters backward  
   - `D` = Navigate 2 meters to the right

3. The robot will autonomously drive to the waypoint while:
   - Adjusting heading to face the goal
   - Following a straight-line path
   - Displaying distance and progress
   - Stopping when the waypoint is reached

4. **Press `N`** again to return to manual Keyboard Mode

**Future Enhancements:**
- Viewport click-to-navigate (click anywhere in Isaac Sim to set goals)
- SLAM integration for mapping and localization
- Advanced path planning (A*, RRT) with obstacle avoidance
- Multi-waypoint paths with smooth trajectory generation

## 📂 Project Structure

```
isaac-sim2real/
├── src/                          # Main source code
│   ├── isaac_dxl_bridge.py      # Full Isaac Sim ↔ Hardware bridge
│   └── simple_gui_test.py       # Hardware-only GUI test
├── scripts/                      # Utility scripts
│   ├── verify_setup.py          # Setup verification
│   └── run_isaac_dxl.sh         # Launcher script
├── tests/                        # Unit & integration tests
│   ├── test_isaac_only.py       # Isaac Sim integration test
│   └── test_bridge_components.py # Component unit tests
├── tools/                        # Development tools
│   └── hardware/                # Hardware diagnostic utilities
│       ├── dxl_idscan.py        # Dynamixel motor ID scanner
│       ├── dxl_change_id.py     # Dynamixel motor ID changer
│       ├── motor_test_single.py # Dynamixel single motor test
│       ├── esp32_pwm/           # ESP32 PWM motor control (alternative)
│       │   ├── send_receive.py  # ESP32 command-line tool
│       │   ├── usb_comm.py      # ESP32 communication library
│       │   ├── usb_monitor_raw.py # ESP32 debug monitor
│       │   └── README.md        # ESP32 system documentation
│       └── README.md            # Hardware tools documentation
├── assets/                       # Robot models and resources
│   └── ROBOT.usd                # Custom robot USD file
├── config/                       # Configuration files
│   └── config.example.py        # Example configuration
├── docs/                         # Documentation
│   ├── HARDWARE_SETUP.md        # Hardware setup guide
│   └── TROUBLESHOOTING.md       # Common issues & solutions
├── requirements.txt             # Python dependencies
├── LICENSE                      # MIT License
├── CONTRIBUTING.md              # Contribution guidelines
└── README.md                    # This file
```

## 🔧 Configuration

Copy and customize the example config:

```bash
cp config/config.example.py config/config.py
# Edit config/config.py with your hardware settings
```

Key configuration options:
- `DEVICE_PORT`: Serial port (default: `/dev/ttyUSB0`)
- `MOTOR_IDS`: Motor ID mapping [FL, FR, RL, RR]
- `VELOCITY_LIMIT`: Maximum motor velocity
- `UPDATE_RATE_HZ`: Control loop frequency

## 🧹 Maintenance & Cleanup

### Quick Cleanup

Use the automated cleanup script to remove build artifacts and cache files:

```bash
cd ~/Desktop/isaac-sim2real
./scripts/cleanup.sh
```

This interactive script will:
- Clean ROS workspace (build, install, log directories)
- Remove Python cache files (__pycache__, .pyc, .pytest_cache)
- Optionally recreate Python virtual environment
- Optionally rebuild ROS workspace

### Manual Cleanup

**Clean ROS Workspace:**
```bash
cd ~/Desktop/isaac-sim2real/isaac_ros_ws
rm -rf build install log
```

**Rebuild ROS Workspace:**
```bash
cd ~/Desktop/isaac-sim2real/isaac_ros_ws
source /opt/ros/humble/setup.bash  # or your ROS distro
colcon build --symlink-install
source install/setup.bash
```

**Recreate Python Environment:**
```bash
cd ~/Desktop/isaac-sim2real
rm -rf .venv
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

**Clean Python Cache:**
```bash
find . -type d -name "__pycache__" -not -path "./.venv/*" -exec rm -rf {} + 2>/dev/null
find . -type f -name "*.pyc" -not -path "./.venv/*" -delete 2>/dev/null
find . -type d -name ".pytest_cache" -not -path "./.venv/*" -exec rm -rf {} + 2>/dev/null
```

See [CLEANUP_SUMMARY.md](CLEANUP_SUMMARY.md) for detailed cleanup documentation.

## 🧪 Testing Components

Test each component independently before running the full bridge:

```bash
# 1. Test hardware only (no Isaac Sim) - Use virtual environment Python
cd ~/Desktop/isaac-sim2real
source .venv/bin/activate
python src/simple_gui_test.py

# 2. Test Isaac Sim only (no hardware) - Use Isaac Sim Python
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/isaac-sim2real/tests/test_isaac_only.py

# 3. Run full integration tests - Use virtual environment Python
cd ~/Desktop/isaac-sim2real
source .venv/bin/activate
pytest tests/ -v
```

**Important:** 
- Scripts that import `isaacsim` **must** use Isaac Sim's `python.sh`
- Scripts that only test hardware can use the virtual environment
- See [HOW_TO_RUN.md](HOW_TO_RUN.md) for details

## 📚 Documentation

- **[Hardware Setup Guide](docs/HARDWARE_SETUP.md)** - Detailed hardware assembly and wiring
- **[Troubleshooting](docs/TROUBLESHOOTING.md)** - Common issues and solutions
- **[Contributing](CONTRIBUTING.md)** - How to contribute to this project

## 🐛 Troubleshooting

### Common Issues

**Permission denied on /dev/ttyUSB0**
```bash
sudo usermod -aG dialout $USER
newgrp dialout
```

**Motors not responding**
- Verify power supply is connected (12V)
- Check motor IDs: `python xl430-w250-T_tests/dxl_idscan.py`
- Ensure baudrate matches (default: 57600)

**Isaac Sim import errors**
```bash
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh -m pip install dynamixel-sdk pynput
```

**Robot not visible in viewport**
- Press `F` to frame camera on robot
- Check terminal for USD loading errors
- Verify `ROBOT.usd` exists at `~/Desktop/ROBOT.usd`

See [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) for more solutions.

## � Contributing

Contributions are welcome! Please read [CONTRIBUTING.md](CONTRIBUTING.md) before submitting PRs.

### Development Setup

```bash
# Install development dependencies
pip install -r requirements.txt
pip install black pytest pytest-cov

# Format code
black src/ tests/

# Run tests
pytest tests/ -v --cov=src
```

## 📄 License

This project is licensed under the MIT License - see [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- NVIDIA Isaac Sim team for the simulation platform
- ROBOTIS for Dynamixel SDK and hardware
- Contributors and testers

## � Support

- **Issues**: [GitHub Issues](https://github.com/isaac-sim/IsaacSim/issues)
- **Documentation**: See `docs/` directory
- **Hardware**: 
  - [XL430-W250-T e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/)
  - [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- **Isaac Sim**: [Official Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)

---

**Note**: This is a development prototype. Always test in a safe environment with emergency stop procedures in place.