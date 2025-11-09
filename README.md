# Isaac Sim2Real 🤖

Real-time bidirectional control bridge between NVIDIA Isaac Sim 5.1 and Dynamixel XL430-W250-T servo motors. **Learn in simulation, transfer to hardware** - enabling seamless sim-to-real transfer for robotics research and development.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.11+](https://img.shields.io/badge/python-3.11+-blue.svg)](https://www.python.org/downloads/)
[![Isaac Sim 5.1](https://img.shields.io/badge/Isaac%20Sim-5.1-green.svg)](https://developer.nvidia.com/isaac-sim)

## ✨ Features

- 🎮 **Real-time Control**: Keyboard-driven control with <100ms latency
- 🔄 **Bi-directional**: Sim → Hardware and Hardware → Sim feedback loops
- 🛡️ **Safe Operation**: Emergency stop, velocity limiting, error handling
- 🧪 **Modular Testing**: Test hardware and simulation independently
- 📊 **Live Monitoring**: Real-time status display and diagnostics
- 🐍 **Clean Python**: Type hints, docstrings, PEP 8 compliant

## 📋 Prerequisites

### Hardware
- 4× Dynamixel XL430-W250-T servo motors
- 1× U2D2 USB communication adapter
- 12V power supply (SMPS recommended)
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

```bash
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/isaac-sim2real/src/isaac_dxl_bridge.py
```

**Keyboard Controls:**
- `↑` / `↓` : Forward / Backward
- `←` / `→` : Turn Left / Right
- `SPACE` : Emergency Stop
- `ESC` : Quit Application

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
│       ├── dxl_idscan.py        # Motor ID scanner
│       ├── dxl_change_id.py     # Motor ID changer
│       ├── motor_test_single.py # Single motor test
│       └── README.md            # Tools documentation
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

## 🧪 Testing Components

Test each component independently before running the full bridge:

```bash
# 1. Test hardware only (no Isaac Sim)
source .venv/bin/activate
python src/simple_gui_test.py

# 2. Test Isaac Sim only (no hardware)
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/isaac-sim2real/tests/test_isaac_only.py

# 3. Run full integration tests
source .venv/bin/activate
pytest tests/ -v
```

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
- **Hardware**: [Dynamixel e-Manual](https://emanual.robotis.com/)
- **Isaac Sim**: [Official Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)

---

**Note**: This is a development prototype. Always test in a safe environment with emergency stop procedures in place.