# Hardware Utility Tools

This directory contains standalone diagnostic and setup tools for multiple hardware-in-the-loop systems:

- **Dynamixel XL430-W250-T** - Smart servo motors with Protocol 2.0 (main system)
- **ESP32 PWM Motors** - DC motors controlled via ESP32 microcontroller (alternative system)

These tools help you configure, test, and troubleshoot your hardware before running the full sim-to-real bridge.

## 📋 Prerequisites

```bash
# Install Dynamixel SDK (if not already installed)
pip install dynamixel-sdk

# For ESP32 PWM system, install pyserial
pip install pyserial

# On Linux, ensure serial port permissions
sudo usermod -aG dialout $USER
newgrp dialout  # or logout and login again
```

## 🔧 Hardware Systems

### System 1: Dynamixel XL430-W250-T (Primary)
- **Control**: Position/Velocity via Protocol 2.0
- **Port**: `/dev/ttyUSB0` (via U2D2 adapter)
- **Tools**: `dxl_idscan.py`, `dxl_change_id.py`, `motor_test_single.py`
- **Use Case**: Precise servo control for wheeled robots

### System 2: ESP32 PWM Motors (Alternative)
- **Control**: PWM duty cycle (0-255) via custom protocol
- **Port**: `/dev/ttyESP32` (direct USB connection)
- **Tools**: `esp32_pwm/send_receive.py`, `esp32_pwm/usb_comm.py`
- **Use Case**: DC motor control, lower cost alternative

---

## 🔧 Dynamixel Tools (Primary System)

Located in: `tools/hardware/`

### 1. `dxl_idscan.py` - Motor ID Scanner

**Purpose**: Scan the Dynamixel bus to discover which motor IDs are connected and responding.

**Use Cases**:
- Initial hardware setup verification
- Finding motors after wiring changes
- Troubleshooting communication issues
- Checking multiple baud rates

**Basic Usage**:
```bash
# Scan IDs 1-8 at default baud rate (57600)
python dxl_idscan.py

# Scan specific ID range
python dxl_idscan.py --ids 1-12

# Try multiple baud rates
python dxl_idscan.py --bauds 57600,115200,1000000

# Windows example
python dxl_idscan.py --port COM7 --ids 1-8
```

**Common Options**:
- `--port`: Serial port (default: `/dev/ttyUSB0`)
- `--ids`: ID range to scan (default: `1-8`, supports `1,2,3` or `1-5` or `1,3,5-8`)
- `--baud`: Single baud rate (default: `57600`)
- `--bauds`: Multiple baud rates (overrides `--baud`)

**Expected Output**:
```
[INFO] Scanning 8 ID(s) on /dev/ttyUSB0 @ 57600 …
  [FOUND] ID   1 → model 1060
  [FOUND] ID   2 → model 1060
  [FOUND] ID   3 → model 1060
  [FOUND] ID   4 → model 1060

=== Scan Summary ===
Found 4 device(s) at 57600 baud:
  - ID   1 → model 1060
  - ID   2 → model 1060
  - ID   3 → model 1060
  - ID   4 → model 1060
```

**Model Number Reference**:
- `1060` = XL430-W250-T (our target motor)
- `1020` = XL320
- Other values = See [ROBOTIS e-Manual](https://emanual.robotis.com/)

---

### 2. `dxl_change_id.py` - Motor ID Changer

**Purpose**: Change the ID of a single Dynamixel motor. Essential for setting up multiple motors on the same bus.

**⚠️ SAFETY**: Connect **only the motor you want to change** to avoid accidentally changing multiple IDs.

**Basic Usage**:
```bash
# Change motor from ID 1 to ID 3
python dxl_change_id.py --port /dev/ttyUSB0 --old 1 --new 3

# Windows example
python dxl_change_id.py --port COM7 --baud 57600 --old 1 --new 4
```

**Options**:
- `--port`: Serial port (required)
- `--old`: Current motor ID (required)
- `--new`: Desired new ID (required, 0-252)
- `--baud`: Baud rate (default: `57600`)
- `--force`: Override collision check (NOT recommended)

**Workflow for Setting Up 4 Motors**:
```bash
# 1. Connect ONLY motor #1, change to ID 1
python dxl_idscan.py  # Find current ID (often default: 1)
python dxl_change_id.py --port /dev/ttyUSB0 --old 1 --new 1

# 2. Connect ONLY motor #2, change to ID 2
python dxl_change_id.py --port /dev/ttyUSB0 --old 1 --new 2

# 3. Connect ONLY motor #3, change to ID 3
python dxl_change_id.py --port /dev/ttyUSB0 --old 1 --new 3

# 4. Connect ONLY motor #4, change to ID 4
python dxl_change_id.py --port /dev/ttyUSB0 --old 1 --new 4

# 5. Connect all motors, verify
python dxl_idscan.py --ids 1-4
```

**Expected Output**:
```
[INFO] Opening /dev/ttyUSB0 @ 57600...
[INFO] Checking current ID 1...
[OK] Found device model 1060 on ID 1.
[INFO] Changing ID 1 -> 3 ...
[INFO] Verifying new ID 3...
[SUCCESS] ID changed: 1 -> 3 (model 1060).
[INFO] Done. Torque remains disabled for safety.
```

---

### 3. `motor_test_single.py` - Single Motor Test

**Purpose**: Test a single motor with position control. Moves through a simple sequence to verify:
- Communication works
- Motor can hold position
- Position feedback is accurate

**Basic Usage**:
```bash
# Test motor at ID 3 (update DXL_ID in script first)
python motor_test_single.py
```

**Configuration** (edit in script):
```python
PORT_NAME  = "/dev/ttyUSB0"  # Your serial port
BAUDRATE   = 57600           # Match motor settings
DXL_ID     = 3               # Motor ID to test
```

**Test Sequence**:
1. Opens serial port
2. Pings motor to confirm communication
3. Sets Position Control Mode
4. Enables torque
5. Moves: 0° → 90° → 0°
6. Prints position feedback during movement
7. Disables torque and closes port

**Expected Output**:
```
Opening port...
Pinging servo...
✓ Connected: DXL ID 3, model 1060
Setting Position Mode...
Enabling torque...
Moving to 0.0° ...
  → Position now:    2.11°
  → Position now:    1.58°
  → Position now:    0.26°
Moving to 90.0° ...
  → Position now:   45.67°
  → Position now:   88.42°
  → Position now:   90.18°
Moving to 0.0° ...
  → Position now:   45.23°
  → Position now:    1.05°
Done.
Disabling torque and closing port...
```

**Troubleshooting**:
- If motor doesn't move: Check power supply voltage (11-14.8V recommended)
- If position drifts: Increase holding torque or reduce mechanical load
- If overheating: Reduce test duration or add cooling

---

## 🔄 Typical Workflow

### Initial Setup
```bash
# 1. Scan for motors
python dxl_idscan.py

# 2. If needed, change IDs (one motor at a time!)
python dxl_change_id.py --port /dev/ttyUSB0 --old 1 --new 2

# 3. Test each motor individually
# (edit DXL_ID in motor_test_single.py)
python motor_test_single.py
```

### Before Running Bridge
```bash
# Quick check that all 4 motors respond
python dxl_idscan.py --ids 1-4

# Verify configuration with project's verify script
cd ~/Desktop/isaac-sim2real
python scripts/verify_setup.py
```

### Troubleshooting
```bash
# Can't find motors? Try multiple baud rates
python dxl_idscan.py --bauds 57600,115200,1000000

# Check if specific motor is alive
python dxl_idscan.py --ids 3

# Test individual motor movement
python motor_test_single.py  # (after editing DXL_ID)
```

---

## 📚 Additional Resources

- **Dynamixel SDK Documentation**: https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
- **XL430-W250-T Manual**: https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/
- **Control Table**: https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table
- **U2D2 Setup Guide**: https://emanual.robotis.com/docs/en/parts/interface/u2d2/

---

## ⚠️ Safety Notes

1. **Power Supply**: Use 12V nominal (11-14.8V range). DO NOT exceed 16V.
2. **Current Draw**: Each XL430 can draw up to 1.5A under load. Ensure adequate power supply capacity.
3. **Heat**: Motors get warm during extended operation. Add ventilation or heatsinks if needed.
4. **Wiring**: Always check yellow signal wire alignment before powering on.
5. **Emergency Stop**: Keep power disconnection accessible during testing.
6. **Single Motor Operations**: When changing IDs, connect ONLY the target motor to avoid conflicts.

---

## 🐛 Common Issues

### Motor Not Found
- **Check**: Power supply ON and connected
- **Check**: Yellow signal wire aligned correctly (not reversed)
- **Check**: USB cable connected to U2D2
- **Try**: Different baud rate: `--bauds 57600,115200,1000000`
- **Try**: Different USB port or cable

### Permission Denied on /dev/ttyUSB0
```bash
sudo usermod -aG dialout $USER
newgrp dialout  # or logout/login
```

### Motor Vibrates or Doesn't Hold Position
- Increase voltage (closer to 12V)
- Reduce mechanical load
- Check for loose horn/wheel connections
- Verify motor isn't in Velocity or PWM mode

### Communication Errors (error=6, error=3)
- Velocity Limit may be too low → use main bridge scripts which set proper limits
- Check baud rate matches motor settings
- Verify U2D2 firmware is up to date

---

## 🎮 ESP32 PWM Tools (Alternative System)

Located in: `tools/hardware/esp32_pwm/`

### Quick Start

```bash
# Control Motor ID 1, CounterClockwise, PWM 200
python esp32_pwm/send_receive.py "01CCWPWM200"

# Control multiple motors
python esp32_pwm/send_receive.py "01CCWPWM20002CWSPWM150"

# Emergency stop
python esp32_pwm/send_receive.py "BRAKE"
```

### Command Syntax

**Single Motor:**
- Format: `[ID:2][DIRECTION:2-3][MODE:3-4][VALUE:1-3]`
- Example: `01CCWPWM200` = Motor 1, CounterClockwise, PWM 200

**Multi-Motor (concatenated):**
- Example: `01CCWPWM20002CWSPWM15503CCWPWM250`

**Special Commands:**
- `BRAKE` - Emergency stop (active braking)
- `COAST` - Release motors (passive stop)

### Key Files

1. **`send_receive.py`** - Command-line interface
   ```bash
   python esp32_pwm/send_receive.py "01CCWPWM200"
   ```

2. **`usb_comm.py`** - Core communication library
   - Frame-based protocol with CRC-16 checksums
   - Robust error handling (timeout, CRC, format errors)
   - Context manager support

3. **`usb_monitor_raw.py`** - Debug monitor
   ```bash
   python esp32_pwm/usb_monitor_raw.py /dev/ttyESP32
   ```

### Documentation

For detailed ESP32 PWM system documentation, see:
**[ESP32 PWM README](esp32_pwm/README.md)**

Includes:
- Protocol specification with CRC-16
- Python API reference
- Integration examples
- Troubleshooting guide
- Future Isaac Sim bridge plans

---

For integration with Isaac Sim, see the main project README and use the scripts in `src/` and `scripts/`.
