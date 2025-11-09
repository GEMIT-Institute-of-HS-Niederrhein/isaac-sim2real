# ESP32 PWM Motor Control

USB communication system for controlling motors via ESP32 microcontroller with PWM commands. This provides an alternative hardware-in-the-loop setup for motor control experiments.

## 📋 Overview

This module provides a robust serial communication protocol for controlling motors through an ESP32 board. Unlike the Dynamixel system (which uses Protocol 2.0), this uses a custom frame-based protocol with CRC-16 CCITT checksums for reliability.

**Key Differences from Dynamixel:**
- **Communication**: USB serial (ESP32) vs. Dynamixel Protocol 2.0
- **Control**: PWM values (0-255) vs. Velocity/Position commands
- **Hardware**: ESP32 microcontroller vs. Dynamixel servos
- **Port**: `/dev/ttyESP32` vs. `/dev/ttyUSB0`

## 🔧 Hardware Setup

### Components
- ESP32 development board
- DC motors with H-bridge motor drivers
- USB cable (ESP32 to computer)
- Power supply for motors

### Connections
1. Connect ESP32 to computer via USB
2. Ensure `/dev/ttyESP32` is accessible (may need udev rules)
3. Connect motors to ESP32 PWM pins via motor drivers

### Permissions
```bash
# Check if device exists
ls -l /dev/ttyESP32

# If needed, add user to dialout group
sudo usermod -aG dialout $USER
newgrp dialout
```

## 📝 Protocol Format

### Frame Structure
```
[LENGTH:4hex][PAYLOAD:n bytes][*][CRC:4hex][\n]
```

**Example:**
```
000C01CCWPWM200*A3F2\n
^^^^            ^^^^
length (12)     CRC-16
```

### Command Syntax

**Single Motor:**
```
[ID:2][DIRECTION:2-3][MODE:3-4][VALUE:1-3]
```

**Examples:**
- `01CCWPWM200` - Motor 1, CounterClockwise, PWM 200
- `02CWSPWM150` - Motor 2, Clockwise, PWM 150
- `03CCWPWM255` - Motor 3, CounterClockwise, PWM 255 (max)

**Multi-Motor (concatenated):**
```
01CCWPWM20002CWSPWM15503CCWPWM250
```

**Special Commands:**
- `BRAKE` - Emergency stop (active braking)
- `COAST` - Release motors (passive stop)

### Command Components

| Component | Description | Examples |
|-----------|-------------|----------|
| **ID** | Motor ID (2 digits) | `01`, `02`, `03` |
| **Direction** | Rotation direction | `CW` (Clockwise), `CCW` (CounterClockwise) |
| **Mode** | Control mode | `PWM`, `SPD` (if supported) |
| **Value** | PWM duty cycle | `0-255` (0=stop, 255=max) |

## 🚀 Usage

### 1. Basic Motor Control Script

```bash
# Control Motor ID 1, CounterClockwise, PWM 200
python send_receive.py "01CCWPWM200"

# Control two motors simultaneously
python send_receive.py "01CCWPWM20002CWSPWM150"

# Three motors control
python send_receive.py "01CCWPWM20002CWSPWM15503CCWPWM250"

# Emergency stop
python send_receive.py "BRAKE"

# Release motors (coast to stop)
python send_receive.py "COAST"
```

### 2. Python API Usage

```python
from usb_comm import USBComm

# Send a single command
with USBComm("/dev/ttyESP32") as comm:
    comm.send("01CCWPWM200")
    payload, status = comm.recv(timeout=0.5)
    print(f"Response: {payload}, Status: {status}")
```

### 3. Monitoring Raw Serial Output

```bash
# Monitor raw bytes from ESP32 (debugging)
python usb_monitor_raw.py /dev/ttyESP32
```

## 🐍 Module Reference

### `usb_comm.py` - Core Communication Module

#### `USBComm` Class

**Constructor:**
```python
USBComm(port="/dev/ttyESP32", baud=115200, timeout=1.0)
```

**Methods:**

##### `send(payload: str) -> None`
Send a command string to the ESP32.

```python
with USBComm() as comm:
    comm.send("01CCWPWM200")
```

##### `recv(timeout=None) -> tuple[str | None, str]`
Receive a response from ESP32.

**Returns:**
- `(payload, "ok")` - Successful read with valid CRC
- `(None, "timeout")` - No data received within timeout
- `(None, "crc")` - CRC mismatch (corrupted data)
- `(None, "format")` - Invalid frame format

```python
with USBComm() as comm:
    comm.send("01CCWPWM200")
    payload, status = comm.recv(timeout=1.0)
    if status == "ok":
        print(f"Success: {payload}")
    else:
        print(f"Error: {status}")
```

#### `crc16_ccitt_false(data: bytes) -> int`
Calculate CRC-16 CCITT (0xFFFF initial, 0x1021 polynomial).

### `send_receive.py` - Command-Line Tool

Simple CLI wrapper for sending commands via command-line arguments.

**Usage:**
```bash
python send_receive.py "<command>"
```

**Default command** (if no argument): `01CWSPWM25502CCWPWM25503CCWPWM255`

### `usb_monitor_raw.py` - Debugging Tool

Raw serial port monitor for troubleshooting communication issues.

**Usage:**
```bash
python usb_monitor_raw.py [port]  # default: /dev/ttyESP32
```

## 🔄 Integration with Isaac Sim

This system can be integrated alongside the Dynamixel bridge to control different motor types:

```python
# Future integration example (not yet implemented)
from tools.hardware.esp32_pwm.usb_comm import USBComm
from src.isaac_dxl_bridge import DynamixelController

# Control both systems simultaneously
with USBComm() as esp_motors, DynamixelController() as dxl_motors:
    # ESP32 motors (PWM control)
    esp_motors.send("01CCWPWM200")
    
    # Dynamixel motors (velocity control)
    dxl_motors.set_velocities([0.5, 0.5, 0.5, 0.5])
```

## 🧪 Testing

### Test Individual Motor
```bash
# Test motor 1 at low speed
python send_receive.py "01CCWPWM50"

# Test motor 1 at medium speed
python send_receive.py "01CCWPWM150"

# Test motor 1 at high speed
python send_receive.py "01CCWPWM255"

# Stop
python send_receive.py "BRAKE"
```

### Test Multiple Motors
```bash
# All motors forward at same speed
python send_receive.py "01CCWPWM20002CCWPWM20003CCWPWM200"

# Differential (left slower than right for turning)
python send_receive.py "01CCWPWM10002CCWPWM200"
```

### Test Error Handling
```bash
# Monitor for communication errors
python usb_monitor_raw.py /dev/ttyESP32 &
python send_receive.py "01CCWPWM200"
```

## ⚠️ Safety Notes

1. **PWM Limits**: Start with low values (50-100) before increasing to 255
2. **Emergency Stop**: Always have `BRAKE` command ready
3. **Power Supply**: Ensure adequate current capacity for all motors
4. **Wiring**: Double-check motor driver connections before powering on
5. **Timeout**: Set appropriate timeouts to detect communication failures

## 🐛 Troubleshooting

### Device Not Found: `/dev/ttyESP32`

**Solution 1: Check USB connection**
```bash
# List all USB serial devices
ls -l /dev/tty* | grep USB
ls -l /dev/tty* | grep ACM

# ESP32 typically shows as ttyUSB0 or ttyACM0
```

**Solution 2: Create udev rule for consistent naming**
```bash
# Find vendor/product ID
lsusb | grep -i esp

# Create udev rule (example)
sudo nano /etc/udev/rules.d/99-esp32.rules

# Add line (replace XXXX:YYYY with your ESP32 IDs):
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", SYMLINK+="ttyESP32"

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Solution 3: Modify port in code**
```python
# If ESP32 appears as /dev/ttyUSB1
with USBComm("/dev/ttyUSB1") as comm:
    comm.send("01CCWPWM200")
```

### Motors Not Responding

1. **Check ESP32 firmware**: Ensure ESP32 is programmed with motor control firmware
2. **Check baud rate**: Default is 115200, verify ESP32 matches
3. **Monitor raw output**: Use `usb_monitor_raw.py` to see ESP32 responses
4. **Check wiring**: Verify motor driver connections and power supply

### CRC Errors

```python
payload, status = comm.recv()
if status == "crc":
    print("CRC mismatch - retry command")
    comm.send("01CCWPWM200")  # Retry
```

**Common causes:**
- Electrical noise on USB cable
- Loose connections
- Insufficient power supply causing brownouts

### Timeout Errors

```python
payload, status = comm.recv(timeout=2.0)  # Increase timeout
if status == "timeout":
    print("No response from ESP32")
    # Check if device is connected and powered
```

## 📚 Protocol Details

### CRC-16 CCITT (False)
- **Initial value**: `0xFFFF`
- **Polynomial**: `0x1021`
- **Reflected**: No
- **Final XOR**: None

### Baud Rate
- **Default**: 115200
- **Alternatives**: 9600, 57600, 115200, 230400 (configure in ESP32 firmware)

### Frame Timing
- **Buffer clear**: 150ms after opening port
- **Write timeout**: 0.5s
- **Read timeout**: Configurable (default 1.0s)

## 🔗 Related Files

- `send_receive.py` - Command-line interface
- `usb_comm.py` - Core communication library
- `usb_monitor_raw.py` - Debug monitor
- `../dxl_idscan.py` - Dynamixel motor scanner (different system)
- `../../src/isaac_dxl_bridge.py` - Main Isaac Sim bridge (Dynamixel)

## 📝 Future Enhancements

- [ ] Create ESP32 firmware bridge for Isaac Sim integration
- [ ] Add encoder feedback reading
- [ ] Implement speed control mode (closed-loop)
- [ ] Add battery voltage monitoring
- [ ] Create GUI control panel similar to `simple_gui_test.py`
- [ ] Unified bridge controlling both Dynamixel and ESP32 motors

## 📄 Example: Full Control Loop

```python
#!/usr/bin/env python3
"""Example: ESP32 motor control loop with error handling"""

from usb_comm import USBComm
import time

def control_loop():
    with USBComm("/dev/ttyESP32") as comm:
        try:
            # Start motors gradually
            for pwm in range(0, 200, 50):
                cmd = f"01CCWPWM{pwm:03d}02CCWPWM{pwm:03d}"
                comm.send(cmd)
                print(f"Sent: {cmd}")
                
                # Check response
                payload, status = comm.recv(timeout=0.5)
                if status != "ok":
                    print(f"Warning: {status}")
                else:
                    print(f"Response: {payload}")
                
                time.sleep(1)
            
            # Run for 5 seconds
            print("Running at PWM 200...")
            time.sleep(5)
            
            # Stop
            comm.send("BRAKE")
            print("Motors stopped")
            
        except KeyboardInterrupt:
            comm.send("BRAKE")
            print("\nEmergency stop!")

if __name__ == "__main__":
    control_loop()
```

---

For Dynamixel motor control, see `../README.md` in the parent directory.
