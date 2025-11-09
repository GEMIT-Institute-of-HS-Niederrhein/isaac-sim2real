# Hardware Setup Guide

## Required Hardware

### Dynamixel Servos
- 4x Dynamixel XL430-W250-T servos
- 1x U2D2 USB adapter
- 1x 12V power supply (SMPS recommended)
- TTL cable(s) for daisy-chaining

### Computer Requirements
- Ubuntu 20.04+ (tested on 24.04)
- NVIDIA GPU (for Isaac Sim)
- USB port for U2D2

## Physical Setup

### 1. Motor Configuration

Default motor IDs (can be changed in config):
- Motor 1: Front-Left (FL)
- Motor 2: Front-Right (FR)
- Motor 3: Rear-Left (RL)
- Motor 4: Rear-Right (RR)

### 2. Wiring Diagram

```
Power Supply (12V)
    │
    ├─── Motor 1 (FL) ──┬── Motor 2 (FR) ──┬── Motor 3 (RL) ──┬── Motor 4 (RR)
    │                   │                   │                   │
    └─── U2D2 ──────────┘                   │                   │
                TTL Daisy Chain ─────────────┴───────────────────┘
```

### 3. Motor ID Setup

If you need to change motor IDs, use the Dynamixel Wizard or:

```bash
cd ~/Desktop/Prototype
source .venv/bin/activate
python xl430-w250-T_tests/dxl_change_id.py
```

### 4. Serial Port Permissions

```bash
# Add user to dialout group
sudo usermod -aG dialout $USER

# Apply immediately (or logout/login)
newgrp dialout

# Verify
ls -l /dev/ttyUSB0
```

## Troubleshooting

### Cannot open /dev/ttyUSB0
- Check USB connection
- Verify permissions (see above)
- Try different USB port
- Check `dmesg | grep tty` for device detection

### Motors not responding
- Verify power supply is connected and 12V
- Check TTL cable connections
- Run motor scan: `python xl430-w250-T_tests/dxl_idscan.py`
- Verify baudrate matches (default: 57600)

### Motors overheat
- Reduce VELOCITY_LIMIT in config
- Add cooling or heat sinks
- Check for mechanical binding
- Reduce continuous operation time

### Communication errors
- Check cable quality (use official Dynamixel cables)
- Shorten cable runs if possible
- Verify U2D2 firmware is up to date
- Try lower baudrate (e.g., 115200 → 57600)

## Safety Notes

⚠️ **Important Safety Guidelines**:

1. **Test in safe environment** - Motors can generate significant torque
2. **Emergency stop** - Keep spacebar (stop) within reach
3. **Power disconnection** - Know where the power supply switch is
4. **Mechanical limits** - Ensure robot has room to move
5. **Supervision** - Never leave running unattended

## Next Steps

After hardware setup:
1. Run verification: `scripts/verify_setup.py`
2. Test motors only: `src/simple_gui_test.py`
3. Test Isaac Sim only: `tests/test_isaac_only.py`
4. Run full bridge: `src/isaac_dxl_bridge.py`

See main README.md for detailed software setup instructions.
