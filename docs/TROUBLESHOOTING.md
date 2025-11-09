# Troubleshooting Guide

## Common Issues and Solutions

### Setup Issues

#### Virtual Environment Problems

**Problem**: `command not found: python3`
```bash
# Solution: Install Python 3
sudo apt update
sudo apt install python3 python3-pip python3-venv
```

**Problem**: `ModuleNotFoundError` when running scripts
```bash
# Solution: Activate venv and install requirements
cd ~/Desktop/Prototype
source .venv/bin/activate
pip install -r requirements.txt
```

#### Isaac Sim Issues

**Problem**: Isaac Sim not found
```bash
# Solution: Verify Isaac Sim path
ls ~/Desktop/isaacsim/_build/linux-x86_64/release/python.sh
# Update path in scripts if needed
```

**Problem**: CUDA/GPU errors
- Check NVIDIA driver: `nvidia-smi`
- Verify GPU compute capability ≥ 5.0
- Update drivers if needed

**Problem**: Import errors in Isaac Sim
```bash
# Solution: Install packages to Isaac Sim's Python
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh -m pip install dynamixel-sdk pynput
```

### Hardware Issues

#### Serial Communication

**Problem**: Permission denied on `/dev/ttyUSB0`
```bash
# Solution 1: Temporary fix
sudo chmod 666 /dev/ttyUSB0

# Solution 2: Permanent fix (recommended)
sudo usermod -aG dialout $USER
# Then logout and login
```

**Problem**: Device not found
```bash
# Check USB devices
lsusb
dmesg | grep tty

# Verify U2D2 connection
ls /dev/ttyUSB*
```

#### Motor Problems

**Problem**: Motors don't move
1. Check power supply (12V, sufficient amperage)
2. Verify motor IDs with scan: `python xl430-w250-T_tests/dxl_idscan.py`
3. Check torque enable in code
4. Test individual motor: `python xl430-w250-T_tests/xl430_single_test.py`

**Problem**: Jerky or inconsistent movement
- Reduce velocity limit in config
- Check for loose cable connections
- Verify power supply is stable
- Add small delay in control loop

**Problem**: Communication timeouts
```python
# Increase timeout in code
port_handler.setPacketTimeout(100)  # milliseconds
```

### Software Issues

#### Keyboard Control

**Problem**: Arrow keys don't work in bridge
- Ensure Isaac Sim window has focus
- Try WASD keys as alternative
- Check if running on Wayland (try Xorg session)
- Verify pynput is installed

**Problem**: Global keyboard hook fails
```bash
# Solution: Use Xorg instead of Wayland
# At login screen, select "Ubuntu on Xorg"
```

#### GUI Issues

**Problem**: Tkinter import error
```bash
# Solution: Install tkinter
sudo apt install python3-tk
```

**Problem**: GUI window doesn't appear
- Check DISPLAY variable: `echo $DISPLAY`
- Verify X server is running
- Try: `xhost +local:`

#### Isaac Sim Viewport

**Problem**: Robot not visible in viewport
1. Check if USD loaded: Look in stage hierarchy
2. Camera might need repositioning: Press 'F' to frame selection
3. Verify lighting in scene
4. Check for errors in terminal output

**Problem**: Slow performance
- Reduce physics simulation quality
- Close other GPU applications
- Check GPU memory: `nvidia-smi`
- Reduce viewport resolution

### Testing & Debugging

#### Verification Script Fails

```bash
# Run with verbose output
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/Prototype/scripts/verify_setup.py 2>&1 | tee verify.log
```

Check each section:
1. Python environment ✓
2. Dynamixel SDK ✓
3. Hardware connection ✓
4. Isaac Sim imports ✓
5. pynput library ✓

#### Integration Issues

**Problem**: Bridge crashes on startup
1. Check all dependencies installed
2. Verify hardware connections first
3. Test components separately:
   - `src/simple_gui_test.py` (hardware only)
   - `tests/test_isaac_only.py` (Isaac Sim only)
4. Check terminal for error messages

### Performance Issues

#### High CPU Usage
- Reduce UPDATE_RATE_HZ in config
- Disable status printing (set interval higher)
- Check for infinite loops in custom code

#### High GPU Memory
- Close other applications using GPU
- Reduce Isaac Sim viewport size
- Lower physics simulation quality

#### Latency in Motor Response
- Check USB cable quality
- Reduce serial communication overhead
- Increase priority of Python process:
  ```bash
  sudo nice -n -10 ./python.sh script.py
  ```

## Getting Help

If you're still stuck:

1. **Check logs**: Look in `~/.cache/packman/chk/kit-kernel/.../logs/`
2. **Enable verbose output**: Add `print()` statements for debugging
3. **Test incrementally**: Start simple, add complexity
4. **Check hardware**: Use Dynamixel Wizard to test motors
5. **Open an issue**: Include:
   - Error messages (full stack trace)
   - System info (`lsb_release -a`, `nvidia-smi`)
   - Steps to reproduce
   - Hardware configuration

## Useful Commands

```bash
# Check Python version
python3 --version

# Check Isaac Sim Python
~/Desktop/isaacsim/_build/linux-x86_64/release/python.sh --version

# List USB devices
lsusb

# Check serial devices
ls -l /dev/ttyUSB*

# Monitor system resources
htop
nvidia-smi -l 1

# Test motor scan
cd ~/Desktop/Prototype
source .venv/bin/activate
python xl430-w250-T_tests/dxl_idscan.py

# Check package versions
pip list | grep -i dynamixel
pip list | grep -i pynput
```

## Additional Resources

- [Dynamixel SDK Documentation](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [XL430-W250 Manual](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/)
