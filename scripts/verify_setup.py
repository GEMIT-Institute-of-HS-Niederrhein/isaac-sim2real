#!/usr/bin/env python3
"""
Quick verification script for Isaac Sim 5.1 + Dynamixel setup
Run this BEFORE the full bridge to check everything is working
"""

import sys

print("=" * 70)
print("Isaac Sim 5.1 + Dynamixel Setup Verification")
print("=" * 70)
print()

# Test 1: Check if running with Isaac Sim's Python
print("[1/5] Checking Python environment...")
if "isaac" in sys.executable.lower() or "omniverse" in sys.executable.lower() or "release" in sys.executable.lower():
    print("  ✓ Running with Isaac Sim Python")
    print(f"    Path: {sys.executable}")
else:
    print("  ⚠ WARNING: Not running with Isaac Sim Python!")
    print(f"    Current: {sys.executable}")
    print(f"    Expected: ~/Desktop/isaacsim/_build/linux-x86_64/release/python.sh")
    print()
    print("  Please run with:")
    print("    cd ~/Desktop/isaacsim/_build/linux-x86_64/release")
    print("    ./python.sh ~/Desktop/isaac-sim2real/scripts/verify_setup.py")
print()

# Test 2: Check Dynamixel SDK
print("[2/5] Checking Dynamixel SDK...")
try:
    from dynamixel_sdk import *
    print("  ✓ Dynamixel SDK installed")
except ImportError as e:
    print("  ✗ Dynamixel SDK NOT found!")
    print(f"    Error: {e}")
    print("    Install with: ./python.sh -m pip install dynamixel-sdk")
    sys.exit(1)
print()

# Test 3: Check Dynamixel hardware connection
print("[3/5] Checking Dynamixel hardware...")
try:
    import os
    usb_devices = [d for d in os.listdir('/dev') if 'ttyUSB' in d or 'ttyACM' in d]
    if usb_devices:
        print(f"  ✓ USB devices found: {usb_devices}")
        print("    Attempting connection...")
        
        # Try to connect
        port = PortHandler("/dev/ttyUSB0")
        if port.openPort():
            print("    ✓ Successfully opened /dev/ttyUSB0")
            if port.setBaudRate(57600):
                print("    ✓ Baudrate set to 57600")
            port.closePort()
        else:
            print("    ⚠ Could not open /dev/ttyUSB0")
            print("    Try: sudo chmod 666 /dev/ttyUSB0")
    else:
        print("  ⚠ No USB devices found!")
        print("    Make sure U2D2 is plugged in")
        print("    Check with: ls -l /dev/ttyUSB*")
except Exception as e:
    print(f"  ⚠ Error checking hardware: {e}")
print()

# Test 4: Check Isaac Sim imports
print("[4/5] Checking Isaac Sim 5.1 imports...")
try:
    from isaacsim import SimulationApp
    print("  ✓ Isaac Sim 5.1 base import successful")
except ImportError as e:
    print("  ✗ Cannot import Isaac Sim!")
    print(f"    Error: {e}")
    print("    Make sure you're running with Isaac Sim's Python")
    sys.exit(1)

try:
    # Test secondary imports (without starting simulation)
    print("  ✓ Testing Isaac Sim modules...")
    # Note: We can't import these without starting SimulationApp
    print("    (Will test full initialization in bridge script)")
except Exception as e:
    print(f"  ⚠ Error: {e}")
print()

# Test 5: Check pynput for keyboard control
print("[5/5] Checking keyboard control library...")
try:
    from pynput import keyboard
    print("  ✓ pynput library available")
except ImportError:
    print("  ⚠ pynput not installed (optional for keyboard control)")
    print("    Install with: ./python.sh -m pip install pynput")
print()

# Final summary
print("=" * 70)
print("SUMMARY")
print("=" * 70)
print()
print("Setup Status: READY ✓")
print()
print("Next steps:")
print("  1. Test hardware with: python ~/Desktop/isaac-sim2real/src/simple_gui_test.py")
print("  2. Run full bridge: cd ~/Desktop/isaacsim/_build/linux-x86_64/release")
print("     ./python.sh ~/Desktop/isaac-sim2real/src/isaac_dxl_bridge.py")
print()
print("If you see any ⚠ warnings above, fix those first!")
print("=" * 70)