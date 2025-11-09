#!/bin/bash
# Quick start script for Isaac Sim 5.1 + Dynamixel Bridge
# Run from: ~/Desktop/isaac-sim2real/scripts/
# Make executable: chmod +x run_isaac_dxl.sh

# Your Isaac Sim path
ISAAC_SIM_PATH="$HOME/Desktop/isaacsim/_build/linux-x86_64/release"
REPO_PATH="$HOME/Desktop/isaac-sim2real"

echo "=========================================="
echo "Isaac Sim 5.1 + Dynamixel Quick Start"
echo "=========================================="
echo ""

# Check if Isaac Sim exists
if [ ! -d "$ISAAC_SIM_PATH" ]; then
    echo "ERROR: Isaac Sim not found at $ISAAC_SIM_PATH"
    echo "Please update ISAAC_SIM_PATH in this script"
    exit 1
fi

# Check if python.sh exists
if [ ! -f "$ISAAC_SIM_PATH/python.sh" ]; then
    echo "ERROR: python.sh not found in $ISAAC_SIM_PATH"
    exit 1
fi

echo "Isaac Sim found at: $ISAAC_SIM_PATH"
echo ""

# Menu
echo "Choose an option:"
echo "  1) Install dependencies (dynamixel-sdk, pynput)"
echo "  2) Verify setup"
echo "  3) Test Isaac Sim only (no hardware)"
echo "  4) Test hardware only (GUI)"
echo "  5) Run full bridge (Isaac Sim + Hardware)"
echo "  6) Minimal Jetbot Example (working prototype)"
echo "  7) Jetbot + Servo Integration (IDs 1 & 2)"
echo "  q) Quit"
echo ""
read -p "Enter choice [1-7]: " choice

case $choice in
    1)
        echo ""
        echo "Installing dependencies..."
        cd "$ISAAC_SIM_PATH"
        ./python.sh -m pip install dynamixel-sdk pynput
        echo ""
        echo "Done! Run this script again and choose option 2 to verify."
        ;;
    2)
        echo ""
        echo "Verifying setup..."
        cd "$ISAAC_SIM_PATH"
        ./python.sh "$REPO_PATH/scripts/verify_setup.py"
        ;;
    3)
        echo ""
        echo "Testing Isaac Sim only..."
        echo "(No hardware needed - just opens GUI with robot)"
        cd "$ISAAC_SIM_PATH"
        ./python.sh "$REPO_PATH/tests/test_isaac_only.py"
        ;;
    4)
        echo ""
        echo "Testing Dynamixel hardware with GUI..."
        echo "(Make sure motors are connected!)"
        source "$REPO_PATH/.venv/bin/activate"
        python "$REPO_PATH/src/simple_gui_test.py"
        ;;
    5)
        echo ""
        echo "Starting full bridge..."
        echo "(Make sure motors are connected!)"
        echo ""
        echo "Controls:"
        echo "  Arrow Up/Down  : Forward/Backward"
        echo "  Arrow Left/Right : Turn"
        echo "  SPACE : Stop"
        echo "  ESC : Quit"
        echo ""
        cd "$ISAAC_SIM_PATH"
        ./python.sh "$REPO_PATH/src/isaac_dxl_bridge.py"
        ;;
    
    6)
        echo ""
        echo "Starting Minimal Jetbot Example..."
        echo "(Simple working prototype - no hardware needed)"
        echo ""
        cd "$ISAAC_SIM_PATH"
        ./python.sh "$REPO_PATH/examples/minimal_jetbot.py"
        ;;
    
    7)
        echo ""
        echo "Starting Jetbot + Servo Integration..."
        echo "(Dynamixel IDs 1 & 2)"
        echo ""
        echo "Make sure motors are connected:"
        echo "  - Motor ID 1: Left wheel"
        echo "  - Motor ID 2: Right wheel"
        echo "  - Port: /dev/ttyUSB0"
        echo ""
        read -p "Press Enter to continue or Ctrl+C to cancel..."
        cd "$ISAAC_SIM_PATH"
        ./python.sh "$REPO_PATH/examples/jetbot_servo_bridge.py"
        ;;

    q|Q)
        echo "Bye!"
        exit 0
        ;;
    *)
        echo "Invalid choice!"
        exit 1
        ;;
esac

echo ""
echo "=========================================="
echo "Done!"
echo "=========================================="