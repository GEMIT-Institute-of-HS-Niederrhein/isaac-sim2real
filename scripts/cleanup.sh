#!/bin/bash
# Cleanup script for isaac-sim2real project
# Run this script to clean build artifacts, Python cache, and optionally rebuild

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo -e "${GREEN}=== Isaac Sim2Real Cleanup Script ===${NC}"
echo "Project root: $PROJECT_ROOT"
echo

# Function to ask yes/no question
ask_yes_no() {
    local prompt="$1"
    local default="${2:-n}"
    
    if [ "$default" = "y" ]; then
        prompt="$prompt [Y/n]: "
    else
        prompt="$prompt [y/N]: "
    fi
    
    read -p "$prompt" response
    response=${response:-$default}
    
    if [[ "$response" =~ ^[Yy]$ ]]; then
        return 0
    else
        return 1
    fi
}

# Clean ROS workspace
echo -e "${YELLOW}1. Cleaning ROS workspace...${NC}"
cd "$PROJECT_ROOT/isaac_ros_ws"
if [ -d "build" ] || [ -d "install" ] || [ -d "log" ]; then
    rm -rf build install log
    echo -e "${GREEN}✓ Removed build/, install/, and log/ directories${NC}"
else
    echo -e "${GREEN}✓ ROS workspace already clean${NC}"
fi
echo

# Clean Python cache
echo -e "${YELLOW}2. Cleaning Python cache files...${NC}"
cd "$PROJECT_ROOT"
PYCACHE_COUNT=$(find . -type d -name "__pycache__" -not -path "./.venv/*" -not -path "./.git/*" 2>/dev/null | wc -l)
PYC_COUNT=$(find . -type f -name "*.pyc" -not -path "./.venv/*" -not -path "./.git/*" 2>/dev/null | wc -l)
PYTEST_COUNT=$(find . -type d -name ".pytest_cache" -not -path "./.venv/*" -not -path "./.git/*" 2>/dev/null | wc -l)

if [ "$PYCACHE_COUNT" -gt 0 ]; then
    find . -type d -name "__pycache__" -not -path "./.venv/*" -not -path "./.git/*" -exec rm -rf {} + 2>/dev/null || true
    echo -e "${GREEN}✓ Removed $PYCACHE_COUNT __pycache__ directories${NC}"
fi

if [ "$PYC_COUNT" -gt 0 ]; then
    find . -type f -name "*.pyc" -not -path "./.venv/*" -not -path "./.git/*" -delete 2>/dev/null || true
    echo -e "${GREEN}✓ Removed $PYC_COUNT .pyc files${NC}"
fi

if [ "$PYTEST_COUNT" -gt 0 ]; then
    find . -type d -name ".pytest_cache" -not -path "./.venv/*" -not -path "./.git/*" -exec rm -rf {} + 2>/dev/null || true
    echo -e "${GREEN}✓ Removed $PYTEST_COUNT .pytest_cache directories${NC}"
fi

if [ "$PYCACHE_COUNT" -eq 0 ] && [ "$PYC_COUNT" -eq 0 ] && [ "$PYTEST_COUNT" -eq 0 ]; then
    echo -e "${GREEN}✓ Python cache already clean${NC}"
fi
echo

# Optional: Clean and rebuild Python virtual environment
if ask_yes_no "3. Do you want to recreate the Python virtual environment?" "n"; then
    echo -e "${YELLOW}Recreating Python virtual environment...${NC}"
    cd "$PROJECT_ROOT"
    rm -rf .venv
    python3 -m venv .venv
    source .venv/bin/activate
    pip install --upgrade pip -q
    pip install -r requirements.txt -q
    echo -e "${GREEN}✓ Python virtual environment recreated${NC}"
    echo -e "${GREEN}✓ Dependencies installed from requirements.txt${NC}"
    echo
else
    echo -e "${GREEN}✓ Skipping virtual environment recreation${NC}"
    echo
fi

# Optional: Rebuild ROS workspace
if ask_yes_no "4. Do you want to rebuild the ROS workspace?" "n"; then
    echo -e "${YELLOW}Rebuilding ROS workspace...${NC}"
    cd "$PROJECT_ROOT/isaac_ros_ws"
    
    # Check if ROS is sourced
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${RED}Warning: ROS is not sourced. Please source your ROS installation first.${NC}"
        echo "Example: source /opt/ros/humble/setup.bash"
        exit 1
    fi
    
    if ask_yes_no "   Use symlink install? (faster for development)" "y"; then
        colcon build --symlink-install
    else
        colcon build
    fi
    
    echo -e "${GREEN}✓ ROS workspace rebuilt${NC}"
    echo -e "${YELLOW}Remember to source the workspace: source install/setup.bash${NC}"
    echo
else
    echo -e "${GREEN}✓ Skipping ROS workspace rebuild${NC}"
    echo
fi

# Summary
echo -e "${GREEN}=== Cleanup Complete ===${NC}"
echo
echo "Next steps:"
echo "  1. Activate Python environment: source .venv/bin/activate"
echo "  2. Build ROS workspace: cd isaac_ros_ws && colcon build --symlink-install"
echo "  3. Source ROS workspace: source isaac_ros_ws/install/setup.bash"
echo "  4. Run tests: pytest tests/"
echo

# Show disk usage
TOTAL_SIZE=$(du -sh "$PROJECT_ROOT" 2>/dev/null | cut -f1)
echo -e "Current project size: ${GREEN}$TOTAL_SIZE${NC}"
