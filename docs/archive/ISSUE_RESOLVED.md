# Issue Resolution: ModuleNotFoundError for 'isaacsim'

## Problem
When running `isaac_dxl_bridge.py` with regular Python or the virtual environment Python:
```
ModuleNotFoundError: No module named 'isaacsim'
```

## Root Cause
The `isaacsim` module is only available in Isaac Sim's Python environment, not in system Python or virtual environments.

## Solution

### ✅ Correct Way to Run Isaac Sim Scripts

**Option 1: Use the Helper Script (Easiest)**
```bash
cd ~/Desktop/isaac-sim2real/scripts
./run_isaac_dxl.sh
# Select option 5: Run full bridge
```

**Option 2: Direct Command**
```bash
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/isaac-sim2real/src/isaac_dxl_bridge.py
```

**Option 3: Create an Alias** (add to `~/.bashrc`)
```bash
alias isaac-python='~/Desktop/isaacsim/_build/linux-x86_64/release/python.sh'
```

Then:
```bash
source ~/.bashrc
isaac-python ~/Desktop/isaac-sim2real/src/isaac_dxl_bridge.py
```

### ❌ Incorrect Ways (Will Fail)
```bash
# These will all fail with ModuleNotFoundError
python src/isaac_dxl_bridge.py
python3 src/isaac_dxl_bridge.py
source .venv/bin/activate && python src/isaac_dxl_bridge.py
```

## Which Python to Use?

| Script Type | Python Interpreter | Example |
|-------------|-------------------|---------|
| **Isaac Sim scripts** (imports `isaacsim`) | Isaac Sim Python | `isaac_dxl_bridge.py`, `test_isaac_only.py` |
| **Hardware-only scripts** | Virtual env Python | `simple_gui_test.py`, `dxl_*.py` tools |
| **General utilities** | Either works | `verify_setup.py` |

### Quick Rule of Thumb:
- **If the script imports `isaacsim` or `omni.*`** → Use Isaac Sim Python
- **If the script only uses `dynamixel_sdk` and standard Python** → Use virtual env Python

## Files Updated

1. **`src/isaac_dxl_bridge.py`** - Added prominent comment about required Python interpreter
2. **`HOW_TO_RUN.md`** - Complete guide on running Isaac Sim scripts correctly
3. **`README.md`** - Updated Quick Start and Testing sections with clear warnings

## Verification

Check that dependencies are installed in Isaac Sim Python:
```bash
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh -m pip list | grep -E "(dynamixel|pynput)"
```

Should show:
```
dynamixel-sdk    3.8.4
pynput          1.8.1
```

✅ **Status**: Both packages are already installed and ready to use!

## Testing

Run a quick test to verify Isaac Sim integration:
```bash
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/isaac-sim2real/tests/test_isaac_only.py
```

## Next Steps

1. **Read HOW_TO_RUN.md** for comprehensive documentation
2. **Use the helper script** (`scripts/run_isaac_dxl.sh`) for easy launching
3. **Test hardware separately** first with `simple_gui_test.py` using virtual env
4. **Then run full bridge** with Isaac Sim Python

## Quick Command Reference

```bash
# Run full bridge (Isaac Sim + Hardware)
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/isaac-sim2real/src/isaac_dxl_bridge.py

# Test hardware only (no Isaac Sim needed)
cd ~/Desktop/isaac-sim2real
source .venv/bin/activate
python src/simple_gui_test.py

# Hardware diagnostics
cd ~/Desktop/isaac-sim2real
source .venv/bin/activate
python tools/hardware/dxl_idscan.py

# Helper script (interactive menu)
cd ~/Desktop/isaac-sim2real/scripts
./run_isaac_dxl.sh
```

---

**Remember:** Isaac Sim scripts = Isaac Sim Python (`python.sh`) | Hardware scripts = Virtual env Python
