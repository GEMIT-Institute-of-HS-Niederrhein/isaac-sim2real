# How to Run Isaac Sim Scripts

## ⚠️ IMPORTANT: Use Isaac Sim's Python Interpreter

Isaac Sim scripts **MUST** be run with Isaac Sim's Python interpreter, **NOT** your system Python or virtual environment Python.

## Why?

The `isaacsim` module is only available in Isaac Sim's Python environment. If you try to run with regular Python:

```bash
# ❌ WRONG - This will fail with "ModuleNotFoundError: No module named 'isaacsim'"
python src/isaac_dxl_bridge.py
source .venv/bin/activate && python src/isaac_dxl_bridge.py
```

## ✅ Correct Way to Run

### Option 1: Use the Helper Script (Recommended)

```bash
cd ~/Desktop/isaac-sim2real/scripts
./run_isaac_dxl.sh
```

Then select option 5 to run the full bridge.

### Option 2: Run Directly with Isaac Sim Python

```bash
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh ~/Desktop/isaac-sim2real/src/isaac_dxl_bridge.py
```

### Option 3: Create an Alias (Convenience)

Add this to your `~/.bashrc`:

```bash
alias isaac-python='~/Desktop/isaacsim/_build/linux-x86_64/release/python.sh'
```

Then reload:
```bash
source ~/.bashrc
```

Now you can run:
```bash
isaac-python ~/Desktop/isaac-sim2real/src/isaac_dxl_bridge.py
```

## Script Types

### Scripts that use Isaac Sim (use Isaac Sim Python)
- `src/isaac_dxl_bridge.py` - Main bridge with Isaac Sim GUI
- `tests/test_isaac_only.py` - Isaac Sim only test

**Run with:** `~/Desktop/isaacsim/_build/linux-x86_64/release/python.sh <script>`

### Scripts that DON'T use Isaac Sim (use virtual environment)
- `src/simple_gui_test.py` - Hardware-only GUI test
- `tools/hardware/dxl_*.py` - Hardware diagnostic tools
- `scripts/verify_setup.py` - Can use either

**Run with:** `source .venv/bin/activate && python <script>`

## Quick Reference

| Task | Command |
|------|---------|
| Run full bridge | `cd ~/Desktop/isaacsim/_build/linux-x86_64/release && ./python.sh ~/Desktop/isaac-sim2real/src/isaac_dxl_bridge.py` |
| Test Isaac Sim only | `cd ~/Desktop/isaacsim/_build/linux-x86_64/release && ./python.sh ~/Desktop/isaac-sim2real/tests/test_isaac_only.py` |
| Test hardware only | `cd ~/Desktop/isaac-sim2real && source .venv/bin/activate && python src/simple_gui_test.py` |
| Hardware diagnostics | `cd ~/Desktop/isaac-sim2real && source .venv/bin/activate && python tools/hardware/dxl_idscan.py` |

## Troubleshooting

### "ModuleNotFoundError: No module named 'isaacsim'"
- **Cause:** Using wrong Python interpreter
- **Fix:** Use Isaac Sim's `python.sh` instead of system Python

### "ModuleNotFoundError: No module named 'dynamixel_sdk'"
- **Cause:** Dependencies not installed in Isaac Sim's Python
- **Fix:** 
  ```bash
  cd ~/Desktop/isaacsim/_build/linux-x86_64/release
  ./python.sh -m pip install dynamixel-sdk pynput
  ```

### Permission denied on python.sh
- **Fix:** `chmod +x ~/Desktop/isaacsim/_build/linux-x86_64/release/python.sh`

### Permission denied on /dev/ttyUSB0
- **Fix:** 
  ```bash
  sudo usermod -aG dialout $USER
  newgrp dialout
  ```

## Installation Check

Verify Isaac Sim dependencies:
```bash
cd ~/Desktop/isaacsim/_build/linux-x86_64/release
./python.sh -m pip list | grep -E "(dynamixel|pynput)"
```

Should show:
```
dynamixel-sdk    3.8.4
pynput          1.8.1
```

Verify system dependencies (for hardware-only scripts):
```bash
cd ~/Desktop/isaac-sim2real
source .venv/bin/activate
pip list | grep -E "(dynamixel|pynput)"
```

Should show the same packages.

---

**Remember:** Isaac Sim scripts = Isaac Sim Python | Hardware-only scripts = Virtual environment Python
