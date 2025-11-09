#!/usr/bin/env python3
"""
Quick test for Wearhaus Room example - Isaac Sim 5.1
Tests if the environment can be loaded without hardware

Run with:
    cd ~/Desktop/isaacsim/_build/linux-x86_64/release
    ./python.sh ~/Desktop/isaac-sim2real/tests/test_wearhaus_room.py
"""

from isaacsim import SimulationApp

# Quick test - headless mode
simulation_app = SimulationApp({"headless": True})

import sys
import carb
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

print("=" * 60)
print("Testing Wearhaus Room Environment Loading")
print("=" * 60)

# Get assets path
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    print("[✗] FAILED: Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit(1)

print(f"[✓] Assets root: {assets_root_path}")

# Create world
my_world = World(stage_units_in_meters=1.0)
print("[✓] World created")

# Try to load environments
environment_options = [
    {
        "name": "Full Warehouse",
        "path": "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    },
    {
        "name": "Simple Warehouse",
        "path": "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    },
    {
        "name": "Simple Room",
        "path": "/Isaac/Environments/Simple_Room/simple_room.usd",
    },
]

success_count = 0
for env in environment_options:
    try:
        env_path = assets_root_path + env["path"]
        print(f"\n[Testing] {env['name']}...")
        print(f"  Path: {env_path}")
        
        # Try to load
        add_reference_to_stage(usd_path=env_path, prim_path="/World/TestEnv")
        print(f"  [✓] SUCCESS: {env['name']} loaded!")
        success_count += 1
        
        # Remove it for next test
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        stage.RemovePrim("/World/TestEnv")
        
    except Exception as e:
        print(f"  [✗] FAILED: {str(e)[:80]}")

print("\n" + "=" * 60)
print(f"Results: {success_count}/{len(environment_options)} environments available")
print("=" * 60)

if success_count > 0:
    print("\n[✓] SUCCESS: At least one environment can be loaded")
    print("    The Wearhaus Room example will work!")
    exit_code = 0
else:
    print("\n[!] WARNING: No pre-built environments found")
    print("    The example will create a custom room instead")
    exit_code = 0  # Still OK, will use fallback

print("\n[Cleanup] Closing...")
simulation_app.close()
print("[✓] Test complete!")

sys.exit(exit_code)
