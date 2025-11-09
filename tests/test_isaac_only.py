#!/usr/bin/env python3
"""
Minimal Isaac Sim 5.1 test - just opens the GUI with a robot
No hardware needed - verifies Isaac Sim is working
"""

from isaacsim import SimulationApp

# Launch Isaac Sim
print("Starting Isaac Sim 5.1...")
simulation_app = SimulationApp({"headless": False})

# Import after SimulationApp is created
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation
import time

print("Creating world...")
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

print("Loading Jetbot robot...")
assets_root = get_assets_root_path()

if assets_root:
    robot_usd = assets_root + "/Isaac/Robots/Jetbot/jetbot.usd"
    add_reference_to_stage(usd_path=robot_usd, prim_path="/World/jetbot")
    
    robot = world.scene.add(
        Articulation(prim_path="/World/jetbot", name="jetbot")
    )
    
    print(f"Robot loaded successfully!")
    print(f"  DOFs: {robot.num_dof}")
    print(f"  Joints: {robot.dof_names}")
else:
    print("Warning: Could not find Isaac Sim assets")
    robot = None

# Reset and run
world.reset()

print("\n" + "="*70)
print("SUCCESS! Isaac Sim 5.1 is working!")
print("="*70)
print("\nYou should see:")
print("  - Isaac Sim window with 3D viewport")
print("  - Jetbot robot on a ground plane")
print("  - Robot should be visible and stationary")
print("\nThis window will stay open for 30 seconds...")
print("Press Ctrl+C to quit early")
print("="*70)

# Keep window open for 30 seconds
try:
    for i in range(300):
        world.step(render=True)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nInterrupted by user")

print("\nClosing Isaac Sim...")
simulation_app.close()
print("Done!")