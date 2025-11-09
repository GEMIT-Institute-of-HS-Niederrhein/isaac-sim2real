#!/usr/bin/env python3
"""
Isaac Sim 5.1 - Sample Room with Robot Base
Creates a warehouse environment with a wheeled robot (Carter or Jetbot)
This serves as a base for future Dynamixel servo integration

IMPORTANT: Run with Isaac Sim's Python!
    cd ~/Desktop/isaacsim/_build/linux-x86_64/release
    ./python.sh ~/Desktop/isaac-sim2real/examples/sample_room_robot.py

Future Integration:
    - The robot's joint velocities can be controlled from external servo data
    - Dynamixel servo feedback can be mirrored to Isaac Sim joints
    - Isaac Sim joint commands can drive physical Dynamixel servos
"""

from isaacsim import SimulationApp

# Configuration
CONFIG = {
    "headless": False,  # Set to True for headless mode
    "width": 1920,
    "height": 1080,
}

simulation_app = SimulationApp(CONFIG)

import sys
import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from pxr import UsdGeom, Gf, UsdPhysics

print("=" * 80)
print("Isaac Sim 5.1 - Sample Room with Robot")
print("=" * 80)
print()

# Get assets path
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    carb.log_error("Make sure Isaac Sim is properly installed")
    simulation_app.close()
    sys.exit(1)

print(f"[✓] Assets root: {assets_root_path}")

# Create world
my_world = World(stage_units_in_meters=1.0)
print("[✓] World created")

# Add ground plane
my_world.scene.add_default_ground_plane()
print("[✓] Ground plane added")

# ========== Option 1: Simple Warehouse Room ==========
print("\n[Setup] Creating warehouse environment...")

# Try to load warehouse environment (if available)
try:
    warehouse_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    add_reference_to_stage(usd_path=warehouse_path, prim_path="/World/Warehouse")
    print(f"[✓] Warehouse loaded: {warehouse_path}")
except Exception as e:
    print(f"[!] Warehouse not available, creating simple room instead")
    print(f"    Error: {e}")
    
    # Create a simple room with walls using UsdGeom
    print("[Setup] Building simple room with walls...")
    
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    
    # Create room group
    room_prim = stage.DefinePrim("/World/Room", "Xform")
    
    # Simple approach: Just add a larger ground plane and some box obstacles
    # The default ground plane will serve as the floor
    
    # Add some obstacles using USD directly
    from pxr import UsdGeom, Gf
    
    # Box 1
    box1 = UsdGeom.Cube.Define(stage, "/World/Room/Box1")
    box1.GetSizeAttr().Set(0.5)
    box1.AddTranslateOp().Set(Gf.Vec3f(2.0, 2.0, 0.25))
    
    # Box 2
    box2 = UsdGeom.Cube.Define(stage, "/World/Room/Box2")
    box2.GetSizeAttr().Set(0.5)
    box2.AddTranslateOp().Set(Gf.Vec3f(-2.0, -2.0, 0.25))
    
    # Box 3
    box3 = UsdGeom.Cube.Define(stage, "/World/Room/Box3")
    box3.GetSizeAttr().Set(0.5)
    box3.AddTranslateOp().Set(Gf.Vec3f(-2.0, 2.0, 0.25))
    
    print("[✓] Simple room created with obstacles")

# ========== Add Robot ==========
print("\n[Setup] Adding wheeled robot...")

# Choose robot type (change this to try different robots)
ROBOT_TYPE = "jetbot"  # Options: "jetbot", "carter", "nova_carter"

if ROBOT_TYPE == "jetbot":
    # Jetbot - Small 2-wheel differential drive robot
    jetbot_asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
    my_robot = my_world.scene.add(
        WheeledRobot(
            prim_path="/World/Robot",
            name="my_robot",
            wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
            create_robot=True,
            usd_path=jetbot_asset_path,
            position=np.array([0, 0.0, 0.05]),
        )
    )
    my_controller = DifferentialController(
        name="robot_controller",
        wheel_radius=0.03,
        wheel_base=0.1125
    )
    print(f"[✓] Jetbot robot loaded (2-wheel differential drive)")
    
elif ROBOT_TYPE == "carter":
    # Carter - Medium 4-wheel differential drive robot
    carter_asset_path = assets_root_path + "/Isaac/Robots/Carter/carter_v1.usd"
    my_robot = my_world.scene.add(
        WheeledRobot(
            prim_path="/World/Robot",
            name="my_robot",
            wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
            create_robot=True,
            usd_path=carter_asset_path,
            position=np.array([0, 0.0, 0.0]),
        )
    )
    my_controller = DifferentialController(
        name="robot_controller",
        wheel_radius=0.125,
        wheel_base=0.4132
    )
    print(f"[✓] Carter robot loaded (4-wheel differential drive)")
    
elif ROBOT_TYPE == "nova_carter":
    # Nova Carter - Advanced autonomous vehicle platform
    nova_asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
    add_reference_to_stage(usd_path=nova_asset_path, prim_path="/World/Robot")
    my_robot = Articulation(prim_paths_expr="/World/Robot", name="my_robot")
    my_controller = None  # Nova Carter has more complex control
    print(f"[✓] Nova Carter robot loaded (advanced platform)")
    
else:
    print(f"[!] Unknown robot type: {ROBOT_TYPE}")
    simulation_app.close()
    sys.exit(1)

# Set camera view
set_camera_view(
    eye=[5.0, 5.0, 3.0],
    target=[0.0, 0.0, 0.5],
    camera_prim_path="/OmniverseKit_Persp"
)
print("[✓] Camera positioned")

# Initialize world
print("\n[Setup] Initializing simulation...")
my_world.reset()
print("[✓] Simulation ready!")

# ========== Simulation Loop ==========
print("\n" + "=" * 80)
print("SIMULATION RUNNING")
print("=" * 80)
print()
print("Controls:")
print("  - Observe the robot in the environment")
print("  - Robot will perform a simple movement pattern")
print("  - Close the window to exit")
print()
print("Future Integration Points:")
print("  1. Read Dynamixel servo positions → Update Isaac Sim joints")
print("  2. Read Isaac Sim joint commands → Send to Dynamixel servos")
print("  3. Bidirectional sync for sim-to-real transfer")
print()
print("=" * 80)
print()

# Demo movement pattern
i = 0
reset_needed = False

try:
    while simulation_app.is_running():
        my_world.step(render=True)
        
        if my_world.is_stopped() and not reset_needed:
            reset_needed = True
        
        if my_world.is_playing():
            if reset_needed:
                my_world.reset()
                if my_controller:
                    my_controller.reset()
                reset_needed = False
                i = 0
            
            # Simple movement pattern demonstration
            if my_controller:  # Only for robots with differential controller
                if i >= 0 and i < 500:
                    # Move forward
                    my_robot.apply_wheel_actions(
                        my_controller.forward(command=[0.2, 0])
                    )
                elif i >= 500 and i < 700:
                    # Rotate
                    my_robot.apply_wheel_actions(
                        my_controller.forward(command=[0.0, np.pi / 12])
                    )
                elif i >= 700 and i < 1200:
                    # Move forward again
                    my_robot.apply_wheel_actions(
                        my_controller.forward(command=[0.2, 0])
                    )
                elif i >= 1200 and i < 1400:
                    # Rotate back
                    my_robot.apply_wheel_actions(
                        my_controller.forward(command=[0.0, -np.pi / 12])
                    )
                elif i >= 1400:
                    # Reset cycle
                    i = 0
            else:
                # For Nova Carter or other robots without simple controller
                if i == 0:
                    print("[Info] Robot loaded but no automatic movement pattern")
                    print("       You can manually control via API or add custom controller")
            
            i += 1
            
            # Print robot status every 100 steps
            if i % 100 == 0 and my_controller:
                position, _ = my_robot.get_world_pose()
                velocity = my_robot.get_linear_velocity()
                print(f"[Status] Step: {i:4d} | Position: {position} | Velocity: {velocity}")

except KeyboardInterrupt:
    print("\n[Interrupted] Stopping simulation...")

# Cleanup
print("\n[Cleanup] Shutting down...")
my_world.stop()
simulation_app.close()
print("[✓] Goodbye!")
