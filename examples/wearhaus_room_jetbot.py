#!/usr/bin/env python3
"""
Isaac Sim 5.1 - Wearhaus Room with Jetbot + Dynamixel Integration
Creates a realistic environment using Isaac Sim content browser assets
and integrates with real Dynamixel servos

IMPORTANT: Run with Isaac Sim's Python!
    cd ~/Desktop/isaacsim/_build/linux-x86_64/release
    ./python.sh ~/Desktop/isaac-sim2real/examples/wearhaus_room_jetbot.py

This example:
1. Loads a warehouse/office environment from Isaac Sim assets
2. Adds a Jetbot robot
3. Connects to Dynamixel servos (Motor IDs 1 and 2)
4. Bidirectionally syncs simulation ↔ hardware
"""

from isaacsim import SimulationApp

# Configuration
CONFIG = {
    "headless": False,
    "width": 1920,
    "height": 1080,
}

simulation_app = SimulationApp(CONFIG)

import sys
import carb
import numpy as np
import time
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from pxr import UsdGeom, Gf

# Dynamixel imports (optional - runs without hardware too)
try:
    from dynamixel_sdk import *
    DYNAMIXEL_AVAILABLE = True
except ImportError:
    DYNAMIXEL_AVAILABLE = False
    print("[!] Dynamixel SDK not found. Running in simulation-only mode.")

print("=" * 80)
print("Isaac Sim 5.1 - Wearhaus Room with Jetbot + Dynamixel Integration")
print("=" * 80)
print()

# ========== Dynamixel Setup (Optional) ==========
DEV_PORT = "/dev/ttyUSB0"
BAUDRATE = 57600
MOTOR_IDS = [1, 2]  # Left wheel (ID 1), Right wheel (ID 2)

# Control table addresses for XL-320 or similar
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_VELOCITY_LIMIT = 44
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_VELOCITY = 128

MODE_VELOCITY = 1
VELOCITY_LIMIT = 700
MAX_VELOCITY_SCALE = 600

class DynamixelController:
    """Controller for Dynamixel servos"""
    def __init__(self):
        if not DYNAMIXEL_AVAILABLE:
            raise Exception("Dynamixel SDK not available")
            
        self.port_handler = PortHandler(DEV_PORT)
        self.packet_handler = PacketHandler(2.0)
        
        if not self.port_handler.openPort():
            raise Exception(f"Failed to open port {DEV_PORT}")
        if not self.port_handler.setBaudRate(BAUDRATE):
            raise Exception("Failed to set baudrate")
        
        print("[Dynamixel] Initializing motors...")
        for motor_id in MOTOR_IDS:
            # Disable torque
            self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 0)
            
            # Set velocity mode
            self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, ADDR_OPERATING_MODE, MODE_VELOCITY)
            
            # Set velocity limit
            self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT)
            
            # Enable torque
            self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 1)
            
            print(f"  Motor ID {motor_id} ready")
    
    def set_wheel_velocities(self, left_vel, right_vel):
        """Set velocities for left (ID 1) and right (ID 2) wheels"""
        velocities = [left_vel, right_vel]
        
        for i, motor_id in enumerate(MOTOR_IDS):
            vel = int(velocities[i] * MAX_VELOCITY_SCALE)
            
            # Handle negative velocities
            if vel < 0:
                vel = (1 << 32) + vel
            
            self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, ADDR_GOAL_VELOCITY, vel)
    
    def get_velocities(self):
        """Read actual velocities from motors"""
        actual_vels = []
        for motor_id in MOTOR_IDS:
            vel, comm, err = self.packet_handler.read4ByteTxRx(
                self.port_handler, motor_id, ADDR_PRESENT_VELOCITY)
            
            if comm == 0:
                # Convert from unsigned to signed
                if vel > 0x7FFFFFFF:
                    vel = vel - (1 << 32)
                actual_vels.append(vel)
            else:
                actual_vels.append(0)
        
        return actual_vels
    
    def stop(self):
        for motor_id in MOTOR_IDS:
            self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, ADDR_GOAL_VELOCITY, 0)
    
    def cleanup(self):
        self.stop()
        self.port_handler.closePort()

# ========== Initialize Dynamixel (Optional) ==========
dxl = None
if DYNAMIXEL_AVAILABLE:
    try:
        dxl = DynamixelController()
        print("[✓] Dynamixel motors ready\n")
    except Exception as e:
        print(f"[!] Failed to initialize Dynamixel: {e}")
        print("    Continuing in simulation-only mode...")
        dxl = None
else:
    print("[!] Running in simulation-only mode (no hardware)\n")

# ========== Isaac Sim Setup ==========
print("[Isaac Sim] Loading environment...")

# Get assets path
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    if dxl:
        dxl.cleanup()
    simulation_app.close()
    sys.exit(1)

print(f"[✓] Assets root: {assets_root_path}")

# Create world
my_world = World(stage_units_in_meters=1.0)
print("[✓] World created")

# Add ground plane
my_world.scene.add_default_ground_plane()
print("[✓] Ground plane added")

# ========== Load Environment from Isaac Sim Content Browser ==========
print("\n[Setup] Loading Wearhaus-style environment...")

# Try multiple environment options in order of preference
environment_loaded = False
environment_options = [
    {
        "name": "Full Warehouse",
        "path": "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
        "description": "Complete warehouse with shelves and props"
    },
    {
        "name": "Simple Warehouse",
        "path": "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
        "description": "Basic warehouse structure"
    },
    {
        "name": "Small Warehouse",
        "path": "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd",
        "description": "Warehouse with forklifts"
    },
    {
        "name": "Simple Room",
        "path": "/Isaac/Environments/Simple_Room/simple_room.usd",
        "description": "Basic indoor room"
    },
]

for env in environment_options:
    try:
        env_path = assets_root_path + env["path"]
        add_reference_to_stage(usd_path=env_path, prim_path="/World/Environment")
        print(f"[✓] Environment loaded: {env['name']}")
        print(f"    {env['description']}")
        print(f"    Path: {env_path}")
        environment_loaded = True
        break
    except Exception as e:
        print(f"[!] Could not load {env['name']}: {e}")
        continue

if not environment_loaded:
    print("\n[!] Could not load any pre-built environments")
    print("[Setup] Creating custom Wearhaus-style room...")
    
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    
    # Create room group
    room_prim = stage.DefinePrim("/World/Environment", "Xform")
    
    # Add walls (simplified version)
    # Wall 1 - Back wall
    wall1 = UsdGeom.Cube.Define(stage, "/World/Environment/Wall_Back")
    wall1.GetSizeAttr().Set(1.0)
    wall1.AddScaleOp().Set(Gf.Vec3f(10.0, 0.2, 3.0))
    wall1.AddTranslateOp().Set(Gf.Vec3f(0.0, 5.0, 1.5))
    
    # Wall 2 - Front wall
    wall2 = UsdGeom.Cube.Define(stage, "/World/Environment/Wall_Front")
    wall2.GetSizeAttr().Set(1.0)
    wall2.AddScaleOp().Set(Gf.Vec3f(10.0, 0.2, 3.0))
    wall2.AddTranslateOp().Set(Gf.Vec3f(0.0, -5.0, 1.5))
    
    # Wall 3 - Left wall
    wall3 = UsdGeom.Cube.Define(stage, "/World/Environment/Wall_Left")
    wall3.GetSizeAttr().Set(1.0)
    wall3.AddScaleOp().Set(Gf.Vec3f(0.2, 10.0, 3.0))
    wall3.AddTranslateOp().Set(Gf.Vec3f(-5.0, 0.0, 1.5))
    
    # Wall 4 - Right wall
    wall4 = UsdGeom.Cube.Define(stage, "/World/Environment/Wall_Right")
    wall4.GetSizeAttr().Set(1.0)
    wall4.AddScaleOp().Set(Gf.Vec3f(0.2, 10.0, 3.0))
    wall4.AddTranslateOp().Set(Gf.Vec3f(5.0, 0.0, 1.5))
    
    # Add some "furniture" obstacles
    # Obstacle 1 - Table-like structure
    table1 = UsdGeom.Cube.Define(stage, "/World/Environment/Table1")
    table1.GetSizeAttr().Set(1.0)
    table1.AddScaleOp().Set(Gf.Vec3f(1.5, 0.8, 0.8))
    table1.AddTranslateOp().Set(Gf.Vec3f(2.5, 2.5, 0.4))
    
    # Obstacle 2 - Cabinet-like structure
    cabinet1 = UsdGeom.Cube.Define(stage, "/World/Environment/Cabinet1")
    cabinet1.GetSizeAttr().Set(1.0)
    cabinet1.AddScaleOp().Set(Gf.Vec3f(1.0, 0.5, 1.5))
    cabinet1.AddTranslateOp().Set(Gf.Vec3f(-3.0, -2.0, 0.75))
    
    # Obstacle 3 - Shelf-like structure
    shelf1 = UsdGeom.Cube.Define(stage, "/World/Environment/Shelf1")
    shelf1.GetSizeAttr().Set(1.0)
    shelf1.AddScaleOp().Set(Gf.Vec3f(2.0, 0.4, 1.2))
    shelf1.AddTranslateOp().Set(Gf.Vec3f(-2.5, 3.0, 0.6))
    
    print("[✓] Custom Wearhaus-style room created")
    print("    - 4 walls defining the space")
    print("    - 3 obstacle objects (table, cabinet, shelf)")

# ========== Add Jetbot Robot ==========
print("\n[Setup] Adding Jetbot robot...")

jetbot_asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
my_jetbot = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Jetbot",
        name="my_jetbot",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        create_robot=True,
        usd_path=jetbot_asset_path,
        position=np.array([0, 0.0, 0.05]),
    )
)
print("[✓] Jetbot loaded")

# Create controller
my_controller = DifferentialController(
    name="jetbot_controller",
    wheel_radius=0.03,
    wheel_base=0.1125
)
print("[✓] Differential controller created")

# Set camera view
set_camera_view(
    eye=[4.0, -4.0, 3.0],
    target=[0.0, 0.0, 0.5],
    camera_prim_path="/OmniverseKit_Persp"
)
print("[✓] Camera positioned")

# ========== Initialize Simulation ==========
print("\n[Setup] Initializing simulation...")
my_world.reset()
print("[✓] Simulation ready!")

# Start timeline
from omni.timeline import get_timeline_interface
timeline = get_timeline_interface()
timeline.play()
print("[✓] Timeline started")

# Give it a moment to stabilize
time.sleep(0.5)

# ========== Main Loop ==========
print("\n" + "=" * 80)
print("SIMULATION RUNNING")
print("=" * 80)
print()
if dxl:
    print("Mode: Bidirectional Sync (Isaac Sim ↔ Dynamixel)")
    print("  - Simulation commands → Real motors")
    print("  - Motor IDs: 1 (Left), 2 (Right)")
else:
    print("Mode: Simulation Only (No Hardware Connected)")
print()
print("Movement Pattern:")
print("  1. Move forward")
print("  2. Rotate right")
print("  3. Move forward")
print("  4. Rotate left")
print("  5. Repeat")
print()
print("Press Ctrl+C to stop or close the Isaac Sim window")
print("=" * 80)
print()

i = 0
reset_needed = False
last_feedback_time = time.time()
loop_count = 0

try:
    while simulation_app.is_running():
        my_world.step(render=True)
        loop_count += 1
        
        # Skip if not playing
        if not my_world.is_playing():
            continue
        
        if my_world.is_stopped() and not reset_needed:
            reset_needed = True
        
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            timeline.play()
            reset_needed = False
            i = 0
        
        # Generate movement commands (movement pattern)
        if i >= 0 and i < 1000:
            # Forward
            wheel_actions = my_controller.forward(command=[0.15, 0])
        elif i >= 1000 and i < 1300:
            # Rotate right
            wheel_actions = my_controller.forward(command=[0.0, np.pi / 12])
        elif i >= 1300 and i < 2300:
            # Forward
            wheel_actions = my_controller.forward(command=[0.15, 0])
        elif i >= 2300 and i < 2600:
            # Rotate left
            wheel_actions = my_controller.forward(command=[0.0, -np.pi / 12])
        elif i >= 2600:
            # Reset cycle
            i = 0
            wheel_actions = my_controller.forward(command=[0.0, 0])
        else:
            wheel_actions = my_controller.forward(command=[0.0, 0])
        
        # Apply to simulation
        my_jetbot.apply_wheel_actions(wheel_actions)
        
        # Extract wheel velocities
        if wheel_actions.joint_velocities is not None:
            left_vel = float(wheel_actions.joint_velocities[0])
            right_vel = float(wheel_actions.joint_velocities[1])
        else:
            left_vel = 0.0
            right_vel = 0.0
        
        # Send to Dynamixel motors
        if dxl:
            # Normalize to [-1, 1] range
            # Jetbot wheel max speed is about 15 rad/s
            left_normalized = np.clip(left_vel / 15.0, -1.0, 1.0)
            right_normalized = np.clip(right_vel / 15.0, -1.0, 1.0)
            
            dxl.set_wheel_velocities(left_normalized, right_normalized)
        
        # Print feedback every second
        current_time = time.time()
        if current_time - last_feedback_time >= 1.0:
            last_feedback_time = current_time
            
            # Get robot position
            position, _ = my_jetbot.get_world_pose()
            
            print(f"[Step {i:4d}] Pos: [{position[0]:+.2f}, {position[1]:+.2f}, {position[2]:+.2f}] | ", end="")
            print(f"Vel: L={left_vel:+.3f} R={right_vel:+.3f} rad/s", end="")
            
            if dxl:
                actual_vels = dxl.get_velocities()
                print(f" | Motors: L={actual_vels[0]:4d} R={actual_vels[1]:4d}")
            else:
                print(" | (Sim Only)")
        
        i += 1
        time.sleep(0.01)  # 100Hz

except KeyboardInterrupt:
    print("\n[Interrupted by user]")
except Exception as e:
    print(f"\n[Error] {e}")
    import traceback
    traceback.print_exc()
finally:
    print("\n[Cleanup] Stopping...")
    if dxl:
        dxl.cleanup()
        print("[✓] Motors stopped")
    my_world.stop()
    simulation_app.close()
    print("[✓] Done!")
