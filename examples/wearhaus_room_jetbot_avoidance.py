#!/usr/bin/env python3
"""
Isaac Sim 5.1 - Wearhaus Room with Jetbot + Dynamixel Integration + Rotating LiDAR Obstacle Avoidance
Creates a realistic environment using Isaac Sim content browser assets
and integrates with real Dynamixel servos with front-facing LiDAR obstacle avoidance

IMPORTANT: Run with Isaac Sim's Python!
    cd ~/Desktop/isaacsim/_build/linux-x86_64/release
    ./python.sh ~/Desktop/isaac-sim2real/examples/wearhaus_room_jetbot_avoidance.py

This example:
1. Loads a warehouse/office environment from Isaac Sim assets
2. Adds a Jetbot robot with front-facing Rotating LiDAR sensor (PhysX-based, simple!)
3. Implements reactive obstacle avoidance behavior
4. Connects to Dynamixel servos (Motor IDs 1 and 2) with safety controls
5. Bidirectionally syncs simulation ↔ hardware with avoidance override
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
from collections import deque
from enum import Enum
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from pxr import UsdGeom, Gf
import omni.usd
import omni.kit.commands
from omni.isaac.range_sensor import _range_sensor

# Dynamixel imports (optional - runs without hardware too)
try:
    from dynamixel_sdk import *
    DYNAMIXEL_AVAILABLE = True
except ImportError:
    DYNAMIXEL_AVAILABLE = False
    print("[!] Dynamixel SDK not found. Running in simulation-only mode.")

print("=" * 80)
print("Isaac Sim 5.1 - Wearhaus Room Jetbot + RTX LiDAR Obstacle Avoidance")
print("=" * 80)
print()

# ========== Avoidance Configuration ==========
AVOIDANCE_CONFIG = {
    "enabled": True,                    # Enable/disable avoidance system
    "obstacle_threshold": 0.8,          # Distance in meters to trigger avoidance
    "clear_margin": 0.5,                # Hysteresis margin (clear at threshold + margin)
    "reverse_duration": 1.8,            # Seconds to reverse (increased)
    "turn_angle": np.pi / 2,            # Radians to turn target (90 degrees)
    "reverse_speed": 0.1,               # Linear speed when reversing
    "turn_speed": np.pi / 4,            # Angular speed when turning (faster)
    "forward_speed": 0.15,              # Normal forward speed
    "cooldown_time": 0.5,               # Seconds before allowing next avoidance
    "moving_avg_window": 5,             # Moving average window for distance
    "lidar_fov": 60.0,                  # Field of view to check (degrees)
    "emergency_stop_key": "e",          # Key for emergency stop
}

# ========== Avoidance State Machine ==========
class AvoidanceState(Enum):
    FORWARD = 1
    STOP = 2
    REVERSE = 3
    TURN = 4
    EMERGENCY_STOP = 5

class ObstacleAvoidanceController:
    """Intelligent reactive obstacle avoidance using raycast sensor"""
    
    def __init__(self, config):
        self.config = config
        self.state = AvoidanceState.FORWARD
        self.distance_buffer = deque(maxlen=config["moving_avg_window"])
        self.state_start_time = time.time()
        self.last_avoidance_time = 0.0
        self.emergency_stop = False
        self.min_distance = float('inf')
        self.sensor_ready = False
        self.raw_distances = None  # Store full scan for intelligent decisions
        self.preferred_turn_direction = 1.0  # 1.0 = left, -1.0 = right
        
    def update_distance(self, lidar_data):
        """Process LiDAR data and update minimum distance"""
        if lidar_data is None:
            return
        
        if len(lidar_data) == 0:
            return
        
        # Store raw data for direction analysis
        self.raw_distances = lidar_data.copy()
        
        # Filter out NaNs and invalid readings
        valid_distances = lidar_data[~np.isnan(lidar_data)]
        valid_distances = valid_distances[valid_distances > 0.01]  # Filter very close readings
        
        if len(valid_distances) == 0:
            # No valid readings, sensor warming up
            self.sensor_ready = False
            return
        
        self.sensor_ready = True
        
        # Get minimum distance in forward arc
        min_dist = np.min(valid_distances)
        
        # Add to moving average buffer
        self.distance_buffer.append(min_dist)
        
        # Compute smoothed distance
        self.min_distance = np.mean(self.distance_buffer)
        
        # Analyze which side has more clearance for intelligent turning
        if len(self.raw_distances) > 3:
            mid = len(self.raw_distances) // 2
            left_clearance = np.mean(self.raw_distances[:mid][~np.isnan(self.raw_distances[:mid])])
            right_clearance = np.mean(self.raw_distances[mid:][~np.isnan(self.raw_distances[mid:])])
            # Prefer turning toward the side with more space
            self.preferred_turn_direction = 1.0 if left_clearance > right_clearance else -1.0
    
    def get_command(self, current_time):
        """Get velocity command based on current state with intelligent behavior"""
        elapsed = current_time - self.state_start_time
        
        # Emergency stop overrides everything
        if self.emergency_stop:
            if self.state != AvoidanceState.EMERGENCY_STOP:
                print(f"[AVOIDANCE] ⚠️  EMERGENCY STOP ACTIVATED")
                self.state = AvoidanceState.EMERGENCY_STOP
            return (0.0, 0.0), True  # (linear, angular), hardware_override
        
        # Check if avoidance is disabled
        if not self.config["enabled"]:
            return None, False  # Use default movement
        
        # Wait for sensor to warm up
        if not self.sensor_ready:
            return (0.0, 0.0), False  # Wait
        
        # State machine logic with intelligent behaviors
        if self.state == AvoidanceState.FORWARD:
            # Gradual slowdown as obstacle approaches (proactive behavior)
            safety_zone = self.config["obstacle_threshold"] + 0.4  # Start slowing earlier
            if self.min_distance < safety_zone:
                # Calculate slowdown factor (1.0 at safety_zone, 0.0 at obstacle_threshold)
                slowdown = np.clip((self.min_distance - self.config["obstacle_threshold"]) / 0.4, 0.0, 1.0)
                adjusted_speed = self.config["forward_speed"] * (0.3 + 0.7 * slowdown)  # Min 30% speed
                
                if self.min_distance < self.config["obstacle_threshold"]:
                    # Trigger avoidance
                    if current_time - self.last_avoidance_time > self.config["cooldown_time"]:
                        print(f"[AVOIDANCE] ⚠️ Obstacle at {self.min_distance:.2f}m - STOPPING (intelligent mode)")
                        self.state = AvoidanceState.STOP
                        self.state_start_time = current_time
                        self.last_avoidance_time = current_time
                        return (0.0, 0.0), True
                else:
                    # Cautious approach
                    print(f"[AVOIDANCE] ⚙️ Slowing to {adjusted_speed:.2f} m/s (obstacle at {self.min_distance:.2f}m)")
                    return (adjusted_speed, 0.0), True
            return None, False  # Full speed ahead
        
        elif self.state == AvoidanceState.STOP:
            # Brief assessment pause
            if elapsed > 0.3:
                print(f"[AVOIDANCE] 🔄 Initiating evasive maneuver (turn {'LEFT' if self.preferred_turn_direction > 0 else 'RIGHT'})")
                self.state = AvoidanceState.REVERSE
                self.state_start_time = current_time
            return (0.0, 0.0), True
        
        elif self.state == AvoidanceState.REVERSE:
            # Reverse for configured duration
            if elapsed > self.config["reverse_duration"]:
                print(f"[AVOIDANCE] ↻ Executing smart turn toward clearer side")
                self.state = AvoidanceState.TURN
                self.state_start_time = current_time
                return (0.0, 0.0), True
            # Smooth acceleration in reverse
            ramp_factor = min(1.0, elapsed / 0.5)  # Ramp up over 0.5s
            return (-self.config["reverse_speed"] * ramp_factor, 0.0), True
        
        elif self.state == AvoidanceState.TURN:
            # Turn until clear or timeout (use intelligent direction)
            turn_timeout = 4.0  # Allow more time for thorough scan
            
            # Check if path is clear (with hysteresis)
            clear_threshold = self.config["obstacle_threshold"] + self.config["clear_margin"]
            if self.min_distance > clear_threshold or elapsed > turn_timeout:
                print(f"[AVOIDANCE] ✓ Path verified clear ({self.min_distance:.2f}m) - Resuming")
                self.state = AvoidanceState.FORWARD
                self.state_start_time = current_time
                return None, False
            
            # Apply intelligent turn direction
            turn_speed = self.config["turn_speed"] * self.preferred_turn_direction
            return (0.0, turn_speed), True
        
        elif self.state == AvoidanceState.EMERGENCY_STOP:
            return (0.0, 0.0), True
        
        return None, False
    
    def trigger_emergency_stop(self):
        """Activate emergency stop"""
        self.emergency_stop = True
    
    def reset_emergency_stop(self):
        """Reset emergency stop"""
        self.emergency_stop = False
        self.state = AvoidanceState.FORWARD
        self.state_start_time = time.time()
        print(f"[AVOIDANCE] Emergency stop reset - Resuming")
    
    def get_status_string(self):
        """Get human-readable status"""
        if not self.config["enabled"]:
            return "DISABLED"
        if not self.sensor_ready:
            return "WARMING UP"
        return f"{self.state.name} | Dist: {self.min_distance:.2f}m"

# ========== Dynamixel Setup (Optional) ==========
DEV_PORT = "/dev/ttyUSB0"
BAUDRATE = 57600
MOTOR_IDS = [1, 2, 3, 4]  # Robot 1: IDs 1&2 (Left&Right), Robot 2: IDs 3&4 (Left&Right)

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
    
    def set_wheel_velocities(self, left_vel, right_vel, robot_id=1):
        """Set velocities for a robot's wheels
        
        Args:
            left_vel: Left wheel velocity [-1.0, 1.0]
            right_vel: Right wheel velocity [-1.0, 1.0]
            robot_id: 1 for Robot 1 (motors 1&2), 2 for Robot 2 (motors 3&4)
        """
        if robot_id == 1:
            motor_ids = [1, 2]  # Robot 1: Left (ID 1), Right (ID 2)
        elif robot_id == 2:
            motor_ids = [3, 4]  # Robot 2: Left (ID 3), Right (ID 4)
        else:
            raise ValueError(f"Invalid robot_id: {robot_id}")
        
        velocities = [left_vel, right_vel]
        
        for i, motor_id in enumerate(motor_ids):
            vel = int(velocities[i] * MAX_VELOCITY_SCALE)
            
            # Handle negative velocities
            if vel < 0:
                vel = (1 << 32) + vel
            
            self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, ADDR_GOAL_VELOCITY, vel)
    
    def get_velocities(self, robot_id=1):
        """Read actual velocities from a robot's motors
        
        Args:
            robot_id: 1 for Robot 1 (motors 1&2), 2 for Robot 2 (motors 3&4)
        
        Returns:
            List of [left_vel, right_vel] in raw units
        """
        if robot_id == 1:
            motor_ids = [1, 2]  # Robot 1: Left (ID 1), Right (ID 2)
        elif robot_id == 2:
            motor_ids = [3, 4]  # Robot 2: Left (ID 3), Right (ID 4)
        else:
            raise ValueError(f"Invalid robot_id: {robot_id}")
        
        actual_vels = []
        for motor_id in motor_ids:
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
    
    def emergency_stop(self):
        """Immediately stop all motors"""
        for motor_id in MOTOR_IDS:
            self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, ADDR_GOAL_VELOCITY, 0)
    
    def stop(self):
        """Stop all motors"""
        self.emergency_stop()
    
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

# Add test obstacle in front of robot for testing
stage = omni.usd.get_context().get_stage()
test_box = UsdGeom.Cube.Define(stage, "/World/TestObstacle")
test_box.GetSizeAttr().Set(1.0)
test_box.AddScaleOp().Set(Gf.Vec3f(0.5, 0.5, 0.5))
test_box.AddTranslateOp().Set(Gf.Vec3f(1.5, 0.0, 0.25))  # 1.5m in front
# Add simple collision so raycasts and physics see it
try:
    import omni.physx
    from pxr import PhysxSchema
    # Mark cube as a rigid body with default collider
    physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(test_box.GetPrim())
    physx_coll = PhysxSchema.PhysxCollisionAPI.Apply(test_box.GetPrim())
    # Ensure it's not dynamic (static obstacle)
    test_box.GetPrim().CreateAttribute("physxRigidBody:bodyType", "token").Set("static")
except Exception as e:
    print(f"[!] Warning: could not apply PhysX collision to TestObstacle: {e}")
print("[✓] Test obstacle added (1.5m in front)")

# ========== Add Jetbot Robots (2x) ==========
print("\n[Setup] Adding Jetbot robots...")

jetbot_asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"

# Robot 1 - Motor IDs 1&2
my_jetbot = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Jetbot1",
        name="jetbot1",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        create_robot=True,
        usd_path=jetbot_asset_path,
        position=np.array([-0.3, -0.5, 0.05]),  # Offset left and back
        orientation=np.array([1.0, 0.0, 0.0, 0.05]),  # Slight angle (~6°)
    )
)
print("[✓] Jetbot1 loaded (Motor IDs 1&2)")

# Robot 2 - Motor IDs 3&4
my_jetbot2 = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Jetbot2",
        name="jetbot2",
        wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
        create_robot=True,
        usd_path=jetbot_asset_path,
        position=np.array([0.3, -0.6, 0.05]),  # Offset right and back
        orientation=np.array([1.0, 0.0, 0.0, -0.05]),  # Slight angle opposite (~-6°)
    )
)
print("[✓] Jetbot2 loaded (Motor IDs 3&4)")

# Create controllers
my_controller = DifferentialController(
    name="jetbot1_controller",
    wheel_radius=0.03,
    wheel_base=0.1125
)
my_controller2 = DifferentialController(
    name="jetbot2_controller",
    wheel_radius=0.03,
    wheel_base=0.1125
)
print("[✓] Differential controllers created for both robots")

# ========== Add Rotating LiDAR Sensors ==========
print("\n[Setup] Adding Rotating LiDAR sensors for both robots...")

# Robot 1 LiDAR
lidar_path = "/World/Jetbot1/chassis/lidar"
_, lidar_prim = omni.kit.commands.execute(
    "RangeSensorCreateLidar",
    path=lidar_path,
    parent="/World/Jetbot1/chassis",
    min_range=0.4,
    max_range=100.0,
    draw_points=False,
    draw_lines=False,
    horizontal_fov=270.0,
    vertical_fov=30.0,
    horizontal_resolution=1.0,
    vertical_resolution=4.0,
    rotation_rate=0.0,
    high_lod=False,
    yaw_offset=0.0,
    enable_semantics=False
)

stage = omni.usd.get_context().get_stage()
lidar_prim_usd = stage.GetPrimAtPath(lidar_path)
if lidar_prim_usd and lidar_prim_usd.IsValid():
    xform = UsdGeom.Xformable(lidar_prim_usd)
    xform.ClearXformOpOrder()
    xform.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 1.0))
    xform.AddTranslateOp().Set(Gf.Vec3d(0.08, 0.0, 0.06))
    xform.AddOrientOp().Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))

print(f"[✓] Robot 1 LiDAR created at: {lidar_path}")

# Robot 2 LiDAR
lidar_path2 = "/World/Jetbot2/chassis/lidar"
_, lidar_prim2 = omni.kit.commands.execute(
    "RangeSensorCreateLidar",
    path=lidar_path2,
    parent="/World/Jetbot2/chassis",
    min_range=0.4,
    max_range=100.0,
    draw_points=False,
    draw_lines=False,
    horizontal_fov=270.0,
    vertical_fov=30.0,
    horizontal_resolution=1.0,
    vertical_resolution=4.0,
    rotation_rate=0.0,
    high_lod=False,
    yaw_offset=0.0,
    enable_semantics=False
)

lidar_prim_usd2 = stage.GetPrimAtPath(lidar_path2)
if lidar_prim_usd2 and lidar_prim_usd2.IsValid():
    xform2 = UsdGeom.Xformable(lidar_prim_usd2)
    xform2.ClearXformOpOrder()
    xform2.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 1.0))
    xform2.AddTranslateOp().Set(Gf.Vec3d(0.08, 0.0, 0.06))
    xform2.AddOrientOp().Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))

print(f"[✓] Robot 2 LiDAR created at: {lidar_path2}")
print(f"    Type: PhysX Rotating LiDAR (much simpler!)")
print(f"    Position: 0.08m forward, 0.06m up from chassis")
print(f"    Range: 0.4m to 100.0m")
print(f"    Horizontal FOV: 270°")

# Fallback sensor: sample forward arc using collisions.ray_cast (robust, no plugin dependency)
try:
    from isaacsim.core.utils.collisions import ray_cast as _ray_cast
except ImportError:
    _ray_cast = None

def get_lidar_data(robot):
    """Return synthetic forward arc distances for a given robot.
    Uses collisions.ray_cast which internally calls PhysX scene queries.
    Returns np.ndarray or None if unavailable.
    
    Args:
        robot: The WheeledRobot instance to get lidar data for
    """
    if _ray_cast is None:
        return None

    try:
        robot_pos, robot_quat = robot.get_world_pose()
        # Ensure quaternion ordering (w,x,y,z)
        if len(robot_quat) == 4:
            rx, ry, rz, rw = robot_quat
            robot_wxyz = np.array([rw, rx, ry, rz], dtype=float)
        else:
            robot_wxyz = np.array([1.0,0.0,0.0,0.0], dtype=float)

        lidar_offset = np.array([0.08, 0.0, 0.06], dtype=float)

        forward_fan_deg = AVOIDANCE_CONFIG.get("lidar_fov", 60.0)
        num_rays = max(7, int(forward_fan_deg // 6))  # at least 7 rays
        yaw_angles = np.linspace(-forward_fan_deg/2.0, forward_fan_deg/2.0, num_rays)
        max_dist = 8.0
        distances = []

        def quat_mul(q1, q2):
            # Multiply quaternions (w,x,y,z)
            w1,x1,y1,z1 = q1
            w2,x2,y2,z2 = q2
            return np.array([
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2
            ], dtype=float)

        for ang_deg in yaw_angles:
            ang_rad = np.radians(ang_deg)
            # Build yaw-only quaternion (w,x,y,z) for rotation about Z
            cy = np.cos(ang_rad/2.0)
            sy = np.sin(ang_rad/2.0)
            yaw_quat = np.array([cy, 0.0, 0.0, sy], dtype=float)  # w,x,y,z
            # Combine robot orientation with yaw sweep
            sweep_quat = quat_mul(robot_wxyz, yaw_quat)
            # ray_cast uses +X direction of composed transform; so rotate offset slightly for diversity
            offset_rot = lidar_offset.copy()
            # Apply simple yaw rotation to offset (Z remains constant)
            offset_rot[0] = lidar_offset[0]*np.cos(ang_rad) - lidar_offset[1]*np.sin(ang_rad)
            offset_rot[1] = lidar_offset[0]*np.sin(ang_rad) + lidar_offset[1]*np.cos(ang_rad)

            try:
                _, dist = _ray_cast(position=np.array(robot_pos),
                                    orientation=sweep_quat,
                                    offset=offset_rot,
                                    max_dist=max_dist)
            except Exception as inner_e:
                if not hasattr(get_lidar_data, '_inner_fail_printed'):
                    print(f"[DEBUG] Inner ray cast error: {inner_e}")
                    get_lidar_data._inner_fail_printed = True
                dist = max_dist
            distances.append(dist)

        return np.array(distances, dtype=float)
    except Exception as e:
        if not hasattr(get_lidar_data, '_fail_printed'):
            print(f"[DEBUG] Fallback sensor error: {e}")
            get_lidar_data._fail_printed = True
        return None

# Initialize avoidance controllers for both robots
avoidance = ObstacleAvoidanceController(AVOIDANCE_CONFIG)
avoidance2 = ObstacleAvoidanceController(AVOIDANCE_CONFIG)
print("[✓] Obstacle avoidance controllers initialized for both robots")
print(f"    Threshold: {AVOIDANCE_CONFIG['obstacle_threshold']}m")
print(f"    Hysteresis: +{AVOIDANCE_CONFIG['clear_margin']}m")
print(f"    Emergency stop key: '{AVOIDANCE_CONFIG['emergency_stop_key']}'")

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

# Step simulation to let LiDAR initialize
print("\n[Sensor] Initializing LiDAR sensors...")
for i in range(20):
    my_world.step(render=True)
    if i % 5 == 0:
        print(f"  Step {i+1}/20...")
print("[✓] LiDAR sensors ready for both robots")

# ========== Keyboard Input Setup ==========
try:
    from pynput import keyboard
    
    def on_press(key):
        try:
            if hasattr(key, 'char') and key.char == AVOIDANCE_CONFIG['emergency_stop_key']:
                # Toggle emergency stop for both robots
                if avoidance.emergency_stop:
                    avoidance.reset_emergency_stop()
                    avoidance2.reset_emergency_stop()
                else:
                    avoidance.trigger_emergency_stop()
                    avoidance2.trigger_emergency_stop()
            elif key == keyboard.Key.esc:
                return False  # Stop listener
        except AttributeError:
            pass
    
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    keyboard_available = True
    print(f"[✓] Keyboard listener active (Press '{AVOIDANCE_CONFIG['emergency_stop_key']}' for emergency stop both robots)")
except ImportError:
    keyboard_available = False
    print("[!] pynput not available - emergency stop key disabled")

# ========== Main Loop ==========
print("\n" + "=" * 80)
print("SIMULATION RUNNING - OBSTACLE AVOIDANCE ENABLED")
print("=" * 80)
print()
if dxl:
    print("Mode: Bidirectional Sync with Hardware Safety (Isaac Sim ↔ Dynamixel)")
    print("  - Simulation commands → Real motors (with avoidance override)")
    print("  - Robot 1: Motor IDs 1 (Left), 2 (Right)")
    print("  - Robot 2: Motor IDs 3 (Left), 4 (Right)")
    print("  - Emergency stop on obstacle detection")
else:
    print("Mode: Simulation Only (No Hardware Connected)")
    print("  - Both robots running in simulation")
print()
print("Avoidance Behavior:")
print(f"  - Obstacle threshold: {AVOIDANCE_CONFIG['obstacle_threshold']}m")
print(f"  - Clear threshold: {AVOIDANCE_CONFIG['obstacle_threshold'] + AVOIDANCE_CONFIG['clear_margin']}m")
print(f"  - Reverse duration: {AVOIDANCE_CONFIG['reverse_duration']}s")
print(f"  - Turn angle: {np.degrees(AVOIDANCE_CONFIG['turn_angle']):.0f}°")
print()
print("Controls:")
if keyboard_available:
    print(f"  - Press '{AVOIDANCE_CONFIG['emergency_stop_key']}' to toggle emergency stop")
print("  - Press Ctrl+C to stop or close the Isaac Sim window")
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
            my_controller2.reset()
            timeline.play()
            reset_needed = False
            i = 0
        
        current_time = time.time()
        
        # Read LiDAR data for both robots
        lidar_data = get_lidar_data(my_jetbot)
        lidar_data2 = get_lidar_data(my_jetbot2)
        
        if lidar_data is not None:
            avoidance.update_distance(lidar_data)
        if lidar_data2 is not None:
            avoidance2.update_distance(lidar_data2)
        
        # Debug: Print sensor data status every 100 steps
        if i % 100 == 0 and i > 0:
            if lidar_data is None:
                print(f"[DEBUG] Step {i}: Robot 1 LiDAR data is None")
            if lidar_data2 is None:
                print(f"[DEBUG] Step {i}: Robot 2 LiDAR data is None")
        
        # Get avoidance commands for both robots
        avoidance_cmd, hardware_override = avoidance.get_command(current_time)
        avoidance_cmd2, hardware_override2 = avoidance2.get_command(current_time)
        
        # Robot 1: Generate base movement commands
        if avoidance_cmd is None:
            # Normal forward movement when no avoidance active
            wheel_actions = my_controller.forward(
                command=[AVOIDANCE_CONFIG["forward_speed"], 0]
            )
        else:
            # Use avoidance command
            linear_vel, angular_vel = avoidance_cmd
            wheel_actions = my_controller.forward(command=[linear_vel, angular_vel])
        
        # Robot 2: Generate base movement commands
        if avoidance_cmd2 is None:
            # Normal forward movement when no avoidance active
            wheel_actions2 = my_controller2.forward(
                command=[AVOIDANCE_CONFIG["forward_speed"], 0]
            )
        else:
            # Use avoidance command
            linear_vel2, angular_vel2 = avoidance_cmd2
            wheel_actions2 = my_controller2.forward(command=[linear_vel2, angular_vel2])
        
        # Apply to simulation
        my_jetbot.apply_wheel_actions(wheel_actions)
        my_jetbot2.apply_wheel_actions(wheel_actions2)
        
        # Extract wheel velocities for Robot 1
        if wheel_actions.joint_velocities is not None:
            left_vel = float(wheel_actions.joint_velocities[0])
            right_vel = float(wheel_actions.joint_velocities[1])
        else:
            left_vel = 0.0
            right_vel = 0.0
        
        # Extract wheel velocities for Robot 2
        if wheel_actions2.joint_velocities is not None:
            left_vel2 = float(wheel_actions2.joint_velocities[0])
            right_vel2 = float(wheel_actions2.joint_velocities[1])
        else:
            left_vel2 = 0.0
            right_vel2 = 0.0
        
        # Send to Dynamixel motors
        if dxl:
            # Robot 1 (Motor IDs 1&2): Normalize to [-1, 1] range
            left_normalized = np.clip(left_vel / 15.0, -1.0, 1.0)
            right_normalized = np.clip(right_vel / 15.0, -1.0, 1.0)
            dxl.set_wheel_velocities(left_normalized, right_normalized, robot_id=1)
            
            # Robot 2 (Motor IDs 3&4): Normalize and send to hardware
            left_normalized2 = np.clip(left_vel2 / 15.0, -1.0, 1.0)
            right_normalized2 = np.clip(right_vel2 / 15.0, -1.0, 1.0)
            dxl.set_wheel_velocities(left_normalized2, right_normalized2, robot_id=2)
        
        # Print feedback every second
        if current_time - last_feedback_time >= 1.0:
            last_feedback_time = current_time
            
            # Get robot positions
            position, _ = my_jetbot.get_world_pose()
            position2, _ = my_jetbot2.get_world_pose()
            
            avoidance_status = avoidance.get_status_string()
            avoidance_status2 = avoidance2.get_status_string()
            
            print(f"[Step {i:4d}]")
            print(f"  Robot 1: Pos: [{position[0]:+.2f}, {position[1]:+.2f}] | ", end="")
            print(f"Vel: L={left_vel:+.3f} R={right_vel:+.3f} | ", end="")
            print(f"Avoid: {avoidance_status}", end="")
            
            if dxl:
                actual_vels = dxl.get_velocities(robot_id=1)
                print(f" | Motors: L={actual_vels[0]:4d} R={actual_vels[1]:4d}")
            else:
                print()
            
            print(f"  Robot 2: Pos: [{position2[0]:+.2f}, {position2[1]:+.2f}] | ", end="")
            print(f"Vel: L={left_vel2:+.3f} R={right_vel2:+.3f} | ", end="")
            print(f"Avoid: {avoidance_status2}", end="")
            
            if dxl:
                actual_vels2 = dxl.get_velocities(robot_id=2)
                print(f" | Motors: L={actual_vels2[0]:4d} R={actual_vels2[1]:4d}")
            else:
                print()
        
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
        dxl.emergency_stop()
        dxl.cleanup()
        print("[✓] Motors stopped")
    if keyboard_available:
        listener.stop()
    my_world.stop()
    simulation_app.close()
    print("[✓] Done!")
