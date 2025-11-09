#!/usr/bin/env python3
"""
Test script for Rotating LiDAR obstacle avoidance
Runs in headless mode with a box obstacle to verify state machine transitions

IMPORTANT: Run with Isaac Sim's Python!
    cd ~/Desktop/isaacsim/_build/linux-x86_64/release
    ./python.sh ~/Desktop/isaac-sim2real/tests/test_lidar_avoidance.py
"""

from isaacsim import SimulationApp

# Configuration - headless mode
CONFIG = {
    "headless": True,
    "width": 1280,
    "height": 720,
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
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from pxr import UsdGeom, Gf
import omni.usd
import omni.kit.commands
from omni.isaac.range_sensor import _range_sensor

print("=" * 80)
print("TEST: Rotating LiDAR Obstacle Avoidance - Headless Mode")
print("=" * 80)
print()

# ========== Avoidance Configuration ==========
AVOIDANCE_CONFIG = {
    "enabled": True,
    "obstacle_threshold": 0.8,
    "clear_margin": 0.3,
    "reverse_duration": 1.0,
    "turn_angle": np.pi / 3,
    "reverse_speed": 0.1,
    "turn_speed": np.pi / 6,
    "forward_speed": 0.15,
    "cooldown_time": 0.5,
    "moving_avg_window": 5,
    "lidar_fov": 60.0,
}

# ========== Avoidance State Machine ==========
class AvoidanceState(Enum):
    FORWARD = 1
    STOP = 2
    REVERSE = 3
    TURN = 4
    EMERGENCY_STOP = 5

class ObstacleAvoidanceController:
    """Reactive obstacle avoidance using RTX LiDAR"""
    
    def __init__(self, config):
        self.config = config
        self.state = AvoidanceState.FORWARD
        self.distance_buffer = deque(maxlen=config["moving_avg_window"])
        self.state_start_time = time.time()
        self.last_avoidance_time = 0.0
        self.emergency_stop = False
        self.min_distance = float('inf')
        self.sensor_ready = False
        self.state_history = []
        
    def update_distance(self, lidar_data):
        """Process LiDAR data and update minimum distance"""
        if lidar_data is None or len(lidar_data) == 0:
            return
        
        # Filter out NaNs and invalid readings
        valid_distances = lidar_data[~np.isnan(lidar_data)]
        valid_distances = valid_distances[valid_distances > 0.01]
        
        if len(valid_distances) == 0:
            self.sensor_ready = False
            return
        
        self.sensor_ready = True
        
        # Get minimum distance in forward arc
        min_dist = np.min(valid_distances)
        
        # Add to moving average buffer
        self.distance_buffer.append(min_dist)
        
        # Compute smoothed distance
        self.min_distance = np.mean(self.distance_buffer)
    
    def get_command(self, current_time):
        """Get velocity command based on current state"""
        elapsed = current_time - self.state_start_time
        
        # Track state transitions
        old_state = self.state
        
        # Emergency stop overrides everything
        if self.emergency_stop:
            if self.state != AvoidanceState.EMERGENCY_STOP:
                self.state = AvoidanceState.EMERGENCY_STOP
            return (0.0, 0.0), True
        
        if not self.config["enabled"]:
            return None, False
        
        if not self.sensor_ready:
            return (0.0, 0.0), False
        
        # State machine logic
        if self.state == AvoidanceState.FORWARD:
            if self.min_distance < self.config["obstacle_threshold"]:
                if current_time - self.last_avoidance_time > self.config["cooldown_time"]:
                    self.state = AvoidanceState.STOP
                    self.state_start_time = current_time
                    self.last_avoidance_time = current_time
                    return (0.0, 0.0), True
            return None, False
        
        elif self.state == AvoidanceState.STOP:
            if elapsed > 0.2:
                self.state = AvoidanceState.REVERSE
                self.state_start_time = current_time
            return (0.0, 0.0), True
        
        elif self.state == AvoidanceState.REVERSE:
            if elapsed > self.config["reverse_duration"]:
                self.state = AvoidanceState.TURN
                self.state_start_time = current_time
                return (0.0, 0.0), True
            return (-self.config["reverse_speed"], 0.0), True
        
        elif self.state == AvoidanceState.TURN:
            turn_timeout = 3.0
            clear_threshold = self.config["obstacle_threshold"] + self.config["clear_margin"]
            if self.min_distance > clear_threshold or elapsed > turn_timeout:
                self.state = AvoidanceState.FORWARD
                self.state_start_time = current_time
                return None, False
            return (0.0, self.config["turn_speed"]), True
        
        elif self.state == AvoidanceState.EMERGENCY_STOP:
            return (0.0, 0.0), True
        
        # Log state transition
        if old_state != self.state:
            self.state_history.append({
                'time': current_time,
                'from': old_state.name,
                'to': self.state.name,
                'distance': self.min_distance
            })
        
        return None, False

# ========== Setup Isaac Sim ==========
print("[Setup] Initializing Isaac Sim...")

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit(1)

# Create world
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
print("[✓] World created")

# Add test obstacle
stage = omni.usd.get_context().get_stage()
test_box = UsdGeom.Cube.Define(stage, "/World/TestObstacle")
test_box.GetSizeAttr().Set(1.0)
test_box.AddScaleOp().Set(Gf.Vec3f(0.5, 0.5, 0.5))
test_box.AddTranslateOp().Set(Gf.Vec3f(1.2, 0.0, 0.25))  # 1.2m in front
print("[✓] Test obstacle placed at 1.2m")

# Add Jetbot
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
print("[✓] Controller created")

# Add Rotating LiDAR (PhysX-based)
lidar_path = "/World/Jetbot/chassis/lidar"
result, lidar_prim = omni.kit.commands.execute(
    "RangeSensorCreateLidar",
    path=lidar_path,
    parent="/World/Jetbot/chassis",
    min_range=0.4,
    max_range=100.0,
    draw_points=False,
    draw_lines=False,
    horizontal_fov=270.0,
    vertical_fov=1.0,
    horizontal_resolution=1.0,
    vertical_resolution=1.0,
    rotation_rate=0.0,
    high_lod=False,
    yaw_offset=0.0,
    enable_semantics=False
)

if not result:
    print("[!] Failed to create Rotating LiDAR")
    simulation_app.close()
    sys.exit(1)

# Apply local transform (position and orientation)
xform = UsdGeom.Xformable(lidar_prim)
xform.ClearXformOpOrder()
xform.AddTranslateOp().Set(Gf.Vec3d(0.08, 0.0, 0.06))  # Front of chassis
xform.AddOrientOp().Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))  # Identity (w, i, j, k)

print(f"[✓] Rotating LiDAR created (PhysX-based, simple API)")

# LiDAR data helper
def get_lidar_data():
    """Read LiDAR depth data using PhysX interface"""
    try:
        lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
        depth_data = lidar_interface.get_linear_depth_data(lidar_path)
        
        if depth_data is None or len(depth_data) == 0:
            return None
        
        # Convert to numpy array if needed
        if not isinstance(depth_data, np.ndarray):
            depth_data = np.array(depth_data)
        
        # Extract forward arc (center 60° from 270° FOV)
        total_points = len(depth_data)
        if total_points == 0:
            return None
        
        # 270° FOV, we want ~60° forward arc
        fov_fraction = min(1.0, AVOIDANCE_CONFIG["lidar_fov"] / 270.0)
        center_start = int(total_points * (1 - fov_fraction) / 2)
        center_end = int(total_points * (1 + fov_fraction) / 2)
        
        if center_end <= center_start:
            forward_arc_data = depth_data
        else:
            forward_arc_data = depth_data[center_start:center_end]
        
        return forward_arc_data
    except Exception as e:
        return None

# Initialize avoidance
avoidance = ObstacleAvoidanceController(AVOIDANCE_CONFIG)
print("[✓] Avoidance controller initialized")

# Initialize simulation
my_world.reset()

from omni.timeline import get_timeline_interface
timeline = get_timeline_interface()
timeline.play()

# Step simulation for sensor warm-up (PhysX LiDAR auto-initializes)
print("\n[Sensor] Initializing and warming up...")
for i in range(20):
    my_world.step(render=True)
    if i % 5 == 0:
        print(f"  Step {i+1}/20...")
print("[✓] Sensor ready")

# ========== Run Test ==========
print("\n" + "=" * 80)
print("RUNNING TEST")
print("=" * 80)
print()
print("Expected behavior:")
print("  1. FORWARD - Move towards obstacle")
print("  2. STOP - Detect obstacle and stop")
print("  3. REVERSE - Back away from obstacle")
print("  4. TURN - Rotate to find clear path")
print("  5. FORWARD - Resume forward movement")
print()
print("Test duration: 30 seconds")
print("=" * 80)
print()

test_start_time = time.time()
test_duration = 30.0
i = 0

try:
    while simulation_app.is_running():
        my_world.step(render=True)
        
        if not my_world.is_playing():
            continue
        
        current_time = time.time()
        elapsed_test_time = current_time - test_start_time
        
        # Stop test after duration
        if elapsed_test_time > test_duration:
            break
        
        # Read LiDAR
        lidar_data = get_lidar_data()
        if lidar_data is not None:
            avoidance.update_distance(lidar_data)
        
        # Get command
        avoidance_cmd, hardware_override = avoidance.get_command(current_time)
        
        # Generate movement
        if avoidance_cmd is None:
            wheel_actions = my_controller.forward(
                command=[AVOIDANCE_CONFIG["forward_speed"], 0]
            )
        else:
            linear_vel, angular_vel = avoidance_cmd
            wheel_actions = my_controller.forward(command=[linear_vel, angular_vel])
        
        # Apply to simulation
        my_jetbot.apply_wheel_actions(wheel_actions)
        
        # Print status every 10 steps
        if i % 10 == 0:
            position, _ = my_jetbot.get_world_pose()
            print(f"[{elapsed_test_time:5.1f}s] State: {avoidance.state.name:15s} | "
                  f"Dist: {avoidance.min_distance:5.2f}m | "
                  f"Pos: [{position[0]:+.2f}, {position[1]:+.2f}]")
        
        i += 1
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n[Interrupted]")
except Exception as e:
    print(f"\n[Error] {e}")
    import traceback
    traceback.print_exc()
finally:
    print("\n" + "=" * 80)
    print("TEST RESULTS")
    print("=" * 80)
    print()
    
    # Print state transition history
    if len(avoidance.state_history) > 0:
        print("State Transitions:")
        for i, trans in enumerate(avoidance.state_history):
            print(f"  {i+1}. {trans['from']:15s} → {trans['to']:15s} "
                  f"(distance: {trans['distance']:.2f}m)")
        print()
        
        # Check if we got the expected sequence
        expected_states = ['FORWARD', 'STOP', 'REVERSE', 'TURN', 'FORWARD']
        actual_states = [trans['to'] for trans in avoidance.state_history]
        
        print("Expected sequence:", ' → '.join(expected_states))
        print("Actual sequence:  ", ' → '.join(actual_states))
        print()
        
        if actual_states == expected_states:
            print("✓ TEST PASSED - State machine transitions correct!")
        else:
            print("✗ TEST FAILED - State machine transitions incorrect")
            print("  Check if obstacle was detected and avoidance triggered")
    else:
        print("✗ TEST FAILED - No state transitions recorded")
        print("  Possible issues:")
        print("    - LiDAR sensor not working")
        print("    - Obstacle not detected")
        print("    - Threshold too low")
    
    print()
    print("=" * 80)
    
    my_world.stop()
    simulation_app.close()
    print("\n[✓] Test complete")
