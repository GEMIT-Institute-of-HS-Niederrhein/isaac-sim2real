#!/usr/bin/env python3
"""
Isaac Sim Jetbot + Dynamixel Servo Integration
Bidirectional control between simulation and real motors (IDs 1 and 2)

Run with Isaac Sim Python:
    cd ~/Desktop/isaacsim/_build/linux-x86_64/release
    ./python.sh ~/Desktop/isaac-sim2real/examples/jetbot_servo_bridge.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import time
from isaacsim.core.api import World
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.storage.native import get_assets_root_path

# Dynamixel imports
from dynamixel_sdk import *

# ========== Dynamixel Setup ==========
DEV_PORT = "/dev/ttyUSB0"
BAUDRATE = 57600
MOTOR_IDS = [1, 2]  # Left wheel (ID 1), Right wheel (ID 2)

# Control table addresses
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_VELOCITY_LIMIT = 44
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_VELOCITY = 128

MODE_VELOCITY = 1
VELOCITY_LIMIT = 700
MAX_VELOCITY_SCALE = 600

class DynamixelController:
    def __init__(self):
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
        """
        Set velocities for left (ID 1) and right (ID 2) wheels
        velocities in range [-1, 1] normalized
        """
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

# ========== Main Setup ==========
print("=" * 70)
print("Jetbot + Dynamixel Servo Integration")
print("Motors: ID 1 (Left), ID 2 (Right)")
print("=" * 70)

# Initialize Dynamixel
try:
    dxl = DynamixelController()
    print("[✓] Dynamixel motors ready\n")
except Exception as e:
    print(f"[✗] Failed to initialize Dynamixel: {e}")
    print("    Make sure motors are connected to /dev/ttyUSB0")
    print("    Run without hardware? (sim only)")
    dxl = None

# Initialize Isaac Sim
print("[Isaac Sim] Loading...")
my_world = World(stage_units_in_meters=1.0)
assets_root_path = get_assets_root_path()

if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    if dxl:
        dxl.cleanup()
    simulation_app.close()
    exit()

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
my_world.scene.add_default_ground_plane()
my_controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)

# Reset and start simulation
my_world.reset()
print("[✓] Isaac Sim ready")

# Start timeline playing
from omni.timeline import get_timeline_interface
timeline = get_timeline_interface()
timeline.play()
print("[✓] Timeline started - simulation playing")

# Give it a moment to start
time.sleep(0.5)

print("\n" + "=" * 70)
print("INTEGRATION ACTIVE")
print("=" * 70)
print("Mode: Isaac Sim → Dynamixel Servos")
print("  - Simulation drives real motors")
print("  - Left wheel → Motor ID 1")
print("  - Right wheel → Motor ID 2")
print("\nRobot will move forward → rotate → repeat")
print("Physical motors will mirror simulation!")
print("Press Ctrl+C to stop or close Isaac Sim window")
print("=" * 70)
print()

i = 0
reset_needed = False
last_feedback_time = time.time()
loop_count = 0
running = True

try:
    while simulation_app.is_running() and running:
        my_world.step(render=True)
        loop_count += 1
        
        # Debug: print status occasionally
        if loop_count % 100 == 1:
            playing = my_world.is_playing()
            stopped = my_world.is_stopped()
            is_running = simulation_app.is_running()
            print(f"[Debug] Loop {loop_count}: is_running={is_running}, playing={playing}, stopped={stopped}")
        
        # Skip if world is not ready
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
        
        # Generate movement commands
        if i >= 0 and i < 1000:
            # Forward
            wheel_actions = my_controller.forward(command=[0.1, 0])
        elif i >= 1000 and i < 1300:
            # Rotate
            wheel_actions = my_controller.forward(command=[0.0, np.pi / 12])
        elif i >= 1300 and i < 2000:
            # Forward
            wheel_actions = my_controller.forward(command=[0.1, 0])
        elif i == 2000:
            i = 0
            wheel_actions = my_controller.forward(command=[0.0, 0])
        else:
            wheel_actions = my_controller.forward(command=[0.0, 0])
        
        # Apply to simulation
        my_jetbot.apply_wheel_actions(wheel_actions)
        
        # Extract wheel velocities from ArticulationAction object
        # wheel_actions.joint_velocities is a numpy array [left, right]
        if wheel_actions.joint_velocities is not None:
            left_vel = float(wheel_actions.joint_velocities[0])
            right_vel = float(wheel_actions.joint_velocities[1])
        else:
            left_vel = 0.0
            right_vel = 0.0
        
        # Send to Dynamixel motors
        if dxl:
            # Normalize to [-1, 1] range (actions are in rad/s, need to scale)
            # Jetbot wheel max speed is about 15 rad/s
            left_normalized = np.clip(left_vel / 15.0, -1.0, 1.0)
            right_normalized = np.clip(right_vel / 15.0, -1.0, 1.0)
            
            dxl.set_wheel_velocities(left_normalized, right_normalized)
        
        # Print feedback
        current_time = time.time()
        if current_time - last_feedback_time >= 1.0:
            last_feedback_time = current_time
            
            print(f"[Step {i:4d}] Sim: L={left_vel:+.3f} R={right_vel:+.3f} rad/s", end="")
            
            if dxl:
                actual_vels = dxl.get_velocities()
                print(f" | Motors: L={actual_vels[0]:4d} R={actual_vels[1]:4d} units")
            else:
                print(" | (No hardware)")
        
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
