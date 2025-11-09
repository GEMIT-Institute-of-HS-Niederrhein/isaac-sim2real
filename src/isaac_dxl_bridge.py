#!/usr/bin/env python3
"""
Isaac Sim 5.1 <-> Dynamixel XL430 Bridge Demo
Drives 4 physical servo wheels based on Isaac Sim robot wheel velocities
"""

import os
import numpy as np
from dynamixel_sdk import *
import threading
import time

# ========== Dynamixel Setup ==========
DEV_PORT = "/dev/ttyUSB0"
BAUDRATE = 57600
MOTOR_IDS = [1, 2, 3, 4]  # FL, FR, RL, RR (Front-Left, Front-Right, etc.)

# Control table addresses
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_VELOCITY_LIMIT = 44
ADDR_GOAL_VELOCITY = 104

MODE_VELOCITY = 1
VELOCITY_LIMIT = 700  # Max velocity we'll use

class DynamixelController:
    def __init__(self):
        self.port_handler = PortHandler(DEV_PORT)
        self.packet_handler = PacketHandler(2.0)
        
        if not self.port_handler.openPort():
            raise Exception("Failed to open port")
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
            
            print(f"  Motor {motor_id} ready")
    
    def set_velocities(self, velocities):
        """
        Set wheel velocities
        velocities: list of 4 values [FL, FR, RL, RR] in range [-1, 1]
        """
        # Convert normalized velocities to Dynamixel units
        # 1 LSB ≈ 0.229 rpm, so velocity ~600 is reasonable
        for i, motor_id in enumerate(MOTOR_IDS):
            vel = int(velocities[i] * 600)  # Scale to ±600
            # Dynamixel uses signed 32-bit, need to handle negative
            if vel < 0:
                vel = (1 << 32) + vel
            
            self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, ADDR_GOAL_VELOCITY, vel)
    
    def stop_all(self):
        for motor_id in MOTOR_IDS:
            self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, ADDR_GOAL_VELOCITY, 0)
    
    def cleanup(self):
        self.stop_all()
        self.port_handler.closePort()


# ========== Isaac Sim 5.1 Setup ==========
def setup_isaac_sim():
    """Initialize Isaac Sim 5.1 with a simple 4-wheeled robot"""
    from isaacsim import SimulationApp
    
    # Launch Isaac Sim with GUI
    simulation_app = SimulationApp({"headless": False})
    
    # Import after SimulationApp is created (Isaac Sim 5.1 requirement)
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.utils.nucleus import get_assets_root_path
    import omni.usd
    from pxr import Usd, UsdGeom, Gf, Sdf
    
    # Create world
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Reset first to avoid clearing loaded prims afterwards
    world.reset()

    print("[Isaac Sim] Loading robot...")

    # Prefer local USD to avoid Nucleus issues
    # Updated path: assets directory in isaac-sim2real repo
    local_robot = os.path.expanduser("~/Desktop/isaac-sim2real/assets/ROBOT.usd")
    robot = None
    loaded_any = False
    try:
        if os.path.exists(local_robot):
            add_reference_to_stage(usd_path=local_robot, prim_path="/World/Robot")
            print(f"[Isaac Sim] Loaded local robot: {local_robot}")
            loaded_any = True
        else:
            # Fallback to Jetbot via assets root (may require nucleus)
            assets_root_path = get_assets_root_path()
            if assets_root_path:
                robot_usd = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
                add_reference_to_stage(usd_path=robot_usd, prim_path="/World/jetbot")
                print(f"[Isaac Sim] Loaded Jetbot from assets: {robot_usd}")
                loaded_any = True
    except Exception as e:
        print(f"[Isaac Sim] Error loading robot USD: {e}")

    # If nothing loaded, spawn a visible cube for sanity
    if not loaded_any:
        try:
            stage = omni.usd.get_context().get_stage()
            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
            cube = UsdGeom.Cube.Define(stage, Sdf.Path("/World/DebugCube"))
            xform = UsdGeom.Xformable(cube.GetPrim())
            xform.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.5))
            print("[Isaac Sim] Spawned fallback DebugCube at /World/DebugCube")
        except Exception as e:
            print(f"[Isaac Sim] Failed to spawn DebugCube: {e}")

    print("[Isaac Sim] Ready!")
    print("[Isaac Sim] Use Arrow Keys to drive the robot")
    print("[Isaac Sim] The physical wheels will mirror the simulation")

    return simulation_app, world, robot


# ========== Keyboard Control ==========
class KeyboardController:
    def __init__(self):
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.running = True
        
    def get_velocities(self):
        """Return differential drive velocities for 4 wheels"""
        # Differential drive: left/right wheel velocities
        left_vel = self.linear_vel - self.angular_vel * 0.5
        right_vel = self.linear_vel + self.angular_vel * 0.5
        
        # Clamp to [-1, 1]
        left_vel = np.clip(left_vel, -1, 1)
        right_vel = np.clip(right_vel, -1, 1)
        
        # Return [FL, FR, RL, RR]
        return [left_vel, right_vel, left_vel, right_vel]


# ========== Main Bridge Loop ==========
def main():
    print("=" * 70)
    print("Isaac Sim 5.1 + Dynamixel XL430 Bridge Demo")
    print("=" * 70)
    print()
    
    # Initialize Dynamixel
    print("[1/3] Initializing Dynamixel motors...")
    dxl = DynamixelController()
    print("[✓] Dynamixel ready\n")
    
    # Initialize Isaac Sim
    print("[2/3] Starting Isaac Sim 5.1...")
    simulation_app, world, robot = setup_isaac_sim()
    print("[✓] Isaac Sim ready\n")
    
    # Import keyboard after Isaac Sim loads (avoids conflicts)
    from pynput import keyboard
    
    # Keyboard controller
    kb = KeyboardController()
    
    def on_press(key):
        try:
            if key == keyboard.Key.up:
                kb.linear_vel = min(kb.linear_vel + 0.1, 1.0)
            elif key == keyboard.Key.down:
                kb.linear_vel = max(kb.linear_vel - 0.1, -1.0)
            elif key == keyboard.Key.left:
                kb.angular_vel = min(kb.angular_vel + 0.1, 1.0)
            elif key == keyboard.Key.right:
                kb.angular_vel = max(kb.angular_vel - 0.1, -1.0)
            elif key == keyboard.Key.space:
                kb.linear_vel = 0.0
                kb.angular_vel = 0.0
        except AttributeError:
            pass
    
    def on_release(key):
        # Gradual slowdown for smoother control
        if key in [keyboard.Key.up, keyboard.Key.down]:
            kb.linear_vel *= 0.8
        elif key in [keyboard.Key.left, keyboard.Key.right]:
            kb.angular_vel *= 0.8
        
        if key == keyboard.Key.esc:
            kb.running = False
            return False
    
    # Start keyboard listener in background
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    print("[3/3] Starting control loop...")
    print("\n" + "=" * 70)
    print("CONTROLS:")
    print("  ↑/↓  : Forward/Backward")
    print("  ←/→  : Turn Left/Right")
    print("  SPACE: Stop")
    print("  ESC  : Quit")
    print("=" * 70)
    print()
    print("Status: Running... (watch your physical wheels!)")
    print()
    
    step_count = 0
    
    try:
        while simulation_app.is_running() and kb.running:
            # Step simulation (Isaac Sim 5.1 style)
            world.step(render=True)
            
            # Get desired wheel velocities from keyboard
            wheel_vels = kb.get_velocities()
            
            # Send to physical motors
            dxl.set_velocities(wheel_vels)
            
            # Optionally: If we wire an articulation later, we can mirror velocities there
            
            # Print status every 100 steps (~1 second)
            step_count += 1
            if step_count % 100 == 0:
                print(f"[Status] Linear: {kb.linear_vel:+.2f} | Angular: {kb.angular_vel:+.2f} | "
                      f"Wheels: [{wheel_vels[0]:+.2f}, {wheel_vels[1]:+.2f}, {wheel_vels[2]:+.2f}, {wheel_vels[3]:+.2f}]")
            
            # 100Hz update rate
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\n[Interrupted by user]")
    
    finally:
        print("\n[Cleanup] Stopping motors...")
        dxl.cleanup()
        simulation_app.close()
        print("[Done] Goodbye!")


if __name__ == "__main__":
    main()