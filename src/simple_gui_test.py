#!/usr/bin/env python3
"""
Simple GUI to test Dynamixel wheels before Isaac Sim integration
Use keyboard/gamepad to drive, visualize wheel states
"""

import tkinter as tk
from tkinter import ttk
import numpy as np
from dynamixel_sdk import *
import threading
import time

# ========== Config ==========
DEV_PORT = "/dev/ttyUSB0"
BAUDRATE = 57600
MOTOR_IDS = [1, 2, 3, 4]

# Control table
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_VELOCITY_LIMIT = 44
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_VELOCITY = 128

MODE_VELOCITY = 1
VELOCITY_LIMIT = 700


class DynamixelController:
    def __init__(self):
        self.port_handler = PortHandler(DEV_PORT)
        self.packet_handler = PacketHandler(2.0)
        self.running = False
        
        if not self.port_handler.openPort():
            raise Exception("Failed to open port")
        if not self.port_handler.setBaudRate(BAUDRATE):
            raise Exception("Failed to set baudrate")
        
        print("Initializing motors...")
        for motor_id in MOTOR_IDS:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 0)
            self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, ADDR_OPERATING_MODE, MODE_VELOCITY)
            self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT)
            self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 1)
            print(f"  Motor {motor_id} ready")
    
    def set_velocities(self, velocities):
        """velocities: [FL, FR, RL, RR] in range [-1, 1]"""
        for i, motor_id in enumerate(MOTOR_IDS):
            vel = int(velocities[i] * 600)
            if vel < 0:
                vel = (1 << 32) + vel
            self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, ADDR_GOAL_VELOCITY, vel)
    
    def get_velocities(self):
        """Read actual velocities from motors"""
        vels = []
        for motor_id in MOTOR_IDS:
            val, _, _ = self.packet_handler.read4ByteTxRx(
                self.port_handler, motor_id, ADDR_PRESENT_VELOCITY)
            # Convert from unsigned to signed
            if val > 0x7FFFFFFF:
                val = val - (1 << 32)
            vels.append(val / 600.0)  # Normalize
        return vels
    
    def stop_all(self):
        for motor_id in MOTOR_IDS:
            self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, ADDR_GOAL_VELOCITY, 0)
    
    def cleanup(self):
        self.stop_all()
        self.port_handler.closePort()


class RobotGUI:
    def __init__(self, root, controller):
        self.root = root
        self.controller = controller
        self.root.title("Dynamixel 4-Wheel Drive Simulator")
        self.root.geometry("800x600")
        
        # Control state
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Create UI
        self.create_widgets()
        
        # Keyboard bindings
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)
        
        # Start update loop
        self.running = True
        self.update_loop()
    
    def create_widgets(self):
        # Title
        title = tk.Label(self.root, text="4-Wheel Robot Control", 
                         font=("Arial", 20, "bold"))
        title.pack(pady=10)
        
        # Instructions
        instructions = tk.Label(self.root, 
                                text="Arrow Keys: Drive | WASD: Alternative | Space: Stop | ESC: Quit",
                                font=("Arial", 12))
        instructions.pack(pady=5)
        
        # Canvas for robot visualization
        self.canvas = tk.Canvas(self.root, width=400, height=400, bg="white")
        self.canvas.pack(pady=20)
        
        # Draw robot body
        self.robot_body = self.canvas.create_rectangle(150, 150, 250, 250, 
                                                        fill="lightblue", outline="black", width=2)
        
        # Draw wheels (FL, FR, RL, RR)
        self.wheels = [
            self.canvas.create_rectangle(130, 130, 160, 170, fill="gray", outline="black", width=2),  # FL
            self.canvas.create_rectangle(240, 130, 270, 170, fill="gray", outline="black", width=2),  # FR
            self.canvas.create_rectangle(130, 230, 160, 270, fill="gray", outline="black", width=2),  # RL
            self.canvas.create_rectangle(240, 230, 270, 270, fill="gray", outline="black", width=2),  # RR
        ]
        
        # Wheel labels
        labels = ["FL (1)", "FR (2)", "RL (3)", "RR (4)"]
        positions = [(145, 110), (255, 110), (145, 290), (255, 290)]
        for label, pos in zip(labels, positions):
            self.canvas.create_text(pos[0], pos[1], text=label, font=("Arial", 10))
        
        # Velocity display
        self.vel_frame = tk.Frame(self.root)
        self.vel_frame.pack(pady=10)
        
        tk.Label(self.vel_frame, text="Linear Velocity:", font=("Arial", 12)).grid(row=0, column=0, padx=5)
        self.linear_label = tk.Label(self.vel_frame, text="0.00", font=("Arial", 12, "bold"))
        self.linear_label.grid(row=0, column=1, padx=5)
        
        tk.Label(self.vel_frame, text="Angular Velocity:", font=("Arial", 12)).grid(row=1, column=0, padx=5)
        self.angular_label = tk.Label(self.vel_frame, text="0.00", font=("Arial", 12, "bold"))
        self.angular_label.grid(row=1, column=1, padx=5)
        
        # Wheel velocity bars
        self.bar_frame = tk.Frame(self.root)
        self.bar_frame.pack(pady=10)
        
        self.wheel_bars = []
        for i, label in enumerate(labels):
            tk.Label(self.bar_frame, text=label, width=8).grid(row=i, column=0)
            bar = ttk.Progressbar(self.bar_frame, length=300, mode='determinate')
            bar.grid(row=i, column=1, padx=5)
            bar['maximum'] = 200  # -100 to +100
            bar['value'] = 100  # Center
            self.wheel_bars.append(bar)
    
    def on_key_press(self, event):
        key = event.keysym
        
        # Forward/Backward
        if key in ['Up', 'w', 'W']:
            self.linear_vel = 1.0
        elif key in ['Down', 's', 'S']:
            self.linear_vel = -1.0
        
        # Turn
        if key in ['Left', 'a', 'A']:
            self.angular_vel = 1.0
        elif key in ['Right', 'd', 'D']:
            self.angular_vel = -1.0
        
        # Stop
        if key == 'space':
            self.linear_vel = 0.0
            self.angular_vel = 0.0
        
        # Quit
        if key == 'Escape':
            self.cleanup()
    
    def on_key_release(self, event):
        key = event.keysym
        
        if key in ['Up', 'Down', 'w', 'W', 's', 'S']:
            self.linear_vel = 0.0
        if key in ['Left', 'Right', 'a', 'A', 'd', 'D']:
            self.angular_vel = 0.0
    
    def update_loop(self):
        if not self.running:
            return
        
        # Calculate wheel velocities (differential drive)
        left_vel = self.linear_vel - self.angular_vel
        right_vel = self.linear_vel + self.angular_vel
        
        # Clamp
        left_vel = np.clip(left_vel, -1, 1)
        right_vel = np.clip(right_vel, -1, 1)
        
        # FL, FR, RL, RR
        velocities = [left_vel, right_vel, left_vel, right_vel]
        
        # Send to motors
        try:
            self.controller.set_velocities(velocities)
        except Exception as e:
            print(f"Motor control error: {e}")
        
        # Update UI
        self.linear_label.config(text=f"{self.linear_vel:+.2f}")
        self.angular_label.config(text=f"{self.angular_vel:+.2f}")
        
        # Update wheel visualization
        for i, vel in enumerate(velocities):
            # Color based on velocity
            if vel > 0:
                color = f"#{int(100 + vel*155):02x}ff{int(100 + vel*155):02x}"  # Green
            elif vel < 0:
                color = f"#ff{int(100 - vel*155):02x}{int(100 - vel*155):02x}"  # Red
            else:
                color = "gray"
            
            self.canvas.itemconfig(self.wheels[i], fill=color)
            
            # Update progress bar
            self.wheel_bars[i]['value'] = 100 + vel * 100  # Map -1..1 to 0..200
        
        # Schedule next update (100Hz)
        self.root.after(10, self.update_loop)
    
    def cleanup(self):
        self.running = False
        self.controller.cleanup()
        self.root.quit()


def main():
    print("=" * 60)
    print("Dynamixel 4-Wheel Drive Test GUI")
    print("=" * 60)
    print()
    
    try:
        # Initialize controller
        controller = DynamixelController()
        
        # Create GUI
        root = tk.Tk()
        gui = RobotGUI(root, controller)
        
        # Run
        root.mainloop()
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    
    print("\nDone!")


if __name__ == "__main__":
    main()