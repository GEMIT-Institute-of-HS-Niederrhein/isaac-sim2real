# Hardware Configuration
# Copy this to config.py and adjust for your setup

# Dynamixel Serial Port
DEVICE_PORT = "/dev/ttyUSB0"
BAUDRATE = 57600

# Motor IDs (Front-Left, Front-Right, Rear-Left, Rear-Right)
MOTOR_IDS = [1, 2, 3, 4]

# Motor Control Parameters
VELOCITY_LIMIT = 700  # Maximum velocity in Dynamixel units
VELOCITY_SCALE = 600  # Scale factor for normalized velocities

# Isaac Sim Configuration
ISAAC_SIM_PATH = "~/Desktop/isaacsim/_build/linux-x86_64/release"
ROBOT_USD_PATH = "~/Desktop/ROBOT.usd"

# Control Parameters
UPDATE_RATE_HZ = 100  # Control loop frequency
STATUS_PRINT_INTERVAL = 100  # Print status every N steps
