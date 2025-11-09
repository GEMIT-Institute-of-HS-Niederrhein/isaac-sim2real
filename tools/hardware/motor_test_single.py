"""
XL430-W250-T — Simple One-Servo Demo (Python)

What this script does:
1) Opens a serial connection to your U2D2 (USB ↔ Dynamixel)
2) Pings the servo to make sure it's there
3) Puts the motor into "Position Mode"
4) Enables torque (turns the motor “on”)
5) Moves to 0°, then 90°, then back to 0°, printing its current position
6) Turns torque off and closes the port

How to use:
- Set PORT_NAME below (Linux: '/dev/ttyUSB0', Windows: 'COM7', macOS: '/dev/tty.usbserial-*')
- Leave ID = 1 and BAUDRATE = 57600 unless you changed them in DYNAMIXEL Wizard
- Run:  python xl430_simple_demo.py
"""

import time
from dynamixel_sdk import PortHandler, PacketHandler  # pip install dynamixel-sdk

# ------------------------------
# USER SETTINGS — CHANGE THESE IF NEEDED
# ------------------------------
PORT_NAME  = "/dev/ttyUSB0"  # e.g., 'COM7' on Windows
BAUDRATE   = 57600           # XL430 default (unless you changed it)
DXL_ID     = 3               # XL430 default ID
# ------------------------------

# DYNAMIXEL uses "addresses" to read/write settings (Control Table, Protocol 2.0)
ADDR_OPERATING_MODE    = 11   # 0=current, 1=velocity, 3=position, 5=pwm
ADDR_TORQUE_ENABLE     = 64   # 0=off, 1=on
ADDR_GOAL_POSITION     = 116  # where we want to go (0..4095 ticks)
ADDR_PRESENT_POSITION  = 132  # where we are now (0..4095 ticks)

# Useful values
PROTOCOL_VERSION       = 2.0
OPERATING_MODE_POSITION = 3
TORQUE_ENABLE          = 1
TORQUE_DISABLE         = 0

# Helpers: convert between human-friendly degrees and servo "ticks"
# XL430 is 12-bit (0..4095) ~ 0..360 degrees
def deg_to_ticks(deg: float) -> int:
    return int(round((deg % 360.0) * 4095.0 / 360.0))

def ticks_to_deg(ticks: int) -> float:
    return ticks * 360.0 / 4095.0


def open_serial(port_name: str, baudrate: int):
    """Open the USB serial port to U2D2 and set baudrate."""
    port = PortHandler(port_name)
    if not port.openPort():
        raise RuntimeError(f"Could not open serial port: {port_name}")
    if not port.setBaudRate(baudrate):
        port.closePort()
        raise RuntimeError(f"Could not set baudrate to {baudrate}")
    return port


def ping_servo(packet, port, dxl_id: int):
    """Ping the servo to confirm communication and get its model number."""
    model, comm_result, error = packet.ping(port, dxl_id)
    if comm_result != 0:  # COMM_SUCCESS == 0
        raise RuntimeError(f"Ping failed: {packet.getTxRxResult(comm_result)}")
    if error != 0:
        print(f"Ping reported error: {packet.getRxPacketError(error)}")
    print(f"✓ Connected: DXL ID {dxl_id}, model {model}")


def set_position_mode(packet, port, dxl_id: int):
    """Ensure the servo is in Position Mode (required for GOAL_POSITION)."""
    # Must disable torque before changing mode
    packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    _, err = packet.write1ByteTxRx(port, dxl_id, ADDR_OPERATING_MODE, OPERATING_MODE_POSITION)
    if err != 0:
        print(f"Set mode warning: {packet.getRxPacketError(err)}")


def torque_on(packet, port, dxl_id: int, enable: bool):
    """Turn motor torque on/off (True = on)."""
    val = TORQUE_ENABLE if enable else TORQUE_DISABLE
    comm, err = packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, val)
    if comm != 0:
        raise RuntimeError(f"Torque {'enable' if enable else 'disable'} failed: {packet.getTxRxResult(comm)}")
    if err != 0:
        print(f"Torque {'enable' if enable else 'disable'} warning: {packet.getRxPacketError(err)}")


def goto_degrees(packet, port, dxl_id: int, degrees: float):
    """Command a goal position in degrees."""
    goal = deg_to_ticks(degrees)
    comm, err = packet.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, goal)
    if comm != 0:
        raise RuntimeError(f"Write goal failed: {packet.getTxRxResult(comm)}")
    if err != 0:
        print(f"Write goal warning: {packet.getRxPacketError(err)}")


def read_present_degrees(packet, port, dxl_id: int) -> float:
    """Read the current position and return it in degrees."""
    ticks, comm, err = packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    if comm != 0:
        raise RuntimeError(f"Read position failed: {packet.getTxRxResult(comm)}")
    if err != 0:
        print(f"Read position warning: {packet.getRxPacketError(err)}")
    return ticks_to_deg(ticks)


def wait_briefly_and_print_position(packet, port, dxl_id: int, seconds: float):
    """For a few seconds, print where the servo actually is."""
    t_end = time.time() + seconds
    while time.time() < t_end:
        try:
            pos_deg = read_present_degrees(packet, port, dxl_id)
            print(f"  → Position now: {pos_deg:7.2f}°")
        except Exception as e:
            print(f"  (read issue: {e})")
        time.sleep(0.05)  # 20 Hz print is enough


def main():
    print("Opening port...")
    port = open_serial(PORT_NAME, BAUDRATE)
    packet = PacketHandler(PROTOCOL_VERSION)
    try:
        print("Pinging servo...")
        ping_servo(packet, port, DXL_ID)

        print("Setting Position Mode...")
        set_position_mode(packet, port, DXL_ID)

        print("Enabling torque...")
        torque_on(packet, port, DXL_ID, True)

        # Move sequence: 0° → 90° → 0°
        sequence = [(0.0, 1.0), (90.0, 2.0), (0.0, 2.0)]
        for target_deg, hold_s in sequence:
            print(f"Moving to {target_deg:.1f}° ...")
            goto_degrees(packet, port, DXL_ID, target_deg)
            wait_briefly_and_print_position(packet, port, DXL_ID, hold_s)

        print("Done.")

    finally:
        # Always turn torque off when leaving, even on error
        try:
            print("Disabling torque and closing port...")
            torque_on(packet, port, DXL_ID, False)
        except Exception:
            pass
        port.closePort()


if __name__ == "__main__":
    main()
