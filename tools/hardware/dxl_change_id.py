#!/usr/bin/env python3
"""
Change a DYNAMIXEL XL430-W250-T ID from the command line.

Examples
--------
# Change ID 1 -> 3 on /dev/ttyUSB0 at 57600 baud
python dxl_change_id.py --port /dev/ttyUSB0 --baud 57600 --old 1 --new 3

# On Windows (example COM7)
python dxl_change_id.py --port COM7 --baud 57600 --old 1 --new 3

Notes
-----
- Works with Protocol 2.0 servos (XL430 series).
- For safety, connect only the servo you want to change.
- Address map used:
  ID @ 7 (1 byte), TORQUE_ENABLE @ 64 (1 byte)
"""

import sys
import time
import argparse
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# Control Table (Protocol 2.0)
ADDR_ID             = 7    # 1 byte
ADDR_TORQUE_ENABLE  = 64   # 1 byte

TORQUE_ENABLE       = 1
TORQUE_DISABLE      = 0
PROTOCOL_VERSION    = 2.0


def open_serial(port_name: str, baudrate: int) -> PortHandler:
    port = PortHandler(port_name)
    if not port.openPort():
        raise RuntimeError(f"Could not open serial port: {port_name}")
    if not port.setBaudRate(baudrate):
        port.closePort()
        raise RuntimeError(f"Could not set baudrate: {baudrate}")
    return port


def ping(packet: PacketHandler, port: PortHandler, dxl_id: int):
    model, comm, err = packet.ping(port, dxl_id)
    if comm != COMM_SUCCESS:
        raise RuntimeError(f"Ping ID {dxl_id} failed: {packet.getTxRxResult(comm)}")
    if err != 0:
        print(f"[WARN] Ping error: {packet.getRxPacketError(err)}")
    return model


def id_in_use(packet: PacketHandler, port: PortHandler, dxl_id: int) -> bool:
    try:
        ping(packet, port, dxl_id)
        return True
    except Exception:
        return False


def write_id(packet: PacketHandler, port: PortHandler, old_id: int, new_id: int):
    # Disable torque before configuration changes
    comm, err = packet.write1ByteTxRx(port, old_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if comm != COMM_SUCCESS:
        raise RuntimeError(f"Disable torque failed: {packet.getTxRxResult(comm)}")
    if err != 0:
        print(f"[WARN] Disable torque error: {packet.getRxPacketError(err)}")

    # Write new ID
    comm, err = packet.write1ByteTxRx(port, old_id, ADDR_ID, new_id)
    if comm != COMM_SUCCESS:
        raise RuntimeError(f"Write new ID failed: {packet.getTxRxResult(comm)}")
    if err != 0:
        print(f"[WARN] Write new ID error: {packet.getRxPacketError(err)}")


def main():
    ap = argparse.ArgumentParser(description="Change a DYNAMIXEL XL430 ID.")
    ap.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyUSB0, COM7)")
    ap.add_argument("--baud", type=int, default=57600, help="Baud rate (default: 57600)")
    ap.add_argument("--old", type=int, required=True, help="Current servo ID")
    ap.add_argument("--new", type=int, required=True, help="Desired new servo ID")
    ap.add_argument("--force", action="store_true",
                    help="Proceed even if target ID already responds on the bus (NOT recommended)")
    args = ap.parse_args()

    if args.new < 0 or args.new > 252:
        print("[ERROR] New ID must be between 0 and 252 (Protocol 2.0 range).")
        sys.exit(2)

    packet = PacketHandler(PROTOCOL_VERSION)

    print(f"[INFO] Opening {args.port} @ {args.baud}...")
    port = open_serial(args.port, args.baud)

    try:
        print(f"[INFO] Checking current ID {args.old}...")
        model = ping(packet, port, args.old)
        print(f"[OK] Found device model {model} on ID {args.old}.")

        if not args.force and id_in_use(packet, port, args.new):
            print(f"[ERROR] Another device already responds on ID {args.new}. "
                  f"Use --force to override (NOT recommended on a shared bus).")
            sys.exit(3)

        print(f"[INFO] Changing ID {args.old} -> {args.new} ...")
        write_id(packet, port, args.old, args.new)

        # Give the device a brief moment to apply
        time.sleep(0.1)

        print(f"[INFO] Verifying new ID {args.new}...")
        model2 = ping(packet, port, args.new)
        print(f"[SUCCESS] ID changed: {args.old} -> {args.new} (model {model2}).")

        # Optional: leave torque disabled (safer) — user can re-enable later if needed
        print("[INFO] Done. Torque remains disabled for safety.")

    except Exception as e:
        print(f"[FAIL] {e}")
        sys.exit(1)
    finally:
        port.closePort()


if __name__ == "__main__":
    main()
