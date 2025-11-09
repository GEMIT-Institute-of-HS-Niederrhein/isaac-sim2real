# !/usr/bin/env python3
"""
DYNAMIXEL ID Scanner (Protocol 2.0)

Purpose
-------
Scan the TTL bus for servos that respond on given ID ranges and (optionally)
multiple baud rates. Prints a compact table of discovered devices including
their model numbers.

Typical Use
-----------
$ python dxl_scan.py
$ python dxl_scan.py --ids 1-8
$ python dxl_scan.py --ids 1-50 --bauds 57600,115200,1000000
$ python dxl_scan.py --port COM7 --ids 1-8 --baud 57600

Defaults
--------
- port:   /dev/ttyUSB0          (Windows: 'COMx', macOS: '/dev/tty.usbserial-xxxx')
- ids:    1-8                    (interpreted as 1,2,3,4,5,6,7,8)
- baud:   57600 (single baud)
- bauds:  (none). If you pass --bauds, the script will iterate those instead
          of the single --baud. Example: --bauds 57600,115200,1000000

Assumptions
-----------
- Protocol: 2.0 (XL430-W250-T and most current DYNAMIXEL models)
- A single half-duplex TTL bus via U2D2 / Power Hub

Safety
------
This script only pings (no motion). Still, keep hardware powered and wired
correctly (yellow signal wire aligned) and ensure a common ground.

Author
------
Your Team / Project Name
"""

from __future__ import annotations
import argparse
from typing import List, Tuple, Dict

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PROTOCOL_VERSION = 2.0


# -------------------- Utilities --------------------
def parse_ids(ids_arg: str) -> List[int]:
    """
    Parse an ID list specification:
      - Comma-separated: "1,2,3,8"
      - Range: "1-8"
      - Mixed: "1-3,7,9-10"
    Returns a de-duplicated list preserving order.
    """
    ids: List[int] = []
    for part in ids_arg.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            a, b = int(a), int(b)
            step = 1 if a <= b else -1
            ids.extend(range(a, b + step, step))
        else:
            ids.append(int(part))
    seen = set()
    out = []
    for i in ids:
        if 0 <= i <= 252 and i not in seen:
            seen.add(i)
            out.append(i)
    return out


def open_serial(port_name: str, baudrate: int) -> Tuple[PortHandler, PacketHandler]:
    """
    Open the serial port and set baudrate. Returns (port, packet) handlers.

    Raises:
        RuntimeError on failure to open or set baud.
    """
    port = PortHandler(port_name)
    if not port.openPort():
        raise RuntimeError(f"Could not open serial port: {port_name}")
    if not port.setBaudRate(baudrate):
        port.closePort()
        raise RuntimeError(f"Could not set baudrate: {baudrate}")
    packet = PacketHandler(PROTOCOL_VERSION)
    return port, packet


def ping(packet: PacketHandler, port: PortHandler, dxl_id: int):
    """
    Ping an ID. Returns (ok, model_number, comm_err_text, dev_err_text).
    """
    model, comm, dev = packet.ping(port, dxl_id)
    ok = (comm == COMM_SUCCESS)
    comm_txt = None if ok else packet.getTxRxResult(comm)
    dev_txt = None if not dev else packet.getRxPacketError(dev)
    return ok, (model if ok else None), comm_txt, dev_txt


# -------------------- Scanner --------------------
def scan_ids(port_name: str, baud: int, ids: List[int]) -> Dict[int, int]:
    """
    Scan a list of IDs at a single baudrate.

    Returns:
        dict {id: model_number}
    """
    found: Dict[int, int] = {}
    port, packet = open_serial(port_name, baud)
    try:
        print(f"[INFO] Scanning {len(ids)} ID(s) on {port_name} @ {baud} …")
        for i in ids:
            ok, model, comm_txt, dev_txt = ping(packet, port, i)
            if ok:
                found[i] = model
                print(f"  [FOUND] ID {i:3d} → model {model}")
            else:
                # Uncomment to see all misses:
                # print(f"  [MISS]  ID {i:3d} ({comm_txt or ''} {dev_txt or ''})")
                pass
    finally:
        port.closePort()
    return found


def multi_baud_scan(port_name: str, bauds: List[int], ids: List[int]) -> Dict[int, Dict[int, int]]:
    """
    Scan multiple baud rates.

    Returns:
        dict {baud: {id: model_number}}
    """
    results: Dict[int, Dict[int, int]] = {}
    for b in bauds:
        try:
            found = scan_ids(port_name, b, ids)
        except Exception as e:
            print(f"[WARN] Skipping baud {b}: {e}")
            found = {}
        results[b] = found
    return results


# -------------------- CLI --------------------
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Scan a DYNAMIXEL TTL bus for responding IDs (Protocol 2.0)."
    )
    p.add_argument("--port", default="/dev/ttyUSB0",
                   help="Serial port (Linux: /dev/ttyUSB0, Windows: COM7, macOS: /dev/tty.usbserial-xxxx)")
    p.add_argument("--ids", default="1-8",
                   help='IDs to scan, e.g., "1-8" or "1,2,10-12" (default: 1-8)')
    p.add_argument("--baud", type=int, default=57600,
                   help="Single baud to scan (default: 57600)")
    p.add_argument("--bauds", default="",
                   help="Optional list of baud rates to try, e.g., '57600,115200,1000000'. "
                        "If provided, overrides --baud and scans each in order.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    ids = parse_ids(args.ids)

    if args.bauds.strip():
        # Multi-baud mode
        bauds = [int(x.strip()) for x in args.bauds.split(",") if x.strip()]
        print(f"[CFG] port={args.port}, ids={ids}, bauds={bauds}")
        results = multi_baud_scan(args.port, bauds, ids)

        print("\n=== Scan Summary (multi-baud) ===")
        any_found = False
        for b in bauds:
            found = results.get(b, {})
            print(f"Baud {b}: {len(found)} device(s)")
            for i, model in sorted(found.items()):
                print(f"  - ID {i:3d} → model {model}")
                any_found = True
        if not any_found:
            print("No devices found. Check wiring, power, port name, and try other bauds or ID ranges.")

    else:
        # Single-baud mode
        print(f"[CFG] port={args.port}, ids={ids}, baud={args.baud}")
        try:
            found = scan_ids(args.port, args.baud, ids)
        except Exception as e:
            print(f"[FAIL] {e}")
            return

        print("\n=== Scan Summary ===")
        if found:
            print(f"Found {len(found)} device(s) at {args.baud} baud:")
            for i, model in sorted(found.items()):
                print(f"  - ID {i:3d} → model {model}")
        else:
            print("No devices found. Try a different baud (e.g., 115200 or 1000000), "
                  "check port name and cabling, and verify power.")


if __name__ == "__main__":
    main()
