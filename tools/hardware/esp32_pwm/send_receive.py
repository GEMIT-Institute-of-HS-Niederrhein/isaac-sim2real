# send_cmd.py
import sys, time
from usb_comm import USBComm

cmd = sys.argv[1] if len(sys.argv) > 1 else "01CWSPWM25502CCWPWM25503CCWPWM255"

with USBComm("/dev/ttyESP32") as u:
    u.send(cmd)
    print("Sent:", cmd)
    # Optionally wait for a reply
    # time.sleep(1)
    payload, status = u.recv(timeout=0.001)
    print("Recv:", payload, status)
