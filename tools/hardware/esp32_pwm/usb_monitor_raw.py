# monitor_raw.py
import sys, serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyESP32"
ser = serial.Serial(port, 115200, timeout=0.2, rtscts=False, dsrdtr=False)
ser.dtr = False; ser.rts = False
ser.reset_input_buffer()

print("Raw monitor — Ctrl+C to quit")
try:
    while True:
        b = ser.read(1024)
        if b:
            print(repr(b))
except KeyboardInterrupt:
    pass
finally:
    ser.close()

