# usb_comm.py
from __future__ import annotations
import serial, time, string

HEX = set("0123456789ABCDEFabcdef")

def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else ((crc << 1) & 0xFFFF)
    return crc

def hex4(n: int) -> bytes:
    return f"{n & 0xFFFF:04X}".encode("ascii")

class USBComm:
    def __init__(self, port="/dev/ttyESP32", baud=115200, timeout=1.0):
        self.port, self.baud, self.timeout = port, baud, timeout
        self.ser: serial.Serial | None = None

    def __enter__(self) -> "USBComm":
        self.ser = serial.Serial(self.port, self.baud, timeout=0.5, write_timeout=0.5)
        time.sleep(0.15)
        # IMPORTANT: drop any stale bytes
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.ser:
            self.ser.close()
            self.ser = None

    # --- framing ---
    def send(self, payload: str) -> None:
        assert self.ser, "open first"
        raw = payload.encode("ascii")
        frame = hex4(len(raw)) + raw + b"*" + hex4(crc16_ccitt_false(raw)) + b"\n"
        self.ser.write(frame)
        self.ser.flush()

    def recv(self, timeout=None) -> tuple[str | None, str]:
        """
        Returns (payload or None, status) where status in {'ok','crc','timeout','format'}.
        Robust: discards junk until it locks onto a 4-hex length header.
        """
        assert self.ser, "open first"
        ser = self.ser
        deadline = time.time() + (timeout if timeout is not None else self.timeout)

        # --- resync: find 4 ASCII-HEX characters in a row ---
        buf = bytearray()
        while time.time() < deadline:
            b = ser.read(1)
            if not b:
                continue
            c = b.decode('ascii', 'ignore')
            if c and c in string.hexdigits:
                buf += b
                if len(buf) == 4:
                    break
            else:
                buf.clear()  # restart window if non-hex
        else:
            return (None, "timeout")

        # Parse length
        try:
            plen = int(buf.decode('ascii'), 16)
        except Exception:
            return (None, "format")

        # Read payload
        payload = ser.read(plen)
        if len(payload) < plen:
            return (None, "timeout")

        # Expect '*'
        star = ser.read(1)
        if star != b"*":
            return (None, "format")

        # CRC
        crc4 = ser.read(4)
        if len(crc4) < 4:
            return (None, "timeout")
        try:
            want = int(crc4.decode("ascii"), 16)
        except Exception:
            return (None, "format")

        # LF
        lf = ser.read(1)
        if lf != b"\n":
            return (None, "format")

        got = crc16_ccitt_false(payload)
        if got != want:
            return (None, "crc")

        return (payload.decode("ascii", "ignore"), "ok")

