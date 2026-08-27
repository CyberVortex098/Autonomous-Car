import serial
import time
from collections import defaultdict

PORT = "/dev/ttyAMA0"
BAUD = 921600
TYPES = ["$DIST", "$IMU", "$BUTTON", "$ADC", "$COLOR"]

counts = defaultdict(int)
last_time = time.time()

with serial.Serial(PORT, BAUD, timeout=1) as ser:
    buf = b""
    while True:
        buf += ser.read(ser.in_waiting or 1)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            line = line.strip().decode(errors="ignore")
            for t in TYPES:
                if line.startswith(t):
                    counts[t] += 1
                    break

        now = time.time()
        dt = now - last_time
        if dt >= 3.0:
            parts = ", ".join(
                f"{t.lstrip('$')}={counts[t]/dt:.1f}"
                for t in TYPES
            )
            print(f"\r{parts}   ", end="", flush=True)
            counts.clear()
            last_time = now
