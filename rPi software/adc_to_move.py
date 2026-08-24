#!/usr/bin/env python3
"""
Read $ADC,x,y,z,a from /dev/ttyAMA0 (z,a ignored), apply deadband +
scaling to x,y, and write $MOVE,x,y back out on the same port.

Mapping (per axis, input range 0-100):
    40-60           -> 0                (deadband)
    60-100          -> scaled 0..100
    0-40            -> scaled -100..0
"""

import serial

PORT = "/dev/ttyAMA0"
BAUD = 921600

ADC_PREFIX = "$ADC"


def scale(v: float) -> int:
    """Map raw 0-100 ADC value to -100..100 with a 40-60 deadband."""
    v = max(0.0, min(100.0, v))  # clamp to expected range

    if 40 <= v <= 60:
        return 0
    elif v > 60:
        return round((v - 60) / 40 * 100)
    else:  # v < 40
        return round((v - 40) / 40 * 100)


def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Listening on {PORT} @ {BAUD} baud, filtering for '{ADC_PREFIX}'...")

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue  # timeout, no data

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line.startswith(ADC_PREFIX):
                continue

            fields = line.split(",")
            if len(fields) != 5:  # $ADC,x,y,z,a
                print(f"Malformed ADC line, skipping: {line!r}")
                continue

            try:
                x_raw, y_raw = float(fields[1]), float(fields[2])
                # fields[3], fields[4] = z, a -> ignored
            except ValueError:
                print(f"Non-numeric ADC value, skipping: {line!r}")
                continue

            x_out = scale(x_raw) * -1
            y_out = scale(y_raw)

            move_msg = f"$MOVE,{x_out},{y_out},0\n"
            ser.write(move_msg.encode("utf-8"))
            print(f"{line}  ->  {move_msg.strip()}")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
