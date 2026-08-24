#!/usr/bin/env python3
"""
Read from /dev/ttyAMA0 at 921600 baud and print only messages
starting with a chosen prefix, e.g. $IMU, $DIST, $BUTTON, $ADC, $COLOR.
"""

import argparse
import serial

PORT = "/dev/ttyAMA0"
BAUD = 921600

VALID_TAGS = ["IMU", "DIST", "BUTTON", "ADC", "COLOR"]


def parse_args():
    parser = argparse.ArgumentParser(description="Filter serial messages by tag")
    group = parser.add_mutually_exclusive_group(required=True)
    for tag in VALID_TAGS:
        group.add_argument(f"--{tag.lower()}", action="store_const",
                            const=tag, dest="tag", help=f"Show ${tag} messages")
    return parser.parse_args()


def main():
    args = parse_args()
    prefix = f"${args.tag}"

    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Listening on {PORT} @ {BAUD} baud, filtering for '{prefix}'...")

    try:
        while True:
            raw = ser.readline()  # bytes, up to \n or timeout
            if not raw:
                continue  # timeout, no data

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line.startswith(prefix):
                print(line)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
