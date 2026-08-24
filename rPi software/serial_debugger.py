#!/usr/bin/env python3
"""
Watches for Arduino Uno USB connection, sends IP over serial.
"""
import pyudev
import serial
import socket
import time

BAUD = 115200
ARDUINO_VID = "2341"  # Arduino official VID; clones (CH340) often use 1a86:7523

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "0.0.0.0"
    finally:
        s.close()

def send_ip(devnode, retries=3):
    for attempt in range(retries):
        try:
            time.sleep(2)  # let the Uno finish its auto-reset/bootloader wait
            with serial.Serial(devnode, BAUD, timeout=2) as ser:
                time.sleep(5)
                msg = f"IP:{get_ip()}\n"
                ser.write(msg.encode())
                print(f"Sent -> {devnode}: {msg.strip()}")
            return
        except serial.SerialException as e:
            print(f"Attempt {attempt+1} failed: {e}")
            time.sleep(1)

def main():
    context = pyudev.Context()
    monitor = pyudev.Monitor.from_netlink(context)
    monitor.filter_by(subsystem='tty')

    print("Watching for Arduino Uno...")
    for device in iter(monitor.poll, None):
        if device.action != 'add':
            continue
        vid = device.get('ID_VENDOR_ID')
        devnode = device.device_node
        if vid and ("2341" in vid or "1a86" in vid):  # official or CH340 clone
            print(f"Detected Arduino-like device at {devnode} (VID {vid})")
            send_ip(devnode)

if __name__ == "__main__":
    main()
