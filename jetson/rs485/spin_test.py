#!/usr/bin/env python3
# spin_test.py
# Quick test: spins axes 0, 1, 2 the same direction for ~0.5s, then stops all.
# Run on the Jetson.

import serial
import time

RS485_PORT = '/dev/ttyUSB0'
RS485_BAUD = 38400
RUN_TIME = 0.5  # seconds

DIRECTION = '+'  # change to '-' to test the other way

ser = serial.Serial(
    RS485_PORT,
    RS485_BAUD,
    timeout=1,
    rtscts=False,
    dsrdtr=False
)
time.sleep(0.1)

def send(cmd):
    print(f"  -> {cmd.decode().strip()}")
    ser.write(cmd)
    time.sleep(0.05)

# Safety: stop all axes first (avoids Command Error from mid-motion state)
send(b'@0H\r')
send(b'@1H\r')
send(b'@2H\r')

# Set direction and jog mode speed on all three
for axis in ['0', '1', '2']:
    send(f'@{axis}N1_8000000\r'.encode())
    send(f'@{axis}{DIRECTION}\r'.encode())

# Start all three
for axis in ['0', '1', '2']:
    send(f'@{axis}G1\r'.encode())

print(f"Running for {RUN_TIME}s...")
time.sleep(RUN_TIME)

# Stop all three
send(b'@0H\r')
send(b'@1H\r')
send(b'@2H\r')

ser.close()
print("Done.")
