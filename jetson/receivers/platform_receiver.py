# platform_receiver.py
# Runs on the Jetson Orin NX
# Listens for platform commands from the Pi on port 5007
# Translates commands to RS485 stepper motor commands via /dev/ttyUSB0
# Motor addressing: axes 1 & 2 = lift, axis 0 = tilt
# Direction: - = raise (lift), + = lower (lift), + = level (tilt), - = tilt (tilt)

import socket
import serial
import time

HOST = '0.0.0.0'
PORT = 5007
RS485_PORT = '/dev/ttyUSB0'
RS485_BAUD = 38400

# Number of steps for tilt level/tilt commands (200 = 1 rev at full step)
# Update these once real travel values are measured
TILT_STEPS = 800


def send_rs485(commands):
    """
    Opens RS485 port, sends a list of commands, then closes.
    Each command is a bytes object ending with carriage return.
    """
    ser = serial.Serial(RS485_PORT, RS485_BAUD, timeout=1)
    time.sleep(0.1)
    for cmd in commands:
        ser.write(cmd)
        time.sleep(0.05)
    ser.close()


def handle_command(command):
    """
    Translates a received string command into RS485 motor commands.
    Lift axes 1 and 2 move together for raise/lower.
    Tilt axis 0 moves independently for level/tilt.
    """
    print(f"  Received: {command}")

    if command == "raise":
        send_rs485([
            b'@1H\r', b'@2H\r', b'@0H\r',
            b'@1N1_8000000\r', b'@1-\r', b'@1G1\r',
            b'@2N1_8000000\r', b'@2-\r', b'@2G1\r',
            b'@0N1_8000000\r', b'@0-\r', b'@0G1\r',
        ])
    elif command == "lower":
        send_rs485([
            b'@1H\r', b'@2H\r', b'@0H\r',
            b'@1N1_8000000\r', b'@1+\r', b'@1G1\r',
            b'@2N1_8000000\r', b'@2+\r', b'@2G1\r',
            b'@0N1_8000000\r', b'@0+\r', b'@0G1\r',
        ])
    elif command == "stop":
        # Hard stop all axes immediately
        send_rs485([
            b'@1H\r', b'@2H\r', b'@0H\r',  # stop everything first
            b'@1H\r',   # Stop axis 1
            b'@2H\r',   # Stop axis 2
            b'@0H\r',   # Stop tilt axis 0 (safety)
        ])

    elif command == "level":
        # Tilt axis 0: + direction = retract/flatten, 200 steps
        send_rs485([
            b'@1H\r', b'@2H\r', b'@0H\r',  # stop everything first
            b'@0-\r',                           # Set direction: flatten
            f'@0N1_{TILT_STEPS}\r'.encode(),    # Set step count
            b'@0G1\r',                          # Go
        ])

    elif command == "tilt":
        # Tilt axis 0: - direction = extend/dump, 200 steps
        send_rs485([
            b'@1H\r', b'@2H\r', b'@0H\r',  # stop everything first
            b'@0+\r',                           # Set direction: tilt
            f'@0N1_{TILT_STEPS}\r'.encode(),    # Set step count
            b'@0G1\r',                          # Go
        ])

    else:
        print(f"  [!] Unknown command: {command}")


# --- Main server loop ---
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(5)

print(f"Platform receiver listening on port {PORT}...")

while True:
    conn, addr = server.accept()
    data = conn.recv(1024).decode().strip()
    conn.close()
    if data:
        handle_command(data)
