# actuator_receiver.py
# Runs on the Jetson Orin NX
# Listens for actuator commands from the Pi on port 5006
# Commands: "EXTEND" -> drive to full extension and stay
#           "RETRACT" -> drive to full retraction and stay
# No feedback/limit switches -- purely timed. Actuator is a lead-screw
# type so it holds position mechanically once power is cut (non-backdriving).

import socket
import Jetson.GPIO as GPIO
import time

HOST = '0.0.0.0'
PORT = 5006

# GPIO pin definitions (BOARD physical numbering)
IN1 = 29
IN2 = 31

# Calibration -- update once real values are confirmed
TIME_FOR_FULL_INCH = 1.66      # Seconds per inch of travel
FULL_STROKE_INCHES = 0.8       # Actuator's full stroke length
STROKE_RUNTIME = TIME_FOR_FULL_INCH * FULL_STROKE_INCHES
BUFFER = 0.5                   # Extra time to guarantee reaching the hard stop

# Track current commanded state so repeat commands are ignored
current_state = None  # "EXTENDED" | "RETRACTED" | None (unknown at boot)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)


def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)


def extend():
    global current_state
    if current_state == "EXTENDED":
        print("  Already extended, ignoring.")
        return
    print("  Extending...")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    time.sleep(STROKE_RUNTIME + BUFFER)
    stop()
    current_state = "EXTENDED"
    print("  Extended, holding (power cut).")


def retract():
    global current_state
    if current_state == "RETRACTED":
        print("  Already retracted, ignoring.")
        return
    print("  Retracting...")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    time.sleep(STROKE_RUNTIME + BUFFER)
    stop()
    current_state = "RETRACTED"
    print("  Retracted, holding (power cut).")


def handle_command(cmd):
    global current_state
    cmd = cmd.strip().upper()
    if cmd == "EXTEND":
        extend()
    elif cmd == "RETRACT":
        retract()
    else:
        print(f"  Unknown command: {cmd}")


# --- Main server loop ---
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(5)

print(f"Actuator receiver listening on port {PORT}...")

# Start in a known state -- force a retract at boot so current_state is accurate
retract()

try:
    while True:
        conn, addr = server.accept()
        data = conn.recv(1024).decode().strip()
        conn.close()
        if data:
            handle_command(data)
except KeyboardInterrupt:
    stop()
    GPIO.cleanup()
