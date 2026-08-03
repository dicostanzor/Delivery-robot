# actuator_receiver.py
# Runs on the Jetson Orin NX
# Listens for actuator trigger commands from the Pi on port 5006
# On trigger, runs the full timed actuator cycle using Jetson GPIO

import socket
import Jetson.GPIO as GPIO
import time

HOST = '0.0.0.0'
PORT = 5006

# GPIO pin definitions (BOARD physical numbering)
IN1 = 29
IN2 = 31

# Calibration settings — update once real values are confirmed
TIME_FOR_FULL_INCH = 1.66   # Seconds per inch of travel
TARGET_DISTANCE = 0.8       # Target extension in inches
HOLD_TIME = 3.0             # Seconds to hold extended position

# Calculate runtime for target distance
EXTEND_RUNTIME = TIME_FOR_FULL_INCH * TARGET_DISTANCE

# Initialize GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)


def stop():
    """Stops the actuator by setting both control pins LOW."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)


def run_actuator_cycle():
    """
    Runs the full timed actuator cycle:
    1. Fully retract to reset position
    2. Extend to target distance
    3. Hold at target
    4. Retract back to start
    """
    try:
        # Step 1: Reset — fully retract
        print("  Retracting to reset position...")
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        time.sleep(TIME_FOR_FULL_INCH + 0.5)  # Extra buffer to ensure full retraction
        stop()
        time.sleep(1)

        # Step 2: Extend to target distance
        print(f"  Extending to {TARGET_DISTANCE} inches...")
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        time.sleep(EXTEND_RUNTIME)

        # Step 3: Hold at target position
        stop()
        print(f"  Holding for {HOLD_TIME} seconds...")
        time.sleep(HOLD_TIME)

        # Step 4: Retract back to start
        print("  Retracting back to start...")
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        time.sleep(TIME_FOR_FULL_INCH + 0.5)
        stop()
        print("  Cycle complete.")

    except Exception as e:
        stop()
        print(f"  [!] Actuator error: {e}")


# --- Main server loop ---
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(5)

print(f"Actuator receiver listening on port {PORT}...")

while True:
    conn, addr = server.accept()
    data = conn.recv(1024).decode().strip()
    conn.close()
    if data == "actuator":
        print("  Actuator cycle triggered")
        run_actuator_cycle()
    else:
        print(f"  [!] Unknown command: {data}")
