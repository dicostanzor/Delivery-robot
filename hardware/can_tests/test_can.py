import time
import can

print("Attempting to connect directly to CANable SLCAN interface...")

try:
    # Open the serial device directly as an SLCAN bus node at 500kbps (FRC Standard)
    bus = can.interface.Bus(
        bustype='slcan', 
        channel='/dev/ttyACM0', 
        bitrate=500000
    )
    print("Success! Listening for Talon SRX heartbeats... Press Ctrl+C to exit.")
    
    while True:
        message = bus.recv(timeout=1.0)
        if message:
            print(f"ID: {hex(message.arbitration_id)} | Data: {message.data.hex()}")
        else:
            print("No messages received yet. Checking connection loop...")

except Exception as e:
    print(f"Connection Failed: {e}")
