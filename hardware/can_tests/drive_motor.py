import can
import time
import sys

try:
    bus = can.interface.Bus(
        interface="socketcan",
        channel="can0",
        bitrate=1000000
    )
    print("SocketCAN interface initialized successfully!")
except Exception as e:
    print(f"Failed to initialize CAN bus: {e}")
    sys.exit(1)

CTRE_HEARTBEAT_ID = 0x000401BF

def send_heartbeat(bus):
    msg = can.Message(
        arbitration_id=CTRE_HEARTBEAT_ID,
        data=[0x01, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        is_extended_id=True
    )
    bus.send(msg)

def send_throttle(bus, device_id, throttle):
    arbitration_id = 0x02040000 | (device_id & 0x3F)
    demand = int(throttle * 1023)
    data = [
        (demand >> 16) & 0xFF,
        (demand >> 8) & 0xFF,
        demand & 0xFF,
        0x04, 0x00, 0x00, 0x00, 0x00
    ]
    msg = can.Message(
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=True
    )
    bus.send(msg)

print("🚀 RUNNING PRODUCTION OVERRIDE LOOP ON CHANNELS 0-63...")
print("Watch for the orange lights to turn GREEN!")

try:
    while True:
        send_heartbeat(bus)
        for target_id in range(64):
            send_throttle(bus, device_id=target_id, throttle=0.40)
        time.sleep(0.012)

except KeyboardInterrupt:
    print("\nShutting down safely...")
    for target_id in range(64):
        try:
            send_throttle(bus, device_id=target_id, throttle=0.0)
        except:
            pass
    bus.shutdown()
    print("Safe exit completed.")
