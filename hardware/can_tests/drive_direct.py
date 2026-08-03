import time
import sys
from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame

# Scan for CANable
devs = GsUsb.scan()

if len(devs) == 0:
    print("❌ Error: Cannot find gs_usb device.")
    sys.exit(1)

dev = devs[0]

print("Configuring 1 Mbps bit timing...")

try:
    dev.set_bitrate(1000000)
    print("✅ Bitrate configured.")
except Exception as e:
    print(f"⚠️ Bitrate warning: {e}")

try:
    dev.start(0)
    print("✅ CANable started.")
except Exception as e:
    print(f"❌ Failed to start CANable: {e}")
    sys.exit(1)

print("🚀 Starting diagnostic transmit test...")

# Heartbeat frame
heartbeat = GsUsbFrame()
heartbeat.can_id = 0x000401BF | (1 << 31)  # Extended frame
heartbeat.data = [0x01, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

loop_count = 0

try:
    while True:
        loop_count += 1

        try:
            dev.send(heartbeat)

            if loop_count % 100 == 0:
                print(f"Heartbeat transmitted ({loop_count})")

        except Exception as e:
            print(f"❌ Heartbeat send failed: {e}")

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nStopped.")
