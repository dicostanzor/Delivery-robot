import usb.core
import can
import sys

# Locate the physical CANable USB device via hardware ID
dev = usb.core.find(idVendor=0x1D50, idProduct=0x606F)
if dev is None:
    print("Error: CANable hardware adapter not found via USB.")
    sys.exit(1)

try:
    bus = can.interface.Bus(
        interface="gs_usb", channel=dev.product, bus=dev.bus, address=dev.address, bitrate=1000000
    )
except Exception as e:
    print(f"Failed to open bus: {e}")
    sys.exit(1)

print("Listening for passive Talon status packets... Ensure 12V power is ON.")
print("Press Ctrl+C to stop.")

discovered_ids = set()

try:
    while True:
        # Wait up to 1 second for a packet
        msg = bus.recv(timeout=1.0)
        if msg is not None and msg.is_extended_id:
            # Capture the bottom 6 bits representing the physical FRC device ID
            device_id = msg.arbitration_id & 0x3F
            if device_id not in discovered_ids and device_id != 0:
                discovered_ids.add(device_id)
                print(f"🎉 FOUND DEVICE! Talon SRX detected at CAN Device ID: {device_id}")
except KeyboardInterrupt:
    print("\nScan terminated.")
