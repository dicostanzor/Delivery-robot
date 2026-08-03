import time
import sys
import can

def main():
    print("Connecting directly to the SocketCAN network loop...")
    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
    except Exception as e:
        print(f"Failed to open native network layer: {e}")
        return

    # 1. THE UNLOCK SIGNAL: Global FRC Enable/Heartbeat Frame ID
    # Without this frame sent every 20ms, the Talon stays in a hard lockdown
    enable_id = 0x000401BF  
    enable_payload = [0x01, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00]
    enable_msg = can.Message(arbitration_id=enable_id, data=enable_payload, is_extended_id=True)

    # 2. THE SPEED SIGNAL: Control Throttle ID for Device 0 (0x02040000)
    # Setting the payload explicitly to ~20% forward output throttle
    control_id = 0x02040000 
    control_payload = [0x00, 0x1F, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00]
    control_msg = can.Message(arbitration_id=control_id, data=control_payload, is_extended_id=True)

    print("Transmitting both Safety-Enable and Throttle frames to the bus.")
    print("Press Ctrl+C to terminate motor power immediately.")
    
    try:
        while True:
            # Broadcast the vital enablement block
            bus.send(enable_msg)
            time.sleep(0.005) # Brief pause to avoid frame collision
            
            # Broadcast the physical output instruction frame
            bus.send(control_msg)
            
            # Maintain a strict 20ms cycle cadence
            time.sleep(0.015) 
            
    except KeyboardInterrupt:
        print("\nSafety Intercept triggered. Disabling system safely...")
        # Send a neutral zero-throttle frame to kill motor power instantly
        neutral_payload = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        stop_msg = can.Message(arbitration_id=control_id, data=neutral_payload, is_extended_id=True)
        
        # Send a global system disable frame to park the firmware
        disable_payload = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        disable_msg = can.Message(arbitration_id=enable_id, data=disable_payload, is_extended_id=True)
        
        bus.send(stop_msg)
        bus.send(disable_msg)
        sys.exit(0)

if __name__ == '__main__':
    main()

