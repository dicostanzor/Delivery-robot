import serial
import time
import sys

def main():
    port = '/dev/ttyACM0'
    print(f"Opening CANable Pro serial link for a live Device ID sweep...")
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return

    # Initialize CANable Pro
    ser.write(b'C\r')       
    time.sleep(0.05)
    ser.write(b'S8\r')      
    time.sleep(0.05)
    ser.write(b'O\r')       
    time.sleep(0.05)

    # Master FRC Enable Heartbeat Frame
    enable_cmd = b'T000401BF80100050000000000\r'

    print("Sweeping Device IDs 0 through 15. WATCH THE TALON'S LED LIGHTS CLOSELY!")
    
    try:
        while True:
            # Broadcast master unlock
            ser.write(enable_cmd)
            time.sleep(0.002)

            # Rapidly test IDs 0 to 15
            for device_id in range(16):
                # Calculate the 29-bit Extended CAN ID for the target device
                base_id = 0x02040000 + device_id
                # Convert to an 8-character hex string padded with zeros
                hex_id = f"{base_id:08X}"
                
                # Command ~30% forward output drive
                cmd = f"T{hex_id}8002E600000000000\r".encode()
                ser.write(cmd)
                
            time.sleep(0.015) # Maintain 20ms cadence

    except KeyboardInterrupt:
        print("\nScan terminated safely.")
        ser.write(b'C\r')
        ser.close()
        sys.exit(0)

if __name__ == '__main__':
    main()

