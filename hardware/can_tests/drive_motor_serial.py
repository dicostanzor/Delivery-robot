import serial
import time
import sys

def main():
    port = '/dev/ttyACM0'
    print(f"Connecting to CANable Pro serial interface on {port}...")
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return

    # Initialize CANable Pro: Clear, Set Speed to 1Mbps, Open
    ser.write(b'C\r')       
    time.sleep(0.05)
    ser.write(b'S8\r')      
    time.sleep(0.05)
    ser.write(b'O\r')       
    time.sleep(0.05)
    
    print("CANable Pro initialized at 1 Mbps.")
    print("Injecting FRC Security Sync Arrays... STAND CLEAR OF THE MOTOR!")
    print("Press Ctrl+C to terminate motor power instantly.")

    # 1. Primary FRC Enable Heartbeat Block (System State: ENABLED)
    enable_cmd1 = b'T000401BF80100050000000000\r'
    
    # 2. Secondary FRC System Token Frame (System Control Acknowledged)
    enable_cmd2 = b'T0004018080100050000000000\r'

    try:
        while True:
            # Fire both mandatory safety unlocks into the serial bus stream
            ser.write(enable_cmd1)
            time.sleep(0.002)
            ser.write(enable_cmd2)
            time.sleep(0.002)
            
            # Sweep through common Device IDs (0 to 16)
            for device_id in range(17):
                base_id = 0x02040000 + device_id
                
                # Command 50% Forward Power (0x03FF is max power, using 0x01FF for ~50%)
                # This clears the strict 4% factory neutral deadband limit
                control_payload = "0001FF0000000000"
                control_cmd = f"T{base_id:08X}8{control_payload}\r".encode()
                
                ser.write(control_cmd)

            # Maintain strict 20ms safety loop cadence
            time.sleep(0.015)

    except KeyboardInterrupt:
        print("\nSafety Intercept triggered. Disabling system safely...")
        # Clear out all channels on exit to neutralize power instantly
        for device_id in range(17):
            ser.write(f"T{(0x02040000 + device_id):08X}80000000000000000\r".encode())
            
        ser.write(b'T000401BF80000000000000000\r')
        time.sleep(0.05)
        
        ser.write(b'C\r') 
        ser.close()
        sys.exit(0)

if __name__ == '__main__':
    main()

