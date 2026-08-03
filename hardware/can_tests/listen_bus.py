import serial
import time
import sys

def main():
    port = '/dev/ttyACM0'
    print(f"Opening CANable Pro tracker on {port}...")
    
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except Exception as e:
        print(f"Failed to open port: {e}")
        return

    # Initialize and open the channel
    ser.write(b'C\r')       
    time.sleep(0.05)
    ser.write(b'S8\r')      # 1 Mbps speed mode
    time.sleep(0.05)
    ser.write(b'O\r')       # Open channel
    time.sleep(0.05)
    
    # FORCE AUTO-POLL MODE ON
    # This tells the CANable to actively forward all bus data over serial
    ser.write(b'P\r')       
    time.sleep(0.05)
    
    print("Listening for passive Talon SRX frames. Press Ctrl+C to exit.")
    
    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                # Print any line starting with T (Extended frame) or t (Standard frame)
                if line.startswith('T') or line.startswith('t'):
                    print(f"Live Bus Packet: {line}")
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print("\nStopping tracker.")
        ser.write(b'C\r')
        ser.close()
        sys.exit(0)

if __name__ == '__main__':
    main()


