import serial
import time
import sys

port = 'COM17'
baud = 921600

print(f"Opening {port} at {baud} (no reset)...")
try:
    # Open at 921600 baud without toggling reset pins
    ser = serial.Serial(port, baud, timeout=0.1)
    
    print("Monitoring serial output. Please press the START button (IO2) on the robot now...")
    
    end_time = time.time() + 45
    while time.time() < end_time:
        line = ser.readline()
        if line:
            sys.stdout.buffer.write(line)
            sys.stdout.buffer.flush()
            
    ser.close()
    print("\nClosed port.")
except Exception as e:
    print(f"Error: {e}")
