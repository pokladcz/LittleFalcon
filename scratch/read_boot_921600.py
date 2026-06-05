import serial
import time
import sys

port = 'COM8'
baud = 921600

print(f"Opening {port} at {baud}...")
try:
    ser = serial.Serial(port, baud, timeout=0.1)
    
    # Toggle RTS/DTR to reset the ESP32
    ser.setDTR(False)
    ser.setRTS(True)
    time.sleep(0.1)
    ser.setRTS(False)
    time.sleep(0.1)
    
    print("Reading boot logs for 5 seconds...")
    end_time = time.time() + 5
    while time.time() < end_time:
        line = ser.readline()
        if line:
            sys.stdout.buffer.write(line)
            sys.stdout.buffer.flush()
            
    ser.close()
    print("\nClosed port.")
except Exception as e:
    print(f"Error: {e}")
