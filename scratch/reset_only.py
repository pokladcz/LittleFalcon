import serial
import time
import sys

port = 'COM8'
print(f"Resetting {port}...")
try:
    ser = serial.Serial(port, 115200)
    ser.setDTR(False)
    ser.setRTS(True)
    time.sleep(0.1)
    ser.setRTS(False)
    time.sleep(0.1)
    ser.close()
    print("Reset complete.")
except Exception as e:
    print(f"Error resetting: {e}")
    sys.exit(1)
