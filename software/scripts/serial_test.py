import serial
import time

# Open the serial port
ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)

# Give the board time to initialize
time.sleep(2)

# Example: send a command (depends on your firmware)
ser.write(b'Hello Motor2040!\n')

# Read response (if any)
response = ser.readline().decode().strip()
print("Response:", response)

ser.close()
