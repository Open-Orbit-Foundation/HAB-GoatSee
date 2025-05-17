import serial
import time

port = "/dev/ttyAMA0"
baudrate = 9600
timeout = 1

try:
    ser = serial.Serial(port, baudrate, timeout = timeout)
    print(f"Connected to serial port: {port} at {baudrate} baud.")

    while True:
        try:
            line = ser.readline()
            if line:
                print(line.decode('utf-8').strip())
            else:
                print("test")

        except serial.SerialTimeoutException:
            print("Serial timeout. No data received within timeout period.")
        except Exception as e:
            print(f"Error reading serial data: {e}")
            break
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")