import serial
import time

GPS_PORT = "/dev/ttyAMA0"  # Or "/dev/serial0" depending on your Pi model/config
GPS_BAUD = 9600

try:
    gps = serial.Serial(GPS_PORT, GPS_BAUD, timeout=3)
    print(f"[GPS] Opened {GPS_PORT} at {GPS_BAUD} baud.")


except Exception as e:
    print(f"[GPS] Failed to open port: {e}")
    exit(1)

print("[GPS] Reading NMEA sentences... Press Ctrl+C to stop.")

try:
    while True:
        print(gps.in_waiting)
        if gps.in_waiting > 0:
            line = gps.readline().decode("utf-8", errors="replace").strip()
            print(line)
        
        else:
            time.sleep(0.1)

except KeyboardInterrupt:
    print("\n[GPS] Test stopped by user.")
except Exception as e:
    print(f"[GPS] Error while reading: {e}")
finally:
    gps.close()
    print("[GPS] Port closed.")
