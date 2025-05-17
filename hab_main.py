# hab_main.py
# High-Altitude Balloon Main Script

# === General Purpose ===
import os
import sys
import csv
import time
import threading
from datetime import datetime

# === Raspberry Pi Hardware ===
import board
import busio
import RPi.GPIO as GPIO

# === Camera ===
from picamera2 import Picamera2

# === Serial / GPS ===
import serial

# === Sensor Modules ===
from bno08x_sensor import BNO08xSensor
from mcp9600_sensor import MCP9600Sensor
from bmp280_sensor import BMP280Sensor
from bosch_pressure_sensor import BoschPressureSensor



def init_sensors():
    print("[Init] Starting sensor initialization...")
    sensors = {}

    # === Shared I2C Bus ===
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        while not i2c.try_lock():
            time.sleep(0.01)
        i2c.unlock()
        print("[Init] I2C bus ready.")
    except Exception as e:
        print(f"[Init] Failed to initialize I2C bus: {e}")
        return {}

    # === MCP9600 Thermocouple ===
    sensors["mcp"] = MCP9600Sensor(i2c)

    # === BMP280 Barometric Sensor ===
    sensors["bmp280"] = BMP280Sensor(i2c)

    # === Bosch Pressure Sensor (SSCDANN015PA2A3) ===
    sensors["bosch"] = BoschPressureSensor(i2c)

    # === BNO08x IMU (on separate Extended I2C bus 1) ===
    try:
        sensors["bno"] = BNO08xSensor(i2c_bus=1)
    except Exception as e:
        print(f"[BNO08x] IMU failed to initialize: {e}")
        sensors["bno"] = None

    print("[Init] Sensor initialization complete.")
    return sensors

def sensor_logger(sensors):
    LOG_DIR = "flight_logs"
    CSV_DURATION = 600  # seconds = 10 minutes
    os.makedirs(LOG_DIR, exist_ok=True)

    def timestamp():
        return datetime.now().strftime("%Y%m%d_%H%M%S")

    def write_header(writer):
        writer.writerow([
            "timestamp",
            "mcp_temp", "bmp_temp", "bmp_pressure",
            "bosch_pressure", "bosch_temp",
            "accel_x", "accel_y", "accel_z",
            "gyro_x", "gyro_y", "gyro_z",
            "mag_x", "mag_y", "mag_z",
            "quat_i", "quat_j", "quat_k", "quat_real"
        ])

    fname = os.path.join(LOG_DIR, f"sensors_{timestamp()}.csv")
    f = open(fname, "w", newline="")
    writer = csv.writer(f)
    write_header(writer)

    start_time = time.time()
    next_split = start_time + CSV_DURATION

    while True:
        now = time.time()

        # Rotate file if needed
        if now >= next_split:
            f.close()
            fname = os.path.join(LOG_DIR, f"sensors_{timestamp()}.csv")
            f = open(fname, "w", newline="")
            writer = csv.writer(f)
            write_header(writer)
            next_split = now + CSV_DURATION

        row = [datetime.now().isoformat()]

        # === MCP9600 ===
        try:
            row.append(sensors["mcp"].read_temperature())
        except:
            row.append(None)

        # === BMP280 ===
        try:
            row.append(sensors["bmp280"].read_temperature())
            row.append(sensors["bmp280"].read_pressure())
        except:
            row += [None, None]

        # === Bosch Sensor ===
        try:
            pressure, temp = sensors["bosch"].read_data()
            row.append(pressure)
            row.append(temp)
        except:
            row += [None, None]

        # === BNO08x IMU ===
        try:
            imu = sensors["bno"]
            row += list(imu.read_accel())
            row += list(imu.read_gyro())
            row += list(imu.read_mag())
            row += list(imu.read_quat())
        except:
            row += [None] * (3 + 3 + 3 + 4)

        writer.writerow(row)
        f.flush()
        time.sleep(1)

def camera_thread():
    VIDEO_DIR = "flight_video"
    VIDEO_DURATION = 600  # seconds = 10 minutes
    os.makedirs(VIDEO_DIR, exist_ok=True)

    def timestamp():
        return datetime.now().strftime("%Y%m%d_%H%M%S")

    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={"size": (640, 480)}))
    cam.start()
    time.sleep(2)  # Warm-up time

    while True:
        try:
            fname = os.path.join(VIDEO_DIR, f"video_{timestamp()}.h264")
            print(f"[Camera] Recording: {fname}")
            cam.start_recording(output=fname)
            time.sleep(VIDEO_DURATION)
            cam.stop_recording()
        except Exception as e:
            print(f"[Camera] Error: {e}")
            time.sleep(5)  # Delay before retry

def gps_logger():
    GPS_LOG_DIR = "gps_logs"
    GPS_PORT = "/dev/ttyAMA0"  # Or "/dev/serial0"
    GPS_BAUD = 9600
    os.makedirs(GPS_LOG_DIR, exist_ok=True)

    def timestamp():
        return datetime.now().strftime("%Y%m%d_%H%M%S")

    def line_time():
        return datetime.now().isoformat()

    try:
        port = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1)
        print("[GPS] Serial port opened.")
    except Exception as e:
        print(f"[GPS] Failed to open port: {e}")
        return

    fname = os.path.join(GPS_LOG_DIR, f"gps_{timestamp()}.txt")
    f = open(fname, "a")
    next_split = time.time() + 600

    while True:
        try:
            if port.in_waiting > 0:
                line = port.readline().decode("utf-8", errors="replace").strip()

                if time.time() >= next_split:
                    f.close()
                    fname = os.path.join(GPS_LOG_DIR, f"gps_{timestamp()}.txt")
                    f = open(fname, "a")
                    next_split = time.time() + 600

                f.write(f"{line_time()},{line}\n")
                f.flush()
        except Exception as e:
            print(f"[GPS] Error: {e}")
            time.sleep(1)

def shutdown_monitor(timeout_sec=10800):
    DISABLE_PIN = 21  # BCM numbering (GPIO21 = pin 40)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DISABLE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    print(f"[Shutdown] Timer armed for {timeout_sec} seconds.")
    remaining = timeout_sec
    last_check = time.time()

    while True:
        now = time.time()
        dt = now - last_check
        last_check = now

        if GPIO.input(DISABLE_PIN):  # Pulled HIGH (inactive)
            remaining -= dt
            if remaining <= 0:
                print("[Shutdown] Timer expired. Initiating shutdown.")
                os.system("sudo shutdown -h now")
                break
        else:  # Jumper pulled LOW
            remaining = timeout_sec  # Reset the countdown

        time.sleep(1)


def main():
    print("[Main] Booting High-Altitude Flight Stack...")

    # === Initialize Sensors ===
    sensors = init_sensors()

    # === Start Threads ===
    threads = []

    t_logger = threading.Thread(target=sensor_logger, args=(sensors,), daemon=True)
    threads.append(t_logger)

    t_camera = threading.Thread(target=camera_thread, daemon=True)
    threads.append(t_camera)

    t_gps = threading.Thread(target=gps_logger, daemon=True)
    threads.append(t_gps)

    t_shutdown = threading.Thread(target=shutdown_monitor, args=(10800,), daemon=True)
    threads.append(t_shutdown)

    print("[Main] Threads initialized. Starting all systems...")

    for t in threads:
        t.start()

    print("[Main] Flight system running. Ctrl+C to abort (simulated ground mode).")

    # === Keep Alive Until Interrupted ===
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[Main] Interrupted. Gracefully exiting...")
