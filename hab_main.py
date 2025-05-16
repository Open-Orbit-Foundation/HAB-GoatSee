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
