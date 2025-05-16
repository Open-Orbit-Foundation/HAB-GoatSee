# bmp280_sensor.py

import board
import busio
from adafruit_bmp280 import Adafruit_BMP280_I2C

class BMP280Sensor:
    def __init__(self, i2c=None, sea_level_pressure=1013.25):
        try:
            self.i2c = i2c or busio.I2C(board.SCL, board.SDA)
            self.sensor = Adafruit_BMP280_I2C(self.i2c)
            self.sensor.sea_level_pressure = sea_level_pressure
            print("[BMP280] Initialized successfully.")
        except Exception as e:
            print(f"[BMP280] Initialization failed: {e}")
            self.sensor = None

    def read_temperature(self):
        try:
            if not self.sensor:
                raise RuntimeError("Sensor not initialized.")
            return self.sensor.temperature
        except Exception as e:
            print(f"[BMP280] Temperature read failed: {e}")
            return None

    def read_pressure(self):
        try:
            if not self.sensor:
                raise RuntimeError("Sensor not initialized.")
            return self.sensor.pressure
        except Exception as e:
            print(f"[BMP280] Pressure read failed: {e}")
            return None

    def read_altitude(self):
        try:
            if not self.sensor:
                raise RuntimeError("Sensor not initialized.")
            return self.sensor.altitude
        except Exception as e:
            print(f"[BMP280] Altitude read failed: {e}")
            return None
