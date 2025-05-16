# mcp9600_sensor.py

import board
import busio
from adafruit_mcp9600 import MCP9600

class MCP9600Sensor:
    def __init__(self, i2c=None):
        try:
            self.i2c = i2c or busio.I2C(board.SCL, board.SDA)
            self.sensor = MCP9600(self.i2c)
            print("[MCP9600] Thermocouple sensor initialized.")
        except Exception as e:
            print(f"[MCP9600] Initialization failed: {e}")
            self.sensor = None

    def read_temperature(self):
        try:
            if not self.sensor:
                raise RuntimeError("Sensor not initialized.")
            return self.sensor.temperature
        except Exception as e:
            print(f"[MCP9600] Temperature read failed: {e}")
            return None
