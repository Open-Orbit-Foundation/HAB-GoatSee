# bosch_pressure_sensor.py

import board
import busio
from adafruit_bus_device.i2c_device import I2CDevice

class BoschPressureSensor:
    def __init__(self, i2c=None, address=0x28, p_min=-15, p_max=15):
        self.i2c = i2c or busio.I2C(board.SCL, board.SDA)
        self.addr = address
        self.device = I2CDevice(self.i2c, self.addr)

        # Constants for conversion
        self.output_min = 1638  # 10% of 2^14
        self.output_max = 14745 # 90% of 2^14
        self.p_min = p_min
        self.p_max = p_max

        print("[Bosch] SSCDANN015PA2A3 initialized at I2C address 0x28.")

    def read_data(self):
        try:
            buf = bytearray(4)
            with self.device:
                self.device.readinto(buf)

            status = (buf[0] & 0xC0) >> 6
            raw_pressure = ((buf[0] & 0x3F) << 8) | buf[1]
            raw_temp = (buf[2] << 3) | (buf[3] >> 5)

            # Status bits (S1, S0)
            if status == 0b11:
                raise RuntimeError("Sensor diagnostic fault")
            elif status == 0b10:
                print("[Bosch] Warning: Stale data")
            elif status == 0b01:
                print("[Bosch] Warning: Command mode active")

            pressure = self._convert_pressure(raw_pressure)
            temperature = self._convert_temperature(raw_temp)

            return pressure, temperature

        except Exception as e:
            print(f"[Bosch] Read failed: {e}")
            return None, None

    def _convert_pressure(self, output):
        span = self.output_max - self.output_min
        pressure = self.p_min + ((output - self.output_min) * (self.p_max - self.p_min)) / span
        return round(pressure, 2)

    def _convert_temperature(self, raw_temp):
        # Equation from datasheet:
        # T(C) = 200 * (raw / 2047) - 50
        temp_c = 200 * (raw_temp / 2047) - 50
        return round(temp_c, 2)
