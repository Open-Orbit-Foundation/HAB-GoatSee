# bno08x_sensor.py

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

class BNO08xSensor:
    def __init__(self, i2c=None):
        try:
            self.i2c = i2c or busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(self.i2c)

            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

            print("[BNO08x] IMU initialized with all sensor features.")
        except Exception as e:
            print(f"[BNO08x] Initialization failed: {e}")
            self.bno = None

    def _safe_read(self, prop, length):
        try:
            data = prop  # ← fixed typo: use 'prop' instead of 'method'
            if len(data) != length:
                raise ValueError("Malformed IMU data")
            return data
        except KeyError as e:
            print(f"[BNO08x] Unknown report ID: {e}")
            return (None,) * length
        except Exception as e:
            print(f"[BNO08x] Read error: {e}")
            return (None,) * length

    def read_accel(self):
        return self._safe_read(self.bno.acceleration, 3) if self.bno else (None, None, None)

    def read_gyro(self):
        return self._safe_read(self.bno.gyro, 3) if self.bno else (None, None, None)

    def read_mag(self):
        return self._safe_read(self.bno.magnetic, 3) if self.bno else (None, None, None)

    def read_quat(self):
        return self._safe_read(self.bno.quaternion, 4) if self.bno else (None, None, None, None)
