# bno08x_sensor.py
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

class BNO08xSensor:
    def __init__(self, i2c_bus=1):
        self.i2c = I2C(i2c_bus)
        self.bno = BNO08X_I2C(self.i2c)

        # Enable only features needed
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        print("[BNO08x] IMU initialized with all sensor features enabled.")

    def _safe_read(self, method, length):
        try:
            data = method()
            if len(data) != length:
                raise ValueError("Malformed IMU data")
            return data
        except KeyError as e:
            print(f"[BNO08x] Unknown report ID (often harmless): {e}")
            return (None,) * length
        except Exception as e:
            print(f"[BNO08x] Sensor read error: {e}")
            return (None,) * length

    def read_accel(self):
        return self._safe_read(self.bno.acceleration, 3)

    def read_gyro(self):
        return self._safe_read(self.bno.gyro, 3)

    def read_mag(self):
        return self._safe_read(self.bno.magnetic, 3)

    def read_quat(self):
        return self._safe_read(self.bno.quaternion, 4)
