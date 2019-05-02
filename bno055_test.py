import machine
import time
from bno055 import BNO055

i2c = machine.I2C(1)
imu = BNO055(i2c, transpose=(1, 0, 2), sign=(0, 0, 1))
time.sleep(1)
print(imu.temperature())
print(imu.mag())
print(imu.gyro())
print(imu.accel())
