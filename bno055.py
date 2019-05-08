# bno055.py MicroPython driver for Bosch BNO055 nine degree of freedom inertial
# measurement unit module with sensor fusion.

# The MIT License (MIT)
#
# Copyright (c) 2017 Radomir Dopieralski for Adafruit Industries.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# This is a port of the Adafruit CircuitPython driver to MicroPython, with some
# modified/enhanced functionality.

# Original Author: Radomir Dopieralski
# Ported to MicroPython and extended by Peter Hinch
# This port copyright (c) Peter Hinch 2019

import utime as time
import ustruct
from micropython import const
try:
    from bno055_help import *
except ImportError:
    print('Not ', end='')
print('using bno055_help.py')

_CHIP_ID = const(0xa0)

_CONFIG_MODE = const(0)
_NDOF_MODE = const(0x0c)

_POWER_NORMAL = const(0x00)
_POWER_LOW = const(0x01)
_POWER_SUSPEND = const(0x02)

_MODE_REGISTER = const(0x3d)
_PAGE_REGISTER = const(0x07)
_CALIBRATION_REGISTER = const(0x35)
_TRIGGER_REGISTER = const(0x3f)
_POWER_REGISTER = const(0x3e)
_ID_REGISTER = const(0x00)
_AXIS_MAP_SIGN = const(0x42)
_AXIS_MAP_CONFIG = const(0x41)

# Convert two bytes to signed integer (little endian) Can be used in an interrupt handler
def bytes_toint(lsb, msb):
    if not msb & 0x80:
        return msb << 8 | lsb  # +ve
    return - (((msb ^ 255) << 8) | (lsb ^ 255) + 1)

# Transposition (x, y, z) 0 == x 1 == y 2 == z hence (0, 1, 2) is no change
# Scaling (x, y, z) 0 == normal 1 == invert
class BNO055:

    @staticmethod
    def argcheck(arg, name):
        if len(arg) != 3 or not (isinstance(arg, list) or isinstance(arg, tuple)):
            raise ValueError(name + ' must be a 3 element list or tuple')

    def __init__(self, i2c, address=0x28, crystal=True, transpose=(0, 1, 2), sign=(0, 0, 0)):
        self._i2c = i2c
        self.address = address
        self.crystal = crystal
        self.argcheck(sign, 'Sign')
        self.sign = sign
        self.argcheck(transpose, 'Transpose')
        if set(transpose) != {0, 1, 2}:
            raise ValueError('Transpose indices must be unique and in range 0-2')
        self.transpose = transpose
        self.buf6 = bytearray(6)
        self.buf8 = bytearray(8)
        self.w = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.mag = lambda : self.scaled_tuple(0x0e, 1/16)  # microteslas (x, y, z)
        self.accel = lambda : self.scaled_tuple(0x08, 1/100)  # m.s^-2
        self.lin_acc = lambda : self.scaled_tuple(0x28, 1/100)  # m.s^-2
        self.gravity = lambda : self.scaled_tuple(0x2e, 1/100)  # m.s^-2
        self.gyro = lambda : self.scaled_tuple(0x14, 1/16)  # deg.s^-1
        self.euler = lambda : self.scaled_tuple(0x1a, 1/16)  # degrees (heading, roll, pitch)
        self.quaternion = lambda : self.scaled_tuple(0x20, 1/(1<<14), bytearray(8), '<hhhh')  # (w, x, y, z)
        try:
            chip_id = self._read(_ID_REGISTER)
        except OSError:
            raise RuntimeError('No BNO055 chip detected.')
        if chip_id != _CHIP_ID:
            raise RuntimeError("bad chip id (%x != %x)" % (chip_id, _CHIP_ID))
        self.reset()

    def reset(self):
        self.mode(_CONFIG_MODE)
        try:
            self._write(_TRIGGER_REGISTER, 0x20)
        except OSError: # error due to the chip resetting
            pass
        # wait for the chip to reset (650 ms typ.)
        time.sleep_ms(700)
        self._write(_POWER_REGISTER, _POWER_NORMAL)
        self._write(_PAGE_REGISTER, 0x00)
        self._write(_TRIGGER_REGISTER, 0x80 if self.crystal else 0)
        time.sleep_ms(500 if self.crystal else 10)  # Crystal osc seems to take time to start.

        if self.transpose != (0, 1, 2):
            a = self.transpose
            self._write(_AXIS_MAP_CONFIG, (a[2] << 4) + (a[1] << 2) + a[0])
        if self.sign != (0, 0, 0):
            a = self.sign
            self._write(_AXIS_MAP_SIGN, a[2] + (a[1] << 1) + (a[0] << 2))
        self.mode(_NDOF_MODE)

    def scaled_tuple(self, addr, scale, buf=bytearray(6), fmt='<hhh'):
        return tuple(b*scale for b in ustruct.unpack(fmt, self._readn(buf, addr)))

    def temperature(self):
        t = self._read(0x34)  # Celcius signed (corrected from Adafruit)
        return t if t < 128 else t - 256

    # Return tuple containing sys, gyro, accel, and mag calibration data.
    def calibration_status(self):
        calibration_data = self._read(_CALIBRATION_REGISTER)
        sys = (calibration_data >> 6) & 0x03
        gyro = (calibration_data >> 4) & 0x03
        accel = (calibration_data >> 2) & 0x03
        mag = calibration_data & 0x03
        return sys, gyro, accel, mag

    def calibrated(self):
        return min(self.calibration_status()) == 3

    # read byte from register, return int
    def _read(self, memaddr, buf=bytearray(1)):  # memaddr = memory location within the I2C device
        self._i2c.readfrom_mem_into(self.address, memaddr, buf)
        return buf[0]

    # write byte to register
    def _write(self, memaddr, data, buf=bytearray(1)):
        buf[0] = data
        self._i2c.writeto_mem(self.address, memaddr, buf)

    # read n bytes, return buffer
    def _readn(self, buf, memaddr):  # memaddr = memory location within the I2C device
        self._i2c.readfrom_mem_into(self.address, memaddr, buf)
        return buf

    def mode(self, new_mode=None):
        old_mode = self._read(_MODE_REGISTER)
        if new_mode is not None:
            self._write(_MODE_REGISTER, _CONFIG_MODE)  # This is empirically necessary if the mode is to be changed
            time.sleep_ms(20)  # Datasheet table 3.6
            if new_mode != _CONFIG_MODE:
                self._write(_MODE_REGISTER, new_mode)
                time.sleep_ms(10)  # Table 3.6
        return old_mode

    def external_crystal(self):
        return bool(self._read(_TRIGGER_REGISTER) & 0x80)

    # Configuration: if a tuple is passed, convert to int using function from bno055_help.py
    def config(self, dev, value=None):
        if dev not in (ACC, MAG, GYRO):
            raise ValueError('Unknown device:', dev)
        if isinstance(value, tuple):
            value = tuple_to_int(dev, value)  # Convert tuple to register value
        last_mode = self.mode(_CONFIG_MODE)
        self._write(_PAGE_REGISTER, 1)
        old_val = self._read(dev)
        if value is not None:
            self._write(dev, value)
        self._write(_PAGE_REGISTER, 0)
        self.mode(last_mode)
        return old_val

    def iget(self, reg):
        if reg == 0x20:
            n = 4
            buf = self.buf8
        else:
            n = 3
            buf = self.buf6
        self._i2c.readfrom_mem_into(self.address, reg, buf)
        if n == 4:
            self.w = bytes_toint(buf[0], buf[1])
            i = 2
        else:
            self.w = 0
            i = 0
        self.x = bytes_toint(buf[i], buf[i+1])
        self.y = bytes_toint(buf[i+2], buf[i+3])
        self.z = bytes_toint(buf[i+4], buf[i+5])
