# bno055.py MicroPython driver for Bosch cls nine degree of freedom inertial
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

# This is a port of the Adafruit CircuitPython driver to MicroPython, with
# modified/enhanced functionality.

# Original Author: Radomir Dopieralski
# Ported to MicroPython and extended by Peter Hinch
# This port copyright (c) Peter Hinch 2019

from micropython import const
from bno055_base import BNO055_BASE


CONFIG_MODE = 0x00
ACCONLY_MODE = 0x01
MAGONLY_MODE = 0x02
GYRONLY_MODE = 0x03
ACCMAG_MODE = 0x04
ACCGYRO_MODE = 0x05
MAGGYRO_MODE = 0x06
AMG_MODE = 0x07
IMUPLUS_MODE = 0x08
COMPASS_MODE = 0x09
M4G_MODE = 0x0a
NDOF_FMC_OFF_MODE = 0x0b
NDOF_MODE = 0x0c

ACC = 0x08  # Registers for configuration (page 1)
MAG = 0x09
GYRO = 0x0a

ACC_DATA = 0x08  # Data regsiters (page 0)
MAG_DATA = 0x0e
GYRO_DATA = 0x14
GRAV_DATA = 0x2e
LIN_ACC_DATA = 0x28
EULER_DATA = 0x1a
QUAT_DATA = 0x20

_PAGE_REGISTER = const(0x07)
_AXIS_MAP_SIGN = const(0x42)
_AXIS_MAP_CONFIG = const(0x41)

class BNO055(BNO055_BASE):

    acc_range = (2, 4, 8, 16)  # G
    acc_bw = (8, 16, 31, 62, 125, 250, 500, 1000)
    gyro_range = (2000, 1000, 500, 250, 125)  # dps
    gyro_bw = (523, 230, 116, 47, 23, 12, 64, 32)  # bandwidth (Hz)
    mag_rate = (2, 6, 8, 10, 15, 20, 25, 30)  # rate (Hz)

    @classmethod
    def _tuple_to_int(cls, dev, v):  # Convert (range, bw) to register value
        try:
            if dev == ACC:
                msg = 'Illegal accel range {} or bandwidth {}'
                return cls.acc_range.index(v[0]) | (cls.acc_bw.index(v[1]) << 2)
            elif dev == GYRO:
                msg = 'Illegal gyro range {} or bandwidth {}'
                return cls.gyro_range.index(v[0]) | (cls.gyro_bw.index(v[1]) << 3)
            elif dev == MAG:
                msg = 'Illegal magnetometer rate {}'
                return cls.mag_rate.index(v[0])
        except ValueError:
            raise ValueError(msg.format(*v))

    # Return the current config in human readable form
    @classmethod
    def _int_to_tuple(cls, dev, v):
        try:
            if dev == ACC:
                return (cls.acc_range[v & 3], cls.acc_bw[v >> 2])
            elif dev == GYRO:
                return (cls.gyro_range[v & 7], cls.gyro_bw[v >> 3])
            elif dev == MAG:
                return (cls.mag_rate[v],)
        except IndexError:
            return False  # Can occur e.g. initial config of magnetometer
        raise ValueError('Unknown device.', dev)

    # Convert two bytes to signed integer (little endian) Can be used in an interrupt handler
    @staticmethod
    def _bytes_toint(lsb, msb):
        if not msb & 0x80:
            return msb << 8 | lsb  # +ve
        return - (((msb ^ 255) << 8) | (lsb ^ 255) + 1)

    @staticmethod
    def _argcheck(arg, name):
        if len(arg) != 3 or not (isinstance(arg, list) or isinstance(arg, tuple)):
            raise ValueError(name + ' must be a 3 element list or tuple')

    # Transposition (x, y, z) 0 == x 1 == y 2 == z hence (0, 1, 2) is no change
    # Scaling (x, y, z) 0 == normal 1 == invert
    def __init__(self, i2c, address=0x28, crystal=True, transpose=(0, 1, 2), sign=(0, 0, 0)):
        self._argcheck(sign, 'Sign')
        if [x for x in sign if x not in (0, 1)]:
            raise ValueError('Sign values must be 0 or 1')
        self.sign = sign
        self._argcheck(transpose, 'Transpose')
        if set(transpose) != {0, 1, 2}:
            raise ValueError('Transpose indices must be unique and in range 0-2')
        self.transpose = transpose
        super().__init__(i2c, address, crystal)
        self.buf6 = bytearray(6)
        self.buf8 = bytearray(8)
        self.w = 0
        self.x = 0
        self.y = 0
        self.z = 0

    def orient(self):
        if self.transpose != (0, 1, 2):
            a = self.transpose
            self._write(_AXIS_MAP_CONFIG, (a[2] << 4) + (a[1] << 2) + a[0])
        if self.sign != (0, 0, 0):
            a = self.sign
            self._write(_AXIS_MAP_SIGN, a[2] + (a[1] << 1) + (a[0] << 2))

    # Configuration: if a tuple is passed, convert to int using function from bno055_help.py
    def config(self, dev, value=None):
        if dev not in (ACC, MAG, GYRO):
            raise ValueError('Unknown device:', dev)
        if isinstance(value, tuple):
            value = self._tuple_to_int(dev, value)  # Convert tuple to register value
        elif value is not None:
            raise ValueError('value must be a tuple or None.')
        last_mode = self.mode(CONFIG_MODE)
        self._write(_PAGE_REGISTER, 1)
        old_val = self._read(dev)
        if value is not None:
            self._write(dev, value)
        self._write(_PAGE_REGISTER, 0)
        self.mode(last_mode)
        return self._int_to_tuple(dev, old_val)

    # For use in ISR
    def iget(self, reg):
        if reg == 0x20:
            n = 4
            buf = self.buf8
        else:
            n = 3
            buf = self.buf6
        self._i2c.readfrom_mem_into(self.address, reg, buf)
        if n == 4:
            self.w = self._bytes_toint(buf[0], buf[1])
            i = 2
        else:
            self.w = 0
            i = 0
        self.x = self._bytes_toint(buf[i], buf[i+1])
        self.y = self._bytes_toint(buf[i+2], buf[i+3])
        self.z = self._bytes_toint(buf[i+4], buf[i+5])
