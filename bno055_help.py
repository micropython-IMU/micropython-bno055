# bno055_help.py Helper functions and values for bno055 driver
# Released under the MIT licence.
# Copyright (c) Peter Hinch 2019

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

_acc_range = (2, 4, 8, 16)  # G
_acc_bw = (8, 16, 31, 62, 125, 250, 500, 1000)
_gyro_range = (2000, 1000, 500, 250, 125)  # dps
_gyro_bw = (523, 230, 116, 47, 23, 12, 64, 32)  # bandwidth (Hz)
_mag_rate = (2, 6, 8, 10, 15, 20, 25, 30)  # rate (Hz)

def tuple_to_int(dev, v):  # Convert (range, bw) to register value
    try:
        if dev == ACC:
            msg = 'Illegal accel range {} or bandwidth {}'
            return _acc_range.index(v[0]) | (_acc_bw.index(v[1]) << 2)
        elif dev == GYRO:
            msg = 'Illegal gyro range {} or bandwidth {}'
            return _gyro_range.index(v[0]) | (_gyro_bw.index(v[1]) << 3)
        elif dev == MAG:
            msg = 'Illegal magnetometer rate {}'
            return _mag_rate.index(v[0])
    except ValueError:
        raise ValueError(msg.format(*v))

def get_tuple(dev, v):
    if dev == ACC:
        return (_acc_range[v & 3], _acc_bw[v >> 2])
    elif dev == GYRO:
        return (_gyro_range[v & 7], _gyro_bw[v >> 3])
    elif dev == MAG:
        return (_mag_rate[v],)
    raise ValueError('Unknown device.', dev)
