# micropython-bno055

A MicroPython driver for the Bosch BNO055 inertial measurement unit (IMU). This
chip has the advantage of performing sensor fusion in hardware. The driver is
based on the [Adafruit CircuitPython driver](https://github.com/adafruit/Adafruit_CircuitPython_BNO055.git).

This driver has the following objectives:
 * It runs under official MicroPython.
 * Cross-platform: designed to run on Pyboard 1.x, Pyboard D, ESPx. Should run
 on any hardware which supports the `machine` module and the I2C interface.
 * Supports vehicle-relative coordinate transformation.
 * Supports changing the hardware configuration.
 * Supports access in interrupt service routines.
 * Uses the MicroPython approach to coding.

Testing was done with the [Adafruit BNO055 breakout](https://www.adafruit.com/product/2472).

# Contents

 1. [Files and dependencies](./README.md#1-files-and-dependencies)  
 2. [Getting started](./README.MD#2-getting-started)  
 3. [The BNO055 class](./README.md#3-the-bno055-class)  
  3.1 [Constructor](./README.md#31-constructor)  
  3.2 [Read only methods](./README.md#32-read-only-methods) Read data from device.  
  3.3 [Changing the device configuration](./README.md#33-changing-the-device-configuration)  
   3.3.1 [Mode setting](./README.md#331-mode-setting) Modify device operating mode.  
   3.3.2 [Rate and range control](./README.md#332-rate-and-range-control) Further settings.  
  3.4 [Use in interrupt handlers](./README.md#34-use-in-interrupt-handlers)  
 4. [Calibration](./README.md#4-calibration)  
 5. [References](./README.md#5-references)  

# 1. Files and dependencies

 * `bno055.py` Device driver.
 * `bno055_help.py` Optional helper module.
 * `bno055_test.py` Simple test program.

The driver has no dependencies, but will use the helper module if present. The
helper module provides functions and constants for users wishing to change the
configuration or operating mode of the hardware. The use of a separate module
minimises code size for those using the default `NDOF` mode. To access the
functions and values of the helper module it is recommended to issue
```python
from bno055_help import *
```

# 2. Getting started

The Adafruit breakout board has a voltage regulator and may be powered from a
5V or 3.3V supply. Note that other hardware may require a 3.3V source.

The wiring below is for I2C(1) as used in the test program.

| pyboard | bno055 |
|:-------:|:------:|
| VIN     | VIN    |
| GND     | GND    |
| SCL X9  | SCL    |
| SDA X10 | SDA    |

Basic usage is as follows:

```python
import machine
import time
from bno055 import BNO055

i2c = machine.I2C(1)
imu = BNO055(i2c)  # For hardware with a crystal (e.g. Adafruit)
# imu =BNO055(i2c, crystal=False)
while True:
    time.sleep(1)
    if not imu.calibrated():
        print('Calibration required: sys {} gyro {} accel {} mag {}'.format(*imu.calibration_status()))
    print('Temperature {}°C'.format(imu.temperature()))
    print('Mag       x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.mag()))
    print('Gyro      x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.gyro()))
    print('Accel     x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.accel()))
    print('Lin acc.  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.lin_acc()))
    print('Gravity   x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.gravity()))
    print('Heading     {:4.0f} roll {:4.0f} pitch {:4.0f}'.format(*imu.euler()))
```
To calibrate the chip move the unit as per [section 4](./README.md#4-calibration)
until all calibration values are 3.

###### [Contents](./README.md#contents)

# 3. The BNO055 class

## 3.1 Constructor

This takes the following args

 * `i2c` An initialised I2C instance.
 * `address=0x28` Default device address. The Adafruit breakout allows this to
 be changed to 0x29 for the case where two devices are attached to one bus.
 * `crystal=True` Use an external crystal for higher accuracy if one is
 provided. The Adafruit board has a crystal.

The following optional args provide for vehicle relative coordinates. The
default values assume that the IMU is mounted component side up and in the
horizontal plane. Alternatives cater for cases where the IMU is rotated or
mounted orthogonally relative to the horizontal plane of the vehicle.
 * `transpose=(0, 1, 2)`
 * `sign=(0, 0, 0)`

Axes are numbered 0 for X, 1 for Y and 2 for Z, and the transpose tuple is
`(x, y, z)`. Hence `(0, 1, 2)` implies no transformation. Passing `(1, 0, 2)`
implies a rotation around the Z axis.

Sign values must be 0 (normal) or 1 (inverted). Hence a board mounted upside
down would have `sign=(0, 0, 1)` (Z axis inverted).

The constructor blocks for 700ms.

## 3.2 Read only methods

Return values:
 * `mag()` Magnetometer vector `(x, y, z)` in μT (microtesla).
 * `accel()` Accelerometer vector `(x, y, z)` in m.s^-2
 * `lin_acc()` Acceleration `(x, y, z)` after removal of gravity component
 (m.s^-2).
 * `gravity()` Gravity vector `(x, y, z)` in m.s^-2 after removal of
 acceleration data.
 * `gyro()` Gyro vector `(x, y, z)` in deg.s^-1.
 * `euler()` Euler angles in degrees `(heading, roll, pitch)`.
 * `quaternion()` Quaternion `(w, x, y, z)`.
 * `temperature()` Chip temperature as an integer °C (Celcius).
 * `calibrated()` `True` if all elements of the device are calibrated.
 * `calibration_status()` Returns `(sys, gyro, accel, mag)`. Each element has a
 value of from 0 (uncalibrated) to 3 (fully calibrated).
 * `external_crystal()` `True` if using an external crystal.

## 3.3 Changing the device configuration

Many applications will use the default mode of the chip. This section describes
ways of changing this for special purposes, for example where a high update
rate is required because the readings are used in a feedback loop where latency
presents stability issues.

Such applications can access additional functions and constants by issuing
```python
from bno055_help import *
```

### 3.3.1 Mode setting

 * `mode(new_mode=None)` If a value is passed, change the mode of operation.
 Returns the mode as it was before any change was made.

The mode of operation defines which sensors are enabled, whether fusion occurs
and if measurements are absolute or relative. The `bno055_help` module provides
the mode values listed below as integers.

| Mode             | Accel | Compass | Gyro | Absolute | Fusion mode |
|:----------------:|:-----:|:-------:|:----:|:--------:|:-----------:|
| CONFIG_MODE      |   -   |   -     |  -   |     -    |   N         |
| ACCONLY_MODE     |   X   |   -     |  -   |     -    |   N         |
| MAGONLY_MODE     |   -   |   X     |  -   |     -    |   N         |
| GYRONLY_MODE     |   -   |   -     |  X   |     -    |   N         |
| ACCMAG_MODE      |   X   |   X     |  -   |     -    |   N         |
| ACCGYRO_MODE     |   X   |   -     |  X   |     -    |   N         |
| MAGGYRO_MODE     |   -   |   X     |  X   |     -    |   N         |
| AMG_MODE         |   X   |   X     |  X   |     -    |   N         |
| IMUPLUS_MODE     |   X   |   -     |  X   |     -    |   Y         |
| COMPASS_MODE     |   X   |   X     |  -   |     X    |   Y         |
| M4G_MODE         |   X   |   X     |  -   |     -    |   Y         |
| NDOF_FMC_OFF_MODE|   X   |   X     |  X   |     X    |   Y         |
| NDOF_MODE        |   X   |   X     |  X   |     X    |   Y         |

The default mode is `NDOF_MODE` which supports fusion with absolute heading.
Example usage (`imu` is a `BNO055` instance):
```python
from bno055_help import *
# code omitted
old_mode = imu.mode(ACCGYRO_MODE)
# code omitted
imu.mode(old_mode)
```

### 3.3.2 Rate and range control

In non-fusion modes the chip allows control of the update rate of each sensor
and the range of the accelerometer and gyro. In fusion modes rates are fixed:
the only available change is to the accelerometer range.

| Device | Full scale  | Update rate |
|:------:|:-----------:|:-----------:| Datasheet table 3.14
| Accel  | +-4G        | 100Hz       |
| Gyro   | 2000°/s     | 100Hz       |
| Mag    | -           | 20Hz        |

The magnetometer has a single range: units are Micro Tesla (μT).

In non-fusion modes the sensors may be controlled with the following method.

 * `config(dev, value=None)` `dev` is the device: must be `ACC`, `GYRO` or
 `MAG` (constants defined in bno055_help.py). The `value` arg may be an integer
 holding the raw value for the configuration register or a tuple. The tuple
 contains human readable values which are converted to a register value.

The default value of `None` causes no change to be made. In each case the
method returns the raw register value as it was before any change was made:
this allows old values to be restored, e.g.  
```python
old_config = imu.config(ACC, (2, 250))
# ...
imu.config(ACC, old_config)  # Restore old config
```
Note that the hardware will only allow configuration changes in appropriate
modes. For example to change the accelerometer config successfully, the chip
must be in a non-fusion mode which enables the accelerometer.

#### Accelerometer (dev == ACC)

`value` is normally a 2-tuple comprising `(range, bandwidth)`
Allowable values:  
Range: 2, 4, 8, 16 (G).  
Bandwidth: 8, 16, 31, 62, 125, 250, 500, 1000 (Hz).  
The return value may be restored to human-readable form by means of the
`get_tuple` function:
```python
from bno055_help import *
# code omitted
imu.mode(ACCONLY_MODE)  # non-fusion mode
print(get_tuple(ACC, imu.config(ACC, (2, 1000))))  # Update. Display prior value.
print(get_tuple(ACC, imu.config(ACC)))  # Read back and display new value.
```

#### gyro_config

`value` is normally a 2-tuple comprising `(range, bandwidth)`
Allowable values:  
Range: 125, 250, 500, 1000, 2000 (dps)  
Bandwidth: 12, 23, 32, 47, 64, 116, 230, 523 (Hz).  
The return value may be restored to human-readable form by means of the
`get_tuple(GYRO, value)` function.

#### mag_config

`value` is normally a 1-tuple comprising `(rate,)` being the update rate in Hz.
Allowable values:  
Rate: 2, 6, 8, 10, 15, 20, 25, 30 (Hz)  
The return value may be restored to human-readable form by means of the
`get_tuple(MAG, value)` function.

## 3.4 Use in interrupt handlers

The `BNO055` class supports access in interrupt service routines (ISR's) by
means of the `iget` method and `w`, `x`, `y`, and `z` bound variables. The ISR
calls `iget` with the name of the value to be accessed. On return the bound
variables are updated with the raw data from the device. Each value is a signed
integer and requires scaling to be converted to standard units of measurement.
The variable names are in `bno055_help.py`.

|  Name        | Scaling     | Units    |
|:------------:|:-----------:|:--------:|
| ACC_DATA     | 1/100       | m.s^-2   |
| MAG_DATA     | 1/16        | μT       |
| GYRO_DATA    | 1/16        | °.s^-1   |
| GRAV_DATA    | 1/100       | m.s^-2   |
| LIN_ACC_DATA | 1/100       | m.s^-2   |
| EULER_DATA   | 1/16        | °        |
| QUAT_DATA    | 1/(1 << 14) | unitless |

In each case the integer values must be multiplied by the scaling to give the
units specified. In all cases other than quaternion (`QUAT_DATA`) the `iget`
method sets `.w` to zero. Example usage:

```python
def cb(t):
    imu.iget(ACC_DATA)
    print(imu.w, imu.x, imu.y, imu.z)

t = pyb.Timer(1, period=200, callback=cb)
```

###### [Contents](./README.md#contents)

# 4. Calibration

The following text is adapted from the chip datasheet. Note that calibration
requires only movement of the device while running: the process is internal to
the chip. The calibration status may be read by methods described in
[section 3.2](./README.md#32-read-only-methods).

Until the device is calibrated its orientation will be relative to that when it
was powered on. When system calibration status becomes 1 or higher the device
has found magnetic north and orientation values become absolute.

Though the sensor fusion software runs the calibration algorithm of all the
three sensors (accelerometer, gyroscope and magnetometer) in the background to
remove the offsets, some preliminary steps had to be ensured for this automatic
calibration to take place.

The accelerometer and the gyroscope are relatively less susceptible to external
disturbances, as a result of which the offset is negligible. Whereas the
magnetometer is susceptible to external magnetic field and therefore to ensure
proper heading accuracy, the calibration steps described below have to be
taken.

Depending on the sensors selected, the following simple steps should be taken
after every ‘Power on Reset’ for proper calibration of the device.

### Accelerometer Calibration

 * Place the device in 6 different stable positions for a period of few seconds
 to allow the accelerometer to calibrate.
 * Make sure that there is slow movement between 2 stable positions.
 * The 6 stable positions could be in any direction, but make sure that the
 device is lying at least once perpendicular to the x, y and z axis.
 * The `calibration_status()` method may be used to see the calibration status
 of the accelerometer.

### Gyroscope Calibration

 * Place the device in a single stable position for a period of few seconds to
 allow the gyroscope to calibrate
 * The `calibration_status()` method may be used to see the calibration status
 of the gyroscope.

### Magnetometer Calibration

Magnetometers in general are susceptible to both hard-iron and soft-iron
distortions, but the majority of the cases are rather due to the former. The
steps mentioned below are to calibrate the magnetometer for hard-iron
distortions.

 * Make some random movements (for example: writing the number ‘8’ on air)
 until the `calibration_status()` method indicates fully calibrated.
 * It takes more calibration movements to get the magnetometer calibrated than
 in the NDOF mode.

NDOF:

 * The same random movements have to be made to calibrate the sensor as in the
 FMC_OFF mode, but here it takes relatively less calibration movements (and
 slightly higher current consumption) to get the magnetometer calibrated.
 * The `calibration_status()` method can be used to see the calibration status
 of the magnetometer.

###### [Contents](./README.md#contents)

# 5. References

[Adafruit BNO055 breakout](https://www.adafruit.com/product/2472)
[Adafruit CircuitPython driver](https://github.com/adafruit/Adafruit_CircuitPython_BNO055.git).
[Device datasheet](https://cdn-learn.adafruit.com/assets/assets/000/036/832/original/BST_BNO055_DS000_14.pdf)
