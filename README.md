# micropython-bno055

A MicroPython driver for the Bosch BNO055 inertial measurement unit (IMU). This
chip has the advantage of performing sensor fusion in hardware. The driver is
based on the [Adafruit CircuitPython driver](https://github.com/adafruit/Adafruit_CircuitPython_BNO055.git).

This driver has the following objectives:
 * It runs under official MicroPython.
 * It is cross-platform and designed to run on Pyboard 1.x, Pyboard D, ESPx. It
 should run on any hardware supporting the `machine` module and the I2C
 interface.
 * There is a minimal version with small memory footprint for ESP8266 (~9.7KB).
 * Supports changing the device mode.
 * Supports vehicle-relative coordinate transformation on-chip.
 * Supports changing the hardware configuration.
 * Supports access in interrupt service routines.
 * Uses the MicroPython approach to coding (avoids properties/descriptors).

Testing was done with the [Adafruit BNO055 breakout](https://www.adafruit.com/product/2472).
This chip and breakout come highly recommended. Calibration requires a little
practice, but once done the fusion algorithm is remarkably immune to external
magnetic fields. A field which displaced my hiker's compass by 90° caused at
most 2° of heading change on this device. The raw magnetometer readings changed
radically but heading remained essentially constant.

# Contents

 1. [Files and dependencies](./README.md#1-files-and-dependencies)  
 2. [Getting started](./README.md#2-getting-started)  
 3. [The BNO055 class](./README.md#3-the-bno055-class)  
  3.1 [Constructor](./README.md#31-constructor)  
  3.2 [Read only methods](./README.md#32-read-only-methods) Read data from device.  
  3.3 [Changing the device configuration](./README.md#33-changing-the-device-configuration)  
    3.3.1 [Mode setting](./README.md#331-mode-setting) Modify device operating mode.  
    3.3.2 [Rate and range control](./README.md#332-rate-and-range-control) Further settings.  
  3.4 [Use in interrupt handlers](./README.md#34-use-in-interrupt-handlers)  
  3.5 [Other methods](./README.md#35-other-methods)  
 4. [Calibration](./README.md#4-calibration)  
 5. [Minimal version](./README.md#5-minimal-version) Minimise RAM usage.  
 6. [References](./README.md#6-references)  

# 1. Files and dependencies

 * `bno055_base.py` Base class for device driver.
 * `bno055.py` Device driver.
 * `bno055_test.py` Simple test program. Can run on Pyboard or (with pin
 changes) ESP8266: see code comments.

The driver has no dependencies. It is designed to be imported using
```python
from bno055 import *
```
In addition to the `BNO055` class this imports symbolic names for modes and
data registers. On highly RAM-constrained targets the base class may be used
alone with some loss of functionality, see
[section 5](./README.md#5-minimal-version).

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

Note that pullups (typically 10KΩ to 3.3V) are required on SCL and SDA. The
Pyboard has these on `I2C(1)` and `I2C(2)`, as does the Adafruit BNO055
breakout. ESP8266 boards have pullups on pins 0 and 2. External pullups will
therefore only be required if using a non-Adafruit breakout with MicroPython
board pins lacking pullups.

Basic usage is as follows:

```python
import machine
import time
from bno055 import *
# Pyboard hardware I2C
i2c = machine.I2C(1)
# ESP8266 soft I2C
# i2c = machine.I2C(-1, scl=machine.Pin(2), sda=machine.Pin(0))
imu = BNO055(i2c)
calibrated = False
while True:
    time.sleep(1)
    if not calibrated:
        calibrated = imu.calibrated()
        print('Calibration required: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))
    print('Temperature {}°C'.format(imu.temperature()))
    print('Mag       x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.mag()))
    print('Gyro      x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.gyro()))
    print('Accel     x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.accel()))
    print('Lin acc.  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.lin_acc()))
    print('Gravity   x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.gravity()))
    print('Heading     {:4.0f} roll {:4.0f} pitch {:4.0f}'.format(*imu.euler()))
```
To calibrate the chip move the unit as per [section 4](./README.md#4-calibration)
until calibration values for gyro, accel and mag are 3 and sys is >0.

Note that if code is started automatically on power up (by a line in main.py) a
delay of 500ms should be applied before instantiating the `BNO055`. This is to
allow for the BNO055 chip startup time (400ms typical).

###### [Contents](./README.md#contents)

# 3. The BNO055 class

## 3.1 Constructor

This takes the following args

 * `i2c` An initialised I2C instance.
 * `address=0x28` Default device address. The Adafruit breakout allows this to
 be changed to 0x29 for the case where two devices are attached to one bus.
 * `crystal=True`  If `True` use an external crystal for higher accuracy; the
 Adafruit board has a crystal. If the hardware does not have a crystal this
 should be `False`: the chip's internal oscillator will be used.

The following optional args provide for vehicle relative coordinates. The
default values assume that the IMU is mounted component side up and in the
horizontal plane. Alternatives cater for cases where the IMU is rotated or
mounted orthogonally relative to the horizontal plane of the vehicle.
 * `transpose=(0, 1, 2)`
 * `sign=(0, 0, 0)`

Axes are numbered 0 for X, 1 for Y and 2 for Z, and the transpose tuple is
`(x, y, z)`. Hence `(0, 1, 2)` implies no transformation. Passing `(1, 0, 2)`
implies a rotation around the Z axis.

Sign values must be 0 (normal) or 1 (inverted). Hence a board rotated around
the Y axis and mounted upside down would have `sign=(1, 0, 1)` (X and Z axes
inverted). This is further explained in the 
[Device datasheet](https://cdn-learn.adafruit.com/assets/assets/000/036/832/original/BST_BNO055_DS000_14.pdf)
section 3.4.

The constructor blocks for 700ms (1.2s if `crystal==True`).

## 3.2 Read only methods

Return values (numbers are floats unless stated otherwise):
 * `mag()` Magnetometer vector `(x, y, z)` in μT (microtesla).
 * `accel()` Accelerometer vector `(x, y, z)` in m.s^-2
 * `lin_acc()` Acceleration vector `(x, y, z)` after removal of gravity
 component (m.s^-2).*
 * `gravity()` Gravity vector `(x, y, z)` in m.s^-2 after removal of
 acceleration data.*
 * `gyro()` Gyro vector `(x, y, z)` in deg.s^-1.
 * `euler()` Euler angles in degrees `(heading, roll, pitch)`.*
 * `quaternion()` Quaternion `(w, x, y, z)`.*
 * `temperature()` Chip temperature as an integer °C (Celcius).
 * `calibrated()` `True` if all elements of the device are calibrated.
 * `cal_status()` Returns bytearray  `[sys, gyro, accel, mag]`. Each element
 has a value from 0 (uncalibrated) to 3 (fully calibrated).
 * `external_crystal()` `True` if using an external crystal.
 * `get_config(sensor)` See [Section 3.3.2](./README.md#332-rate-and-range-control).

Some methods only produce valid data if the chip is in a fusion mode. If the
mode is changed from the default to a non-fusion one, methods such as `euler`
will return zeros. Such methods are marked with a * above.

###### [Contents](./README.md#contents)

## 3.3 Changing the device configuration

Many applications will use the default mode of the chip. This section describes
ways of changing this for special purposes, for example where a high update
rate is required. This can arise if readings are used in a feedback loop where
latency can cause stability issues.

### 3.3.1 Mode setting

 * `mode(new_mode=None)` If a value is passed, change the mode of operation.
 Returns the mode as it was before any change was made. The mode is an integer.
 The `BNO055` module provides symbolic names as per the table below.

The mode of operation defines which sensors are enabled, whether fusion occurs
and if measurements are absolute or relative. The `bno055` module provides the
mode values listed below as integers.

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
This example illustrates restoration of the prior mode (`imu` is a `BNO055`
instance):
```python
from bno055 import *
# code omitted to instantiate imu
old_mode = imu.mode(ACCGYRO_MODE)
# code omitted
imu.mode(old_mode)
```
The purpose of the various modes is covered in the
[Device datasheet](https://cdn-learn.adafruit.com/assets/assets/000/036/832/original/BST_BNO055_DS000_14.pdf)
section 3.3.

###### [Contents](./README.md#contents)

### 3.3.2 Rate and range control

In non-fusion modes the chip allows control of the update rate of each sensor
and the range of the accelerometer and gyro. In fusion modes rates are fixed:
the only available change is to the accelerometer range. The folowing shows the
default setings after initialisation (mode is `NDOF`). The update rate of
fusion values is 100Hz.

| Device | Full scale  | Update rate |
|:------:|:-----------:|:-----------:|
| Accel  | +-4G        | 100Hz       |
| Gyro   | 2000°/s     | 100Hz       |
| Mag    | -           | 20Hz        |

The magnetometer has a single range: units are Micro Tesla (μT).

In modes which permit it, sensors may be controlled with the following method.

 * `config(dev, value=None)` `dev` is the device: must be `ACC`, `GYRO` or
 `MAG` (names defined in `bno055.py`). The `value` arg is a tuple holding the
 requested configuration. See below for details specific to each sensor.

The default value of `None` causes no change to be made. In each case the
method returns the config tuple as it was before any change was made. In
certain circumstances the chip can return an unknown value. This was observed
in the case of the initial value from the magnetometer. In such cases the
result will be `False`. Returning the prior value allows old values to be
restored, e.g.  
```python
old_config = imu.config(ACC, (2, 250))
# ...
if old_config:  # Valid config returned
    imu.config(ACC, old_config)  # Restore old config
```
Note that the hardware will only allow configuration changes in appropriate
modes. For example to change gyro settings the chip must be in a non-fusion
mode which enables the gyro. If the mode is such that changes are not allowed,
failure will be silent. If in doubt check the result by reading back the
resultant config:
```python
imu.mode(ACCGYRO_MODE)  # Allows config change to ACC or GYRO
cfg = (2, 250)  # Intended config
imu.config(ACC, cfg)
if imu.config(ACC) == cfg:
    print('Success')
```

#### Accelerometer (dev == ACC)

`value` is a 2-tuple comprising `(range, bandwidth)`
Allowable values:  
Range: 2, 4, 8, 16 (G).  
Bandwidth: 8, 16, 31, 62, 125, 250, 500, 1000 (Hz).  
The outcome of a change may be shown by means of the `.config(ACC)` method.
```python
from bno055 import *
# code omitted
imu.mode(ACCONLY_MODE)  # non-fusion mode
cfg = (2, 1000)  # Intended config
imu.config(ACC, cfg) # Update.
if imu.config(ACC) == cfg:
    print('Success')
```

#### Gyro (dev == GYRO)

`value` is a 2-tuple comprising `(range, bandwidth)`
Allowable values:  
Range: 125, 250, 500, 1000, 2000 (dps)  
Bandwidth: 12, 23, 32, 47, 64, 116, 230, 523 (Hz).  
The outcome of a change may be shown by means of the `.config(GYRO)` method.

#### Magnetometer (dev == MAG)

`value` is a 1-tuple comprising `(rate,)` being the update rate in Hz.
Allowable values:  
Rate: 2, 6, 8, 10, 15, 20, 25, 30 (Hz)  
The outcome of a change may be shown by means of the `.config(MAG)` method.
Note that on first call the prior config may be unknown and the method will
return `False`. This is a chip behaviour.

###### [Contents](./README.md#contents)

## 3.4 Use in interrupt handlers

The `BNO055` class supports access in interrupt service routines (ISR's) by
means of the `iget` method and `w`, `x`, `y`, and `z` bound variables. The ISR
calls `iget` with the name of the value to be accessed. On return the bound
variables are updated with the raw data from the device. Each value is a signed
integer that requires scaling to be converted to standard units of measurement.

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

## 3.5 Other methods

 * `reset` No args. Equivalent to pulsing the chip's reset line: restores all
 power on defaults and resets the calibration status. Blocks for 700ms (1.2s if
 the constructor was called with `crystal==True`). Reinstates vehicle relative
 transformations specified to the constructor.

###### [Contents](./README.md#contents)

# 4. Calibration

Calibration requires only movement of the device while running: the process is
internal to the chip and its nature is opaque. The calibration status may be
read by methods described in [section 3.2](./README.md#32-read-only-methods).

Until the device is calibrated its orientation will be relative to that when it
was powered on. When system calibration status becomes 1 or higher the device
has found magnetic north and orientation values become absolute.
([Source](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/device-calibration)).
The status returned by the chip, and hence the return values of `.calibrated`
and `.cal_status` methods can regress after successful calibration. The meaning
of this is unclear. It seems reasonable to assume that once the chip returns a
good status it can be assumed to be OK; the demo scripts make that assumption.

The following text is adapted from the chip datasheet; it could be clearer. I
recommend watching [this Bosch video](https://youtu.be/Bw0WuAyGsnY) for a good
exposition.

Though the sensor fusion software runs the calibration algorithm of all the
three sensors (accelerometer, gyroscope and magnetometer) in the background to
remove the offsets, some preliminary steps should be ensured for this automatic
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
 * The `cal_status()` method may be used to see the calibration status of the
 accelerometer.

### Gyroscope Calibration

 * Place the device in a single stable position for a period of few seconds to
 allow the gyroscope to calibrate
 * The `cal_status()` method may be used to see the calibration status of the
 gyroscope.

### Magnetometer Calibration

Magnetometers in general are susceptible to both hard-iron and soft-iron
distortions, but the majority of the cases are rather due to the former. The
steps mentioned below are to calibrate the magnetometer for hard-iron
distortions.

 * Make some random movements (for example: writing the number ‘8’ on air)
 until the `cal_status()` method indicates fully calibrated.
 * It takes more calibration movements to get the magnetometer calibrated than
 in the NDOF mode.

NDOF:

 * The same random movements have to be made to calibrate the sensor as in the
 FMC_OFF mode, but here it takes relatively less calibration movements (and
 slightly higher current consumption) to get the magnetometer calibrated.
 * The `cal_status()` method can be used to see the calibration status of the
 magnetometer.

###### [Contents](./README.md#contents)

# 5. Minimal Version

This is intended for devices such as ESP8266 where RAM is limited. Note that
the full version will run on ESP8266 using ~14K of RAM. The minimal version
reduces this to just over 9K.

The minimal version does not support vehicle-relative coordinates, ISR usage
or configuration changes. Mode changes can be done, but symbolic names of modes
are not supplied. The version is primarily intended for use in the default
`NDOF` mode.

In use the `bno055_base` module is imported and the base class is used. Example
tested on an ESP8266:
```python
import machine
import time
from bno055_base import BNO055_BASE

i2c = machine.I2C(-1, scl=machine.Pin(2), sda=machine.Pin(0))
imu = BNO055_BASE(i2c)
calibrated = False
while True:
    time.sleep(1)
    if not calibrated:
        calibrated = imu.calibrated()
        print('Calibration required: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))
    print('Temperature {}°C'.format(imu.temperature()))
    print('Mag       x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.mag()))
    print('Gyro      x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.gyro()))
    print('Accel     x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.accel()))
    print('Lin acc.  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.lin_acc()))
    print('Gravity   x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.gravity()))
    print('Heading     {:4.0f} roll {:4.0f} pitch {:4.0f}'.format(*imu.euler()))
```

# 6. References

[Adafruit BNO055 breakout](https://www.adafruit.com/product/2472)  
[Adafruit CircuitPython driver](https://github.com/adafruit/Adafruit_CircuitPython_BNO055.git).  
[Device datasheet](https://cdn-learn.adafruit.com/assets/assets/000/036/832/original/BST_BNO055_DS000_14.pdf)
