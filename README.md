# MPU6050 Accelerometer and Gyroscope support in C with wiringPi as backend

Basic functions for using the MPU6050 accelerometer and gyroscope module with Raspberry Pi using i2c protocol to read raw accelerometer data and fully corrected (with complementary filters and some logic) angles on any axis (roll, pitch, yaw).

This is heavily based on the [MPU6050 Accelerometer and Gyroscope C++ library](https://github.com/bodoba/MPU6050/tree/master) by [Alex Mous](https://github.com/alex-mous).

## Usage

_TBD_

## Calibrating the sensor

The tool `get_offsets` can be used to compute the offsets needed to calibrate the sensor (see `MPOU650.h`).
Before running, place the module on a completely flat surface (check with a sirit level if possible) and make
sure that it stays still while running this function, the results will be displayed, to you can copy/paste them
into the header file.

```
$ ./get_offsets

*** Calculating offsets for Gyroskoep and accelerometer

  - Taking 10000 samples to calculate average:
    .................... [1000]
    .................... [2000]
    .................... [3000]
    .................... [4000]
    .................... [5000]
    .................... [6000]
    .................... [7000]
    .................... [8000]
    .................... [9000]
    .................... Done!

Please paste the following defines into your MPU6050.h header file
to calibrate the sensor:

// Offsets - calculated with mpu6050GetOffsets function

// Accelerometer
#define A_OFF_X     9819
#define A_OFF_Y    -1772
#define A_OFF_Z    -4781

// Gyroscope
#define G_OFF_X     -237
#define G_OFF_Y      -75
#define G_OFF_Z     -156
```