# MPU6050 Accelerometer and Gyroscope support in C with wiringPi as backend

Basic functions for using the MPU6050 accelerometer and gyroscope module with Raspberry Pi using i2c protocol to read raw accelerometer data and fully corrected (with complementary filters and some logic) angles on any axis (roll, pitch, yaw).

This is heavily based on the [MPU6050 Accelerometer and Gyroscope C++ library](https://github.com/alex-mous/MPU6050-C-CPP-Library-for-Raspberry-Pi) by [Alex Mous](https://github.com/alex-mous).

## Usage

The usage is pretty straight forward. First initialize the device, then get the readings you like.

### Intialization

`int mpu6050Init(int addr, bool update, bool yaw)`

This initializes the MPU6050 and starts a worker thread to keep track of the actual change since startup.

* ```addr``` - Device I2C address
  * Usualliy 0x68, unless the default address has been changed on the device

* ```update``` - Use worker thread?
  * 'true': Worker thread has shall be started
  * 'false': No worker thread requiered. Either because the collected data is not needed, or you want to create your own way of accumulating the data over time. Calls to `mpu6950GetAngle` will fail.

* ```yaw``` - Compute changes to yaw (Z axis rotation) in worker thread?
  * 'true': Yaw will be calculated
  * 'false': Calls to `mpu6950GetAngle` will return 0 for yaw.


### Get Sensor readings

For the accelerometer and the gyroscope reading you can get either the raw values from the sensor, or the preproceed values in degrees per second.

Each call has pointers to the variables for X, Y, and Z velues as parameter.

* ```void mpu6050GetAccel(float *x, float *y, float *z)```

  * Gets the raw accelerometer values from the MPU6050 registers

* ```void mpu6050GetAccelRaw(float *x, float *y, float *z)```

  * Gets the gyroscope values (in degrees/second)

* ```void mpu6050GetGyroRaw(float *x, float *y, float *z)```

  * Gets the raw gyroscope values from the MPU6050 registers

* ```void mpu6050GetGyro(float *x, float *y, float *z)```

  * Gets the gyroscope values (in degrees/second)

### Get accumulated change in angles

To get the angles the device turned since initialization of the sensor, a orker thread takes the sensor reading in an endliess loop and accumulates the data to compute the actual angle.

You can either request the angle of a single axis, or all values at once. Both functions return ```true``` on success and ```false``` on error.

> [!NOTE] This works only when parameter ```update``` of ```mpu6050Init``` has been set to ```true```.

> [!NOTE] The value for yaw (Z axis) will return ```0``` unless ```yaw``` is set to true in the ```mpu6050Init``` call.

* ```bool mpu6050GetAngle(int axis, float *result)```

  * Gets the current combined (accelerometer and gyroscope) angle for an individual axis

  * ```axis``` axis which axis to use (0 for roll (x), 1 for pitch (Y) and 2 for yaw (Z))

  * ```result``` pointer to the variable where the angle will be stored


* ```bool mpu6050GetAngles(float *x, float *y, float *z)```

  * gets the current combined (accelerometer and gyroscope) angle for all axis. The paramters are pointers to the variables for X, Y, and Z velues as parameter.

## Example

This code will setup the sensor and produce a list of readings:

```C
#define MPU6050_ADDR   0x68

int main(int argc, char* argv[]) {
    int mpu6050 = mpu6050Init(MPU6050_ADDR, true, true);
    float roll, pitch, yaw;        // Gyroskope
    float x, y, z;                 // Accelerometer
    float angleX, angleY, angleZ;  // Combined values

    if ( mpu6050 < 0 ) {
        fprintf( stderr, "ERROR: MPU6050 not found at 0x%02x\n", MPU6050_ADDR);
        exit(1);
    }

    // Read and display gyro and accel input
    for(;;) {
        // read current sensor data
        mpu6050GetGyro(&roll, &pitch, &yaw);
        mpu6050GetAccel(&x, &y, &z);
        
        // get accu,ulated angels
        mpu6050GetAngles(&angleX, &angleY, &angleZ);

        printf("| Gyro: % 8.3f % 8.3f % 8.3f | Accel % 8.3f % 8.3f % 8.3f| Angle % 8.3f % 8.3f % 8.3f\n", 
                roll, pitch, yaw, 
                x, y, z,
                angleX, angleY, angleZ);
        
        usleep(100000);
    }
}
```

The output looks like this (device slowly rotating on the Z axis):

```
$ ./mpu6050
| Gyro:    1.779   -0.557   -0.588 | Accel    0.001   -0.001    1.005| Angle    0.000    0.000    0.000
| Gyro:    1.168    7.496    0.305 | Accel    0.029    0.146    1.010| Angle    1.884   -3.059   -0.002
| Gyro:    3.336   -0.008  -23.603 | Accel   -0.020   -0.002    1.006| Angle    1.961   -2.026    0.818
| Gyro:    0.366   -1.023  -16.939 | Accel    0.012    0.002    1.002| Angle    0.636   -0.766   -0.450
| Gyro:    1.153   -1.672  -42.603 | Accel   -0.015    0.003    1.021| Angle    0.235   -0.476   -4.391
| Gyro:    1.076   -1.061  -31.649 | Accel   -0.008    0.007    1.010| Angle   -0.008   -0.407   -8.685
| Gyro:    3.641   -1.206  -40.954 | Accel    0.001    0.004    1.006| Angle    0.041   -0.322  -12.810
| Gyro:    1.420   -2.084  -62.191 | Accel   -0.009    0.005    1.004| Angle    0.040    0.083  -18.462
| Gyro:    1.962   -1.237  -52.107 | Accel   -0.019   -0.000    1.004| Angle   -0.201   -0.157  -25.310
| Gyro:    1.496   -0.611  -26.382 | Accel    0.009   -0.001    1.004| Angle   -0.335   -0.137  -30.046
| Gyro:    2.794   -0.107  -27.603 | Accel    0.009   -0.001    1.005| Angle   -0.219   -0.096  -32.956
| Gyro:    1.473   -0.954  -30.435 | Accel    0.000    0.030    0.997| Angle   -0.509    0.032  -36.885
| Gyro:    1.565   -1.466  -44.786 | Accel   -0.004    0.017    1.008| Angle   -0.452   -0.209  -40.712
| Gyro:    1.824   -0.534  -22.008 | Accel   -0.034    0.020    1.011| Angle   -0.609   -0.299  -44.453
| Gyro:    4.053    1.832  -12.588 | Accel    0.023    0.030    0.989| Angle   -0.787   -0.264  -45.148
```



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