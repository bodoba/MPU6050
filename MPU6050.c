/* *********************************************************************************** */
/*                                                                                     */
/*  Interact wit a MPU6050 Gyroscope and Accelerometer over I2C bus                    */
/*                                                                                     */
/*  Adopted from Alex Mous' MPU6050 Accelerometer and Gyroscope C++ library            */
/* *********************************************************************************** */
/*                                                                                     */
/*  Copyright 2025 by Bodo Bauer <bb@bb-zone.com>                                      */
/*                                                                                     */
/*  This program is free software: you can redistribute it and/or modify               */
/*  it under the terms of the GNU General Public License as published by               */
/*  the Free Software Foundation, either version 3 of the License, or                  */
/*  (at your option) any later version.                                                */
/*                                                                                     */
/*  This program is distributed in the hope that it will be useful,                    */
/*  but WITHOUT ANY WARRANTY; without even the implied warranty of                     */
/*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                      */
/*  GNU General Public License for more details.                                       */
/*                                                                                     */
/*  You should have received a copy of the GNU General Public License                  */
/*  along with this program.  If not, see <http://www.gnu.org/licenses/>.              */
/* *********************************************************************************** */
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdbool.h>
#include <pthread.h>
#include <stdlib.h>
#include <math.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "MPU6050.h"

/* *********************************************************************************** *
 * @brief Update thread
 *
 * Combined sensor data to compute accumulated angle for the three axis 
 *
 * Note: Thread is started only then parameter 'update' in init function is set to 'true'
 * *********************************************************************************** */
void *_update(void *arg);

// Data used to communicate with _update thread 
static float _angle[3];      
static bool  _calc_yaw = false;
static bool  _run_update = false;

// I2C file handle
static int  dev  = -1;

int mpu6050Init(int addr, bool update, bool yaw) {
    dev = wiringPiI2CSetup(addr);
    int status;
	
    _calc_yaw = yaw;
    _run_update = update;

	wiringPiI2CWriteReg8(dev, 0x6b, 0b00000000);   //Take MPU6050 out of sleep mode - see Register Map
	wiringPiI2CWriteReg8(dev, 0x1a, 0b00000011);   //Set DLPF (low pass filter) to 44Hz (so no noise above 44Hz will pass through)
    wiringPiI2CWriteReg8(dev, 0x19, 0b00000100);   //Set sample rate divider (to 200Hz) - see Register Map
	wiringPiI2CWriteReg8(dev, 0x1b, GYRO_CONFIG);  //Configure gyroscope settings - see Register Map (see MPU6050.h for the GYRO_CONFIG parameter)
	wiringPiI2CWriteReg8(dev, 0x1c, ACCEL_CONFIG); //Configure accelerometer settings - see Register Map (see MPU6050.h for the GYRO_CONFIG parameter)

	//Set offsets to zero
    wiringPiI2CWriteReg8(dev, 0x06, 0b00000000);
    wiringPiI2CWriteReg8(dev, 0x07, 0b00000000);
    wiringPiI2CWriteReg8(dev, 0x08, 0b00000000);
    wiringPiI2CWriteReg8(dev, 0x09, 0b00000000);
    wiringPiI2CWriteReg8(dev, 0x0A, 0b00000000);
    wiringPiI2CWriteReg8(dev, 0x0B, 0b00000000);
    wiringPiI2CWriteReg8(dev, 0x00, 0b10000001);
    wiringPiI2CWriteReg8(dev, 0x01, 0b00000001);
    wiringPiI2CWriteReg8(dev, 0x02, 0b10000001);

    if (_update) {
        pthread_t updateThread;
        int threadId=1;

        // Create thread
        if (pthread_create(&updateThread, NULL, _update, &threadId) != 0) {
            perror("Failed to create thread");
            return 1;
        }
    }

    return dev;
}

void mpu6050GetGyroRaw(float *roll, float *pitch, float *yaw) {
	int16_t X = wiringPiI2CReadReg8(dev, MPU6050_GYRO_XOUT) << 8 | wiringPiI2CReadReg8(dev, MPU6050_GYRO_XOUT+1); //Read X registers
	int16_t Y = wiringPiI2CReadReg8(dev, MPU6050_GYRO_YOUT) << 8 | wiringPiI2CReadReg8(dev, MPU6050_GYRO_YOUT+1); //Read Y registers
	int16_t Z = wiringPiI2CReadReg8(dev, MPU6050_GYRO_ZOUT) << 8 | wiringPiI2CReadReg8(dev, MPU6050_GYRO_ZOUT+1); //Read Z registers

	*roll  = (float)X; //Roll on X axis
	*pitch = (float)Y; //Pitch on Y axis
	*yaw   = (float)Z; //Yaw on Z axis
}

void mpu6050GetGyro(float *roll, float *pitch, float *yaw) {
    //Store raw values into variables
	mpu6050GetGyroRaw(roll, pitch, yaw); 
    
    //Remove the offset and divide by the gyroscope sensetivity (use 1000 and round() to round the value to three decimal places)
	*roll  = round((*roll  - G_OFF_X) * 1000.0 / GYRO_SENS) / 1000.0; 
	*pitch = round((*pitch - G_OFF_Y) * 1000.0 / GYRO_SENS) / 1000.0;
	*yaw   = round((*yaw   - G_OFF_Z) * 1000.0 / GYRO_SENS) / 1000.0;
}

void mpu6050GetAccelRaw(float *x, float *y, float *z) {
	int16_t X = wiringPiI2CReadReg8(dev, MPU6050_ACCEL_XOUT) << 8 | wiringPiI2CReadReg8(dev, MPU6050_ACCEL_XOUT+1); //Read X registers
	int16_t Y = wiringPiI2CReadReg8(dev, MPU6050_ACCEL_YOUT) << 8 | wiringPiI2CReadReg8(dev, MPU6050_ACCEL_YOUT+1); //Read Y registers
	int16_t Z = wiringPiI2CReadReg8(dev, MPU6050_ACCEL_ZOUT) << 8 | wiringPiI2CReadReg8(dev, MPU6050_ACCEL_ZOUT+1); //Read Z registers
	*x = (float)X;
	*y = (float)Y;
	*z = (float)Z;
}

void mpu6050GetAccel(float *x, float *y, float *z) {
	mpu6050GetAccelRaw(x, y, z); //Store raw values into variables

    //Remove the offset and divide by the accelerometer sensetivity (use 1000 and round() to round the value to three decimal places)
	*x = round((*x - A_OFF_X) * 1000.0 / ACCEL_SENS) / 1000.0; 
	*y = round((*y - A_OFF_Y) * 1000.0 / ACCEL_SENS) / 1000.0;
	*z = round((*z - A_OFF_Z) * 1000.0 / ACCEL_SENS) / 1000.0;
}

void mpu6050GetOffsets(float *ax_off, float *ay_off, float *az_off, float *gr_off, float *gp_off, float *gy_off) {
	float gyro_off[3]; //Temporary storage
	float accel_off[3];
    #define NUM_SAMPLES 10000

    printf("\n*** Calculating offsets for Gyroskoep and accelerometer\n\n");
    printf("  - Taking %d samples to calculate average offset:\n", NUM_SAMPLES);
    printf("    ");

	*gr_off = 0, *gp_off = 0, *gy_off = 0; //Initialize the offsets to zero
	*ax_off = 0, *ay_off = 0, *az_off = 0; //Initialize the offsets to zero

	for (int i = 0; i < NUM_SAMPLES; i++) { //Use loop to average offsets
		mpu6050GetGyroRaw(&gyro_off[0], &gyro_off[1], &gyro_off[2]); //Raw gyroscope values
		*gr_off = *gr_off + gyro_off[0], *gp_off = *gp_off + gyro_off[1], *gy_off = *gy_off + gyro_off[2]; //Add to sum

		mpu6050GetAccelRaw(&accel_off[0], &accel_off[1], &accel_off[2]); //Raw accelerometer values
		*ax_off = *ax_off + accel_off[0], *ay_off = *ay_off + accel_off[1], *az_off = *az_off + accel_off[2]; //Add to sum
        
        // visualize progress
        if(i%50==0  && i>0) {
            printf(".");
            fflush(stdout);
        }
        if(i%1000==0 && i>0) {
            printf(" [%d]\n    ");
            fflush(stdout);
        }
	}
    printf(". Done!\n");
	
    *gr_off = *gr_off / 10000, *gp_off = *gp_off / 10000, *gy_off = *gy_off / 10000; //Divide by number of loops (to average)
	*ax_off = *ax_off / 10000, *ay_off = *ay_off / 10000, *az_off = *az_off / 10000;
	*az_off = *az_off - ACCEL_SENS; //Remove 1g from the value calculated to compensate for gravity)

    printf("\nPlease paste the following defines into your MPU6050.h header file\n");
    printf("to calibrate the sensor:\n\n");
    printf("// Offsets - calculated with mpu6050GetOffsets function\n");
    printf("\n// Accelerometer\n");
    printf("#define A_OFF_X % 8d\n",   (int)*ax_off);
    printf("#define A_OFF_Y % 8d\n",   (int)*ay_off);
    printf("#define A_OFF_Z % 8d\n",   (int)*az_off);
    printf("\n// Gyroscope\n");
    printf("#define G_OFF_X % 8d\n",   (int)*gr_off);
    printf("#define G_OFF_Y % 8d\n",   (int)*gp_off);
    printf("#define G_OFF_Z % 8d\n\n", (int)*gy_off);
}



bool mpu6050GetAngle(int axis, float *result) {
    if (!_run_update) {
        fprintf(stderr,"ERR (%s) Calculation of 'Angles' has been disabled (param update set to false in init call)\n", __func__);
		*result = 0;  // Set result to zero
		return false;
    }

	if (axis >= 0 && axis <= 2) { //Check that the axis is in the valid range
		*result = _angle[axis]; //Get the result
		return true;
	}
	else {
        fprintf(stderr,"ERR (%s) 'axis' must be between 0 and 2 (for roll, pitch or yaw)\n", __func__);
		*result = 0;  // Set result to zero
		return false;
	}
}

bool mpu6050GetAngles(float *x, float *y, float *z) {
    if (!_run_update) {
        fprintf(stderr,"ERR (%s) Calculation of 'Angles' has been disabled (param update set to false in init call)\n", __func__);
		// Set result to zero
        *x = 0;  
        *y = 0;  
        *z = 0;  
		return false;
    } else {
        // Set results
        *x = _angle[0];  
        *y = _angle[1];  
        *z = _angle[2];  
        return true;
    }
}

// Update thread to keep accumulate data to keep angles, runs continiously
void *_update(void *arg) {
    float ax, ay, az, gr, gp, gy;  // Temporary storage variables used in _update()
    static float dt=0.009;         // Loop time (recalculated with each loop)
    bool first_run = true;         // Whether to set gyro angle to acceleration angle in compFilter
    struct timespec start,end;     // Create a time structure to calculate looptime
    float _accel_angle[3];         // accel roll, accel pitch, accel yaw
    float _gyro_angle[3];          // gyro roll, gyro pitch, gyro yaw


    clock_gettime(CLOCK_REALTIME, &start); // Read current time into start variable

    // Loop forever
	while (1) { 
        // Get the data from the sensors
		mpu6050GetGyro(&gr, &gp, &gy);             
		mpu6050GetAccel(&ax, &ay, &az);

		// X (roll) axis
        // Calculate the angle with z and y convert to degrees and subtract 90 degrees to rotate
		_accel_angle[0] = atan2(az, ay) * RAD_T_DEG - 90.0; 
		_gyro_angle[0] = _angle[0] + gr*dt;                    // Use roll axis (X axis)

		// Y (pitch) axis
        // Calculate the angle with z and x convert to degrees and subtract 90 degrees to rotate
		_accel_angle[1] = atan2(az, ax) * RAD_T_DEG - 90.0; 
		_gyro_angle[1] = _angle[1] + gp*dt;                    // Use pitch axis (Y axis)

		// Z (yaw) axis
		if (_calc_yaw) {
			_gyro_angle[2] = _angle[2] + gy*dt;                // Use yaw axis (Z axis)
		}

		if (first_run) {
            // Set the gyroscope angle reference point if this is the first function run
			for (int i = 0; i <= 1; i++) {
                // Start off with angle from accelerometer (absolute angle since gyroscope is relative)
				_gyro_angle[i] = _accel_angle[i];
			}
            // Set the yaw axis to zero (because the angle cannot be calculated with the accelerometer when vertical)
			_gyro_angle[2] = 0;
			first_run = false;
		}

		float asum = abs(ax) + abs(ay) + abs(az); //Calculate the sum of the accelerations
		float gsum = abs(gr) + abs(gp) + abs(gy); //Calculate the sum of the gyro readings

        // Loop through roll and pitch axes
		for (int i = 0; i <= 1; i++) {
            // Correct for very large drift (or incorrect measurment of gyroscope by longer loop time) 
			if (abs(_gyro_angle[i] - _accel_angle[i]) > 5) { 
				_gyro_angle[i] = _accel_angle[i];
			}

			// Create result from either complementary filter or directly from gyroscope or accelerometer depending on conditions
            // Check that th movement is not very high (therefore providing inacurate angles)
			if (asum > 0.1 && asum < 3 && gsum > 0.3) { 
                // Calculate the angle using a complementary filter
				_angle[i] = (1 - TAU)*(_gyro_angle[i]) + (TAU)*(_accel_angle[i]); 
			}
			else if (gsum > 0.3) { 
                // Use the gyroscope angle if the acceleration is high
				_angle[i] = _gyro_angle[i];
			}
			else if (gsum <= 0.3) {
                // Use accelerometer angle if not much movement
				_angle[i] = _accel_angle[i];
			}
		}

		// The yaw axis will not work with the accelerometer angle, so only use gyroscope angle
		if (_calc_yaw) {
            // Only calculate the angle when we want it to prevent large drift
			_angle[2] = _gyro_angle[2];
		}
		else {
			_angle[2] = 0;
			_gyro_angle[2] = 0;
		}

        // Calculate new looptime dt
		clock_gettime(CLOCK_REALTIME, &end);
		dt = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9; 
		clock_gettime(CLOCK_REALTIME, &start);
	}
}