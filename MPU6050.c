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

void *_update(void *arg);

static float _accel_angle[3];
static float _gyro_angle[3];
static float _angle[3]; //Store all angles (accel roll, accel pitch, accel yaw, gyro roll, gyro pitch, gyro yaw, comb roll, comb pitch comb yaw)
static float ax, ay, az, gr, gp, gy; //Temporary storage variables used in _update()
static float dt; //Loop time (recalculated with each loop)
// static struct timespec start,end; //Create a time structure
static bool _first_run = 1; //Variable for whether to set gyro angle to acceleration angle in compFilter

static int  dev      = -1;
static bool calc_yaw = false;

int mpu6050Init(int addr, bool update, bool yaw) {
    dev = wiringPiI2CSetup(addr);
    int status;

	dt = 0.009;     //Loop time (recalculated with each loop)
	_first_run = 1; //Variable for whether to set gyro angle to acceleration angle in compFilter
	calc_yaw = yaw;

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

    if (update) {
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

int mpu6050GetAngle(int axis, float *result) {
	if (axis >= 0 && axis <= 2) { //Check that the axis is in the valid range
		*result = _angle[axis]; //Get the result
		return 0;
	}
	else {
//		std::cout << "ERR (MPU6050.cpp:getAngle()): 'axis' must be between 0 and 2 (for roll, pitch or yaw)\n"; //Print error message
		*result = 0; //Set result to zero
		return 1;
	}
}

// Update thread, runs continiously
void *_update(void *arg) {
    delay(1000);
}