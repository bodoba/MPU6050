/* *********************************************************************************** */
/*                                                                                     */
/*  Interact with a MPU6050 Gyroscope and Accelerometer over I2C bus                   */
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
#ifndef MPU6050_H
#define MPU6050_H

//-----------------------MODIFY THESE PARAMETERS-----------------------

#define GYRO_RANGE 0 //Select which gyroscope range to use (see the table below) - Default is 0
// Gyroscope Range
//	0	+/- 250 degrees/second
//	1	+/- 500 degrees/second
//	2	+/- 1000 degrees/second
//	3	+/- 2000 degrees/second
// See the MPU6000 Register Map for more information


#define ACCEL_RANGE 0 //Select which accelerometer range to use (see the table below) - Default is 0
// Accelerometer Range
//	0	+/- 2g
//	1	+/- 4g
//	2	+/- 8g
//	3	+/- 16g
// See the MPU6000 Register Map for more information


// Axis for getAngle function
#define MPU6050_X  0
#define MPU6050_Y  1
#define MPU6050_Z  2

// Offsets - calculated with mpu6050GetOffsets function

// Accelerometer
#define A_OFF_X     9763
#define A_OFF_Y    -1683
#define A_OFF_Z    -4811

// Gyroscope
#define G_OFF_X     -236
#define G_OFF_Y      -76
#define G_OFF_Z     -158

//-----------------------END MODIFY THESE PARAMETERS-----------------------

#define MPU6050_ACCEL_XOUT       0x3B   // R
#define MPU6050_ACCEL_YOUT       0x3D   // R
#define MPU6050_ACCEL_ZOUT       0x3F   // R

#define MPU6050_GYRO_XOUT        0x43   // R
#define MPU6050_GYRO_YOUT        0x45   // R
#define MPU6050_GYRO_ZOUT        0x47   // R
 
#define MPU6050_PWR_MGMT_1       0x6B   // R/W

#define TAU 0.05                 //Complementary filter percentage
#define RAD_T_DEG 57.29577951308 //Radians to degrees (180/PI)

//Select the appropriate settings
#if GYRO_RANGE == 1
	#define GYRO_SENS 65.5
	#define GYRO_CONFIG 0b00001000
#elif GYRO_RANGE == 2
	#define GYRO_SENS 32.8
	#define GYRO_CONFIG 0b00010000
#elif GYRO_RANGE == 3
	#define GYRO_SENS 16.4
	#define GYRO_CONFIG 0b00011000
#else //Otherwise, default to 0
	#define GYRO_SENS 131.0
	#define GYRO_CONFIG 0b00000000
#endif
#undef GYRO_RANGE

#if ACCEL_RANGE == 1
	#define ACCEL_SENS 8192.0
	#define ACCEL_CONFIG 0b00001000
#elif ACCEL_RANGE == 2
	#define ACCEL_SENS 4096.0
	#define ACCEL_CONFIG 0b00010000
#elif ACCEL_RANGE == 3
	#define ACCEL_SENS 2048.0
	#define ACCEL_CONFIG 0b00011000
#else //Otherwise, default to 0
	#define ACCEL_SENS 16384.0
	#define ACCEL_CONFIG 0b00000000
#endif
#undef ACCEL_RANGE


#include <stdbool.h>

/* *********************************************************************************** *
 * @brief Initialize MPU6050 sensor over I2C and start thread to track X/Y/Z angles
 *
 * @param addr   sensor address
 * @param update if true, the an update thread will be spawned to track the angles
 * @param yaw    set this to true when you want to calculate the angle around the yaw 
 *               axis (remember to change this back to false after taking the yaw 
 *               readings to prevent gyroscope drift) this is used to prevent drift 
 *               because only the gyroscope is used to calculate the yaw rotation
 * @return       device handle, or -1 on failure
 * *********************************************************************************** */
int mpu6050Init(int addr, bool update, bool yaw);

/* *********************************************************************************** *
 * @brief Compute sensor offsets to calibrate sensor.
 *
 * The function prints whats it's doing provides copy/paste code to include the
 * offsets into the MPU6050.h header file
 *
 * @param *ax_off pointer float where the accelerometer x axis offset will be stored
 * @param *ay_off pointer float where the accelerometer y axis offset will be stored
 * @param *az_off pointer float where the accelerometer z axis offset will be stored
 * @param *gr_off pointer float where the gyroscope roll (x) axis offset will be stored
 * @param *gp_off pointer float where the gyroscope pitch (y) axis offset will be stored
 * @param *gy_off pointer float where the gyroscope yaw (z) axis offset will be stored
 * *********************************************************************************** */
void mpu6050GetOffsets(float *ax_off, float *ay_off, float *az_off, 
					   float *gr_off, float *gp_off, float *gy_off);


/* *********************************************************************************** *
 * @brief gets the accelerometer values (in g), rounded to three decimal places
 *
 * @param *x pointer float where the gyroscope roll (x) axis offset will be stored
 * @param *y pointer float where the gyroscope pitch (y) axis offset will be stored
 * @param *z pointer float where the gyroscope yaw (z) axis offset will be stored
 * *********************************************************************************** */
void mpu6050GetAccel(float *x, float *y, float *z);

/* *********************************************************************************** *
 * @brief Gets the raw accelerometer values from the MPU6050 registers
 *
 * @param *x pointer float where the gyroscope roll (x) axis offset will be stored
 * @param *y pointer float where the gyroscope pitch (y) axis offset will be stored
 * @param *z pointer float where the gyroscope yaw (z) axis offset will be stored
 * *********************************************************************************** */
void mpu6050GetAccelRaw(float *x, float *y, float *z);


/* *********************************************************************************** *
 * @brief Gets the gyroscope values (in degrees/second)
 *
 * @param *x pointer float where the gyroscope roll (x) axis offset will be stored
 * @param *y pointer float where the gyroscope pitch (y) axis offset will be stored
 * @param *z pointer float where the gyroscope yaw (z) axis offset will be stored
 * *********************************************************************************** */
void mpu6050GetGyro(float *x, float *y, float *z);

/* *********************************************************************************** *
 * @brief Gets the raw gyroscope values from the MPU6050 registers
 *
 * @param *x pointer float where the gyroscope roll (x) axis offset will be stored
 * @param *y pointer float where the gyroscope pitch (y) axis offset will be stored
 * @param *z pointer float where the gyroscope yaw (z) axis offset will be stored
 * *********************************************************************************** */
void mpu6050GetGyroRaw(float *x, float *y, float *z);

/* *********************************************************************************** *
 * @brief gets the current combined (accelerometer and gyroscope) angle for an 
 *        individual axis
 *
 * NOTE: Works only when parameter 'update' of init function has been set to 'true' 
 * 
 * NOTE: the yaw axis will return 0 unless 'yaw' is set to true - See Parameters
 *       of mpu6050Init
 * 
 * @param  axis which axis to use (0 for roll (x), 1 for pitch (Y) and 2 for yaw (Z))
 * @param  *result pointer to the variable where the angle will be stored
 * @return true on success, false on error
 * *********************************************************************************** */
bool mpu6050GetAngle(int axis, float *result);

/* *********************************************************************************** *
 * @brief gets the current combined (accelerometer and gyroscope) angle for all axis
 *
 * NOTE: Works only when parameter 'update' of init function has been set to 'true' 
 * 
 * NOTE: the yaw axis will return 0 unless 'yaw' is set to true - See Parameters
 *       of mpu6050Init
 * 
 * @param  *x pointer to the variable where the X angle will be stored
 * @param  *y pointer to the variable where the Y angle will be stored
 * @param  *z pointer to the variable where the Z angle will be stored
 * @return  true on success, false on error
 * *********************************************************************************** */
bool mpu6050GetAngles(float *x, float *y, float *z);
#endif