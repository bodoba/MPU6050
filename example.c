/* *********************************************************************************** */
/*                                                                                     */
/*  Interact wit a MPU6050 Gyroscope and Accelerometer over I2C bus                    */
/*                                                                                     */
/* https://github.com/alex-mous/MPU6050-C-CPP-Library-for-Raspberry-Pi/tree/master     */
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
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "MPU6050.h"

#define MPU6050_ADDR   0x68

int main(int argc, char* argv[]) {
    int mpu6050 = mpu6050Init(MPU6050_ADDR,true, false);
    float roll, pitch, yaw; // Gyroskope
    float x, y, z;          // Accelerometer

    if ( mpu6050 < 0 ) {
        fprintf( stderr, "ERROR: MPU6050 not found at 0x%02x\n", MPU6050_ADDR);
        exit(1);
    }

    // Read and display gyro and accel input
    for(;;) {
        mpu6050GetGyro(&roll, &pitch, &yaw);
        mpu6050GetAccel(&x, &y, &z); 
        printf("|Roll: % 8.3f | Pitch: % 8.3f | Yaw: % 8.3f | Y: % 8.3f | X: % 8.3f | Z: % 8.3f|\n", roll, pitch, yaw, x, y, z);
        usleep(10000);
    }
}
