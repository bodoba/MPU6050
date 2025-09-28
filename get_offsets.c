/* *********************************************************************************** */
/*                                                                                     */
/*  Measure offsets to calibrate MPU6050 sensor                                        */
/*                                                                                     */
/* before running, place the module on a completely flat surface (check with a sirit   */
/* level if possible) and make sure that it stays still while running this function,   */
/* the results will be stored in the variables ax_off, ay_off, az_off, gr_off, gp_off, */
/* gy_off for accel x offset... and gyro roll offset..., take these values and put     */
/* them into the MPU6050.h file                                                        */
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

#include "MPU6050.h"

#define MPU6050_ADDR   0x68

int main(int argc, char* argv[]) {
    float ax_off, ay_off, az_off, gr_off, gp_off, gy_off;
    int mpu6050 = mpu6050Init(MPU6050_ADDR, false, false);

    if ( mpu6050 < 0 ) {
        fprintf( stderr, "ERROR: MPU6050 not found at 0x%02x\n", MPU6050_ADDR);
        exit(1);
    }
  
    // calculate offsets
    mpu6050GetOffsets(&ax_off, &ay_off, &az_off, &gr_off, &gp_off, &gy_off);
}
