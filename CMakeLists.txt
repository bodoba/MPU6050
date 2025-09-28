# *********************************************************************************** #
#                                                                                     #
#  Interact with a MPU6050 Gyroscope and Accelerometer over I2C bus                   #
#                                                                                     #
# *********************************************************************************** #
#                                                                                     #
#  Copyright 2025 by Bodo Bauer <bb@bb-zone.com>                                      #
#                                                                                     #
#  This program is free software: you can redistribute it and/or modify               #
#  it under the terms of the GNU General Public License as published by               #
#  the Free Software Foundation, either version 3 of the License, or                  #
#  (at your option) any later version.                                                #
#                                                                                     #
#  This program is distributed in the hope that it will be useful,                    #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of                     #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                      #
#  GNU General Public License for more details.                                       #
#                                                                                     #
#  You should have received a copy of the GNU General Public License                  #
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.              #
# *********************************************************************************** #

cmake_minimum_required(VERSION 3.10.0)
project(TankBot VERSION 0.1.0 LANGUAGES C)

include_directories(/opt/local/include)
link_directories(/opt/local/lib)

add_executable(mpu6050  example.c   MPU6050.c  MPU6050.h)
target_link_libraries(mpu6050 wiringPi m)

add_executable(get_offsets  get_offsets.c   MPU6050.c  MPU6050.h)
target_link_libraries(get_offsets wiringPi m)