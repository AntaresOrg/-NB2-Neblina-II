/**
 * @file       mpu6050.c
 * @brief      Source implementation for the MPU6050 driver responsible for inertial measurements and calibration routines.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Source
 * @author     git_user_name <git_user_email>
 *
 * @details
 * This source file implements the MPU6050 inertial sensor driver for acceleration and gyroscope acquisition.
 * It contains the communication routines, calibration logic, and data processing needed to monitor motion and
 * orientation during the flight. The implementation is tailored to the rocket avionics workflow and the ESP32
 * platform used by the Neblina II project.
 */
#include "avionics/mpu6050.h"
