/**
 * @file       mpu6050.h
 * @brief      Driver for the MPU6050 inertial sensor used to read acceleration and angular velocity.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Header
 * @author     git_user_name <git_user_email>
 *
 * @details
 * This module controls the MPU6050 inertial measurement unit used to measure acceleration and angular velocity.
 * It provides the sensor access routines, calibration flow, and data processing required for attitude and
 * motion monitoring during flight. The implementation is tailored for the ESP32 and the rocket avionics
 * integration used by the Neblina II system.
 */
#pragma once

#include <stdbool.h>

#define LOG_TAG "MPU6050"

typedef struct mpu6050 mpu6050_t;

void mpu6050_init(mpu6050_t *mpu);

bool mpu6050_is_initialized(const mpu6050_t *mpu);

float mpu6050_get_acceleration_x(mpu6050_t *mpu);

float mpu6050_get_acceleration_y(mpu6050_t *mpu);

float mpu6050_get_acceleration_z(mpu6050_t *mpu);

float mpu6050_get_angular_velocity_x(mpu6050_t *mpu);

float mpu6050_get_angular_velocity_y(mpu6050_t *mpu);

float mpu6050_get_angular_velocity_z(mpu6050_t *mpu);
