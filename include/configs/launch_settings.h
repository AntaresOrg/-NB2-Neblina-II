/**
 * @file       launch_settings.h
 * @brief      Project launch configuration constants and mission parameters for the Neblina II flight profile.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Header
 * @author     Eletronics Team Members
 *
 * @details
 * This module defines the launch and flight configuration parameters for the Neblina II mission profile.
 * It centralizes the constants and thresholds used to guide launch behavior, flight monitoring, and system
 * configuration across the embedded application. The values are intended to be adjusted for campaign-specific
 * requirements while preserving a consistent integration model across the project.
 */
#pragma once

#include <stdint.h>

// I2C configuration
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_NUM I2C_NUM_0

// Flash Memory Configuration
#define SPI_FLASH_HOST SPI2_HOST

// Data package structure
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;    // Time since boot (ms)
    
    // GPS Data
    int32_t  latitude;        // Degrees * 1e7 (1 cm resolution)
    int32_t  longitude;       // Degrees * 1e7 (1 cm resolution)
    
    // Altimeters
    float    altitude_1;      // Barometer 1 / Primary Altitude (m)
    float    altitude_2;      // Barometer 2 / Secondary Altitude (m)
    
    // Accelerometers (m/s^2)
    float    accel1_x;        // Primary IMU (X)
    float    accel1_y;        // Primary IMU (Y)
    float    accel1_z;        // Primary IMU (Z - Vertical Axis)
    float    accel2_x;        // High-G IMU (X)
    float    accel2_y;        // High-G IMU (Y)
    float    accel2_z;        // High-G IMU (Z - Primary Thrust Axis)
    
    // Gyroscopes (rad/s)
    float    gyro1_x;         // Gyro 1 (Roll)
    float    gyro1_y;         // Gyro 1 (Pitch)
    float    gyro1_z;         // Gyro 1 (Yaw)
    float    gyro2_x;         // Gyro 2 (Roll)
    float    gyro2_y;         // Gyro 2 (Pitch)
    float    gyro2_z;         // Gyro 2 (Yaw)
    
    // System Status
    uint8_t  flight_stage;    // State machine ID (Idle, Boost, Apogee, Main, etc.)
    uint8_t  sats_in_use;     // GPS Satellites locked
    uint8_t  battery_v_x10;   // Battery voltage (e.g., 84 = 8.4V)
    uint8_t  crc8;            // Data integrity check byte
} data_package;          // TOTAL: 64 BYTES
