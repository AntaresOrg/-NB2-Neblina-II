/**
 * @file       i2c.h
 * @brief      Shared I2C bus helpers and configuration for communication with onboard sensors.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Header
 * @author     git_user_name <git_user_email>
 *
 * @details
 * This module centralizes the I2C bus configuration used by the onboard avionics sensors. It provides the
 * shared initialization, device addressing, and communication routines required for sensors such as the BMP280,
 * MPU6050, and other I2C-based peripherals. The implementation is designed to keep the embedded system
 * communication layer consistent and reusable across the project.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c.h"

#define LOG_TAG "I2C"

bool initialize_i2c(void);

bool i2c_is_initialized(void);

uint16_t read_u16(const uint8_t *data);

int16_t read_i16(const uint8_t *data);

esp_err_t read_register(
    i2c_port_t i2c_port,
    uint8_t address,
    uint8_t reg,
    uint8_t *data,
    size_t length
);

esp_err_t write_register(
    i2c_port_t i2c_port,
    uint8_t address,
    uint8_t reg,
    uint8_t value
);
