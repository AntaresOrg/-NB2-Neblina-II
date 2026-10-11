/**
 * @file       bmp280.c
 * @brief      Source implementation for the BMP280 barometer driver and pressure/temperature conversion logic.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Source
 * @author     git_user_name <git_user_email>
 *
 * @details
 * This source file implements the BMP280 sensor driver and the conversion routines used to read pressure and
 * temperature values from the device. It also supports altitude estimation based on atmospheric pressure,
 * allowing the system to derive vertical position and flight progression during operation. The implementation
 * follows the project's embedded sensor acquisition model for ESP32-based avionics.
 */
#include "avionics/bmp280.h"
