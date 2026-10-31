/**
 * @file       bmp280.h
 * @brief      Driver for the BMP280 barometer used to measure atmospheric pressure and temperature.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Header
 * @author     git_user_name <git_user_email>
 *
 * @details
 * This module handles the BMP280 pressure and temperature sensor used in the rocket avionics stack.
 * It provides the calibration and conversion routines required to estimate altitude, monitor atmospheric
 * conditions, and support flight-state decisions during ascent and descent. The implementation is designed
 * for the ESP32 platform and integrates with the project's sensor acquisition workflow.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

// BMP280 ADDRESSES
#define BMP280_ADDR_1 0x76
#define BMP280_ADDR_2 0x77

typedef struct bmp280 bmp280_t;

bmp280_t *bmp280_init(uint8_t address, const char *name);

bool bmp280_is_initialized(const bmp280_t *bmp);

float bmp280_get_pressure(bmp280_t *bmp);

float bmp280_get_temperature(bmp280_t *bmp);

float bmp280_get_relative_altitude(bmp280_t *bmp);

float bmp280_get_absolute_altitude(bmp280_t *bmp);