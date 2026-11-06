/**
 * @file       i2c.c
 * @brief      Source implementation for the shared I2C bus operations and device communication helpers.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Source
 * @author     RafaelCarro rafaelcgaudim@yahoo.com.br
 *
 * @details
 * This source file implements the shared I2C communication routines used by the onboard sensors and peripherals.
 * It contains the bus initialization and transaction logic required for reliable device communication across the
 * embedded system. The implementation keeps all sensor interactions consistent and reusable for the Neblina II
 * avionics architecture.
 */
#include "utils/i2c.h"

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c.h"
#include "configs/pins.h"
#include "configs/launch_settings.h"
#include "utils/logs.h"

#define LOG_TAG "I2C"

static bool i2c_initialized = false;

bool initialize_i2c(void) {
    if (i2c_initialized) {
        LOGW(LOG_TAG, "I2C already initialized");
        return true;
    }

    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0
    };

    esp_err_t error = i2c_param_config(I2C_MASTER_NUM, &config);
    if (error != ESP_OK) {
        LOGE(LOG_TAG,
            "I2C parameter configuration failed: %s",
            esp_err_to_name(error));
        return false;
    }

    error = i2c_driver_install(
            I2C_MASTER_NUM,
            I2C_MODE_MASTER,
            0,
            0,
            0);
    if (error != ESP_OK) {
        LOGE(LOG_TAG,
            "I2C driver installation failed: %s",
            esp_err_to_name(error));
        return false;
    }

    i2c_initialized = true;
    LOGI(LOG_TAG, "I2C initialized succesfully");
    return true;
}

bool i2c_is_initialized(void) {
    return i2c_initialized;
}

uint16_t read_u16(const uint8_t *data) {
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

int16_t read_i16(const uint8_t *data) {
    return (int16_t)data[0] | ((int16_t)data[1] << 8);
}

esp_err_t read_register(
    i2c_port_t i2c_port,
    uint8_t address,
    uint8_t reg,
    uint8_t *data,
    size_t length
) {
    esp_err_t error = i2c_master_write_read_device(
        i2c_port,
        address,
        &reg,
        1,
        data,
        length,
        pdMS_TO_TICKS(100)
    );

    if (error != ESP_OK) {
        LOGE(LOG_TAG,
            "I2C read failed (port=%d, address=0x%02X, register=0x%02X, length=%u): %s",
            i2c_port, address, reg, (unsigned int)length, esp_err_to_name(error));
    }

    return error;
}

esp_err_t write_register(
    i2c_port_t i2c_port,
    uint8_t address,
    uint8_t reg,
    uint8_t value
) {
    uint8_t buffer[2] = {reg, value};
    esp_err_t error = i2c_master_write_to_device(
        i2c_port,
        address,
        buffer,
        sizeof(buffer),
        pdMS_TO_TICKS(100)
    );

    if (error != ESP_OK) {
        LOGE(LOG_TAG,
            "I2C write failed (port=%d, address=0x%02X, register=0x%02X, value=0x%02X): %s",
            i2c_port, address, reg, value, esp_err_to_name(error));
    }

    return error;
}