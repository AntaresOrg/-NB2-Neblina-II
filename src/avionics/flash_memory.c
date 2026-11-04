/**
 * @file       flash_memory.c
 * @brief      Source implementation for the external flash storage interface used to save telemetry and recovery data.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Source
 * @author     RafaelCarro rafaelcgaudim@yahoo.com.br
 *
 * @details
 * This source file implements the flash memory access logic used to persist telemetry, configuration, and
 * recovery data in non-volatile storage. It provides the read and write operations needed to save mission
 * information reliably and maintain data integrity across the embedded system. The implementation supports the
 * project's requirement for flight-data retention and post-flight analysis.
 */
#include "avionics/flash_memory.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "esp_err.h"
#include "esp_flash_spi_init.h"
#include "esp_flash.h"
#include "configs/launch_settings.h"
#include "configs/pins.h"
#include "driver/spi_master.h"
#include "utils/logs.h"

#define LOG_TAG "FLASH_MEMORY"
#define FLIGHT_LOG_MARKER 0x4E424C47U

typedef struct __attribute__((packed)) {
    uint32_t marker;
} flight_log_marker_t;

struct flash_memory {
    uint32_t free_address;
    uint32_t flash_size;
    bool is_initialized;
    spi_bus_config_t flash_memory_spi_config;
    esp_flash_t* ext_flash;
};

static uint32_t find_free_memory(const flash_memory_t *flash) {
    uint32_t address = 0;
    uint32_t record_size = sizeof(data_package);
    flight_log_marker_t marker;

    while (address + sizeof(marker) <= flash->flash_size) {
        esp_err_t err = esp_flash_read(flash->ext_flash, &marker, address,
                                       sizeof(marker));
        if (err != ESP_OK) {
            LOGE(LOG_TAG, "Failed to scan flash at address 0x%08x: %s",
                 (unsigned int)address, esp_err_to_name(err));
            return UINT32_MAX;
        }

        if (marker.marker == 0xFFFFFFFFU) {
            return address;
        }

        if (marker.marker == FLIGHT_LOG_MARKER) {
            address += sizeof(marker);
        } else {
            if (address + record_size > flash->flash_size) {
                break;
            }
            address += record_size;
        }
    }

    LOGE(LOG_TAG, "Flash memory is full");
    return UINT32_MAX;
}

static void flash_memory_write_log_marker(flash_memory_t *flash_memory);

flash_memory_t *flash_memory_init() {
    // Creating the Flash Memory
    flash_memory_t *flash_memory = calloc(1, sizeof(flash_memory_t));

    if (!flash_memory) {
        LOGE(LOG_TAG, "Error allocating memory for the flash memory!");
        return NULL;
    }

    flash_memory->is_initialized = false;
    flash_memory->free_address = UINT32_MAX;
    flash_memory->flash_size = 0;
    flash_memory->ext_flash = NULL;

    // Configuration for the W25Q128 Flash Memory
    spi_bus_config_t bus_config = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096, // Maximum transfer size in bytes
    };

    flash_memory->flash_memory_spi_config = bus_config;

    // Initialize the SPI Bus
    esp_err_t err = spi_bus_initialize(SPI_FLASH_HOST, &flash_memory->flash_memory_spi_config, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        LOGE(LOG_TAG, "Failed to initialize the SPI bus: %s", esp_err_to_name(err));
        return NULL;
    }

    // Configuration for the Flash Device on the Bus
    esp_flash_spi_device_config_t device_config =  {
        .host_id = SPI_FLASH_HOST,
        .cs_io_num = SPI_FLASH_CS,
        .io_mode = SPI_FLASH_FASTRD,
        .freq_mhz = 40,
        .cs_id = 0,
    };

    // Adding the flash memory to the SPI Bus
    err = spi_bus_add_flash_device(&flash_memory->ext_flash, &device_config);
    if (err != ESP_OK) {
        LOGE(LOG_TAG, "Failed to add the flash into the SPI bus: %s", esp_err_to_name(err));
        return NULL;
    }

    // Initializing Flash Memory
    err = esp_flash_init(flash_memory->ext_flash);
    if (err != ESP_OK) {
        LOGE(LOG_TAG, "Failed to initialize external Flash Memory: %s", esp_err_to_name(err));
        return NULL;
    }

    // Checking if the module is succesfully initialized
    uint32_t id;
    err = esp_flash_read_id(flash_memory->ext_flash, &id);
    if (err != ESP_OK) {
        LOGE(LOG_TAG, "Failed to read Flash Memory ID: %s", esp_err_to_name(err));
        return NULL;
    }
    LOGI(LOG_TAG, "External flash initialized successfully. Chip ID: 0x%06x",
         (unsigned int)id);

    err = esp_flash_get_physical_size(flash_memory->ext_flash, &flash_memory->flash_size);
    if (err != ESP_OK) {
        LOGE(LOG_TAG, "Failed to get flash size: %s", esp_err_to_name(err));
        return NULL;
    }

    flash_memory->free_address = find_free_memory(flash_memory);
    flash_memory->is_initialized = true;
    flash_memory_write_log_marker(flash_memory);

    return flash_memory;
}

bool flash_memory_is_initialized(const flash_memory_t *flash_memory) {
    return flash_memory != NULL && flash_memory->is_initialized;
}

void flash_memory_write(flash_memory_t *flash_memory, const void *data, size_t size) {
    if (flash_memory == NULL || !flash_memory->is_initialized) {
        LOGE(LOG_TAG, "Cannot write to an uninitialized flash memory");
        return;
    }

    if (size == 0) {
        return;
    }

    if (data == NULL) {
        LOGE(LOG_TAG, "Cannot write NULL data");
        return;
    }

    if (size > (size_t)UINT32_MAX) {
        LOGE(LOG_TAG, "Write size is too large: %u bytes", (unsigned int)size);
        return;
    }

    if (flash_memory->free_address == UINT32_MAX ||
        flash_memory->free_address > flash_memory->flash_size ||
        (uint32_t)size > flash_memory->flash_size - flash_memory->free_address) {
        LOGE(LOG_TAG, "Not enough space to write %u bytes at address 0x%08x",
             (unsigned int)size, (unsigned int)flash_memory->free_address);
        return;
    }

    esp_err_t err = esp_flash_write(flash_memory->ext_flash, data, flash_memory->free_address,
                                    (uint32_t)size);
    if (err != ESP_OK) {
        LOGE(LOG_TAG, "Failed to write flash at address 0x%08x: %s",
             (unsigned int)flash_memory->free_address, esp_err_to_name(err));
        return;
    }

    flash_memory->free_address += (uint32_t)size;
}

static void flash_memory_write_log_marker(flash_memory_t *flash_memory) {
    const flight_log_marker_t marker = {
        .marker = FLIGHT_LOG_MARKER,
    };

    flash_memory_write(flash_memory, &marker, sizeof(marker));
}

void flash_memory_save_flight_package(flash_memory_t *flash_memory, data_package flight_package) {
    flash_memory_write(flash_memory, &flight_package, sizeof(flight_package));
}

void flash_memory_read(flash_memory_t *flash_memory, void *buffer, size_t size) {
    if (flash_memory == NULL || !flash_memory->is_initialized) {
        LOGE(LOG_TAG, "Cannot read from an uninitialized flash memory");
        return;
    }

    if (size == 0) {
        return;
    }

    if (buffer == NULL) {
        LOGE(LOG_TAG, "Cannot read into a NULL buffer");
        return;
    }

    if (size > (size_t)UINT32_MAX) {
        LOGE(LOG_TAG, "Read size is too large: %u bytes", (unsigned int)size);
        return;
    }

    if ((uint32_t)size > flash_memory->flash_size) {
        LOGE(LOG_TAG, "Read size exceeds flash capacity: %u bytes", (unsigned int)size);
        return;
    }

    esp_err_t err = esp_flash_read(flash_memory->ext_flash, buffer, 0, (uint32_t)size);
    if (err != ESP_OK) {
        LOGE(LOG_TAG, "Failed to read flash: %s", esp_err_to_name(err));
    }
}

void flash_memory_reset(flash_memory_t *flash_memory) {
    if (flash_memory == NULL || !flash_memory->is_initialized) {
        LOGE(LOG_TAG, "Cannot reset an uninitialized flash memory");
        return;
    }

    esp_err_t err = esp_flash_erase_chip(flash_memory->ext_flash);
    if (err != ESP_OK) {
        LOGE(LOG_TAG, "Failed to erase flash memory: %s", esp_err_to_name(err));
        return;
    }

    flash_memory->free_address = 0;
    flash_memory_write_log_marker(flash_memory);
}

void flash_memory_read_flight_data(flash_memory_t *flash_memory) {
    uint32_t address = 0;
    uint32_t flight_log_number = 0;
    flight_log_marker_t marker;
    data_package flight_package;

    if (flash_memory == NULL || !flash_memory->is_initialized) {
        LOGE(LOG_TAG, "Cannot read flight data from an uninitialized flash memory");
        return;
    }

    LOGI(LOG_TAG, "========== FLIGHT DATA ==========");

    while (address < flash_memory->free_address) {
        if (flash_memory->free_address - address < sizeof(marker)) {
            LOGE(LOG_TAG, "Incomplete flight log marker at address 0x%08x",
                 (unsigned int)address);
            return;
        }

        esp_err_t err = esp_flash_read(flash_memory->ext_flash, &marker, address,
                                       sizeof(marker));
        if (err != ESP_OK) {
            LOGE(LOG_TAG, "Failed to read flight log marker at address 0x%08x: %s",
                 (unsigned int)address, esp_err_to_name(err));
            return;
        }

        if (marker.marker == FLIGHT_LOG_MARKER) {
            ++flight_log_number;
            LOGI(LOG_TAG, "=========== FLIGHT LOG %u ===========",
                 (unsigned int)flight_log_number);
            LOGI(LOG_TAG, "time (s), latitude, longitude, altitude 1 (m), altitude 2 (m), "
                          "accel1 x, accel1 y, accel1 z, accel2 x, accel2 y, accel2 z, "
                          "gyro1 x, gyro1 y, gyro1 z, gyro2 x, gyro2 y, gyro2 z, "
                          "flight stage, satellites, battery (V), crc8");
            address += sizeof(marker);
            continue;
        }

        if (flash_memory->free_address - address < sizeof(flight_package)) {
            LOGE(LOG_TAG, "Incomplete flight package at address 0x%08x",
                 (unsigned int)address);
            return;
        }

        err = esp_flash_read(flash_memory->ext_flash, &flight_package, address,
                             sizeof(flight_package));
        if (err != ESP_OK) {
            LOGE(LOG_TAG, "Failed to read flight package at address 0x%08x: %s",
                 (unsigned int)address, esp_err_to_name(err));
            return;
        }

        LOGI(LOG_TAG,
             "%u, %ld, %ld, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, "
             "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %u, %u, %.1f, %u",
             (unsigned int)(flight_package.timestamp_ms / 1000U),
             (long)flight_package.latitude,
             (long)flight_package.longitude,
             (double)flight_package.altitude_1,
             (double)flight_package.altitude_2,
             (double)flight_package.accel1_x,
             (double)flight_package.accel1_y,
             (double)flight_package.accel1_z,
             (double)flight_package.accel2_x,
             (double)flight_package.accel2_y,
             (double)flight_package.accel2_z,
             (double)flight_package.gyro1_x,
             (double)flight_package.gyro1_y,
             (double)flight_package.gyro1_z,
             (double)flight_package.gyro2_x,
             (double)flight_package.gyro2_y,
             (double)flight_package.gyro2_z,
             (unsigned int)flight_package.flight_stage,
             (unsigned int)flight_package.sats_in_use,
             (double)flight_package.battery_v_x10 / 10.0,
             (unsigned int)flight_package.crc8);
        address += sizeof(flight_package);
    }
}
