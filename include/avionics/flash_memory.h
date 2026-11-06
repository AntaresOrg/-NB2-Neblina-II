/**
 * @file       flash_memory.h
 * @brief      Interface for the external flash memory used to persist flight data and telemetry logs.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Header
 * @author     git_user_name <git_user_email>
 *
 * @details
 * This module manages the external flash memory used to store telemetry, flight data, and recovery logs.
 * It provides the storage interface, read/write operations, and data-handling routines needed to persist
 * mission information reliably in the embedded system. The implementation is designed for the project's
 * non-volatile memory requirements and high-reliability flight data logging.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

typedef struct flash_memory flash_memory_t;

void flash_memory_init(flash_memory_t *flash);

bool flash_memory_is_initialized(const flash_memory_t *flash);

void flash_memory_write(flash_memory_t *flash, const void *data, size_t size);

void flash_memory_read(flash_memory_t *flash, void *buffer, size_t size);

void flash_memory_reset(flash_memory_t *flash);

void flash_memory_dump(flash_memory_t *flash);
