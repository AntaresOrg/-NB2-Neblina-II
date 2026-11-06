/**
 * @file       flash_memory.c
 * @brief      Source implementation for the external flash storage interface used to save telemetry and recovery data.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Source
 * @author     git_user_name <git_user_email>
 *
 * @details
 * This source file implements the flash memory access logic used to persist telemetry, configuration, and
 * recovery data in non-volatile storage. It provides the read and write operations needed to save mission
 * information reliably and maintain data integrity across the embedded system. The implementation supports the
 * project's requirement for flight-data retention and post-flight analysis.
 */
#include "avionics/flash_memory.h"

#define LOG_TAG "FLASH_MEMORY"
