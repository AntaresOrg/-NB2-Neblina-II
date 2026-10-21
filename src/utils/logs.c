/**
 * @file       logs.c
 * @brief      Source implementation for structured flight logs and operational message handling.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Source
 * @author     RafaelCarro rafaelcgaudim@yahoo.com.br
 *
 * @details
 * This source file implements the project's logging utilities used to produce structured runtime and telemetry
 * messages. It defines the formatting and output logic needed to monitor the system, diagnose faults, and review
 * mission data after flight. The implementation supports clear operational visibility across the embedded stack.
 */
#include "utils/logs.h"

#include <stdio.h>
#include <stdarg.h>
#include "esp_timer.h"

// ANSI Color Constants
#define LOG_COLOR_RESET   "\x1b[0m"
#define LOG_COLOR_INFO    "\x1b[0;32m" // Green
#define LOG_COLOR_WARN    "\x1b[0;33m" // Yellow
#define LOG_COLOR_ERROR   "\x1b[0;31m" // Red
#define LOG_COLOR_DEBUG   "\x1b[0;34m" // Blue

void log_write(const char *tag, log_level_t level, const char *fmt, ...) {
    uint32_t millis = (uint32_t)(esp_timer_get_time() / 1000);
    printf("%8lu ms | ", millis);

    switch (level) {
        case LOG_LEVEL_DEBUG:
            printf("[%sDEBUG%s]", LOG_COLOR_DEBUG, LOG_COLOR_RESET);
            break;
        case LOG_LEVEL_WARNING:
            printf("[%sWARN%s]", LOG_COLOR_WARN, LOG_COLOR_RESET);
            break;
        case LOG_LEVEL_ERROR:
            printf("[%sERROR%s]", LOG_COLOR_ERROR, LOG_COLOR_RESET);
            break;
        case LOG_LEVEL_INFO:
            printf("[%sINFO%s]", LOG_COLOR_INFO, LOG_COLOR_RESET);
            break;
    }

    if (tag != NULL) {
        printf(" [%s]: ", tag);
    } else {
        printf(": ");
    }

    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    printf("\n");
}
