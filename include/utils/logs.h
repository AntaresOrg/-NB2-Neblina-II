/**
 * @file       logs.h
 * @brief      Logging utilities for structured system and telemetry messages during flight operations.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Header
 * @author     RafaelCarro rafaelcgaudim@yahoo.com.br
 *
 * @details
 * This module provides the logging utilities used to record structured system and telemetry messages during
 * flight operations. It supports message formatting, trace output, and debugging information needed for
 * runtime monitoring, fault diagnosis, and mission analysis. The design keeps the logging interface consistent
 * across the embedded firmware.
 */
#pragma once

#include <stdarg.h>

typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_WARNING = 1,
    LOG_LEVEL_ERROR = 2,
    LOG_LEVEL_INFO = 3
} log_level_t;

void log_write(const char *tag, log_level_t level, const char *fmt, ...);

#define LOGD(tag, fmt, ...) \
    log_write(tag, LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)

#define LOGW(tag, fmt, ...) \
    log_write(tag, LOG_LEVEL_WARNING, fmt, ##__VA_ARGS__)

#define LOGE(tag, fmt, ...) \
    log_write(tag, LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)

#define LOGI(tag, fmt, ...) \
    log_write(tag, LOG_LEVEL_INFO, fmt, ##__VA_ARGS__)
