/**
 * @file       logs.h
 * @brief      Logging utilities for structured system and telemetry messages during flight operations.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Header
 * @author     git_user_name <git_user_email>
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
    LOG_LEVEL_ERROR = 2
} log_level_t;

void log_write(const char *tag, log_level_t level, const char *fmt, ...);

#define LOG(tag, level, fmt, ...) \
    log_write(tag, level, fmt, ##__VA_ARGS__)
