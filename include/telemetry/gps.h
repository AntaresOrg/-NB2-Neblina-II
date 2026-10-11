/**
 * @file       gps.h
 * @brief      GNSS interface for the NEO-M9N module used to acquire position, speed, and time data.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Header
 * @author     git_user_name <git_user_email>
 *
 * @details
 * This module provides the interface for the NEO-M9N GNSS receiver used to acquire position, speed, and time
 * information. It encapsulates the configuration and parsing logic needed to read satellite data and share it
 * with the telemetry and flight monitoring system. The implementation supports the mission requirements for
 * tracking and navigation during the rocket flight profile.
 */
#pragma once

#include <stdbool.h>

#define LOG_TAG "GPS"

typedef struct m9n m9n_t;

void m9n_init(m9n_t *m9n);

bool m9n_is_initialized(const m9n_t *m9n);

int m9n_get_satellite_count(m9n_t *m9n);

float m9n_get_latitude(m9n_t *m9n);

float m9n_get_longitude(m9n_t *m9n);

float m9n_get_altitude(m9n_t *m9n);
