/**
 * @file       gps.c
 * @brief      Source implementation for GPS acquisition, parsing, and positioning data handling.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Source
 * @author     git_user_name <git_user_email>
 *
 * @details
 * This source file implements the GPS acquisition logic for the NEO-M9N receiver, including its configuration,
 * communication with the module, and parsing of position and timing data. It converts the received information
 * into usable telemetry values for monitoring the rocket's state and trajectory. The implementation supports the
 * mission's navigation and tracking requirements.
 */
#include "telemetry/gps.h"
