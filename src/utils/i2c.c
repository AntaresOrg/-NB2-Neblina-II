/**
 * @file       i2c.c
 * @brief      Source implementation for the shared I2C bus operations and device communication helpers.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Source
 * @author     git_user_name <git_user_email>
 *
 * @details
 * This source file implements the shared I2C communication routines used by the onboard sensors and peripherals.
 * It contains the bus initialization and transaction logic required for reliable device communication across the
 * embedded system. The implementation keeps all sensor interactions consistent and reusable for the Neblina II
 * avionics architecture.
 */
#include "utils/i2c.h"
