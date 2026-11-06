/**
 * @file       launch_settings.h
 * @brief      Project launch configuration constants and mission parameters for the Neblina II flight profile.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Header
 * @author     Eletronics Team Members
 *
 * @details
 * This module defines the launch and flight configuration parameters for the Neblina II mission profile.
 * It centralizes the constants and thresholds used to guide launch behavior, flight monitoring, and system
 * configuration across the embedded application. The values are intended to be adjusted for campaign-specific
 * requirements while preserving a consistent integration model across the project.
 */
// I2C configuration
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_NUM I2C_NUM_0