/**
 * @file       pins.h
 * @brief      GPIO and peripheral pin mapping used by the flight electronics and telemetry hardware.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Header
 * @author     Eletronics Team Members
 *
 * @details
 * This module contains the GPIO and peripheral pin mapping for the rocket's avionics and telemetry hardware.
 * It centralizes the board-level assignments used to interconnect sensors, radios, and auxiliary devices,
 * keeping the embedded firmware organized and reducing wiring-related configuration errors. The definitions
 * are intended to support the ESP32-based hardware layout of the Neblina II project.
 */
// I2C Pins
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21

// SPI Pins
#define SPI_MISO 19
#define SPI_MOSI 23
#define SPI_CLK 18

// Flash Memory Pins
#define SPI_FLASH_CS 5
