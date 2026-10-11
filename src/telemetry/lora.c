/**
 * @file       lora.c
 * @brief      Source implementation for the LoRa transceiver used to send telemetry and receive commands.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Source
 * @author     git_user_name <git_user_email>
 *
 * @details
 * This source file implements the LoRa telemetry driver for the E22-900M30S transceiver. It manages the radio
 * setup, packet transmission, and data exchange used to relay telemetry from the rocket to the ground station.
 * The implementation is designed to support reliable, low-bandwidth communication during the mission.
 */
#include "telemetry/lora.h"
