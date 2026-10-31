/**
 * @file       lora.h
 * @brief      LoRa telemetry interface for radio communication with the E22-900M30S transceiver.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Header
 * @author     git_user_name <git_user_email>
 *
 * @details
 * This module defines the LoRa communication interface for the E22-900M30S transceiver used in the telemetry
 * link. It provides the radio configuration and data transfer routines to transmit flight data and receive
 * operational commands from the ground station. The design is intended to support reliable telemetry exchange
 * in the mission environment.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

typedef struct lora lora_t;

void lora_init(lora_t *lora);

bool lora_is_initialized(const lora_t *lora);

bool lora_is_available(const lora_t *lora);

void lora_set_frequency(lora_t *lora, float frequency);

float lora_get_frequency(lora_t *lora);

void lora_set_bandwidth(lora_t *lora, float bandwidth);

float lora_get_bandwidth(lora_t *lora);

void lora_send_packet(lora_t *lora, const void *data, size_t size);
