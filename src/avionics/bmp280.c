/**
 * @file       bmp280.c
 * @brief      Source implementation for the BMP280 barometer driver and pressure/temperature conversion logic.
 *
 * @organization Antares Foguetemodelismo
 * @year         2026
 * 
 * @type       Source
 * @author     git_user_name <git_user_email>
 *
 * @details
 * This source file implements the BMP280 sensor driver and the conversion routines used to read pressure and
 * temperature values from the device. It also supports altitude estimation based on atmospheric pressure,
 * allowing the system to derive vertical position and flight progression during operation. The implementation
 * follows the project's embedded sensor acquisition model for ESP32-based avionics.
 */

#include "bmp280.h" //dps vou tentar colocar no header
#include <stdio.h>
#include <stdbool.h>

// Definição do tipo para o callback 
typedef void (*sensor_data_callback_t)(float pressure_pa, float temperature_c, float altitude_m, bool success);

// Estrutura para gerizar o estado do sensor no foguete
typedef struct {
    bmp280_handle_t dev_handle;
    volatile bool data_sent_flag;  // Flag que avisa quando o dado acabou de ser transmitido
    sensor_data_callback_t callback;
} rocket_altimeter_t;

// Instância principal do nosso altímetro
static rocket_altimeter_t rocket_altimeter;

// Callback chamado assim que a leitura e o envio ficam prontos
static void internal_data_callback(float pressure_pa, float temperature_c, float altitude_m, bool success) {
    if (success) {
        // Ativamos a flag para o ESP32 saber que o ciclo terminou com sucesso
        rocket_altimeter.data_sent_flag = true;
        
        printf("[Telemetry] Pressão: %.2f Pa | Temp: %.2f °C | Altura: %.2f m\n", 
               pressure_pa, temperature_c, altitude_m);
    } else {
        rocket_altimeter.data_sent_flag = false;
        printf("[Erro] Opa, deu ruim na leitura do BMP280!\n");
    }
}

/**
 * @brief Configura o BMP280 focado no nosso perfil de voo (Modo Normal, Filtro IIR )
 */
bool rocket_bmp280_init(void) {
    bmp280_error_code_t err;

    // 1. Inicializa via I2C no endereço padrão do sensor (0x76)
    err = bmp280_init(&rocket_altimeter.dev_handle, BMP280_HW_INTERFACE_I2C, BMP280_I2C_ADDRESS_DEFAULT);
    if (err != 0) return false;

    // 2. Ajusta o oversampling para alta precisão (essencial para pegar os 3km de apogeu)
    err = bmp280_set_pressure_oversampling(&rocket_altimeter.dev_handle, BMP280_OVERSAMPLING_16X);
    if (err != 0) return false;

    err = bmp280_set_temperature_oversampling(&rocket_altimeter.dev_handle, BMP280_OVERSAMPLING_2X);
    if (err != 0) return false;

    // 3. Ativa o filtro IIR (COLOQUEI VALOR ALEATORIO)
    err = bmp280_set_filter_coefficient(&rocket_altimeter.dev_handle, BMP280_IIR_FILTER_16);
    if (err != 0) return false;

    // 4. Tempo de standby (configurar depois poensando em sincronizar com o mpu)
    err = bmp280_set_standby_time(&rocket_altimeter.dev_handle, BMP280_STANDBY_TIME_0_5_MS);
    if (err != 0) return false;

    // 5. Joga o sensor em modo contínuo (Normal Mode)
    err = bmp280_set_mode(&rocket_altimeter.dev_handle, BMP280_MODE_NORMAL);
    if (err != 0) return false;

    // Preenche as estruturas de controlo
    rocket_altimeter.data_sent_flag = false;
    rocket_altimeter.callback = internal_data_callback;

    return true;
}

/**
 * @brief Função para chamar no loop principal do ESP32 / FreeRTOS
 */
void rocket_bmp280_task_iteration(void) {
    float temperature = 0.0f;
    uint32_t pressure = 0;
    float altitude = 0.0f;
    bmp280_error_code_t err;

    // Reseta a flag antes de começar a puxar dados novos
    rocket_altimeter.data_sent_flag = false;

    // Pega a temperatura
    err = bmp280_get_temperature(&rocket_altimeter.dev_handle, &temperature);
    if (err != 0) {
        if (rocket_altimeter.callback) rocket_altimeter.callback(0, 0, 0, false);
        return;
    }

    // Pega a pressão em Pascals
    err = bmp280_get_pressure(&rocket_altimeter.dev_handle, &pressure);
    if (err != 0) {
        if (rocket_altimeter.callback) rocket_altimeter.callback(0, 0, 0, false);
        return;
    }

    // Tenta calcular a altitude usando a fórmula hipsométrica (mais precisa com temperatura)
    err = bmp280_calculate_altitude_hypsometric(&rocket_altimeter.dev_handle, &altitude, pressure, temperature);
    if (err != 0) {
        // Se falhar, recorre ao cálculo rápido
        err = bmp280_calculate_altitude_quick(&rocket_altimeter.dev_handle, &altitude, pressure);
        if (err != 0) altitude = 0.0f;
    }

    // Sucesso total! Dispara o callback informando que o dado está pronto/enviado
    if (rocket_altimeter.callback) {
        rocket_altimeter.callback((float)pressure, temperature, altitude, true);
    }
}

int main(void) {
    if (!rocket_bmp280_init()) {
        printf("Erro: O BMP280 não quis responder na inicialização.\n");
        return -1;
    }

    printf("Tudo pronto! Sensor configurado para a missão do foguete.\n");

    while (1) {
        rocket_bmp280_task_iteration();

        // Se a flag acendeu, o dado foi processado e enviado com sucesso
        if (rocket_altimeter.data_sent_flag) {
            // isso aqui seria a parte de "fusão" do bmp com o mpu, acho que é melhor colocar em outro arquivo, mas, por enquanto, vou manter aqui 
        }
    }

    return 0;
}
#include "avionics/bmp280.h"
