/*
 * ESP32 + MPU6050 + BMP280 : EKF de atitude + altitude com correcao de drift
 * Loop de 100 Hz; barometro a 20 Hz.
 */
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "i2c_bus.h"
#include "mpu6050.h"
#include "bmp280.h"
#include "ekf.h"

#define PIN_SDA        21
#define PIN_SCL        22
#define I2C_HZ         400000
#define BMP280_ADDR    BMP280_ADDR_LOW

#define LOOP_MS        10      /* 100 Hz */
#define BARO_DIV       5       /* 20 Hz  */
#define PRINT_DIV      10      /* 10 Hz  */
#define CALIB_SAMPLES  200     /* 2 s parado no boot */
#define RAD2DEG        57.29577951308232f

static const char *TAG = "main";

static void ekf_task(void *arg)
{
    bmp280_t bmp;
    float gyro[3], acc[3];

    if (bmp280_init(&bmp, BMP280_ADDR) != ESP_OK) {
        ESP_LOGE(TAG, "BMP280 nao encontrado em 0x%02X (tente BMP280_ADDR_HIGH)", BMP280_ADDR);
        vTaskDelete(NULL);
    }

    /* ---------- calibracao inicial (sensor PARADO) ---------- */
    ESP_LOGI(TAG, "Calibrando %d amostras - mantenha o sensor parado...", CALIB_SAMPLES);
    float gsum[3] = {0}, asum[3] = {0}, psum = 0;
    int nb = 0, nm = 0;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        if (mpu6050_read(gyro, acc) == ESP_OK) {
            for (int k = 0; k < 3; k++) { gsum[k] += gyro[k]; asum[k] += acc[k]; }
            nm++;
        }
        if (i % BARO_DIV == 0) {
            float p, t;
            if (bmp280_read(&bmp, &p, &t) == ESP_OK) { psum += p; nb++; }
        }
        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
    if (nm == 0 || nb == 0) {
        ESP_LOGE(TAG, "Falha na calibracao (mpu=%d, baro=%d)", nm, nb);
        vTaskDelete(NULL);
    }
    float gmean[3], amean[3];
    for (int k = 0; k < 3; k++) { gmean[k] = gsum[k]/nm; amean[k] = asum[k]/nm; }
    float p0 = psum / nb;

    ekf_params_t par;
    ekf_params_default(&par);
    ekf_t ekf;
    ekf_init(&ekf, &par, amean, gmean);
    ESP_LOGI(TAG, "bias gyro inicial [rad/s]: %.5f %.5f %.5f | P0=%.1f Pa",
             gmean[0], gmean[1], gmean[2], p0);

    /* ---------- loop principal ---------- */
    TickType_t last = xTaskGetTickCount();
    int64_t t_prev = esp_timer_get_time();
    unsigned cnt = 0;

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(LOOP_MS));

        int64_t t_now = esp_timer_get_time();
        float dt = (t_now - t_prev) * 1e-6f;
        t_prev = t_now;
        if (dt < 0.002f) dt = 0.002f;
        if (dt > 0.05f)  dt = 0.05f;

        if (mpu6050_read(gyro, acc) != ESP_OK) continue;

        ekf_predict(&ekf, gyro, acc, dt);        /* integra gyro/acc, propaga P */
        ekf_update_accel(&ekf, acc);             /* corrige drift de roll/pitch */
        ekf_update_zupt(&ekf, gyro, acc);        /* corrige bias do gyro em repouso */

        if (cnt % BARO_DIV == 0) {
            float p, t;
            if (bmp280_read(&bmp, &p, &t) == ESP_OK)
                ekf_update_baro(&ekf, bmp280_altitude(p, p0));
        }

        if (cnt % PRINT_DIV == 0) {
            float r, pi, y;
            ekf_get_euler(&ekf, &r, &pi, &y);
            printf("RPY[deg] %7.2f %7.2f %7.2f | alt %6.2f m  vz %5.2f m/s | bg[mrad/s] %6.2f %6.2f %6.2f | ba %5.3f%s\n",
                   r*RAD2DEG, pi*RAD2DEG, y*RAD2DEG, ekf.z, ekf.vz,
                   ekf.bg[0]*1e3f, ekf.bg[1]*1e3f, ekf.bg[2]*1e3f, ekf.ba,
                   ekf.still_count >= par.still_samples ? " [ZUPT]" : "");
        }
        cnt++;
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_bus_init(PIN_SDA, PIN_SCL, I2C_HZ));
    ESP_ERROR_CHECK(mpu6050_init());

    xTaskCreatePinnedToCore(ekf_task, "ekf", 6144, NULL, 5, NULL, 1);
}
