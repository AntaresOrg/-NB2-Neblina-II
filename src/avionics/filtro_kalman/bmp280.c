#include "bmp280.h"
#include "i2c_bus.h"
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define REG_ID        0xD0
#define REG_RESET     0xE0
#define REG_CTRL_MEAS 0xF4
#define REG_CONFIG    0xF5
#define REG_PRESS_MSB 0xF7
#define REG_CALIB     0x88

static const char *TAG = "bmp280";

esp_err_t bmp280_init(bmp280_t *b, uint8_t addr)
{
    b->addr = addr;
    uint8_t id = 0;
    esp_err_t err = i2c_read_regs(addr, REG_ID, &id, 1);
    if (err != ESP_OK) return err;
    if (id != 0x58) ESP_LOGW(TAG, "ID=0x%02X (BMP280=0x58; 0x60 seria BME280)", id);

    i2c_write_reg(addr, REG_RESET, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t c[24];
    err = i2c_read_regs(addr, REG_CALIB, c, sizeof(c));
    if (err != ESP_OK) return err;
    b->T1 = (uint16_t)(c[1]  << 8 | c[0]);  b->T2 = (int16_t)(c[3]  << 8 | c[2]);
    b->T3 = (int16_t)(c[5]   << 8 | c[4]);
    b->P1 = (uint16_t)(c[7]  << 8 | c[6]);  b->P2 = (int16_t)(c[9]  << 8 | c[8]);
    b->P3 = (int16_t)(c[11]  << 8 | c[10]); b->P4 = (int16_t)(c[13] << 8 | c[12]);
    b->P5 = (int16_t)(c[15]  << 8 | c[14]); b->P6 = (int16_t)(c[17] << 8 | c[16]);
    b->P7 = (int16_t)(c[19]  << 8 | c[18]); b->P8 = (int16_t)(c[21] << 8 | c[20]);
    b->P9 = (int16_t)(c[23]  << 8 | c[22]);

    /* standby 0.5 ms, IIR x4 */
    ESP_ERROR_CHECK(i2c_write_reg(addr, REG_CONFIG, (0 << 5) | (2 << 2)));
    /* osrs_t x2, osrs_p x16, modo normal  (~23 Hz de saida) */
    ESP_ERROR_CHECK(i2c_write_reg(addr, REG_CTRL_MEAS, (2 << 5) | (5 << 2) | 3));
    return ESP_OK;
}

static int32_t comp_T(bmp280_t *b, int32_t adc_T)
{
    int32_t v1 = ((((adc_T >> 3) - ((int32_t)b->T1 << 1))) * (int32_t)b->T2) >> 11;
    int32_t d  = (adc_T >> 4) - (int32_t)b->T1;
    int32_t v2 = (((d * d) >> 12) * (int32_t)b->T3) >> 14;
    b->t_fine = v1 + v2;
    return (b->t_fine * 5 + 128) >> 8;           /* 0.01 degC */
}

static uint32_t comp_P(const bmp280_t *b, int32_t adc_P)
{
    int64_t v1 = (int64_t)b->t_fine - 128000;
    int64_t v2 = v1 * v1 * (int64_t)b->P6;
    v2 += (v1 * (int64_t)b->P5) << 17;
    v2 += ((int64_t)b->P4) << 35;
    v1 = ((v1 * v1 * (int64_t)b->P3) >> 8) + ((v1 * (int64_t)b->P2) << 12);
    v1 = (((((int64_t)1) << 47) + v1) * (int64_t)b->P1) >> 33;
    if (v1 == 0) return 0;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - v2) * 3125) / v1;
    v1 = ((int64_t)b->P9 * (p >> 13) * (p >> 13)) >> 25;
    v2 = ((int64_t)b->P8 * p) >> 19;
    p = ((p + v1 + v2) >> 8) + (((int64_t)b->P7) << 4);
    return (uint32_t)p;                          /* Pa em Q24.8 */
}

esp_err_t bmp280_read(bmp280_t *b, float *pressure_pa, float *temp_c)
{
    uint8_t d[6];
    esp_err_t err = i2c_read_regs(b->addr, REG_PRESS_MSB, d, sizeof(d));
    if (err != ESP_OK) return err;
    int32_t adc_P = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
    int32_t adc_T = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
    if (adc_P == 0x80000 || adc_T == 0x80000) return ESP_ERR_INVALID_RESPONSE; /* ainda sem amostra */

    int32_t t = comp_T(b, adc_T);
    uint32_t p = comp_P(b, adc_P);
    if (p == 0) return ESP_ERR_INVALID_RESPONSE;
    *temp_c = t / 100.0f;
    *pressure_pa = p / 256.0f;
    return ESP_OK;
}

float bmp280_altitude(float p, float p0)
{
    return 44330.0f * (1.0f - powf(p / p0, 0.190295f));
}
