#pragma once
/*
 * EKF multiplicativo (error-state) para MPU6050 + BMP280.
 *
 * Estado nominal : q (body->nav, [w x y z]), bg[3], z, vz, ba
 * Estado de erro : [dtheta(3) | dbg(3) | dz | dvz | dba]  (9 estados)
 *
 * Frame nav: Z para cima.  Acelerometro mede forca especifica (em repouso: +g em Z).
 * Sem magnetometro o YAW NAO E OBSERVAVEL: so e estabilizado pela estimativa de bias (ZUPT).
 */
#include <stdbool.h>

#define EKF_N 9

typedef struct {
    /* ruido de processo (densidades, valores 1-sigma por sqrt(s) ou por s) */
    float gyro_noise;      /* rad/s        */
    float gyro_bias_rw;    /* rad/s/sqrt(s)*/
    float acc_noise;       /* m/s^2        */
    float acc_bias_rw;     /* m/s^2/sqrt(s)*/
    float z_noise;         /* m/sqrt(s)    */
    /* ruido de medicao */
    float acc_sigma;       /* direcao da gravidade (adimensional) */
    float baro_sigma;      /* m                                  */
    float zupt_gyro_sigma; /* rad/s                              */
    float zupt_vel_sigma;  /* m/s                                */
    /* acelerometro: rejeicao / covariance inflation */
    float acc_reject;      /* |‖f‖-g| acima disso => ignora      */
    float acc_inflate;     /* R *= 1 + inflate*dev^2             */
    /* deteccao de repouso */
    float still_gyro;      /* rad/s */
    float still_acc;       /* m/s^2 */
    int   still_samples;
} ekf_params_t;

typedef struct {
    float q[4];
    float bg[3];
    float z, vz, ba;
    float P[EKF_N][EKF_N];
    int   still_count;
    ekf_params_t p;
} ekf_t;

void ekf_params_default(ekf_params_t *p);
void ekf_init(ekf_t *e, const ekf_params_t *p, const float acc_mean[3], const float gyro_mean[3]);
void ekf_predict(ekf_t *e, const float gyro[3], const float acc[3], float dt);
void ekf_update_accel(ekf_t *e, const float acc[3]);
void ekf_update_baro(ekf_t *e, float alt_m);
void ekf_update_zupt(ekf_t *e, const float gyro[3], const float acc[3]);
void ekf_get_euler(const ekf_t *e, float *roll, float *pitch, float *yaw); /* radianos */
