#include "ekf.h"
#include <math.h>
#include <string.h>

#define G_STD 9.80665f
#define N     EKF_N

/* ---------------- quaternios ---------------- */
static void quat_mul(const float a[4], const float b[4], float o[4])
{
    o[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    o[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    o[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    o[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

static void quat_normalize(float q[4])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-12f) { q[0] = 1; q[1] = q[2] = q[3] = 0; return; }
    for (int i = 0; i < 4; i++) q[i] /= n;
}

/* exp_q(v/2) para vetor de rotacao v */
static void quat_from_rotvec(const float v[3], float q[4])
{
    float n = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (n < 1e-8f) {
        q[0] = 1.0f; q[1] = 0.5f*v[0]; q[2] = 0.5f*v[1]; q[3] = 0.5f*v[2];
    } else {
        float s = sinf(0.5f*n) / n;
        q[0] = cosf(0.5f*n); q[1] = v[0]*s; q[2] = v[1]*s; q[3] = v[2]*s;
    }
}

/* R body->nav */
static void quat_to_rot(const float q[4], float R[3][3])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    R[0][0] = 1 - 2*(y*y + z*z); R[0][1] = 2*(x*y - w*z);     R[0][2] = 2*(x*z + w*y);
    R[1][0] = 2*(x*y + w*z);     R[1][1] = 1 - 2*(x*x + z*z); R[1][2] = 2*(y*z - w*x);
    R[2][0] = 2*(x*z - w*y);     R[2][1] = 2*(y*z + w*x);     R[2][2] = 1 - 2*(x*x + y*y);
}

static float norm3(const float v[3]) { return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); }

/* ---------------- parametros ---------------- */
void ekf_params_default(ekf_params_t *p)
{
    p->gyro_noise      = 0.005f;
    p->gyro_bias_rw    = 2e-5f;
    p->acc_noise       = 0.3f;
    p->acc_bias_rw     = 1e-3f;
    p->z_noise         = 0.01f;
    p->acc_sigma       = 0.02f;
    p->baro_sigma      = 0.4f;
    p->zupt_gyro_sigma = 0.005f;
    p->zupt_vel_sigma  = 0.05f;
    p->acc_reject      = 3.0f;
    p->acc_inflate     = 25.0f;
    p->still_gyro      = 0.05f;
    p->still_acc       = 0.25f;
    p->still_samples   = 50;
}

/* Inicializacao: roll/pitch pelo acelerometro (equivalente as Eq. 11 do artigo), yaw = 0 */
void ekf_init(ekf_t *e, const ekf_params_t *p, const float acc_mean[3], const float gyro_mean[3])
{
    memset(e, 0, sizeof(*e));
    e->p = *p;

    float roll  = atan2f(acc_mean[1], acc_mean[2]);
    float pitch = atan2f(-acc_mean[0], sqrtf(acc_mean[1]*acc_mean[1] + acc_mean[2]*acc_mean[2]));
    float cr = cosf(0.5f*roll), sr = sinf(0.5f*roll);
    float cp = cosf(0.5f*pitch), sp = sinf(0.5f*pitch);
    e->q[0] = cr*cp; e->q[1] = sr*cp; e->q[2] = cr*sp; e->q[3] = -sr*sp;
    quat_normalize(e->q);

    for (int i = 0; i < 3; i++) e->bg[i] = gyro_mean[i];
    e->z = 0; e->vz = 0; e->ba = 0;

    for (int i = 0; i < 3; i++) e->P[i][i]     = 0.05f*0.05f;   /* atitude  */
    for (int i = 3; i < 6; i++) e->P[i][i]     = 0.01f*0.01f;   /* bias gyro*/
    e->P[6][6] = 0.5f*0.5f;
    e->P[7][7] = 0.1f*0.1f;
    e->P[8][8] = 0.1f*0.1f;
}

/* ---------------- predicao ---------------- */
void ekf_predict(ekf_t *e, const float gyro[3], const float acc[3], float dt)
{
    float w[3] = { gyro[0]-e->bg[0], gyro[1]-e->bg[1], gyro[2]-e->bg[2] };
    float R[3][3];
    quat_to_rot(e->q, R);

    /* aceleracao vertical (nav, Z cima) */
    float a_up = R[2][0]*acc[0] + R[2][1]*acc[1] + R[2][2]*acc[2] - G_STD - e->ba;

    /* estado nominal */
    e->z  += e->vz*dt + 0.5f*a_up*dt*dt;
    e->vz += a_up*dt;

    float rv[3] = { w[0]*dt, w[1]*dt, w[2]*dt }, dq[4], qn[4];
    quat_from_rotvec(rv, dq);
    quat_mul(e->q, dq, qn);
    memcpy(e->q, qn, sizeof(qn));
    quat_normalize(e->q);

    /* Phi = I + F*dt */
    float Phi[N][N];
    memset(Phi, 0, sizeof(Phi));
    for (int i = 0; i < N; i++) Phi[i][i] = 1.0f;

    /* d(dtheta)/dt = -[w x] dtheta - dbg */
    Phi[0][1] += w[2]*dt;  Phi[0][2] -= w[1]*dt;
    Phi[1][0] -= w[2]*dt;  Phi[1][2] += w[0]*dt;
    Phi[2][0] += w[1]*dt;  Phi[2][1] -= w[0]*dt;
    for (int i = 0; i < 3; i++) Phi[i][3+i] -= dt;

    /* z' = vz ; vz' = a_up(dtheta, ba) */
    Phi[6][7] += dt;
    float r[3] = { R[2][0], R[2][1], R[2][2] };           /* e3^T R */
    float rxf[3] = { r[1]*acc[2] - r[2]*acc[1],
                     r[2]*acc[0] - r[0]*acc[2],
                     r[0]*acc[1] - r[1]*acc[0] };
    for (int i = 0; i < 3; i++) Phi[7][i] -= rxf[i]*dt;
    Phi[7][8] -= dt;

    /* P = Phi P Phi^T + Qd */
    float T[N][N], Pn[N][N];
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            float s = 0;
            for (int k = 0; k < N; k++) s += Phi[i][k]*e->P[k][j];
            T[i][j] = s;
        }
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            float s = 0;
            for (int k = 0; k < N; k++) s += T[i][k]*Phi[j][k];
            Pn[i][j] = s;
        }

    const ekf_params_t *p = &e->p;
    for (int i = 0; i < 3; i++) Pn[i][i]     += p->gyro_noise*p->gyro_noise*dt;
    for (int i = 3; i < 6; i++) Pn[i][i]     += p->gyro_bias_rw*p->gyro_bias_rw*dt;
    Pn[6][6] += p->z_noise*p->z_noise*dt;
    Pn[7][7] += p->acc_noise*p->acc_noise*dt;
    Pn[8][8] += p->acc_bias_rw*p->acc_bias_rw*dt;

    for (int i = 0; i < N; i++)
        for (int j = i; j < N; j++) {
            float s = 0.5f*(Pn[i][j] + Pn[j][i]);
            e->P[i][j] = e->P[j][i] = s;
        }
}

/* ---------------- atualizacao escalar sequencial ---------------- */
static void inject(ekf_t *e, const float dx[N])
{
    float dq[4], qn[4];
    quat_from_rotvec(dx, dq);
    quat_mul(e->q, dq, qn);
    memcpy(e->q, qn, sizeof(qn));
    quat_normalize(e->q);
    for (int i = 0; i < 3; i++) e->bg[i] += dx[3+i];
    e->z  += dx[6];
    e->vz += dx[7];
    e->ba += dx[8];
}

/* gate = limite chi2 (r^2 > gate*S => rejeita); gate<=0 desliga */
static bool scalar_update(ekf_t *e, const float h[N], float r, float Rv, float gate)
{
    float PHt[N];
    for (int i = 0; i < N; i++) {
        float s = 0;
        for (int j = 0; j < N; j++) s += e->P[i][j]*h[j];
        PHt[i] = s;
    }
    float S = Rv;
    for (int i = 0; i < N; i++) S += h[i]*PHt[i];
    if (S < 1e-12f) return false;
    if (gate > 0 && r*r > gate*S) return false;

    float dx[N];
    for (int i = 0; i < N; i++) dx[i] = PHt[i]/S*r;
    inject(e, dx);

    for (int i = 0; i < N; i++)
        for (int j = i; j < N; j++) {
            float v = e->P[i][j] - PHt[i]*PHt[j]/S;
            e->P[i][j] = e->P[j][i] = v;
        }
    for (int i = 0; i < N; i++) if (e->P[i][i] < 1e-12f) e->P[i][i] = 1e-12f;
    return true;
}

/* Correcao de drift de roll/pitch: direcao da gravidade.
 * h(x) = R^T e3,  H_theta = [h x].  R e inflado por |‖f‖-g| (covariance inflation). */
void ekf_update_accel(ekf_t *e, const float acc[3])
{
    float n = norm3(acc);
    if (n < 1e-3f) return;
    float dev = fabsf(n - G_STD);
    if (dev > e->p.acc_reject) return;

    float Rv = e->p.acc_sigma*e->p.acc_sigma*(1.0f + e->p.acc_inflate*dev*dev);
    float zm[3] = { acc[0]/n, acc[1]/n, acc[2]/n };

    for (int k = 0; k < 3; k++) {
        float R[3][3];
        quat_to_rot(e->q, R);                 /* relinearizado a cada componente */
        float hh[3] = { R[2][0], R[2][1], R[2][2] };

        float H[N] = {0};
        switch (k) {                           /* linha k de [h x] */
            case 0: H[1] = -hh[2]; H[2] =  hh[1]; break;
            case 1: H[0] =  hh[2]; H[2] = -hh[0]; break;
            default:H[0] = -hh[1]; H[1] =  hh[0]; break;
        }
        scalar_update(e, H, zm[k] - hh[k], Rv, 0.0f);
    }
}

/* Barometro: altitude relativa (m) */
void ekf_update_baro(ekf_t *e, float alt_m)
{
    float H[N] = {0};
    H[6] = 1.0f;
    scalar_update(e, H, alt_m - e->z, e->p.baro_sigma*e->p.baro_sigma, 9.0f);
}

/* ZUPT: em repouso, gyro = bias (corrige drift do gyro, inclusive yaw) e vz = 0 */
void ekf_update_zupt(ekf_t *e, const float gyro[3], const float acc[3])
{
    float wc[3] = { gyro[0]-e->bg[0], gyro[1]-e->bg[1], gyro[2]-e->bg[2] };
    float dev = fabsf(norm3(acc) - G_STD);

    if (norm3(wc) < e->p.still_gyro && dev < e->p.still_acc) e->still_count++;
    else e->still_count = 0;
    if (e->still_count < e->p.still_samples) return;

    for (int i = 0; i < 3; i++) {
        float H[N] = {0};
        H[3+i] = 1.0f;
        scalar_update(e, H, gyro[i] - e->bg[i],
                      e->p.zupt_gyro_sigma*e->p.zupt_gyro_sigma, 0.0f);
    }
    float H[N] = {0};
    H[7] = 1.0f;
    scalar_update(e, H, -e->vz, e->p.zupt_vel_sigma*e->p.zupt_vel_sigma, 0.0f);
}

void ekf_get_euler(const ekf_t *e, float *roll, float *pitch, float *yaw)
{
    float w = e->q[0], x = e->q[1], y = e->q[2], z = e->q[3];
    float sp = 2.0f*(w*y - z*x);
    if (sp >  1.0f) sp =  1.0f;
    if (sp < -1.0f) sp = -1.0f;
    *roll  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
    *pitch = asinf(sp);
    *yaw   = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
}
