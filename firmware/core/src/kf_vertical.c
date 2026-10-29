#include "fc/kf_vertical.h"

#include <math.h>
#include <string.h>

#define KF_INITIAL_ALTITUDE_VAR 4.0f
#define KF_INITIAL_SPEED_VAR 1.0f
#define KF_INITIAL_BIAS_VAR 0.25f

void fc_kf_init(fc_kf_t *kf, float altitude_m)
{
    if (kf == NULL) {
        return;
    }
    memset(kf, 0, sizeof *kf);
    kf->h = altitude_m;
    kf->p[0][0] = KF_INITIAL_ALTITUDE_VAR;
    kf->p[1][1] = KF_INITIAL_SPEED_VAR;
    kf->p[2][2] = KF_INITIAL_BIAS_VAR;
}

void fc_kf_predict(fc_kf_t *kf, float accel_up_mps2, float dt_s, float accel_sigma,
                   float bias_sigma)
{
    float f[3][3];
    float fp[3][3];
    float g[3];
    float accel;
    float qa;
    int i;
    int j;
    int k;

    if (kf == NULL || !(dt_s > 0.0f) || !isfinite(accel_up_mps2)) {
        return;
    }
    accel = accel_up_mps2 - kf->b;
    kf->h += kf->v * dt_s + 0.5f * accel * dt_s * dt_s;
    kf->v += accel * dt_s;

    /* x' = F x + G u, with F = [[1, dt, -dt^2/2], [0, 1, -dt], [0, 0, 1]]. */
    memset(f, 0, sizeof f);
    f[0][0] = 1.0f;
    f[0][1] = dt_s;
    f[0][2] = -0.5f * dt_s * dt_s;
    f[1][1] = 1.0f;
    f[1][2] = -dt_s;
    f[2][2] = 1.0f;

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            fp[i][j] = 0.0f;
            for (k = 0; k < 3; k++) {
                fp[i][j] += f[i][k] * kf->p[k][j];
            }
        }
    }
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            kf->p[i][j] = 0.0f;
            for (k = 0; k < 3; k++) {
                kf->p[i][j] += fp[i][k] * f[j][k];
            }
        }
    }

    /* Q = G G^T sigma_a^2 + diag(0, 0, sigma_b^2 dt), with G = [dt^2/2, dt, 0]. */
    g[0] = 0.5f * dt_s * dt_s;
    g[1] = dt_s;
    g[2] = 0.0f;
    qa = accel_sigma * accel_sigma;
    for (i = 0; i < 2; i++) {
        for (j = 0; j < 2; j++) {
            kf->p[i][j] += g[i] * g[j] * qa;
        }
    }
    kf->p[2][2] += bias_sigma * bias_sigma * dt_s;
}

uint8_t fc_kf_update_altitude(fc_kf_t *kf, float altitude_m, float variance_m2, float gate_sigma,
                              float *innovation)
{
    float nu;
    float s;
    float gain[3];
    float row0[3];
    int i;
    int j;

    if (kf == NULL || !isfinite(altitude_m) || !(variance_m2 > 0.0f)) {
        return 0u;
    }
    nu = altitude_m - kf->h;
    if (innovation != NULL) {
        *innovation = nu;
    }
    s = kf->p[0][0] + variance_m2;
    if (nu * nu > gate_sigma * gate_sigma * s) {
        return 0u;
    }
    for (i = 0; i < 3; i++) {
        gain[i] = kf->p[i][0] / s;
        row0[i] = kf->p[0][i];
    }
    kf->h += gain[0] * nu;
    kf->v += gain[1] * nu;
    kf->b += gain[2] * nu;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            kf->p[i][j] -= gain[i] * row0[j];
        }
    }
    /* Keep P symmetric against rounding. */
    for (i = 0; i < 3; i++) {
        for (j = i + 1; j < 3; j++) {
            float mean = 0.5f * (kf->p[i][j] + kf->p[j][i]);
            kf->p[i][j] = mean;
            kf->p[j][i] = mean;
        }
    }
    return 1u;
}

void fc_kf_reset_altitude(fc_kf_t *kf, float altitude_m, float variance_m2)
{
    int i;

    if (kf == NULL || !isfinite(altitude_m)) {
        return;
    }
    kf->h = altitude_m;
    for (i = 0; i < 3; i++) {
        kf->p[0][i] = 0.0f;
        kf->p[i][0] = 0.0f;
    }
    kf->p[0][0] = variance_m2;
}
