/*
 * Vertical Kalman filter: altitude above the pad, vertical speed and
 * accelerometer bias (findings log, section 14.3).
 *
 * The accelerometer drives the prediction (vertical acceleration with gravity
 * removed); each barometer corrects the altitude in turn, and readings that
 * disagree with the prediction by more than a few standard deviations are
 * rejected. Single precision, no dynamic memory.
 */
#ifndef FC_KF_VERTICAL_H
#define FC_KF_VERTICAL_H

#include <stdint.h>

#include "fc/fc_api.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float h;       /* altitude above the pad reference, m */
    float v;       /* vertical speed, m/s, positive up */
    float b;       /* accelerometer bias along the vertical, m/s^2 */
    float p[3][3]; /* state covariance */
} fc_kf_t;

FC_API void fc_kf_init(fc_kf_t *kf, float altitude_m);

/* Propagates by dt seconds with the measured vertical acceleration (gravity removed). */
FC_API void fc_kf_predict(fc_kf_t *kf, float accel_up_mps2, float dt_s, float accel_sigma,
                          float bias_sigma);

/*
 * Corrects with one altitude measurement of variance `variance_m2`. Returns 1
 * if accepted, 0 if the innovation exceeds `gate_sigma` standard deviations
 * (or the input is not finite). *innovation receives z - h either way.
 */
FC_API uint8_t fc_kf_update_altitude(fc_kf_t *kf, float altitude_m, float variance_m2,
                                     float gate_sigma, float *innovation);

/* Re-centres the altitude on a trusted measurement after the filter diverged. */
FC_API void fc_kf_reset_altitude(fc_kf_t *kf, float altitude_m, float variance_m2);

#ifdef __cplusplus
}
#endif

#endif /* FC_KF_VERTICAL_H */
