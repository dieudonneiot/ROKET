#include "fc/flight.h"

#include <math.h>
#include <string.h>

#define DEG_TO_RAD 0.017453292f
#define SPEED_OF_SOUND_MPS 340.0f
#define MAX_DT_S 0.5f
#define MIN_SIN_ELEVATION 0.3f
#define STUCK_MIN_SPEED_MPS 5.0f
#define BARO_TRACK_TAU_S 0.2f
#define MACH_DEWEIGHT_MAX 100.0f

void fc_flight_default_params(fc_flight_params_t *params)
{
    if (params == NULL) {
        return;
    }
    params->launch_accel_mps2 = 3.0f * FC_GRAVITY_MPS2; /* boost is 7-22 g (F-65) */
    params->launch_accel_hold_s = 0.05f;
    params->launch_confirm_altitude_m = 15.0f;
    params->launch_confirm_speed_mps = 15.0f;
    params->launch_confirm_window_s = 1.0f;
    params->burnout_hold_s = 0.2f;
    params->burnout_min_time_s = 0.0f; /* off until the committee answers Q-03 */
    /* Low enough that a short flight still deploys; false descents are blocked by
     * the burnout, speed and barometric-drop criteria. */
    params->arm_altitude_m = 300.0f;
    params->arm_hold_s = 0.2f;
    params->tilt_limit_deg = 70.0f;
    params->lateral_accel_limit_mps2 = 2.0f * FC_GRAVITY_MPS2;
    params->angle_hold_s = 0.2f;
    params->descent_hold_s = 0.3f; /* with the 3 m drop: about 0.85 s after apogee (F-35) */
    params->descent_drop_m = 3.0f;
    params->fallback_descent_speed_mps = 10.0f;
    params->main_altitude_m = 530.0f; /* canopy full near 490 m, inside 400-600 m */
    params->main_hold_s = 0.3f;
    params->baro_sigma_m = 2.0f;
    params->accel_sigma_mps2 = 1.0f;
    params->accel_sigma_chute_mps2 = 5.0f;
    params->bias_sigma = 0.05f;
    params->gate_sigma = 3.0f;
    params->reject_timeout_s = 0.3f;
    params->stuck_time_s = 1.0f;
    params->mach_deweight_start = 0.6f; /* as AltOS and CATS do (F-66) */
    params->mach_ignore = 0.75f;
    params->pyro_deweight_s = 1.0f;
    params->pyro_deweight_factor = 25.0f;
    params->ground_tau_s = 2.0f;
    params->max_coast_accel_mps2 = FC_GRAVITY_MPS2;
    params->max_accel_mps2 = 30.0f * FC_GRAVITY_MPS2;
}

void fc_flight_init(fc_flight_t *flight, const fc_flight_params_t *params)
{
    size_t i;

    if (flight == NULL) {
        return;
    }
    memset(flight, 0, sizeof *flight);
    if (params != NULL) {
        flight->params = *params;
    } else {
        fc_flight_default_params(&flight->params);
    }
    fc_kf_init(&flight->kf, 0.0f);
    flight->phase = FC_PHASE_PAD;
    for (i = 0u; i < FC_BARO_COUNT; i++) {
        flight->baro_healthy[i] = 1u;
        flight->last_raw_m[i] = NAN;
    }
}

float fc_pressure_to_altitude_m(float pressure_mbar)
{
    if (!(pressure_mbar > 0.0f) || !isfinite(pressure_mbar)) {
        return NAN;
    }
    return 44330.77f * (1.0f - powf(pressure_mbar / 1013.25f, 0.190263f));
}

float fc_vertical_accel_mps2(const float accel_mps2[3], float roll_deg, float pitch_deg)
{
    float roll = roll_deg * DEG_TO_RAD;
    float pitch = pitch_deg * DEG_TO_RAD;
    /* Third row of the body-to-NED rotation for Z-Y-X Euler angles (Attachment-1). */
    float down = -sinf(pitch) * accel_mps2[0] + sinf(roll) * cosf(pitch) * accel_mps2[1] +
                 cosf(roll) * cosf(pitch) * accel_mps2[2];

    return -down - FC_GRAVITY_MPS2;
}

static uint8_t all_finite(const fc_flight_input_t *input)
{
    return (uint8_t)(isfinite(input->accel_mps2[0]) && isfinite(input->accel_mps2[1]) &&
                     isfinite(input->accel_mps2[2]) && isfinite(input->roll_deg) &&
                     isfinite(input->pitch_deg));
}

/* Adds dt to a hold timer while the condition holds, resets it otherwise. */
static float hold(float timer_s, uint8_t condition, float dt_s)
{
    return condition ? timer_s + dt_s : 0.0f;
}

/* Measurement variance of the barometers for this step; 0 means "do not use". */
static float baro_variance(const fc_flight_t *flight)
{
    const fc_flight_params_t *p = &flight->params;
    float variance = p->baro_sigma_m * p->baro_sigma_m;

    if (flight->speed_mach >= p->mach_ignore) {
        return 0.0f;
    }
    if (flight->speed_mach > p->mach_deweight_start) {
        float ramp = (flight->speed_mach - p->mach_deweight_start) /
                     (p->mach_ignore - p->mach_deweight_start);
        variance *= 1.0f + (MACH_DEWEIGHT_MAX - 1.0f) * ramp;
    }
    if ((flight->status & ATT7_STATUS_DROGUE_COMMAND) != 0u &&
        flight->time_s - flight->pyro_time_s < p->pyro_deweight_s) {
        variance *= p->pyro_deweight_factor;
    }
    return variance;
}

static void update_barometers(fc_flight_t *flight, const fc_flight_input_t *input, float dt_s)
{
    const fc_flight_params_t *p = &flight->params;
    float variance = baro_variance(flight);
    float agl[FC_BARO_COUNT];
    uint8_t used[FC_BARO_COUNT];
    float accepted_sum = 0.0f;
    float healthy_sum = 0.0f;
    uint8_t accepted_count = 0u;
    uint8_t healthy_count = 0u;
    uint8_t rejecting_count = 0u;
    uint8_t on_pad = (uint8_t)((flight->status & ATT7_STATUS_LAUNCH) == 0u);
    size_t i;

    for (i = 0u; i < FC_BARO_COUNT; i++) {
        float raw = input->baro_altitude_m[i];
        float innovation = 0.0f;

        used[i] = 0u;
        if (!isfinite(raw)) {
            continue;
        }
        /* The pad reference follows the barometers until a launch candidate appears. */
        if (on_pad && flight->confirm_left_s <= 0.0f) {
            if (flight->ground_valid[i] == 0u) {
                flight->ground_m[i] = raw;
                flight->ground_valid[i] = 1u;
            } else {
                float alpha = dt_s / p->ground_tau_s;
                flight->ground_m[i] += (raw - flight->ground_m[i]) * (alpha < 1.0f ? alpha : 1.0f);
            }
        }
        if (flight->ground_valid[i] == 0u || flight->baro_healthy[i] == 0u) {
            continue;
        }
        if (!on_pad && raw == flight->last_raw_m[i] && fabsf(flight->kf.v) > STUCK_MIN_SPEED_MPS) {
            flight->stuck_s[i] += dt_s;
            if (flight->stuck_s[i] >= p->stuck_time_s) {
                flight->baro_healthy[i] = 0u;
            }
            continue;
        }
        flight->stuck_s[i] = 0.0f;
        flight->last_raw_m[i] = raw;
        agl[i] = raw - flight->ground_m[i];
        healthy_sum += agl[i];
        healthy_count++;
        if (variance <= 0.0f) {
            continue; /* near Mach 1: barometers ignored */
        }
        used[i] = 1u;
        if (fc_kf_update_altitude(&flight->kf, agl[i], variance, p->gate_sigma, &innovation) !=
            0u) {
            flight->rejected_s[i] = 0.0f;
            accepted_sum += agl[i];
            accepted_count++;
        } else {
            flight->rejected_s[i] += dt_s;
        }
    }

    /* A barometer rejected for too long while another is accepted is faulty; if every
     * barometer is rejected, the filter has diverged and is re-centred on them. */
    for (i = 0u; i < FC_BARO_COUNT; i++) {
        if (used[i] != 0u && flight->rejected_s[i] >= p->reject_timeout_s) {
            rejecting_count++;
        }
    }
    if (rejecting_count > 0u && accepted_count > 0u) {
        for (i = 0u; i < FC_BARO_COUNT; i++) {
            if (used[i] != 0u && flight->rejected_s[i] >= p->reject_timeout_s) {
                flight->baro_healthy[i] = 0u;
            }
        }
    } else if (rejecting_count > 0u && healthy_count > 0u) {
        fc_kf_reset_altitude(&flight->kf, healthy_sum / (float)healthy_count, variance);
        for (i = 0u; i < FC_BARO_COUNT; i++) {
            flight->rejected_s[i] = 0.0f;
        }
    }

    /* Barometer-only altitude, independent of the accelerometer, for the drop criterion. */
    if (accepted_count > 0u) {
        float mean = accepted_sum / (float)accepted_count;
        if (flight->baro_track_valid == 0u) {
            flight->baro_track_m = mean;
            flight->baro_track_valid = 1u;
        } else {
            float alpha = dt_s / BARO_TRACK_TAU_S;
            flight->baro_track_m += (mean - flight->baro_track_m) * (alpha < 1.0f ? alpha : 1.0f);
        }
    }
}

static void detect_events(fc_flight_t *flight, float axial_mps2, float lateral_mps2, uint8_t imu_ok,
                          float dt_s)
{
    const fc_flight_params_t *p = &flight->params;
    fc_kf_t *kf = &flight->kf;

    /* Bit 0: launch = sustained axial force, confirmed by a climb within the window. */
    if ((flight->status & ATT7_STATUS_LAUNCH) == 0u) {
        flight->launch_hold_s = hold(flight->launch_hold_s,
                                     (uint8_t)(imu_ok && axial_mps2 >= p->launch_accel_mps2), dt_s);
        if (flight->launch_hold_s >= p->launch_accel_hold_s && flight->confirm_left_s <= 0.0f) {
            flight->confirm_left_s = p->launch_confirm_window_s;
        }
        if (flight->confirm_left_s > 0.0f) {
            if (kf->h >= p->launch_confirm_altitude_m || kf->v >= p->launch_confirm_speed_mps) {
                flight->status |= ATT7_STATUS_LAUNCH;
                flight->phase = FC_PHASE_BOOST;
                flight->launch_time_s = flight->time_s;
                flight->confirm_left_s = 0.0f;
            } else {
                flight->confirm_left_s -= dt_s;
                if (flight->confirm_left_s < 0.0f) {
                    flight->confirm_left_s = 0.0f; /* a bump, not a launch */
                }
            }
        }
        return;
    }

    /* Bit 1: burnout = axial force below zero (drag only); the minimum time only delays. */
    if ((flight->status & ATT7_STATUS_BURNOUT_LOCKOUT) == 0u) {
        flight->burnout_hold_s =
            hold(flight->burnout_hold_s, (uint8_t)(imu_ok && axial_mps2 < 0.0f), dt_s);
        if (flight->burnout_hold_s >= p->burnout_hold_s &&
            flight->time_s - flight->launch_time_s >= p->burnout_min_time_s) {
            flight->status |= ATT7_STATUS_BURNOUT_LOCKOUT;
            flight->phase = FC_PHASE_COAST;
        }
    }

    /* Bit 2: minimum altitude. */
    if ((flight->status & ATT7_STATUS_MIN_ALTITUDE) == 0u) {
        flight->arm_hold_s = hold(flight->arm_hold_s, (uint8_t)(kf->h >= p->arm_altitude_m), dt_s);
        if (flight->arm_hold_s >= p->arm_hold_s) {
            flight->status |= ATT7_STATUS_MIN_ALTITUDE;
        }
    }

    /* Bit 3: nose-down, from the attitude or the lateral acceleration. */
    if ((flight->status & ATT7_STATUS_BODY_ANGLE) == 0u) {
        uint8_t tilted = (uint8_t)(imu_ok && (flight->tilt_deg >= p->tilt_limit_deg ||
                                              lateral_mps2 >= p->lateral_accel_limit_mps2));
        flight->angle_hold_s = hold(flight->angle_hold_s, tilted, dt_s);
        if (flight->angle_hold_s >= p->angle_hold_s) {
            flight->status |= ATT7_STATUS_BODY_ANGLE;
        }
    }

    if ((flight->status & ATT7_STATUS_BURNOUT_LOCKOUT) == 0u) {
        return; /* no descent logic during the boost */
    }
    if (flight->baro_track_valid != 0u && flight->baro_track_m > flight->baro_max_m) {
        flight->baro_max_m = flight->baro_track_m;
    }

    /* Bit 4: descending = filtered speed below zero and an independent barometric drop. */
    if ((flight->status & ATT7_STATUS_DESCENDING) == 0u) {
        flight->descent_hold_s = hold(flight->descent_hold_s, (uint8_t)(kf->v < 0.0f), dt_s);
        if (flight->descent_hold_s >= p->descent_hold_s && flight->baro_track_valid != 0u &&
            flight->baro_max_m - flight->baro_track_m >= p->descent_drop_m) {
            flight->status |= ATT7_STATUS_DESCENDING;
        }
    }

    /* Bit 5: primary chute and payload (Pyro 1). */
    if ((flight->status & ATT7_STATUS_DROGUE_COMMAND) == 0u) {
        flight->fallback_hold_s =
            hold(flight->fallback_hold_s, (uint8_t)(kf->v <= -p->fallback_descent_speed_mps), dt_s);
        if ((flight->status & ATT7_STATUS_MIN_ALTITUDE) != 0u &&
            (flight->status & ATT7_STATUS_DESCENDING) != 0u &&
            ((flight->status & ATT7_STATUS_BODY_ANGLE) != 0u ||
             flight->fallback_hold_s >= p->descent_hold_s)) {
            flight->status |= ATT7_STATUS_DROGUE_COMMAND;
            flight->phase = FC_PHASE_DROGUE;
            flight->pyro_time_s = flight->time_s;
        }
        return;
    }

    /* Bits 6-7: main chute (Pyro 2). Fires right away if apogee was below the main altitude. */
    if ((flight->status & ATT7_STATUS_BELOW_MAIN) == 0u) {
        flight->main_hold_s =
            hold(flight->main_hold_s, (uint8_t)(kf->h < p->main_altitude_m && kf->v < 0.0f), dt_s);
        if (flight->main_hold_s >= p->main_hold_s) {
            flight->status |= ATT7_STATUS_BELOW_MAIN;
        }
    }
    if ((flight->status & ATT7_STATUS_BELOW_MAIN) != 0u &&
        (flight->status & ATT7_STATUS_MAIN_COMMAND) == 0u) {
        flight->status |= ATT7_STATUS_MAIN_COMMAND;
        flight->phase = FC_PHASE_MAIN;
        flight->pyro_time_s = flight->time_s;
    }
}

uint16_t fc_flight_step(fc_flight_t *flight, const fc_flight_input_t *input)
{
    const fc_flight_params_t *p;
    float dt_s;
    float axial = 0.0f;
    float lateral = 0.0f;
    float accel_up;
    float sin_elevation;
    uint8_t imu_ok;

    if (flight == NULL || input == NULL) {
        return 0u;
    }
    p = &flight->params;
    dt_s = input->dt_s;
    if (!isfinite(dt_s) || dt_s <= 0.0f) {
        return flight->status;
    }
    if (dt_s > MAX_DT_S) {
        dt_s = MAX_DT_S;
    }
    flight->time_s += dt_s;

    /* Accelerometer plausibility: saturation, spikes, and thrust after burnout. */
    imu_ok = all_finite(input);
    if (imu_ok) {
        axial = input->accel_mps2[0];
        lateral = sqrtf(input->accel_mps2[1] * input->accel_mps2[1] +
                        input->accel_mps2[2] * input->accel_mps2[2]);
        if (fabsf(axial) > p->max_accel_mps2 || lateral > p->max_accel_mps2 ||
            (flight->phase >= FC_PHASE_COAST && flight->phase < FC_PHASE_DROGUE &&
             axial > p->max_coast_accel_mps2)) {
            imu_ok = 0u;
        }
    }
    if (imu_ok) {
        accel_up = fc_vertical_accel_mps2(input->accel_mps2, input->roll_deg, input->pitch_deg);
        flight->last_accel_up_mps2 = accel_up;
        flight->tilt_deg = 90.0f - input->pitch_deg;
    } else {
        accel_up = flight->last_accel_up_mps2;
    }

    fc_kf_predict(&flight->kf, accel_up, dt_s,
                  (flight->status & ATT7_STATUS_DROGUE_COMMAND) != 0u ? p->accel_sigma_chute_mps2
                                                                      : p->accel_sigma_mps2,
                  p->bias_sigma);

    sin_elevation = fabsf(sinf((90.0f - flight->tilt_deg) * DEG_TO_RAD));
    if (sin_elevation < MIN_SIN_ELEVATION) {
        sin_elevation = MIN_SIN_ELEVATION;
    }
    flight->speed_mach = fabsf(flight->kf.v) / sin_elevation / SPEED_OF_SOUND_MPS;

    update_barometers(flight, input, dt_s);
    detect_events(flight, axial, lateral, imu_ok, dt_s);
    return flight->status;
}

uint16_t fc_flight_step_att7(fc_flight_t *flight, const att7_sample_t *sample, float dt_s)
{
    fc_flight_input_t input;

    if (sample == NULL) {
        return flight != NULL ? flight->status : 0u;
    }
    input.dt_s = dt_s;
    input.baro_altitude_m[0] = sample->altitude_m;
    input.baro_altitude_m[1] = fc_pressure_to_altitude_m(sample->pressure_mbar);
    input.accel_mps2[0] = sample->accel_x_mps2;
    input.accel_mps2[1] = sample->accel_y_mps2;
    input.accel_mps2[2] = sample->accel_z_mps2;
    input.roll_deg = sample->angle_x_deg;
    input.pitch_deg = sample->angle_y_deg;
    input.yaw_deg = sample->angle_z_deg;
    return fc_flight_step(flight, &input);
}

uint8_t fc_flight_pyro1(const fc_flight_t *flight)
{
    return (uint8_t)(flight != NULL && (flight->status & ATT7_STATUS_DROGUE_COMMAND) != 0u);
}

uint8_t fc_flight_pyro2(const fc_flight_t *flight)
{
    return (uint8_t)(flight != NULL && (flight->status & ATT7_STATUS_MAIN_COMMAND) != 0u);
}

size_t fc_flight_sizeof(void)
{
    return sizeof(fc_flight_t);
}
