/*
 * Flight algorithm of the I-FCC: state estimation, event detection and pyro
 * commands, mapped one-to-one onto the Attachment-7 status bits (findings log
 * F-14, F-26, F-35 and section 14.3).
 *
 * Rules this module implements:
 *   - every recovery event needs at least two independent criteria (3.3.6.23);
 *   - decisions use filtered sensor data, never raw readings (3.3.6.25);
 *   - no recovery event is triggered by GPS or by a timer (3.3.6.32); the
 *     optional burnout minimum time only delays, it never triggers (Q-03);
 *   - status bits latch: once set, only a re-initialisation clears them.
 *
 * The same code runs in flight, in the referee Synthetic Flight Test (fed by
 * fc_flight_step_att7) and in our simulations, so we test exactly what flies.
 * fc_flight_t is plain data, so firmware can copy it to battery-backed memory
 * and resume after a watchdog reset.
 */
#ifndef FC_FLIGHT_H
#define FC_FLIGHT_H

#include <stdint.h>

#include "fc/att7.h"
#include "fc/fc_api.h"
#include "fc/kf_vertical.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FC_BARO_COUNT 2u
#define FC_GRAVITY_MPS2 9.80665f

#define FC_PHASE_PAD 0u
#define FC_PHASE_BOOST 1u
#define FC_PHASE_COAST 2u
#define FC_PHASE_DROGUE 3u
#define FC_PHASE_MAIN 4u

typedef struct {
    /* Launch (bit 0): axial force above a threshold, confirmed by a climb. */
    float launch_accel_mps2;
    float launch_accel_hold_s;
    float launch_confirm_altitude_m;
    float launch_confirm_speed_mps;
    float launch_confirm_window_s;
    /* Burnout (bit 1): axial force below zero. The minimum time only delays (Q-03). */
    float burnout_hold_s;
    float burnout_min_time_s;
    /* Minimum altitude (bit 2). */
    float arm_altitude_m;
    float arm_hold_s;
    /* Nose-down (bit 3): tilt from vertical, or lateral acceleration. */
    float tilt_limit_deg;
    float lateral_accel_limit_mps2;
    float angle_hold_s;
    /* Descent (bit 4): filtered speed below zero and a barometric drop. */
    float descent_hold_s;
    float descent_drop_m;
    /* Primary chute when nose-down never comes: a clear descent instead. */
    float fallback_descent_speed_mps;
    /* Main chute (bits 6-7). */
    float main_altitude_m;
    float main_hold_s;
    /* Estimator. */
    float baro_sigma_m;
    float accel_sigma_mps2;
    float accel_sigma_chute_mps2; /* larger under the chutes: trust the barometers */
    float bias_sigma;
    float gate_sigma;
    float reject_timeout_s; /* rejected this long while the other agrees: faulty */
    float stuck_time_s;     /* identical readings this long while moving: stuck */
    float mach_deweight_start;
    float mach_ignore;
    float pyro_deweight_s; /* barometers de-weighted after our own ejection charge */
    float pyro_deweight_factor;
    float ground_tau_s;         /* pad reference filter */
    float max_coast_accel_mps2; /* larger axial force after burnout is a spike */
    float max_accel_mps2;       /* larger readings are rejected as saturation or spikes */
} fc_flight_params_t;

typedef struct {
    float dt_s;
    float baro_altitude_m[FC_BARO_COUNT]; /* above sea level; NaN when missing */
    float accel_mps2[3];                  /* specific force, body x nose, y right, z down */
    float roll_deg;                       /* Z-Y-X Euler angles relative to North-East-Down */
    float pitch_deg;
    float yaw_deg;
} fc_flight_input_t;

typedef struct {
    fc_flight_params_t params;
    fc_kf_t kf;
    float time_s;
    float launch_time_s;
    float ground_m[FC_BARO_COUNT];
    float last_raw_m[FC_BARO_COUNT];
    float stuck_s[FC_BARO_COUNT];
    float rejected_s[FC_BARO_COUNT];
    float baro_track_m; /* smoothed barometer-only altitude, for the drop criterion */
    float baro_max_m;
    float last_accel_up_mps2;
    float tilt_deg;
    float speed_mach;
    float pyro_time_s;
    float launch_hold_s;
    float confirm_left_s;
    float burnout_hold_s;
    float arm_hold_s;
    float angle_hold_s;
    float descent_hold_s;
    float fallback_hold_s;
    float main_hold_s;
    uint16_t status;
    uint8_t phase;
    uint8_t ground_valid[FC_BARO_COUNT];
    uint8_t baro_healthy[FC_BARO_COUNT];
    uint8_t baro_track_valid;
    uint8_t reserved[3];
} fc_flight_t;

FC_API void fc_flight_default_params(fc_flight_params_t *params);

/* Resets the state on the pad; NULL params selects the defaults. */
FC_API void fc_flight_init(fc_flight_t *flight, const fc_flight_params_t *params);

/* Runs one step and returns the latched ATT7_STATUS_* bits. */
FC_API uint16_t fc_flight_step(fc_flight_t *flight, const fc_flight_input_t *input);

/*
 * Runs one step on a referee test frame: the altitude field and the altitude
 * derived from the pressure field are used as the two barometers.
 */
FC_API uint16_t fc_flight_step_att7(fc_flight_t *flight, const att7_sample_t *sample, float dt_s);

FC_API uint8_t fc_flight_pyro1(const fc_flight_t *flight); /* primary chute and payload */
FC_API uint8_t fc_flight_pyro2(const fc_flight_t *flight); /* main chute */

/* ISA altitude above sea level for a pressure in mbar; NaN for non-positive input. */
FC_API float fc_pressure_to_altitude_m(float pressure_mbar);

/* Vertical acceleration (gravity removed) from body specific force and Z-Y-X Euler angles. */
FC_API float fc_vertical_accel_mps2(const float accel_mps2[3], float roll_deg, float pitch_deg);

FC_API size_t fc_flight_sizeof(void);

#ifdef __cplusplus
}
#endif

#endif /* FC_FLIGHT_H */
