/*
 * Referee test protocol of TEKNOFEST A4 (Attachment-7): frame codec, streaming
 * parser and test-mode session.
 *
 * At the range a referee test device talks to the I-FCC over RS-232
 * (115200 8N1). It sends 5-byte commands and, during the Synthetic Flight Test
 * (SFT), 36-byte frames of synthetic sensor data at 10 Hz. The I-FCC answers
 * with 36-byte sensor frames during the Sensor Monitoring Test (SMT) and with
 * 6-byte status frames during the SFT, both at 10 Hz.
 *
 * Attachment-7's worked examples contradict its own checksum rule, and its
 * byte order is ambiguous, so every convention is a runtime setting
 * (findings log F-28, F-29):
 *   - receive liberally: commands are accepted with either checksum family
 *     (header + command, or the printed Table 1 value = command + 0x6C); a
 *     mismatch is counted rather than rejected unless strict mode is on;
 *   - send strictly: checksum over every preceding byte including the header,
 *     big-endian floats rounded to 2 decimals.
 *
 * Pure C11: no dynamic memory, no hardware access, single precision only.
 */
#ifndef FC_ATT7_H
#define FC_ATT7_H

#include <stddef.h>
#include <stdint.h>

#include "fc/fc_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Frame layout (Attachment-7 Tables 1-6) ----------------------------- */

#define ATT7_HEADER_COMMAND 0xAAu /* commands to the FCC, status frames from it */
#define ATT7_HEADER_DATA 0xABu    /* 36-byte sensor frames, both directions */
#define ATT7_FOOTER_1 0x0Du
#define ATT7_FOOTER_2 0x0Au

#define ATT7_CMD_START_SMT 0x20u
#define ATT7_CMD_START_SFT 0x22u
#define ATT7_CMD_STOP 0x24u

#define ATT7_COMMAND_LEN 5u
#define ATT7_DATA_LEN 36u
#define ATT7_STATUS_LEN 6u
#define ATT7_DATA_CHECKSUM_INDEX 33u
#define ATT7_STATUS_CHECKSUM_INDEX 3u

/* Table 1 prints checksum = command + 0x6C, which no byte sum reproduces. */
#define ATT7_TABLE1_CHECKSUM_OFFSET 0x6Cu

/* ---- Timing (Attachment-7 2.1.2, 2.1.4, 2.2) ---------------------------- */

#define ATT7_ACTIVATION_DELAY_MS 1000u /* a test mode starts 1 s after its command */
#define ATT7_TX_PERIOD_MS 100u         /* SMT data and SFT status at 10 Hz */
#define ATT7_SFT_DATA_TIMEOUT_MS 1000u /* the test device ends the SFT after this gap */
/*
 * Silence after which a partial frame is dropped: longer than USB-serial
 * adapter latency (FTDI default 16 ms), shorter than the ~97 ms gap between
 * 10 Hz frames, so a truncated frame never swallows the start of the next one.
 */
#define ATT7_FRAME_TIMEOUT_MS 50u

/* ---- SFT status bits (Attachment-7 Table 5) ----------------------------- */

#define ATT7_STATUS_LAUNCH (1u << 0)          /* launch detected */
#define ATT7_STATUS_BURNOUT_LOCKOUT (1u << 1) /* burnout protection has expired */
#define ATT7_STATUS_MIN_ALTITUDE (1u << 2)    /* minimum altitude exceeded */
#define ATT7_STATUS_BODY_ANGLE (1u << 3)      /* body angle / lateral acceleration limit */
#define ATT7_STATUS_DESCENDING (1u << 4)      /* altitude started to decrease */
#define ATT7_STATUS_DROGUE_COMMAND (1u << 5)  /* primary (drogue) deployment commanded */
#define ATT7_STATUS_BELOW_MAIN (1u << 6)      /* below the main deployment altitude */
#define ATT7_STATUS_MAIN_COMMAND (1u << 7)    /* main deployment commanded */

/* ---- Configuration -------------------------------------------------------- */

/* Byte range of the checksum: all bytes before it, with or without the header. */
#define ATT7_CHECKSUM_WITH_HEADER 0u
#define ATT7_CHECKSUM_WITHOUT_HEADER 1u

/* Byte order of every float field, in both directions. */
#define ATT7_FLOAT_BIG_ENDIAN 0u
#define ATT7_FLOAT_LITTLE_ENDIAN 1u

/* Checksum style used when encoding a command (test tooling only). */
#define ATT7_COMMAND_CHECKSUM_HEADER_SUM 0u /* 0xAA + command, e.g. AA 20 CA 0D 0A */
#define ATT7_COMMAND_CHECKSUM_TABLE1 1u     /* command + 0x6C, e.g. AA 20 8C 0D 0A */

typedef struct {
    uint8_t tx_checksum;        /* ATT7_CHECKSUM_* for frames we send */
    uint8_t float_order;        /* ATT7_FLOAT_* */
    uint8_t round_2dp;          /* 1: round sent floats to 2 decimals (Attachment-7 2.1.1) */
    uint8_t strict_rx_checksum; /* 1: drop received frames whose checksum fits no known rule */
    uint8_t status_low_first;   /* 1: Data1 carries status bits 0-7, Data2 bits 8-15 */
} att7_config_t;

/* Defaults chosen from past teams' implementations (F-29). */
FC_API void att7_config_default(att7_config_t *config);

/* ---- Frame codec ---------------------------------------------------------- */

/* Body frame (Attachment-7 1.2): x toward the nose, y right, z down. */
typedef struct {
    float altitude_m; /* above sea level */
    float pressure_mbar;
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
    float angle_x_deg; /* roll */
    float angle_y_deg; /* pitch */
    float angle_z_deg; /* yaw */
} att7_sample_t;

/* Sum mod 256 of frame[0 or 1 .. checksum_index - 1], per ATT7_CHECKSUM_*. */
FC_API uint8_t att7_checksum(const uint8_t *frame, size_t checksum_index, uint8_t mode);

/* Rounds to 2 decimals (half away from zero); a rounded zero is +0.0. */
FC_API float att7_round_2dp(float value);

/* Each encoder writes one complete frame and returns its length, or 0 on bad arguments. */
FC_API size_t att7_encode_command(uint8_t command, uint8_t checksum_style,
                                  uint8_t out[ATT7_COMMAND_LEN]);
FC_API size_t att7_encode_data(const att7_config_t *config, const att7_sample_t *sample,
                               uint8_t out[ATT7_DATA_LEN]);
FC_API size_t att7_encode_status(const att7_config_t *config, uint16_t status,
                                 uint8_t out[ATT7_STATUS_LEN]);

/* ---- Streaming parser (frames arriving from the test device) ----------- */

#define ATT7_EVENT_NONE 0u
#define ATT7_EVENT_COMMAND 1u
#define ATT7_EVENT_DATA 2u

typedef struct {
    uint8_t type;        /* ATT7_EVENT_* */
    uint8_t command;     /* ATT7_CMD_* for a command event */
    uint8_t checksum_ok; /* 1 if the checksum matched one of the accepted rules */
    uint8_t reserved;
    att7_sample_t sample; /* for a data event */
} att7_event_t;

typedef struct {
    uint8_t buf[ATT7_DATA_LEN];
    uint8_t len;      /* bytes collected so far */
    uint8_t expected; /* frame length implied by buf[0], 0 while idle */
    uint8_t reserved[2];
    uint32_t last_byte_ms;
    uint32_t frames_ok;
    uint32_t checksum_mismatches; /* accepted when lenient, dropped when strict */
    uint32_t frames_dropped;      /* bad footer, unknown command or strict checksum failure */
    uint32_t timeouts;            /* partial frames dropped after ATT7_FRAME_TIMEOUT_MS */
} att7_parser_t;

FC_API void att7_parser_init(att7_parser_t *parser);

/*
 * Feeds one received byte. Returns 1 and fills *event when a frame completes.
 * After a 1, keep calling att7_parser_poll() until it returns 0: a dropped frame
 * can leave a complete frame buffered behind it.
 */
FC_API uint8_t att7_parser_push(att7_parser_t *parser, const att7_config_t *config, uint8_t byte,
                                uint32_t now_ms, att7_event_t *event);
FC_API uint8_t att7_parser_poll(att7_parser_t *parser, const att7_config_t *config,
                                att7_event_t *event);

/* ---- Test-mode session ---------------------------------------------------- */

#define ATT7_MODE_NORMAL 0u
#define ATT7_MODE_SMT 1u
#define ATT7_MODE_SFT 2u

typedef struct {
    uint8_t mode;             /* active ATT7_MODE_* */
    uint8_t pending;          /* mode waiting for its activation delay, or NORMAL */
    uint8_t sft_data_timeout; /* 1 while SFT data has been missing for too long */
    uint8_t reserved;
    uint32_t pending_since_ms;
    uint32_t next_tx_ms;
    uint32_t last_sft_data_ms;
} att7_session_t;

FC_API void att7_session_init(att7_session_t *session);

/*
 * Applies a received command and returns 1 if it was accepted. Stop is always
 * accepted; a start command only when test_allowed is 1, which the flight
 * software sets only while the rocket is on the pad and not armed.
 */
FC_API uint8_t att7_session_command(att7_session_t *session, uint8_t command, uint32_t now_ms,
                                    uint8_t test_allowed);

/* Call for every SFT data frame received while the SFT is active. */
FC_API void att7_session_note_sft_data(att7_session_t *session, uint32_t now_ms);

/* Advances the timers; returns 1 when the periodic SMT or SFT frame is due now. */
FC_API uint8_t att7_session_poll(att7_session_t *session, uint32_t now_ms);

/* ---- Layout checks for the ctypes bindings -------------------------------- */

FC_API size_t att7_sizeof_config(void);
FC_API size_t att7_sizeof_sample(void);
FC_API size_t att7_sizeof_event(void);
FC_API size_t att7_sizeof_parser(void);
FC_API size_t att7_sizeof_session(void);

#ifdef __cplusplus
}
#endif

#endif /* FC_ATT7_H */
